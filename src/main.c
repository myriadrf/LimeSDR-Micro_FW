/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2024 NXP
 */

#include "FreeRTOS.h"
#include "config.h"
#include "la9310_gpio.h"
#include "exceptions.h"
#include "la9310_pinmux.h"
#include "la9310_dcs_api.h"
#include <phytimer.h>

#include "la9310_host_if.h"
#include "la9310_sirq.h"

#include "log.h"
#include "core_cm4.h"

#include "drivers/tmu.h"
#include "drivers/serial/serial_ns16550.h"

#include "iqstream/iqstream.h"

#include "vspa/vspa_regs.h"

#if NXP_ERRATUM_A_009410
    #include "la9310_pci.h"
#endif

#ifdef LMS7002M_CLOCK
    #include "limesdr_micro/limesdr_micro.h"
#endif

static struct vspa_regs *const s_VspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;
static struct ccsr_dcr *const s_Dcr = (void *)DCR_BASE_ADDR;
struct la9310_hif *const s_Hif = (struct la9310_hif *)((uint32_t)TCML_PHY_ADDR + LA9310_EP_HIF_OFFSET);
struct la9310_sirq softirq;

extern int InitBlinkLEDs();
extern void vLa9310_do_handshake(struct ccsr_dcr *pxDcr);
extern void la9310_m4_init_complete(struct ccsr_dcr *pxDcr);
extern void vHardwareEarlyInit( void );

// Empty functions to disable linker warning of them not being implemented in arm libs
void _close(void) {};
void _lseek(void) {};
void _read(void) {};
void _write(void) {};
void _fstat_r(void) {};
void _getpid_r(void) {};
void _getpid(void) {};
void _isatty_r(void) {};
void _kill_r(void) {};

static void axiq_loopback(bool bLoopbackEnable, uint32_t rx_mask)
{
    if (bLoopbackEnable)
        OUT_32(DBGGNCR, ((SET_AXIQ_LOOPBACK_MASK | rx_mask) | IN_32(DBGGNCR)));
    else
        OUT_32(DBGGNCR, (REMOVE_AXIQ_LOOPBACK_MASK & IN_32(DBGGNCR)));

    log_dbg("%s: IN_32( DBGGNCR ) = %#x\r\n", __func__, IN_32(DBGGNCR));
}

void v_main_Hif_Init(struct la9310_hif *pxHif)
{
    pxHif->adc_mask = 0xf;
    pxHif->adc_rate_mask = 0xf;
    pxHif->dac_mask = 0x1;
    pxHif->dac_rate_mask = 0x1;

    pxHif->hif_ver = LA9310_VER_MAKE( LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION );
    log_dbg("Initialized HIF - %d.%d\n", LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION);
}

/* iLa9310HostPreInit: Host and La9310 both do a handshake for clock configuration
 * and init synchronization vLa9310_do_handshake(). Host waits in while loop for
 * La9310 to indicate Host that it can proceed with Initialization
 * (LA9310_HOST_START_DRIVER_INIT).
 *
 * So Do all the initialization which your host peer
 * code needs for initialization. If your host code wants some values
 * initialized by la9310 for it's initialization, then add call your init
 * function here.
 */
static int iLa9310HostPreInit()
{
    int irc = 0;

    v_main_Hif_Init(s_Hif);

    // vVSPAMboxInit();

    // I2C already initialized in vBoardEarlyInit()
#ifdef LMS7002M_CLOCK
    if (initialize_lms7002m_clock_generator())
        log_err( "Failed to configure LMS7002M clock\n");
#endif
    return irc;
}

/* iLa9310HostPostInit: Host and La9310 both do a handshake for clock configuration
 * and init synchronization vLa9310_do_handshake(). Host waits in while loop for
 * La9310 to indicate Host that it can proceed with Initialization
 * (LA9310_HOST_START_DRIVER_INIT).
 *
 * If your module code is dependent on any initialization that has to be done by
 * your peer code on host then add your post handskake init code here.
 */
static int iLa9310HostPostInit()
{
    int irc = 0;

    // irc = iEdmaInit();

    // if( irc == 0 )
    // {
    //     log_info( "%s: eDMA init DONE\n\r", __func__ );
    // }

    return irc;
}

static void iGpioInitRFIC(void)
{
    int iCnt = 0;

    for( iCnt = 6; iCnt < 12; iCnt++ )
    {
        vGpioSetPinMuxSingle( iCnt, SET_MUX_GPIO_MODE );
        iGpioInit( iCnt, output, false );
    }
}

static void vInitMsgUnit(void)
{
    NVIC_SetPriority( IRQ_MSG3, 3 );
    NVIC_EnableIRQ( IRQ_MSG3 );
}

static void SetupVSPA_IRQ(struct vspa_regs *regs)
{
    NVIC_SetPriority(IRQ_VSPA, VSPA_IRQ_PRIORITY);

    OUT_32(&regs->dma_irq_stat, IN_32(&regs->dma_irq_stat));
    const uint32_t irqen_flags = IN_32(&regs->vspa_irqen) | VSPA_DMA_CMP_MASK | (1 << 2); // VCPU_HOST_FLAGS0
    OUT_32(&regs->vspa_irqen, irqen_flags);

    NVIC_ClearPendingIRQ(IRQ_VSPA);
    NVIC_EnableIRQ(IRQ_VSPA);
}

extern struct MemoryLog memlog;
static int iInitHandler()
{
    int irc = 0;

    // fill scratch registers with Host Interface offset and size, for host driver to pick up.
    OUT_32(&s_Dcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_REG], LA9310_EP_HIF_OFFSET);
    dmb();
    OUT_32(&s_Dcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_SIZ_REG], sizeof(struct la9310_hif));
    dmb();

    OUT_32(&s_Dcr->ulScratchrw[9], (uint32_t)&memlog);

    /* XXX:NOTE - Do all initialization that is required by Host driver to
     * function like IRQ MUX, IPC, DMA in iLa9310HostPreInit().
     */
    irc = iLa9310HostPreInit();
    if( irc )
    {
        log_err( "%s: iLa9310HostPreInit Failed, rc %d\n\r", __func__, irc );
        return irc;
    }

    // Till Here system is running of PCIe 100 Mhz clock
    vLa9310_do_handshake(s_Dcr);

#if NXP_ERRATUM_A_009410
    vPCIEInterruptInit();
#endif

    la9310_sirq_initialize(&softirq, s_Dcr->ulScratchrw);

    vInitMsgUnit();
    irc = iLa9310HostPostInit();

    if( irc )
    {
        log_err( "%s: iLa9310HostPostInit Failed, rc %d\n\r", __func__, irc );
        goto out;
    }

    vPhyTimerReset();
    vPhyTimerEnable( PHY_TMR_DIVISOR );
#ifndef LA9310_DFE_APP
    vPhyTimerPPSOUTConfigGPSlike();
#endif

    SetupVSPA_IRQ(s_VspaRegs);

    vDcsInit(IN_32(&s_Hif->adc_mask), IN_32(&s_Hif->adc_rate_mask), IN_32(&s_Hif->dac_mask), IN_32(&s_Hif->dac_rate_mask));
    irc = SUCCESS;

out:
    return irc;
}

static void gps_module_stop(void)
{
    // // wait for any data from GPS before submiting command, otherwise it might get ignored
    log_info("Stopping GPS module...");
    uint8_t temp;
    vSerialReadBlocking((void *)UART_BASEADDR, &temp, 1);
    // Put GPS module to low power standby mode
    vSerialWriteBlocking((void *)UART_BASEADDR, (uint8_t *)"$PMTK161,0*28\r\n", 15);
    // following Tx activity on UART will wake it up
    log_info("done\r\n");
}

extern void ServiceCommands();
/*
 *        La9310 Application Entry point
 */
int main( void )
{
    int irc = 0;
    vEnableExceptions();

    memlog_clear();
    // Initialize hardware
    vHardwareEarlyInit();
    InitBlinkLEDs();

    const uint32_t BootSource = ((IN_32((uint32_t *)DCR_BASE_ADDR)) >> LX9310_BOOT_SRC_SHIFT) & LX9310_BOOT_SRC_MASK;
    // PRINTF("Boot Source ");
    if ( BootSource == LA9310_BOOT_SRC_PCIE ) {
        // PRINTF("PCIe\n");
    } else if ( BootSource == LA9310_BOOT_SRC_I2C ) {
        // PRINTF("I2C\n");
    } else {
        // log_err("Invalid\n");
        goto out;
    }

    // PRINTF("FreeRTOS " tskKERNEL_VERSION_NUMBER "\n");
    irc = iInitHandler();
    if ( irc )
    {
        goto out;
    }

#ifdef LA9310_ENABLE_COMMAND_LINE
    // vUARTCommandConsoleStart( mainUART_COMMAND_CONSOLE_STACK_SIZE, mainUART_COMMAND_CONSOLE_TASK_PRIORITY );
#endif

    tmuInit((TmuRegs_t *)TMU_BASE_ADDR);

#ifdef LMS7002M_CLOCK
    if (LMS64C_protocol_init() != 0)
    {
        log_err("LMS64C_protocol_init failed\r\n");
        goto out;
    }
#endif
    // It appears the default UART baudrate of GPS module is not always 9600 as per data sheet.
    // TODO: detect GPS module UART baudrate
    // gps_module_stop();

    axiq_loopback(false, 0xF);

    iqstream_init();
    la9310_m4_init_complete(s_Dcr);

    // /* Start FreeRTOS scheduler */
    vTaskStartScheduler();
    // ServiceCommands();

out:
    log_err( "main() failed %d\n", irc);

    /* Should never reach this point */
    while( true )
    {
    }
}
