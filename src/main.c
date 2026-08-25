/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "la9310_tmu.h"
#include "la9310_irq.h"
#include "la9310_gpio.h"
#include "la9310_edmaAPI.h"
#include "exceptions.h"
#include "la9310_pinmux.h"
#include "la9310_dcs_api.h"
#include <phytimer.h>

#include "la9310_info.h"
#include "la9310_host_if.h"

#include "log.h"
#include "core_cm4.h"

#include "drivers/serial/serial_ns16550.h"

#include "iqstream/iqstream.h"

#include <string.h>

#if NXP_ERRATUM_A_009410
    #include "la9310_pci.h"
#endif

#include "la9310_avi.h"
#ifdef LMS7002M_CLOCK
    #include "limesdr_micro/limesdr_micro.h"
#endif

extern int InitBlinkLEDs();

struct la9310_info g_la9310_info;

extern void vVSPAMboxInit();

extern void vLa9310_do_handshake(struct ccsr_dcr *pxDcr);
extern void la9310_m4_init_complete(struct ccsr_dcr *pxDcr);
extern void vHardwareEarlyInit( void );

// Empty functions to disable linker warning of them not being implemented in arm libs
void _close() {};
void _lseek() {};
void _read() {};
void _write() {};

void v_main_Hif_Init( struct la9310_info * pLa9310Info )
{
    struct la9310_hif * pxHif = pLa9310Info->pHif;

    pxHif->adc_mask = 0xf;
    pxHif->adc_rate_mask = 0xf;
    pxHif->dac_mask = 0x1;
    pxHif->dac_rate_mask = 0x1;

    pxHif->hif_ver = LA9310_VER_MAKE( LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION );
    log_dbg( "Initialized HIF - %d.%d\n", LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION );

    pLa9310Info->stats = &pxHif->stats;
}

static void prvInitLa9310Info( struct la9310_info * pLa9310Info )
{
    pLa9310Info->itcm_addr = ( void * ) TCML_PHY_ADDR;
    pLa9310Info->dtcm_addr = ( void * ) TCMU_PHY_ADDR;
    pLa9310Info->pcie_addr = ( void * ) PCIE_BASE_ADDR;
    pLa9310Info->msg_unit = ( struct la9310_msg_unit * ) MSG_UNIT_BASE_ADDR;

    pLa9310Info->pcie_obound = ( void * ) PCIE_PHY_ADDR;
    pLa9310Info->pHif = ( struct la9310_hif * ) ( ( uint32_t ) pLa9310Info->itcm_addr + LA9310_EP_HIF_OFFSET );
    pLa9310Info->pHif->dbg_log_regs.log_level = LA9310_LOG_LEVEL_INFO;
    pLa9310Info->pxDcr = ( void * ) DCR_BASE_ADDR;

    log_initialize(&pLa9310Info->pHif->dbg_log_regs);

    // fill scratch registers with Host Interface offset and size, for host driver to pick up.
    struct ccsr_dcr *pxDcr = pLa9310Info->pxDcr;
    OUT_32(&pxDcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_REG], LA9310_EP_HIF_OFFSET);
    dmb();
    OUT_32(&pxDcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_SIZ_REG], sizeof(struct la9310_hif));
    dmb();
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
static int iLa9310HostPreInit(struct la9310_info *pLa9310Info)
{
    int irc = 0;

    v_main_Hif_Init( pLa9310Info );

    vVSPAMboxInit();

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
int iLa9310HostPostInit( struct la9310_info * pLa9310Info )
{
    int irc = 0;

    irc = iEdmaInit();

    if( irc == 0 )
    {
        log_info( "%s: eDMA init DONE\n\r", __func__ );
    }

    return irc;
}

void iGpioInitRFIC( void )
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

void tmuInit( void ) {
	int idx;
	TmuRegs_t *pTmuHandle = ( TmuRegs_t * ) TMU_BASE_ADDR;
	out_le32( &pTmuHandle->tmr, TMU_TMR_DISABLE );

	out_le32( &pTmuHandle->ttrcr[ 0 ], TMU_TTRCR0_INIT );
	for (idx = 0;idx < TMU_TTRCR0_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
			(TMU_TTCFGR_INIT0 + (idx*TMU_TTCFGR_DIFF)) );
		if (idx%2 == 0) {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT0 +
					(((idx/2)*(TMU_TSCFGR_DIFF0)) +
					((idx/2)*(TMU_TSCFGR_DIFF0 + 1)))));
		} else {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT0 +
					((((idx + 1)/2)*(TMU_TSCFGR_DIFF0)) +
					((idx/2)*(TMU_TSCFGR_DIFF0 + 1)))));
		}
	}

	out_le32( &pTmuHandle->ttrcr[ 1 ], TMU_TTRCR1_INIT );
	for (idx = 0;idx < TMU_TTRCR1_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT1 + (idx * TMU_TTCFGR_DIFF)));
		out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT1 +
					(idx * TMU_TSCFGR_DIFF1)));
	}

	for (idx = 0;idx < TMU_TTRCR2_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT2 + (idx * TMU_TTCFGR_DIFF)));
		out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT2 +
					(idx * TMU_TSCFGR_DIFF2)));
	}

	out_le32( &pTmuHandle->ttrcr[ 3 ], TMU_TTRCR3_INIT );
	for (idx=0;idx < TMU_TTRCR3_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT3 + (idx * TMU_TTCFGR_DIFF)));
		if (idx == (TMU_TTRCR3_POINT - 1)) {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 +
						(idx * TMU_TSCFGR_DIFF3) + 1));
		}
		else {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 +
						(idx * TMU_TSCFGR_DIFF3)));
		}
	}
	out_le32( &pTmuHandle->teumr[ 0 ], TMU_TEUMR0_ENABLE );
	out_le32( &pTmuHandle->tdemar, TMU_TDEMAR_ENABLE );
	out_le32( &pTmuHandle->tmtmir, TMU_TMTMIR_ENABLE );
	out_le32( &pTmuHandle->monitoringSite[0].tmsar, TMU_TMSAR0_INIT );
	out_le32( &pTmuHandle->monitoringSite[1].tmsar, TMU_TMSAR1_INIT );
	out_le32( &pTmuHandle->monitoringSite[2].tmsar, TMU_TMSAR2_INIT );
	out_le32( &pTmuHandle->tmrtrcr, TMU_TMRTRCTR_INIT );
	out_le32( &pTmuHandle->tmftrcr, TMU_TMFTRCTR_INIT );
	out_le32( &pTmuHandle->tsr, TMU_TSR_INIT );

	for(int i=0;i<10000;i++);
	out_le32( &pTmuHandle->tmr, TMU_TMR_ENABLE );
}

static int iInitHandler(struct la9310_info *pLa9310Info)
{
    int irc = 0;

    memset( pLa9310Info, 0, sizeof( struct la9310_info ) );
    if( sizeof( struct la9310_hif ) > LA9310_EP_HIF_SIZE )
    {
        // PRINTF( "Invalid HIF size\r\n" );
        return FAILURE;
    }
    /*XXX: DO NOT CALL log_*() before prvInitLa9310Info(), use PRINTF instead.*/
    prvInitLa9310Info( pLa9310Info );

    InitBlinkLEDs();

    /* XXX:NOTE - Do all initialization that is required by Host driver to
     * function like IRQ MUX, IPC, DMA in iLa9310HostPreInit().
     */
    irc = iLa9310HostPreInit(pLa9310Info);
    if( irc )
    {
        log_err( "%s: iLa9310HostPreInit Failed, rc %d\n\r", __func__, irc );
        return irc;
    }

    // Till Here system is running of PCIe 100 Mhz clock
    vLa9310_do_handshake(pLa9310Info->pxDcr);

#if NXP_ERRATUM_A_009410
    vPCIEInterruptInit();
#endif

    la9310_sirq_initialize(&pLa9310Info->softirq, pLa9310Info->pxDcr->ulScratchrw);

    vInitMsgUnit();
    irc = iLa9310HostPostInit( pLa9310Info );

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
    /*VSPA AVI Init*/
#ifdef LA9310_DFE_APP
    /* Tell AVI driver that MBOX0 should not be monitored */
    vVSPAMboxMonitorMaskSet(CM4_MBOX1_STATUS | VSPA_MBOX1_STATUS /* | CM4_MBOX0_STATUS | VSPA_MBOX0_STATUS */);
#endif
    void *avihndl = iLa9310AviInit();
    if( NULL == avihndl )
    {
        log_err( "ERR: %s: AVI Initialization Failed\n\r", __func__ );
    }
    vAxiqLoopbackSet(false, SET_AXIQ_LOOPBACK_MASK_ALL);
#ifdef TURN_ON_STANDALONE_MODE
	iLoadTableToTCM();
    LoadVSPAImage();
#endif //TURN_ON_STANDALONE_MODE
    // iLa9310AviConfig();
    iLa9310AviDirectConfig();
    // NVIC_EnableIRQ( IRQ_AXIQ );

    vDcsInit(IN_32(&pLa9310Info->pHif->adc_mask),
		IN_32(&pLa9310Info->pHif->adc_rate_mask),
		IN_32(&pLa9310Info->pHif->dac_mask),
		IN_32(&pLa9310Info->pHif->dac_rate_mask));
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

    // Initialize hardware
    vHardwareEarlyInit();

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
    irc = iInitHandler(&g_la9310_info);
    if ( irc )
    {
        goto out;
    }

#ifdef LA9310_ENABLE_COMMAND_LINE
    // vUARTCommandConsoleStart( mainUART_COMMAND_CONSOLE_STACK_SIZE, mainUART_COMMAND_CONSOLE_TASK_PRIORITY );
#endif

    tmuInit();

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

    iqstream_init();

    la9310_m4_init_complete(g_la9310_info.pxDcr);

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
