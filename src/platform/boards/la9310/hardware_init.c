/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#include "immap.h"
#include "io.h"
#include "la9310_info.h"
#include "la9310_host_if.h"
#include "debug_console.h"
#include "la9310_pci.h"
#include "log.h"

#include <la9310.h>
#include <la9310_i2cAPI.h>

#define CLOCK_CONFIG_BY_HOST 1

extern struct la9310_info g_la9310_info;
extern void vSocResetHandshake(void);
extern void vSocInit();

static void vBoardEarlyInit(void)
{
    xDebugConsoleInit((void *)UART_BASEADDR, EARLY_UART_CLOCK_FREQUENCY, UART_BAUDRATE);
    iLa9310_I2C_Init(LA9310_FSL_I2C1, EARLY_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ);
}

static void vBoardFinalInit(void)
{
    xDebugConsoleInit((void *)UART_BASEADDR, FINAL_UART_CLOCK_FREQUENCY, UART_BAUDRATE);

    iLa9310_I2C_Init(LA9310_FSL_I2C1, FINAL_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ);
}

void vLa9310_do_handshake( struct la9310_info * vLa9310Info )
{
#if CLOCK_CONFIG_BY_HOST
    struct ccsr_dcr * pxDcr = ( struct ccsr_dcr * ) vLa9310Info->pxDcr;
    log_info("LA9310->Host: START_CLOCK_CONFIG\r\n");
    OUT_32( &pxDcr->ulScratchrw[ 1 ], LA9310_HOST_START_CLOCK_CONFIG );
    dmb();

    while( IN_32( &pxDcr->ulScratchrw[ 1 ] ) !=
           LA9310_HOST_COMPLETE_CLOCK_CONFIG )
    {
    }

    log_info("Host->LA9310: HOST_COMPLETE_CLOCK_CONFIG\r\n");
    dmb();
#endif
    log_info("System clock switch...");
    vSocResetHandshake();
    dmb();
    log_info("done\r\n");

    // Clock changed from 100 Mhz to 160Mhz after handshake
    vBoardFinalInit();

#if CLOCK_CONFIG_BY_HOST
    log_info("LA9310->HOST: START_DRIVER_INIT\n");
    OUT_32(&pxDcr->ulScratchrw[1], LA9310_HOST_START_DRIVER_INIT);
#endif
}

void vHardwareEarlyInit( void )
{
#if NXP_ERRATUM_A_009531
    vPCIEIDOClear();
#endif
    vSocInit();
    vBoardEarlyInit();
}
