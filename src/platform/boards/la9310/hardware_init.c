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

#define CLOCK_CONFIG_BY_HOST 1

extern struct la9310_info g_la9310_info;
extern void vRaiseMsi(struct la9310_info *pla9310Info, enum la9310_msi_id msi);
extern void vSocResetHandshake( void );
extern void vBoardFinalInit();
extern void vSocInit();
extern void vBoardEarlyInit();

void vLa9310_do_handshake( struct la9310_info * vLa9310Info )
{
#if CLOCK_CONFIG_BY_HOST
    struct ccsr_dcr * pxDcr = ( struct ccsr_dcr * ) vLa9310Info->pxDcr;
    PRINTF("LA9310->Host: START_CLOCK_CONFIG\n");
    OUT_32( &pxDcr->ulScratchrw[ 1 ], LA9310_HOST_START_CLOCK_CONFIG );
    dmb();

    while( IN_32( &pxDcr->ulScratchrw[ 1 ] ) !=
           LA9310_HOST_COMPLETE_CLOCK_CONFIG )
    {
    }

    PRINTF("Host->LA9310: HOST_COMPLETE_CLOCK_CONFIG\n");
    dmb();
#endif
    vSocResetHandshake();
    dmb();

    /*Clock changes from 100 Mhz to 160Mhz after handshake*/
    vBoardFinalInit();

#if CLOCK_CONFIG_BY_HOST
    PRINTF("LA9310->HOST: START_DRIVER_INIT\n");
    OUT_32( &pxDcr->ulScratchrw[ 1 ], LA9310_HOST_START_DRIVER_INIT );

    // #ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
    //     vWaitForPCIeLinkStability();
    //     /*Raise Msi for Host handshaking*/
    //     vRaiseMsi( vLa9310Info, MSI_IRQ_HOST_HANDSHAKE );
    // #endif
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
