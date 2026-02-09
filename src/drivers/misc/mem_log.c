/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#include "la9310_host_if.h"
#include "config.h"
#include "io.h"
#include <stddef.h>

uint32_t ulMemLogIndex = 0;

void vMemlogWrite( const uint8_t * pucData,
                   size_t xLength )
{
    struct la9310_hif * pxHif = ( struct la9310_hif * ) ( ( uint8_t * ) TCML_PHY_ADDR
                                                          + LA9310_EP_HIF_OFFSET );
    int i;
    struct debug_log_regs * pxDbglog = &pxHif->dbg_log_regs;
    uint8_t * ucdbgptr = ( uint8_t * ) ( pxDbglog->buf );
    if (ucdbgptr == NULL)
        return;

    ucdbgptr += ulMemLogIndex;

    for( i = 0; i < xLength; i++ )
    {
        OUT_8( ucdbgptr, *pucData );
        pucData++;
        ucdbgptr++;
    }

    ulMemLogIndex += xLength;

    if( ulMemLogIndex >= pxDbglog->len )
    {
        ulMemLogIndex = 0;
    }
}
