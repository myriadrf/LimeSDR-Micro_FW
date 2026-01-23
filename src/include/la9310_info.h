#ifndef LA9310_INFO_H
#define LA9310_INFO_H

#include "la9310_irq.h"

#include "io.h"

#include <stdint.h>

struct la9310_msi_info
{
    uint32_t volatile addr;
    uint32_t data;
};

struct la9310_info
{
    struct ccsr_dcr * pxDcr;
    void * itcm_addr;
    void * dtcm_addr;
    void * pcie_addr;
    void * pcie_obound;
    struct la9310_msg_unit * msg_unit;
    uint32_t llcp_rfic_addr;
    struct la9310_stats * stats;
    struct la9310_hif * pHif;
    struct la9310_msi_info msi_info[ LA9310_MSI_MAX_CNT ];
    struct la9310_irq_evt_info evt_info;
};

#endif