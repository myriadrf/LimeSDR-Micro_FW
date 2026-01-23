/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_IRQ_H__
#define __LA9310_IRQ_H__

#define LA9310_MSI_MAX_CNT      8

#define LA9310_EVT_MAX        32
#define LA9310_IRQ_MUX_MSI    ( MSI_IRQ_MUX )

#define IPC_IRQ_PRIORITY      3
#define EDMA_IRQ_PRIORITY     3
#define MSG1_IRQ_PRIORITY     3
#define VSPA_IRQ_PRIORITY     3

struct la9310_info;

enum la9310_evt_type
{
    LA9310_EVT_TYPE_UNUSED = 0,
    LA9310_EVT_TYPE_VSPA,
    LA9310_EVT_TYPE_IPC,
    LA9310_EVT_TYPE_TEST,
    LA9310_EVT_TYPE_END,
};

/* Set IRQ_REAL_MSI_BIT to enable dedicated MSI interrupt line ,
 * and virtual irq line can be used by setting the TEST or LAST
 * EVT bits */

typedef enum {
    IRQ_EVT_IPC_CH1_BIT = 0,
    IRQ_EVT_IPC_CH2_BIT,
    IRQ_EVT_IPC_CH3_BIT,
    IRQ_EVT_IPC_CH4_BIT,
    IRQ_EVT_VSPA_BIT,
    IRQ_EVT_TEST_BIT,
    IRQ_EVT_LAST_BIT,
    IRQ_REAL_MSI_BIT
} la9310_irq_evt_bits_t;

struct la9310_evt_hdlr
{
    enum la9310_evt_type type;
    void * cookie;
    void (* mask) ( struct la9310_info * pLa9310Info,
                    la9310_irq_evt_bits_t evt_bit,
                    void * cookie );
    void (* unmask) ( struct la9310_info * pLa9310Info,
                      la9310_irq_evt_bits_t evt_bit,
                      void * cookie );
};

struct la9310_irq_evt_info
{
    int ievt_count;
    int ievt_en_mask;
    struct la9310_evt_hdlr * phdlr_tbl;
};

#endif /* ifndef __LA9310_IRQ_H__ */
