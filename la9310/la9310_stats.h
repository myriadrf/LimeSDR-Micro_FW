#ifndef LA9310_STATS_H
#define LA9310_STATS_H

#define MAX_SENT_RESUME 5
#define LA9310_eDMA_CHANNELS 14

struct la9310_eDMA {
    uint8_t status;
    uint32_t xfer_req;
    uint32_t success_interrupt;
    uint32_t error_interrupt;
    uint32_t no_callback_reg;
};

typedef struct la9310_stats {
    uint32_t disabled_evt_try_cnt;
    uint32_t irq_evt_raised;
    uint32_t irq_evt_cleared;
    uint32_t irq_mux_tx_msi_cnt;
    uint32_t irq_mux_rx_msi_cnt;
    uint32_t avi_cm4_mbox0_tx_cnt;
    uint32_t avi_cm4_mbox1_tx_cnt;
    uint32_t avi_cm4_mbox0_rx_cnt;
    uint32_t avi_cm4_mbox1_rx_cnt;
    uint32_t avi_err_queue_full;
    uint32_t avi_intr_raised;
    uint32_t avi_mbox_intr_raised;
    uint32_t eDMA_ch_allocated;
    struct la9310_eDMA la9310_eDMA_ch[LA9310_eDMA_CHANNELS];
    uint32_t WDOG_interrupt;
    uint32_t v2h_intr_enabled;
    uint32_t dbg1;
    uint32_t dbg2;
} la9310_stats_t;

extern la9310_stats_t la9310_global_stats;

#endif
