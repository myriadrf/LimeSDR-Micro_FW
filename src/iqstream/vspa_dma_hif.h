#ifndef LIME_VSPA_IQPLAYER_H
#define LIME_VSPA_IQPLAYER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct VSPA_DMA_TCD {
    uint32_t address;
    uint32_t size;
    uint32_t flags;
} vspa_dma_tcd_t;

// Directly accessable data from host for TCD submission and status readback
typedef struct VSPA_DMA_HIF {
    vspa_dma_tcd_t input_tcd;
    uint32_t tcd_done_counter; // how many transactions have been completed
    uint32_t htv_tcd_pending_flag_mask; // Host to VSPA signal that input TCD is prepared
    uint32_t vth_tcd_done_flag_mask; // VSPA to host, signal that TCD has been completed
} vspa_dma_hif_t;

void signal_to_vspa(uint32_t flags);
uint32_t vspa_signal_status();

#endif // LIME_VSPA_IQPLAYER_H
