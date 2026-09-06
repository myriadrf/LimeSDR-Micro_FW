#ifndef LIME_VSPA_DMA_HIF_H
#define LIME_VSPA_DMA_HIF_H

#include <stdint.h>
#include <stdbool.h>

#include "dma_tcd_fifo.h"

// Directly accessable data from host for TCD submission and status readback
typedef struct VSPA_DMA_HIF {
    dma_tcd_fifo_t tcd_fifo;
    uint32_t htv_tcd_pending_flag_mask; // Host to VSPA signal that input TCD is prepared
    uint32_t vth_tcd_done_flag_mask; // VSPA to host, signal that TCD has been completed
} vspa_dma_hif_t;

void signal_to_vspa(uint32_t flags);
uint32_t vspa_signal_status();

#endif // LIME_VSPA_DMA_HIF_H
