#ifndef LIME_STREAMER_H
#define LIME_STREAMER_H

#include <stdint.h>
#include <stdbool.h>

#include "vspa_dma_hif.h"
#include "host_dma.h"

#define IQSTREAM_AFE_PAYLOAD_SIZE 2048 // granurality of data transfers to ADC/DAC

void iqstream_init(void);

int iqstream_enable(uint32_t rx_mask, uint32_t tx_mask);
int iqstream_disable(uint32_t rx_mask, uint32_t tx_mask);

void iqstream_service(void);
void iqstream_process_dma_tcd_input(void);

void iqstream_handle_vspa_dma_irq(uint32_t dma_irq_stat);
void iqstream_handle_vspa_flags_irq(uint32_t flags);
void iqstream_handle_axiq_irq(void);

bool push_tcd_to_vspa(volatile vspa_dma_hif_t *hif, const dma_tcd_t *tcd);

extern uint64_t stream_phytime_origin;
extern uint64_t stream_phytime_origin_rx;

#endif // LIME_STREAMER_H