#ifndef LIME_M4_IQPLAYER_TRANSMITTER_H
#define LIME_M4_IQPLAYER_TRANSMITTER_H

#include "host_dma.h"
#include "vspa_dma_hif.h"

#include <stdint.h>

#define TX_MAX_PIPELINES_COUNT 1

typedef struct TxLane {
    host_dma_channel_t host_dma;
    uint64_t next_completion_ts;
    uint16_t phytimer_id;
    volatile vspa_dma_hif_t *vspa_dma;
    uint8_t oversample_pow2;
    uint8_t next_burst_pending;
    uint8_t wait_trigger_change;
    uint8_t expected_trigger;
} tx_lane_t;

void transmitter_init(void);
int transmitter_lane_enable(uint16_t lane, bool enabled);
int transmitter_lane_select_channel(uint16_t lane, uint16_t channel);

void transmitter_process_host_tcd_input(void);
void transmitter_handle_vspa_flags_irq(uint32_t flags);

void transmitter_service(void);

#endif // LIME_M4_IQPLAYER_TRANSMITTER_H
