#ifndef LIME_M4_IQPLAYER_RECEIVER_H
#define LIME_M4_IQPLAYER_RECEIVER_H

#include "host_dma.h"
#include "vspa_dma_hif.h"

#include <stdint.h>

#define RX_MAX_PIPELINES_COUNT 1

typedef struct RxLane {
    host_dma_channel_t host_dma;
    uint64_t next_completion_ts;
    uint16_t phytimer_id;
    volatile vspa_dma_hif_t *vspa_dma;
    uint8_t oversample_pow2;
} rx_lane_t;

void receiver_init(void);
int receiver_lane_enable(uint16_t lane, bool enabled);
int receiver_lane_set_channel(uint16_t lane, uint16_t channel);

void receiver_process_host_tcd_input(void);
void receiver_handle_vspa_flags_irq(uint32_t flags);
void receiver_service(void);

// int rx_tcd_input(rx_lane_t* pipe);

#endif // LIME_M4_IQPLAYER_RECEIVER_H
