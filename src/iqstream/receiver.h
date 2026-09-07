#ifndef LIME_M4_IQPLAYER_RECEIVER_H
#define LIME_M4_IQPLAYER_RECEIVER_H

#include "host_dma.h"
#include "vspa_dma_hif.h"

#include <stdint.h>

#define RX_MAX_PIPELINES_COUNT 1

typedef struct RxLane {
    host_dma_channel_t host_dma;
    uint16_t phytimer_id;
    vspa_dma_hif_t *vspa_dma;
    uint8_t oversample_pow2;
    uint8_t channel;
} rx_lane_t;

void receiver_init(void);
int receiver_lane_enable(uint16_t lane, bool enabled);
int receiver_lane_set_channel(uint16_t lane, uint16_t channel);
int receiver_lane_set_oversample(uint16_t lane, uint16_t oversample_pow2);

void receiver_handle_vspa_flags_irq(uint32_t flags);
void receiver_service(void);

// int rx_tcd_input(rx_lane_t* pipe);

#endif // LIME_M4_IQPLAYER_RECEIVER_H
