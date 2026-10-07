#ifndef LIME_M4_IQPLAYER_TRANSMITTER_H
#define LIME_M4_IQPLAYER_TRANSMITTER_H

#include "host_dma.h"
#include "vspa_dma_hif.h"

#include <stdint.h>

#define TX_MAX_PIPELINES_COUNT 1

typedef struct TxChannelConfig {
    uint8_t oversample_pow2;
} tx_config_t;

void transmitter_init(void);
int transmitter_lane_enable(uint16_t lane, bool enabled);
int transmitter_lane_select_channel(uint16_t lane, uint16_t channel);
int transmitter_lane_set_oversample(uint16_t lane, uint16_t oversample_pow2);

void transmitter_process_host_tcd_input(void);
void transmitter_handle_vspa_flags_irq(uint32_t flags);

void transmitter_service(void);

#endif // LIME_M4_IQPLAYER_TRANSMITTER_H
