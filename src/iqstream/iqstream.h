#ifndef LIME_STREAMER_H
#define LIME_STREAMER_H

#include <stdint.h>
#include <stdbool.h>

#define IQSTREAM_AFE_PAYLOAD_SIZE 2048 // granurality of data transfers to ADC/DAC

void iqstream_init(void);

int iqstream_enable(uint32_t rx_mask, uint32_t tx_mask);
int iqstream_disable(uint32_t rx_mask, uint32_t tx_mask);

void iqstream_handle_vspa_flags_irq(uint32_t flags);

void iqstream_vspa_irq_handler(void);

int vspa_command_sync(uint64_t vspa_msg64);

#endif // LIME_STREAMER_H