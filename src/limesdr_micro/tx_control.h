#ifndef LIME_TX_CONTROL_H
#define LIME_TX_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct transmission_event {
    int64_t time_point;
    uint32_t data_src_offset;
    uint32_t data_length;
    uint32_t flags;
} tx_event_t;

void tx_control_init(void);
void tx_control_clear_fifo(void);

void tx_start_immediate(void);
void tx_stop_immediate(void);

void process_tx_schedule(void);
int TxControlCommand(void *data);

#endif
