#ifndef LIME_LA9310_OUTBOUND_MSI_EVENTS_H
#define LIME_LA9310_OUTBOUND_MSI_EVENTS_H

#include <stdint.h>

#define IPC_IRQ_PRIORITY 3
#define EDMA_IRQ_PRIORITY 3
#define MSG1_IRQ_PRIORITY 3
#define VSPA_IRQ_PRIORITY 3

// Indexes of the events that the M4 firmware signals to the host
// The driver only transports them, their meaning must agreed between the M4 firmware and the host application.
enum M4_EVENT {
    M4_MSI_GENERIC = 1,
    M4_MSI_RX_WRITE_DONE = 2,
    M4_MSI_TX_READ_DONE = 3,
};

// Initializes msi_events, with the MSI configuration that the host has set up by now, and publishes its address in the
// scratch registers
void la9310_msi_events_init(uint32_t *scratch_regs);
// event_bits: bit i signals event i, unless the host has it masked
void la9310_msi_events_raise(uint32_t event_bits);

#endif // LIME_LA9310_OUTBOUND_MSI_EVENTS_H
