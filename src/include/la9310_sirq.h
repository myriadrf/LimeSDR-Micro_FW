#ifndef LIME_LA9310_SIRQ_H
#define LIME_LA9310_SIRQ_H

#define IPC_IRQ_PRIORITY 3
#define EDMA_IRQ_PRIORITY 3
#define MSG1_IRQ_PRIORITY 3
#define VSPA_IRQ_PRIORITY 3

enum LA9310_SIGNALS {
    IRQ_FLAGS_CHANGED = 0,
    HOST_COMMAND_POSTED = 1,
};

enum LA9310_VIRQ {
    HOST_COMMAND_DONE = 1,
    VSPA_DDR_WRITE_DONE = 2,
    VSPA_DDR_READ_DONE = 3,
};

// Software based interrupts
struct la9310_sirq {
    uint32_t *counter_addr;
    uint32_t *status;
    uint32_t *enable;
    uint32_t *host_clear;
    uint32_t *host_msi_addr;
    uint32_t host_msi_data;
    uint32_t counter;
};

extern struct la9310_info g_la9310_info;

void la9310_sirq_initialize(struct la9310_sirq *sirq, uint32_t *scratch_regs);
void la9310_sirq_raise_events(struct la9310_sirq *sirq, uint32_t event_bits);

#endif // LIME_LA9310_SIRQ_H
