#ifndef LIME_M4_HOST_IF_H
#define LIME_M4_HOST_IF_H

#include <stdint.h>

#include "la9310_host_if.h"

// Legacy host interface, its sw_cmd_desc is the host command descriptor (LA9310_HIF_COMMAND)
extern volatile struct la9310_legacy_hif legacy_hif;

/// Resets the host interface and publishes its table in the DCR scratch registers, for the host to find
/// (LA9310_SCRATCH_HIF_TABLE_ADDR). Call it before the boot handshake completes.
void host_if_init(uint32_t *scratch_registers);

#endif // LIME_M4_HOST_IF_H
