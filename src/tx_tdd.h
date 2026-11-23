#ifndef LIME_TX_TDD_H
#define LIME_TX_TDD_H

#include "limesdr_micro/m4_commands.h"

// Tx TDD timing the M4 uses to program the PHY timer comparators on the VSPA's TDD start/end requests.
// Written by the host through the host interface table (LA9310_HIF_TX_TDD_CONFIG).
extern volatile tx_tdd_config_t tx_tdd_config;

#endif
