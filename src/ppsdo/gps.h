#ifndef LIME_GPS_H
#define LIME_GPS_H

#include "gps_data.h"

#include <stdbool.h>
#include <stdint.h>

#include "nmea.h"

// M10578-A3 GPS module's default baud rate differs between revisions or if it was modified by previous firmware run,
// need to detect the baudrate at runtime:
// each candidate rate is tried in turn, a PMTK test command is sent as a probe (it also wakes the module from standby).
// The rate is accepted once valid NMEA sentences are received.

// @brief Sets up UART interrupt and starts baud rate detection.
void gps_initialize(uint32_t uart_input_clock_hz);

// @brief Drains received data, runs baud rate detection and decodes sentences.
// Must be called periodically
void gps_process(void);

// @brief Reconfigures UART after its input clock changed.
void gps_set_uart_clock(uint32_t uart_clock_hz);

// @brief Sends a sentence, adding '$', the checksum and CRLF, e.g. gps_send_sentence("PMTK161,0").
// @return false if the link is not established or the sentence is too long.
bool gps_send_sentence(const char *body);

// @brief Request to generate PPS pulses.
// The request is (re)sent whenever UART link is established or the module restarts.
void gps_request_pps(bool required);

const gps_state_t *gps_get_state(void);
void gps_uart_irq_handler(void);

#endif // LIME_GPS_H
