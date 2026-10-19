#ifndef LIME_PPSDO_H
#define LIME_PPSDO_H

#include <stdbool.h>
#include <stdint.h>

/* Accumulates clock ticks over a window of `interval` PPS pulses. */
typedef struct PPS_Counter {
    uint64_t clock_count;
    uint64_t clock_target; /* Expected ticks over the whole window. */
    int32_t error_tolerance; /* Max tolerated |deviation| in ticks. */
    uint16_t interval; /* Window length in PPS pulses. */
    uint16_t pulse_count;
} pps_counter_t;

/* State machine for VCTCXO tuning. */
typedef enum state { COARSE_TUNE_MIN, COARSE_TUNE_MAX, COARSE_TUNE_DONE, FINE_TUNE, DO_NOTHING } ppsdo_tune_state_t;

/* Structure that represents a point on a line. Used for calibrating the
   VCTCXO.
 */
typedef struct point {
    int32_t x; /* Error counts. */
    uint16_t y; /* DAC count. */
} point_t;

typedef struct line {
    point_t point[2];
    float slope;
    uint16_t y_intercept; /* In DAC counts. */
} line_t;

typedef void (*ppsdo_dac_write_t)(uint16_t value);
typedef uint16_t (*ppsdo_dac_read_t)(void);

typedef struct PPSDO_Context {
    ppsdo_tune_state_t tune_state;
    line_t trimdac_cal_line;

    pps_counter_t pps_1s;
    pps_counter_t pps_10s;
    pps_counter_t pps_100s;

    uint16_t vctcxo_trim_dac_value;
    uint16_t trimdac_max;
    uint16_t trimdac_initial; /* DAC value before tuning was enabled. */

    ppsdo_dac_write_t dac_write;
    ppsdo_dac_read_t dac_read;

    bool enabled;
    bool skip_next_interval; /* Next PPS interval is not a valid measurement (DAC/clock changed). */

    /* Written by the PPS_IN interrupt, consumed by ppsdo_process(). */
    volatile uint32_t pps_delta;
    volatile bool pps_pending;
    volatile bool pps_overrun;
} ppsdo_context_t;

extern ppsdo_context_t ppsdo;

void ppsdo_initialize(ppsdo_context_t *ctx, ppsdo_dac_write_t dac_write, ppsdo_dac_read_t dac_read);
/* Enables or disables VCTCXO disciplining. Returns false if it can't be enabled. */
bool ppsdo_enable(ppsdo_context_t *ctx, bool enable);
/* Runs the tuning state machine on a captured PPS interval. Must be called from
   task context (it performs I2C transfers), from the same thread as other I2C users. */
void ppsdo_process(ppsdo_context_t *ctx);
void ppsdo_set_clock_frequency(ppsdo_context_t *ctx, uint32_t freq_hz);

void pps_out_initialize();

#endif // LIME_PPSDO_H
