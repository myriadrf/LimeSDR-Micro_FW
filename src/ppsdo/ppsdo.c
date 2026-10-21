#include "ppsdo.h"

#include "core_cm4.h"
#include "phytimer.h"
#include "la9310.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define PHYTIMER_HZ PHYTIMER_CLOCK_IN_FROM_PLATFORM(PLATFORM_CLOCK_HZ)

static uint32_t pps_in_count = 0;
static uint32_t last_pps_in_phytime = 0;
static uint32_t pps_high_duration_ticks = PHYTIMER_HZ / 10;
static uint32_t pps_low_duration_ticks = PHYTIMER_HZ - PHYTIMER_HZ / 10;
// PPS_IN intervals shorter than this are repeated triggers of the same pulse.
static uint32_t pps_in_min_delta = PHYTIMER_HZ / 2;
// PPS_IN intervals further than this from the nominal clock are not valid measurements.
static uint32_t pps_in_max_deviation = 0;
static uint32_t ppsdo_clock_hz = PHYTIMER_HZ;
ppsdo_context_t ppsdo;

static volatile bool pps_in_masked = false;
static volatile uint32_t pps_in_masked_phytime = 0;

uint8_t pps_out_stage = 0;
bool pps_out_resync_to_pps_in = false;

#define VCTCXO_DEFAULT_DAC_VALUE 0x77FA

// Maximum tolerated deviation, in clock ticks, for each measurement window.
#define PPS_1S_ERROR_TOLERANCE 4
#define PPS_10S_ERROR_TOLERANCE 4
#define PPS_100S_ERROR_TOLERANCE 4
// Measured 1s intervals deviating more than this from the nominal clock are discarded.
#define PPS_INTERVAL_MAX_DEVIATION_PPM 100

static uint32_t last_pps_out_phytime = 0;
static uint32_t last_pps_out_isr_phytime = 0;
static void schedule_pps_out(uint32_t phytime, enum ePhyTimerComparatorTrigger level)
{
    vPhyTimerComparatorConfig(
        PHY_TIMER_COMP_PPS_OUT, PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG, level, phytime);
}

/* Adds one PPS interval to the counter. Returns true when the counter's window is complete. */
static inline bool pps_counter_increment(pps_counter_t *counter, uint32_t delta)
{
    counter->clock_count += delta;
    return ++counter->pulse_count >= counter->interval;
}

static inline void pps_counter_reset(pps_counter_t *counter)
{
    counter->clock_count = 0;
    counter->pulse_count = 0;
}

static inline int32_t pps_counter_deviation(const pps_counter_t *counter)
{
    return (int32_t)((int64_t)counter->clock_target - (int64_t)counter->clock_count);
}

static inline bool pps_counter_out_of_tolerance(const pps_counter_t *counter, int32_t *deviation)
{
    *deviation = pps_counter_deviation(counter);
    return abs(*deviation) > counter->error_tolerance;
}

static void vctcxo_trim_dac_write(ppsdo_context_t *ctx, uint16_t dac_value)
{
    ctx->vctcxo_trim_dac_value = dac_value;
    if (ctx->dac_write)
        ctx->dac_write(dac_value);
}

static inline void pps_in_disable_irq(void)
{
    NVIC_DisableIRQ(IRQ_PPS_IN);
}

static inline void pps_in_reenable_irq(void)
{
    if (!pps_in_masked)
        NVIC_EnableIRQ(IRQ_PPS_IN);
}

static void pps_in_debounce(void)
{
    if (!pps_in_masked)
        return;
    const uint32_t delta = ulPhyTimerComparatorRead(10) - last_pps_in_phytime;
    if (delta > pps_high_duration_ticks && delta < 0x7FFFFFFFu)
    {
        pps_in_masked = false;
        NVIC_ClearPendingIRQ(IRQ_PPS_IN);
        NVIC_EnableIRQ(IRQ_PPS_IN);
    }
}

static void vctcxo_tamer_reset_counters(ppsdo_context_t *ctx)
{
    pps_counter_reset(&ctx->pps_1s);
    pps_counter_reset(&ctx->pps_10s);
    pps_counter_reset(&ctx->pps_100s);

    pps_in_disable_irq();
    ctx->pps_pending = false;
    ctx->pps_overrun = false;
    pps_in_reenable_irq();
    ctx->skip_next_interval = true;
}

static uint16_t clamp_trim_dac(const ppsdo_context_t *ctx, int32_t value)
{
    if (value > ctx->trimdac_max)
        return ctx->trimdac_max;
    if (value < 0)
        return 0;
    return value;
}

/* Adjusts the trim DAC value based on error, slope, and scale.
 *
 * @param error The PPS error value.
 * @param slope The calibration slope.
 * @param scale The scaling factor (1, 10, or 100).
 */
static void adjust_trim_dac(ppsdo_context_t *ctx, int32_t error, float slope, int scale)
{
    // Compute new trim DAC value
    // Use signed 32-bit integer for calculation to handle negatives
    int32_t new_value = ctx->vctcxo_trim_dac_value;
    new_value -= (int32_t)lroundf((float)error * slope / (float)scale);

    log_info("PPSDO %is error:%li, DAC %u -> %li" LOG_EOL, scale, error, ctx->vctcxo_trim_dac_value, new_value);

    // Write value to VCTCXO Tamer.
    vctcxo_trim_dac_write(ctx, clamp_trim_dac(ctx, new_value));
    vctcxo_tamer_reset_counters(ctx);
}

static void ppsdo_fsm_tune_min(ppsdo_context_t *ctx)
{
    log_dbg("COARSE_TUNE_MIN" LOG_EOL);

    // Set trim DAC to minimum value.
    vctcxo_trim_dac_write(ctx, ctx->trimdac_cal_line.point[0].y);
    vctcxo_tamer_reset_counters(ctx);

    // Set next interrupt state.
    ctx->tune_state = COARSE_TUNE_MAX;
}

static void ppsdo_fsm_tune_max(ppsdo_context_t *ctx)
{
    log_dbg("COARSE_TUNE_MAX" LOG_EOL);

    // We have the error from the minimum DAC setting, store it as
    // the 'x' coordinate for the first point.
    ctx->trimdac_cal_line.point[0].x = pps_counter_deviation(&ctx->pps_1s);

    vctcxo_trim_dac_write(ctx, ctx->trimdac_cal_line.point[1].y);
    vctcxo_tamer_reset_counters(ctx);
    ctx->tune_state = COARSE_TUNE_DONE;
}

static void ppsdo_fsm_tune_done(ppsdo_context_t *ctx)
{
    log_dbg("COARSE_TUNE_DONE" LOG_EOL);

    // We have the error from the maximum DAC setting
    // store it as the 'x' coordinate for the second point.
    ctx->trimdac_cal_line.point[1].x = pps_counter_deviation(&ctx->pps_1s);

    // We now have two points, so we can calculate the equation for
    // a line plotted with DAC counts on the Y axis and error on
    // the X axis. We want a PPM of zero, which ideally corresponds
    // to the y-intercept of the line.
    const int32_t dx = ctx->trimdac_cal_line.point[1].x - ctx->trimdac_cal_line.point[0].x;
    if (dx != 0)
    {
        const float dy = (float)ctx->trimdac_cal_line.point[1].y - (float)ctx->trimdac_cal_line.point[0].y;
        ctx->trimdac_cal_line.slope = dy / (float)dx;

        const int32_t y_intercept = ctx->trimdac_cal_line.point[0].y -
                                    (int32_t)lroundf(ctx->trimdac_cal_line.slope * (float)ctx->trimdac_cal_line.point[0].x);
        ctx->trimdac_cal_line.y_intercept = clamp_trim_dac(ctx, y_intercept);
    }
    else
    {
        // The DAC has no effect on the measured clock (e.g. external reference clock is used).
        log_err("PPSDO: clock does not respond to XO DAC, can't calibrate" LOG_EOL);
        ctx->trimdac_cal_line.y_intercept = ctx->trimdac_initial;
    }

    log_info("PPSDO calibration: err(%u)=%li, err(%u)=%li, DAC=%u" LOG_EOL,
        ctx->trimdac_cal_line.point[0].y,
        ctx->trimdac_cal_line.point[0].x,
        ctx->trimdac_cal_line.point[1].y,
        ctx->trimdac_cal_line.point[1].x,
        ctx->trimdac_cal_line.y_intercept);

    // Set the trim DAC count to the y-intercept.
    vctcxo_trim_dac_write(ctx, ctx->trimdac_cal_line.y_intercept);
    vctcxo_tamer_reset_counters(ctx);

    // Set next interrupt state.
    ctx->tune_state = (dx != 0) ? FINE_TUNE : DO_NOTHING;
}

static void ppsdo_fsm_tune_fine(ppsdo_context_t *ctx, bool pps_10s_done, bool pps_100s_done)
{
    log_dbg("FINE_TUNE" LOG_EOL);

    // We should be extremely close to a perfectly tuned VCTCXO, but
    // some minor adjustments need to be made.

    // Check the magnitude of the errors starting with the one second count.
    // If an error is greater than the maximum tolerated error,
    // adjust the trim DAC by the error (Hz) multiplied by the slope (in counts/Hz)
    // and scale the result by the precision interval (e.g. 1s, 10s, 100s).

    int32_t deviation;
    if (pps_counter_out_of_tolerance(&ctx->pps_1s, &deviation))
        adjust_trim_dac(ctx, deviation, ctx->trimdac_cal_line.slope, 1);
    else if (pps_10s_done && pps_counter_out_of_tolerance(&ctx->pps_10s, &deviation))
        adjust_trim_dac(ctx, deviation, ctx->trimdac_cal_line.slope, 10);
    else if (pps_100s_done && pps_counter_out_of_tolerance(&ctx->pps_100s, &deviation))
        adjust_trim_dac(ctx, deviation, ctx->trimdac_cal_line.slope, 100);
    else if (pps_100s_done)
        log_info("PPSDO locked, 100s error:%li, DAC:%u" LOG_EOL, deviation, ctx->vctcxo_trim_dac_value);
}

void ppsdo_initialize(ppsdo_context_t *ctx, ppsdo_dac_write_t dac_write, ppsdo_dac_read_t dac_read)
{
    /* Trim DAC constants. */
#ifdef CONFIG_DAC_MAX
    ctx->trimdac_max = CONFIG_DAC_MAX; /* Decimal value = Defined in SOC build process. */
#else
    ctx->trimdac_max = 0xFFFF; /* Decimal value = 65535. */
#endif
    ctx->tune_state = DO_NOTHING;

    // Set the known/default values of the trim DAC cal line.
    ctx->trimdac_cal_line.point[0].x = 0;
    ctx->trimdac_cal_line.point[0].y = 0;
    ctx->trimdac_cal_line.point[1].x = 0;
    ctx->trimdac_cal_line.point[1].y = ctx->trimdac_max;
    ctx->trimdac_cal_line.slope = 0;
    ctx->trimdac_cal_line.y_intercept = 0;

    ctx->dac_write = dac_write;
    ctx->dac_read = dac_read;
    // The board init has already set the XO DAC, so don't overwrite it here.
    ctx->vctcxo_trim_dac_value = dac_read ? dac_read() : VCTCXO_DEFAULT_DAC_VALUE;
    ctx->trimdac_initial = ctx->vctcxo_trim_dac_value;

    ctx->enabled = false;
    ctx->skip_next_interval = true;
    ctx->pps_pending = false;
    ctx->pps_overrun = false;

    ctx->pps_1s.interval = 1;
    ctx->pps_1s.error_tolerance = PPS_1S_ERROR_TOLERANCE;
    ctx->pps_10s.interval = 10;
    ctx->pps_10s.error_tolerance = PPS_10S_ERROR_TOLERANCE;
    ctx->pps_100s.interval = 100;
    ctx->pps_100s.error_tolerance = PPS_100S_ERROR_TOLERANCE;
    pps_counter_reset(&ctx->pps_1s);
    pps_counter_reset(&ctx->pps_10s);
    pps_counter_reset(&ctx->pps_100s);
    ppsdo_set_clock_frequency(ctx, ppsdo_clock_hz);

    pps_out_initialize();

    vPhyTimerPPSINEnable();
}

bool ppsdo_enable(ppsdo_context_t *ctx, bool enable)
{
    if (enable)
    {
        if (!ctx->dac_write || !ctx->dac_read)
            return false;
        if (ctx->enabled)
            return true;

        ctx->trimdac_initial = ctx->dac_read();
        ctx->vctcxo_trim_dac_value = ctx->trimdac_initial;
        ctx->tune_state = COARSE_TUNE_MIN;
        vctcxo_tamer_reset_counters(ctx);
        ctx->enabled = true;
        log_info("PPSDO enabled, DAC:%u" LOG_EOL, ctx->trimdac_initial);
    }
    else
    {
        if (!ctx->enabled)
            return true;

        ctx->enabled = false;
        // Calibration did not complete, the DAC might be left at its min/max value.
        if (ctx->tune_state != FINE_TUNE)
            vctcxo_trim_dac_write(ctx, ctx->trimdac_initial);
        ctx->tune_state = DO_NOTHING;
        log_info("PPSDO disabled, DAC:%u" LOG_EOL, ctx->vctcxo_trim_dac_value);
    }
    return true;
}

void ppsdo_process(ppsdo_context_t *ctx)
{
    pps_in_debounce();
    if (!ctx->pps_pending)
        return;

    pps_in_disable_irq();
    const uint32_t delta = ctx->pps_delta;
    const bool overrun = ctx->pps_overrun;
    ctx->pps_pending = false;
    ctx->pps_overrun = false;
    pps_in_reenable_irq();

    if (!ctx->enabled || ctx->tune_state == DO_NOTHING)
        return;

    // Missed PPS pulses, or noise on the PPS input, invalidate the measurement.
    const int32_t interval_error = (int32_t)(delta - ppsdo_clock_hz);
    if (overrun || (uint32_t)abs(interval_error) > pps_in_max_deviation)
    {
        log_dbg("PPSDO: discarding PPS interval %u" LOG_EOL, delta);
        vctcxo_tamer_reset_counters(ctx);
        return;
    }

    if (ctx->skip_next_interval)
    {
        ctx->skip_next_interval = false;
        return;
    }

    const bool pps_1s_done = pps_counter_increment(&ctx->pps_1s, delta);
    const bool pps_10s_done = pps_counter_increment(&ctx->pps_10s, delta);
    const bool pps_100s_done = pps_counter_increment(&ctx->pps_100s, delta);

    switch (ctx->tune_state)
    {
    case COARSE_TUNE_MIN:
        ppsdo_fsm_tune_min(ctx);
        break;

    case COARSE_TUNE_MAX:
        ppsdo_fsm_tune_max(ctx);
        break;

    case COARSE_TUNE_DONE:
        ppsdo_fsm_tune_done(ctx);
        break;

    case FINE_TUNE:
        ppsdo_fsm_tune_fine(ctx, pps_10s_done, pps_100s_done);
        break;

    default:
        break;
    }

    // Start a new window on the counters that have completed theirs.
    if (pps_1s_done)
        pps_counter_reset(&ctx->pps_1s);
    if (pps_10s_done)
        pps_counter_reset(&ctx->pps_10s);
    if (pps_100s_done)
        pps_counter_reset(&ctx->pps_100s);
}

void ppsdo_set_clock_frequency(ppsdo_context_t *ctx, uint32_t freq_hz)
{
    // The calibration slope is in DAC counts per clock tick, rescale it to the new tick rate.
    if (ppsdo_clock_hz != 0 && freq_hz != 0)
        ctx->trimdac_cal_line.slope *= (float)ppsdo_clock_hz / (float)freq_hz;
    ppsdo_clock_hz = freq_hz;

    ctx->pps_1s.clock_target = freq_hz;
    ctx->pps_10s.clock_target = (uint64_t)freq_hz * 10;
    ctx->pps_100s.clock_target = (uint64_t)freq_hz * 100;
    pps_in_max_deviation = (uint32_t)((uint64_t)freq_hz * PPS_INTERVAL_MAX_DEVIATION_PPM / 1000000);
    pps_in_min_delta = freq_hz / 2;

    pps_high_duration_ticks = freq_hz / 10;
    pps_low_duration_ticks = freq_hz - pps_high_duration_ticks;

    if (ctx->enabled)
    {
        // Coarse calibration measurements taken at the old clock are not comparable, start over.
        if (ctx->tune_state != FINE_TUNE && ctx->tune_state != DO_NOTHING)
            ctx->tune_state = COARSE_TUNE_MIN;
        vctcxo_tamer_reset_counters(ctx);
    }
}

void pps_out_initialize()
{
    NVIC_SetPriority(IRQ_PPS_OUT, 1);
    NVIC_EnableIRQ(IRQ_PPS_OUT);

    vPhyTimerComparatorForce(PHY_TIMER_COMP_PPS_OUT, ePhyTimerComparatorOut0);
    schedule_pps_out(last_pps_out_phytime + (pps_high_duration_ticks + pps_low_duration_ticks), ePhyTimerComparatorOut0);
}

// Overrides the weak default in phytimer.c.
void vPhyTimerPPSINHandler()
{
    // PPS_IN interrupt is triggered by signal level.
    // GPS module PPS signal is active for some time duration
    // that can result in multiple consecutive interrupt handler calls on each PPS
    // PPS_IN irq priority is higher than others, so constant triggering of PPS_IN ISR,
    // would delay processing of other interrupts.

    // Disable interrupt for the duration of PPS, ppsdo_process() reenables it once the pulse is over.
    NVIC_DisableIRQ(IRQ_PPS_IN);
    NVIC_ClearPendingIRQ(IRQ_PPS_IN);

    const uint32_t phytime = ulPhyTimerComparatorRead(PHY_TIMER_COMP_PPS_IN);
    pps_in_masked_phytime = phytime;
    pps_in_masked = true;
    const uint32_t delta = phytime - last_pps_in_phytime;
    if (delta < pps_in_min_delta)
        return; // repeated trigger of the same pulse
    last_pps_in_phytime = phytime;

    log_info("PPS_IN[%i]:%08x, d:%u" LOG_EOL, pps_in_count, phytime, delta);
    ++pps_in_count;

    // The tuning itself writes the XO DAC over I2C, so it's done in ppsdo_process() from task context.
    if (ppsdo.pps_pending)
        ppsdo.pps_overrun = true;
    ppsdo.pps_delta = delta;
    ppsdo.pps_pending = true;

    pps_out_resync_to_pps_in = true;
    // schedule_pps_out(last_pps_in_phytime + pps_high_duration_ticks, ePhyTimerComparatorOut0);
}

void vPhyTimerPPSOUTHandler()
{
    NVIC_ClearPendingIRQ(IRQ_PPS_OUT);
    uint32_t phytime = ulPhyTimerComparatorRead(PHY_TIMER_COMP_PPS_OUT);
    const uint32_t delta = phytime - last_pps_out_isr_phytime;
    last_pps_out_isr_phytime = phytime;
    // log_info("PPS_OUT[%i]:%08x, d:%u" LOG_EOL, pps_out_stage & 1, phytime, delta);

    enum ePhyTimerComparatorTrigger next_level;
    uint32_t next_isr_phytime;
    if (pps_out_stage & 1) // schedule rising edge
    {
        next_isr_phytime = last_pps_out_isr_phytime + pps_low_duration_ticks;
        next_level = ePhyTimerComparatorOut1;
    }
    else // schedule falling edge
    {
        if (pps_out_resync_to_pps_in)
        {
            next_isr_phytime = last_pps_in_phytime + pps_high_duration_ticks;
            pps_out_resync_to_pps_in = false;
        }
        else
            next_isr_phytime = last_pps_out_isr_phytime + pps_high_duration_ticks;
        next_level = ePhyTimerComparatorOut0;
    }
    schedule_pps_out(next_isr_phytime, next_level);
    ++pps_out_stage;
}
