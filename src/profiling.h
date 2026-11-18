// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_PROFILING_H
#define LIME_PROFILING_H

#include <stdint.h>

// Cortex-M4 core cycle counting.
//
// The LA9310 M4 has the minimal DWT configuration without the cycle counter (DWT_CTRL.NOCYCCNT is set), so the
// SysTick timer is used instead: it runs free from the processor clock, without interrupts, and its down-counting
// value is presented as an up-counting one.
//
// All values are raw core clock cycles. The counter is 24-bit and wraps every 2^24 cycles (~68 ms at 245.76 MHz).
// Durations computed with profiling_cycles_elapsed() are wrap-safe as long as the measured interval is shorter than one wrap
// period. For longer intervals use profiling_cycles_read64(), which must be called at least once per wrap period to keep
// track of wraps.
//
// Usage:
//     profiling_cycles_init(); // once at startup
//
//     uint32_t t0 = profiling_cycles_read();
//     do_work();
//     uint32_t cycles = profiling_cycles_elapsed(t0);
//
//     static profiling_cycles_stat_t work_stat;
//     PROFILING_CYCLES_MEASURE(&work_stat) { do_work(); }
//     log_info("min %u max %u avg %u cycles" LOG_EOL, work_stat.min, work_stat.max, profiling_cycles_stat_avg(&work_stat));

#define PROFILING_CYCLES_BITS 24
#define PROFILING_CYCLES_MASK ((1UL << PROFILING_CYCLES_BITS) - 1)

// SysTick registers
#define PROFILING_SYST_CSR (*(volatile uint32_t *)0xE000E010UL)
#define PROFILING_SYST_RVR (*(volatile uint32_t *)0xE000E014UL)
#define PROFILING_SYST_CVR (*(volatile uint32_t *)0xE000E018UL)
#define PROFILING_SYST_CSR_ENABLE (1UL << 0)
#define PROFILING_SYST_CSR_TICKINT (1UL << 1)
#define PROFILING_SYST_CSR_CLKSOURCE (1UL << 2) // processor clock

// Starts the cycle counter from 0. Takes over SysTick, which must not be used for anything else.
void profiling_cycles_init(void);

// Resets the cycle counter to 0, including the 64-bit extension.
void profiling_cycles_reset(void);

// Current cycle count, 0 .. PROFILING_CYCLES_MASK.
static inline uint32_t profiling_cycles_read(void)
{
    return ~PROFILING_SYST_CVR & PROFILING_CYCLES_MASK;
}

// Cycles elapsed since `start`, correct across a single counter wrap.
static inline uint32_t profiling_cycles_elapsed(uint32_t start)
{
    return (profiling_cycles_read() - start) & PROFILING_CYCLES_MASK;
}

// Current cycle count extended to 64 bits. Must be called at least once per counter wrap period (2^24 cycles).
// Safe to call from interrupts.
uint64_t profiling_cycles_read64(void);

// Busy waits for the given number of cycles, less than 2^24.
static inline void profiling_cycles_delay(uint32_t cycles)
{
    const uint32_t start = profiling_cycles_read();
    while (profiling_cycles_elapsed(start) < cycles)
    {
    }
}

// Accumulated duration statistics of a repeatedly measured code section.
typedef struct profiling_cycles_stat_s {
    uint32_t count; // number of measurements
    uint32_t last; // last measured duration
    uint32_t min;
    uint32_t max;
    uint64_t total; // sum of all measured durations
} profiling_cycles_stat_t;

void profiling_cycles_stat_clear(profiling_cycles_stat_t *stat);

// Adds one measured duration to the statistics.
void profiling_cycles_stat_add(profiling_cycles_stat_t *stat, uint32_t cycles);

// Adds the duration since `start` to the statistics and returns it.
static inline uint32_t profiling_cycles_stat_stop(profiling_cycles_stat_t *stat, uint32_t start)
{
    const uint32_t cycles = profiling_cycles_elapsed(start);
    profiling_cycles_stat_add(stat, cycles);
    return cycles;
}

static inline uint32_t profiling_cycles_stat_avg(const profiling_cycles_stat_t *stat)
{
    return stat->count ? (uint32_t)(stat->total / stat->count) : 0;
}

// Measures the following statement or block and adds its duration to `stat`.
// The block must not exit with break, return or goto, else the measurement is lost.
//     PROFILING_CYCLES_MEASURE(&stat) { ... }
#define PROFILING_CYCLES_MEASURE(stat) \
    for (uint32_t _prof_start = profiling_cycles_read(), _prof_once = 1; _prof_once; \
         _prof_once = 0, profiling_cycles_stat_stop((stat), _prof_start))

#endif // LIME_PROFILING_H
