// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#include "profiling.h"

static uint64_t cycles_high = 0; // wraps counted so far, in cycles
static uint32_t cycles_last = 0; // counter value at the last profiling_cycles_read64()

void profiling_cycles_init(void)
{
    PROFILING_SYST_CSR = 0;
    PROFILING_SYST_RVR = PROFILING_CYCLES_MASK;
    profiling_cycles_reset();
    PROFILING_SYST_CSR = PROFILING_SYST_CSR_CLKSOURCE | PROFILING_SYST_CSR_ENABLE; // no TICKINT, free running
}

void profiling_cycles_reset(void)
{
    uint32_t primask;
    __asm volatile("MRS %0, primask\n\tCPSID i" : "=r"(primask)::"memory");
    PROFILING_SYST_CVR = 0; // any write clears it, it reloads with PROFILING_CYCLES_MASK on the next clock
    cycles_high = 0;
    cycles_last = 0;
    __asm volatile("MSR primask, %0" ::"r"(primask) : "memory");
}

uint64_t profiling_cycles_read64(void)
{
    uint32_t primask;
    __asm volatile("MRS %0, primask\n\tCPSID i" : "=r"(primask)::"memory");
    const uint32_t now = profiling_cycles_read();
    if (now < cycles_last)
        cycles_high += 1ULL << PROFILING_CYCLES_BITS;
    cycles_last = now;
    const uint64_t result = cycles_high + now;
    __asm volatile("MSR primask, %0" ::"r"(primask) : "memory");
    return result;
}

void profiling_cycles_stat_clear(profiling_cycles_stat_t *stat)
{
    stat->count = 0;
    stat->last = 0;
    stat->min = UINT32_MAX;
    stat->max = 0;
    stat->total = 0;
}

void profiling_cycles_stat_add(profiling_cycles_stat_t *stat, uint32_t cycles)
{
    if (stat->count == 0 || cycles < stat->min)
        stat->min = cycles;
    if (cycles > stat->max)
        stat->max = cycles;
    stat->last = cycles;
    stat->total += cycles;
    ++stat->count;
}
