// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#include "vspa_debug.h"
#include "core_cm4.h"
#include "immap.h"
#include "io.h"
#ifdef M4_DEBUGGER
    #include "m4_monitor.h"
#endif

#ifdef VSPA_DEBUGGER
#define VSPA_DBG_BASE 0xE0046000u
#define VSPA_DBG_SPAN 0x1000u
#endif

#define MSIR3 (MSG_UNIT_BASE_ADDR + 0x14u) /* clear on read */

#ifdef M4_DEBUGGER
#define CCR_BFHFNMIGN (1u << 8)
#define CFSR_BFSR_MASK 0x0000FF00u
#define CFSR_BUS_ERROR ((1u << 9) | (1u << 10)) /* PRECISERR, IMPRECISERR */

/* Regions op 3/4 may touch; holes inside them end the request with a bus error. */
static const struct {
    uint32_t start, end;
} m4_regions[] = {
    {0x1F800000u, 0x1F820000u}, /* TCML */
    {0x20000000u, 0x20010000u}, /* TCMU */
    {0x40000000u, 0x50000000u}, /* CCSR */
    {0xE0000000u, 0xE0100000u}, /* private peripheral bus */
};
#endif

/* 64-byte aligned: data[] starts on a 32-byte boundary for the host's block reads */
static volatile struct vspa_dbg_proxy proxy __attribute__((aligned(64)));

void VspaDebugProxyInit(volatile uint32_t *scratch_regs)
{
    proxy.version = VSPA_DBG_VERSION;
    proxy.self = (uint32_t)&proxy;
    proxy.max_words = VSPA_DBG_MAX_WORDS;
    proxy.cmd = 0;
    proxy.status = 0;
    proxy.magic = VSPA_DBG_MAGIC;
    dsb();
    scratch_regs[VSPA_DBG_SCRATCH_PTR] = (uint32_t)&proxy;
}

#ifdef M4_DEBUGGER
/* Words at any M4 address. A bus error must not fault the firmware: the
 * accesses run at priority -1 (FAULTMASK) with CCR.BFHFNMIGN set, where the
 * core ignores data bus faults and only records them in BFSR. */
static uint32_t M4Access(uint32_t op, uint32_t addr, uint32_t count, uint32_t fixed)
{
    const uint32_t span = fixed ? 4u : count * 4u;
    uint32_t inside = 0;
    for (uint32_t i = 0; i < sizeof(m4_regions) / sizeof(m4_regions[0]); i++)
        if (addr >= m4_regions[i].start && addr < m4_regions[i].end && span <= m4_regions[i].end - addr)
            inside = 1;
    if ((addr & 3u) || !inside)
        return VSPA_DBG_ERR_OFFSET;

    uint32_t faultmask;
    __asm volatile("mrs %0, faultmask" : "=r"(faultmask));
    __asm volatile("cpsid f" ::: "memory");
    const uint32_t ccr = SCB->CCR;
    SCB->CCR = ccr | CCR_BFHFNMIGN;
    SCB->CFSR = CFSR_BFSR_MASK; /* write 1 to clear */
    dsb();
    isb();

    uint32_t error = 0;
    for (uint32_t i = 0; i < count && !error; i++)
    {
        if (op == VSPA_DBG_OP_M4_READ)
            proxy.data[i] = IN_32(addr);
        else
            OUT_32(addr, proxy.data[i]);
        dsb();
        if (SCB->CFSR & CFSR_BUS_ERROR)
            error = VSPA_DBG_ERR_BUS;
        if (!fixed)
            addr += 4u;
    }

    SCB->CFSR = CFSR_BFSR_MASK;
    SCB->CCR = ccr;
    dsb();
    isb();
    if (!faultmask)
        __asm volatile("cpsie f" ::: "memory");
    return error;
}
#endif

static uint32_t Execute(void)
{
    const uint32_t op = proxy.cmd & 0xFFu;
    const uint32_t fixed = proxy.cmd & VSPA_DBG_FLAG_FIXED;
    const uint32_t count = proxy.count;
    const uint32_t offset = proxy.offset;

#ifdef M4_DEBUGGER
    if (op == VSPA_DBG_OP_MON_INFO)
    {
        proxy.data[0] = M4MonitorAddress();
        proxy.data[1] = M4_MON_VERSION;
        return 0;
    }
    if (op == VSPA_DBG_OP_MON_HALT)
    {
        M4MonitorRequestHalt();
        return 0;
    }
    if (op == VSPA_DBG_OP_M4_READ || op == VSPA_DBG_OP_M4_WRITE)
    {
        if (count == 0 || count > VSPA_DBG_MAX_WORDS)
            return VSPA_DBG_ERR_COUNT;
        return M4Access(op, offset, count, fixed);
    }
#endif
#ifdef VSPA_DEBUGGER
    if (op == VSPA_DBG_OP_READ || op == VSPA_DBG_OP_WRITE)
    {
        if (count == 0 || count > VSPA_DBG_MAX_WORDS)
            return VSPA_DBG_ERR_COUNT;
        const uint32_t span = fixed ? 4u : count * 4u;
        if ((offset & 3u) || offset >= VSPA_DBG_SPAN || span > VSPA_DBG_SPAN - offset)
            return VSPA_DBG_ERR_OFFSET;

        uint32_t addr = VSPA_DBG_BASE + offset;
        for (uint32_t i = 0; i < count; i++)
        {
            if (op == VSPA_DBG_OP_READ)
                proxy.data[i] = IN_32(addr);
            else
                OUT_32(addr, proxy.data[i]);
            if (!fixed)
                addr += 4u;
        }
        return 0;
    }
#endif
    (void)fixed;
    (void)count;
    (void)offset;
    return VSPA_DBG_ERR_OP;
}

void VspaDebugProxyPoll(void)
{
    if (!IN_32(MSIR3)) /* clear on read */
        return;
    NVIC_ClearPendingIRQ(IRQ_MSG3);
    /* a request that was already served (by the interrupt, or by the
     * monitor's polling loop) carries the tag of the status word */
    const uint32_t tag = proxy.cmd & 0xFF000000u;
    if (tag == (proxy.status & 0xFF000000u))
        return;
#ifdef M4_DEBUGGER
    const uint32_t debugmon = M4MonitorDeferStops();
#endif
    const uint32_t error = Execute();
    dsb();
    proxy.status = tag | error;
#ifdef M4_DEBUGGER
    dsb();
    M4MonitorAllowStops(debugmon);
#endif
}
