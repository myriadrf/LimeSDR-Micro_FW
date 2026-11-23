// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#include "m4_monitor.h"
#include "core_cm4.h"
#include "exceptions.h"
#include "io.h"
#include "vspa_debug.h"

/* Core debug registers (ARMv7-M ARM, C1.6) */
#define DHCSR (*(volatile uint32_t *)0xE000EDF0u)
#define DHCSR_C_DEBUGEN (1u << 0)
#define DEMCR (*(volatile uint32_t *)0xE000EDFCu)
#define DEMCR_TRCENA (1u << 24)
#define DEMCR_MON_REQ (1u << 19)
#define DEMCR_MON_STEP (1u << 18)
#define DEMCR_MON_PEND (1u << 17)
#define DEMCR_MON_EN (1u << 16)

/* Flash Patch and Breakpoint unit */
#define FP_CTRL (*(volatile uint32_t *)0xE0002000u)
#define FP_CTRL_KEY (1u << 1)
#define FP_COMP(n) (*(volatile uint32_t *)(0xE0002008u + 4u * (n)))

/* Data Watchpoint and Trace unit */
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_COMP(n) (*(volatile uint32_t *)(0xE0001020u + 16u * (n)))
#define DWT_FUNCTION(n) (*(volatile uint32_t *)(0xE0001028u + 16u * (n)))
#define DWT_FUNCTION_MATCHED (1u << 24)

#define SHP_DEBUGMON 8u /* SCB->SHP[] index of exception 12 */
#define EXC_DEBUGMON 12u
#define PRIO_SHIFT (8u - __NVIC_PRIO_BITS)
/* BASEPRI while stepping: masks every interrupt below DebugMonitor */
#define STEP_BASEPRI ((M4_MON_PRIORITY + 1u) << PRIO_SHIFT)

/* EXC_RETURN bits */
#define EXC_RETURN_PSP (1u << 2)
#define EXC_RETURN_BASIC_FRAME (1u << 4)
#define XPSR_FRAME_PAD (1u << 9)

static volatile struct m4_monitor mon __attribute__((aligned(64)));
static uint32_t step_saved_basepri;
static uint32_t step_masked;

static uint32_t FpbCodeComparators(void)
{
    const uint32_t ctrl = FP_CTRL;
    return ((ctrl >> 8) & 0x70u) | ((ctrl >> 4) & 0xFu);
}

static uint32_t DwtComparators(void)
{
    return DWT_CTRL >> 28;
}

void M4MonitorInit(void)
{
    mon.version = M4_MON_VERSION;
    mon.self = (uint32_t)&mon;
    mon.size = sizeof(mon);
    mon.state = M4_MON_RUNNING;
    mon.halt_count = 0;
    mon.resume = M4_MON_RESUME_NONE;
    mon.magic = M4_MON_MAGIC;
    step_masked = 0;

    DEMCR &= ~(DEMCR_MON_PEND | DEMCR_MON_STEP | DEMCR_MON_REQ);
    DEMCR |= DEMCR_TRCENA; /* DWT register access */
    dsb();

    /* A firmware reload does not reset the core: drop comparators that the
     * previous firmware's debug session left armed, unless a JTAG debugger
     * owns them. */
    if (!(DHCSR & DHCSR_C_DEBUGEN))
    {
        FP_CTRL = FP_CTRL_KEY; /* disabled */
        for (uint32_t n = 0; n < FpbCodeComparators(); n++)
            FP_COMP(n) = 0;
        for (uint32_t n = 0; n < DwtComparators(); n++)
        {
            DWT_FUNCTION(n) = 0;
            (void)DWT_FUNCTION(n); /* clears a stale MATCHED */
        }
    }

    SCB->SHP[SHP_DEBUGMON] = (M4_MON_PRIORITY << PRIO_SHIFT) & 0xFFu;
    SCB->DFSR = SCB->DFSR; /* write 1 to clear */
    DEMCR |= DEMCR_MON_EN;
    dsb();
    isb();
}

uint32_t M4MonitorAddress(void)
{
    return (uint32_t)&mon;
}

/* priority of the exception the caller runs in (0 for thread mode) */
static uint8_t CurrentPriority(void)
{
    uint32_t ipsr;
    __asm volatile("mrs %0, ipsr" : "=r"(ipsr));
    if (ipsr >= 16u)
        return NVIC->IP[ipsr - 16u];
    if (ipsr >= 4u)
        return SCB->SHP[ipsr - 4u];
    return 0;
}

void M4MonitorRequestHalt(void)
{
    if (mon.state == M4_MON_HALTED)
        return;
    /* At the caller's (the proxy interrupt's) priority, DebugMonitor cannot
     * preempt it, and tail-chains when it returns: the stop lands in the
     * code the proxy interrupted, not in the proxy. M4MonitorStop() puts
     * the priority back. */
    SCB->SHP[SHP_DEBUGMON] = CurrentPriority();
    DEMCR |= DEMCR_MON_PEND;
    dsb();
}

uint32_t M4MonitorDeferStops(void)
{
    const uint32_t saved = SCB->SHP[SHP_DEBUGMON];
    SCB->SHP[SHP_DEBUGMON] = CurrentPriority();
    dsb();
    isb();
    return saved;
}

void M4MonitorAllowStops(uint32_t saved)
{
    /* A stop that came up meanwhile (a halt request, a cross-trigger halt
     * request, a watchpoint) stays deferred: it tail-chains when the
     * interrupt returns, and M4MonitorStop() restores the priority. */
    if ((DEMCR & DEMCR_MON_PEND) || ((SCB->ICSR >> 12) & 0x1FFu) == EXC_DEBUGMON)
        return;
    SCB->SHP[SHP_DEBUGMON] = saved;
    dsb();
}

static void Snapshot(uint32_t *frame, uint32_t *callee, uint32_t exc_return, uint32_t exception)
{
    mon.exception = exception;
    mon.dfsr = SCB->DFSR;
    mon.hfsr = SCB->HFSR;
    mon.cfsr = SCB->CFSR;
    mon.mmfar = SCB->MMFAR;
    mon.bfar = SCB->BFAR;
    uint32_t matched = 0;
    for (uint32_t n = 0; n < DwtComparators(); n++)
        if (DWT_FUNCTION(n) & DWT_FUNCTION_MATCHED) /* clear on read */
            matched |= 1u << n;
    mon.dwt_matched = matched;
    mon.exc_return = exc_return;
    mon.frame = (uint32_t)frame;

    for (uint32_t i = 0; i < 4; i++)
        mon.regs[i] = frame[i];
    for (uint32_t i = 0; i < 8; i++)
        mon.regs[4 + i] = callee[i];
    mon.regs[12] = frame[4];
    mon.regs[M4_MON_REG_LR] = frame[5];
    mon.regs[M4_MON_REG_PC] = frame[6];
    mon.regs[M4_MON_REG_XPSR] = frame[7];
    /* the stopped code's SP: above the frame and its alignment padding */
    uint32_t sp = (uint32_t)frame + ((exc_return & EXC_RETURN_BASIC_FRAME) ? 0x20u : 0x68u);
    if (frame[7] & XPSR_FRAME_PAD)
        sp += 4u;
    mon.regs[M4_MON_REG_SP] = sp;

    uint32_t msp, psp, primask, basepri, faultmask, control;
    __asm volatile("mrs %0, msp" : "=r"(msp));
    __asm volatile("mrs %0, psp" : "=r"(psp));
    __asm volatile("mrs %0, primask" : "=r"(primask));
    __asm volatile("mrs %0, basepri" : "=r"(basepri));
    __asm volatile("mrs %0, faultmask" : "=r"(faultmask));
    __asm volatile("mrs %0, control" : "=r"(control));
    if (exc_return & EXC_RETURN_PSP)
    {
        psp = sp;
        msp = (uint32_t)(callee + 10); /* MSP before the monitor entry pushed 10 words */
        control |= 2u; /* SPSEL of the stopped code */
    }
    else
        msp = sp;
    mon.msp = msp;
    mon.psp = psp;
    mon.primask = primask;
    mon.basepri = basepri;
    mon.faultmask = faultmask;
    mon.control = control;
}

static void WriteBack(uint32_t *frame, uint32_t *callee)
{
    for (uint32_t i = 0; i < 4; i++)
        frame[i] = mon.regs[i];
    for (uint32_t i = 0; i < 8; i++)
        callee[i] = mon.regs[4 + i];
    frame[4] = mon.regs[12];
    frame[5] = mon.regs[M4_MON_REG_LR];
    frame[6] = mon.regs[M4_MON_REG_PC] & ~1u;
    frame[7] = mon.regs[M4_MON_REG_XPSR] | (1u << 24); /* keep the Thumb bit */
}

/* Called by M4MonitorEntry with the exception frame, the saved r4-r11 and
 * EXC_RETURN; returns when the host resumes. */
__attribute__((used)) void M4MonitorStop(uint32_t *frame, uint32_t *callee, uint32_t exc_return)
{
    uint32_t exception;
    __asm volatile("mrs %0, ipsr" : "=r"(exception));
    exception &= 0x1FFu;

    /* undo M4MonitorRequestHalt() and a masked step */
    SCB->SHP[SHP_DEBUGMON] = (M4_MON_PRIORITY << PRIO_SHIFT) & 0xFFu;
    DEMCR &= ~(DEMCR_MON_PEND | DEMCR_MON_STEP);
    if (step_masked)
    {
        __asm volatile("msr basepri, %0" ::"r"(step_saved_basepri) : "memory");
        step_masked = 0;
    }

    Snapshot(frame, callee, exc_return, exception);
    if (exception != EXC_DEBUGMON)
        vLogFault(frame);
    mon.halt_count = mon.halt_count + 1u;
    dsb();
    mon.state = M4_MON_HALTED;
    dsb();

    /* The proxy interrupt cannot preempt this handler: serve it here. */
    uint32_t resume;
    while ((resume = mon.resume) == M4_MON_RESUME_NONE)
        VspaDebugProxyPoll();
    mon.resume = M4_MON_RESUME_NONE;

    WriteBack(frame, callee);
    SCB->DFSR = SCB->DFSR;
    if (exception != EXC_DEBUGMON)
    {
        SCB->CFSR = SCB->CFSR;
        SCB->HFSR = SCB->HFSR;
    }
    if (resume == M4_MON_RESUME_STEP || resume == M4_MON_RESUME_STEP_IRQ)
    {
        if (resume == M4_MON_RESUME_STEP)
        {
            /* BASEPRI is not part of the exception frame, so it holds in
             * the stopped code for the one step */
            __asm volatile("mrs %0, basepri" : "=r"(step_saved_basepri));
            __asm volatile("msr basepri, %0" ::"r"(STEP_BASEPRI) : "memory");
            step_masked = 1;
        }
        DEMCR |= DEMCR_MON_STEP;
    }
    mon.state = M4_MON_RUNNING;
    dsb();
    isb();
}

__attribute__((naked)) void M4MonitorEntry(void)
{
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "push {r4-r11, ip, lr}\n" /* 10 words: keeps 8-byte alignment */
                   "mov r1, sp\n"
                   "mov r2, lr\n"
                   "bl M4MonitorStop\n"
                   "pop {r4-r11, ip, lr}\n"
                   "bx lr\n");
}
