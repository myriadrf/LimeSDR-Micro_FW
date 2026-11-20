// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#ifndef LIME_M4_MONITOR_H
#define LIME_M4_MONITOR_H

#include <stdint.h>

/* Debug monitor: self-hosted debugging of this firmware over PCIe, without a
 * JTAG probe.
 *
 * The core's debug events (FPB breakpoints, BKPT instructions, DWT
 * watchpoints, single steps and halt requests) raise the DebugMonitor
 * exception (DEMCR.MON_EN). Its handler saves the interrupted context into
 * struct m4_monitor below, in TCML (host BAR1), and waits there until the
 * host resumes it. Faults (HardFault and the faults escalated to it) stop
 * the same way, so a crashed firmware can be inspected in place.
 *
 * The host reaches everything else through the VSPA debug proxy
 * (vspa_debug.h), which the monitor keeps serving while the firmware is
 * stopped: VSPA_DBG_OP_M4_READ/WRITE access any M4 address (the FPB and
 * DWT comparators sit on the private peripheral bus, which no PCIe BAR
 * reaches), VSPA_DBG_OP_MON_INFO returns the address of struct
 * m4_monitor, and VSPA_DBG_OP_MON_HALT stops the running firmware.
 *
 * Halting debug through JTAG takes precedence: while a probe has set
 * DHCSR.C_DEBUGEN, the core ignores MON_EN and halts on debug events.
 *
 * Priorities: DebugMonitor runs at M4_MON_PRIORITY (0, the highest
 * configurable priority), so breakpoints and steps work in thread code
 * and in every interrupt handler of priority 1 or lower, and a stopped
 * firmware stops all of those interrupts. Code at priority 0, code with
 * PRIMASK set, and the HardFault/NMI handlers cannot be stopped: a BKPT or
 * breakpoint there escalates to HardFault, which also stops in the monitor,
 * but cannot be single-stepped.
 */

#define M4_MON_MAGIC 0x4E4F4D34u /* "4MON" */
#define M4_MON_VERSION 1u
#define M4_MON_PRIORITY 0u

/* state */
#define M4_MON_RUNNING 0u
#define M4_MON_HALTED 1u

/* resume, written by the host while halted, cleared by the M4 when taken */
#define M4_MON_RESUME_NONE 0u
#define M4_MON_RESUME_CONTINUE 1u
#define M4_MON_RESUME_STEP 2u /* one instruction, interrupts masked */
#define M4_MON_RESUME_STEP_IRQ 3u /* one instruction, interrupts may enter */

/* struct m4_monitor register indices */
#define M4_MON_REG_SP 13u
#define M4_MON_REG_LR 14u
#define M4_MON_REG_PC 15u
#define M4_MON_REG_XPSR 16u
#define M4_MON_REG_COUNT 17u

/* Written by the M4, except resume and the writable registers.
 * Word offsets are part of the host protocol (m4mon.py). */
struct m4_monitor {
    uint32_t magic; /* M4_MON_MAGIC */
    uint32_t version; /* M4_MON_VERSION */
    uint32_t self; /* address of this struct */
    uint32_t size; /* sizeof(struct m4_monitor) */
    uint32_t state; /* M4_MON_RUNNING / M4_MON_HALTED, written last on a stop */
    uint32_t halt_count; /* incremented on every stop */
    uint32_t resume; /* M4_MON_RESUME_*, written by the host */
    uint32_t exception; /* exception number of the stop: 12 DebugMonitor, 3 HardFault */
    uint32_t dfsr; /* DFSR at the stop (HALTED, BKPT, DWTTRAP, VCATCH, EXTERNAL) */
    uint32_t hfsr; /* HFSR at the stop */
    uint32_t cfsr; /* CFSR at the stop */
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t dwt_matched; /* bit n: DWT comparator n matched (FUNCTIONn.MATCHED) */
    uint32_t exc_return; /* EXC_RETURN of the stop */
    uint32_t frame; /* address of the exception frame */
    /* r0-r12, sp, lr, pc, xpsr of the stopped code. Written back on resume,
     * except sp, which is fixed by the frame position. */
    uint32_t regs[M4_MON_REG_COUNT];
    uint32_t msp;
    uint32_t psp;
    uint32_t primask;
    uint32_t basepri;
    uint32_t faultmask;
    uint32_t control;
};

void M4MonitorInit(void);
uint32_t M4MonitorAddress(void);
/* Stop the running firmware (from the proxy's MSG3 interrupt): the stop
 * lands in the code that the interrupt preempted. */
void M4MonitorRequestHalt(void);

/* Exception entry for the vector table: DebugMonitor and the faults */
void M4MonitorEntry(void);

#endif
