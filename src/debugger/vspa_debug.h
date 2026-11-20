// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#ifndef LIME_VSPA_DEBUG_H
#define LIME_VSPA_DEBUG_H

#include <stdint.h>

/* Host-driven proxy for the VSPA debug register block (VSPA_DBG), and for
 * the M4's own address space (the debug monitor, m4_monitor.h).
 *
 * The block lives at 0xE0046000 on the Cortex-M4 external PPB and is not
 * reachable over any PCIe BAR, so the host asks the M4 to access it. The
 * request sits in struct vspa_dbg_proxy in M4 RAM (TCML, host BAR1); at
 * startup the firmware publishes the struct's address in DCFG SCRATCHRW32.
 * The host signals a request with message unit 3 (MSIIR3, any bit), and
 * VspaDebugProxy() runs in the MSG3 interrupt, so the proxy works
 * independently of the firmware's command mailbox and of its tasks.
 *
 * SCRATCHRW32 is cleared by the reload bootloader, so a pointer never
 * outlives the firmware that wrote it. The host still checks magic,
 * version and self before writing anything.
 *
 * A request runs once: when its tag (cmd bits 31-24) differs from the tag
 * in status. While the debug monitor holds the firmware stopped, it serves
 * requests by polling (VspaDebugProxyPoll()), as the MSG3 interrupt cannot
 * preempt it.
 *
 * Ops 3-6 came after version 1 without changing the struct; firmware
 * without them answers VSPA_DBG_ERR_OP.
 *
 * Build options (src/debugger/CMakeLists.txt): VSPA_DEBUGGER provides ops 1-2,
 * M4_DEBUGGER ops 3-6 and the monitor. The proxy (DEBUG_PROXY) exists
 * when either is enabled; a disabled op answers VSPA_DBG_ERR_OP.
 */

#define VSPA_DBG_SCRATCH_PTR 31u /* ulScratchrw[] index: SCRATCHRW32 */
#define VSPA_DBG_MAGIC 0x47424456u /* "VDBG" */
#define VSPA_DBG_VERSION 1u
#define VSPA_DBG_MAX_WORDS 256u

#define VSPA_DBG_OP_READ 1u
#define VSPA_DBG_OP_WRITE 2u
/* words at an M4 address (offset = address): TCML, TCMU, CCSR or the
 * private peripheral bus; a bus error ends the request with
 * VSPA_DBG_ERR_BUS */
#define VSPA_DBG_OP_M4_READ 3u
#define VSPA_DBG_OP_M4_WRITE 4u
/* debug monitor: data[0] = address of struct m4_monitor, data[1] = its version */
#define VSPA_DBG_OP_MON_INFO 5u
/* debug monitor: stop the running firmware */
#define VSPA_DBG_OP_MON_HALT 6u
/* burst on one register, e.g. the auto-incrementing RAVID portal */
#define VSPA_DBG_FLAG_FIXED (1u << 8)

#define VSPA_DBG_ERR_OP 1u
#define VSPA_DBG_ERR_OFFSET 2u
#define VSPA_DBG_ERR_COUNT 3u
#define VSPA_DBG_ERR_BUS 4u

struct vspa_dbg_proxy {
    uint32_t magic; /* VSPA_DBG_MAGIC */
    uint32_t version; /* VSPA_DBG_VERSION */
    uint32_t self; /* address of this struct */
    uint32_t max_words; /* capacity of data[] */
    uint32_t cmd; /* bits[7:0] op, bit[8] fixed, bits[31:24] tag */
    uint32_t offset; /* byte offset in the 4 KB block (ops 3-4: M4 address), word aligned */
    uint32_t count; /* words, 1..max_words */
    uint32_t status; /* written last: bits[31:24] tag, bits[7:0] error */
    uint32_t data[VSPA_DBG_MAX_WORDS];
};

#ifdef DEBUG_PROXY
void VspaDebugProxyInit(volatile uint32_t *scratch_regs);
/* serve a pending request, if any: from the MSG3 interrupt, or polled */
void VspaDebugProxyPoll(void);
#else
/* no proxy: SCRATCHRW32 stays 0, so the host finds none */
static inline void VspaDebugProxyInit(volatile uint32_t *scratch_regs)
{
    (void)scratch_regs;
}
static inline void VspaDebugProxyPoll(void) {}
#endif

#endif
