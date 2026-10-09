// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#ifndef LIME_VSPA_DEBUG_H
#define LIME_VSPA_DEBUG_H

#include <stdint.h>

/* Host-driven proxy for the VSPA debug register block (VSPA_DBG).
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
 */

#define VSPA_DBG_SCRATCH_PTR 31u /* ulScratchrw[] index: SCRATCHRW32 */
#define VSPA_DBG_MAGIC 0x47424456u /* "VDBG" */
#define VSPA_DBG_VERSION 1u
#define VSPA_DBG_MAX_WORDS 256u

#define VSPA_DBG_OP_READ 1u
#define VSPA_DBG_OP_WRITE 2u
/* burst on one register, e.g. the auto-incrementing RAVID portal */
#define VSPA_DBG_FLAG_FIXED (1u << 8)

#define VSPA_DBG_ERR_OP 1u
#define VSPA_DBG_ERR_OFFSET 2u
#define VSPA_DBG_ERR_COUNT 3u

struct vspa_dbg_proxy {
    uint32_t magic; /* VSPA_DBG_MAGIC */
    uint32_t version; /* VSPA_DBG_VERSION */
    uint32_t self; /* address of this struct */
    uint32_t max_words; /* capacity of data[] */
    uint32_t cmd; /* bits[7:0] op, bit[8] fixed, bits[31:24] tag */
    uint32_t offset; /* byte offset in the 4 KB block, word aligned */
    uint32_t count; /* words, 1..max_words */
    uint32_t status; /* written last: bits[31:24] tag, bits[7:0] error */
    uint32_t data[VSPA_DBG_MAX_WORDS];
};

void VspaDebugProxyInit(volatile uint32_t *scratch_regs);
void VspaDebugProxy(void);

#endif
