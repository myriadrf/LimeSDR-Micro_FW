// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Deutsches Forschungszentrum für Künstliche Intelligenz GmbH (DFKI)

#include "vspa_debug.h"
#include "io.h"

#define VSPA_DBG_BASE 0xE0046000u
#define VSPA_DBG_SPAN 0x1000u

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

static uint32_t Execute(void)
{
    const uint32_t op = proxy.cmd & 0xFFu;
    const uint32_t fixed = proxy.cmd & VSPA_DBG_FLAG_FIXED;
    const uint32_t count = proxy.count;
    const uint32_t offset = proxy.offset;

    if (op != VSPA_DBG_OP_READ && op != VSPA_DBG_OP_WRITE)
        return VSPA_DBG_ERR_OP;
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

void VspaDebugProxy(void)
{
    const uint32_t tag = proxy.cmd & 0xFF000000u;
    const uint32_t error = Execute();
    dsb();
    proxy.status = tag | error;
}
