#include "host_if.h"

#include <stddef.h>

#include "io.h"
#include "log.h"
#include "tx_tdd.h"

_Static_assert(offsetof(struct la9310_legacy_hif, hif_ver) == 0x4, "legacy host interface layout");
_Static_assert(offsetof(struct la9310_legacy_hif, sw_cmd_desc) == 0x240, "legacy host interface layout");
_Static_assert(sizeof(struct la9310_legacy_hif) == 808, "legacy host interface layout");

// At TCML + LA9310_LEGACY_HIF_OFFSET (linker script), where software that predates the host interface table expects
// it. Uninitialized (NOLOAD) memory, host_if_init() clears it.
volatile struct la9310_legacy_hif legacy_hif __attribute__((section(".legacy_hif")));

// Pieces of the host interface that the host can discover, see la9310_host_if.h
static const struct la9310_hif_entry hif_table[] = {
    { LA9310_HIF_COMMAND, (uint32_t)&legacy_hif.sw_cmd_desc },
    { LA9310_HIF_MEMORY_LOG, (uint32_t)&memlog },
    { LA9310_HIF_TX_TDD_CONFIG, (uint32_t)&tx_tdd_config },
    { LA9310_HIF_NONE, 0 },
};
_Static_assert(sizeof(hif_table) / sizeof(hif_table[0]) <= LA9310_HIF_TABLE_MAX_ENTRIES, "host interface table is too long");

void host_if_init(uint32_t *scratch_registers)
{
    volatile uint32_t *words = (volatile uint32_t *)&legacy_hif;
    for (size_t i = 0; i < sizeof(legacy_hif) / sizeof(uint32_t); ++i)
        words[i] = 0;
    legacy_hif.sw_cmd_desc.status = LA9310_SW_CMD_STATUS_FREE;
    legacy_hif.hif_ver = LA9310_LEGACY_HIF_VERSION;

    // the interface has to be initialized before the host can find it
    dsb();
    OUT_32(&scratch_registers[LA9310_SCRATCH_LEGACY_HIF_SIZE], sizeof(legacy_hif));
    OUT_32(&scratch_registers[LA9310_SCRATCH_HIF_TABLE_ADDR], (uint32_t)hif_table);
    dsb();
}
