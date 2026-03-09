#ifndef LA9310_ROM_BOOTHEADER_H
#define LA9310_ROM_BOOTHEADER_H

#include <stdint.h>

struct la9310_boot_header {
    uint32_t preamble;
    uint32_t plugin_size;
    uint32_t plugin_offset;
    uint32_t bl_size;
    uint32_t bl_src_offset;
    uint32_t bl_dest;
    uint32_t bl_entry;

#define BOOTROM_SKIP_EDMA_COPY (1 << 0)
#define BOOTROM_SKIP_BOOT_PLUGIN (1 << 16)
    uint32_t flags;
} __attribute__((packed));

#endif // LA9310_ROM_BOOTHEADER_H
