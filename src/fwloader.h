#ifndef FWLOADER_H_
#define FWLOADER_H_

#include <stdint.h>

#define M4_THUMB_BIT 0x1         /* M4 Thumb bit */

/* Boot header structure */
struct header {
#define PREAMBLE 0xaa55aa55
    uint32_t preamble;
    uint32_t plugin_size;
    uint32_t plugin_offset;
    uint32_t bl_size;
    uint32_t bl_src_offset;
    uint32_t bl_dest;
    uint32_t bl_entry;
#define PCIE_EDMA_DIS_MASK 0x00000001
#define RESET_HS_DIS_MASK 0x00010000
    uint32_t flags;
};

void prepare_fwloader();
void fwloader();

#endif
