#include "m4_memorymap.h"
#include "iqstream/receiver.h"
#include "iqstream/transmitter.h"

#include "config.h"
#include "la9310_host_if.h"

static const m4_memory_map_t features_map[] = {
    { M4_MMAP_COMMAND_HIF, (uint32_t) & ((struct la9310_hif *)(TCML_PHY_ADDR + LA9310_EP_HIF_OFFSET))->sw_cmd_desc },
    { M4_MMAP_NONE, 0 }
};

const void *GetFeaturesMap(void)
{
    return &features_map;
}