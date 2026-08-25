#include "m4_memorymap.h"
#include "iqstream/receiver.h"
#include "iqstream/transmitter.h"
#include "la9310_info.h"

#include "config.h"
#include "la9310_host_if.h"

extern rx_lane_t rx_pipe[];
extern tx_lane_t tx_pipe[];
extern struct la9310_info g_la9310_info;

static const m4_memory_map_t features_map[] = {
    { M4_MMAP_COMMAND_HIF, (uint32_t) & ((struct la9310_hif *)(TCML_PHY_ADDR + LA9310_EP_HIF_OFFSET))->sw_cmd_desc },
    { M4_MMAP_IQPLAYER_RXPIPE0, (uint32_t)&rx_pipe[0].host_dma.hif },
    // { M4_MMAP_IQPLAYER_RXPIPE1, (uint32_t)&rx_pipe[1].host_dma.hif },
    { M4_MMAP_IQPLAYER_TXPIPE0, (uint32_t)&tx_pipe[0].host_dma.hif },
    { M4_MMAP_NONE, 0 }
};

const void *GetFeaturesMap(void)
{
    return &features_map;
}