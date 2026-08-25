#ifndef LIME_M4_HOST_DMA_H
#define LIME_M4_HOST_DMA_H

#define HOST_DMA_TABLE_TCD_COUNT 16

#include "host_dma_hif.h"

#include "queue.h"

typedef struct HostDMAChannel {
    volatile host_dma_hif_t hif;
    dma_tcd_t tcd_table[HOST_DMA_TABLE_TCD_COUNT];
    queue_t tcd_fifo;
    bool enabled;
    bool loop_mode;
} host_dma_channel_t;

void host_dma_msg_handler(uint32_t channel_mask);

static inline bool dma_tcd_isvalid(dma_tcd_t *tcd)
{
    return tcd->size > 0;
}

dma_tcd_t tcd_split(dma_tcd_t *src, uint32_t size);

void signal_host_tcd_write_complete(void);
bool host_dma_accept_tcd_input(host_dma_channel_t *channel);
int init_host_dma_channel(host_dma_channel_t *dma);

void host_dma_enable(host_dma_channel_t *channel, bool loop);
void host_dma_disable_and_clear(host_dma_channel_t *channel);

#endif // LIME_M4_HOST_DMA_H