#include "host_dma.h"

#include "log.h"

#include "la9310_sirq.h"

#define RX_CH_COUNT 4
#define BYTES_PER_SAMPLE 4

#include "phytimer.h"

#if 0
    #define dma_log(...) \
        { \
            log_info("[%8x]", ulPhyTimerComparatorRead(10)); \
            log_info(__VA_ARGS__); \
        }
#else
    #define dma_log(...)
#endif

static int reset_host_dma_channel(host_dma_channel_t *dma)
{
    queue_init(&dma->tcd_fifo, HOST_DMA_TABLE_TCD_COUNT, dma->tcd_table, sizeof(dma_tcd_t));
    dma->hif.tcd_complete_counter = 0;
    dma->hif.bytes_xferred = 0;
    dma->hif.input_tcd.flags = 0;
    dma->hif.input_tcd.size = 0;
    dma->hif.input_tcd.la9310_mem_address = 0;
    dma->hif.input_tcd.timestamp = 0;
    dma->hif.error = 0;
    return 0;
}

int init_host_dma_channel(host_dma_channel_t *dma)
{
    dma_log("DMA_HIF@%8x" LOG_EOL, dma);
    dma->enabled = false;
    dma->loop_mode = false;
    dma->hif.enable = false;
    dma->hif.clear = false;
    dma->hif.pending = false;
    dma->hif.tcd_pending = false;
    dma->hif.error = 0;
    return reset_host_dma_channel(dma);
}

bool host_dma_accept_tcd_input(host_dma_channel_t *channel)
{
    volatile host_dma_hif_t *hif = &channel->hif;
    bool dma_needs_update = false;

    if (hif->pending)
    {
        if (channel->enabled && !hif->enable)
        {
            dma_log("DMAdisable" LOG_EOL);
            channel->enabled = false;
        }
        else if (!channel->enabled && hif->enable)
        {
            channel->enabled = true;
            channel->loop_mode = hif->loop_mode;
            dma_log("DMAenable,loop:%i" LOG_EOL, channel->loop_mode);
            dma_needs_update = true;
        }
        if (hif->clear)
        {
            dma_log("DMAclear" LOG_EOL);
            reset_host_dma_channel(channel);
            hif->clear = false;
            hif->tcd_pending = false;
        }
        hif->pending = false;
    }

    // if (!hif->tcd_pending)
    //     return dma_needs_update;

    if (hif->input_tcd.size == 0)
    {
        hif->tcd_pending = false;
        hif->error = 1;
        return dma_needs_update;
    }

    if (queue_isfull(&channel->tcd_fifo))
    {
        return false;
    }

    dma_log("ackTCD 0x%08x sz:%i" LOG_EOL, hif->input_tcd.la9310_mem_address, hif->input_tcd.size);
    dma_tcd_t temp = hif->input_tcd;
    queue_push(&channel->tcd_fifo, &temp);
    hif->input_tcd.size = 0; // indicate that tcd was taken
    hif->input_tcd.la9310_mem_address = 0;
    hif->input_tcd.flags = 0;
    hif->input_tcd.timestamp = 0;
    hif->tcd_pending = false;
    dma_needs_update = true;
    return dma_needs_update;
}

void host_dma_enable(host_dma_channel_t *channel, bool loop)
{
    channel->enabled = true;
    channel->loop_mode = loop;
}

void host_dma_disable_and_clear(host_dma_channel_t *channel)
{
    channel->enabled = false;
    reset_host_dma_channel(channel);
}