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
    tcd_fifo_reset(&dma->hif.tcd_fifo);
    dma->hif.bytes_xferred = 0;
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
    dma->hif.error = 0;
    return reset_host_dma_channel(dma);
}

void host_dma_update_state(host_dma_channel_t *channel)
{
    volatile host_dma_hif_t *hif = &channel->hif;
    if (!hif->pending)
        return;

    if (hif->clear)
    {
        dma_log("%x DMAclear" LOG_EOL, hif);
        reset_host_dma_channel(channel);
        hif->clear = false;
    }
    if (channel->enabled && !hif->enable)
    {
        dma_log("%x DMAdisable, d:%i" LOG_EOL, hif, hif->tcd_fifo.done);
        channel->enabled = false;
    }
    else if (!channel->enabled && hif->enable)
    {
        channel->enabled = true;
        channel->loop_mode = hif->loop_mode;
        dma_log("%x DMAenable,loop:%i" LOG_EOL, hif, channel->loop_mode);
    }
    hif->pending = false;
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