#include "receiver.h"

#include "memory.h"
#include "log.h"

#include <phytimer.h>
#include "limesdr_micro/timer64.h"
#include "vspa_memorymap.h"

#include "core_cm4.h"
#include "immap.h"
#include "io.h"
#include "drivers/avi/la9310_avi_ds.h"
#include "iqstream_signals.h"
#include "iqstream.h"
#include "iqplayer_commands.h"

#include "la9310_sirq.h"

#define TCD_PREFIL_LIMIT 16

#if 0
    #define dbg_info(...) \
        { \
            log_info("[%8x]", ulPhyTimerComparatorRead(10)); \
            log_info(__VA_ARGS__); \
        }

#else
    #define dbg_info(...)
#endif

extern struct la9310_sirq softirq;

rx_lane_t rx_pipe[RX_MAX_PIPELINES_COUNT] __attribute__((section(".hif")));

static rx_config_t rx_settings[4];

const uint8_t adc_clock_divisor_disabled = 0; // when clock divisor disabled, 1 phytimer == 2 samples step

void receiver_init(void)
{
    for (int i = 0; i < RX_MAX_PIPELINES_COUNT; ++i)
        init_host_dma_channel(&rx_pipe[i].host_dma);
    receiver_lane_set_channel(0, 2);
    receiver_lane_set_channel(1, 3);
}

static bool rx_schedule_next_host_tcd(rx_lane_t *pipe)
{
    if (!pipe->host_dma.enabled)
        return false;

    if (!pipe->vspa_dma)
        return false;

    host_dma_hif_t *hdma = &pipe->host_dma.hif;
    if (tcd_fifo_isempty(&hdma->tcd_fifo))
        return false;

    dma_tcd_t *next_tcd = tcd_fifo_front(&hdma->tcd_fifo);
    vspa_dma_hif_t *vdma = pipe->vspa_dma;
    if (!push_tcd_to_vspa(vdma, next_tcd))
        return false;

    const uint32_t trigger_status = ulPhyTimerComparatorGetStatus(pipe->phytimer_id);
    const bool trigger_scheduled = trigger_status & PHY_TIMER_COMPARATOR_STATUS_ENABLED;
    const bool trigger_active = trigger_status & PHY_TIMER_COMPARATOR_STATUS_OUT_HIGH;

    if (next_tcd->flags & PKT_HAS_TIMESTAMP)
    {
        if (next_tcd->flags & PKT_START)
        {
            if (trigger_scheduled)
                return true; // Burst end is pending, do nothing until it completes
            else if (trigger_active)
            {
                log_err("RxDMAAllowed,should be off" LOG_EOL);
                return true;
            }
        }

        // TODO: check if not late
        if (next_tcd->flags & PKT_START)
        {
            uint64_t ts = next_tcd->timestamp_msb;
            ts <<= 32;
            ts |= next_tcd->timestamp_lsb;

            const uint64_t on_phytime =
                stream_phytime_origin + ((ts << rx_settings[pipe->channel].oversample_pow2) >> adc_clock_divisor_disabled);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, on_phytime);
            dbg_info("-RX-schedon %08X" LOG_EOL, (uint32_t)on_phytime);
        }
        else if (next_tcd->flags & PKT_END)
        {
            uint64_t ts = next_tcd->timestamp_msb;
            ts <<= 32;
            ts |= next_tcd->timestamp_lsb;
            const uint64_t off_phytime =
                stream_phytime_origin +
                (((ts + next_tcd->size / 4) << rx_settings[pipe->channel].oversample_pow2) >> adc_clock_divisor_disabled);
            dbg_info("-RX-schedoff %08X" LOG_EOL, (uint32_t)off_phytime);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, off_phytime);
        }
    }
    else
    {
        if (!trigger_active && !trigger_scheduled)
        {
            const uint32_t start_delay_samples =
                8 * 2048; // gives some time to schedule other channels, so they could start working from the 0 timestamp
            const uint64_t on_phytime =
                stream_phytime_origin +
                ((start_delay_samples << rx_settings[pipe->channel].oversample_pow2) >> adc_clock_divisor_disabled);
            stream_phytime_origin_rx = on_phytime;
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, on_phytime);
            dbg_info("-RX-schedon %08X, orig: %8X" LOG_EOL, (uint32_t)on_phytime, (uint32_t)now);
        }
    }

    tcd_fifo_pop(&hdma->tcd_fifo);
    if (pipe->host_dma.loop_mode)
    {
        dma_tcd_t *dest = tcd_fifo_back(&hdma->tcd_fifo);
        *dest = *next_tcd;
        tcd_fifo_push(&hdma->tcd_fifo);
    }
    return true;
}

static inline void rx_fill_up_vspa_tcds(rx_lane_t *pipe)
{
    int i = 0;
    for (; rx_schedule_next_host_tcd(pipe) && i < TCD_PREFIL_LIMIT; ++i)
        ;
}

int receiver_lane_enable(uint16_t lane, bool enabled)
{
    rx_lane_t *pipe = &rx_pipe[lane];
    log_info("RX[%i]_lane_enable:%i" LOG_EOL, lane, enabled);
    if (enabled)
    {
        // refresh DMA interface address in case VSPA firmware has changed
        pipe->vspa_dma = vspa_memorymap_find(VSPA_MMAP_RXDMA_LANE0 + lane);
        if (!pipe->vspa_dma)
        {
            log_err("VSPA:lane[%i] DMA hif not found" LOG_EOL, lane);
            return -1;
        }

        // configure channel selection and oversampling
        uint32_t hiword = MBOX_OPC_RX_CHAN_SELECT << 24;
        uint32_t loword = lane & 0xFF;
        loword |= (pipe->channel & 0xFF) << 8;
        uint64_t value = ((uint64_t)hiword << 32) | loword;
        if (vspa_command_sync(value))
            return -1;

        hiword = MBOX_OPC_RX_CONFIGURE << 24;
        loword = lane & 0xFF;
        loword |= ((uint32_t)rx_settings[pipe->channel].oversample_pow2 & 0xFF) << 8;
        value = ((uint64_t)hiword << 32) | loword;
        if (vspa_command_sync(value))
            return -2;

        vPhyTimerComparatorForce(pipe->phytimer_id, ePhyTimerComparatorOut0); // set trigger to known state 0
        const uint32_t prime_flag = HTV_SIGNAL_RXLANE0_PRIME << lane;
        signal_to_vspa(prime_flag); // get vspa adc ready, it'll wait for phytimer trigger
        while (vspa_signal_status() & prime_flag)
        {
        }

        // timer will be configured by DMA TCD
        rx_fill_up_vspa_tcds(&rx_pipe[lane]);
    }
    else
    {
        vPhyTimerComparatorForce(pipe->phytimer_id, ePhyTimerComparatorOut1); // set trigger to 1 for proper AXIQ FIFO reset
        const uint32_t abort_flag = HTV_SIGNAL_RXLANE0_ABORT << lane;
        signal_to_vspa(abort_flag);
        while (vspa_signal_status() & abort_flag)
        {
        }
        vPhyTimerComparatorForce(pipe->phytimer_id, ePhyTimerComparatorOut0);
        pipe->vspa_dma = NULL;
    }
    return 0;
}

int receiver_lane_set_channel(uint16_t lane, uint16_t channel)
{
    if (lane >= RX_MAX_PIPELINES_COUNT || channel >= 4)
        return -1;
    rx_pipe[lane].phytimer_id = PHY_TIMER_COMP_CH1_RX_ALLOWED + channel;
    rx_pipe[lane].channel = channel;
    log_info("RxLane[%i] set channel %i" LOG_EOL, lane, channel);
    return 0;
}

int receiver_set_oversample(uint16_t channel, uint16_t oversample_pow2)
{
    if (channel >= 4 || oversample_pow2 > 2)
        return -1;
    rx_settings[channel].oversample_pow2 = oversample_pow2;
    log_info("RxChannel[%i] set oversample 2^%i" LOG_EOL, channel, oversample_pow2);
    return 0;
}

void receiver_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (flags & (VTH_SIGNAL_RXLANE0_TCD_DONE << lane))
        {
            raise_irq |= true;
            rx_pipe[lane].host_dma.hif.tcd_fifo.done = rx_pipe[lane].vspa_dma->tcd_fifo.done;
        }
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_WRITE_DONE));
}

void receiver_service(void)
{
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        host_dma_update_state(&rx_pipe[lane].host_dma);
        rx_fill_up_vspa_tcds(&rx_pipe[lane]);
    }
}