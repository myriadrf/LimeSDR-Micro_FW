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

#include "la9310_sirq.h"

#define TCD_PREFIL_LIMIT 32

#if 0
    #define dbg_info(...) log_info(__VA_ARGS__)
#else
    #define dbg_info(...)
#endif

extern struct la9310_sirq softirq;

rx_lane_t rx_pipe[RX_MAX_PIPELINES_COUNT] __attribute__((section(".hif")));

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
    host_dma_channel_t *dma = &pipe->host_dma;
    if (!dma->enabled)
        return false;

    if (!pipe->vspa_dma)
        return false;

    if (queue_isempty(&dma->tcd_fifo))
        return false;

    dma_tcd_t *next_tcd = (dma_tcd_t *)(queue_front(&dma->tcd_fifo));
    if (!push_tcd_to_vspa(pipe->vspa_dma, next_tcd))
        return false;
    // log_info("rx_tcd_vspa_push %x, sz:%i f:%x" LOG_EOL, next_tcd->la9310_mem_address, next_tcd->size, next_tcd->flags);

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
            const uint64_t on_phytime =
                stream_phytime_origin + ((next_tcd->timestamp << pipe->oversample_pow2) >> adc_clock_divisor_disabled);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, on_phytime);
            dbg_info("-RX-schedon %08X" LOG_EOL, (uint32_t)on_phytime);
        }
        else if (next_tcd->flags & PKT_END)
        {
            const uint64_t off_phytime =
                stream_phytime_origin +
                (((next_tcd->timestamp + next_tcd->size / 4) << pipe->oversample_pow2) >> adc_clock_divisor_disabled);
            dbg_info("-RX-schedoff %08X" LOG_EOL, (uint32_t)off_phytime);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, off_phytime);
        }
    }
    else
    {
        // if (next_tcd->flags & PKT_END)
        // {
        //     const uint32_t now = timer64_get_counter();
        //     const uint64_t off_timestamp = now + next_tcd->size/4;
        //     vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, off_timestamp);
        //     log_info("-RX-schedoff %08X" LOG_EOL, (uint32_t)next_tcd->timestamp);
        // }
        if (!trigger_active && !trigger_scheduled)
        {
            // const uint32_t now = timer64_get_counter();
            const uint32_t start_delay_samples =
                2048; // 16*1024;// 8*1024; // gives some time to schedule other channels, so they could start working from the 0 timestamp
            const uint64_t on_phytime =
                stream_phytime_origin + ((start_delay_samples << pipe->oversample_pow2) >> adc_clock_divisor_disabled);
            stream_phytime_origin_rx = on_phytime;
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, on_phytime);
            dbg_info("-RX-schedon %08X, orig: %8X" LOG_EOL, (uint32_t)on_phytime, (uint32_t)stream_phytime_origin);
        }
    }

    queue_pop(&dma->tcd_fifo);
    if (dma->loop_mode)
        queue_push(&dma->tcd_fifo, next_tcd);
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
            log_err("VSPA:lane DMA hif not found" LOG_EOL);
            return -1;
        }

        vPhyTimerComparatorForce(pipe->phytimer_id, ePhyTimerComparatorOut0); // set trigger to known state
        signal_to_vspa(HTV_SIGNAL_RXLANE0_PRIME); // get vspa adc ready, it'll wait for phytimer trigger

        // timer will be configured by DMA TCD
        rx_fill_up_vspa_tcds(&rx_pipe[lane]);
        // log_info("RxPrefil:%i" LOG_EOL, i);
    }
    else
    {
        signal_to_vspa(HTV_SIGNAL_RXLANE0_ABORT);
        while (vspa_signal_status() & HTV_SIGNAL_RXLANE0_ABORT)
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
    rx_pipe[lane].oversample_pow2 = 0;
    return 0;
}

// Insert host dma request into DMA table
static void rx_tcd_input(rx_lane_t *pipe)
{
    host_dma_channel_t *dma = &pipe->host_dma;

    if (!host_dma_accept_tcd_input(dma))
        return;
    rx_fill_up_vspa_tcds(pipe);
}

void receiver_process_host_tcd_input(void)
{
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        rx_tcd_input(&rx_pipe[lane]);
        rx_fill_up_vspa_tcds(&rx_pipe[lane]);
    }
}

void receiver_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (flags & (VTH_SIGNAL_RXLANE0_TCD_DONE << lane))
        {
            raise_irq |= true;
            ++rx_pipe[lane].host_dma.hif.tcd_complete_counter;
            rx_fill_up_vspa_tcds(&rx_pipe[lane]);
        }
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_WRITE_DONE));
}
