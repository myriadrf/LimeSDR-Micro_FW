#include "transmitter.h"

#include "memory.h"
#include "log.h"

#include <phytimer.h>
#include "limesdr_micro/timer64.h"
#include "vspa_memorymap.h"
#include "host_dma_hif.h"

#include "core_cm4.h"
#include "immap.h"
#include "io.h"
#include "drivers/avi/la9310_avi_ds.h"
#include "iqstream_signals.h"
#include "iqstream.h"

#include "la9310_sirq.h"

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

tx_lane_t tx_pipe[TX_MAX_PIPELINES_COUNT] __attribute__((section(".hif")));

void transmitter_init(void)
{
    vPhyTimerComparatorForce(11, ePhyTimerComparatorOut0);
    transmitter_lane_select_channel(0, 0);
    for (int i = 0; i < TX_MAX_PIPELINES_COUNT; ++i)
        init_host_dma_channel(&tx_pipe[i].host_dma);
}

static void validate_tcd(dma_tcd_t *tcd)
{
    // xfer size should be multiple of DMA FIFO threashold
    const uint32_t dma_bytes_threshold = IQSTREAM_AFE_PAYLOAD_SIZE;
    uint32_t suffix = tcd->size & (dma_bytes_threshold - 1);
    if (suffix)
    {
        log_dbg("tcd_padding %i->", tcd->size);
        tcd->size -= suffix;
        tcd->size += dma_bytes_threshold;
        log_dbg("%i" LOG_EOL, tcd->size);
    }
}

static bool tx_schedule_next_host_tcd(tx_lane_t *pipe)
{
    host_dma_channel_t *dma = &pipe->host_dma;
    if (!dma->enabled)
        return false;

    if (queue_isempty(&dma->tcd_fifo))
        return false;

    if (!pipe->vspa_dma)
    {
        log_err("tx_vspa_dma_null" LOG_EOL);
        return false;
    }

    const uint32_t trigger_status = ulPhyTimerComparatorGetStatus(pipe->phytimer_id);
    const bool trigger_scheduled = trigger_status & PHY_TIMER_COMPARATOR_STATUS_ENABLED;
    const bool trigger_active = trigger_status & PHY_TIMER_COMPARATOR_STATUS_OUT_HIGH;

    if (pipe->wait_trigger_change)
    {
        if (pipe->expected_trigger != trigger_active)
            return false;

        pipe->wait_trigger_change = false;
        dbg_info("got %x" LOG_EOL, trigger_status);
    }

    dma_tcd_t *next_tcd = (dma_tcd_t *)(queue_front(&dma->tcd_fifo));
    validate_tcd(next_tcd);

    if (next_tcd->flags & PKT_HAS_TIMESTAMP)
    {
        next_tcd->flags |= PKT_IRQ;
        if ((next_tcd->flags & PKT_START) && (next_tcd->flags & PKT_END))
        {
            log_err("NotsupportedSTART/END" LOG_EOL);
            return false;
        }
        if (next_tcd->flags & PKT_START)
        {
            if (trigger_active | trigger_scheduled)
            {
                // if (!pipe->next_burst_pending)
                // dbg_info("TxDefer %8x" LOG_EOL, trigger_status);
                pipe->wait_trigger_change = true;
                pipe->expected_trigger = 0;
                dbg_info("startWT:%i" LOG_EOL, pipe->expected_trigger);

                // pipe->next_burst_pending = true;
                return false; // can't yet schedule next burst start
            }
            // else
            //     dbg_info("PKT_START %x" LOG_EOL, (uint32_t)next_tcd->timestamp);
        }
        if (next_tcd->flags & PKT_END)
        {
            if (trigger_scheduled)
            {
                pipe->wait_trigger_change = true;
                pipe->expected_trigger = 1;
                dbg_info("endWT:%i" LOG_EOL, pipe->expected_trigger);
                // if (!pipe->next_burst_pending)
                // log_info("[%8x]TxDeferEnd after start%x" LOG_EOL, ulPhyTimerComparatorRead(10), trigger_status);
                // pipe->next_burst_pending = true;
                return false; // can't yet schedule next burst start
            }
            // else
            //     dbg_info("PKT_END %x" LOG_EOL, (uint32_t)next_tcd->timestamp);
        }
        // TODO: check if late
    }

    if (!push_tcd_to_vspa(pipe->vspa_dma, next_tcd))
    {
        // log_info("tcd_vspa_push_fail" LOG_EOL);
        return false;
    }
    // log_info("tx_tcd_vspa_push %x, f:%x" LOG_EOL, next_tcd->la9310_mem_address, next_tcd->flags);
    // log_info("[%8x]tcdpush, f:%x" LOG_EOL, ulPhyTimerComparatorRead(10), next_tcd->flags);

    if (next_tcd->flags & PKT_HAS_TIMESTAMP)
    {
        if (next_tcd->flags & PKT_END && next_tcd->flags & PKT_START)
            log_err("BAD, start/stop in same batch" LOG_EOL);
        if (next_tcd->flags & PKT_END)
        {
            const uint64_t off_phytime =
                stream_phytime_origin_rx + ((next_tcd->timestamp + next_tcd->size / 4) << pipe->oversample_pow2);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, off_phytime);
            dbg_info("tx_schedoff %08X" LOG_EOL, (uint32_t)off_phytime);
        }
        else if (next_tcd->flags & PKT_START)
        {
            const uint64_t on_phytime = stream_phytime_origin_rx + (next_tcd->timestamp << pipe->oversample_pow2);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, on_phytime);
            dbg_info("tx_schedon %8x" LOG_EOL, (uint32_t)on_phytime);
        }
    }
    else
    {
        if (!trigger_active && !trigger_scheduled)
        {
            // no timestamp, signal tx_dma_allowed immediately
            vPhyTimerComparatorForce(pipe->phytimer_id, ePhyTimerComparatorOut1);
            dbg_info("tx_schednow" LOG_EOL);
        }
    }

    queue_pop(&dma->tcd_fifo);
    if (dma->loop_mode)
        queue_push(&dma->tcd_fifo, next_tcd);
    return true;
}

static inline void tx_pipe_reset(tx_lane_t *pipe)
{
    pipe->next_burst_pending = false;
    pipe->wait_trigger_change = false;
    pipe->expected_trigger = 0;
}

static inline void tx_fill_up_vspa_tcds(tx_lane_t *pipe)
{
    for (int i = 0; tx_schedule_next_host_tcd(pipe) && i < 10; ++i)
        ;
}

int transmitter_lane_enable(uint16_t lane, bool enabled)
{
    log_info("TX[%i]_lane_enable:%i, trig:%x" LOG_EOL, lane, enabled, ulPhyTimerComparatorGetStatus(11));
    if (enabled)
    {
        tx_pipe_reset(&tx_pipe[lane]);
        // refresh address in case VSPA firmware changed
        tx_pipe[lane].vspa_dma = vspa_memorymap_find(VSPA_MMAP_TXDMA_LANE0 << lane);
        if (!tx_pipe[lane].vspa_dma)
        {
            log_err("VSPA:lane DMA hif not found" LOG_EOL);
            return -1;
        }
        // vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut0); // make sure tx_dma_allowed is in known state at start

        // vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut1); // must have tx_dma_allowed during abort, to properly do dma fifo_ptr_rst
        // signal_to_vspa(HTV_SIGNAL_TXLANE0_ABORT);
        // while(vspa_signal_status() & HTV_SIGNAL_TXLANE0_ABORT)
        // {
        // }
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut0);

        signal_to_vspa(HTV_SIGNAL_TXLANE0_PRIME); // get vspa adc ready, it'll wait for phytimer trigger
        // timer will be triggered by DMA TCD

        // prefill VSPA if TCD are already available
        tx_fill_up_vspa_tcds(&tx_pipe[lane]);
    }
    else
    {
        tx_pipe_reset(&tx_pipe[lane]);
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id,
            ePhyTimerComparatorOut1); // must have tx_dma_allowed during abort, to properly do dma fifo_ptr_rst
        signal_to_vspa(HTV_SIGNAL_TXLANE0_ABORT);
        while (vspa_signal_status() & HTV_SIGNAL_TXLANE0_ABORT)
        {
        }
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut0);
        tx_pipe[lane].vspa_dma = NULL;
    }
    return 0;
}

int transmitter_lane_select_channel(uint16_t lane, uint16_t channel)
{
    if (lane >= TX_MAX_PIPELINES_COUNT || channel >= 1)
        return -1;

    tx_pipe[lane].phytimer_id = PHY_TIMER_COMP_CH5_TX_ALLOWED + channel;
    tx_pipe[lane].oversample_pow2 = 0;
    return 0;
}

// Insert host dma request into DMA table
static void tx_tcd_input(tx_lane_t *pipe)
{
    host_dma_channel_t *dma = &pipe->host_dma;

    if (!host_dma_accept_tcd_input(dma))
        return;

    tx_fill_up_vspa_tcds(pipe);
}

void transmitter_process_host_tcd_input(void)
{
    for (int i = 0; i < TX_MAX_PIPELINES_COUNT; ++i)
        tx_tcd_input(&tx_pipe[i]);
}

void transmitter_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    if (flags & VTH_SIGNAL_TXLANE0_TCD_DONE)
    {
        raise_irq = true;
        ++tx_pipe[0].host_dma.hif.tcd_complete_counter;
        // log_info("[%8x]TCD_DONE, t:%x" LOG_EOL, ulPhyTimerComparatorRead(10), ulPhyTimerComparatorGetStatus(11));
        // tx_fill_up_vspa_tcds(&tx_pipe[lane]);
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_READ_DONE));
}

void transmitter_service(void)
{
    for (int lane = 0; lane < TX_MAX_PIPELINES_COUNT; ++lane)
    {
        tx_fill_up_vspa_tcds(&tx_pipe[lane]);
        // const uint32_t trigger_status = ulPhyTimerComparatorGetStatus(pipe->phytimer_id);
        // const bool trigger_scheduled = trigger_status & PHY_TIMER_COMPARATOR_STATUS_ENABLED;
        // const bool trigger_active = trigger_status & PHY_TIMER_COMPARATOR_STATUS_OUT_HIGH;

        // if (tx_pipe[i].next_burst_pending && !(ulPhyTimerComparatorGetStatus(11)))
        // {
        //     tx_pipe[i].next_burst_pending = false;
        // }
    }
}