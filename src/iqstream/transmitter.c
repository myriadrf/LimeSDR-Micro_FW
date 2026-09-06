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

const uint8_t dac_clock_divisor_disabled = 0; // when clock divisor disabled, 1 phytimer tick == 2 samples step

tx_lane_t tx_pipe[TX_MAX_PIPELINES_COUNT] __attribute__((section(".hif")));

void transmitter_init(void)
{
    vPhyTimerComparatorForce(11, ePhyTimerComparatorOut0);
    transmitter_lane_select_channel(0, 0);
    for (int i = 0; i < TX_MAX_PIPELINES_COUNT; ++i)
        init_host_dma_channel(&tx_pipe[i].host_dma);
}

static inline void validate_tcd(dma_tcd_t *tcd)
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
    host_dma_channel_t *const dma = &pipe->host_dma;
    if (!dma->enabled)
        return false;

    if (tcd_fifo_isempty(&dma->hif.tcd_fifo))
    {
        return false;
    }

    if (!pipe->vspa_dma)
    {
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
    }

    dma_tcd_t *next_tcd = tcd_fifo_front(&dma->hif.tcd_fifo);
    validate_tcd(next_tcd);

    if (next_tcd->flags & PKT_HAS_TIMESTAMP)
    {
        if ((next_tcd->flags & PKT_START) && (next_tcd->flags & PKT_END))
        {
            log_err("NotsupportedSTART/END" LOG_EOL);
            return false;
        }
        if (next_tcd->flags & PKT_START)
        {
            if (trigger_active | trigger_scheduled)
            {
                pipe->wait_trigger_change = true;
                pipe->expected_trigger = 0;
                dbg_info("Tx start wait trig:%i" LOG_EOL, pipe->expected_trigger);
                return false; // can't yet schedule next burst start
            }
        }
        if (next_tcd->flags & PKT_END)
        {
            if (trigger_scheduled)
            {
                pipe->wait_trigger_change = true;
                pipe->expected_trigger = 1;
                dbg_info("Tx stop wait trig:%i" LOG_EOL, pipe->expected_trigger);
                return false; // can't yet schedule next burst start
            }
        }
        // TODO: check if late
    }

    if (!push_tcd_to_vspa(pipe->vspa_dma, next_tcd))
    {
        return false;
    }

    pipe->host_dma.hif.tcd_fifo.done = pipe->vspa_dma->tcd_fifo.done;
    if (next_tcd->flags & PKT_HAS_TIMESTAMP)
    {
        if (next_tcd->flags & PKT_END && next_tcd->flags & PKT_START)
            log_err("BAD, start/stop in same batch" LOG_EOL);
        if (next_tcd->flags & PKT_END)
        {
            if (!trigger_active)
                log_err("TxTriggerShouldBeON" LOG_EOL);
            uint64_t ts = next_tcd->timestamp_msb;
            ts <<= 32;
            ts |= next_tcd->timestamp_lsb;
            const uint64_t off_phytime =
                stream_phytime_origin_rx + (((ts + next_tcd->size / 4) << pipe->oversample_pow2) >> dac_clock_divisor_disabled);
            vPhyTimerComparatorConfig(pipe->phytimer_id, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, off_phytime);
            dbg_info("tx_schedoff %08X" LOG_EOL, (uint32_t)off_phytime);
        }
        else if (next_tcd->flags & PKT_START)
        {
            if (trigger_active)
                log_err("TxTriggerShouldBeOFF" LOG_EOL);
            uint64_t ts = next_tcd->timestamp_msb;
            ts <<= 32;
            ts |= next_tcd->timestamp_lsb;
            const uint64_t on_phytime = stream_phytime_origin_rx + ((ts << pipe->oversample_pow2) >> dac_clock_divisor_disabled);
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

    if (pipe->host_dma.loop_mode)
    {
        dma_tcd_t *dest = tcd_fifo_back(&pipe->host_dma.hif.tcd_fifo);
        *dest = *next_tcd;
        tcd_fifo_push(&pipe->host_dma.hif.tcd_fifo);
    }
    tcd_fifo_pop(&pipe->host_dma.hif.tcd_fifo);

    return true;
}

static inline void tx_pipe_reset(tx_lane_t *pipe)
{
    pipe->wait_trigger_change = false;
    pipe->expected_trigger = 0;
}

static inline void tx_fill_up_vspa_tcds(tx_lane_t *pipe)
{
    int i = 0;
    for (; tx_schedule_next_host_tcd(pipe) && i < 10; ++i)
        ;
}

int transmitter_lane_enable(uint16_t lane, bool enabled)
{
    log_info("TX[%i]_lane_enable:%i" LOG_EOL, lane, enabled);
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

        // must have tx_dma_allowed enabled, to properly do dma fifo_ptr_rst
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut1);
        signal_to_vspa(HTV_SIGNAL_TXLANE0_PRIME); // prepare pipeline, RF transmission will be started by phytimer trigger
        while (vspa_signal_status() & HTV_SIGNAL_TXLANE0_PRIME) // wait for priming to complete, so trigger could be disabled
        {
        }
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut0);
        // prefill VSPA if TCD are already available
        tx_fill_up_vspa_tcds(&tx_pipe[lane]);
    }
    else
    {
        // must have tx_dma_allowed during abort, to properly do dma fifo_ptr_rst
        vPhyTimerComparatorForce(tx_pipe[lane].phytimer_id, ePhyTimerComparatorOut1);
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

void transmitter_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    if (flags & VTH_SIGNAL_TXLANE0_TCD_DONE)
    {
        // dbg_info("TCD_DONE" LOG_EOL);
        raise_irq = true;
        tx_pipe[0].host_dma.hif.tcd_fifo.done = tx_pipe[0].vspa_dma->tcd_fifo.done;
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_READ_DONE));
}

void transmitter_service(void)
{
    for (int lane = 0; lane < TX_MAX_PIPELINES_COUNT; ++lane)
    {
        host_dma_update_state(&tx_pipe[lane].host_dma);
        tx_fill_up_vspa_tcds(&tx_pipe[lane]);
    }
}