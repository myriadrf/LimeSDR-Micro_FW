#include "scheduled_commands.h"
#include "log.h"

#include <phytimer.h>
#include <stdbool.h>

#include "io.h"
#include "immap.h"

#include "iqplayer_commands.h"
#include "timer64.h"

#include "drivers/avi/la9310_avi_ds.h"

#include "m4_commands.h"

#define MAX_QUEUE_SIZE 16 // must be power of 2
#define QUEUE_SIZE_MASK (MAX_QUEUE_SIZE - 1)

#define LOG_CMDS(x)

static struct vspa_regs *pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;

static void printf64_t(uint64_t value)
{
    uint32_t *words = (uint32_t *)&value;
    for (int i = 0; i < 2; ++i)
        log_info("%08X", words[1 - i]);
}

typedef struct TimedSwitch {
    uint64_t timepoint;
    uint32_t data[16];
} timed_switch_t;

enum QueueId {
    eQUEUE_DAC_EN = 0,
    eQUEUE_RO0_EN,
    eQUEUE_RO1_EN,
    eQUEUE_RX0_EN,
    eQUEUE_RX1_EN,
    eQUEUE_TX_BAND,
    eQUEUE_PA_ENABLE,
    eQUEUE_TX_WND,
    eQUEUE_COMMON,
    eQUEUE_COUNT,
};

typedef struct SwitchesFIFO {
    timed_switch_t items[MAX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
} switches_fifo_t;

static const uint8_t queue_timer_ids[] = { 11, 1, 2, 3, 4, 15, 20, 11, 10 };
static const char *switch_names[] = { "DAC_IQ", "RO0", "RO1", "RX0", "RX1", "TXRX1", "PA_EN", "TX_WND", "COMMON" };
static switches_fifo_t command_fifos[eQUEUE_COUNT];

uint16_t fifo_size(const switches_fifo_t *fifo)
{
    return fifo->tail - fifo->head;
}
bool fifo_isfull(const switches_fifo_t *fifo)
{
    return fifo_size(fifo) == QUEUE_SIZE_MASK;
}

void fifo_reset(switches_fifo_t *fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
}

bool fifo_push(switches_fifo_t *fifo, const timed_switch_t *event)
{
    if (fifo_isfull(fifo))
        return false;

    fifo->items[fifo->tail & QUEUE_SIZE_MASK] = *event;
    ++fifo->tail;
    return true;
}

bool fifo_pop(switches_fifo_t *fifo)
{
    if (fifo->tail == fifo->head)
        return false;
    ++fifo->head;
    return true;
}

timed_switch_t *fifo_front(switches_fifo_t *fifo)
{
    return &fifo->items[fifo->head & QUEUE_SIZE_MASK];
}

void scheduled_commands_clear(void)
{
    log_info("scheduled_commands_clear\n");
    for (uint32_t q = 0; q < eQUEUE_COUNT; ++q)
    {
        vPhyTimerComparatorDisable(queue_timer_ids[q]);
        vPhyTimerComparatorClearCIF(queue_timer_ids[q]);
        fifo_reset(&command_fifos[q]);
    }
}

uint32_t scheduled_commands_enqueue(uint64_t timepoint, uint32_t cmd, uint32_t *data, uint32_t len)
{
    uint8_t queue_id = cmd;
    if (cmd > LIME_M4_TIMED_CMD_COUNT)
    {
        queue_id = eQUEUE_COMMON;
    }

    if (fifo_isfull(&command_fifos[queue_id]))
        return 1;

    timed_switch_t event;
    event.timepoint = timepoint;
    for (int i = 0; i < len; ++i)
        event.data[i] = data[i];

    fifo_push(&command_fifos[queue_id], &event);
    LOG_CMDS(log_info("["); printf64_t(timer64_get_counter()); log_info("] Host pushed %s v:", switch_names[queue_id]);
             // printf64_t(value);
             log_info(" time:");
             printf64_t(timepoint);
             log_info("\r\n"));
    return 0;
}

static void TxControl(uint64_t vspa_msg64)
{
    uint32_t loword = vspa_msg64 & 0xFFFFFFFFu;
    uint32_t hiword = (vspa_msg64 >> 32);
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);
}

static void RxControl(uint64_t vspa_msg64)
{
    uint32_t loword = vspa_msg64 & 0xFFFFFFFFu;
    uint32_t hiword = (vspa_msg64 >> 32);
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);
}

static void TxBurstLength(uint32_t bytes)
{
    uint32_t loword = bytes;
    uint32_t hiword = MBOX_OPC_TX_BURST_LENGTH << 24;
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);
}

static void ConfigTrigger(uint32_t timerId, int64_t counter, uint32_t value)
{
    const int64_t now = timer64_get_counter();
    if (counter - now < 512)
    {
        // no time to configure comparator, set value dirrectly
        log_err("[%X]T%i f%i, d:%i\r\n", (uint32_t)now, timerId, value, (int32_t)(counter - now));
        vPhyTimerComparatorForce(timerId, value);
    }
    else
    {
        LOG_CMDS(log_info(">[%X]T%i f%i, d:%i\r\n", (uint32_t)now, timerId, value, (int32_t)(counter - now)));
        vPhyTimerComparatorConfig(timerId, PHY_TIMER_COMPARATOR_CLEAR_INT, value, counter & 0xFFFFFFFF);
    }
}

static void ProcessDAC_event(const timed_switch_t *event)
{
    const struct tx_dac_allowed_payload *payload = (struct tx_dac_allowed_payload *)event->data;
    const bool enable = (payload->vspa_cmd >> 32) & IQPLAYER_BIT_START;
    if (enable)
    {
        // tx_dma_allowed disable handled by VSPA
        ConfigTrigger(queue_timer_ids[eQUEUE_DAC_EN], event->timepoint, ePhyTimerComparatorOut1);
        // vPhyTimerComparatorConfig( ,
        //     PHY_TIMER_COMPARATOR_CLEAR_INT,
        //     enable ? ePhyTimerComparatorOut1 : ePhyTimerComparatorOut0,
        //     event->timepoint & 0xFFFFFFFF );
    }
    else
        ConfigTrigger(queue_timer_ids[eQUEUE_DAC_EN], event->timepoint, ePhyTimerComparatorOut0);

    TxControl(payload->vspa_cmd);
    LOG_CMDS(
        log_info("TxControl 0x%08X_%08X\r\n", (uint32_t)(payload->vspa_cmd >> 32), (uint32_t)(payload->vspa_cmd & 0xFFFFFFFFu)));
}

static void ProcessADC_event(const timed_switch_t *event)
{
    const struct tx_dac_allowed_payload *payload = (struct tx_dac_allowed_payload *)event->data;
    const bool enable = (payload->vspa_cmd >> 32) & IQPLAYER_BIT_START;
    const uint32_t channelIndex = (payload->vspa_cmd >> 52) & 0x3;

    ConfigTrigger(queue_timer_ids[eQUEUE_RO0_EN] + channelIndex,
        event->timepoint,
        enable ? ePhyTimerComparatorOut1 : ePhyTimerComparatorOut0);
    // vPhyTimerComparatorConfig( queue_timer_ids[eQUEUE_RO0_EN] + channelIndex,
    //     PHY_TIMER_COMPARATOR_CLEAR_INT,
    //     enable ? ePhyTimerComparatorOut1 : ePhyTimerComparatorOut0,
    //     event->timepoint & 0xFFFFFFFF );

    RxControl(payload->vspa_cmd);
    // const uint32_t burstlen = payload->vspa_cmd & 0xFFFFFFFF;
    LOG_CMDS(
        log_info("RxControl 0x%08X_%08X\r\n", (uint32_t)(payload->vspa_cmd >> 32), (uint32_t)(payload->vspa_cmd & 0xFFFFFFFFu)));
    LOG_CMDS(log_info("\r\n"));
}

static void ProcessTxBand_event(const timed_switch_t *event)
{
    const struct tx_band_switch_payload *payload = (struct tx_band_switch_payload *)event->data;
    const uint8_t rf_switch_value = payload->tx_rf_switch_control;
    ConfigTrigger(
        queue_timer_ids[eQUEUE_TX_BAND], event->timepoint, rf_switch_value ? ePhyTimerComparatorOut1 : ePhyTimerComparatorOut0);
    // vPhyTimerComparatorConfig( queue_timer_ids[eQUEUE_TX_BAND],
    //     PHY_TIMER_COMPARATOR_CLEAR_INT,
    //     rf_switch_value ? ePhyTimerComparatorOut1 : ePhyTimerComparatorOut0,
    //     event->timepoint & 0xFFFFFFFF);
}

static void ProcessTimer_event(const timed_switch_t *event, uint8_t queue_id)
{
    ConfigTrigger(queue_timer_ids[queue_id], event->timepoint, event->data[0]);
    // vPhyTimerComparatorConfig( queue_timer_ids[queue_id],
    //     PHY_TIMER_COMPARATOR_CLEAR_INT,
    //     event->data,
    //     event->timepoint & 0xFFFFFFFF);
}

static void ProcessTxWindow_event(const timed_switch_t *event)
{
    const struct tx_window_payload *payload = (struct tx_window_payload *)event->data;
    const bool enable = (payload->vspa_cmd >> 32) & IQPLAYER_BIT_START;

    const int64_t now = timer64_get_counter();
    if (event->timepoint - now < 150)
    {
        // no time to configure comparator, set value dirrectly
        log_err("[%X]TxWnd d:%i\r\n", (uint32_t)now, (int32_t)(event->timepoint - now));
        if (enable)
        {
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_DAC_EN], ePhyTimerComparatorOut1);
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_TX_BAND], payload->tx_rf_switch_control);
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_PA_ENABLE], ePhyTimerComparatorOut1);
        }
        else
        {
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_DAC_EN], ePhyTimerComparatorOut0);
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_TX_BAND], payload->tx_rf_switch_control);
            vPhyTimerComparatorForce(queue_timer_ids[eQUEUE_PA_ENABLE], ePhyTimerComparatorOut0);
        }
    }
    else
    {
        int64_t c0 = timer64_get_counter();
        LOG_CMDS(log_err(">[%X]TxWnd d:%i\r\n", (uint32_t)now, (int32_t)(event->timepoint - now)));
        if (enable)
        {
            vPhyTimerComparatorConfig(queue_timer_ids[eQUEUE_DAC_EN],
                PHY_TIMER_COMPARATOR_CLEAR_INT,
                ePhyTimerComparatorOut1,
                event->timepoint & 0xFFFFFFFF);
            vPhyTimerComparatorConfig(queue_timer_ids[eQUEUE_PA_ENABLE],
                PHY_TIMER_COMPARATOR_CLEAR_INT,
                ePhyTimerComparatorOut1,
                (event->timepoint + payload->pa_switch_offset) & 0xFFFFFFFF);
        }
        else
        {
            vPhyTimerComparatorConfig(queue_timer_ids[eQUEUE_DAC_EN],
                PHY_TIMER_COMPARATOR_CLEAR_INT,
                ePhyTimerComparatorOut0,
                event->timepoint & 0xFFFFFFFF);
            vPhyTimerComparatorConfig(queue_timer_ids[eQUEUE_PA_ENABLE],
                PHY_TIMER_COMPARATOR_CLEAR_INT,
                ePhyTimerComparatorOut0,
                (event->timepoint + payload->pa_switch_offset) & 0xFFFFFFFF);
        }
        vPhyTimerComparatorConfig(queue_timer_ids[eQUEUE_TX_BAND],
            PHY_TIMER_COMPARATOR_CLEAR_INT,
            payload->tx_rf_switch_control,
            (event->timepoint + payload->rf_switch_offset) & 0xFFFFFFFF);
        int64_t c1 = timer64_get_counter();
        log_info("switch t:%i\r\n", (int32_t)(c1 - c0));
    }

    TxControl(payload->vspa_cmd);
    LOG_CMDS(
        log_info("TxControl 0x%08X_%08X\r\n", (uint32_t)(payload->vspa_cmd >> 32), (uint32_t)(payload->vspa_cmd & 0xFFFFFFFFu)));
}

void scheduled_commands_update(void)
{
    for (uint32_t q = 0; q < eQUEUE_COUNT; ++q)
    {
        const uint8_t timer_id = queue_timer_ids[q];
        const uint32_t timer_status = ulPhyTimerComparatorGetStatus(timer_id);
        if (timer_status & PHY_TIMER_COMPARATOR_STATUS_ENABLED)
        {
            continue;
        }
        else if ((timer_status & PHY_TIMER_COMPARATOR_STATUS_INT)) // event has been triggered
        {
            // log_info("[");
            // printf64_t(timer64_get_counter());
            // log_info("]/%s\r\n", switch_names[q]);

            vPhyTimerComparatorClearCIF(timer_id);
        }

        if (fifo_size(&command_fifos[q]) == 0)
            continue;

        timed_switch_t *event = fifo_front(&command_fifos[q]);

        const uint64_t now = timer64_get_counter();
        // if ( event->timepoint < now)
        // {
        //     // event already late
        //     log_err("[");
        //     printf64_t(timer64_get_counter());
        //     log_err("] Event late [%s] ", switch_names[q]);
        //     printf64_t(event->timepoint);
        //     log_err("\r\n");
        //     fifo_pop(&command_fifos[q]);
        //     continue;
        // }

        // const uint64_t diff = event->timepoint - now;
        if (event->timepoint + (uint64_t)(0xFFFFFFFF) < now)
        {
            continue; // event is more than one phytimer period away into the future
        }

        switch (q)
        {
        case eQUEUE_DAC_EN:
            ProcessDAC_event(event);
            break;
        case eQUEUE_RO0_EN:
        case eQUEUE_RO1_EN:
        case eQUEUE_RX0_EN:
        case eQUEUE_RX1_EN:
            ProcessADC_event(event);
            break;
        case eQUEUE_TX_BAND:
            //ProcessTxBand_event(event);
            break;
        case LIME_M4_TX_WINDOW:
            //ProcessTxWindow_event(event);
            break;
        default:
            ProcessTimer_event(event, q);
        }

        LOG_CMDS(log_info("["); printf64_t(timer64_get_counter());
                 log_info("] scheduled %s v:%08X_%08X ", switch_names[q], (uint32_t)(event->data[0] >> 32), event->data);
                 printf64_t(event->timepoint);
                 log_info("\r\n"));

        fifo_pop(&command_fifos[q]);
    }
}
