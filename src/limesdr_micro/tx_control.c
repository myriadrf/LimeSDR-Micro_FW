#include "tx_control.h"
#include "log.h"

#include <phytimer.h>
#include <stdbool.h>

#include "timer64.h"

#include "io.h"
#include "immap.h"
#include "drivers/avi/la9310_avi_ds.h"
#include "queue.h"
#include "m4_commands.h"
#include "iqplayer_commands.h"

static struct vspa_regs *pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;

#define TX_QUEUE_COUNT 64
static tx_event_t tx_fifo_mem[TX_QUEUE_COUNT];
static queue_t tx_queue;

static const uint8_t timer_tx_dma_allowed = 11;
// static const uint8_t timer_rf_switch = 15;
static const uint8_t timer_pa_enable = 20;

static uint8_t tx_schedule_parity = 0; // ensure each scheduled start/stop comes in pairs

static const int event_time_margin = 150; // time it takes to configure comparators

#define TIME_UNKNOWN 0xFFFFFFFFFFFFFFFF

void tx_control_init(void)
{
    tx_schedule_parity = 0;
    queue_init(&tx_queue, TX_QUEUE_COUNT, tx_fifo_mem, sizeof(tx_fifo_mem[0]));
}

void tx_control_clear_fifo(void)
{
    tx_schedule_parity = 0;
    log_info("%s\r\n", __func__);
    queue_clear(&tx_queue);
}

int tx_schedule_start(int64_t time_point, uint32_t data_src_offset, uint32_t data_length)
{
    if (tx_schedule_parity & 0x1)
        return -1;

    if (queue_isfull(&tx_queue))
        return -2;

    ++tx_schedule_parity;

    tx_event_t event;
    event.time_point = time_point;
    event.data_src_offset = data_src_offset;
    event.data_length = data_length;
    event.flags = TX_CONTROL_START;

    const int64_t now = timer64_get_counter();
    if (event.time_point < now + event_time_margin)
        return -3; // event already too late

    queue_push(&tx_queue, &event);
    return 0;
}

int tx_schedule_stop(int64_t time_point, int32_t data_length)
{
    if ((tx_schedule_parity & 0x1) == 0)
        return -1; // no start was previously scheduled

    if (queue_isfull(&tx_queue))
        return -2;

    ++tx_schedule_parity;

    tx_event_t event;
    event.time_point = time_point;
    // event.data_src_offset = data_src_offset; // don't care about data location
    event.data_length = data_length;
    event.flags = TX_CONTROL_STOP;

    // const int64_t now = timer64_get_counter();
    // if (event.time_point < now + event_time_margin)
    //     return -3; // event already too late

    // even if 'stop' is late, it still must be done to ensure proper VSPA DMA state
    queue_push(&tx_queue, &event);

    return 0;
}

void tx_start_immediate(void)
{
    uint32_t hiword = (MBOX_OPC_TX_CONTROL << 24);
    hiword |= 1; // ddr enable
    uint32_t loword = 0;
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);

    vPhyTimerComparatorForce(timer_tx_dma_allowed, ePhyTimerComparatorOut1);
    // vPhyTimerComparatorForce(timer_rf_switch, payload->tx_rf_switch_control);
    vPhyTimerComparatorForce(timer_pa_enable, ePhyTimerComparatorOut1);
}

void tx_stop_immediate(void)
{
    uint32_t hiword = (MBOX_OPC_TX_CONTROL << 24);
    // hiword |= 1; // ddr enable
    uint32_t loword = 0;
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);

    vPhyTimerComparatorForce(timer_tx_dma_allowed, ePhyTimerComparatorOut0);
    // vPhyTimerComparatorForce(timer_rf_switch, payload->tx_rf_switch_control);
    vPhyTimerComparatorForce(timer_pa_enable, ePhyTimerComparatorOut0);
}

static void ConfigureTxOn(tx_event_t *evt)
{
    uint32_t hiword = (MBOX_OPC_TX_CONTROL << 24);
    hiword |= 1; // ddr enable
    uint32_t loword = evt->data_src_offset;
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);

    vPhyTimerComparatorConfig(
        timer_tx_dma_allowed, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, evt->time_point & 0xFFFFFFFF);
    vPhyTimerComparatorConfig(
        timer_pa_enable, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut1, (evt->time_point) & 0xFFFFFFFF);
}

static void ConfigureTxOff(tx_event_t *evt)
{
    uint32_t hiword = (MBOX_OPC_TX_CONTROL << 24);
    // hiword |= 1; // ddr enable
    uint32_t loword = evt->data_length;
    OUT_32(&pVspaRegs->host_out_1_msb, hiword);
    dmb();
    OUT_32(&pVspaRegs->host_out_1_lsb, loword);

    vPhyTimerComparatorConfig(
        timer_tx_dma_allowed, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, evt->time_point & 0xFFFFFFFF);
    vPhyTimerComparatorConfig(
        timer_pa_enable, PHY_TIMER_COMPARATOR_CLEAR_INT, ePhyTimerComparatorOut0, (evt->time_point) & 0xFFFFFFFF);
}

void process_tx_schedule(void)
{
    while (!queue_isempty(&tx_queue))
    {
        const uint32_t timer_status = ulPhyTimerComparatorGetStatus(timer_tx_dma_allowed);
        if (timer_status & PHY_TIMER_COMPARATOR_STATUS_ENABLED)
        {
            // trigger is scheduled, do nothing until that time
            return;
        }

        tx_event_t *event = queue_front(&tx_queue);
        int64_t now = timer64_get_counter();

        if (event->time_point + (uint64_t)(0xFFFFFFFF) < now)
            return; // event is more than one phytimer period away into the future

        if (now + event_time_margin < event->time_point)
        {
            if (event->flags & TX_CONTROL_START)
                ConfigureTxOn(event);
            else if (event->flags & TX_CONTROL_STOP)
                ConfigureTxOff(event);

            queue_pop(&tx_queue);
            return;
        }

        // event is late at this point
        if (event->flags & TX_CONTROL_START)
        {
            queue_pop(&tx_queue);
            if (!queue_isempty(&tx_queue))
                queue_pop(&tx_queue); // Tx not have been started, so remove pending stop as well
        }
        else if (event->flags & TX_CONTROL_STOP)
        {
            queue_pop(&tx_queue);
            tx_stop_immediate(); // even if late, still need to perform stop to ensure proper VSPA DMA state
        }
    }
}

int TxControlCommand(void *data)
{
    struct tx_control_payload *payload = (struct tx_control_payload *)data;

    if (!(payload->flags & TX_CONTROL_HAS_TIME))
    {
        if (payload->flags & TX_CONTROL_START)
        {
            tx_start_immediate();
            return 0;
        }
        else if (payload->flags & TX_CONTROL_STOP)
        {
            tx_stop_immediate();
            return 0;
        }
        else
        {
            log_err("TxControlCommand error\r\n");
            return -1;
        }
    }

    if (payload->flags & TX_CONTROL_START)
        return tx_schedule_start(payload->timepoint, payload->data_src_offset, payload->data_length);
    else if (payload->flags & TX_CONTROL_STOP)
        return tx_schedule_stop(payload->timepoint, payload->data_length);
    else
        return -1;
}