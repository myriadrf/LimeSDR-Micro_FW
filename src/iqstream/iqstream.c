#include "iqstream.h"

#include "receiver.h"
#include "transmitter.h"

#include <stddef.h>

#include "la9310_avi_ds.h"
#include "immap.h"
#include "io.h"
#include "log.h"
#include <phytimer.h>

#include "iqstream_signals.h"

struct vspa_regs *vspa_csr = (struct vspa_regs *)VSPA_BASE_ADDR;

uint64_t stream_phytime_origin = 0; // phytime when stream was enabled, to have common reference for timestamping Rx/Tx streams
uint64_t stream_phytime_origin_rx = 256;

void iqstream_init(void)
{
    receiver_init();
    transmitter_init();
}

int iqstream_enable(uint32_t rx_mask, uint32_t tx_mask)
{
    int ret = 0;
    OUT_32(&vspa_csr->vspa_irqen, IN_32(&vspa_csr->vspa_irqen)
        | (1 << 4) // irqen_dma_cmp
        | (1 << 2) // irqen_flags0
    ); // VSPA_IRQ_EN

    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (rx_mask & (1 << lane))
            ret = receiver_lane_enable(lane, true);
        if (ret)
            return ret;
    }
    for (int lane = 0; lane < TX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (tx_mask & (1 << lane))
            transmitter_lane_enable(lane, true);
    }
    if (rx_mask | tx_mask)
        stream_phytime_origin = ulPhyTimerComparatorRead(10);
    return ret;
}

int iqstream_disable(uint32_t rx_mask, uint32_t tx_mask)
{
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (rx_mask & (1 << lane))
            receiver_lane_enable(lane, false);
    }
    for (int lane = 0; lane < TX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (tx_mask & (1 << lane))
            transmitter_lane_enable(lane, false);
    }
    return 0;
}

inline static void iqstream_handle_error(void)
{
    // TODO: get errno code
    log_err("VSPA_ERROR" LOG_EOL);
}

void iqstream_handle_vspa_flags_irq(uint32_t flags)
{
    if (flags & VTH_SIGNAL_ERROR)
        iqstream_handle_error();

    receiver_handle_vspa_flags_irq(flags);
    transmitter_handle_vspa_flags_irq(flags);
}

void iqstream_vspa_irq_handler(void)
{
    struct vspa_regs *const pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;
    const uint32_t signal_flags = IN_32(&pVspaRegs->vcpu_host_flags0);
    if (signal_flags)
    {
        OUT_32(&pVspaRegs->vcpu_host_flags0, signal_flags);
        iqstream_handle_vspa_flags_irq(signal_flags);
    }
}

int vspa_command_sync(uint64_t vspa_msg64)
{
    const int mbox_index = 0;
    if (IN_32(&vspa_csr->host_mbox_status) & (1 << mbox_index))
        return -1;

    uint32_t loword = vspa_msg64 & 0xFFFFFFFFu;
    uint32_t hiword = (vspa_msg64 >> 32);
    log_info("VSPA_CMD, %08x_%08x" LOG_EOL, hiword, loword);
    OUT_32(&vspa_csr->host_out_1_msb, hiword);
    dmb();
    OUT_32(&vspa_csr->host_out_1_lsb, loword);

    // wait for response
    if (IN_32(&vspa_csr->vspa_status) & VSPA_MBOX0_STATUS)
    {
        uint32_t msb = IN_32(&vspa_csr->host_in_0_msb);
        uint32_t lsb = IN_32(&vspa_csr->host_in_0_lsb);
        log_info("VSPA_CMD_RET, %08x_%08x" LOG_EOL, msb, lsb);
        OUT_32(&vspa_csr->vspa_status, VSPA_MBOX0_STATUS);
    }
    return 0;
}