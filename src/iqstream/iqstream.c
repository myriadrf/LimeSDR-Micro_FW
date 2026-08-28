#include "iqstream.h"

#include "receiver.h"
#include "transmitter.h"

#include <stddef.h>

#include "drivers/avi/la9310_avi_ds.h"
#include "immap.h"
#include "io.h"
#include "log.h"

#include "iqstream_signals.h"

struct vspa_regs *vspa_csr = (struct vspa_regs *)VSPA_BASE_ADDR;

uint64_t stream_phytime_origin = 256; // phytime when stream was enabled, to have common reference for timestamping Rx/Tx streams
uint64_t stream_phytime_origin_rx = 256;

void iqstream_init(void)
{
    receiver_init();
    transmitter_init();
}

int iqstream_enable(uint32_t rx_mask, uint32_t tx_mask)
{
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (rx_mask & (1 << lane))
            receiver_lane_enable(lane, true);
    }
    for (int lane = 0; lane < TX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (tx_mask & (1 << lane))
            transmitter_lane_enable(lane, true);
    }
    OUT_32(&vspa_csr->vspa_irqen, (IN_32(&vspa_csr->vspa_irqen) | (1 << 4))); // VSPA_IRQ_EN
    return 0;
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

void iqstream_service(void)
{
    receiver_process_host_tcd_input();
    transmitter_process_host_tcd_input();
    transmitter_service();
}

void iqstream_handle_vspa_dma_irq(uint32_t dma_irq_stat)
{
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

bool push_tcd_to_vspa(volatile vspa_dma_hif_t *hif, const dma_tcd_t *tcd)
{
    if (vspa_signal_status() & hif->htv_tcd_pending_flag_mask)
        return false;
    // tcd->timestamp, timing is managed by M4, not relevant to VSPA

    hif->input_tcd.address = tcd->la9310_mem_address;
    hif->input_tcd.size = tcd->size;
    hif->input_tcd.flags = tcd->flags;
    signal_to_vspa(hif->htv_tcd_pending_flag_mask);
    return true;
}

// Once AXIQ problems start, they can result in interrupt storm
// void iqstream_handle_axiq_irq(void)
// {
//     uint32_t vspa_gpin0 = iord(GPI(0)); // rx axiq status
//     uint32_t vspa_gpin1 = iord(GPI(1)); // tx axiq status
//     // log_info("in0:%x in1:%x" LOG_EOL, vspa_gpin0, vspa_gpin1);

//     const uint32_t rx_overrun = vspa_gpin0 & 0xCCCC;
//     const uint32_t tx_underrun = vspa_gpin1 & (0x3 << 18);

//     if (rx_overrun)
//     {
//         uint32_t gpo4 = iord(GPO(4));
//         gpo4 |= 0x10101010;
//         // log_info("wr4:%x" LOG_EOL,gpo4);
//         iowr(GPO(4), gpo4);
//     }
//     if (tx_underrun)
//     {
//         uint32_t gpo7 = iord(GPO(7));
//         gpo7 |= 0x10;
//         log_info("wr7:%x" LOG_EOL, gpo7);
//         iowr(GPO(7), gpo7);
//         iowr(GPO(7), gpo7 & ~0x10);
//         signal_to_vspa(HTV_SIGNAL_TXLANE0_UNBRICK); // get vspa adc ready, it'll wait for phytimer trigger
//     }
// }

void iqstream_vspa_irq_handler(void)
{
    struct vspa_regs *pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;
    const uint32_t dma_irq_stat = IN_32(&pVspaRegs->dma_irq_stat);
    // log_info( "VSPA_");
    if (dma_irq_stat)
    {
        OUT_32(&pVspaRegs->dma_irq_stat, dma_irq_stat);
        // log_info( "_DMA, %x", dma_irq_stat);
        iqstream_handle_vspa_dma_irq(dma_irq_stat);
        // la9310_sirq_raise_events(&pLa9310Info->softirq, (1 << VSPA_DDR_WRITE_DONE) );
    }

    const uint32_t signal_flags = IN_32(&pVspaRegs->vcpu_host_flags0);
    if (signal_flags)
    {
        OUT_32(&pVspaRegs->vcpu_host_flags0, signal_flags);
        // log_info("_sig, %x", signal_flags);
        iqstream_handle_vspa_flags_irq(signal_flags);
    }
    // log_info(LOG_EOL);
}