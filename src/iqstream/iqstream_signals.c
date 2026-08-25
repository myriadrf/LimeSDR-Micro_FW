#include "iqstream_signals.h"

#include "drivers/avi/la9310_avi_ds.h"
#include "log.h"

#include "immap.h"
#include "io.h"

#include <phytimer.h>
#include <string.h>

#define DMA_CHANNEL_COUNT 16

struct vspa_regs *vspa_csr = (struct vspa_regs *)VSPA_BASE_ADDR;

// void vspa_axiq_irq_handler(void)
// {
//     const uint8_t ci = pipe->channelIndex;
//     const enum axiq_fifo_e fifo_index = Rx_Antenna2fifo_index[ci];
//     const uint8_t field_shift = axiq_sr_shift(fifo_index);
//     // Check AXIQ rx fifo is not full or overrun
//     uint32_t status = axiq_fifo_rx_sr(AXIQ_BANK_0, fifo_index, AXIQ_SR_FIELD_ERROVER | AXIQ_SR_FIELD_ERRUNDER);
//     if (status == 0)
//         return;

//     status >>= field_shift;
//     if (status & AXIQ_SR_FIELD_ERROVER) {
//         ++player_state.data_flow.rx_issues[ci].overrun;
//         TRACE_COUNTER(CNT_RX0_OVR + pipe->channelIndex, player_state.data_flow.rx_issues[ci].overrun);
//     }
//     if (status & AXIQ_SR_FIELD_ERRUNDER) {
//         ++player_state.data_flow.rx_issues[ci].underrun;
//         TRACE_COUNTER(CNT_RX0_UDR + pipe->channelIndex, player_state.data_flow.rx_issues[ci].underrun);
//     }
//     EnqueueProxyUpdate(PROXY_UPDATE_FLOW);
//     axiq_fifo_rx_cr(AXIQ_BANK_0, fifo_index, AXIQ_CR_CLRERR, AXIQ_CR_CLRERR);
//     axiq_fifo_rx_cr(AXIQ_BANK_0, fifo_index, AXIQ_CR_CLRERR, 0);
// }

void signal_to_vspa(uint32_t flags)
{
    OUT_32(&vspa_csr->host_vcpu_flags0, flags);
}

uint32_t vspa_signal_status()
{
    return IN_32(&vspa_csr->host_vcpu_flags0);
}
