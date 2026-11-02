#include "la9310_avi_ds.h"
#include "immap.h"
#include "io.h"
#include "log.h"

#include "iqstream_signals.h"
#include "la9310_sirq.h"

// Streaming is controlled by the host directly on VSPA, M4 only relays VSPA signals as host interrupts.

extern struct la9310_sirq softirq;

#define VTH_SIGNAL_RX_TCD_DONE_MASK \
    (VTH_SIGNAL_RXLANE0_TCD_DONE | VTH_SIGNAL_RXLANE1_TCD_DONE | VTH_SIGNAL_RXLANE2_TCD_DONE | VTH_SIGNAL_RXLANE3_TCD_DONE)

inline static void iqstream_handle_error(void)
{
    // TODO: get errno code
    log_err("VSPA_ERROR" LOG_EOL);
}

static inline void iqstream_handle_vspa_flags_irq(uint32_t flags)
{
    if (flags & VTH_SIGNAL_ERROR)
        iqstream_handle_error();

    uint32_t events = 0;
    if (flags & VTH_SIGNAL_RX_TCD_DONE_MASK)
        events |= (1 << VSPA_DDR_WRITE_DONE);
    if (flags & VTH_SIGNAL_TXLANE0_TCD_DONE)
        events |= (1 << VSPA_DDR_READ_DONE);
    if (events)
        la9310_sirq_raise_events(&softirq, events);
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
