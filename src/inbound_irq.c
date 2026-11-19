#include "core_cm4.h"
#include "immap.h"
#include "log.h"

#include "vspa_regs.h"
#include "outbound_msi.h"
#include "iqstream_signals.h"
#include "io.h"
#include "vspa_debug.h"

struct la9310_msg_unit {
    uint32_t msiir;
    uint32_t msir;
} __attribute__((packed));

static struct la9310_msg_unit *const pMsgUnit = (struct la9310_msg_unit *)MSG_UNIT_BASE_ADDR;

void inbound_msg1_isr(void)
{
    NVIC_DisableIRQ(IRQ_MSG1);

    uint32_t msir = IN_32(&pMsgUnit[0].msir); // clear on read
    log_isr("MSG1:%lx" LOG_EOL, msir);
    (void)msir;

    NVIC_ClearPendingIRQ(IRQ_MSG1);
    NVIC_EnableIRQ(IRQ_MSG1);
    dsb();
}

void inbound_msg2_isr(void)
{
    uint32_t msir = IN_32(&pMsgUnit[1].msir); // clear on read
    log_isr("MSG2:%lx" LOG_EOL, msir);
    (void)msir;
    NVIC_ClearPendingIRQ(IRQ_MSG2);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

void inbound_msg3_isr(void)
{
    VspaDebugProxyPoll(); // reads MSIR3 (clear on read)
    NVIC_ClearPendingIRQ(IRQ_MSG3);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

void la9310_axiq_isr(void)
{
    log_isr("AXIQ_IRQ" LOG_EOL);
    NVIC_ClearPendingIRQ(IRQ_AXIQ);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

#define VTH_SIGNAL_RX_TCD_DONE_MASK \
    (VTH_SIGNAL_RXLANE0_TCD_DONE | VTH_SIGNAL_RXLANE1_TCD_DONE | VTH_SIGNAL_RXLANE2_TCD_DONE | VTH_SIGNAL_RXLANE3_TCD_DONE)

void la9310_vspa_isr(void)
{
    log_isr("irq:VSPA" LOG_EOL);

    struct vspa_regs *const pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;
    const uint32_t vspa_flags = IN_32(&pVspaRegs->vcpu_host_flags0);
    if (vspa_flags)
    {
        OUT_32(&pVspaRegs->vcpu_host_flags0, vspa_flags); // clear flags

        uint32_t events = 0;
        if (vspa_flags & VTH_SIGNAL_RX_TCD_DONE_MASK)
            events |= (1 << M4_MSI_RX_WRITE_DONE);
        if (vspa_flags & VTH_SIGNAL_TXLANE0_TCD_DONE)
            events |= (1 << M4_MSI_TX_READ_DONE);
        if (events)
            la9310_msi_events_raise(events);
    }

    NVIC_ClearPendingIRQ(IRQ_VSPA);
#if ARM_ERRATUM_838869
    dsb();
#endif
}