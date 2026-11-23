#include "core_cm4.h"
#include "immap.h"
#include "log.h"

#include "vspa_regs.h"
#include "outbound_msi.h"
#include "iqstream_signals.h"
#include "io.h"
#include "vspa_debug.h"
#include "vspa_mem.h"
#include "profiling.h"
#include "phytimer.h"
#include "tx_tdd.h"

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

#define PHY_TIMER_COMP_CH5_TX_ALLOWED 11
#define PHY_TIMER_COMP_RFCTL_0 15 // on board RF switch
#define PHY_TIMER_COMP_RFCTL_4 19 // LMS7002M TXEN pin
#define PHY_TIMER_COMP_RFCTL_5 20 // PA enable

volatile m4_phytimer_t *vspa_timer_request;

// Defaults match the VSPA's transmitter_init(): PA follows the DAC, RF switch untouched until configured
volatile tx_tdd_config_t tx_tdd_config = {
    .pa_on_comparator_out = ePhyTimerComparatorOut1,
    .pa_off_comparator_out = ePhyTimerComparatorOut0,
    .rf_sw_on_comparator_out = ePhyTimerComparatorNoChange,
    .rf_sw_off_comparator_out = ePhyTimerComparatorNoChange,
};

static inline void tx_tdd_timers(uint32_t phytime, bool start)
{
    if (start)
    {
        vPhyTimerComparatorConfig(PHY_TIMER_COMP_CH5_TX_ALLOWED, 0, ePhyTimerComparatorOut1, phytime + tx_tdd_config.dac_allowed);
        if (tx_tdd_config.pa_on_comparator_out != ePhyTimerComparatorNoChange)
            vPhyTimerComparatorConfig(PHY_TIMER_COMP_RFCTL_5, 0, tx_tdd_config.pa_on_comparator_out, phytime + tx_tdd_config.pa_on);
        if (tx_tdd_config.rf_sw_on_comparator_out != ePhyTimerComparatorNoChange)
            vPhyTimerComparatorConfig(
                PHY_TIMER_COMP_RFCTL_0, 0, tx_tdd_config.rf_sw_on_comparator_out, phytime + tx_tdd_config.rf_sw_on);
    }
    else
    {
        vPhyTimerComparatorConfig(PHY_TIMER_COMP_CH5_TX_ALLOWED, 0, ePhyTimerComparatorOut0, phytime + tx_tdd_config.dac_allowed);
        if (tx_tdd_config.pa_off_comparator_out != ePhyTimerComparatorNoChange)
            vPhyTimerComparatorConfig(
                PHY_TIMER_COMP_RFCTL_5, 0, tx_tdd_config.pa_off_comparator_out, phytime + tx_tdd_config.pa_off);
        if (tx_tdd_config.rf_sw_off_comparator_out != ePhyTimerComparatorNoChange)
            vPhyTimerComparatorConfig(
                PHY_TIMER_COMP_RFCTL_0, 0, tx_tdd_config.rf_sw_off_comparator_out, phytime + tx_tdd_config.rf_sw_off);
    }
}

static inline void handle_vspa_phytime(void)
{
    switch (vspa_timer_request->cmd)
    {
    case M4_PHYTIME_CMD_IMMEDIATE:
        vPhyTimerComparatorForce(vspa_timer_request->timer_id, vspa_timer_request->trig_value);
        break;
    case M4_PHYTIME_CMD_SCHEDULE:
        vPhyTimerComparatorConfig(vspa_timer_request->timer_id, 0, vspa_timer_request->trig_value, vspa_timer_request->phytime);
        break;
    case M4_PHYTIME_CMD_TDD_START:
        tx_tdd_timers(vspa_timer_request->phytime, true);
        break;
    case M4_PHYTIME_CMD_TDD_END:
        tx_tdd_timers(vspa_timer_request->phytime, false);
        break;
    }
}

void la9310_vspa_isr(void)
{
    struct vspa_regs *const pVspaRegs = (struct vspa_regs *)VSPA_BASE_ADDR;
    uint32_t vspa_flags0 = IN_32(&pVspaRegs->vcpu_host_flags0);
    if (vspa_flags0 & (1 << 16))
    {
        handle_vspa_phytime();
        OUT_32(&pVspaRegs->vcpu_host_flags0, (1 << 16)); // time critical, release VSPA as soon as possible
        vspa_flags0 ^= (1 << 16);
    }

    if (vspa_flags0)
    {
        uint32_t events = 0;
        if (vspa_flags0 & VTH_SIGNAL_RX_TCD_DONE_MASK)
            events |= (1 << M4_MSI_RX_WRITE_DONE);
        if (vspa_flags0 & VTH_SIGNAL_TXLANE0_TCD_DONE)
            events |= (1 << M4_MSI_TX_READ_DONE);

        OUT_32(&pVspaRegs->vcpu_host_flags0, vspa_flags0); // clear flags
        if (events)
            la9310_msi_events_raise(events);
    }
    NVIC_ClearPendingIRQ(IRQ_VSPA);
#if ARM_ERRATUM_838869
    dsb();
#endif
}