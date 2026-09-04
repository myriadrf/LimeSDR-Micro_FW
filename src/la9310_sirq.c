
#include "la9310_host_if.h"

#include "log.h"
#include <string.h>
#include "core_cm4.h"
#include "immap.h"
#include "config.h"

#include "la9310_sirq.h"
#include "iqstream/iqstream.h"
#include "io.h"

#define LA9310_SCRATCH_SIRQ_STATUS_REG 5
#define LA9310_SCRATCH_SIRQ_COUNT_REG 6
#define LA9310_SCRATCH_SIRQ_ENABLE_REG 7
#define LA9310_SCRATCH_SIRQ_CLEAR_REG 8

static struct la9310_sirq *g_sirq = NULL;

void la9310_sirq_initialize(struct la9310_sirq *sirq, uint32_t *scratch_regs)
{
    g_sirq = sirq;
    const uint32_t __IO *pMsiAddrReg = (uint32_t *)(PCIE_BASE_ADDR + PCIE_MSI_ADDR_REG);
    const uint32_t __IO *pMsiDataAddr = (uint32_t *)(PCIE_BASE_ADDR + PCIE_MSI_DATA_REG_1);

    // According to PCI bus standard multiple MSIs are allocated consecutively
    uint32_t uMSIAddrVal = (IN_32(pMsiAddrReg) & 0xFFF);

    for (int i = 0; i < 1; ++i)
    {
        sirq->host_msi_addr = (uint32_t *)(LA9310_EP_TOHOST_MSI_PHY_ADDR | uMSIAddrVal);
        sirq->host_msi_data = (IN_32(pMsiDataAddr) + i);
        log_isr("MSI addr 0x%x, data 0x%x\n\r", sirq->host_msi_addr, sirq->host_msi_data);
    }

    sirq->counter_addr = &scratch_regs[LA9310_SCRATCH_SIRQ_COUNT_REG];
    sirq->status = &scratch_regs[LA9310_SCRATCH_SIRQ_STATUS_REG];
    sirq->enable = &scratch_regs[LA9310_SCRATCH_SIRQ_ENABLE_REG];
    sirq->host_clear = &scratch_regs[LA9310_SCRATCH_SIRQ_CLEAR_REG];
    sirq->counter = 0;
    OUT_32(sirq->counter_addr, sirq->counter);
    OUT_32(sirq->status, 0);
    OUT_32(sirq->enable, 0);
    OUT_32(sirq->host_clear, 0);
    dsb();

    NVIC_SetPriority(IRQ_MSG1, MSG1_IRQ_PRIORITY);
    // Initialize Message unit IRQ for Host clear notification
    NVIC_EnableIRQ(IRQ_MSG1);
}

static void la9310_sirq_raise_to_host(struct la9310_sirq *sirq)
{
    OUT_32(sirq->counter_addr, ++sirq->counter);
    OUT_32(sirq->host_msi_addr, sirq->host_msi_data);
    log_isr("RaiseMsi addr=%p, data=%d\n", sirq->host_msi_addr, sirq->host_msi_data);
    dsb();
}

void la9310_sirq_clear_events(struct la9310_sirq *sirq)
{
    disable_irq();
    uint32_t host_clear = IN_32(sirq->host_clear);
    OUT_32(sirq->host_clear, 0);
    uint32_t status = IN_32(sirq->status);
    OUT_32(sirq->status, status & ~host_clear);
    dsb();
    log_isr("IRQ_CLR: clr:%04X st:%04X" LOG_EOL, host_clear, status);
    // uint32_t pending_evt = IN_32(sirq->status);
    enable_irq();

    // if( pending_evt )
    //     la9310_sirq_raise_to_host(sirq);
    dsb();
}

void la9310_msg1_irq_handler(void)
{
    NVIC_DisableIRQ(IRQ_MSG1);
    struct la9310_sirq *sirq = g_sirq;

    struct la9310_msg_unit *pMsgUnit = (struct la9310_msg_unit *)MSG_UNIT_BASE_ADDR;
    uint32_t msir = IN_32(&pMsgUnit->msir);
    log_isr("MSG1 IRQ: %8X x%8x" LOG_EOL, msir);

    // IRQ MUX
    if ((msir & BITMASK(IRQ_FLAGS_CHANGED)))
        la9310_sirq_clear_events(sirq);

    // if ((msir & BITMASK(HOST_COMMAND_POSTED)))
    //     HostSentCommand();

    NVIC_ClearPendingIRQ(IRQ_MSG1);
    NVIC_EnableIRQ(IRQ_MSG1);
    dsb();
}

void la9310_msg2_irq_handler(void)
{
    log_isr("MSG2 IRQ" LOG_EOL);
    NVIC_ClearPendingIRQ(IRQ_MSG2);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

void la9310_msg3_irq_handler(void)
{
    log_isr("MSG3 IRQ" LOG_EOL);
    NVIC_ClearPendingIRQ(IRQ_MSG3);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

void la9310_sirq_raise_events(struct la9310_sirq *sirq, uint32_t events)
{
    dmb();
    // Host has told us not to bother it with this event! It has more important work to do.
    // if( !( IN_32(sirq->enable) & evt_mask ) )
    //     return;

    uint32_t status = IN_32(sirq->status);
    dmb();
    // if (status & events) // already signalled, host not processed yet
    // {
    //     log_isr("IRQ skip %x" LOG_EOL, status);
    //     return;
    // }

    disable_irq();

    OUT_32(sirq->status, status | events);
    dsb();

    la9310_sirq_raise_to_host(sirq);
    enable_irq();
    dsb();
}

void la9310_axiq_irq_handler(void)
{
    log_isr("AXIQ_IRQ" LOG_EOL);

    // iqstream_handle_axiq_irq();

    NVIC_ClearPendingIRQ(IRQ_AXIQ);
#if ARM_ERRATUM_838869
    dsb();
#endif
}

void la9310_vspa_irq_handler(void)
{
    log_isr("irq:VSPA" LOG_EOL);

    iqstream_vspa_irq_handler();

    NVIC_ClearPendingIRQ(IRQ_VSPA);
#if ARM_ERRATUM_838869
    dsb();
#endif
}