#include "la9310_host_if.h"

#include "log.h"
#include <string.h>
#include "core_cm4.h"
#include "immap.h"
#include "config.h"

#include "la9310_irq.h"
#include "vspa_regs.h"
#include "outbound_msi.h"
#include "iqstream_signals.h"
#include "io.h"
#include "vspa_debug.h"

#define BITMASK(n) (1 << n)
#define LA9310_EVENT_ALL ((uint32_t)(((uint64_t)1 << LA9310_EVENT_COUNT) - 1))

// In TCMU, the host maps it through BAR2. NOLOAD, la9310_msi_events_init() initializes it.
volatile struct la9310_msi_events msi_events __attribute__((section(".hif"), aligned(8)));

void la9310_msi_events_init(uint32_t *scratch_regs)
{
    volatile struct la9310_msi_events *const ev = &msi_events;
    // nothing is signalled until the host enables it
    ev->mask = 0xFFFFFFFF;
    ev->event_count = LA9310_EVENT_COUNT;
    ev->msi_count = 0;
    for (int i = 0; i < LA9310_EVENT_MAX_COUNT; ++i)
        ev->counters[i] = 0;

    // The host driver sets up MSI when it probes, before firmware is loaded.
    // If host's MSI config changes, when firmware is already active, host updates these cached values
    const uint32_t msi_cap = IN_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_MSI_CAP_REG));
    const uint32_t msi_addr = IN_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_MSI_ADDR_REG));
    // The outbound window LA9310_EP_TOHOST_MSI_PHY_ADDR maps the host's 4K MSI page
    ev->msi_addr = (msi_cap & PCIE_MSI_CAP_ENABLE) ? (LA9310_EP_TOHOST_MSI_PHY_ADDR | (msi_addr & 0xFFF)) : 0;
    ev->msi_data = IN_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_MSI_DATA_REG_1)) & 0xFFFF;
    ev->msi_vector_count = 1u << ((msi_cap & PCIE_MSI_CAP_MME_MASK) >> PCIE_MSI_CAP_MME_SHIFT);
    log_info("MSI %s, %d vec" LOG_EOL, ev->msi_addr ? "on" : "off", ev->msi_vector_count);

    // the interface has to be initialized before the host can find it
    dsb();
    OUT_32(&scratch_regs[LA9310_SCRATCH_MSI_EVENTS_ADDR], (uint32_t)ev);
    dsb();
}

void la9310_msi_events_raise(uint32_t events)
{
    volatile struct la9310_msi_events *const ev = &msi_events;
    // Masked events are not counted either, so the host doesn't see them as pending when it unmasks them. The host
    // masks every event while it updates the MSI configuration, read it only after the mask.
    events &= ~ev->mask & LA9310_EVENT_ALL;
    if (!events)
        return;

    uint32_t *const msi_addr = (uint32_t *)ev->msi_addr;
    const uint32_t msi_data = ev->msi_data;
    const uint32_t vector_count = ev->msi_vector_count;
    uint32_t direct_vectors = 0;
    int use_mux_vector = 0;

    disable_irq();
    for (uint32_t i = 0; i < LA9310_EVENT_COUNT; ++i)
    {
        if (!(events & (1u << i)))
            continue;
        ++ev->counters[i];
        // events 1..N-1 have their own vectors, the rest go through the mux vector
        if (i != LA9310_EVENT_MUX_VECTOR && i < vector_count)
            direct_vectors |= (1u << i);
        else
            use_mux_vector = 1;
    }
    // counters must be visible before the host gets the MSI
    dsb();

    if (msi_addr)
    {
        // A vector is selected by modifying the low bits of the message data, the host allocates them aligned
        uint32_t msi_count = ev->msi_count;
        for (uint32_t v = 1; v < vector_count; ++v)
        {
            if (direct_vectors & (1u << v))
            {
                OUT_32(msi_addr, msi_data | v);
                ++msi_count;
            }
        }
        if (use_mux_vector)
        {
            OUT_32(msi_addr, msi_data | LA9310_EVENT_MUX_VECTOR);
            ++msi_count;
        }
        ev->msi_count = msi_count;
        log_isr("RaiseMsi ev=%x vec=%x mux=%d n=%d" LOG_EOL, events, direct_vectors, use_mux_vector, vector_count);
    }
    dsb();
    enable_irq();
}
