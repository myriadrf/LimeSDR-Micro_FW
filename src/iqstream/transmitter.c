#include "transmitter.h"

#include "memory.h"
#include "log.h"

#include <phytimer.h>
#include "limesdr_micro/timer64.h"
#include "vspa_memorymap.h"
#include "host_dma_hif.h"

#include "core_cm4.h"
#include "immap.h"
#include "io.h"
#include "drivers/avi/la9310_avi_ds.h"
#include "iqstream_signals.h"
#include "iqstream.h"
#include "iqplayer_commands.h"

#include "la9310_sirq.h"

#if 0
    #define dbg_info(...) \
        { \
            log_info("[%8x]", ulPhyTimerComparatorRead(10)); \
            log_info(__VA_ARGS__); \
        }

#else
    #define dbg_info(...)
#endif

extern struct la9310_sirq softirq;
static tx_config_t tx_settings;

void transmitter_init(void)
{
    transmitter_lane_select_channel(0, 0);
}

int transmitter_lane_enable(uint16_t lane, bool enabled)
{
    log_info("TX[%i]_lane_enable:%i" LOG_EOL, lane, enabled);
    if (enabled)
    {
        // configure channel selection and oversampling
        uint32_t hiword = 0; //MBOX_OPC_TX_CHAN_SELECT << 24;
        uint32_t loword = lane & 0xFF;
        uint64_t value = ((uint64_t)hiword << 32) | loword;
        // if (vspa_command_sync(value))
        //     return -1;

        hiword = MBOX_OPC_TX_CONFIGURE << 24;
        loword = lane & 0xFF;
        loword |= ((uint32_t)tx_settings.oversample_pow2 & 0xFF) << 8;
        value = ((uint64_t)hiword << 32) | loword;
        if (vspa_command_sync(value))
            return -2;

        signal_to_vspa(HTV_SIGNAL_TXLANE0_PRIME); // prepare pipeline, RF transmission will be started by phytimer trigger
        uint32_t cnt = 1000;
        while ((vspa_signal_status() & HTV_SIGNAL_TXLANE0_PRIME) && --cnt)
        {
        }
    }
    else
    {
        signal_to_vspa(HTV_SIGNAL_TXLANE0_ABORT);
        uint32_t cnt = 1000;
        while ((vspa_signal_status() & HTV_SIGNAL_TXLANE0_ABORT) && --cnt)
        {
        }
    }
    return 0;
}

int transmitter_lane_select_channel(uint16_t lane, uint16_t channel)
{
    if (lane >= TX_MAX_PIPELINES_COUNT || channel >= 1)
        return -1;

    return 0;
}

int transmitter_lane_set_oversample(uint16_t lane, uint16_t oversample_pow2)
{
    if (lane >= TX_MAX_PIPELINES_COUNT || oversample_pow2 > 2)
        return -1;
    tx_settings.oversample_pow2 = oversample_pow2;
    log_info("TxLane[%i] set oversample 2^%i" LOG_EOL, lane, oversample_pow2);
    return 0;
}

void transmitter_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    if (flags & VTH_SIGNAL_TXLANE0_TCD_DONE)
    {
        raise_irq = true;
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_READ_DONE));
}
