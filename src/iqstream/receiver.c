#include "receiver.h"

#include "memory.h"
#include "log.h"

#include <phytimer.h>
#include "limesdr_micro/timer64.h"
#include "vspa_memorymap.h"

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

rx_lane_t rx_pipe[RX_MAX_PIPELINES_COUNT] __attribute__((section(".hif")));

static rx_config_t rx_settings[4];

void receiver_init(void)
{
    receiver_lane_set_channel(0, 2);
    receiver_lane_set_channel(1, 3);
}

int receiver_lane_enable(uint16_t lane, bool enabled)
{
    rx_lane_t *pipe = &rx_pipe[lane];
    log_info("RX[%i]_lane_enable:%i" LOG_EOL, lane, enabled);
    if (enabled)
    {
        // configure channel selection and oversampling
        uint32_t hiword = MBOX_OPC_RX_CHAN_SELECT << 24;
        uint32_t loword = lane & 0xFF;
        loword |= (pipe->channel & 0xFF) << 8;
        uint64_t value = ((uint64_t)hiword << 32) | loword;
        if (vspa_command_sync(value))
            return -1;

        hiword = MBOX_OPC_RX_CONFIGURE << 24;
        loword = lane & 0xFF;
        loword |= ((uint32_t)rx_settings[pipe->channel].oversample_pow2 & 0xFF) << 8;
        value = ((uint64_t)hiword << 32) | loword;
        if (vspa_command_sync(value))
            return -2;

        const uint32_t prime_flag = HTV_SIGNAL_RXLANE0_PRIME << lane;
        signal_to_vspa(prime_flag); // get vspa adc ready, it'll wait for phytimer trigger
        uint32_t cnt = 1000;
        while ((vspa_signal_status() & prime_flag) && --cnt)
        {
        }
    }
    else
    {
        const uint32_t abort_flag = HTV_SIGNAL_RXLANE0_ABORT << lane;
        signal_to_vspa(abort_flag);
        uint32_t cnt = 1000;
        while ((vspa_signal_status() & abort_flag) && --cnt)
        {
        }
    }
    return 0;
}

int receiver_lane_set_channel(uint16_t lane, uint16_t channel)
{
    if (lane >= RX_MAX_PIPELINES_COUNT || channel >= 4)
        return -1;
    rx_pipe[lane].phytimer_id = PHY_TIMER_COMP_CH1_RX_ALLOWED + channel;
    rx_pipe[lane].channel = channel;
    log_info("RxLane[%i] set channel %i" LOG_EOL, lane, channel);
    return 0;
}

int receiver_set_oversample(uint16_t channel, uint16_t oversample_pow2)
{
    if (channel >= 4 || oversample_pow2 > 2)
        return -1;
    rx_settings[channel].oversample_pow2 = oversample_pow2;
    log_info("RxChannel[%i] set oversample 2^%i" LOG_EOL, channel, oversample_pow2);
    return 0;
}

void receiver_handle_vspa_flags_irq(uint32_t flags)
{
    bool raise_irq = false;
    for (int lane = 0; lane < RX_MAX_PIPELINES_COUNT; ++lane)
    {
        if (flags & (VTH_SIGNAL_RXLANE0_TCD_DONE << lane))
        {
            raise_irq |= true;
        }
    }
    if (raise_irq)
        la9310_sirq_raise_events(&softirq, (1 << VSPA_DDR_WRITE_DONE));
}
