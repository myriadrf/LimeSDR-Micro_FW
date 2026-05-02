#include "timer64.h"

#include "log.h"

#include <phytimer.h>
#include <stdbool.h>

#include "io.h"
#include "immap.h"

#include "drivers/avi/la9310_avi_ds.h"

static void printf64_t(uint64_t value)
{
    uint32_t *words = (uint32_t *)&value;
    for (int i = 0; i < 2; ++i)
        log_info("%08X", words[1 - i]);
}

static const uint8_t running_phytimer_id = 10;

static uint32_t counter_msb = 0;
static uint32_t counter_lsb = 0; // phytimer value

void timer64_reset(void)
{
    log_info("timer64_reset\n");

    counter_msb = 0;
    counter_lsb = 0;
    vPhyTimerReset();
    const uint32_t divisor = 1;
    vPhyTimerEnable(divisor);
    ulPhyTimerCapture(running_phytimer_id);
}

uint64_t timer64_get_counter(void)
{
    const uint32_t phytimer_counter_now = ulPhyTimerComparatorRead(running_phytimer_id);
    if (phytimer_counter_now < counter_lsb)
        ++counter_msb;
    counter_lsb = phytimer_counter_now;
    return (((uint64_t)counter_msb) << 32) | counter_lsb;
}
