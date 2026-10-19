#include "la9310.h"

#include "io.h"
#include "immap.h"

#include <stdint.h>

uint32_t g_la9310_current_platform_clk_hz = PRE_BOOT_PLATFORM_CLK_HZ;

void vUDelay(uint32_t microseconds)
{
    /*
     * 245.76(CPU Frequency)/5(Cycle count consumed by below loop) = ~49
     * sub = 1 cycle, cmp = 1 cycle , bne = 3 cycle(if branch taken)
     */
    // TODO: scale based on g_la9310_current_platform_clk_hz, to delay in real timed
    asm("mov r1, #49    \n"
        "mul r0, r0, r1 \n"
        "loop:          \n"
        "sub r0, #1     \n"
        "cmp r0, #0     \n"
        "bne loop       \n"
        :
        :
        :);
}

bool la9310_clk_src_pcie(void)
{
    // check if the platform clock has already been switched.
    const uint32_t ulPbitCompletion = IN_32((uint32_t *)PBI_COMPLETIONR);
    const uint32_t ulRcwCompletion = IN_32((uint32_t *)RCW_COMPLETIONR);
    bool already_on_dcs_clk = (ulRcwCompletion & RCW_COMPLETION_DONE) | (ulPbitCompletion & PBI_COMPLETION_DONE);
    if (already_on_dcs_clk)
    {
        // The actual frequency is not known at this point.
        // Previous firmware run might have left any value.
        g_la9310_current_platform_clk_hz = LA9310_DCS_CLK_HZ;
    }
    return !already_on_dcs_clk;
}