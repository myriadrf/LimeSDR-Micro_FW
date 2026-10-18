#include <stdint.h>

uint32_t g_la9310_current_platform_clk_hz;

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