#include "la9310_boot_header.h"
#include "config.h"
#include "core_cm4.h"
#include "io.h"

#include "../src/drivers/misc/la9310_gpio.h"

#define PREAMBLE 0xaa55aa55
#define M4_THUMB_BIT 0x1

#define LED1_PIN 17
#define LED2_PIN 18

// TODO: Code of Loading from host memory already exists in ROM. Should find it's address and reuse it.
// Having this procedure run from TCML risks overwriting itself if the image is large enough.
__attribute__((noreturn)) static void fw_boot_from_host_memory(void)
{
    disable_irq();
    volatile struct la9310_boot_header *boot_header = (struct la9310_boot_header *)TCMU_PHY_ADDR;

    iGpioInit(LED1_PIN, output, false);
    uint32_t cnt = 0;
    while (boot_header->preamble != PREAMBLE)
    {
        iGpioSetData(LED1_PIN, ++cnt & 0x10000);
        dmb();
    }

    uint32_t *dst = (uint32_t *)boot_header->bl_dest;
    const volatile uint32_t *src = (uint32_t *)(boot_header->bl_src_offset + PCIE_PHY_ADDR);
    const uint32_t size = boot_header->bl_size;

    for (uint32_t copied = 0; copied < size; copied += sizeof(uint32_t))
        *dst++ = *src++;

    iGpioSetData(LED1_PIN, 1);

    void (*jump_to_fw_entry)(void) = (void *)(boot_header->bl_entry | M4_THUMB_BIT);
    jump_to_fw_entry();

    // Control should never reach here
    while (1)
    {
    };
}

extern uint32_t __bootloader_stack; // provided by linker script

__attribute__((noreturn)) void bootloader_reset_handler(void)
{
    disable_irq();
    // Initialize Core Registers
    asm("mov    r0, #0x0\n"
        "mov    r1, r0\n"
        "mov    r2, r0\n"
        "mov    r3, r0\n"
        "mov    r4, r0\n"
        "mov    r5, r0\n"
        "mov    r6, r0\n"
        "mov    r7, r0\n"
        :
        :
        :);

    // Initialize stack pointer
    asm("ldr r0, = __bootloader_stack\n"
        "msr msp, r0\n"
        :
        :
        :);

    // Set Priviledged Mode
    asm("mrs    r0, control\n"
        "bic    r0, 0x1\n"
        "msr    control, r0\n"
        :
        :
        :);

    SYST->STCSR = (SYST->STCSR & ~(0x1)); // disable systick
    NVIC->ICER[0] = 0xFFFFFFFF; // disable external interrupts
    NVIC->ICPR[0] = 0xFFFFFFFF; // clear pending interrupt flags

    // switch back to ROM ISR vectors
    SCB->VTOR = 0;
    SCB->ICSR = (1 << 30) | (1 << 27) | (1 << 25); // Clear pending systick, nmi, pendsv interrupts
    dmb();
    isb();

    // clear DCFG scratch registers, they were used to indicate FW startup status
    // and indicate for software that M4 has active firmware loaded
    for (uint32_t addr = 0x41E00200; addr <= 0x41E00220; addr += 4)
        OUT_32(addr, 0);

    volatile struct la9310_boot_header *boot_header = (struct la9310_boot_header *)TCMU_PHY_ADDR;
    boot_header->preamble = 0;

    fw_boot_from_host_memory();

    while (1)
    {
    };
}
