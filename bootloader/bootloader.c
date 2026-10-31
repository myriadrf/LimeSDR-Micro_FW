#include "la9310_boot_header.h"
#include "config.h"
#include "core_cm4.h"
#include "io.h"

#include "la9310_gpio.h"

#define M4_THUMB_BIT 0x1

#define LED1_PIN 17
#define LED2_PIN 18

static inline void LED_GPIO_config_output(uint32_t pin)
{
    uint32_t *p_reg = (uint32_t *)GPIO_BASE_ADDR;
    pin = 31 - pin;
    // output, not openDrain
    setbits_le32((uint32_t)(&p_reg[GPIO_DIR]), (1 << pin));
    setbits_le32((uint32_t)(&p_reg[GPIO_IBE]), (1 << pin));
}

static inline void LED_GPIO_set_value(uint32_t pin, bool value)
{
    uint32_t *p_reg = (uint32_t *)GPIO_BASE_ADDR;
    pin = 31 - pin;
    if (value)
        setbits_le32((uint32_t)(&p_reg[GPIO_DAT]), (1 << pin));
    else
        clrbits_le32((uint32_t)(&p_reg[GPIO_DAT]), (1 << pin));
}

// Boot-from-PCIe path of the ROM's main function, at the point where PCIe is already initialized.
// From here the ROM:
//  - polls TCMU for the boot preamble, then validates the boot header, error code is left in DCFG SCRATCHRW1 (0x41E00200)
//  - copies the image from host memory and jumps to it
#define ROM_PCIE_BOOT_ENTRY (0x00000CAE | M4_THUMB_BIT)
// The function keeps these in sp and r4, the same values it has at this point after a reset
#define ROM_PCIE_BOOT_STACK 0x2000FF8C
#define ROM_BOOT_SOURCE_PCIE 3

__attribute__((noreturn)) static void ROM_boot_firmware_over_pcie(void)
{
    // ROM execution path: Clears TCMU, PCIe init, load firmware.
    // Must skip the PCIe init, so explicitly clear TCMU and jump straight to the firmware load section.
    volatile struct la9310_boot_header *boot_header = (struct la9310_boot_header *)TCMU_PHY_ADDR;
    boot_header->preamble = 0;

    // MSP can only be set from privileged context
    asm volatile("msr msp, %[stack]\n"
                 "mov r4, %[boot_source]\n"
                 "bx %[entry]\n"
                 :
                 : [stack] "r"(ROM_PCIE_BOOT_STACK), [boot_source] "r"(ROM_BOOT_SOURCE_PCIE), [entry] "r"(ROM_PCIE_BOOT_ENTRY)
                 : "r4");
    __builtin_unreachable();
}

__attribute__((noreturn)) void bootloader_reset_handler(void)
{
    disable_irq();
    // Set Priviledged Mode.
    asm volatile("mrs    r0, control\n"
                 "bic    r0, 0x1\n"
                 "msr    control, r0\n"
                 :
                 :
                 : "r0");

    SYST->STCSR = (SYST->STCSR & ~(0x1)); // disable systick
#pragma GCC unroll 8
    for (int i = 0; i < 8; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFF; // disable external interrupts
        NVIC->ICPR[i] = 0xFFFFFFFF; // clear pending interrupt flags
    }

    SCB->ICSR = (1 << 30) | (1 << 27) | (1 << 25); // Clear pending systick, nmi, pendsv interrupts
    // switch back to ROM ISR vectors
    SCB->VTOR = 0;
    dsb();
    isb();

    // clear DCFG scratch registers, they were used to indicate FW startup status
    // and indicate for software that M4 has active firmware loaded
    for (uint32_t addr = 0x41E00200; addr <= 0x41E00220; addr += 4)
        OUT_32(addr, 0);
    // SCRATCHRW32: VSPA debug proxy pointer (src/vspa_debug.h), must not
    // outlive the firmware that published it
    OUT_32(0x41E0027C, 0);

    // on while waiting for the next firmware, which turns it off when it initializes the LEDs
    LED_GPIO_config_output(LED1_PIN);
    LED_GPIO_set_value(LED1_PIN, 0);
    dsb();

    ROM_boot_firmware_over_pcie();
    __builtin_unreachable();
    while (1)
    {
    };
}
