#include "boot_header.h"
#include "config.h"
#include "core_cm4.h"
#include "io.h"
#include "isrvectors.h"

#define PREAMBLE 0xaa55aa55
#define M4_THUMB_BIT 0x1

__attribute__((noreturn)) static void bootloader_main(void)
{
    volatile struct la9310_boot_header *boot_header = (struct la9310_boot_header *)TCMU_PHY_ADDR;

    while (boot_header->preamble != PREAMBLE)
        dmb();

    // log_info("boot_header->bl_src_offset: 0x%08x\n", src_offset);
    // log_info("boot_header->bl_dest: 0x%08x\n", dst_addr);
    // log_info("boot_header->bl_size: 0x%08x\n", size);
    // log_info("boot_header->bl_entry: 0x%08x\n", boot_header->bl_entry);

    uint32_t *dst = (uint32_t *)boot_header->bl_dest;
    const volatile uint32_t *src = (uint32_t *)(boot_header->bl_src_offset + PCIE_PHY_ADDR);
    const uint32_t size = boot_header->bl_size;

    for (uint32_t copied = 0; copied < size; copied += sizeof(uint32_t))
        *dst++ = *src++;

    boot_header->preamble = 0x0;

    void (*jump_to_fw_entry)(void) = (void *)(boot_header->bl_entry | M4_THUMB_BIT);
    jump_to_fw_entry();

    // Control should never reach here
    while (1)
    {
    };
}

__attribute__((interrupt)) static void DefaultISR(void)
{
    while (1)
    {
    };
}

__attribute__((interrupt)) void bootloader_reset_handler(void);
extern uint32_t __bootloader_stack;

// isr_vector must be aligned to a power of two boundary, with the minimum alignment of 0x100 bytes
__attribute__((aligned(0x100))) static const struct la9310_cm4_isrvectors bootloader_isrvector = {
    .stacktop = &__bootloader_stack,
    .reset_handler = &bootloader_reset_handler,
    .nmi_handler = &DefaultISR,
    .hard_fault_handler = &DefaultISR,
    .mpu_fault_handler = &DefaultISR,
    .bus_fault_handler = &DefaultISR,
    .usage_fault_handler = &DefaultISR,
    .reserved_0 = 0,
    .reserved_1 = 0,
    .reserved_2 = 0,
    .reserved_3 = 0,
    .svcall_handler = &DefaultISR,
    .debug_monitor_handler = &DefaultISR,
    .reserved_4 = 0,
    .pendsv_handler = &DefaultISR,
    .sys_tick_handler = &DefaultISR,

    // External Interrupts
    .gpio = &DefaultISR,
    .i2c1 = &DefaultISR,
    .i2c2 = &DefaultISR,
    .pcie = &DefaultISR,
    .spi1 = &DefaultISR,
    .reserved_5 = &DefaultISR,
    .ip1 = &DefaultISR,
    .ip2 = &DefaultISR,
    .ip3 = &DefaultISR,
    .edma = &DefaultISR,
    .msg1 = &DefaultISR,
    .msg2 = &DefaultISR,
    .msg3 = &DefaultISR,
    .watchdog = &DefaultISR,
    .uart = &DefaultISR,
    .aem = &DefaultISR,
    .mbee = &DefaultISR,
    .axiq = &DefaultISR,
    .adc_dac = &DefaultISR,
    .vspa = &DefaultISR,
    .thermal_alarm = &DefaultISR,
    .thermal_critical_alarm = &DefaultISR,
    .epu = &DefaultISR,
    .pps_in = &DefaultISR,
    .pps_out = &DefaultISR,
    .reserved_25 = &DefaultISR,
    .reserved_26 = &DefaultISR,
    .reserved_27 = &DefaultISR,
    .reserved_28 = &DefaultISR,
    .reserved_29 = &DefaultISR,
    .reserved_30 = &DefaultISR,
    .reserved_31 = &DefaultISR,

    .reserved_48 = &DefaultISR,
    .reserved_49 = &DefaultISR,
    .reserved_50 = &DefaultISR,
    .reserved_51 = &DefaultISR,
    .reserved_52 = &DefaultISR,
    .reserved_53 = &DefaultISR,
    .reserved_54 = &DefaultISR,
    .reserved_55 = &DefaultISR,
    .reserved_56 = &DefaultISR,
    .reserved_57 = &DefaultISR,
    .reserved_58 = &DefaultISR,
    .reserved_59 = &DefaultISR,
    .reserved_60 = &DefaultISR,
    .reserved_61 = &DefaultISR,
    .reserved_62 = &DefaultISR,
    .reserved_63 = &DefaultISR,
    .reserved_64 = &DefaultISR,
    .reserved_65 = &DefaultISR,
    .reserved_66 = &DefaultISR,
    .reserved_67 = &DefaultISR,
    .reserved_68 = &DefaultISR,
    .reserved_69 = &DefaultISR,
    .reserved_70 = &DefaultISR,
    .reserved_71 = &DefaultISR,
    .reserved_72 = &DefaultISR,
    .reserved_73 = &DefaultISR,
    .reserved_74 = &DefaultISR,
    .reserved_75 = &DefaultISR,
    .reserved_76 = &DefaultISR,
    .reserved_77 = &DefaultISR,
    .reserved_78 = &DefaultISR,
    .reserved_79 = &DefaultISR,
    .reserved_80 = &DefaultISR,
    .reserved_81 = &DefaultISR,
    .reserved_82 = &DefaultISR,
    .reserved_83 = &DefaultISR,
    .reserved_84 = &DefaultISR,
    .reserved_85 = &DefaultISR,
    .reserved_86 = &DefaultISR,
    .reserved_87 = &DefaultISR,
    .reserved_88 = &DefaultISR,
    .reserved_89 = &DefaultISR,
    .reserved_90 = &DefaultISR,
    .reserved_91 = &DefaultISR,
    .reserved_92 = &DefaultISR,
    .reserved_93 = &DefaultISR,
    .reserved_94 = &DefaultISR,
    .reserved_95 = &DefaultISR,
    .reserved_96 = &DefaultISR,
    .reserved_97 = &DefaultISR,
    .reserved_98 = &DefaultISR,
    .reserved_99 = &DefaultISR,
    .reserved_100 = &DefaultISR,
    .reserved_101 = &DefaultISR,
    .reserved_102 = &DefaultISR,
    .reserved_103 = &DefaultISR,
    .reserved_104 = &DefaultISR,
    .reserved_105 = &DefaultISR,
    .reserved_106 = &DefaultISR,
    .reserved_107 = &DefaultISR,
    .reserved_108 = &DefaultISR,
    .reserved_109 = &DefaultISR,
    .reserved_110 = &DefaultISR,
    .reserved_111 = &DefaultISR,
    .reserved_112 = &DefaultISR,
    .reserved_113 = &DefaultISR,
    .reserved_114 = &DefaultISR,
    .reserved_115 = &DefaultISR,
    .reserved_116 = &DefaultISR,
    .reserved_117 = &DefaultISR,
    .reserved_118 = &DefaultISR,
    .reserved_119 = &DefaultISR,
    .reserved_120 = &DefaultISR,
    .reserved_121 = &DefaultISR,
    .reserved_122 = &DefaultISR,
    .reserved_123 = &DefaultISR,
    .reserved_124 = &DefaultISR,
    .reserved_125 = &DefaultISR,
    .reserved_126 = &DefaultISR,
    .reserved_127 = &DefaultISR,
    .reserved_128 = &DefaultISR,
    .reserved_129 = &DefaultISR,
    .reserved_130 = &DefaultISR,
    .reserved_131 = &DefaultISR,
    .reserved_132 = &DefaultISR,
    .reserved_133 = &DefaultISR,
    .reserved_134 = &DefaultISR,
    .reserved_135 = &DefaultISR,
    .reserved_136 = &DefaultISR,
    .reserved_137 = &DefaultISR,
    .reserved_138 = &DefaultISR,
    .reserved_139 = &DefaultISR,
    .reserved_140 = &DefaultISR,
    .reserved_141 = &DefaultISR,
    .reserved_142 = &DefaultISR,
    .reserved_143 = &DefaultISR,
};

__attribute__((noreturn)) void bootloader_reset_handler(void)
{
    asm("cpsid i" : : :); // disable interupts

    // Initialize stack pointer
    asm("ldr r0, = __bootloader_stack\n"
        "msr msp, r0\n"
        :
        :
        :);

    // relocate interrupt vector table
    SCB->VTOR = (uint32_t)&bootloader_isrvector;
    isb();
    dmb();

    // Set Priviledged Mode
    asm("mrs    r0, control\n"
        "bic    r0, 0x1\n"
        "msr    control, r0\n"
        :
        :
        :);

    bootloader_main();

    // Never returns to here
    while (1)
    {
    };
}
