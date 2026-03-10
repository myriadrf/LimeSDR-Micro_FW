#include "isrvectors.h"
#include "core_cm4.h"
#include "io.h"

extern void NMI_Handler(void);
extern void vHardFaultHandler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void PCIE_IRQHandler(void);
extern void La9310eDMA_IRQHandler(void);
extern void La9310MSG_1_IRQHandler(void);
extern void La9310MSG_2_IRQHandler(void);
extern void La9310MSG_3_IRQHandler(void);
extern void La9310WDOG_IRQHandler(void);
extern void La9310VSPA_IRQHandler(void);
extern void vPhyTimerPPSINHandler(void);
extern void vPhyTimerPPSOUTHandler(void);

extern void _start(void);

static void DefaultISR(void)
{
    while (1)
    {
    };
}

extern uint32_t __stack;
void m4_reset_handler(void);

// isr_vector must be aligned to a power of two boundary, with the minimum alignment of 0x100 bytes
__attribute__((aligned(0x100), section(".isr_vector"))) struct la9310_cm4_isrvectors isrvector = {
    .stacktop = &__stack,
    .reset_handler = &m4_reset_handler,
    .nmi_handler = &DefaultISR,
    .hard_fault_handler = &vHardFaultHandler,
    .mpu_fault_handler = &DefaultISR,
    .bus_fault_handler = &DefaultISR,
    .usage_fault_handler = &DefaultISR,
    .reserved_0 = 0,
    .reserved_1 = 0,
    .reserved_2 = 0,
    .reserved_3 = 0,
    .svcall_handler = &SVC_Handler,
    .debug_monitor_handler = &DefaultISR,
    .reserved_4 = 0,
    .pendsv_handler = &PendSV_Handler,
    .sys_tick_handler = &SysTick_Handler,

    // External Interrupts
    .gpio = &DefaultISR,
    .i2c1 = &DefaultISR,
    .i2c2 = &DefaultISR,
    .pcie = &PCIE_IRQHandler,
    .spi1 = &DefaultISR,
    .reserved_5 = &DefaultISR,
    .ip1 = &DefaultISR,
    .ip2 = &DefaultISR,
    .ip3 = &DefaultISR,
    .edma = &La9310eDMA_IRQHandler,
    .msg1 = &La9310MSG_1_IRQHandler,
    .msg2 = &DefaultISR,
    .msg3 = &La9310MSG_3_IRQHandler,
    .watchdog = &La9310WDOG_IRQHandler,
    .uart = &DefaultISR,
    .aem = &DefaultISR,
    .mbee = &DefaultISR,
    .axiq = &DefaultISR,
    .adc_dac = &DefaultISR,
    .vspa = &La9310VSPA_IRQHandler,
    .thermal_alarm = &DefaultISR,
    .thermal_critical_alarm = &DefaultISR,
    .epu = &DefaultISR,
    .pps_in = &vPhyTimerPPSINHandler,
    .pps_out = &vPhyTimerPPSOUTHandler,
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

void m4_reset_handler(void)
{
    disable_irq();
    SYST->STCSR = (SYST->STCSR & ~(0x1)); // disable systick

    /* Initialize Core Registers */
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

    /* Initialize stack pointer */
    asm("ldr r0, = __stack\n"
        "msr msp, r0\n"
        :
        :
        :);

    // relocate interrupt vector table
    SCB->VTOR = (uint32_t)&isrvector;
    isb();
    dmb();

    // Set Priviledged Mode
    asm("mrs    r0, control\n"
        "bic    r0, 0x1\n"
        "msr    control, r0\n"
        :
        :
        :);

    /* Jump to IBR Main Code */
    enable_irq();
    _start();

    // Never returns from here
    while (1)
    {
    };
}
