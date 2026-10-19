/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021-2022 NXP
 */
#include <stdint.h>
#include <stdbool.h>

#ifndef __LA9310_H__
    #define __LA9310_H__

    #define PRE_BOOT_PLATFORM_CLK_HZ 100000000 // before clock switch, running on PCIe clock 100 MHz

    #ifndef LA9310_DCS_CLK_HZ
        #warning LA9310_DCS_CLK_HZ not defined, using default 122880000
        #define LA9310_DCS_CLK_HZ 122880000
    #endif

    #define PLATFORM_CLOCK_HZ ((LA9310_DCS_CLK_HZ) * 4)

    /*this macro control clock switch from 100Mz to 122.88MHz.
 * When this macro is undefind VSPA, RFIC , DCS block will be disabled
 */
    #define UART_BAUDRATE 9600
    #define LA9310_I2C_FREQ 100000

    #define M4_CLOCK_IN_FROM_PLATFORM(plat) ((plat) / 2)
    #define UART_CLOCK_IN_FROM_PLATFORM(plat) ((plat) / 2)
    #define SPI_CLOCK_IN_FROM_PLATFORM(plat) ((plat) / 2)
    #define I2C_CLOCK_IN_FROM_PLATFORM(plat) ((plat) / 2)
    #define PHYTIMER_CLOCK_IN_FROM_PLATFORM(plat) ((plat) / 8)

    #define LX9310_BOOT_SRC_SHIFT 14
    #define LX9310_BOOT_SRC_MASK 3

typedef enum { LA9310_BOOT_SRC_RSV0, LA9310_BOOT_SRC_RSV1, LA9310_BOOT_SRC_I2C, LA9310_BOOT_SRC_PCIE } la9310_boot_src_t;

extern uint32_t g_la9310_current_platform_clk_hz;

void vUDelay(uint32_t microseconds);
bool la9310_clk_src_pcie(void);

#endif /* __LA9310_H__ */
