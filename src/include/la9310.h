/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021-2022 NXP
 */

#ifndef __LA9310_H__
#define __LA9310_H__

#define PRE_SYS_FREQ     100000000
#ifndef LA9310_SYS_CLK_FREQ
	#define POST_SYS_FREQ    122880000 /*NLM final DCS frequency to switch */
#else
	#define POST_SYS_FREQ    LA9310_SYS_CLK_FREQ
#endif

/*this macro control clock switch from 100Mz to 122.88MHz.
 * When this macro is undefind VSPA, RFIC , DCS block will be disabled
 */
#define UART_BAUDRATE                     9600
#define EARLY_UART_CLOCK_FREQUENCY        PRE_SYS_FREQ
#define EARLY_I2C_CLOCK_FREQUENCY         ( PRE_SYS_FREQ / 2 )

#define FINAL_UART_CLOCK_FREQUENCY    ( POST_SYS_FREQ * 2 ) /* Fix Me */
#define FINAL_I2C_CLOCK_FREQUENCY     ( POST_SYS_FREQ / 2 ) /*Fix Me */
#define PLAT_FREQ                     ( POST_SYS_FREQ * 4 )

#define LA9310_I2C_FREQ                   100000

#define LX9310_BOOT_SRC_SHIFT             14
#define LX9310_BOOT_SRC_MASK              3
#endif /* __LA9310_H__ */
