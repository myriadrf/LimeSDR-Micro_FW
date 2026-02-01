/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#include "immap.h"
#include <la9310.h>
#include <la9310_i2cAPI.h>
#include "debug_console.h"
#include "drivers/serial/serial_ns16550.h"

void vBoardEarlyInit( void )
{
	xDebugConsoleInit( ( void * ) UART_BASEADDR, EARLY_UART_CLOCK_FREQUENCY,
			UART_BAUDRATE );
	iLa9310_I2C_Init( LA9310_FSL_I2C1, EARLY_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ );
}

void vBoardFinalInit( void )
{
	xDebugConsoleInit( ( void * ) UART_BASEADDR, FINAL_UART_CLOCK_FREQUENCY,
			UART_BAUDRATE );

	iLa9310_I2C_Init( LA9310_FSL_I2C1, FINAL_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ );

	// put GNSS module to low power standby mode, any further activity on UART will wake it up
	vSerialWriteBlocking(( void * ) UART_BASEADDR, "$PMTK161,0*28\r\n", 15);
}
