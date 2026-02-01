#include "eeprom.h"

#include "la9310_i2cAPI.h"
#include "immap.h"

static const uint8_t eeprom_i2c_address = 0x50;

void EEPROM_Write(uint16_t mem_address, const void* buffer, uint32_t length)
{
	int bytesWritten = iLa9310_I2C_Write(LA9310_FSL_I2C1, eeprom_i2c_address, mem_address, LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, buffer, length);
}

void EEPROM_Read(uint16_t mem_address, void* buffer, uint32_t length)
{
	int bytesRead = iLa9310_I2C_Read(LA9310_FSL_I2C1, eeprom_i2c_address, mem_address, LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, buffer, length);
}
