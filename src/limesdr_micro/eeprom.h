#ifndef LIMESDR_EEPROM_H
#define LIMESDR_EEPROM_H

#include <stdint.h>

extern void EEPROM_Write(uint16_t address, const void* buffer, uint32_t length);
extern void EEPROM_Read(uint16_t address, void* buffer, uint32_t length);

#endif