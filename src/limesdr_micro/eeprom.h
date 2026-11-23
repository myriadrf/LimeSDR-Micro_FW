#ifndef LIMESDR_EEPROM_H
#define LIMESDR_EEPROM_H

#include <stdbool.h>
#include <stdint.h>

extern void EEPROM_Write(uint16_t address, const void* buffer, uint32_t length);
extern void EEPROM_Read(uint16_t address, void* buffer, uint32_t length);

// Reads the board serial number from the EEPROM identification page.
extern bool EEPROM_ReadSerialNumber(uint64_t* serial);

// Reads whether the EEPROM identification page is permanently write protected. Returns false if the EEPROM does not respond.
// The EEPROM write control (WC) pin must be low, otherwise the page reads as locked.
extern bool EEPROM_ReadIdPageLockStatus(bool* locked);

#endif
