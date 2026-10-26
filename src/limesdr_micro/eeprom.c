#include "eeprom.h"

#include "la9310_i2cAPI.h"
#include "immap.h"

static const uint8_t eeprom_i2c_address = 0x50;
// M24xxx-D identification page is accessed with device type identifier 0b1011 instead of 0b1010
static const uint8_t eeprom_id_page_i2c_address = 0x58;

// OEM data layout in the identification page
#define OTP_MAGIC 0x4C494D45 // "LIME"
#define OTP_FORMAT_VERSION 1
#define OTP_HEADER_LENGTH 12
#define OTP_DATA_LENGTH 68
#define OTP_UUID_OFFSET 0x12
#define OTP_UUID_LENGTH 16

void EEPROM_Write(uint16_t mem_address, const void* buffer, uint32_t length)
{
    //int bytesWritten =
    iLa9310_I2C_Write(LA9310_FSL_I2C1, eeprom_i2c_address, mem_address, LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, buffer, length);
}

void EEPROM_Read(uint16_t mem_address, void* buffer, uint32_t length)
{
    //int bytesRead =
    iLa9310_I2C_Read(LA9310_FSL_I2C1, eeprom_i2c_address, mem_address, LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, buffer, length);
}

static uint32_t ReadU32BigEndian(const uint8_t* src)
{
    return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) | ((uint32_t)src[2] << 8) | src[3];
}

// IEEE 802.3 CRC-32
static uint32_t CRC32(const uint8_t* data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

// The serial number is the lowest 8 bytes of the UUID. Returns false if the identification page content is not valid.
bool EEPROM_ReadSerialNumber(uint64_t* serial)
{
    uint8_t otp[OTP_HEADER_LENGTH + OTP_DATA_LENGTH];
    *serial = 0;

    const int bytesRead =
        iLa9310_I2C_Read(LA9310_FSL_I2C1, eeprom_id_page_i2c_address, 0, LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, otp, sizeof(otp));
    if (bytesRead != sizeof(otp))
        return false;

    if (ReadU32BigEndian(&otp[0]) != OTP_MAGIC)
        return false;
    if (otp[4] != OTP_FORMAT_VERSION)
        return false;
    const uint16_t dataLength = (otp[6] << 8) | otp[7];
    if (otp[5] != OTP_HEADER_LENGTH || dataLength != OTP_DATA_LENGTH)
        return false;
    if (ReadU32BigEndian(&otp[8]) != CRC32(&otp[OTP_HEADER_LENGTH], dataLength))
        return false;

    // lowest 8 bytes of the big-endian UUID
    uint64_t value = 0;
    for (int i = OTP_UUID_OFFSET + OTP_UUID_LENGTH - 8; i < OTP_UUID_OFFSET + OTP_UUID_LENGTH; ++i)
        value = (value << 8) | otp[i];
    *serial = value;
    return true;
}
