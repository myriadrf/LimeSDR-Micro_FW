#include "FreeRTOS.h"
#include <semphr.h>

#include "fsl_dspi.h"
#include "la9310_dcs_api.h"
#include "la9310_host_if.h"
#include "la9310_i2cAPI.h"
#include "debug_console.h"
#include "io.h"
#include "core_cm4.h"
#include "drivers/serial/serial_ns16550.h"

#include "log.h"
#include "immap.h"
#include <string.h>

#include "la9310.h"
#include "la9310_info.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "lms7002m/spi.h"

#include "eeprom.h"
#include "fwloader.h"

static uint16_t xo_dac_value = 0;

extern struct LA931xDspiInstance *lmsspihandle;
extern int32_t spi_lms7002m_read( struct LA931xDspiInstance * pDspiHandle, uint16_t addr, uint16_t *value );
extern int32_t spi_lms7002m_write( struct LA931xDspiInstance * pDspiHandle, uint16_t addr, uint16_t value );
extern void UseExternalReferenceClock(bool external);
extern int IsExternalRefClkUsed();

extern struct lms7002m_context* rfsoc;
extern struct la9310_info g_la9310_info;

static SemaphoreHandle_t xSwCmdSemaphore;
static volatile int runEngine = 1;

enum CommandStatus { cmd_status_Undefined, cmd_status_Completed, cmd_status_Unknown, cmd_status_Busy, cmd_status_TooManyBlocks, cmd_status_Error, cmd_status_WrongOrder, cmd_status_ResourceDenied, cmd_status_Count };

struct LMS64CPacket {
    uint8_t cmd;
    uint8_t status;
    uint8_t blockCount; ///< The count of blocks in the payload.
    uint8_t periphID; ///< The ID of the peripheral to use.
    uint8_t subDevice; ///< The ID of the subdevice to use.
    uint8_t reserved[3]; ///< Currently unused
    uint8_t payload[56]; ///< The information of the payload.
};

void ReadBigEndiann(void* littleEndiannDest, const void* srcBigEndiann, uint8_t byteCount)
{
    uint8_t* dest = (uint8_t*)littleEndiannDest;
    uint8_t* src = (uint8_t*)srcBigEndiann;
    for (int i=0; i<byteCount; ++i)
        dest[i] = src[byteCount-1-i];
}

void LMS64C_I2CWrite(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    uint32_t i2c_address = 0;
    uint32_t register_offset = 0;

    const int address_length = packet->payload[0];
    const int reg_offset_length = packet->payload[1];
    const int data_length = packet->payload[2];
    // payload[3] reserved

    int payloadOffset = 4;
    ReadBigEndiann(&i2c_address, &packet->payload[payloadOffset], address_length);
    payloadOffset += address_length;
    ReadBigEndiann(&register_offset, &packet->payload[payloadOffset], reg_offset_length);
    payloadOffset += reg_offset_length;
    memcpy(responsePacket->payload, packet->payload, payloadOffset + data_length);

    int bytesWritten = iLa9310_I2C_Write(LA9310_FSL_I2C1, i2c_address, register_offset, reg_offset_length, &responsePacket->payload[payloadOffset], data_length);

    responsePacket->cmd = packet->cmd;
    if (bytesWritten != data_length)
        responsePacket->status = cmd_status_Error;
    else
        responsePacket->status = cmd_status_Completed;
}

void LMS64C_I2CRead(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    uint32_t i2c_address = 0;
    uint32_t register_offset = 0;

    const int address_length = packet->payload[0];
    const int reg_offset_length = packet->payload[1];
    const int data_length = packet->payload[2];
    // payload[3] reserved

    int payloadOffset = 4;
    ReadBigEndiann(&i2c_address, &packet->payload[payloadOffset], address_length);
    payloadOffset += address_length;
    ReadBigEndiann(&register_offset, &packet->payload[payloadOffset], reg_offset_length);
    payloadOffset += reg_offset_length;

    memcpy(responsePacket->payload, packet->payload, payloadOffset);
    int bytesRead = iLa9310_I2C_Read(LA9310_FSL_I2C1, i2c_address, register_offset, reg_offset_length, &responsePacket->payload[payloadOffset], data_length);

    responsePacket->cmd = packet->cmd;
    if (bytesRead != data_length)
        responsePacket->status = cmd_status_Error;
    else
        responsePacket->status = cmd_status_Completed;
    responsePacket->blockCount = payloadOffset + bytesRead;
}

uint16_t ReadXODAC_EEPROM()
{
    uint16_t dac_value = 0;
    EEPROM_Read(0xFFFE, &dac_value, 2);
    return dac_value;
}

void SetXODAC(uint16_t value)
{
    uint8_t buffer[2] = {value >> 8, value & 0xFF};
    int bytesWritten = iLa9310_I2C_Write(LA9310_FSL_I2C1, 0x4C, 0x30, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, buffer, 2);
    if (bytesWritten != 2)
    {
        log_err("Failed to write XODAC(%d)\n", value);
    }
    else
    {
        xo_dac_value = value;
        log_info("XO_DAC set to (%d)\n", value);
    }
}

void LMS64C_CustomParameterWrite(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    for (int i=0; i<packet->blockCount; ++i)
    {
        const int offset = 4*i;
        uint8_t id = packet->payload[offset];
        // uint8_t powerOf10 = packet->payload[offset+1] & 0xF;
        // uint8_t units = (packet->payload[offset+1] >> 4) & 0xF;
        uint16_t value = ((uint16_t)packet->payload[offset+2]) << 8 | packet->payload[offset+3];

        switch(id)
        {
            case 0:
            {
                SetXODAC(value);
                break;
            }
        }
    }
    memcpy(responsePacket, packet, sizeof(struct LMS64CPacket));
    responsePacket->status = cmd_status_Completed;
}

static int32_t hex2temperature_mC(int16_t raw_value)
{
    const int16_t temperature_mC[] = {-12800, -5000, -2500, -25, -6, 0, 6, 25, 2500, 5000, 7500, 8000, 10000, 12793};
    const uint16_t digital_value[] = {0x8000, 0xCE00, 0xE700, 0xFFC0, 0xFFF0, 0x0, 0x0010, 0x0040, 0x1900, 0x3200, 0x4B00, 0x5000, 0x6400, 0x7FF0};

    for (int i=0; i<sizeof(digital_value)/sizeof(digital_value[0])-1; ++i)
    {
        if (raw_value >= (int16_t)digital_value[i] && raw_value < (int16_t)digital_value[i+1])
        {
            int32_t temp_delta = temperature_mC[i+1] - temperature_mC[i];
            int32_t value_delta = digital_value[i+1] - digital_value[i];
            int32_t xrange = raw_value - digital_value[i];
            int32_t delta = (xrange << 16) / value_delta;
            int32_t temp_value = (temp_delta * delta) >> 16;
            return (temp_value + temperature_mC[i]) * 10;
        }
    }
    return 0;
}

void LMS64C_CustomParameterRead(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    int byteIndex = 0;
    memcpy(responsePacket, packet, 8);
    responsePacket->blockCount = 0;
    for (int i=0; i<packet->blockCount; ++i)
    {
        uint8_t id = packet->payload[i];
        switch(id)
        {
            case 0: // XO DAC
            {
                responsePacket->payload[byteIndex++] = id;
                responsePacket->payload[byteIndex++] = 0; // RAW
                responsePacket->payload[byteIndex++] = xo_dac_value >> 8;
                responsePacket->payload[byteIndex++] = xo_dac_value & 0xFF;
                ++responsePacket->blockCount;
                break;
            }
            case 1: // on board temperature
            {
                responsePacket->payload[byteIndex++] = id;
                responsePacket->payload[byteIndex++] = 5 << 4; // degrees
                uint8_t raw_value[2];
                int bytesRead = iLa9310_I2C_Read(LA9310_FSL_I2C1, 0x4B, 0x00, 1, raw_value, 2);
                int16_t raw_value_int = ((int16_t)raw_value[0]) << 8 | raw_value[1];
                int32_t temp = hex2temperature_mC(raw_value_int) / 100;
                responsePacket->payload[byteIndex++] = temp >> 8;
                responsePacket->payload[byteIndex++] = temp & 0xFF;
                ++responsePacket->blockCount;
                break;
            }
        }
    }
    responsePacket->status = cmd_status_Completed;
}

void LMS64C_MemoryWrite(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    memcpy(responsePacket, packet, sizeof(struct LMS64CPacket));

    // uint8_t mode = packet->payload[0];
    uint16_t memoryTarget = (int16_t)(packet->payload[10] << 8) | packet->payload[11];
    uint8_t size = packet->payload[5];
    uint32_t address = ((int32_t)packet->payload[6]) << 24;
    address |= ((int32_t)packet->payload[7]) << 16;
    address |= ((int32_t)packet->payload[8]) << 8;
    address |= ((int32_t)packet->payload[9]);

    uint8_t* data = &packet->payload[24];

    if (memoryTarget != 3) // EEPROM
    {
        responsePacket->status = cmd_status_Error;
        return;
    }

    EEPROM_Write(address, data, size);
    responsePacket->status = cmd_status_Completed;
}

void LMS64C_MemoryRead(const struct LMS64CPacket* packet, struct LMS64CPacket* responsePacket)
{
    memcpy(responsePacket, packet, sizeof(struct LMS64CPacket));

    // uint8_t mode = packet->payload[0];
    uint16_t memoryTarget = (int16_t)(packet->payload[10] << 8) | packet->payload[11];
    uint8_t size = packet->payload[5];
    uint32_t address = ((int32_t)packet->payload[6]) << 24;
    address |= ((int32_t)packet->payload[7]) << 16;
    address |= ((int32_t)packet->payload[8]) << 8;
    address |= ((int32_t)packet->payload[9]);

    if (memoryTarget != 3) // EEPROM
    {
        responsePacket->status = cmd_status_Error;
        return;
    }

    EEPROM_Read(address, &responsePacket->payload[24], size);
    responsePacket->status = cmd_status_Completed;
}

static int ProcessLMS64C_Command(const void* dataIn, void* dataOut)
{
    const struct LMS64CPacket* packet = (struct LMS64CPacket*)(dataIn);
    struct LMS64CPacket* outPacket = (struct LMS64CPacket*)(dataOut);
    outPacket->status = 2;
    switch(packet->cmd)
    {
    case 0x00: // GET_INFO
    {
        outPacket->status = 1;
        outPacket->payload[0] = 1; //firmware version
        outPacket->payload[1] = 29; // device id
        outPacket->payload[2] = 0; // protocol version
        outPacket->payload[3] = 0; // hardware version
        outPacket->payload[4] = 0; // expansion/daughter board id
        for (int i = 10; i < 18; i++) // serial number
        {
            outPacket->payload[i] = 0;
        }
        break;
    }
    case 0x21: // LMS7002_WR
    {
        for (int i=0; i<packet->blockCount && i < sizeof(packet->payload)/sizeof(uint32_t); ++i)
        {
            uint16_t addr = (packet->payload[i*4] << 8) | packet->payload[i*4+1];
            uint16_t value = (packet->payload[i*4+2] << 8) | packet->payload[i*4+3];
            spi_lms7002m_write(lmsspihandle, addr, value);
            outPacket->payload[i*4] = addr >> 8;
            outPacket->payload[i*4+1] = addr;
            outPacket->payload[i*4+2] = value >> 8;
            outPacket->payload[i*4+3] = value;
        }
        outPacket->status = 1;
        outPacket->blockCount = packet->blockCount;
        break;
    }
    case 0x22: // LMS7002_RD
    {
        for (int i=0; i<packet->blockCount && i < sizeof(packet->payload)/sizeof(uint32_t); ++i)
        {
            uint16_t addr = (packet->payload[i*2] << 8) | packet->payload[i*2+1];
            uint16_t value = 0;
            spi_lms7002m_read(lmsspihandle, addr, &value);
            outPacket->payload[i*4] = addr >> 8;
            outPacket->payload[i*4+1] = addr;
            outPacket->payload[i*4+2] = value >> 8;
            outPacket->payload[i*4+3] = value;
        }
        outPacket->status = 1;
        outPacket->blockCount = packet->blockCount;
        break;
    }
    case 0x25: // I2C_WR
    {
        LMS64C_I2CWrite(packet, outPacket);
        break;
    }
    case 0x26: // I2C_RD
    {
        LMS64C_I2CRead(packet, outPacket);
        break;
    }
    case 0x61: // ANALOG_VAL_WR
    {
        LMS64C_CustomParameterWrite(packet, outPacket);
        break;
    }
    case 0x62: // ANALOG_VAL_RD
    {
        LMS64C_CustomParameterRead(packet, outPacket);
        break;
    }
    case 0x8C: // Memory write
    {
        LMS64C_MemoryWrite(packet, outPacket);
        break;
    }
    case 0x8D: // Memory read
    {
        LMS64C_MemoryRead(packet, outPacket);
        break;
    }
    default:
        outPacket->status = 2; // UNKNOWN
        break;
    }
    return 0;
}

static lime_Result SetLA9310SystemClock(struct la9310_info * pLa9310Info, uint32_t system_clk_hz)
{
    if (system_clk_hz == 0)
        return lime_Result_InvalidValue;

    lime_Result result = lms7002m_set_frequency_cgen(rfsoc, 4 * system_clk_hz);
    if (result != lime_Result_Success)
        return result;

    vSerialInit((void *)UART_BASEADDR, UART_BAUDRATE, system_clk_hz * 2);

    uint32_t i2c_input_clk_hz = system_clk_hz / 2;
    iLa9310_I2C_Init( LA9310_FSL_I2C1, i2c_input_clk_hz, LA9310_I2C_FREQ );

    uint32_t spi_input_clk_hz = system_clk_hz * 4 / 2;
    uint32_t spi_frequency = 4000000;
    if (spi_input_clk_hz < spi_frequency*2)
        spi_frequency = spi_input_clk_hz/2;
    vDspiClkSet( lmsspihandle, spi_input_clk_hz, spi_frequency );

    vDcsInit(IN_32(&pLa9310Info->pHif->adc_mask),
        IN_32(&pLa9310Info->pHif->adc_rate_mask),
        IN_32(&pLa9310Info->pHif->dac_mask),
        IN_32(&pLa9310Info->pHif->dac_rate_mask));

    // TODO: reconfigure PPS out phytimer

    return result;
}

static lime_Result ConfigureReferenceClock(uint32_t clk_hz, bool external)
{
    if (clk_hz == 0)
        return lime_Result_InvalidValue;

    if (clk_hz < 10000000 || clk_hz > 52000000)
        return lime_Result_OutOfRange;

    uint32_t currentRefClk = lms7002m_get_reference_clock(rfsoc);
    if (clk_hz == currentRefClk)
    {
        // just flip the switch
        UseExternalReferenceClock(external);
        return lime_Result_Success;
    }
    bool isExtClkUsed = IsExternalRefClkUsed();

    uint32_t currentCGEN_Hz = lms7002m_get_frequency_cgen(rfsoc);
    lime_Result status = lime_Result_Error;
    if (clk_hz < currentRefClk)
    {
        // preconfigure CGEN to a higher frequency, so that when ref clock source switches CGEN would not fall below supported frequency
        uint32_t upscaleRatio = (currentRefClk / clk_hz) + ((currentRefClk % clk_hz) != 0);
        status = lms7002m_set_frequency_cgen(rfsoc, currentCGEN_Hz * upscaleRatio);
    }
    else
    {
        // preconfigure CGEN to a lower frequency, so that when ref clock source switches CGEN would not fall above supported frequency
        uint32_t downscaleRatio = (clk_hz / currentRefClk) + ((clk_hz % currentRefClk) != 0);
        status = lms7002m_set_frequency_cgen(rfsoc, currentCGEN_Hz / downscaleRatio);
    }

    // flip clock source switch
    UseExternalReferenceClock(external);
    lms7002m_set_reference_clock(rfsoc, clk_hz);

    // retune with new reference clock values
    status = lms7002m_set_frequency_cgen(rfsoc, currentCGEN_Hz);
    if (status != lime_Result_Success)
    {
        // revert back to original settings
        UseExternalReferenceClock(isExtClkUsed);
        lms7002m_set_reference_clock(rfsoc, currentRefClk);
        lms7002m_set_frequency_cgen(rfsoc, currentCGEN_Hz);
    }
    return status;
}

static void vSwCmdTask( void * pvParameters )
{
    struct la9310_hif * pxHif = g_la9310_info.pHif;
    volatile struct la9310_sw_cmd_desc * pxCmdDesc = &( pxHif->sw_cmd_desc );

    uint8_t response[64];

    while( runEngine )
    {
        // xSemaphoreTake( xSwCmdSemaphore, portMAX_DELAY );

        if( pxCmdDesc->status != LA9310_SW_CMD_STATUS_POSTED )
        {
            // log_err( "sw cmd status is not posted\r\n" );
            continue;
        }

        memset(response, 0, sizeof(response));
        int status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
        switch( pxCmdDesc->cmd )
        {
            case 1:
                ProcessLMS64C_Command(pxCmdDesc->data, response);
                memcpy(pxCmdDesc->data, response, sizeof(response));
                status = LA9310_SW_CMD_STATUS_DONE;
                break;
            case 2: { // Set LA9310 system clock
                pxCmdDesc->status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
                uint32_t frequency = pxCmdDesc->data[0];
                lime_Result result = SetLA9310SystemClock(&g_la9310_info, frequency);
                pxCmdDesc->data[0] = (uint32_t)result;
                status = LA9310_SW_CMD_STATUS_DONE;
                break;
            }
            case 3: { // Get Reference clock
                pxCmdDesc->status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
                uint32_t frequency = lms7002m_get_reference_clock(rfsoc);
                pxCmdDesc->data[0] = frequency;
                status = LA9310_SW_CMD_STATUS_DONE;
                break;
            }
            case 4: { // Set Reference clock and source
                pxCmdDesc->status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
                uint32_t frequency = pxCmdDesc->data[0];
                uint32_t external = pxCmdDesc->data[1];
                lime_Result result = ConfigureReferenceClock(frequency, external);
                pxCmdDesc->data[0] = (uint32_t)result;
                status = LA9310_SW_CMD_STATUS_DONE;
                break;
            }
            case 5: { // Enter firmware reloading mode 
                prepare_fwloader();
                fwloader();
                break;
            }
            case 6: { // Signalize firmware is alive
                uint32_t pattern = pxCmdDesc->data[0];
                pxCmdDesc->status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
                pxCmdDesc->data[0] = ~pattern;
                status = LA9310_SW_CMD_STATUS_DONE;
                log_info("Alive Pattern - changed 0x%08x to 0x%08x\n", pattern, ~pattern);

                break;
            }
            default:
                log_err( "sw cmd not implemented: %d\r\n", pxCmdDesc->cmd );
                status = LA9310_SW_CMD_STATUS_ERROR;
                break;
        }

        dmb();
        pxCmdDesc->status = status;

        // xSemaphoreGive(xSwCmdSemaphore);
    }
}

static int lSwCmdEngineInit()
{
    BaseType_t xRet;
    xSwCmdSemaphore = xSemaphoreCreateBinary();
    if( xSwCmdSemaphore == NULL )
    {
        return -1;
    }
    // xSemaphoreGive(xSwCmdSemaphore);
    xRet = xTaskCreate( vSwCmdTask, "", 512, NULL, tskIDLE_PRIORITY + 1, NULL );
    if( xRet != pdPASS )
    {
        return -1;
    }

    NVIC_SetPriority( IRQ_MSG2, 3 );
    NVIC_EnableIRQ( IRQ_MSG2 );

    return 0;
}

int LMS64C_protocol_init(void)
{
    int ret = 0;
    PRINTF("LMS64C init\n");

    if( lSwCmdEngineInit() != 0 )
    {
        ret = -1;
        goto out;
    }
out:
    return ret;
}
