#include "FreeRTOS.h"
#include <semphr.h>

#include "fsl_dspi.h"
#include "la9310_host_if.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "lms7002m/spi.h"

extern struct LA931xDspiInstance *lmsspihandle;
extern int32_t spi_lms7002m_read( struct LA931xDspiInstance * pDspiHandle, uint16_t addr, uint16_t *value );
extern int32_t spi_lms7002m_write( struct LA931xDspiInstance * pDspiHandle, uint16_t addr, uint16_t value );

static SemaphoreHandle_t xSwCmdSemaphore;
static volatile int runEngine = 1;

struct LMS64CPacket {
    uint8_t cmd;
    uint8_t status;
    uint8_t blockCount; ///< The count of blocks in the payload.
    uint8_t periphID; ///< The ID of the peripheral to use.
    uint8_t subDevice; ///< The ID of the subdevice to use.
    uint8_t reserved[3]; ///< Currently unused
    uint8_t payload[56]; ///< The information of the payload.
};

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
        for (int i=0; i<packet->blockCount && i < 7; ++i)
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
        for (int i=0; i<packet->blockCount && i < 7; ++i)
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
    return 0;
}

static void vSwCmdTask( void * pvParameters )
{
    struct la9310_hif * pxHif = pLa9310Info->pHif;
    struct la9310_sw_cmd_desc * pxCmdDesc = &( pxHif->sw_cmd_desc );

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

        switch( pxCmdDesc->cmd )
        {
            case 1:
                pxCmdDesc->status = LA9310_SW_CMD_STATUS_IN_PROGRESS;
                ProcessLMS64C_Command(pxCmdDesc->data, response);
                memcpy(pxCmdDesc->data, response, sizeof(response));
                break;
            default:
                log_err( "sw cmd not implemented: %d\r\n", pxCmdDesc->cmd );
                break;
        }

        pxCmdDesc->status = LA9310_SW_CMD_STATUS_DONE;
        dmb();

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
    xRet = xTaskCreate( vSwCmdTask, "lms64c task", 512, NULL, tskIDLE_PRIORITY + 1, NULL );
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
    PRINTF("LMS64C init\r\n");

    if( lSwCmdEngineInit() != 0 )
    {
        ret = -1;
        goto out;
    }
out:
    return ret;
}
