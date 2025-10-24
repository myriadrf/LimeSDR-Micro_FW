#include "la9310_i2cAPI.h"
#include "fsl_dspi.h"
#include "debug_console.h"

#include "utils/delay.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "lms7002m/spi.h"
#include "lms7002m/csr_data.h"

struct LA931xDspiInstance *lmsspihandle = NULL;

extern uint16_t ReadXODAC_EEPROM();
extern void SetXODAC(uint16_t value);

// ARM bare metal does not have usleep(), it's used within lms7002m procedures
void usleep(int microSeconds)
{
	vUDelay(microSeconds);
}

int i2c_write8(int module, uint32_t i2c_moduleAddress, uint8_t addr, uint8_t value)
{
    return iLa9310_I2C_Write(module, i2c_moduleAddress, addr, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &value, 1);
}

int32_t spi_lms7002m_write( struct LA931xDspiInstance * pDspiHandle,
                                 uint16_t addr,
                                 uint16_t value )
{
    int32_t iRet = 0;
    int32_t j=0;
    uint8_t ucData[ 4 ] = { 0 };

    uint32_t data = (1 << 31) | (addr << 16) | (value);
    /* Write synthesizer register */
    for (int i = 3; i >=0 ; i--) {
        ucData[j] = (data >> ( i * 8) ) & 0xFF;
        j++;
    }

    iRet = lDspiPush( pDspiHandle, DSPI_CS0, DSPI_DEV_WRITE,
              ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "lDspiPush failed, error[%d]\r\n", iRet );
    }

    return iRet;
}

int32_t spi_lms7002m_read( struct LA931xDspiInstance * pDspiHandle,
                                uint16_t addr,
                                uint16_t *value )
{
    int32_t iRet = 0;
    uint8_t ucData[ 4 ] = { 0 };

    /* Push address to be read */
    ucData[ 1 ] = addr;
    ucData[ 0 ] = (addr >> 8);
    iRet = lDspiPush( pDspiHandle, DSPI_CS0, DSPI_DEV_READ,
                      ucData, 4 );

    if( 0 > iRet )
    {
        *value = 0xDEAD;
        log_err( "lDspiPush failed, error[%d]\r\n", iRet );
        return iRet;
    }

    iRet = lDspiPop( pDspiHandle, ucData, 4 );

    if( 0 > iRet )
    {
        *value = 0xBEEF;
        log_err( "lDspiPop failed, error[%d]\r\n", iRet );
        return iRet;
    }
    /* index 0 should be ignored, this is outcome
     * of above push operation */
    *value = (ucData[ 2 ] << 8 ) | (ucData[ 3 ] );
    // *data = (ucData[ 3 ] << 24 ) | (ucData[ 2 ] << 16 ) | (ucData[ 1 ] << 8 )| ucData[ 0 ];

    return iRet;
}

static int spi16_transact(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData)
{
    struct LA931xDspiInstance * pDspiHandle = userData;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (mosi[i] & (1 << 31))
        {
            uint16_t addr = mosi[i] >> 16;
            addr &= 0x7FFF; // clear write bit for now
            uint16_t value = mosi[i] & 0xFFFF;
            spi_lms7002m_write(pDspiHandle, addr, value);
        }
        else
        {
            uint16_t addr = mosi[i] >> 16;
            uint16_t value = 0;
            spi_lms7002m_read(pDspiHandle, addr, &value);
            if (miso)
                miso[i] = value;
        }
    }
    return 0;
}

int initialize_lms7002m_clock_generator()
{
    uint16_t IC26_ADDR = 0x60;
    uint16_t IC30_ADDR = 0x60;

    log_info("lime i2c config start\n\r");

    // I2C I/O expander (IC15, MCP23017-E/ML)
    const uint8_t i2c_expander_addr = 0x20;
    i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x0A, 0x80); // IOCON, set addresing mode to separate banks

    // IC15 GPA
    i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x00, 0x00); // IODIR, bits 0-output, 1-input
    // IC15 GPIOA output values
    uint8_t gpioa = 0;
    gpioa |= (1 << 0); // LMS_RESET, reset active when low
    i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x09, gpioa);

    // IC15 GPB
    uint8_t iodir = 0;
    // inputs
    iodir |= (1 << 7); // PCIE_RESERVED
    iodir |= (1 << 6); // HW_VER1
    iodir |= (1 << 5); // HW_VER0
    iodir |= (1 << 4); // BOM_VER0
    // outputs
    iodir |= (0 << 3); // I2C_SDA_SEL
    iodir |= (0 << 2); // RX_SW3
    iodir |= (0 << 1); // TX_SW
    iodir |= (0 << 0); // RX_SW2
    i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x10, iodir); // IODIR, bits 0-output, 1-input

    // Switching regulators configuration
    {
        // IC15, set I2C_SDA_SEL to direct I2C to LA_I2C_SDA0 (IC26)
        uint8_t gpb = 0;
        gpb |= (0 << 3); // I2C_SDA_SEL: 0-IC26, 1-IC30
        i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x19, gpb);

        // IC26 configuration
        const uint8_t buck_ctrl0 = 0x88; // EN_BUCK, EN_RDIS
        const uint8_t buck_ctrl2 = 0x12; // ILIM (2.5A) SLEW_RATE (10 mV/μs)
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x02, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x03, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x04, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x05, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x06, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x07, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x08, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x09, buck_ctrl2);

        // Set IC26 output voltages
        //i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x0A, 0x39); // Default (0.9V) is OK. Do not touch it.
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x0C, 0x93);  // BUCK1_VSET Set to 1.35V
        //i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x0E, 0xB1); // Default (1.8V) is OK. Do not touch it.
        i2c_write8(LA9310_FSL_I2C1, IC26_ADDR, 0x10, 0xFC);  // BUCK3_VSET Set to 3.3V
    }
    {
        // IC15, set I2C_SDA_SEL to direct I2C to LA_I2C_SDA0 (IC30)
        uint8_t gpb = 0;
        gpb |= (1 << 3); // I2C_SDA_SEL: 0-IC26, 1-IC30
        gpb |= (1 << 2); // RX_SW3
        i2c_write8(LA9310_FSL_I2C1, i2c_expander_addr, 0x19, gpb);

        // IC30 configuration
        const uint8_t buck_ctrl0 = 0x88; // EN_BUCK, EN_RDIS
        const uint8_t buck_ctrl2 = 0x12; // ILIM (2.5A) SLEW_RATE (10 mV/μs)
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x02, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x03, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x04, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x05, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x06, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x07, buck_ctrl2);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x08, buck_ctrl0);
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x09, buck_ctrl2);

        // Set IC30 output voltages
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x0A, 0xA2); // BUCK0_VSET Set to 1.5V
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x0C, 0xAE); // BUCK1_VSET Set to 1.74V
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x0E, 0xBD); // BUCK2_VSET Set to 2.04V
        i2c_write8(LA9310_FSL_I2C1, IC30_ADDR, 0x10, 0xFC); // BUCK3_VSET Set to 3.3V
    }

    log_info("lime i2c config end\n\r");

    // LMS7002 clock config
    log_info("lime spi/lms7002m config start\n\r");

    // Initialize DSPI Handler
    lmsspihandle = pxDspiInit( ( ( 1 << DSPI_CS0 ) ), 4000000 );
    if (lmsspihandle == NULL)
    {
        *(int*)0x41E00200=0xDEADBEEF;
        while((*(int*)0x41E00200)==0xDEADBEEF){};
    }

    struct lms7002m_hooks hooks;
    memset(&hooks, 0, sizeof(hooks));

    hooks.spi16_userData = lmsspihandle;
    hooks.spi16_transact = spi16_transact;

    struct lms7002m_context* rfsoc = lms7002m_create(&hooks);
    lime_Result result = lms7002m_set_frequency_cgen(rfsoc, 4* LA9310_REF_CLK_FREQ );
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_ADCCLKH_CLKGN, 0x0); // FCLKH to ADC
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_CLKH_OV_CLKL_CGEN, 0x2); // divide clock by 4
    lms7002m_spi_modify_csr(rfsoc, LMS7002M_MCLK1SRC, 0x3); // switch MCLK1 clock source to RxTSPCLKA

    const uint16_t defaults[] = {
    // { 0x0082, 0x803E }, // Power down AFE ADCs/DACs
     0x0020, 0xFFFD ,
     0x00A6, 0x000F ,
     0x010A, 0xD54C ,
     0x010C, 0x8865 ,
     0x010D, 0x011A ,
     0x010F, 0x3142 ,
     0x0110, 0x2B14 ,
     0x0111, 0x0000 ,
     0x0112, 0x000C ,
     0x0115, 0x000D ,
     0x0118, 0x418C ,
     0x0119, 0xD28C ,
     0x0120, 0x29DC ,
    };

    for (int i=0; i<sizeof(defaults)/4; ++i)
        lms7002m_spi_write(rfsoc, defaults[i*2], defaults[i*2+1]);

    lms7002m_destroy(rfsoc);
    log_info("lime spi/lms7002m config end\n\r");

    uint16_t xo_dac = ReadXODAC_EEPROM();
    SetXODAC(xo_dac);

    return result != lime_Result_Success;
}
