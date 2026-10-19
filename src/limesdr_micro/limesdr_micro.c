#include "la9310_i2cAPI.h"
#include "fsl_dspi.h"

#include "io.h"
#include "log.h"
#include "immap.h"
#include "la9310.h"
#include "serial_ns16550.h"
#include "la9310_dcs_api.h"
#include "ppsdo/ppsdo.h"
#include "ppsdo/gps.h"
#include "phytimer.h"
#include "la9310_host_if.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "lms7002m/spi.h"
#include "lms7002m/csr_data.h"

#include <string.h>

#define LMS7002M_SPI_SPEED_HZ 4000000
const uint8_t i2c_expander_addr = 0x20;

la9310_dspi_t g_dspi;
struct lms7002m_context *rfsoc = NULL;

extern uint16_t ReadXODAC_EEPROM();
extern void SetXODAC(uint16_t value);

uint8_t hardware_version = 0;

// ARM bare metal does not have usleep(), it's used within lms7002m procedures
void usleep(int microSeconds)
{
    vUDelay(microSeconds);
}

static int i2c_write8(int module, uint32_t i2c_moduleAddress, uint8_t addr, uint8_t value)
{
    return iLa9310_I2C_Write(module, i2c_moduleAddress, addr, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &value, 1);
}

int32_t spi_lms7002m_write(la9310_dspi_t *dspi, uint16_t addr, uint16_t value)
{
    int32_t j = 0;
    uint8_t ucData[4];

    const uint32_t data = (1 << 31) | (addr << 16) | (value);
    // MSB goes first
    for (int i = 3; i >= 0; i--)
    {
        ucData[j] = (data >> (i * 8)) & 0xFF;
        j++;
    }

    int32_t iRet = lDspiPush(dspi, DSPI_CS0, DSPI_DEV_WRITE, ucData, 4);
    if (0 > iRet)
        log_err("lDspiPush failed, error[%d]" LOG_EOL, iRet);
    return iRet;
}

int32_t spi_lms7002m_read(la9310_dspi_t *dspi, uint16_t addr, uint16_t *value)
{
    uint8_t ucData[4];

    /* Push address to be read */
    ucData[1] = addr;
    ucData[0] = (addr >> 8);
    int32_t iRet = lDspiPush(dspi, DSPI_CS0, DSPI_DEV_READ, ucData, 4);

    if (0 > iRet)
    {
        *value = 0xDEAD;
        log_err("lDspiPush failed, error[%d]" LOG_EOL, iRet);
        return iRet;
    }

    iRet = lDspiPop(dspi, ucData, 4);
    if (0 > iRet)
    {
        *value = 0xBEEF;
        log_err("lDspiPop failed, error[%d]" LOG_EOL, iRet);
        return iRet;
    }
    // index 0 should be ignored, this is outcome of above push operation
    *value = (ucData[2] << 8) | (ucData[3]);
    return iRet;
}

static int spi16_transact(const uint32_t *mosi, uint32_t *miso, uint32_t count, void *userData)
{
    struct LA931xDspiInstance *pDspiHandle = userData;
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

#define MCP23017_OLATA 0x14
int IsExternalRefClkUsed()
{
    uint8_t gpioa = 0;
    iLa9310_I2C_Read(LA9310_FSL_I2C1, i2c_expander_addr, MCP23017_OLATA, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &gpioa, 1);
    return (gpioa & (1 << 5)) ? 1 : 0;
}

void UseExternalReferenceClock(bool external)
{
    uint8_t gpioa = 0;
    iLa9310_I2C_Read(LA9310_FSL_I2C1, i2c_expander_addr, MCP23017_OLATA, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &gpioa, 1);
    gpioa &= ~(1 << 5);
    gpioa |= (external ? 1 : 0) << 5;
    iLa9310_I2C_Write(LA9310_FSL_I2C1, i2c_expander_addr, MCP23017_OLATA, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &gpioa, 1);
}

static void initialize_power_regulators(uint32_t i2c_handle)
{
    const uint16_t dcconv_addr = 0x60; // LP8758A2E3YFFR DCDC Converter
    // Switching regulators configuration
    // const uint8_t buck_ctrl0 = 0x88; // EN_BUCK, EN_RDIS
    // const uint8_t buck_ctrl2 = 0x12; // ILIM (2.5A) SLEW_RATE (10 mV/μs)
    const uint8_t buck_ctrl[8] = { 0x88, 0x12, 0x88, 0x12, 0x88, 0x12, 0x88, 0x12 };
    {
        // set I2C_SDA_SEL to direct I2C to LA_I2C_SDA0
        uint8_t gpio_b = 0;
        gpio_b |= (0 << 3); // I2C_SDA_SEL
        i2c_write8(i2c_handle, i2c_expander_addr, 0x13, gpio_b);
        iLa9310_I2C_Write(i2c_handle, dcconv_addr, 0x02, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, buck_ctrl, 8);
        // Set output voltages
        //i2c_write8(i2c_handle, dcconv_addr, 0x0A, 0x39); // Default (0.9V) is OK. Do not touch it.
        i2c_write8(i2c_handle, dcconv_addr, 0x0C, 0x93); // BUCK1_VSET Set to 1.35V
        //i2c_write8(i2c_handle, dcconv_addr, 0x0E, 0xB1); // Default (1.8V) is OK. Do not touch it.
        i2c_write8(i2c_handle, dcconv_addr, 0x10, 0xFC); // BUCK3_VSET Set to 3.3V
    }
    {
        // set I2C_SDA_SEL to direct I2C to LA_I2C_SDA1
        uint8_t gpio_b = 0;
        gpio_b |= (1 << 3); // I2C_SDA_SEL
        gpio_b |= (1 << 2); // RX_SW3
        i2c_write8(i2c_handle, i2c_expander_addr, 0x13, gpio_b);
        iLa9310_I2C_Write(i2c_handle, dcconv_addr, 0x02, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, buck_ctrl, 8);

        // Set output voltages
        i2c_write8(i2c_handle, dcconv_addr, 0x0A, 0xA2); // BUCK0_VSET Set to 1.5V
        i2c_write8(i2c_handle, dcconv_addr, 0x0C, 0xAE); // BUCK1_VSET Set to 1.74V
        i2c_write8(i2c_handle, dcconv_addr, 0x0E, 0xBD); // BUCK2_VSET Set to 2.04V
        i2c_write8(i2c_handle, dcconv_addr, 0x10, 0xFC); // BUCK3_VSET Set to 3.3V
    }
}

static void initialize_i2c_expander(uint32_t i2c_handle)
{
    // I2C I/O expander (IC15, MCP23017-E/ML)
    const uint8_t i2c_expander_addr = 0x20;

    // IC15 GPIOA output values
    {
        uint8_t gpio_a = 0;
        gpio_a |= (1 << 0); // LMS_RESET, reset active when low
        gpio_a |= (1 << 6); // SMB_CLK_OUT, disable ref clock output to mPCIe SMBus CLK
        // First set output values, and only then configure IODIR, to avoid connecting
        // Ref clk to SMBus CLK, as that can affect host system stability (freeze/reboot)
        i2c_write8(i2c_handle, i2c_expander_addr, 0x12, gpio_a);
    }
    {
        uint8_t iodir_a = 0; // IODIR, bits 0-output, 1-input
        iodir_a |= (1 << 4); // GNSS_FIX
        i2c_write8(i2c_handle, i2c_expander_addr, 0x00, iodir_a);
    }

    // IC15 GPB
    {
        uint8_t iodir_b = 0; // IODIR, bits 0-output, 1-input
        // inputs
        iodir_b |= (1 << 7); // PCIE_RESERVED
        iodir_b |= (1 << 6); // HW_VER1
        iodir_b |= (1 << 5); // HW_VER0
        iodir_b |= (1 << 4); // BOM_VER0
        // outputs
        iodir_b |= (0 << 3); // I2C_SDA_SEL
        iodir_b |= (0 << 2); // RX_SW3
        iodir_b |= (0 << 1); // TX_SW
        iodir_b |= (0 << 0); // RX_SW2
        i2c_write8(i2c_handle, i2c_expander_addr, 0x01, iodir_b);

        // pull-ups
        i2c_write8(i2c_handle, i2c_expander_addr, 0x0D, 0x70);
    }
}

lime_Result limesdr_micro_set_dcs_clk(struct la9310_hif *hif, uint32_t dcs_clk_hz)
{
    if (dcs_clk_hz == 0)
        return lime_Result_InvalidValue;

    // No clear limits is known, but going lower introduces a chance that VSPA DMA might
    // get stuck, with no recovery except power cycle.
    if (dcs_clk_hz < 20e6)
        return lime_Result_OutOfRange;

    // Let pending UART TX finish at the current baud rate before the clock changes
    vSerialWaitTxEmpty((void *)UART_BASEADDR, UART_CLOCK_IN_FROM_PLATFORM(g_la9310_current_platform_clk_hz));

    const uint32_t platform_clock = dcs_clk_hz * 4;
    // DCS_CLK=MCLK1, MCLK1 output is CGEN/4, LA9310 platform clock is DCS_CLK*4
    lime_Result result = lms7002m_set_frequency_cgen(rfsoc, platform_clock);
    if (result != lime_Result_Success)
        return result;
    g_la9310_current_platform_clk_hz = platform_clock;
    ppsdo_set_clock_frequency(&ppsdo, PHYTIMER_CLOCK_IN_FROM_PLATFORM(platform_clock));

    // UART bus
    gps_set_uart_clock(UART_CLOCK_IN_FROM_PLATFORM(platform_clock));
    // I2C bus
    iLa9310_I2C_Init(LA9310_FSL_I2C1, I2C_CLOCK_IN_FROM_PLATFORM(platform_clock), LA9310_I2C_FREQ);
    // SPI bus
    vDspiClkSet(&g_dspi, SPI_CLOCK_IN_FROM_PLATFORM(platform_clock), LMS7002M_SPI_SPEED_HZ);

    // ADC/DAC sample rate
    vDcsInit(IN_32(&hif->adc_mask), IN_32(&hif->adc_rate_mask), IN_32(&hif->dac_mask), IN_32(&hif->dac_rate_mask));
    return result;
}

void limesdr_micro_initialize(void)
{
    // Enable LMS7002M transmitter, PHYTimer 19, controls LMS_TXEN pin
    vPhyTimerComparatorForce(19, ePhyTimerComparatorOut1);

    struct lms7002m_hooks hooks;
    memset(&hooks, 0, sizeof(hooks));

    hooks.spi16_userData = &g_dspi;
    hooks.spi16_transact = spi16_transact;

    rfsoc = lms7002m_create(&hooks);

    // If this is not the first firmware run, the clocks can be already changed from PCIe ref clk.
    if (la9310_clk_src_pcie())
    {
        // all initial clocks state is known.
        log_info("ColdStart, platform clk %u" LOG_EOL, PRE_BOOT_PLATFORM_CLK_HZ);
        iLa9310_I2C_Init(LA9310_FSL_I2C1, I2C_CLOCK_IN_FROM_PLATFORM(PRE_BOOT_PLATFORM_CLK_HZ), LA9310_I2C_FREQ);
        initialize_i2c_expander(LA9310_FSL_I2C1);
        initialize_power_regulators(LA9310_FSL_I2C1);

        DspiInit(&g_dspi, ((1 << DSPI_CS0)), SPI_CLOCK_IN_FROM_PLATFORM(PRE_BOOT_PLATFORM_CLK_HZ), LMS7002M_SPI_SPEED_HZ);

#ifdef EXT_REF_CLK_FREQ
        log_info("Ext. Ref. Clk %u" LOG_EOL, EXT_REF_CLK_FREQ);
        lms7002m_set_reference_clock(rfsoc, EXT_REF_CLK_FREQ);
        UseExternalReferenceClock(true);
#else
        UseExternalReferenceClock(false);
#endif
        log_info("Config MCLK1/DCS_CLK %u" LOG_EOL, LA9310_DCS_CLK_HZ);
        lime_Result result = lms7002m_set_frequency_cgen(rfsoc, 4 * LA9310_DCS_CLK_HZ);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_ADCCLKH_CLKGN, 0x0); // FCLKH to ADC
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_CLKH_OV_CLKL_CGEN, 0x2); // divide clock by 4
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MCLK1SRC, 0x3); // switch MCLK1 clock source to RxTSPCLKA
        if (result != lime_Result_Success)
            log_err("MCLK1 fail", LOG_EOL);
    }
    else
    {
        log_info("WarmStart, already on unknown DCS_CLK" LOG_EOL);
        // unknown what current DCS_CLK frequency was left over.
        // Can use any SPI speed to readback DCS_CLK, assume platform is at max freq.
        DspiInit(&g_dspi, ((1 << DSPI_CS0)), SPI_CLOCK_IN_FROM_PLATFORM(640000000), LMS7002M_SPI_SPEED_HZ);

#ifdef EXT_REF_CLK_FREQ
        log_info("Ext. Ref. Clk %u" LOG_EOL, EXT_REF_CLK_FREQ);
        lms7002m_set_reference_clock(rfsoc, EXT_REF_CLK_FREQ);
        UseExternalReferenceClock(true);
#else
        UseExternalReferenceClock(false);
#endif

        const uint32_t cgen_freq = lms7002m_get_frequency_cgen(rfsoc); // CGEN freq is same as LA9310 platform
        g_la9310_current_platform_clk_hz = cgen_freq;
        log_info("Current DCS_CLK:%u" LOG_EOL, g_la9310_current_platform_clk_hz / 4);

        // set comms to known speeds
        DspiInit(&g_dspi, ((1 << DSPI_CS0)), SPI_CLOCK_IN_FROM_PLATFORM(g_la9310_current_platform_clk_hz), LMS7002M_SPI_SPEED_HZ);

        // no need to reinitialize power regulators, they are not modified after initial setup, so should be good from previous run
        log_info("Config DCS_CLK %u" LOG_EOL, LA9310_DCS_CLK_HZ);
        lime_Result result = lms7002m_set_frequency_cgen(rfsoc, 4 * LA9310_DCS_CLK_HZ);
        if (result != lime_Result_Success)
            log_err("MCLK1 fail", LOG_EOL);
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_EN_ADCCLKH_CLKGN, 0x0); // FCLKH to ADC
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_CLKH_OV_CLKL_CGEN, 0x2); // divide clock by 4
        lms7002m_spi_modify_csr(rfsoc, LMS7002M_MCLK1SRC, 0x3); // switch MCLK1 clock source to RxTSPCLKA

        DspiInit(&g_dspi, ((1 << DSPI_CS0)), SPI_CLOCK_IN_FROM_PLATFORM(g_la9310_current_platform_clk_hz), LMS7002M_SPI_SPEED_HZ);

        iLa9310_I2C_Init(LA9310_FSL_I2C1, I2C_CLOCK_IN_FROM_PLATFORM(g_la9310_current_platform_clk_hz), LA9310_I2C_FREQ);
        initialize_i2c_expander(LA9310_FSL_I2C1);
    }

    iLa9310_I2C_Read(LA9310_FSL_I2C1, i2c_expander_addr, 0x13, LA9310_I2C_DEV_OFFSET_LEN_1_BYTE, &hardware_version, 1);
    hardware_version = (hardware_version & 0x60) >> 5;

    // clang-format off
    const uint16_t defaults[] = {
    // { 0x0082, 0x803E }, // Power down AFE ADCs/DACs
        0x0020, 0xFFFD,
        0x00A6, 0x000F,
        0x010A, 0xD54C,
        0x010C, 0x8865,
        0x010D, 0x011A,
        0x010F, 0x3142,
        0x0110, 0x2B14,
        0x0111, 0x0000,
        0x0112, 0x000C,
        0x0115, 0x000D,
        0x0118, 0x418C,
        0x0119, 0xD28C,
        0x0120, 0x29DC,
    };
    // clang-format on

    for (int i = 0; i < sizeof(defaults) / 4; ++i)
        lms7002m_spi_write(rfsoc, defaults[i * 2], defaults[i * 2 + 1]);

    const uint16_t xo_dac = ReadXODAC_EEPROM();
    SetXODAC(xo_dac);
    log_info("ClockInit done" LOG_EOL);
}
