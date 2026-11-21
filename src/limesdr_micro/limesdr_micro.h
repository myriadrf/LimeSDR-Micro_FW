#ifndef LIMESDR_MICRO_H
#define LIMESDR_MICRO_H

#include "la9310_dcs_api.h"

#include <stdint.h>

// ADCs and DAC that are in use (vDcsInit() masks)
#define LA9310_ADC_MASK 0xF
#define LA9310_DAC_MASK 0x1
// Highest DCS_CLK at which the ADCs and the DAC run at half of DCS_CLK by default, above it they run at DCS_CLK.
// The host chooses the rates itself with LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY.
#define LA9310_DCS_HALF_RATE_MAX_HZ 80000000

/// Powers up the ADCs and the DAC in use, at the default converter rate for dcs_clk_hz (LA9310_DCS_HALF_RATE_MAX_HZ).
static inline void limesdr_micro_dcs_init(uint32_t dcs_clk_hz)
{
    const int half_rate = dcs_clk_hz <= LA9310_DCS_HALF_RATE_MAX_HZ;
    vDcsInit(LA9310_ADC_MASK, half_rate ? LA9310_ADC_MASK : 0, LA9310_DAC_MASK, half_rate ? LA9310_DAC_MASK : 0);
}

int initialize_lms7002m_clock_generator();
int LMS64C_protocol_init(void);

extern uint8_t hardware_version;

int limesdr_micro_initialize(void);
/// @param adc_rate_mask, dac_rate_mask Converters that run at half of DCS_CLK, a set bit selects half rate.
int limesdr_micro_set_dcs_clk(uint32_t dcs_clk_hz, uint32_t adc_rate_mask, uint32_t dac_rate_mask);

#endif