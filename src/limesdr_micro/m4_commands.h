#ifndef LIME_M4_COMMANDS_H
#define LIME_M4_COMMANDS_H

#include "ppsdo/gps.h"

enum M4_Command {
    // first commands with dedicated timers
    LIME_M4_TX_DAC_ALLOWED,
    LIME_M4_RO0_ADC_ALLOWED,
    LIME_M4_RO1_ADC_ALLOWED,
    LIME_M4_RX0_ADC_ALLOWED,
    LIME_M4_RX1_ADC_ALLOWED,
    LIME_M4_TX_BAND_SWITCH,
    LIME_M4_PA_ENABLE,
    LIME_M4_TX_WINDOW,
    LIME_M4_TIMED_CMD_COUNT = LIME_M4_TX_WINDOW,
    LIME_M4_SCHEDULE_CMD,

    LIME_M4_EMPTY, // 10
    LIME_M4_LMS64C_PACKET,
    LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY, // data[0]: DCS_CLK Hz, data[1]/data[2]: ADC/DAC half rate masks
    LIME_M4_GET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_SET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_BOOTLOADER,
    LIME_M4_HEARTHBEAT,

    LIME_M4_HARDWARE_COUNTER_GET,
    LIME_M4_HARDWARE_COUNTER_RESET,
    LIME_M4_DIGITAL_LOOPBACK,

    LIME_M4_TX_SET_OVERSAMPLE, //20, unused
    LIME_M4_RX_SET_OVERSAMPLE, // unused
    LIME_M4_GET_FEATURES, // unused, the host finds the host interface through its table (la9310_host_if.h)
    LIME_M4_DMA,
    LIME_M4_IQSTREAM_CTRL, // unused
    LIME_M4_RX_CHANNEL_SELECT, // unused
    LIME_M4_RX_GET_OVERSAMPLE, // unused
    LIME_M4_TX_GET_OVERSAMPLE, // unused
    LIME_M4_PPSDO_CTRL,
    LIME_M4_GPS_GET_STATUS,
    LIME_M4_GPS_SEND,
    LIME_M4_PPSDO_GET_STATUS,
    LIME_M4_PPSDO_CONFIG,
    LIME_M4_OTP_GET_LOCK_STATUS // probes the EEPROM identification page lock, takes a few hundred microseconds
};

// Tx TDD timing, same layout as the VSPA's tx_tdd_config_t (VSPA_MMAP_TX_TDD_CONFIG). The M4 publishes its copy in the
// host interface table (LA9310_HIF_TX_TDD_CONFIG) and the host writes it directly; the M4 reads it on each TDD
// start/end request from the VSPA and programs the comparators. Offsets are PHY timer ticks relative to the Tx
// start/end phytime, *_comparator_out are enum ePhyTimerComparatorTrigger.
typedef struct TxTDD_Config {
    int16_t dac_allowed;
    int16_t pa_on : 8;
    int16_t pa_off : 8;
    int16_t rf_sw_on : 8;
    int16_t rf_sw_off : 8;
    uint16_t rf_sw_on_comparator_out : 2;
    uint16_t rf_sw_off_comparator_out : 2;
    uint16_t pa_on_comparator_out : 2;
    uint16_t pa_off_comparator_out : 2;
} tx_tdd_config_t;

struct ppsdo_control_payload {
    uint32_t enable;
};

// LIME_M4_PPSDO_CONFIG request
struct ppsdo_config_payload {
    uint32_t tolerance_ppb; // frequency error tolerated by each measurement window, 0 for firmware defaults
};

// Same values as ppsdo_tune_state_t
enum ppsdo_tune_state_payload {
    PPSDO_TUNE_STATE_COARSE_MIN,
    PPSDO_TUNE_STATE_COARSE_MAX,
    PPSDO_TUNE_STATE_COARSE_DONE,
    PPSDO_TUNE_STATE_FINE,
    PPSDO_TUNE_STATE_IDLE,
};

// LIME_M4_PPSDO_CTRL, LIME_M4_PPSDO_GET_STATUS and LIME_M4_PPSDO_CONFIG response
struct ppsdo_status_payload {
    uint32_t result; // lime_Result
    uint32_t enabled;
    uint32_t tune_state; // ppsdo_tune_state_payload
    uint32_t xo_dac_value;
    int32_t error_1s; // measured minus expected clock ticks of the last completed window
    int32_t error_10s;
    int32_t error_100s;
    int32_t tolerance_1s; // clock ticks
    int32_t tolerance_10s;
    int32_t tolerance_100s;
    uint32_t accuracy; // number of windows (1s, 10s, 100s) in tolerance while fine tuning, 0..3
    uint32_t pps_active;
    uint32_t clock_hz; // frequency of the clock counted between PPS pulses
};

// GPS payloads, sized to fit la9310_sw_cmd_desc.data (128 bytes)
enum gps_link_state_payload {
    GPS_LINK_STATE_IDLE,
    GPS_LINK_STATE_DETECTING,
    GPS_LINK_STATE_FAIL, // no baud rate worked
    GPS_LINK_STATE_RUNNING,
};

// Index into gps_status_payload.data.satellites_in_view
enum gps_system_payload {
    GPS_SYSTEM_UNKNOWN,
    GPS_SYSTEM_GPS,
    GPS_SYSTEM_GLONASS,
    GPS_SYSTEM_GALILEO,
    GPS_SYSTEM_BEIDOU,
    GPS_SYSTEM_QZSS,
    GPS_SYSTEM_COMBINED,
    GPS_SYSTEM_COUNT_PAYLOAD = 8
};

// LIME_M4_GPS_GET_STATUS response
struct gps_status_payload {
    uint32_t result; // lime_Result
    uint32_t link_state; // gps_link_state_payload
    uint32_t baudrate; // detected UART baud rate, 0 if not detected

    gps_data_aggregate_t data;
    gps_stats_t stats;
};

#define GPS_SEND_MAX_TEXT 120
// LIME_M4_GPS_SEND request: sentence body without '$' and checksum, e.g. "PMTK161,0", NUL terminated.
// The response is a uint32_t lime_Result.
struct gps_send_payload {
    char body[GPS_SEND_MAX_TEXT];
};

// LIME_M4_OTP_GET_LOCK_STATUS response
struct otp_lock_status_payload {
    uint32_t result; // lime_Result
    uint32_t locked; // 1 if the OTP (EEPROM identification page) is permanently write protected, valid on success
};

#endif
