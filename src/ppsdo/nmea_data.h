#ifndef LIME_NMEA_DATA_H
#define LIME_NMEA_DATA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NMEA_GSA_MAX_PRNS 12
#define NMEA_GSV_SATS_PER_MSG 4

// Constellation, derived from the talker ID.
typedef enum nmea_system {
    NMEA_SYSTEM_UNKNOWN = 0,
    NMEA_SYSTEM_GPS, // GP
    NMEA_SYSTEM_GLONASS, // GL
    NMEA_SYSTEM_GALILEO, // GA
    NMEA_SYSTEM_BEIDOU, // GB, BD
    NMEA_SYSTEM_QZSS, // GQ
    NMEA_SYSTEM_COMBINED, // GN, a solution from several constellations
    NMEA_SYSTEM_COUNT
} nmea_system_t;

typedef enum nmea_msg_type {
    NMEA_MSG_UNKNOWN = 0, // valid checksum, but the sentence type is not decoded
    NMEA_MSG_RMC,
    NMEA_MSG_GGA,
    NMEA_MSG_GSA,
    NMEA_MSG_GSV,
    NMEA_MSG_VTG,
    NMEA_MSG_GLL,
    NMEA_MSG_ZDA,
    NMEA_MSG_PMTK_ACK, // $PMTK001
    NMEA_MSG_PMTK_SYS, // $PMTK010
} nmea_msg_type_t;

typedef struct nmea_time {
    uint16_t millisecond;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} nmea_time_t;

typedef struct nmea_date {
    uint16_t year; // full year, e.g. 2026
    uint8_t month; // 1-12
    uint8_t day; // 1-31
} nmea_date_t;

// Degrees scaled by 1e7, positive is north/east.
typedef struct nmea_position {
    int32_t latitude_1e7;
    int32_t longitude_1e7;
} nmea_position_t;

// RMC: recommended minimum data
typedef struct nmea_rmc {
    uint32_t speed_knots_milli; // knots * 1000
    uint32_t course_centideg; // course over ground, degrees * 100
    int32_t magnetic_variation_centideg; // degrees * 100, positive east
    nmea_position_t position;
    nmea_date_t date;
    nmea_time_t time;
    char mode; // 'A' autonomous, 'D' differential, 'E' estimated, 'N' not valid, 0 if absent
    bool active; // status 'A' (data valid), 'V' is a warning
    // Field presence, an empty field is decoded as zeros
    bool time_valid;
    bool date_valid;
    bool position_valid;
    bool speed_valid;
    bool course_valid;
} nmea_rmc_t;

// GGA: fix data
typedef struct nmea_gga {
    int32_t altitude_mm; // above mean sea level
    int32_t geoid_separation_mm;
    nmea_position_t position;
    nmea_time_t time;
    uint16_t hdop_centi; // HDOP * 100, 0 if absent
    uint8_t fix_quality; // 0 invalid, 1 GPS, 2 DGPS, 6 estimated...
    uint8_t satellites_used;
    // Field presence, an empty field is decoded as zeros
    bool time_valid;
    bool position_valid;
    bool altitude_valid; // altitude and geoid separation
} nmea_gga_t;

// GSA: DOP and active satellites
typedef struct nmea_gsa {
    uint16_t prn[NMEA_GSA_MAX_PRNS];
    uint16_t pdop_centi;
    uint16_t hdop_centi;
    uint16_t vdop_centi;
    uint8_t system_id; // NMEA 4.1 GNSS system ID, 0 if absent
    uint8_t fix_type; // 1 none, 2 2D, 3 3D
    uint8_t prn_count;
    char selection_mode; // 'M' manual, 'A' automatic
    bool dop_valid; // PDOP, HDOP and VDOP fields are present
} nmea_gsa_t;

typedef struct nmea_satellite {
    uint16_t prn;
    int16_t azimuth_deg; // -1 if absent
    int8_t elevation_deg; // -1 if absent
    int8_t snr_db; // C/N0 in dB-Hz, -1 if not tracked
} nmea_satellite_t;

// GSV: satellites in view, one sentence of a sequence
typedef struct nmea_gsv {
    uint8_t total_messages;
    uint8_t message_number; // 1 based
    uint8_t satellites_in_view;
    uint8_t satellite_count; // entries used in satellites[]
    nmea_satellite_t satellites[NMEA_GSV_SATS_PER_MSG];
} nmea_gsv_t;

// VTG: course and speed over ground
typedef struct nmea_vtg {
    uint32_t course_true_centideg;
    uint32_t course_magnetic_centideg;
    uint32_t speed_knots_milli;
    uint32_t speed_kmh_milli;
    char mode;
} nmea_vtg_t;

// GLL: geographic position
typedef struct nmea_gll {
    nmea_position_t position;
    nmea_time_t time;
    char mode;
    bool active;
} nmea_gll_t;

// ZDA: UTC date and time
typedef struct nmea_zda {
    nmea_time_t time;
    nmea_date_t date;
    int8_t local_zone_hours;
    uint8_t local_zone_minutes;
    bool time_valid;
    bool date_valid;
} nmea_zda_t;

#endif // LIME_NMEA_DATA_H
