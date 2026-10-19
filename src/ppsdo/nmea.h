#ifndef LIME_NMEA_H
#define LIME_NMEA_H

#include "nmea_data.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// NMEA 0183 framing and parsing for the MediaTek based GNSS module (M10578-A3).
// All values are fixed point integers, the M4 is built with a soft float ABI.

// NMEA limits sentences to 82 characters, but some receivers exceed it.
#define NMEA_MAX_SENTENCE_LEN 120
#define NMEA_MAX_FIELDS 32

// PMTK001: command acknowledge
typedef struct nmea_pmtk_ack {
    uint16_t command;
    uint8_t flag; // 0 invalid command, 1 unsupported, 2 valid but failed, 3 succeeded
} nmea_pmtk_ack_t;

// PMTK010: system message
typedef struct nmea_pmtk_sys {
    uint8_t message; // 0 unknown, 1 startup, 2 extension mode notification, 3 normal mode
} nmea_pmtk_sys_t;

typedef struct nmea_message {
    nmea_msg_type_t type;
    nmea_system_t system; // NMEA_SYSTEM_UNKNOWN for proprietary sentences
    union {
        nmea_rmc_t rmc;
        nmea_gga_t gga;
        nmea_gsa_t gsa;
        nmea_gsv_t gsv;
        nmea_vtg_t vtg;
        nmea_gll_t gll;
        nmea_zda_t zda;
        nmea_pmtk_ack_t pmtk_ack;
        nmea_pmtk_sys_t pmtk_sys;
    };
} nmea_message_t;

// Assembles sentences from a byte stream and validates their checksums.
typedef struct nmea_framer {
    uint32_t sentences_ok;
    uint32_t checksum_errors;
    uint32_t framing_errors; // invalid characters, overlong or truncated sentences
    uint16_t length;
    char buffer[NMEA_MAX_SENTENCE_LEN + 1];
    bool in_sentence;
} nmea_framer_t;

void nmea_framer_reset(nmea_framer_t *framer);

// Feeds one received byte. When it completes a sentence with a valid checksum, returns
// the sentence body between '$' and '*', e.g. "GPGGA,...". The pointer is valid until
// the next call. Returns NULL otherwise.
const char *nmea_framer_feed(nmea_framer_t *framer, uint8_t byte);

// XOR of all characters, as used by NMEA checksums.
uint8_t nmea_checksum(const char *data, size_t length);

// Decodes a sentence body returned by nmea_framer_feed().
// Returns false if the sentence type is known but its fields are malformed.
// Unknown sentence types return true with msg->type == NMEA_MSG_UNKNOWN.
bool nmea_parse(const char *sentence, nmea_message_t *msg);

const char *nmea_system_name(nmea_system_t system);

#endif // LIME_NMEA_H
