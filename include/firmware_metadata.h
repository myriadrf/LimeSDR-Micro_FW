#ifndef FIRMWARE_METADATA_H
#define FIRMWARE_METADATA_H

#include <stdint.h>

// Build information placed right after the ROM boot header, for firmware files management.
// Layout: FIRMWARE_METADATA_MAGIC (uint32_t), followed by a list of fields. Each field is a struct
// firmware_metadata_field header followed by its data. next_field_offset is the distance in bytes from the start of
// this field to the start of the next one, so a reader can skip types it doesn't know, and the data length is
// next_field_offset - sizeof(struct firmware_metadata_field) (including any padding). Fields start 4 byte aligned.
// The list ends with a FIRMWARE_METADATA_END field, which has no data and next_field_offset 0.
#define FIRMWARE_METADATA_MAGIC 0x4154454DUL // "META" when read as bytes

// Field types are wire IDs: append new ones, never reuse or renumber
enum firmware_metadata_type {
    FIRMWARE_METADATA_END = 0, // end of list, no data
    FIRMWARE_METADATA_GIT_REVISION = 1, // uint32_t: first 8 hex digits of the commit hash (0 if unknown)
    FIRMWARE_METADATA_PRODUCT_ID = 2, // uint16_t: eLMS_DEV board ID (LimeSuiteNG LMSBoards.h)
    FIRMWARE_METADATA_VERSION = 3, // struct firmware_metadata_version
    FIRMWARE_METADATA_DESCRIPTION = 4, // NUL terminated string
};

struct firmware_metadata_field {
    uint16_t type; // enum firmware_metadata_type
    uint16_t next_field_offset; // from the start of this field, 0 for the last field
    uint8_t data[];
} __attribute__((packed));

// Semantic version plus a revision, major.minor.patch.revision
struct firmware_metadata_version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t revision;
} __attribute__((packed));

#endif // FIRMWARE_METADATA_H
