/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#ifndef __LA9310_HOST_IF_H__
#define __LA9310_HOST_IF_H__

#include <stdint.h>

#define LA9310_MAX_SCRATCH_BUF_SIZE (256 * 1024 * 1024)

/*Common Addresses and offsets*/
/* PCIe outbound window base address for address allocator */
#define PCI_OUTBOUND_WINDOW_BASE_ADDR 0xA0000000
#define LA9310_EP_DMA_BUF_PHYS_ADDR PCI_OUTBOUND_WINDOW_BASE_ADDR
#define LA9310_EP_TOHOST_MSI_PHY_ADDR (LA9310_EP_DMA_BUF_PHYS_ADDR + LA9310_MAX_SCRATCH_BUF_SIZE)

/* Scratch registers for Host <> LA9310 boot hand shake and host interface discovery */
#define LA9310_BOOT_HSHAKE_SCRATCH_REG 1
#define LA9310_SCRATCH_HIF_TABLE_ADDR 3 /* M4 address of the host interface table, struct la9310_hif_entry[] */
#define LA9310_SCRATCH_LEGACY_HIF_SIZE 4 /* sizeof(struct la9310_legacy_hif), for software that predates the table */

/* Events that the M4 signals to the host, identified only by their index. What an event means is agreed between the M4
 * firmware and the host application, the transport below doesn't depend on it.
 *
 * The interface is struct la9310_msi_events, in the M4's TCMU (host BAR2). On startup the M4 firmware initializes it,
 * with every event masked and the MSI configuration read from its PCIe controller, and then publishes its M4 address in
 * scratch register LA9310_SCRATCH_MSI_EVENTS_ADDR. The bootloader clears that register when the M4 firmware is
 * reloaded, so the host has to look the interface up again after a reload (the driver's LA9310_IOCTL_EVENT_REINIT).
 *
 * Every event has an 8-bit counter. Only the M4 writes them, it increments the counter and then sends an MSI:
 *  - if the host enabled N > 1 MSI vectors (MSI Multiple Message Enable), events 1..N-1 are sent on the vector of the
 *    same number,
 *  - event 0 and events >= N are sent on vector LA9310_EVENT_MUX_VECTOR. Its handler finds which of them happened by
 *    comparing the counters with the values it saw last time.
 * The M4 sends MSIs with the cached MSI configuration (msi_*), it doesn't read its PCIe controller again. A host that
 * changes the MSI configuration while the firmware is running (e.g. its driver is reloaded) updates the cache, with
 * every event masked meanwhile.
 * The host owns the mask: the M4 neither counts nor sends the events whose bit is set in it. */
#define LA9310_EVENT_COUNT 16 /* events that the M4 firmware signals */
#define LA9310_EVENT_MAX_COUNT 32 /* the width of the mask */
#define LA9310_EVENT_MUX_VECTOR 0
#define LA9310_SCRATCH_MSI_EVENTS_ADDR 2 /* M4 address of struct la9310_msi_events */

/* The counters are last, so they can grow with the event count */
struct la9310_msi_events {
    uint32_t mask; /* written by the host, events that the M4 doesn't signal */
    uint32_t event_count; /* LA9310_EVENT_COUNT of the firmware */
    uint32_t msi_count; /* total number of MSIs that the M4 sent, for debugging */
    /* MSI configuration cache */
    uint32_t msi_addr; /* M4 address that MSIs are written to: LA9310_EP_TOHOST_MSI_PHY_ADDR window + offset of the
                          host's MSI address in its 4K page, 0 if MSI is disabled */
    uint32_t msi_data; /* message data of vector 0, vector v is sent as msi_data | v */
    uint32_t msi_vector_count; /* vectors that the host enabled, a power of 2 */
    uint8_t counters[LA9310_EVENT_MAX_COUNT]; /* counters[i]: number of times event i happened, modulo 256 */
};

enum la9310_boot_fsm {
    NONE = 0,
    LA9310_HOST_START_CLOCK_CONFIG,
    LA9310_HOST_COMPLETE_CLOCK_CONFIG,
    LA9310_HOST_START_DRIVER_INIT,
};

enum la9310_sw_cmd_status {
    LA9310_SW_CMD_STATUS_FREE,
    LA9310_SW_CMD_STATUS_POSTED,
    LA9310_SW_CMD_STATUS_IN_PROGRESS,
    LA9310_SW_CMD_STATUS_TIMEOUT,
    LA9310_SW_CMD_STATUS_ERROR,
    LA9310_SW_CMD_STATUS_DONE
};

#define LA9310_SW_CMD_DATA_SIZE (32)

struct la9310_sw_cmd_desc {
    uint32_t cmd; /* command id */
    enum la9310_sw_cmd_status status; /* desc status */
    uint32_t data[LA9310_SW_CMD_DATA_SIZE]; /* command specific data */
};

/* Host interface table
 *
 * The M4 firmware describes the pieces of its host interface in a table of {type, address} entries, terminated by an
 * entry of type LA9310_HIF_NONE. Like the VSPA firmware's features map, the table lets the firmware place the pieces
 * wherever it wants, and lets the host find out which of them a firmware provides: the host looks up the entries it
 * needs and treats a missing entry as an unsupported feature.
 *
 * Before the boot handshake reaches LA9310_HOST_START_DRIVER_INIT, the firmware publishes the M4 address of the table
 * in scratch register LA9310_SCRATCH_HIF_TABLE_ADDR. The table's length is given only by its terminating entry, hosts
 * stop after LA9310_HIF_TABLE_MAX_ENTRIES entries. The bootloader clears the register when the M4 firmware is
 * reloaded, so the host has to look the table up again after a reload.
 * Addresses are M4 addresses, in TCML (host BAR1) or TCMU (host BAR2).
 *
 * Entry types are wire IDs: append new ones at the end, never reorder or reuse them. */
enum la9310_hif_type {
    LA9310_HIF_NONE = 0, /* end of table */
    LA9310_HIF_COMMAND = 1, /* struct la9310_sw_cmd_desc, host commands (enum M4_Command) */
    LA9310_HIF_MEMORY_LOG = 2, /* struct MemoryLog, the firmware's memory log */
    LA9310_HIF_TX_TDD_CONFIG = 3, /* tx_tdd_config_t (m4_commands.h), Tx TDD timing, written by the host */
};

struct la9310_hif_entry {
    uint32_t type; /* enum la9310_hif_type */
    uint32_t address; /* M4 address */
};

#define LA9310_HIF_TABLE_MAX_ENTRIES 32 /* upper bound for hosts walking the table */

/* Legacy host interface
 *
 * Firmware and host software that predate the table use struct la9310_legacy_hif at TCML + LA9310_LEGACY_HIF_OFFSET,
 * whose size the firmware writes into scratch register LA9310_SCRATCH_LEGACY_HIF_SIZE. Its legacy firmware also writes
 * LA9310_LEGACY_HIF_OFFSET into scratch register LA9310_SCRATCH_HIF_TABLE_ADDR. Only the parts that are needed to
 * reload the firmware are kept, so that any version of the host software can replace any version of the firmware:
 *  - hif_ver, which hosts check after they have loaded a firmware,
 *  - sw_cmd_desc, for the LIME_M4_HEARTHBEAT and LIME_M4_BOOTLOADER commands. The firmware places its host command
 *    descriptor (LA9310_HIF_COMMAND) here.
 * The rest of the structure is unused. */
#define LA9310_LEGACY_HIF_OFFSET 0x1B000
#define LA9310_LEGACY_HIF_VERSION 0x00010000 /* hif_ver: version 1.0 */

struct la9310_legacy_hif {
    uint32_t ver; /* unused */
    uint32_t hif_ver; /* LA9310_LEGACY_HIF_VERSION */
    uint32_t reserved0[142];
    struct la9310_sw_cmd_desc sw_cmd_desc; /* @ 0x240 */
    uint32_t reserved1[24];
}; /* 808 bytes */

#endif
