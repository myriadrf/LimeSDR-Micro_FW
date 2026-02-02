#include <stdint.h>
#include "la9310_host_if.h"

// values provided by linker script
extern uint32_t __firmware_bin_size;
extern uint32_t __m_startup_start;
extern uint32_t __firmware_bin_src_offset;

// Linker script will place this data at the start of binary image
struct la9310_boot_header boot_header = {
    .preamble = 0xaa55aa55, // ROM reads I2C offset 0x0 for a valid Boot Preamble
    .plugin_size = 0,
    .plugin_offset = 0,
    .bl_size = (uint32_t)&__firmware_bin_size, // firmware size to be copied
    .bl_src_offset = (uint32_t)&__firmware_bin_src_offset, // I2C memory offset
    .bl_dest = (uint32_t)&__m_startup_start, // absolute 32-bit physical address to which firmware will be copied
    .bl_entry = (uint32_t)&__m_startup_start, // absolute 32-bit physical address of the code entry point in firmware
    .reserved = 0x00010001 // Boot options specified by Host.
};
