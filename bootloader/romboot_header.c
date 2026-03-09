#include "boot_header.h"

// values provided by linker script
extern uint32_t __bootloader_bin_size;
extern uint32_t __bootloader_bin_src_offset;
extern uint32_t __bootloader_dest;
extern void bootloader_reset_handler(void);

// empty function just to remove linker warnings of them not implemented in arm lib
void _close(void)
{
}
void _lseek(void)
{
}
void _write(void)
{
}
void _read(void)
{
}

// Linker script will place this data at the start of binary image
// needs "used" to be kept by compiler, and KEEP() in linker script
__attribute__((used, section(".romboot_header"))) static const struct la9310_boot_header romboot_header = {
    .preamble = 0xaa55aa55, // ROM reads I2C offset 0x0 for a valid Boot Preamble
    .plugin_size = 0,
    .plugin_offset = 0,
    .bl_size = (uint32_t)&__bootloader_bin_size,
    .bl_src_offset = (uint32_t)&__bootloader_bin_src_offset, // I2C memory offset
    .bl_dest = (uint32_t)&__bootloader_dest, // absolute 32-bit physical address to which firmware will be copied
    .bl_entry = (uint32_t)&bootloader_reset_handler, // absolute 32-bit physical address of the code entry point in firmware
    .flags = BOOTROM_SKIP_BOOT_PLUGIN | BOOTROM_SKIP_EDMA_COPY // Boot options
};
