#include "FreeRTOS.h"
#include <semphr.h>
#include "core_cm4.h"

#include "immap.h"
#include "log.h"
#include "immap.h"
#include "fwloader.h"
#include <string.h>
#include "la9310_host_if.h"
#include "la9310_info.h"

extern struct la9310_info g_la9310_info;

void prepare_fwloader()
{
    volatile struct header *boot_header;
    struct la9310_hif * pxHif = g_la9310_info.pHif;
    volatile struct la9310_sw_cmd_desc * pxCmdDesc = &( pxHif->sw_cmd_desc );
    extern uintptr_t __FWL_DEST_ADDR__;
    extern uintptr_t __FWL_LOAD_ADDR__;
    extern uintptr_t __FWL_LOAD_END__;

    // Shut down FreeRTOS
    log_info("Shutdown FreeRTOS\n");
    disable_irq();

    // We still have memset, so let's clear the scratch buffer
    log_info("Clear Header from Scratchbuffer... ");
    boot_header = (struct header *) g_la9310_info.dtcm_addr;
    memset((void *) boot_header, 0, sizeof(struct header));
    boot_header->preamble = 0xc001c0de;
    log_info("Done!\n");

    // Copy bootloader code to its destination
    uint8_t *src = (uint8_t *) &__FWL_LOAD_ADDR__;
    uint8_t *dst = (uint8_t *) &__FWL_DEST_ADDR__;
    uint32_t size = ((void *) &__FWL_LOAD_END__) - ((void *) &__FWL_LOAD_ADDR__);
    log_info("Copy %d bytes firmware from %p to %p loading code to end of RAM... ", size, src, dst);
    memcpy(dst, src, size);
    log_info("Done!\n");

    // Tell the host we are ready for firmware reloading
    pxCmdDesc->status = LA9310_SW_CMD_STATUS_DONE;
    dmb();

    log_info("Ready for reloading the firmware...\n\r");

    // Wait for a firmware image
    while (boot_header->preamble != PREAMBLE)
        dmb();

    log_info("Received new firmware image!\n\r");
}



void fwloader(void) __attribute__((section(".fwloader")));
void fwloader()
{
    volatile struct header *boot_header;
    __attribute__((__noreturn__)) void (*jump_bl_entry)(void) = NULL;
    unsigned int pc_value;

    __asm__ volatile (
            "mov %0, pc"  // Der Befehl speichert den Wert des PC in der Variablen pc_value
            : "=r" (pc_value)  // Output Operand
            );

    log_info("I am in the upper RAM part now: 0x%08x\n", pc_value);

    // We still have memset, so let's clear the scratch buffer
    boot_header = (struct header *) g_la9310_info.dtcm_addr;

    // The image is there, let's load it
    dmb();
    uint32_t src_offset = boot_header->bl_src_offset;
    uint32_t dst_addr = boot_header->bl_dest;
    uint32_t size = boot_header->bl_size;

    log_info("boot_header->bl_src_offset: 0x%08x\n", src_offset);
    log_info("boot_header->bl_dest: 0x%08x\n", dst_addr);
    log_info("boot_header->bl_size: 0x%08x\n", size);
    log_info("boot_header->bl_entry: 0x%08x\n", boot_header->bl_entry);

    // As we most likely will overwrite memcpy we have to do it manually
    uint32_t *dst = (uint32_t *) dst_addr;
    uint32_t *src = (uint32_t *) (src_offset + g_la9310_info.pcie_obound);

    log_info("Copy from 0x%p to 0x%p\n",  src, dst);

    for (uint32_t copied = 0; copied < size; copied += 4)
    {
        *dst++ = *src++;
    }

    boot_header->preamble = 0xaffed00f;

    jump_bl_entry = (void *)(boot_header->bl_entry | M4_THUMB_BIT);

    jump_bl_entry();

    // Control should never reach here
    while (1);
}
