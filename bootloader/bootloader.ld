OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(bootloader_reset_handler)

PROVIDE_HIDDEN(STACK_SIZE = 0x100);

MEMORY
{
  fw_image   (R)  : ORIGIN = 0x0, LENGTH = 0x00020000
  m_tcml     (RX) : ORIGIN = 0x1F81FC00, LENGTH = 0x300
  m_dtcm     (RW) : ORIGIN = 0x2000FF00, LENGTH = 0x1000
}

SECTIONS
{
  .romboot_header :
  {
    KEEP(*(.romboot_header))
  } > fw_image

  .text :
  {
    . = ALIGN(0x100);
    /* isr_vector must be aligned to a power of two boundary, with the minimum alignment of 0x100 bytes */
    *(*.bootloader_isrvector)
    *(.text*)
    *(.rodata*)
    *(.init)
    *(.fini)
    PROVIDE_HIDDEN(__bootloader_end = .);
  } > m_tcml AT > fw_image

  .data (NOLOAD) :
  {
    *(*.data)
    . = ALIGN(8);
    . += STACK_SIZE;
    . = ALIGN(8);
    PROVIDE_HIDDEN(__bootloader_stack = .);
  } > m_dtcm

  /* Pointers for boot header */
  PROVIDE_HIDDEN(__bootloader_bin_src_offset = LOADADDR(.text));
  PROVIDE_HIDDEN(__bootloader_bin_size = SIZEOF(.text));
  PROVIDE_HIDDEN(__bootloader_dest = ADDR(.text));
}
