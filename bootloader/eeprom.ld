OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(bootloader_reset_handler)

PROVIDE_HIDDEN(STACK_SIZE = 0x100);

MEMORY
{
  fw_image   (R)  : ORIGIN = 0x0, LENGTH = 0x10000
  m_tcml     (RX) : ORIGIN = 0x1F800000, LENGTH = 0x10000

  /* place bootloader at the end of TCML, so it would be unlikely to overwrite itself while loading next image.
     TODO: it should be possible to reuse memory load code from BootROM, so there would be no issues with self overwrite.
  */
  m_tcml_bootloader     (RX) : ORIGIN = 0x1F81FC00, LENGTH = 0x400
  m_dtcm     (RW) : ORIGIN = 0x2000FC00, LENGTH = 0x400
}

SECTIONS
{
  .romboot_header :
  {
    KEEP(*(.romboot_header))
  } > fw_image

  /* PCIe configuration is done within BootROM plugin routine, as I2C runs at 100kHz,
     reading EEPROM and configuring PCIe has to be done within 100ms
  */
  .plugin :
  {
    KEEP(*/pcie_init.c.obj(.text .text*))
    KEEP(*/la9310_gpio.c.obj(.text .text*))
  } > m_tcml AT > fw_image

  .text :
  {
    *(.text*)
    *(.rodata*)
    *(.init)
    *(.fini)
  } > m_tcml_bootloader AT > fw_image

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

  PROVIDE_HIDDEN(__bootloader_plugin_offset = LOADADDR(.plugin));
  PROVIDE_HIDDEN(__bootloader_plugin_size = SIZEOF(.plugin));
}
