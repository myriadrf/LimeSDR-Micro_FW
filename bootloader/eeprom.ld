OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(bootloader_reset_handler)

PROVIDE_HIDDEN(STACK_SIZE = 0x100);

MEMORY
{
  eeprom   (R)  : ORIGIN = 0x0, LENGTH = 64K
  /* m_tcml   (RX) : ORIGIN = 0x1F800000, LENGTH = 128K */

  /*
    Plugin is utilized to perform PCIe configuration.
    Limit it's size so 100kHz I2C could load it and setup in <100ms for PCIe enumeration.
  */
  m_plugin (RX) : ORIGIN = 0x1F800000, LENGTH = 800

  /* place bootloader at the end of TCML, so it would be unlikely to overwrite itself while loading next image.
     TODO: it should be possible to reuse memory load code from BootROM, so there would be no issues with self overwrite.
  */
  m_bootloader  (RX) : ORIGIN = 0x1F820000 - 1K, LENGTH = 1K
  m_dtcm     (RW) : ORIGIN = 0x20000000 + 63K , LENGTH = 1K
}

SECTIONS
{
  .romboot_header :
  {
    KEEP(*(.romboot_header))
  } > eeprom

  /*
     BootROM loads plugin into 0x1F800000, and executes routine at that address. Then continues to load rest of firmware.
  */
  .plugin :
  {
    KEEP(*/pcie_init.c.obj(.text .text*))
  } > m_plugin AT > eeprom

  .text :
  {
    *(.text*)
    *(.rodata*)
    *(.init)
    *(.fini)
  } > m_bootloader AT > eeprom

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
