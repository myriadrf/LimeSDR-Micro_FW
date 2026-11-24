OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(bootloader_reset_handler)

PROVIDE_HIDDEN(STACK_SIZE = 256);

MEMORY
{
  eeprom   (R)  : ORIGIN = 0x0, LENGTH = 64K
  m_tcml   (RX) : ORIGIN = 0x1F800000, LENGTH = 128K
  m_dtcm   (RW) : ORIGIN = 0x20000000 + 1K , LENGTH = 1K
}

NOCROSSREFS_TO(.text .plugin)

SECTIONS
{
  .firmware_header :
  {
    KEEP(*(.romboot_header))
    /* struct firmware_metadata (include/firmware_metadata.h), right after the boot header */
    . = ALIGN(4);
    KEEP(*(.firmware_metadata))
  } > eeprom

  /*
     BootROM loads plugin into 0x1F800000, and executes routine at that address. Then continues to load rest of firmware.
    Plugin is utilized to perform PCIe configuration.
    Limit it's size so 100kHz I2C could load it and setup in <100ms for PCIe enumeration.
  */
  .plugin :
  {
    KEEP(*/pcie_init.c.obj(.text .text*))
  } > m_tcml AT > eeprom
  ASSERT(SIZEOF(.plugin) < 400, "Plugin routine too big. Needs to be read from EEPROM and executed in <100ms for PCIe enumeration.")


  .text :
  {
    __executable_start = .;
    *(.text*)
    *(.rodata*)
    *(.init)
    *(.fini)
  } > m_tcml AT > eeprom

  .data :
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    . = ALIGN(4);
    __executable_end = .;
  } > m_tcml AT > eeprom

  .stack (NOLOAD) :
  {
    *(*.data)
    . = ALIGN(8);
    . += STACK_SIZE;
    . = ALIGN(8);
    PROVIDE_HIDDEN(__bootloader_stack = .);
  } > m_dtcm

  /* Pointers for boot header */
  PROVIDE_HIDDEN(__bootloader_bin_src_offset = LOADADDR(.text));
  PROVIDE_HIDDEN(__bootloader_bin_size = __executable_end - __executable_start);
  PROVIDE_HIDDEN(__bootloader_dest = ADDR(.text));

  PROVIDE_HIDDEN(__bootloader_plugin_offset = LOADADDR(.plugin));
  PROVIDE_HIDDEN(__bootloader_plugin_size = SIZEOF(.plugin));
}
