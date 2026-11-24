CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

# firmware reload capability to be put into other firmwares
add_library(bootloader STATIC)
target_sources(bootloader PUBLIC
    bootloader.c
)

target_compile_options(bootloader PRIVATE
    -nostartfiles
    -nolibc
    -nostdlib
    -fno-builtin
    -fno-common
    -ffreestanding
    -fno-zero-initialized-in-bss
    -fdata-sections
    -ffunction-sections
    -mlittle-endian
    -Wall
    -Os
)

target_link_options(bootloader PRIVATE
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    -Wl,--strip-all
    -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals
)

target_link_libraries(bootloader PUBLIC la9310_cm4)

# standalone bootloader for initial boot from EEPROM that configures PCIe descriptors
# and switches to further boot from host memory
add_executable(la9310-eeprom-bootup-pcie
    eeprom_header.c
    pcie_init.c
)

target_compile_options(la9310-eeprom-bootup-pcie PRIVATE
    -g
    -ggdb
    -nostartfiles
    -nolibc
    -nostdlib
    -fno-builtin
    -fno-common
    -ffreestanding
    -fno-zero-initialized-in-bss
    -fdata-sections
    -ffunction-sections
    -mlittle-endian
    -Wall
    -mthumb
    -Os
)

set(EEPROM_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/eeprom.ld")
set_target_properties(la9310-eeprom-bootup-pcie PROPERTIES LINK_DEPENDS ${EEPROM_LINKER_SCRIPT})
set_target_properties(la9310-eeprom-bootup-pcie PROPERTIES ADDITIONAL_CLEAN_FILES $<TARGET_FILE:la9310-eeprom-bootup-pcie>.map)
target_link_options(la9310-eeprom-bootup-pcie PRIVATE
    -static
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    # -Wl,--strip-all
    # -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals

    -T${EEPROM_LINKER_SCRIPT}
    -Wl,-Map=$<TARGET_FILE:la9310-eeprom-bootup-pcie>.map
)

target_link_libraries(la9310-eeprom-bootup-pcie PUBLIC bootloader la9310_cm4)
target_firmware_metadata(la9310-eeprom-bootup-pcie
    PRODUCT_ID ${FIRMWARE_PRODUCT_ID}
    VERSION 1.0.0.0
    GIT_REVISION
    DESCRIPTION "Lime LA9310 PCIe bootloader"
)

set(BINARY_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/la9310-eeprom-bootup-pcie.bin")
add_custom_target(la9310-eeprom-bootup-pcie-bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:la9310-eeprom-bootup-pcie> ${BINARY_OUTPUT_FILE}
    BYPRODUCTS ${BINARY_OUTPUT_FILE}
    DEPENDS la9310-eeprom-bootup-pcie
    COMMENT "Generating ${BINARY_OUTPUT_FILE} file"
)
