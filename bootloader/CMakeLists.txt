CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

# firmware reload capability to be put into other firmwares
add_library(bootloader STATIC)
target_sources(bootloader PUBLIC
    bootloader.c
    ../src/drivers/misc/la9310_gpio.c
)

target_compile_options(bootloader PUBLIC
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
    -s
    -mthumb
)

target_link_options(bootloader PRIVATE
    --specs=nosys.specs
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

# TODO: restructure non changing la9310 headers into linkable target
target_include_directories(la9310-eeprom-bootup-pcie PUBLIC
    ${PROJECT_SOURCE_DIR}/src/include
    ${PROJECT_SOURCE_DIR}/include
)

set(EEPROM_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/eeprom.ld")
set_target_properties(la9310-eeprom-bootup-pcie PROPERTIES LINK_DEPENDS ${EEPROM_LINKER_SCRIPT})
set_target_properties(la9310-eeprom-bootup-pcie PROPERTIES ADDITIONAL_CLEAN_FILES $<TARGET_FILE:la9310-eeprom-bootup-pcie>.map)
target_link_options(la9310-eeprom-bootup-pcie PUBLIC
    --specs=nano.specs
    -static
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    -Wl,--strip-all
    -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals

    -T${EEPROM_LINKER_SCRIPT}
    -Wl,-Map=$<TARGET_FILE:la9310-eeprom-bootup-pcie>.map
)

target_link_libraries(la9310-eeprom-bootup-pcie PUBLIC bootloader la9310_cm4)

set(BINARY_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/la9310-eeprom-bootup-pcie.bin")
add_custom_target(la9310-eeprom-bootup-pcie-bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:la9310-eeprom-bootup-pcie> ${BINARY_OUTPUT_FILE}
    BYPRODUCTS ${BINARY_OUTPUT_FILE}
    DEPENDS la9310-eeprom-bootup-pcie
    COMMENT "Generating ${BINARY_OUTPUT_FILE} file"
)
