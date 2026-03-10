CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

# firmware reload capability to be put into other firmwares
add_library(bootloader STATIC)
target_sources(bootloader PUBLIC
    bootloader.c
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
    -mgeneral-regs-only # no floats
    -mlittle-endian
    -Wall
    -Os
    -s
)
target_link_options(bootloader PUBLIC
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    -Wl,--strip-all
    -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals
)

target_link_libraries(bootloader PUBLIC la9310_cm4)

# standalone bootloader for initial boot from EEPROM
add_executable(bootloader-standalone romboot_header.c)
target_link_libraries(bootloader-standalone PUBLIC bootloader)
target_link_options(bootloader-standalone PUBLIC
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    -Wl,--strip-all
    -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals
)

set(BOOTLOADER_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/bootloader.ld")
set_target_properties(bootloader-standalone PROPERTIES LINK_DEPENDS ${BOOTLOADER_LINKER_SCRIPT})
set_target_properties(bootloader-standalone PROPERTIES ADDITIONAL_CLEAN_FILES $<TARGET_FILE:bootloader-standalone>.map)
target_link_options(bootloader-standalone PUBLIC
    -T${BOOTLOADER_LINKER_SCRIPT}
    -Wl,-Map=$<TARGET_FILE:bootloader-standalone>.map
)

set(BINARY_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/bootloader-standalone.bin")
add_custom_target(bootloader-bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:bootloader-standalone> ${BINARY_OUTPUT_FILE}
    BYPRODUCTS ${BINARY_OUTPUT_FILE}
    DEPENDS bootloader-standalone
    COMMENT "Generating ${BINARY_OUTPUT_FILE} file"
)
