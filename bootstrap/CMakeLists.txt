CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

add_executable(bootstrap)

target_sources(bootstrap PUBLIC
    boot_strapper.c
    start.S
    log.c
    utils.c
    i2c/i2c.c
)

target_compile_options(bootstrap PUBLIC
    -mlittle-endian
    -fno-zero-initialized-in-bss
    -Wall
    -Wstrict-prototypes
    -Werror
    -Os
    -g
    -fpic
    -nostdlib
    -fno-builtin
)

target_include_directories(bootstrap PRIVATE include i2c)

set(BOOTSTRAP_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/boot_strapper.ld")
set_target_properties(bootstrap PROPERTIES LINK_DEPENDS ${BOOTSTRAP_LINKER_SCRIPT})

target_link_options(bootstrap PUBLIC
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    -T${BOOTSTRAP_LINKER_SCRIPT}
    -Wl,-Map=bootstrap.map
)

set(BINARY_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/bootstrap.bin")
add_custom_target(bootstrap_bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:bootstrap> ${BINARY_OUTPUT_FILE}
    BYPRODUCTS ${BINARY_OUTPUT_FILE}
    DEPENDS bootstrap
    COMMENT "Generating ${BINARY_OUTPUT_FILE} file"
)
