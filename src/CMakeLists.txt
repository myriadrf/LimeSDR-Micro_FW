CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

add_compile_options(-g -Wall)
add_executable(la9310 "main.c")

target_compile_options(la9310 PUBLIC
    -fno-strict-aliasing
    -Wno-unused-function
    -mno-sched-prolog
    -Wno-packed-bitfield-compat
    -Wno-address-of-packed-member
    -ffreestanding
    -fno-builtin
    -Os
)

target_link_options(la9310 PUBLIC
    --specs=nano.specs
    -z muldefs
    -Wl,-gc-sections
    -Wl,-print-memory-usage
)
target_link_libraries(la9310 PUBLIC freertos_kernel)

# Reference clock setup
SET(EXT_REF_CLK "0" CACHE STRING "External reference clock frequency")
if (${EXT_REF_CLK})
    add_compile_definitions(EXT_REF_CLK_FREQ=${EXT_REF_CLK})
    message(STATUS "Use external reference clock: ${EXT_REF_CLK}")
endif()

SET(LA9310_SYS_CLK_FREQ "61440000" CACHE STRING "LA9310 post init system clock frequency")
add_compile_definitions(LA9310_SYS_CLK_FREQ=${LA9310_SYS_CLK_FREQ})
message(STATUS "LA9310 system clock: ${LA9310_SYS_CLK_FREQ}")

target_include_directories(la9310 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# BOOT MODE SETUP
set(LA9310_BOOT_MODE_OPTIONS "pcie" "i2c")
set(LA9310_BOOT_MODE "pcie" CACHE STRING "LA9310 boot mode")
set_property(CACHE LA9310_BOOT_MODE PROPERTY STRINGS ${LA9310_BOOT_MODE_OPTIONS})
if (${LA9310_BOOT_MODE} STREQUAL "pcie")
    target_compile_definitions(la9310 PUBLIC "-DTURN_ON_HOST_MODE=1")
    set(LA9310_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/platform/ARM_CM4/la9310_host_pcie_mode.ld)
    target_sources(la9310 PRIVATE
        # "drivers/misc/bbdev_ipc.c"
        "drivers/pcie/la9310_pcie.c"
        "utils/task_stats.c"
    )
elseif(${LA9310_BOOT_MODE} STREQUAL "i2c")
    target_compile_definitions(la9310 PUBLIC "-DTURN_ON_STANDALONE_MODE")
    set(LA9310_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/platform/ARM_CM4/la9310_standalone_i2c_mode.ld)
    target_sources(la9310 PRIVATE
        "drivers/avi/la9310_vspa_dma.c"
    )
else()
    message(FATAL_ERROR "LA9310_BOOT_MODE must be one of: ${LA9310_BOOT_MODE_OPTIONS}")
endif()
set_target_properties(la9310 PROPERTIES LINK_DEPENDS ${LA9310_LINKER_SCRIPT})
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LA9310_LINKER_SCRIPT} -static")
message(STATUS "LA9310 Boot mode: ${LA9310_BOOT_MODE}")

# Treat warnings as errors
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

option(LA9310_COMMAND_LINE "Enable LA9310 command line interface" OFF)
option(LMS7002M_CLOCK "Configure LMS7002M clock generator to 153.6MHz" ON)
option(BOOT_HEADER "Prefix firmware with a boot header" OFF)

if (BOOT_HEADER)
    target_sources(la9310 PRIVATE bootloader_header.c)
endif()

if(LMS7002M_CLOCK)
    add_subdirectory(limesdr_micro)
endif()

if(LA9310_COMMAND_LINE)
    target_compile_definitions(la9310 PUBLIC "-DLA9310_ENABLE_COMMAND_LINE")
    target_include_directories(la9310 PRIVATE utils/FreeRTOS-Plus-CLI)
    target_sources(la9310 PRIVATE
        "utils/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c"
        "utils/UARTCommandConsole.c"
        "drivers/timesync/sync_timing_device_cli.c"
    )
endif()

target_include_directories(la9310 PRIVATE
    ${LA9310_COMMON_HEADERS}/
    include
    platform/ARM_CM4/include/
    platform/boards/la9310/
    drivers/misc/
    drivers/edma/
    drivers/watchdog/
    drivers/i2c/
    drivers/dspi
    drivers/phy_timer
    drivers/timesync
    utils
    drivers/dcs
)

target_sources(la9310 PRIVATE
    "log.c"
    "drivers/serial/debug_console.c"
    "drivers/serial/print_scan.c"
    "drivers/serial/serial_ns16550.c"
    "drivers/edma/la9310_edma.c"
    "drivers/misc/mem_log.c"
    "drivers/watchdog/la9310_watchdog.c"
    "platform/ARM_CM4/soc.c"
    "platform/ARM_CM4/start.S"
    "platform/boards/la9310/board.c"
    "platform/ARM_CM4/exceptions.c"
    "platform/boards/la9310/hardware_init.c"
    "la9310_irq.c"
    "drivers/avi/la9310_avi.c"
    "drivers/misc/la9310_gpio.c"
    "drivers/misc/la9310_pinmux.c"
    "drivers/i2c/la9310_i2c.c"
    "drivers/dspi/fsl_dspi.c"
    "utils/delay.c"
    "drivers/phy_timer/phytimer.c"
    "drivers/timesync/sync_timing_device.c"
    "drivers/dspi/dspi_test.c"
    "drivers/dcs/la9310_dcs.c"
)

TARGET_LINK_LIBRARIES(la9310 PRIVATE -Wl,--start-group)
# Additional libraries

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(la9310 PRIVATE m c gcc nosys)
TARGET_LINK_LIBRARIES(la9310 PRIVATE -Wl,--end-group)

# MAP FILE
set(MEMORY_MAP_FILENAME "${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/la9310")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  -Xlinker -Map=${MEMORY_MAP_FILENAME}.map")
set_target_properties(la9310 PROPERTIES ADDITIONAL_CLEAN_FILES "${MEMORY_MAP_FILENAME}.map")

# BIN AND HEX
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})

add_custom_target(la9310_hex ALL
    COMMAND ${CMAKE_OBJCOPY} -Oihex "$<TARGET_FILE:la9310>" "${MEMORY_MAP_FILENAME}.hex"
    BYPRODUCTS "${MEMORY_MAP_FILENAME}.hex"
    DEPENDS la9310
    COMMENT "Generating .HEX file"
)
add_custom_target(la9310_bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary "$<TARGET_FILE:la9310>" "${MEMORY_MAP_FILENAME}.bin"
    BYPRODUCTS "${MEMORY_MAP_FILENAME}.bin"
    DEPENDS la9310
    COMMENT "Generating .BIN file"
)

# feature name, status, description
add_feature_info("LA9310_COMMAND_LINE" LA9310_COMMAND_LINE "Enable LA9310 command line interface")
add_feature_info("EXT_REF_CLK" EXT_REF_CLK "Use external reference clock (${EXT_REF_CLK} Hz)")
add_feature_info("LMS7002M_CLOCK" LMS7002M_CLOCK "Configure LMS7002M MCLK1 output (${LA9310_SYS_CLK_FREQ} Hz)")
add_feature_info("BOOT_HEADER" BOOT_HEADER "Prefix firmware with a boot header")

#########################################################################
# summary
#########################################################################
message(STATUS "")
message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} disabled features")
message(STATUS "######################################################")
feature_summary(WHAT DISABLED_FEATURES)
message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} enabled features")
message(STATUS "######################################################")
feature_summary(WHAT ENABLED_FEATURES)
