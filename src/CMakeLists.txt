CMAKE_MINIMUM_REQUIRED (VERSION 3.18)

include(FeatureSummary)

add_executable(la9310 "main.c")

SET(LA9310_REF_CLK_FREQ "61440000" CACHE STRING "LA9310 post init reference clock")
add_compile_definitions(LA9310_REF_CLK_FREQ=${LA9310_REF_CLK_FREQ})
message(STATUS "LA9310 reference clock: ${LA9310_REF_CLK_FREQ}")

target_include_directories(la9310 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# BOOT MODE SETUP
set(LA9310_BOOT_MODE_OPTIONS "pcie" "i2c")
set(LA9310_BOOT_MODE "pcie" CACHE STRING "LA9310 boot mode")
set_property(CACHE LA9310_BOOT_MODE PROPERTY STRINGS ${LA9310_BOOT_MODE_OPTIONS})
if (${LA9310_BOOT_MODE} STREQUAL "pcie")
    target_compile_definitions(la9310 PUBLIC "-DTURN_ON_HOST_MODE=1")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_CURRENT_LIST_DIR}/platform/ARM_CM4/la9310_host_pcie_mode.ld -static")
    target_sources(la9310 PRIVATE
        "drivers/misc/bbdev_ipc.c"
        "drivers/pcie/la9310_pcie.c"
        "utils/task_stats.c"
    )
elseif(${LA9310_BOOT_MODE} STREQUAL "i2c")
    target_compile_definitions(la9310 PUBLIC "-DTURN_ON_STANDALONE_MODE")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_CURRENT_LIST_DIR}/platform/ARM_CM4/la9310_standalone_i2c_mode.ld -static")
    target_sources(la9310 PRIVATE
        "drivers/avi/la9310_vspa_dma.c"
    )
else()
    message(FATAL_ERROR "LA9310_BOOT_MODE must be one of: ${LA9310_BOOT_MODE_OPTIONS}")
endif()
message(STATUS "LA9310 Boot mode: ${LA9310_BOOT_MODE}")

# Treat warnings as errors
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

set(COMMON_COMPILER_FLAGS "-mcpu=cortex-m4 -mfloat-abi=soft -g -Wall -Wno-packed-bitfield-compat -Wno-address-of-packed-member -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -Os -mapcs")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_COMPILER_FLAGS} -std=gnu99")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_COMPILER_FLAGS} -fno-strict-aliasing -MMD -MP -std=gnu99 -Wno-unused-function -mno-sched-prolog")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COMMON_COMPILER_FLAGS} --specs=nano.specs -lm -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

option(LA9310_COMMAND_LINE "Enable LA9310 command line interface" ON)
option(LMS7002M_CLOCK "Configure LMS7002M clock generator to 153.6MHz" ON)

if(LMS7002M_CLOCK)
    add_subdirectory(limesdr_micro)
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(LA9310_COMMAND_LINE ON)
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
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4
    ${FREERTOS_DIR}/include
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
    sw_cmd_engine
    utils
    drivers/dcs
)

target_sources(la9310 PRIVATE
    "${FREERTOS_DIR}/portable/GCC/ARM_CM4/port.c"
    "${FREERTOS_DIR}/portable/MemMang/heap_2.c"
    "${FREERTOS_DIR}/croutine.c"
    "${FREERTOS_DIR}/event_groups.c"
    "${FREERTOS_DIR}/list.c"
    "${FREERTOS_DIR}/queue.c"
    "${FREERTOS_DIR}/tasks.c"
    "${FREERTOS_DIR}/timers.c"
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
    "sw_cmd_engine/sw_cmd_engine.c"
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
add_feature_info("LMS7002M_CLOCK" LMS7002M_CLOCK "Configure LMS7002M clock output")

#########################################################################
# summary
#########################################################################
message(STATUS "")
message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} enabled features")
message(STATUS "######################################################")
feature_summary(WHAT ENABLED_FEATURES)
message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} disabled features")
message(STATUS "######################################################")
feature_summary(WHAT DISABLED_FEATURES)
