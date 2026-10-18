CMAKE_MINIMUM_REQUIRED (VERSION 3.18)
include(FeatureSummary)

add_compile_options(-Wall)
add_executable(limesdr-micro-m4 "main.c")

target_compile_options(limesdr-micro-m4 PRIVATE
    -fno-strict-aliasing
    -Wno-unused-function
    -mno-sched-prolog
    # -Wno-packed-bitfield-compat
    # -Wno-address-of-packed-member
    -ffreestanding
    -fno-builtin

    -fdata-sections
    -ffunction-sections
)

set(LA9310_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/platform/ARM_CM4/la9310.ld)
set_target_properties(limesdr-micro-m4 PROPERTIES LINK_DEPENDS ${LA9310_LINKER_SCRIPT}) # relinks if script is modified

target_link_options(limesdr-micro-m4 PRIVATE
    -z muldefs
    -Wl,-gc-sections
    -Wl,-print-memory-usage
    # -Wl,--strip-all
    # -Wl,--strip-discarded
    -Wl,--discard-all
    -Wl,--discard-locals
    -T${LA9310_LINKER_SCRIPT}
)
target_link_libraries(limesdr-micro-m4 PUBLIC
    la9310_cm4
    # freertos_kernel # not used
    bootloader
)

# Reference clock setup
SET(EXT_REF_CLK "0" CACHE STRING "External reference clock frequency")
if (${EXT_REF_CLK})
    add_compile_definitions(EXT_REF_CLK_FREQ=${EXT_REF_CLK})
    message(STATUS "Use external reference clock: ${EXT_REF_CLK}")
endif()

target_include_directories(limesdr-micro-m4 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Treat warnings as errors
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

option(LMS7002M_CLOCK "Configure LMS7002M clock generator to 153.6MHz" ON)
option(BOOT_HEADER "Prefix firmware with a boot header" ON)

if (BOOT_HEADER)
    target_sources(limesdr-micro-m4 PRIVATE romboot_header.c)
endif()

if(LMS7002M_CLOCK)
    add_subdirectory(limesdr_micro)
endif()
add_subdirectory(iqstream)
add_subdirectory(ppsdo)

target_include_directories(limesdr-micro-m4 PRIVATE
    platform/ARM_CM4/include/
    platform/boards/la9310/
)

target_sources(limesdr-micro-m4 PRIVATE
    print_scan.c
    m4_memorymap.c
    la9310_sirq.c
    vspa_debug.c
    platform/ARM_CM4/startup.c
    log.c
    platform/ARM_CM4/soc.c
    platform/ARM_CM4/exceptions.c
    platform/boards/la9310/hardware_init.c
)

TARGET_LINK_LIBRARIES(limesdr-micro-m4 PRIVATE -Wl,--start-group)
# Additional libraries

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(limesdr-micro-m4 PRIVATE m c gcc nosys)
TARGET_LINK_LIBRARIES(limesdr-micro-m4 PRIVATE -Wl,--end-group)

# MAP FILE
set(MEMORY_MAP_FILENAME "${PROJECT_BINARY_DIR}/Release/limesdr-micro-m4")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  -Xlinker -Map=${MEMORY_MAP_FILENAME}.map")
set_target_properties(limesdr-micro-m4 PROPERTIES ADDITIONAL_CLEAN_FILES "${MEMORY_MAP_FILENAME}.map")

# BIN AND HEX
# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
# SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/Release)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/Release)

add_custom_target(limesdr-micro-m4-bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary "$<TARGET_FILE:limesdr-micro-m4>" "${MEMORY_MAP_FILENAME}.bin"
    BYPRODUCTS "${MEMORY_MAP_FILENAME}.bin"
    DEPENDS limesdr-micro-m4
    COMMENT "Generating .BIN file"
)

# feature name, status, description
add_feature_info("EXT_REF_CLK" EXT_REF_CLK "Use external reference clock (${EXT_REF_CLK} Hz)")
add_feature_info("LMS7002M_CLOCK" LMS7002M_CLOCK "Provide DCS_CLK from LMS7002M MCLK1 output")
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
