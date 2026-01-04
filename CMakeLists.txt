CMAKE_MINIMUM_REQUIRED (VERSION 3.18)

# Default to Release build
IF(NOT CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Release)
ENDIF()

project("limesdr-micro-la9310-freertos"
    VERSION 1.0.0
    LANGUAGES C ASM)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

message(STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

# if ARM GCC toolchain is not supplied, automatically download it
if (NOT CMAKE_TOOLCHAIN_FILE)
    include(cmake/toolchain/armgcc.cmake)
endif()

set(COMMON_COMPILER_FLAGS "-mcpu=cortex-m4 -mfloat-abi=soft -g -Wall -Wno-packed-bitfield-compat -Wno-address-of-packed-member -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -Os -mapcs")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_COMPILER_FLAGS} -std=gnu99")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_COMPILER_FLAGS} -fno-strict-aliasing -MMD -MP -std=gnu99 -Wno-unused-function -mno-sched-prolog")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COMMON_COMPILER_FLAGS} --specs=nano.specs -lm -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

set(FREERTOS_DIR "${PROJECT_SOURCE_DIR}/la93xx_freertos/FreeRTOS-Kernel")
message(STATUS "FREERTOS_DIR: ${FREERTOS_DIR}")

set(LA9310_COMMON_HEADERS "${PROJECT_SOURCE_DIR}/la93xx_freertos/common_headers")
message(STATUS "LA9310_COMMON_HEADERS: ${LA9310_COMMON_HEADERS}")

add_subdirectory(src)
add_subdirectory(limesuiteng/embedded)
