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

set(LIMESUITENG_ROOT "${PROJECT_SOURCE_DIR}/limesuiteng")
message(STATUS "LIMESUITENG_ROOT: ${LIMESUITENG_ROOT}")

set(FREERTOS_DIR "${PROJECT_SOURCE_DIR}/la93xx_freertos/FreeRTOS-Kernel")
message(STATUS "FREERTOS_DIR: ${FREERTOS_DIR}")

set(LA9310_COMMON_HEADERS "${PROJECT_SOURCE_DIR}/la93xx_freertos/common_headers")
message(STATUS "LA9310_COMMON_HEADERS: ${LA9310_COMMON_HEADERS}")

add_subdirectory(src)