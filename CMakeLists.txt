CMAKE_MINIMUM_REQUIRED (VERSION 3.18)

project("limesdr-micro-la9310-freertos"
    VERSION 1.0.0
    LANGUAGES C ASM)

# Default to Release build
IF(NOT CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Release)
ENDIF()
message(STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

add_subdirectory(FreeRTOS)
include_directories(include)
add_subdirectory(src)
add_subdirectory(limesuiteng/embedded)
