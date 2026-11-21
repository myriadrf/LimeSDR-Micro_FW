CMAKE_MINIMUM_REQUIRED (VERSION 3.18)

project("limesdr-micro-la9310-freertos"
    VERSION 1.0.0
    LANGUAGES C ASM)

# Default to Release build
IF(NOT CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Release)
ENDIF()
message(STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

add_compile_options(
    # -Os
    -fdata-sections
    -ffunction-sections
    -fno-common
    -ggdb3
    # -ggdb
    # debug info names files relative to a fixed LimeSDR-Micro_FW/ root,
    # independent of where the tree is checked out and built
    -fdebug-prefix-map=${CMAKE_SOURCE_DIR}/=LimeSDR-Micro_FW/
    -fdebug-prefix-map=${CMAKE_BINARY_DIR}/=LimeSDR-Micro_FW/build/
)

add_link_options(--specs=nano.specs)

# Build time log level for a logging domain: messages above it are compiled
# out, together with their strings. Values match LOG_LEVEL_* in la9310/log_core.h.
set(LOG_LEVEL_NAMES NONE ERR WARN INFO DBG ISR ALL)
function(add_log_level_option cache_var description target define default_level)
    set(${cache_var} "${default_level}" CACHE STRING "${description} (${LOG_LEVEL_NAMES})")
    set_property(CACHE ${cache_var} PROPERTY STRINGS ${LOG_LEVEL_NAMES})
    string(TOUPPER "${${cache_var}}" level_name)
    list(FIND LOG_LEVEL_NAMES "${level_name}" level)
    if(level EQUAL -1)
        message(FATAL_ERROR "${cache_var}=${${cache_var}} is invalid, use one of: ${LOG_LEVEL_NAMES}")
    endif()
    target_compile_definitions(${target} PRIVATE ${define}=${level})
    message(STATUS "${cache_var}: ${level_name}")
endfunction()

add_subdirectory(la9310)
add_subdirectory(bootloader)
# currently not utilizing FreeRTOS
# add_subdirectory(FreeRTOS)
include_directories(include)
add_subdirectory(src)
add_subdirectory(limesuiteng/embedded)
