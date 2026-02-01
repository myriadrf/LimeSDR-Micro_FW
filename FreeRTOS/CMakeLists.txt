include(FetchContent)

### ARM_CM4 port ###
add_library(freertos_kernel_port STATIC)
target_sources(freertos_kernel_port
    PRIVATE
    GCC/ARM_CM4/port.c
    GCC/ARM_CM4/portmacro.h
)
target_include_directories(freertos_kernel_port
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/GCC/ARM_CM4
)

target_link_libraries(freertos_kernel_port
    PRIVATE
    freertos_kernel_include
)

### FreeRTOS config ###
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
)
target_sources(freertos_config PUBLIC ${CMAKE_CURRENT_LIST_DIR}/FreeRTOSConfig.h)

### Download FreeRTOS ###
FetchContent_Declare(freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  # GIT_TAG        1dbc77697f4c63e1b18a2c7f7a15aad4ae14af7e
  GIT_TAG V10.6.2
  GIT_SHALLOW TRUE # get only last version
  GIT_PROGRESS TRUE # show progress of download
  USES_TERMINAL_DOWNLOAD TRUE # show progress in ninja generator
)
set( FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set( FREERTOS_PORT "A_CUSTOM_PORT" CACHE STRING "" FORCE)
FetchContent_MakeAvailable(freertos_kernel)
