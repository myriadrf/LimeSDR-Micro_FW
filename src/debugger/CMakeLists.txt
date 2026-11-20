# Host debugging over PCIe: the debug proxy (vspa_debug.h) and the M4 debug
# monitor (m4_monitor.h). The proxy is built when either part is enabled.
option(VSPA_DEBUGGER "VSPA debugging over PCIe: debug proxy ops 1-2 to the VSPA debug block (src/debugger/vspa_debug.h)" ON)
option(M4_DEBUGGER "Self-hosted M4 debugging over PCIe: debug monitor exception handler and proxy ops 3-6 (src/debugger/m4_monitor.h)" ON)

if(VSPA_DEBUGGER OR M4_DEBUGGER)
    add_library(la9310-dbg-client-proxy STATIC vspa_debug.c)
    target_compile_definitions(la9310-dbg-client-proxy PUBLIC DEBUG_PROXY)
    target_compile_options(la9310-dbg-client-proxy PRIVATE
        -fno-strict-aliasing
        -ffreestanding
        -fno-builtin
    )
    target_include_directories(la9310-dbg-client-proxy
        PUBLIC ${CMAKE_CURRENT_LIST_DIR}
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../platform/ARM_CM4/include
    )
    target_link_libraries(la9310-dbg-client-proxy PUBLIC la9310_cm4)

    if(VSPA_DEBUGGER)
        target_compile_definitions(la9310-dbg-client-proxy PUBLIC VSPA_DEBUGGER)
    endif()
    if(M4_DEBUGGER)
        target_compile_definitions(la9310-dbg-client-proxy PUBLIC M4_DEBUGGER)
        # M4MonitorEntry calls vLogFault() from platform/ARM_CM4/exceptions.c
        target_sources(la9310-dbg-client-proxy PRIVATE m4_monitor.c)
    endif()
else()
    # headers only: the proxy calls compile to nothing
    add_library(la9310-dbg-client-proxy INTERFACE)
    target_include_directories(la9310-dbg-client-proxy INTERFACE ${CMAKE_CURRENT_LIST_DIR})
endif()

add_feature_info("VSPA_DEBUGGER" VSPA_DEBUGGER "VSPA debugging through the PCIe debug proxy")
add_feature_info("M4_DEBUGGER" M4_DEBUGGER "Self-hosted M4 debugging through the PCIe using debug monitor exception")
