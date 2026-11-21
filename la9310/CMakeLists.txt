
add_library(la9310_cm4)

target_include_directories(la9310_cm4 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/cm4
)

target_sources(la9310_cm4 PRIVATE
    la9310.c
    tmu.c
    fsl_dspi.c
    # la9310_avi.c
    la9310_dcs.c
    la9310_edma.c
    la9310_gpio.c
    la9310_i2c.c
    la9310_pinmux.c
    la9310_pcie.c
    la9310_vspa_dma.c
    # la9310_watchdog.c # TODO
    llcp_api.c
    phytimer.c
    serial_ns16550.c
)

# Check la9310_log_*() format strings (the library is not built with -Wall)
target_compile_options(la9310_cm4 PRIVATE -Wformat)

add_log_level_option(LA9310_LOG_LEVEL "la9310_cm4 library build time log level"
    la9310_cm4 LA9310_LOG_MAX_LEVEL_DEFAULT WARN)

SET(LA9310_DCS_CLK_HZ "122880000" CACHE STRING "LA9310 post init DCS_CLK frequency")
target_compile_definitions(la9310_cm4 PUBLIC LA9310_DCS_CLK_HZ=${LA9310_DCS_CLK_HZ})
message(STATUS "LA9310_DCS_CLK: ${LA9310_DCS_CLK_HZ}")
