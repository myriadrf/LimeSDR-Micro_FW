
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

SET(LA9310_DCS_CLK_HZ "122880000" CACHE STRING "LA9310 post init DCS_CLK frequency")
target_compile_definitions(la9310_cm4 PUBLIC LA9310_DCS_CLK_HZ=${LA9310_DCS_CLK_HZ})
message(STATUS "LA9310_DCS_CLK: ${LA9310_DCS_CLK_HZ}")
