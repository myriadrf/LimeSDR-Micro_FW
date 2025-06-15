if (NOT LIMESUITENG_ROOT)
    message(FATAL_ERROR "LIMESUITENG_ROOT not specified")
endif()

target_compile_definitions(la9310 PUBLIC "-DLMS7002M_CLOCK")
target_sources(la9310 PRIVATE
    "${LIMESUITENG_ROOT}/embedded/lms7002m/calibrations.c"
    "${LIMESUITENG_ROOT}/embedded/lms7002m/lms7002m.c"
    "${LIMESUITENG_ROOT}/embedded/lms7002m/privates.c"
    "${LIMESUITENG_ROOT}/embedded/lms7002m/save_state.c"
    "${LIMESUITENG_ROOT}/embedded/lms7002m/spi.c"
    "limesdr_micro_init.c"
    "limesdr_commands.c"
)

target_include_directories(la9310 PRIVATE "${LIMESUITENG_ROOT}/embedded/include")
target_include_directories(la9310 PRIVATE "${LIMESUITENG_ROOT}/embedded")
