target_compile_definitions(la9310 PUBLIC "-DLMS7002M_CLOCK")
target_sources(la9310 PRIVATE
    "limesdr_micro_init.c"
    "limesdr_commands.c"
    "eeprom.c"
    leds.c
)

target_link_libraries(la9310 PRIVATE lms7002m)
