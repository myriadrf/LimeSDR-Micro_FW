target_compile_definitions(limesdr-micro-m4 PUBLIC "-DLMS7002M_CLOCK")
target_sources(limesdr-micro-m4 PRIVATE
    "limesdr_micro_init.c"
    "limesdr_commands.c"
    "eeprom.c"
    leds.c
    scheduled_commands.c
    timer64.c
    tx_control.c
)

target_link_libraries(limesdr-micro-m4 PRIVATE lms7002m)
