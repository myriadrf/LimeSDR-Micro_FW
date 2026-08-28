#include "log.h"

#include "config.h"
#include "la9310_host_if.h"
#include "io.h"

#include "debug_console.h"
#include "drivers/serial/print_scan.h"

#define LOG_TO_UART 0
#define LOG_TO_EXTERNAL_MEMORY 1

char log_buffer[4096] __attribute__((section(".hif")));
struct MemoryLog memlog = { .buffer_addr = &log_buffer[0],
    .buffer_size = sizeof(log_buffer),
    .produced = 0,
    .host_consumed = 0,
    .log_level = LA9310_LOG_LEVEL_INFO };

static void vMemlogWrite(const uint8_t *pucData, size_t xLength)
{
    for (int i = 0; i < xLength; i++)
    {
        OUT_8(&memlog.buffer_addr[memlog.produced], *pucData);
        memlog.produced = (memlog.produced + 1) % memlog.buffer_size;
        ++pucData;
    }
}

static int log_putc(int ich, void* )
{
    uint8_t uch = ich;
#if LOG_TO_UART
    debug_putchar(uch);
#endif
#if LOG_TO_EXTERNAL_MEMORY
    vMemlogWrite(&uch, 1);
#endif
    return ich;
}

void log_format_output(int32_t log_level, const char* fmt_s, ...)
{
    va_list ap;
    if (memlog.log_level >= log_level)
    {
        va_start( ap, fmt_s );
        _doprint( NULL, log_putc, -1, ( char * ) fmt_s, ap );
        va_end( ap );
    }
}

void memlog_clear()
{
    memlog.produced = 0;
    memlog.host_consumed = 0;
    memset(memlog.buffer_addr, 0, memlog.buffer_size);
}