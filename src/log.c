// #include "la9310_info.h"
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "la9310_info.h"
#include "la9310_host_if.h"

#include "debug_console.h"
#include "drivers/serial/print_scan.h"

#define LOG_TO_UART 0
#define LOG_TO_EXTERNAL_MEMORY 1

extern struct la9310_info g_la9310_info;

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

    struct la9310_hif * pHif = g_la9310_info.pHif;
    struct debug_log_regs * pDbgLogRegs = &pHif->dbg_log_regs;
    if( pDbgLogRegs->log_level >= log_level )
    {
        va_start( ap, fmt_s );
        _doprint( NULL, log_putc, -1, ( char * ) fmt_s, ap );
        va_end( ap );
    }
}
