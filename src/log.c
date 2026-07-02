#include "log.h"

#include "config.h"
#include "la9310_host_if.h"
#include "io.h"

#include "debug_console.h"
#include "drivers/serial/print_scan.h"

#define LOG_TO_UART 0
#define LOG_TO_EXTERNAL_MEMORY 1

static uint32_t ulMemLogIndex = 0;
static struct debug_log_regs *pxDbglog = NULL;

static void vMemlogWrite(const uint8_t *pucData, size_t xLength)
{
    if (pxDbglog == NULL)
        return;
    uint8_t *ucdbgptr = (uint8_t *)(pxDbglog->buf);
    if (ucdbgptr == NULL)
        return;

    ucdbgptr += ulMemLogIndex;

    for (int i = 0; i < xLength; i++)
    {
        OUT_8(ucdbgptr, *pucData);
        pucData++;
        ucdbgptr++;
    }

    ulMemLogIndex += xLength;

    if (ulMemLogIndex >= pxDbglog->len)
    {
        ulMemLogIndex = 0;
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
    if (!pxDbglog)
        return;

    va_list ap;
    if (pxDbglog->log_level >= log_level)
    {
        va_start( ap, fmt_s );
        _doprint( NULL, log_putc, -1, ( char * ) fmt_s, ap );
        va_end( ap );
    }
}

void log_initialize(struct debug_log_regs *log)
{
    pxDbglog = log;
    ulMemLogIndex = 0;
}