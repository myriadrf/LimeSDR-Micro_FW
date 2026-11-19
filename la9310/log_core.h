#ifndef LA9310_LOG_CORE_H
#define LA9310_LOG_CORE_H

/*
 * Logging core shared by the la9310_cm4 library (la9310_log.h) and the
 * LimeSDR-Micro board firmware (src/log.h). Do not use it directly; include
 * one of those headers instead.
 *
 * There are two filters:
 * - Build time: each logging domain has its own maximum level. A message above
 *   it compiles to dead code, so its format string is left out of the binary.
 * - Runtime: one level shared by all domains (memlog.log_level, which the host
 *   can change). A message compiled in is printed only if its level is at or
 *   below the runtime level.
 */

#include <stdint.h>

// Message levels. The values are also the runtime log level the host sets.
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DBG 4
#define LOG_LEVEL_ISR 5
#define LOG_LEVEL_ALL 6

#define LOG_EOL "\r\n"

// Implemented by the firmware that links the library (src/log.c). The format
// is checked like printf, but _doprint() (src/print_scan.c) supports only a
// subset: no floating point, no z/j/t length modifiers and no '*' width or
// precision, so avoid those even though the check accepts them.
void log_format_output(int32_t log_level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// Prints the message when `enabled` is nonzero. `enabled` must be a constant
// expression: when it is 0, the compiler drops the call and its format string,
// but the arguments are still type-checked and count as used, so turning a
// level off does not cause unused-variable warnings. The arguments are not
// evaluated when the level is disabled.
#define LOG_EMIT_IF_(enabled, level, ...) \
    do \
    { \
        if (enabled) \
            log_format_output((level), __VA_ARGS__); \
    } while (0)

#endif // LA9310_LOG_CORE_H
