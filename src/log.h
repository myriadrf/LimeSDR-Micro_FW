#ifndef DEVICE_LOG_H
#define DEVICE_LOG_H

/*
 * Logging for the LimeSDR-Micro board firmware (everything under src/). It is
 * filtered at build time independently of the la9310_cm4 library logging
 * (la9310_log.h); the runtime level (memlog.log_level) is shared.
 *
 * Build time selection, from the narrowest scope to the widest:
 * 1. Per level: define DEVICE_LOG_<LEVEL>_ENABLED to 0 or 1
 *    (LEVEL = ERR, WARN, INFO, DBG, ISR).
 * 2. Per file: define DEVICE_LOG_MAX_LEVEL to one of LOG_LEVEL_* (or 0..6).
 * 3. Whole firmware: the DEVICE_LOG_LEVEL CMake cache option, which sets
 *    DEVICE_LOG_MAX_LEVEL_DEFAULT.
 *
 * Per-file overrides go before the first include of this header, e.g.
 *     #define DEVICE_LOG_MAX_LEVEL LOG_LEVEL_DBG // up to debug
 *     #include "log.h"
 * or from CMake:
 *     set_source_files_properties(ppsdo/ppsdo.c PROPERTIES
 *         COMPILE_DEFINITIONS DEVICE_LOG_MAX_LEVEL=LOG_LEVEL_DBG)
 */

#include "log_core.h"

#include <stdint.h>

#ifndef DEVICE_LOG_MAX_LEVEL
    #ifdef DEVICE_LOG_MAX_LEVEL_DEFAULT
        #define DEVICE_LOG_MAX_LEVEL DEVICE_LOG_MAX_LEVEL_DEFAULT
    #else
        #define DEVICE_LOG_MAX_LEVEL LOG_LEVEL_ERR
    #endif
#endif

#ifndef DEVICE_LOG_ERR_ENABLED
    #define DEVICE_LOG_ERR_ENABLED (DEVICE_LOG_MAX_LEVEL >= LOG_LEVEL_ERR)
#endif
#ifndef DEVICE_LOG_WARN_ENABLED
    #define DEVICE_LOG_WARN_ENABLED (DEVICE_LOG_MAX_LEVEL >= LOG_LEVEL_WARN)
#endif
#ifndef DEVICE_LOG_INFO_ENABLED
    #define DEVICE_LOG_INFO_ENABLED (DEVICE_LOG_MAX_LEVEL >= LOG_LEVEL_INFO)
#endif
#ifndef DEVICE_LOG_DBG_ENABLED
    #define DEVICE_LOG_DBG_ENABLED (DEVICE_LOG_MAX_LEVEL >= LOG_LEVEL_DBG)
#endif
#ifndef DEVICE_LOG_ISR_ENABLED
    #define DEVICE_LOG_ISR_ENABLED (DEVICE_LOG_MAX_LEVEL >= LOG_LEVEL_ISR)
#endif

#define log_err(...) LOG_EMIT_IF_(DEVICE_LOG_ERR_ENABLED, LOG_LEVEL_ERR, __VA_ARGS__)
#define log_warn(...) LOG_EMIT_IF_(DEVICE_LOG_WARN_ENABLED, LOG_LEVEL_WARN, __VA_ARGS__)
#define log_info(...) LOG_EMIT_IF_(DEVICE_LOG_INFO_ENABLED, LOG_LEVEL_INFO, __VA_ARGS__)
#define log_dbg(...) LOG_EMIT_IF_(DEVICE_LOG_DBG_ENABLED, LOG_LEVEL_DBG, __VA_ARGS__)
#define log_isr(...) LOG_EMIT_IF_(DEVICE_LOG_ISR_ENABLED, LOG_LEVEL_ISR, __VA_ARGS__)

// In-memory log ring buffer the host reads. The host finds it through the host
// interface table (LA9310_HIF_MEMORY_LOG, la9310_host_if.h); the layout must match
// MemoryLog_hif in limesuiteng src/chips/LA9310/firmware/memory_log.h.
struct MemoryLog {
    char *buffer_addr;
    uint32_t buffer_size;
    uint32_t produced;
    uint32_t host_consumed;
    uint32_t log_level; // runtime level shared by all logging domains
};

extern struct MemoryLog memlog;

void memlog_clear();

#endif // DEVICE_LOG_H
