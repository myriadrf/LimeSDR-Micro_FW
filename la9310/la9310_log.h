#ifndef LA9310_LOG_H
#define LA9310_LOG_H

/*
 * Logging for the la9310_cm4 SoC library. It is filtered at build time
 * independently of the board logging (src/log.h); the runtime level is shared.
 *
 * Build time selection, from the narrowest scope to the widest:
 * 1. Per level: define LA9310_LOG_<LEVEL>_ENABLED to 0 or 1
 *    (LEVEL = ERR, WARN, INFO, DBG, ISR).
 * 2. Per file: define LA9310_LOG_MAX_LEVEL to one of LOG_LEVEL_* (or 0..6).
 * 3. Whole library: the LA9310_LOG_LEVEL CMake cache option, which sets
 *    LA9310_LOG_MAX_LEVEL_DEFAULT.
 *
 * Per-file overrides go before the first include of this header, e.g.
 *     #define LA9310_LOG_MAX_LEVEL LOG_LEVEL_ERR // errors only
 *     #include "la9310_log.h"
 * or from CMake:
 *     set_source_files_properties(la9310_i2c.c PROPERTIES
 *         COMPILE_DEFINITIONS LA9310_LOG_DBG_ENABLED=0)
 */

#include "log_core.h"

#ifndef LA9310_LOG_MAX_LEVEL
    #ifdef LA9310_LOG_MAX_LEVEL_DEFAULT
        #define LA9310_LOG_MAX_LEVEL LA9310_LOG_MAX_LEVEL_DEFAULT
    #else
        #define LA9310_LOG_MAX_LEVEL LOG_LEVEL_ERR
    #endif
#endif

#ifndef LA9310_LOG_ERR_ENABLED
    #define LA9310_LOG_ERR_ENABLED (LA9310_LOG_MAX_LEVEL >= LOG_LEVEL_ERR)
#endif
#ifndef LA9310_LOG_WARN_ENABLED
    #define LA9310_LOG_WARN_ENABLED (LA9310_LOG_MAX_LEVEL >= LOG_LEVEL_WARN)
#endif
#ifndef LA9310_LOG_INFO_ENABLED
    #define LA9310_LOG_INFO_ENABLED (LA9310_LOG_MAX_LEVEL >= LOG_LEVEL_INFO)
#endif
#ifndef LA9310_LOG_DBG_ENABLED
    #define LA9310_LOG_DBG_ENABLED (LA9310_LOG_MAX_LEVEL >= LOG_LEVEL_DBG)
#endif
#ifndef LA9310_LOG_ISR_ENABLED
    #define LA9310_LOG_ISR_ENABLED (LA9310_LOG_MAX_LEVEL >= LOG_LEVEL_ISR)
#endif

#define la9310_log_err(...) LOG_EMIT_IF_(LA9310_LOG_ERR_ENABLED, LOG_LEVEL_ERR, __VA_ARGS__)
#define la9310_log_warn(...) LOG_EMIT_IF_(LA9310_LOG_WARN_ENABLED, LOG_LEVEL_WARN, __VA_ARGS__)
#define la9310_log_info(...) LOG_EMIT_IF_(LA9310_LOG_INFO_ENABLED, LOG_LEVEL_INFO, __VA_ARGS__)
#define la9310_log_dbg(...) LOG_EMIT_IF_(LA9310_LOG_DBG_ENABLED, LOG_LEVEL_DBG, __VA_ARGS__)
#define la9310_log_isr(...) LOG_EMIT_IF_(LA9310_LOG_ISR_ENABLED, LOG_LEVEL_ISR, __VA_ARGS__)

#endif // LA9310_LOG_H
