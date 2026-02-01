#ifndef LA9310_LOGING_H
#define LA9310_LOGING_H

#include <stdint.h>

#define LA9310_LOG_LEVEL_ERR    1
#define LA9310_LOG_LEVEL_INFO   2
#define LA9310_LOG_LEVEL_DBG    3
#define LA9310_LOG_LEVEL_ISR    4
#define LA9310_LOG_LEVEL_ALL    5

#if 1 // LOGGING_ENABLED
    void log_format_output(int32_t log_level, const char* fmt, ...);
#else
    #define log_format_output(...)
#endif

#define log_err( ... ) \
    do { \
        log_format_output(LA9310_LOG_LEVEL_ERR, __VA_ARGS__ ); \
    } while( 0 )

#define log_info( ... ) \
    do { \
        log_format_output(LA9310_LOG_LEVEL_INFO, __VA_ARGS__ ); \
    } while( 0 )

#define log_dbg( ... ) \
    do { \
        log_format_output(LA9310_LOG_LEVEL_DBG, __VA_ARGS__ ); \
    } while( 0 )

#define log_isr( ... ) \
    do { \
        log_format_output(LA9310_LOG_LEVEL_ISR, __VA_ARGS__ ); \
    } while( 0 )

#endif // LA9310_LOGING_H
