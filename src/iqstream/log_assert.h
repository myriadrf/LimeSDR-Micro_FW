#ifndef LOG_ASSERT_H
#define LOG_ASSERT_H

#include "log.h"

#define ASSERT_EN 1

#if ASSERT_EN
    #define log_assert(x, msg) \
        do \
        { \
            if (!(x)) \
            { \
                log_err(msg LOG_EOL); \
            } \
        } while (0)

#else
    #define log_assert(x, msg) \
        do \
        { \
        } while (0)
#endif

#endif // LOG_ASSERT_H