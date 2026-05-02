#ifndef LIME_TIMER64_H
#define LIME_TIMER64_H

#include <stdint.h>

void timer64_reset(void);
uint64_t timer64_get_counter(void);

#endif // LIME_TIMER64_H