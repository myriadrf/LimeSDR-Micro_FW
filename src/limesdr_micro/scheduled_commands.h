#ifndef LIME_SCHEDULED_COMMANDS_H
#define LIME_SCHEDULED_COMMANDS_H

#include <stdint.h>
#include <stdbool.h>

void scheduled_commands_clear(void);
void scheduled_commands_update(void);
uint32_t scheduled_commands_enqueue(uint64_t timepoint, uint32_t cmd, uint32_t *data, uint32_t len);

#endif