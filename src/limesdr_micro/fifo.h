// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_FIFO_H
#define LIME_FIFO_H

#include <stdint.h>
#include <stdbool.h>

#define MFIFO_SIZE 2 // must be power of 2
#define MFIFO_SIZE_MASK (MFIFO_SIZE - 1)

typedef struct MemoryFIFO {
    uint16_t items[MFIFO_SIZE];
    uint8_t head;
    uint8_t cnt;
} fifo_t;

static inline void fifo_reset(fifo_t *fifo)
{
    fifo->head = 0;
    fifo->cnt = 0;
}

static inline void fifo_push(fifo_t *fifo, const uint16_t block)
{
    fifo->items[(fifo->head + fifo->cnt) & MFIFO_SIZE_MASK] = block;
    ++fifo->cnt;
}

static inline uint16_t fifo_front(const fifo_t *fifo)
{
    return fifo->items[fifo->head];
}

static inline void fifo_pop(struct MemoryFIFO *fifo)
{
    ++fifo->head;
    fifo->head &= MFIFO_SIZE_MASK;
    --fifo->cnt;
}

static inline uint16_t fifo_size(const fifo_t *fifo)
{
    return fifo->cnt;
}

static inline bool fifo_isfull(const fifo_t *fifo)
{
    return fifo->cnt == MFIFO_SIZE;
}

static inline bool fifo_isempty(const fifo_t *fifo)
{
    return fifo->cnt == 0;
}

#endif // LIME_FIFO_H