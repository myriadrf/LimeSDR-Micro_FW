#include "queue.h"

#include <string.h>

void queue_clear(queue_t *fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->fill = 0;
}

void queue_init(queue_t *fifo, uint16_t capacity, void *mem, uint16_t item_size)
{
    fifo->items = mem;
    fifo->item_size = item_size;
    fifo->capacity = capacity;
    queue_clear(fifo);
}

uint16_t queue_size(const queue_t *fifo)
{
    return fifo->fill;
}

bool queue_isfull(const queue_t *fifo)
{
    return fifo->fill == fifo->capacity;
}

bool queue_isempty(const queue_t *fifo)
{
    return fifo->fill == 0;
}

void queue_push(queue_t *fifo, const void *item)
{
    void *dest = &((uint8_t *)fifo->items)[fifo->tail * fifo->item_size];
    memcpy(dest, item, fifo->item_size);
    // fifo->items[fifo->tail & QUEUE_SIZE_MASK] = *event;
    ++fifo->tail;
    if (fifo->tail == fifo->capacity)
        fifo->tail = 0;
    ++fifo->fill;
}

void queue_pop(queue_t *fifo)
{
    ++fifo->head;
    if (fifo->head == fifo->capacity)
        fifo->head = 0;
    --fifo->fill;
}

void *queue_front(queue_t *fifo)
{
    void *src = &((uint8_t *)fifo->items)[fifo->head * fifo->item_size];
    return src;
}

void *queue_back(queue_t *fifo)
{
    void *src = &((uint8_t *)fifo->items)[fifo->head + (fifo->fill - 1) * fifo->item_size];
    return src;
}
