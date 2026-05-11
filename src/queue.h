#ifndef LIME_QUEUE_H
#define LIME_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct generic_fifo {
    void *items;
    uint16_t head;
    uint16_t tail;
    uint16_t item_size;
    uint16_t capacity;
    uint16_t fill;
} queue_t;

void queue_clear(queue_t *fifo);
void queue_init(queue_t *fifo, uint16_t capacity, void *mem, uint16_t item_size);
uint16_t queue_size(const queue_t *fifo);
bool queue_isfull(const queue_t *fifo);
bool queue_isempty(const queue_t *fifo);
void queue_push(queue_t *fifo, const void *item);
void queue_pop(queue_t *fifo);
void *queue_front(queue_t *fifo);
void *queue_back(queue_t *fifo);

#endif