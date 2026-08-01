#pragma once

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint8_t *buffer;         // Pointer to the buffer
    volatile unsigned head;  // Index of the head element
    volatile unsigned tail;  // Index of the tail element
    volatile unsigned size;  // Maximum size of the buffer
} CircularBuffer;

unsigned CircularBuffer_count(CircularBuffer *cb);
void CircularBuffer_deinit(CircularBuffer *cb);
void CircularBuffer_init(CircularBuffer *cb, unsigned size);
void CircularBuffer_read(CircularBuffer *cb, uint8_t *data, const unsigned size);
void CircularBuffer_write(CircularBuffer *cb, const uint8_t *data, const unsigned size);
unsigned CircularBuffer_unused(CircularBuffer *cb);

void CircularBuffer_init(CircularBuffer *cb, unsigned size) {
    cb->buffer = (uint8_t *)malloc(size + 1);
    /*if(cb->buffer == NULL){while (1);}*/
    cb->size = size + 1;
    cb->head = 0;
    cb->tail = 0;
}

void CircularBuffer_deinit(CircularBuffer *cb) {
    if (cb->buffer != NULL) {
        free(cb->buffer);
        cb->buffer = NULL;
    }
}

// Amount of data stored in buffer
unsigned CircularBuffer_count(CircularBuffer *cb) { return (cb->size - cb->tail + cb->head) % cb->size; }
// Amount of space available in buffer
unsigned CircularBuffer_unused(CircularBuffer *cb) { return cb->size - 1 - CircularBuffer_count(cb); }

static inline uint8_t CircularBuffer_get(CircularBuffer *cb) {
    // if (cb->tail == cb->head) {while (1);}
    const uint8_t data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % cb->size;

    return data;
}
static inline void CircularBuffer_put(CircularBuffer *cb, uint8_t data) {
    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % cb->size;
    // if (cb->tail == cb->head) {while (1);}
}

void CircularBuffer_read(CircularBuffer *cb, uint8_t *data, const unsigned size) {
    for (unsigned i = 0; i < size; i++) {
        data[i] = CircularBuffer_get(cb);
    }
}

void CircularBuffer_write(CircularBuffer *cb, const uint8_t *data, const unsigned size) {
    for (unsigned i = 0; i < size; i++) {
        CircularBuffer_put(cb, data[i]);
    }
}