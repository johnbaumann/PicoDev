#pragma once

#include <stdint.h>

typedef struct {
    uint8_t *buffer;   // Pointer to the buffer
    volatile unsigned head;     // Index of the head element
    volatile unsigned tail;     // Index of the tail element
    volatile unsigned size;  // Maximum size of the buffer
} CircularBuffer;

void CircularBuffer_deinit(CircularBuffer *cb);
void CircularBuffer_init(CircularBuffer *cb, unsigned size);
unsigned CircularBuffer_count(CircularBuffer *cb);
unsigned CircularBuffer_unused(CircularBuffer *cb);
void CircularBuffer_read(CircularBuffer *cb, uint8_t *data, const unsigned size);
void CircularBuffer_write(CircularBuffer *cb, const uint8_t *data, const unsigned size);
