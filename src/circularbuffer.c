#include "circularbuffer.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

void CircularBuffer_deinit(CircularBuffer *cb) {
    if (cb->buffer != NULL) {
        free(cb->buffer);
        cb->buffer = NULL;
    }

    cb->head = 0;
    cb->tail = 0;
}

void CircularBuffer_init(CircularBuffer *cb, unsigned size) {
    cb->buffer = (uint8_t *)malloc(size + 1);
    assert(cb->buffer != NULL);  // Bonk
    cb->size = size + 1;
    cb->head = 0;
    cb->tail = 0;
}

unsigned CircularBuffer_count(CircularBuffer *cb) { return (cb->size - cb->tail + cb->head) % cb->size; }
unsigned CircularBuffer_unused(CircularBuffer *cb) { return cb->size - 1 - CircularBuffer_count(cb); }

void CircularBuffer_read(CircularBuffer *cb, uint8_t *data, const unsigned size) {
    for (unsigned i = 0; i < size; i++) {
        if (cb->tail == cb->head) {
            // assert(false);  // Buffer underflow
            while (1);  // Asserts optimized out in release builds, set breakpoint here for debugging
        }
        data[i] = cb->buffer[cb->tail];
        cb->tail = (cb->tail + 1) % cb->size;
    }
}

void CircularBuffer_write(CircularBuffer *cb, const uint8_t *data, const unsigned size) {
    for (unsigned i = 0; i < size; i++) {
        cb->buffer[cb->head] = data[i];
        cb->head = (cb->head + 1) % cb->size;
        if (cb->head == cb->tail) {
            // assert(false);  // Buffer overflow
            while (1);  // Asserts optimized out in release builds, set breakpoint here for debugging
        }
    }
}