/**
 * OctroBot Robot Arm Firmware - Utilities Header
 * Copyright (c) 2026 OctroBot Project
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OCTROBOT_UTILS_H
#define OCTROBOT_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Ring Buffer Implementation */
#define RING_BUFFER_SIZE 256
/**
 * @brief Macro to declare a ring buffer for a specific type
 * With this macro, you can create a ring buffer for any data type by providing a name and the type.
 * @param name: Prefix for the ring buffer type and functions
 * @param type: Data type to store in the ring buffer (e.g., uint8_t, struct packet, etc.)
 */
#define UTILS_DECLARE_RING_BUFFER(name, type) \
    typedef struct { \
        type buffer[RING_BUFFER_SIZE]; \
        volatile uint16_t head; \
        volatile uint16_t tail; \
    } name##_ring_buffer_t; \
    void name##_ring_buffer_init(name##_ring_buffer_t *rb) \
    { \
        rb->head = 0; \
        rb->tail = 0; \
    } \
    bool name##_ring_buffer_is_empty(const name##_ring_buffer_t *rb) \
    { \
        return rb->head == rb->tail; \
    } \
    bool name##_ring_buffer_is_full(const name##_ring_buffer_t *rb) \
    { \
        return ((rb->head + 1) % RING_BUFFER_SIZE) == rb->tail; \
    } \
    bool name##_ring_buffer_append(name##_ring_buffer_t *rb, type item) \
    { \
        bool overwritten = false; \
        if (name##_ring_buffer_is_full(rb)) { \
            /* Drop oldest item to make room for the incoming one */ \
            rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE; \
            overwritten = true; \
        } \
        rb->buffer[rb->head] = item; \
        rb->head = (rb->head + 1) % RING_BUFFER_SIZE; \
        return !overwritten; \
    } \
    bool name##_ring_buffer_pop(name##_ring_buffer_t *rb, type *item) \
    { \
        if (name##_ring_buffer_is_empty(rb)) { \
            return false; /* Buffer empty */ \
        } \
        *item = rb->buffer[rb->tail]; \
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE; \
        return true; \
    } \
    bool name##_ring_buffer_contains(const name##_ring_buffer_t *rb, type item) \
    { \
        uint16_t idx = rb->tail; \
        while (idx != rb->head) { \
            if (rb->buffer[idx] == item) { \
                return true; \
            } \
            idx = (idx + 1) % RING_BUFFER_SIZE; \
        } \
        return false; \
    } \
    size_t name##_ring_buffer_extract_until(name##_ring_buffer_t *rb, type *out_buf, size_t max_len, type terminator) \
    { \
        size_t count = 0; \
        while (count < max_len && !name##_ring_buffer_is_empty(rb)) { \
            type item; \
            name##_ring_buffer_pop(rb, &item); \
            out_buf[count++] = item; \
            if (item == terminator) { \
                break; \
            } \
        } \
        return count; \
    }

#ifdef __cplusplus
}
#endif

#endif /* OCTROBOT_UTILS_H */