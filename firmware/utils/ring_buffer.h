#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/sync.h"

/* Core0(생산자) → Core1(소비자) inter-core 링 버퍼 */

#define RING_BUF_CAPACITY 64

typedef struct {
    uint16_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

typedef struct {
    can_frame_t  frames[RING_BUF_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    spin_lock_t *lock;
} ring_buffer_t;

void     ring_buffer_init(ring_buffer_t *rb);
bool     ring_buffer_push(ring_buffer_t *rb, const can_frame_t *frame);
bool     ring_buffer_pop(ring_buffer_t *rb, can_frame_t *frame);
bool     ring_buffer_is_empty(const ring_buffer_t *rb);
bool     ring_buffer_is_full(const ring_buffer_t *rb);
