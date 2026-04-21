#include "ring_buffer.h"
#include "pico/stdlib.h"

void ring_buffer_init(ring_buffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->lock = spin_lock_init(spin_lock_claim_unused(true));
}

bool ring_buffer_is_full(const ring_buffer_t *rb) {
    return ((rb->head - rb->tail) >= RING_BUF_CAPACITY);
}

bool ring_buffer_is_empty(const ring_buffer_t *rb) {
    return (rb->head == rb->tail);
}

bool ring_buffer_push(ring_buffer_t *rb, const can_frame_t *frame) {
    uint32_t save = spin_lock_blocking(rb->lock);
    if (ring_buffer_is_full(rb)) {
        spin_unlock(rb->lock, save);
        return false;
    }
    rb->frames[rb->head % RING_BUF_CAPACITY] = *frame;
    rb->head++;
    spin_unlock(rb->lock, save);
    return true;
}

bool ring_buffer_pop(ring_buffer_t *rb, can_frame_t *frame) {
    uint32_t save = spin_lock_blocking(rb->lock);
    if (ring_buffer_is_empty(rb)) {
        spin_unlock(rb->lock, save);
        return false;
    }
    *frame = rb->frames[rb->tail % RING_BUF_CAPACITY];
    rb->tail++;
    spin_unlock(rb->lock, save);
    return true;
}
