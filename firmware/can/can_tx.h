#pragma once
#include <stdint.h>
#include "../utils/ring_buffer.h"
#include "can_protocol.h"

/* 단일존 센서 (L4CD) → 1 CAN 프레임 */
void can_pack_single_zone(uint16_t base_id, uint16_t dist_mm,
                          uint8_t status, ring_buffer_t *rb);

/* 다존 센서 (L5/L7/L8CX) → 멀티프레임 */
void can_pack_multizone(uint16_t base_id, const uint16_t *dist_mm,
                        uint8_t zones, ring_buffer_t *rb);

/* TMF8828 (81존) → 멀티프레임 */
void can_pack_tmf8828(const uint16_t *dist_mm, uint8_t zones,
                      ring_buffer_t *rb);

/* Core1: 링 버퍼에서 프레임 꺼내 MCP25625 전송 */
void can_tx_task(ring_buffer_t *rb);
