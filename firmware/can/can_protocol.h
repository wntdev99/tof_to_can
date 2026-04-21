#pragma once
#include <stdint.h>

/* CAN 비트레이트 */
#define CAN_BITRATE_500KBPS

/* 센서별 Base CAN ID */
#define CAN_ID_L5CX_BASE    0x100U
#define CAN_ID_L4CD_BASE    0x200U
#define CAN_ID_L7CX_BASE    0x300U
#define CAN_ID_L8CX_BASE    0x400U
#define CAN_ID_TMF8828_BASE 0x500U

/* 멀티프레임: CAN ID = BaseID | frame_index (0-based) */
#define CAN_FRAME_ID(base, idx) ((base) | (uint16_t)(idx))

/* 단일 CAN 프레임 최대 데이터 크기 */
#define CAN_PAYLOAD_BYTES   8

/*
 * 프레임 레이아웃 (8byte)
 *
 * [단일존 센서 - VL53L4CD]
 *   byte 0-1 : distance_mm (uint16_t, little-endian)
 *   byte 2   : status
 *   byte 3-7 : 예약
 *
 * [다존 센서 - VL53L5/L7/L8CX, TMF8828]
 *   첫 프레임 헤더 (frame_index == 0):
 *     byte 0   : total_frames  (전체 프레임 수)
 *     byte 1   : zones_per_dim (예: 8 → 8×8)
 *     byte 2-7 : 첫 3존 distance_mm[0..2] 하위 바이트 + status
 *   이후 프레임:
 *     byte 0-7 : 순서대로 distance_mm (uint16_t LE) × 4존
 *
 * TMF8828 (9×9=81존):
 *   총 ceil(81*2/8) + 1 헤더 = 22 프레임
 */

/* 각 다존 센서의 프레임 수 */
#define FRAMES_L5CX    16   /* 64존 × 2byte = 128byte / 8 */
#define FRAMES_L7CX    16
#define FRAMES_L8CX    16
#define FRAMES_TMF8828 22   /* 81존 × 2byte = 162byte, ceil(162/8)+헤더 */

/* 폴링 주기 (ms) */
#define POLL_PERIOD_MULTIZONE_MS  100   /* 10 Hz */
#define POLL_PERIOD_L4CD_MS        20   /* 50 Hz */
#define POLL_PERIOD_TMF8828_MS     50   /* 20 Hz */
