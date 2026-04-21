#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../../utils/ring_buffer.h"

/*
 * AMS TMF8828 드라이버 (register-level, AMS SDK 불필요)
 *
 * 하드웨어:
 *   I2C 주소  : 0x41 (7-bit, 고정)
 *   EN 핀     : GPIO12 (active-high, sensor_manager에서 초기화)
 *
 * 측정 모드  : 9채널 × 9 = 81존 (TMF8828 특유의 SPADs 분할 구성)
 * 최대 거리  : 5000mm
 *
 * 레지스터 맵 주요 주소 (AMS application note DS001051):
 *   0xE0 ENABLE   — CPU 활성화
 *   0x00 APPID    — 실행 중인 앱 ID (0x03 = measurement app)
 *   0x10 CMD_STAT — 명령/상태 레지스터
 *   0x20 RESULT   — 결과 데이터 시작 오프셋
 */

/* TMF8828 I2C 주소 (7-bit) */
#define TMF8828_I2C_ADDR   0x41

/* 주요 레지스터 */
#define TMF8828_REG_ENABLE       0xE0
#define TMF8828_REG_APPID        0x00
#define TMF8828_REG_CMD_STAT     0x10
#define TMF8828_REG_RESULT_NUMB  0x1F  /* 결과 개수 */
#define TMF8828_REG_RESULT_BASE  0x20  /* 결과 데이터 시작 */

/* APPID 값 */
#define TMF8828_APPID_BOOTLOADER  0x80
#define TMF8828_APPID_MEASUREMENT 0x03

/* CMD 값 */
#define TMF8828_CMD_START     0x02
#define TMF8828_CMD_STOP      0xFF
#define TMF8828_CMD_CLR_INT   0x01

/* 결과 개당 크기 (channel + sub_capture + distance_mm[2] + confidence) */
#define TMF8828_RESULT_ENTRY_SIZE  6
#define TMF8828_MAX_RESULTS        75   /* 측정 모드에 따라 최대 75개 */

typedef struct {
    bool initialized;
    bool ranging;
} TMF8828_Drv;

bool tmf8828_drv_init(TMF8828_Drv *drv);
bool tmf8828_drv_start(TMF8828_Drv *drv);

/*
 * 인터럽트 핀 없이 폴링 방식으로 결과 확인.
 * 새 결과 있으면 CAN 링 버퍼에 push.
 */
bool tmf8828_drv_poll_and_push(TMF8828_Drv *drv, ring_buffer_t *rb);
