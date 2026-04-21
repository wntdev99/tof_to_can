#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../../utils/ring_buffer.h"

/*
 * VL53L5CX_Configuration은 이 헤더에 노출하지 않는다.
 * ST ULD API 헤더(vl53l5cx_api.h)는 vl53l5cx_driver.c 내부에서만 include한다.
 * → 여러 ST ULD 헤더를 동시에 include할 때 발생하는 매크로/union 충돌 방지.
 */

typedef struct {
    bool initialized;
    bool ranging;
} VL53L5CX_Drv;

bool vl53l5cx_drv_init(VL53L5CX_Drv *drv, uint8_t i2c_addr8);
bool vl53l5cx_drv_start(VL53L5CX_Drv *drv);
bool vl53l5cx_drv_poll_and_push(VL53L5CX_Drv *drv, ring_buffer_t *rb);
