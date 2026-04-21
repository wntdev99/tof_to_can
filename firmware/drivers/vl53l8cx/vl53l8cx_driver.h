#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../../utils/ring_buffer.h"

typedef struct {
    bool initialized;
    bool ranging;
} VL53L8CX_Drv;

bool vl53l8cx_drv_init(VL53L8CX_Drv *drv, uint8_t i2c_addr8);
bool vl53l8cx_drv_start(VL53L8CX_Drv *drv);
bool vl53l8cx_drv_poll_and_push(VL53L8CX_Drv *drv, ring_buffer_t *rb);
