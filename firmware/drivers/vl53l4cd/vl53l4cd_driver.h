#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../../utils/ring_buffer.h"

typedef struct {
    bool initialized;
    bool ranging;
} VL53L4CD_Drv;

bool vl53l4cd_drv_init(VL53L4CD_Drv *drv, uint8_t i2c_addr8,
                       uint32_t timing_budget_ms, uint32_t inter_meas_ms);
bool vl53l4cd_drv_start(VL53L4CD_Drv *drv);
bool vl53l4cd_drv_poll_and_push(VL53L4CD_Drv *drv, ring_buffer_t *rb);
