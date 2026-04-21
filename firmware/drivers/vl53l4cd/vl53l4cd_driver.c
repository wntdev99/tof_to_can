#include "vl53l4cd_driver.h"
#include "platform.h"
#include "VL53L4CD_api.h"
#include "../../can/can_tx.h"
#include "../../can/can_protocol.h"
#include <stdio.h>

static VL53L4CD_Platform _platform;

bool vl53l4cd_drv_init(VL53L4CD_Drv *drv, uint8_t i2c_addr8,
                        uint32_t timing_budget_ms, uint32_t inter_meas_ms) {
    _platform.address = i2c_addr8;
    drv->initialized  = false;
    drv->ranging      = false;

    Dev_t dev = &_platform;

    if (VL53L4CD_SensorInit(dev)) {
        printf("[L4CD] init failed (0x%02X)\n", i2c_addr8);
        return false;
    }
    if (VL53L4CD_SetRangeTiming(dev, timing_budget_ms, inter_meas_ms)) {
        printf("[L4CD] SetRangeTiming failed\n");
        return false;
    }
    drv->initialized = true;
    printf("[L4CD] OK @ 0x%02X  tb=%lums\n", i2c_addr8, (unsigned long)timing_budget_ms);
    return true;
}

bool vl53l4cd_drv_start(VL53L4CD_Drv *drv) {
    if (!drv->initialized) return false;
    if (VL53L4CD_StartRanging(&_platform)) return false;
    drv->ranging = true;
    return true;
}

bool vl53l4cd_drv_poll_and_push(VL53L4CD_Drv *drv, ring_buffer_t *rb) {
    if (!drv->ranging) return false;

    uint8_t ready = 0;
    VL53L4CD_CheckForDataReady(&_platform, &ready);
    if (!ready) return false;

    VL53L4CD_ResultsData_t result = {0};
    if (VL53L4CD_GetResult(&_platform, &result)) return false;

    VL53L4CD_ClearInterrupt(&_platform);

    can_pack_single_zone(CAN_ID_L4CD_BASE,
                         result.distance_mm,
                         result.range_status, rb);
    return true;
}
