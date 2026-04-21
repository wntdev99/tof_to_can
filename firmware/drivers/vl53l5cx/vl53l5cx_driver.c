#include "vl53l5cx_driver.h"
#include "platform.h"
#include "vl53l5cx_api.h"
#include "../../can/can_tx.h"
#include "../../can/can_protocol.h"
#include <stdio.h>

/* Configuration이 크므로 (.temp_buffer ~1KB+) 정적 전역으로 선언 */
static VL53L5CX_Configuration _dev;

bool vl53l5cx_drv_init(VL53L5CX_Drv *drv, uint8_t i2c_addr8) {
    _dev.platform.address = i2c_addr8;
    drv->initialized = false;
    drv->ranging     = false;

    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&_dev, &alive) || !alive) {
        printf("[L5CX] not alive (0x%02X)\n", i2c_addr8);
        return false;
    }
    if (vl53l5cx_init(&_dev)) {
        printf("[L5CX] init failed\n");
        return false;
    }
    vl53l5cx_set_resolution(&_dev, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&_dev, 10);

    drv->initialized = true;
    printf("[L5CX] OK @ 0x%02X\n", i2c_addr8);
    return true;
}

bool vl53l5cx_drv_start(VL53L5CX_Drv *drv) {
    if (!drv->initialized) return false;
    if (vl53l5cx_start_ranging(&_dev)) return false;
    drv->ranging = true;
    return true;
}

bool vl53l5cx_drv_poll_and_push(VL53L5CX_Drv *drv, ring_buffer_t *rb) {
    if (!drv->ranging) return false;

    uint8_t ready = 0;
    vl53l5cx_check_data_ready(&_dev, &ready);
    if (!ready) return false;

    static VL53L5CX_ResultsData results;
    if (vl53l5cx_get_ranging_data(&_dev, &results)) return false;

    uint16_t dist[VL53L5CX_RESOLUTION_8X8];
    for (int i = 0; i < VL53L5CX_RESOLUTION_8X8; i++) {
        int16_t d = results.distance_mm[i];
        dist[i] = (d < 0) ? 0U : (uint16_t)d;
    }
    can_pack_multizone(CAN_ID_L5CX_BASE, dist, VL53L5CX_RESOLUTION_8X8, rb);
    return true;
}
