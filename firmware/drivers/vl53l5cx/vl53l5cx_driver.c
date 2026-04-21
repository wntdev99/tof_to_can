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
    uint8_t st = vl53l5cx_init(&_dev);
    if (st) {
        printf("[L5CX] init failed (err=%d)\n", st);
        return false;
    }

    /* init 완료 후 0x50으로 이동 — 0x52를 L4CD 핫플러그용으로 해방
     * vl53l5cx_set_i2c_address: page0→reg[0x04]=new7bit→page2 순서 */
    if (i2c_addr8 == 0x52) {
        vl53l5cx_set_i2c_address(&_dev, 0x50);
        printf("[L5CX] addr moved 0x52→0x50\n");
    }

    vl53l5cx_set_resolution(&_dev, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&_dev, 10);

    drv->initialized = true;
    printf("[L5CX] OK @ 0x%02X\n", (uint8_t)_dev.platform.address);
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

#ifdef L5CX_DEBUG_SERIAL
    printf("[L5CX] 8x8 distance map (mm):\n");
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++)
            printf("%4u ", dist[row * 8 + col]);
        printf("\n");
    }
    printf("\n");
#endif

    can_pack_multizone(CAN_ID_L5CX_BASE, dist, VL53L5CX_RESOLUTION_8X8, rb);
    return true;
}
