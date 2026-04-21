#include "tmf8828_driver.h"
#include "../../can/can_tx.h"
#include "../../can/can_protocol.h"
#include "../../sensor/sensor_manager.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

/* ── I2C 헬퍼 (8-bit 레지스터 주소) ─────────────────────── */

#define TMF_TIMEOUT_US  5000u

static int tmf_wr(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_timeout_us(SENSOR_I2C_PORT, TMF8828_I2C_ADDR, buf, 2, false, TMF_TIMEOUT_US);
}

static int tmf_rd(uint8_t reg, uint8_t *val) {
    if (i2c_write_timeout_us(SENSOR_I2C_PORT, TMF8828_I2C_ADDR, &reg, 1, true, TMF_TIMEOUT_US) != 1)
        return -1;
    return i2c_read_timeout_us(SENSOR_I2C_PORT, TMF8828_I2C_ADDR, val, 1, false, TMF_TIMEOUT_US);
}

static int tmf_rd_burst(uint8_t reg, uint8_t *buf, uint32_t len) {
    if (i2c_write_timeout_us(SENSOR_I2C_PORT, TMF8828_I2C_ADDR, &reg, 1, true, TMF_TIMEOUT_US) != 1)
        return -1;
    return i2c_read_timeout_us(SENSOR_I2C_PORT, TMF8828_I2C_ADDR, buf, len, false,
                                (uint32_t)(len) * 30u + TMF_TIMEOUT_US);
}

/* ── 초기화 ──────────────────────────────────────────────── */

bool tmf8828_drv_init(TMF8828_Drv *drv) {
    drv->initialized = false;
    drv->ranging     = false;

    /* EN 핀은 sensor_manager에서 HIGH로 이미 설정됨 */
    sleep_ms(5);

    /* CPU 활성화 */
    tmf_wr(TMF8828_REG_ENABLE, 0x01);
    sleep_ms(5);

    /* Measurement App 로딩 대기 (최대 200ms) */
    uint8_t appid = 0;
    for (int i = 0; i < 20; i++) {
        tmf_rd(TMF8828_REG_APPID, &appid);
        if (appid == TMF8828_APPID_MEASUREMENT)
            break;
        sleep_ms(10);
    }

    if (appid != TMF8828_APPID_MEASUREMENT) {
        printf("[TMF8828] app not ready (APPID=0x%02X)\n", appid);
        return false;
    }

    drv->initialized = true;
    printf("[TMF8828] init OK\n");
    return true;
}

bool tmf8828_drv_start(TMF8828_Drv *drv) {
    if (!drv->initialized)
        return false;

    /* 연속 측정 시작 */
    tmf_wr(TMF8828_REG_CMD_STAT, TMF8828_CMD_START);
    sleep_ms(5);

    drv->ranging = true;
    return true;
}

/* ── 폴링 + CAN push ─────────────────────────────────────── */

bool tmf8828_drv_poll_and_push(TMF8828_Drv *drv, ring_buffer_t *rb) {
    if (!drv->ranging)
        return false;

    /* 결과 개수 확인 */
    uint8_t nresults = 0;
    if (tmf_rd(TMF8828_REG_RESULT_NUMB, &nresults) < 0 || nresults == 0)
        return false;

    if (nresults > TMF8828_MAX_RESULTS)
        nresults = TMF8828_MAX_RESULTS;

    /*
     * 결과 데이터 포맷 (per entry, 6 bytes):
     *   byte 0   : channel
     *   byte 1   : sub_capture
     *   byte 2-3 : distance_mm (uint16_t, little-endian)
     *   byte 4   : confidence
     *   byte 5   : 예약
     */
    static uint8_t raw[TMF8828_MAX_RESULTS * TMF8828_RESULT_ENTRY_SIZE];
    uint32_t read_len = (uint32_t)nresults * TMF8828_RESULT_ENTRY_SIZE;
    if (tmf_rd_burst(TMF8828_REG_RESULT_BASE, raw, read_len) < 0)
        return false;

    /* 인터럽트 클리어 */
    tmf_wr(TMF8828_REG_CMD_STAT, TMF8828_CMD_CLR_INT);

    /* distance_mm 추출 → uint16_t 배열 */
    static uint16_t dist[TMF8828_MAX_RESULTS];
    for (uint8_t i = 0; i < nresults; i++) {
        uint8_t *e = raw + i * TMF8828_RESULT_ENTRY_SIZE;
        dist[i] = (uint16_t)e[2] | ((uint16_t)e[3] << 8);
    }

    can_pack_tmf8828(dist, nresults, rb);
    return true;
}
