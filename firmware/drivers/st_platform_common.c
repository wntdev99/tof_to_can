#include "st_platform_common.h"
#include "hardware/i2c.h"
#include "../sensor/sensor_manager.h"
#include <stdio.h>
#include <string.h>

/*
 * ST ULD firmware 업로드 시 최대 ~4KB를 단일 WrMulti로 전송함.
 * i2c_write_blocking은 하나의 트랜잭션으로 전송해야 하므로
 * [regHi, regLo, data...] 를 합친 정적 버퍼를 사용한다.
 */
static uint8_t _wr_buf[4096 + 2];

/* Per-byte I2C 시간 = ceil(9 bits / SCL_Hz) + 오버헤드 (clock stretching 여유).
 *  100 kHz → 90 + 50 = 140 µs/byte
 *  400 kHz → 23 + 50 = 73  µs/byte
 *  1 MHz   → 9  + 50 = 59  µs/byte
 * SENSOR_I2C_SPEED 를 바꾸면 자동으로 scale.                               */
#define ST_BYTE_TIME_US    ((uint32_t)((9ULL * 1000000ULL + SENSOR_I2C_SPEED - 1U) \
                                        / (uint32_t)SENSOR_I2C_SPEED) + 50u)
#define ST_FIXED_SLACK_US  100000u   /* 트랜잭션 고정 오버헤드 + MCU 지터 */
#define st_timeout_us(n)   ((uint32_t)(n) * ST_BYTE_TIME_US + ST_FIXED_SLACK_US)

/* 단일 바이트 R/W 용 고정 timeout: SCL_Hz 와 무관하게 안전한 5 ms */
#define ST_TIMEOUT_SMALL_US  5000u

uint8_t st_rd_byte(uint8_t addr7, uint16_t reg, uint8_t *val) {
    uint8_t r[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_timeout_us(SENSOR_I2C_PORT, addr7, r, 2, true, ST_TIMEOUT_SMALL_US) != 2) {
        printf("[ST] rd_byte wr fail @0x%02X reg=0x%04X\n", addr7, reg);
        return 1;
    }
    int ret = i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, val, 1, false, ST_TIMEOUT_SMALL_US);
    if (ret != 1) {
        printf("[ST] rd_byte rd fail @0x%02X reg=0x%04X ret=%d\n", addr7, reg, ret);
        return 1;
    }
    return 0;
}

uint8_t st_wr_byte(uint8_t addr7, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    int ret = i2c_write_timeout_us(SENSOR_I2C_PORT, addr7, buf, 3, false, ST_TIMEOUT_SMALL_US);
    if (ret != 3) {
        printf("[ST] wr_byte fail @0x%02X reg=0x%04X val=0x%02X ret=%d\n",
               addr7, reg, val, ret);
        return 1;
    }
    return 0;
}

/* VL53L5CX 펌웨어 업로드는 32KB 단위로 WrMulti 호출함 → 4KB 청크로 분할 전송 */
#define ST_MAX_CHUNK  4094u

uint8_t st_wr_multi(uint8_t addr7, uint16_t reg, const uint8_t *data, uint32_t size) {
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = (size - offset) < ST_MAX_CHUNK ? (size - offset) : ST_MAX_CHUNK;
        uint16_t cur_reg = (uint16_t)(reg + offset);
        _wr_buf[0] = (uint8_t)(cur_reg >> 8);
        _wr_buf[1] = (uint8_t)(cur_reg & 0xFF);
        memcpy(_wr_buf + 2, data + offset, chunk);
        uint32_t tmo = st_timeout_us(chunk + 2);
        int ret = i2c_write_timeout_us(SENSOR_I2C_PORT, addr7, _wr_buf, chunk + 2, false, tmo);
        if (ret != (int)(chunk + 2)) {
            printf("[ST] wr_multi fail @0x%02X reg=0x%04X chunk=%u tmo_us=%u ret=%d\n",
                   addr7, cur_reg, (unsigned)chunk, (unsigned)tmo, ret);
            return 1;
        }
        offset += chunk;
    }
    return 0;
}

uint8_t st_rd_multi(uint8_t addr7, uint16_t reg, uint8_t *data, uint32_t size) {
    uint8_t r[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_timeout_us(SENSOR_I2C_PORT, addr7, r, 2, true, ST_TIMEOUT_SMALL_US) != 2) {
        printf("[ST] rd_multi wr fail @0x%02X reg=0x%04X size=%u\n",
               addr7, reg, (unsigned)size);
        return 1;
    }
    uint32_t tmo = st_timeout_us(size);
    int ret = i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, data, size, false, tmo);
    if (ret != (int)size) {
        printf("[ST] rd_multi rd fail @0x%02X reg=0x%04X size=%u tmo_us=%u ret=%d\n",
               addr7, reg, (unsigned)size, (unsigned)tmo, ret);
        return 1;
    }
    return 0;
}

void st_swap_buffer(uint8_t *buf, uint16_t size) {
    /* ST ULD가 요구하는 32-bit word byte-swap (big-endian → little-endian) */
    for (uint16_t i = 0; i + 3 < size; i += 4) {
        uint8_t t;
        t = buf[i];     buf[i]     = buf[i + 3]; buf[i + 3] = t;
        t = buf[i + 1]; buf[i + 1] = buf[i + 2]; buf[i + 2] = t;
    }
}
