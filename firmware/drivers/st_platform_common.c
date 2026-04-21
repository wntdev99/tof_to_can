#include "st_platform_common.h"
#include "hardware/i2c.h"
#include "../sensor/sensor_manager.h"
#include <string.h>

/*
 * ST ULD firmware 업로드 시 최대 ~4KB를 단일 WrMulti로 전송함.
 * i2c_write_blocking은 하나의 트랜잭션으로 전송해야 하므로
 * [regHi, regLo, data...] 를 합친 정적 버퍼를 사용한다.
 */
static uint8_t _wr_buf[4096 + 2];

uint8_t st_rd_byte(uint8_t addr7, uint16_t reg, uint8_t *val) {
    uint8_t r[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_blocking(SENSOR_I2C_PORT, addr7, r, 2, true) != 2)
        return 1;
    return (i2c_read_blocking(SENSOR_I2C_PORT, addr7, val, 1, false) == 1) ? 0 : 1;
}

uint8_t st_wr_byte(uint8_t addr7, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    return (i2c_write_blocking(SENSOR_I2C_PORT, addr7, buf, 3, false) == 3) ? 0 : 1;
}

uint8_t st_wr_multi(uint8_t addr7, uint16_t reg, const uint8_t *data, uint32_t size) {
    if (size > sizeof(_wr_buf) - 2)
        return 1;
    _wr_buf[0] = (uint8_t)(reg >> 8);
    _wr_buf[1] = (uint8_t)(reg & 0xFF);
    memcpy(_wr_buf + 2, data, size);
    int ret = i2c_write_blocking(SENSOR_I2C_PORT, addr7, _wr_buf, size + 2, false);
    return (ret == (int)(size + 2)) ? 0 : 1;
}

uint8_t st_rd_multi(uint8_t addr7, uint16_t reg, uint8_t *data, uint32_t size) {
    uint8_t r[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_blocking(SENSOR_I2C_PORT, addr7, r, 2, true) != 2)
        return 1;
    int ret = i2c_read_blocking(SENSOR_I2C_PORT, addr7, data, size, false);
    return (ret == (int)size) ? 0 : 1;
}

void st_swap_buffer(uint8_t *buf, uint16_t size) {
    /* ST ULD가 요구하는 32-bit word byte-swap (big-endian → little-endian) */
    for (uint16_t i = 0; i + 3 < size; i += 4) {
        uint8_t t;
        t = buf[i];     buf[i]     = buf[i + 3]; buf[i + 3] = t;
        t = buf[i + 1]; buf[i + 1] = buf[i + 2]; buf[i + 2] = t;
    }
}
