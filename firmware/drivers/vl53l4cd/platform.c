#include "platform.h"
#include "../st_platform_common.h"
#include "pico/stdlib.h"

static inline uint8_t addr7(Dev_t dev) { return (uint8_t)(dev->address >> 1); }

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t reg, uint8_t *val) {
    return st_rd_byte(addr7(dev), reg, val);
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t reg, uint8_t val) {
    return st_wr_byte(addr7(dev), reg, val);
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t reg, uint16_t *val) {
    uint8_t buf[2];
    if (st_rd_multi(addr7(dev), reg, buf, 2)) return 1;
    *val = ((uint16_t)buf[0] << 8) | buf[1];   /* big-endian */
    return 0;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t reg, uint16_t val) {
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return st_wr_multi(addr7(dev), reg, buf, 2);
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t reg, uint32_t *val) {
    uint8_t buf[4];
    if (st_rd_multi(addr7(dev), reg, buf, 4)) return 1;
    *val = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
         | ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    return 0;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t reg, uint32_t val) {
    uint8_t buf[4] = {
        (uint8_t)(val >> 24), (uint8_t)(val >> 16),
        (uint8_t)(val >>  8), (uint8_t)(val & 0xFF)
    };
    return st_wr_multi(addr7(dev), reg, buf, 4);
}

uint8_t VL53L4CD_WrMulti(Dev_t dev, uint16_t reg, uint8_t *vals, uint32_t size) {
    return st_wr_multi(addr7(dev), reg, vals, size);
}

uint8_t VL53L4CD_RdMulti(Dev_t dev, uint16_t reg, uint8_t *vals, uint32_t size) {
    return st_rd_multi(addr7(dev), reg, vals, size);
}

uint8_t VL53L4CD_Reset_Sensor(Dev_t dev) {
    (void)dev;
    return 0;
}

void VL53L4CD_SwapBuffer(uint8_t *buf, uint16_t size) {
    st_swap_buffer(buf, size);
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t ms) {
    (void)dev;
    sleep_ms(ms);
    return 0;
}
