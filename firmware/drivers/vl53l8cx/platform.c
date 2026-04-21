#include "platform.h"
#include "../st_platform_common.h"
#include "pico/stdlib.h"

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *val) {
    return st_rd_byte((uint8_t)(p->address >> 1), reg, val);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t val) {
    return st_wr_byte((uint8_t)(p->address >> 1), reg, val);
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p, uint16_t reg,
                          uint8_t *vals, uint32_t size) {
    return st_wr_multi((uint8_t)(p->address >> 1), reg, vals, size);
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p, uint16_t reg,
                          uint8_t *vals, uint32_t size) {
    return st_rd_multi((uint8_t)(p->address >> 1), reg, vals, size);
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p) {
    (void)p;
    /* LPn(CS 핀, GPIO11) — sensor_manager에서 LOW/HIGH로 관리 */
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buf, uint16_t size) {
    st_swap_buffer(buf, size);
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p, uint32_t ms) {
    (void)p;
    sleep_ms(ms);
    return 0;
}
