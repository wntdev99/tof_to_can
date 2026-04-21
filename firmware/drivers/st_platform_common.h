#pragma once
#include <stdint.h>

/*
 * ST 계열 ToF 센서 공통 I2C 헬퍼
 * - 16-bit 레지스터 주소 (big-endian)
 * - Pico SDK i2c_write/read_blocking 래핑
 * - 반환값: 0 = 성공, 1 = 오류
 */

uint8_t st_rd_byte(uint8_t addr7, uint16_t reg, uint8_t *val);
uint8_t st_wr_byte(uint8_t addr7, uint16_t reg, uint8_t val);
uint8_t st_wr_multi(uint8_t addr7, uint16_t reg, const uint8_t *data, uint32_t size);
uint8_t st_rd_multi(uint8_t addr7, uint16_t reg, uint8_t *data, uint32_t size);
void    st_swap_buffer(uint8_t *buf, uint16_t size);

/* I2C transaction 실패 후 호출. SDA 가 LOW 에 stuck 되어 있으면 SCL 을
 * 최대 36 clock 토글해 slave 의 mid-byte 상태를 해제하고 I2C peripheral
 * 을 재초기화한다. SDA 가 HIGH 면 no-op (정상 NACK 등). 정상 NACK 경로
 * 에서 호출해도 안전.                                                     */
void    st_bus_recover_if_stuck(void);
