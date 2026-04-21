#pragma once
#include <stdint.h>
#include <string.h>

/*
 * ST ULD VL53L8CX 표준 platform 인터페이스.
 * ST ULD 원본 파일들(vl53l8cx_api.h/c 등)을
 * 이 디렉토리에 복사한 후 빌드하십시오.
 *
 * ST ULD 다운로드:
 *   https://www.st.com/en/embedded-software/stsw-img040.html
 *   (STSW-IMG040 — VL53L8CX ULD)
 *
 * I2C 주소: 기본 0x52 → 부팅 시 0x58 으로 재할당 (sensor_manager)
 */

typedef struct {
    uint16_t address;   /* I2C 8-bit 주소 (7-bit << 1) */
} VL53L8CX_Platform;

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t *p_value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t value);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform);
void    VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs);
