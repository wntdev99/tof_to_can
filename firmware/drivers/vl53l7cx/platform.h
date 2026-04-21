#pragma once
#include <stdint.h>
#include <string.h>

/*
 * ST ULD VL53L7CX 표준 platform 인터페이스.
 * ST ULD 원본 파일들(vl53l7cx_api.h/c 등)을
 * 이 디렉토리에 복사한 후 빌드하십시오.
 *
 * ST ULD 다운로드:
 *   https://www.st.com/en/embedded-software/stsw-img039.html
 *   (STSW-IMG039 — VL53L7CX ULD)
 *
 * I2C 주소: 기본 0x52 → 부팅 시 0x56 으로 재할당 (sensor_manager)
 */

typedef struct {
    uint16_t address;   /* I2C 8-bit 주소 (7-bit << 1) */
} VL53L7CX_Platform;

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t *p_value);
uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t value);
uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L7CX_Reset_Sensor(VL53L7CX_Platform *p_platform);
void    VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *p_platform, uint32_t TimeMs);
