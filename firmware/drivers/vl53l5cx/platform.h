#pragma once
#include <stdint.h>
#include <string.h>

/*
 * ST ULD VL53L5CX 표준 platform 인터페이스.
 * ST ULD 원본 파일들(vl53l5cx_api.h/c, vl53l5cx_buffers.h 등)을
 * 이 디렉토리에 복사한 후 빌드하십시오.
 *
 * ST ULD 다운로드:
 *   https://www.st.com/en/embedded-software/stsw-img023.html
 *   (STSW-IMG023 — VL53L5CX ULD)
 */

typedef struct {
    uint16_t address;   /* I2C 8-bit 주소 (7-bit << 1), e.g. 0x52 */
} VL53L5CX_Platform;

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t *p_value);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t value);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform,
                          uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform);
void    VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);
