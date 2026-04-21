#pragma once
#include <stdint.h>
#include <string.h>

/*
 * ST ULD VL53L4CD 표준 platform 인터페이스.
 * ST ULD 원본 파일들(vl53l4cd_api.h/c 등)을
 * 이 디렉토리에 복사한 후 빌드하십시오.
 *
 * ST ULD 다운로드:
 *   https://www.st.com/en/embedded-software/stsw-img026.html
 *   (STSW-IMG026 — VL53L4CD ULD)
 */

typedef struct {
    uint16_t address;   /* I2C 8-bit 주소 (7-bit << 1), e.g. 0x52 */
} VL53L4CD_Platform;

/* VL53L4CD ULD API 인수 타입 */
typedef VL53L4CD_Platform *Dev_t;

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAddress, uint8_t *p_value);
uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAddress, uint8_t value);
uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAddress, uint16_t *p_value);
uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAddress, uint16_t value);
uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAddress, uint32_t *p_value);
uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAddress, uint32_t value);
uint8_t VL53L4CD_WrMulti(Dev_t dev, uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L4CD_RdMulti(Dev_t dev, uint16_t RegisterAddress,
                          uint8_t *p_values, uint32_t size);
uint8_t VL53L4CD_Reset_Sensor(Dev_t dev);
void    VL53L4CD_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs);
