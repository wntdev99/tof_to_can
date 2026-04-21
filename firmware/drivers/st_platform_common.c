#include "st_platform_common.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "../sensor/sensor_manager.h"
#include <stdio.h>
#include <string.h>

/* ── 런타임 I2C 버스 recovery ──────────────────────────────
 *   STEMMA QT 같은 핫플러그 커넥터 삽입 순간 SDA/SCL 가 글리치되어
 *   slave 가 mid-byte 상태에서 SDA 를 LOW 에 잡고 있으면 이후 모든
 *   START 가 실패한다. 이때 SCL 를 강제로 토글해 slave 를 풀어주고
 *   I2C peripheral 을 재초기화해 런타임에 자동 복구한다.                  */
void st_bus_recover_if_stuck(void) {
    /* 빠른 경로: 현재 SDA/SCL 을 입력으로 읽어 상태 확인.
     *   GPIO_FUNC_I2C 에서는 내부 풀업 + 슬레이브만 전압을 low 로 끌 수 있음.
     *   SDA == HIGH 이면 버스 idle → 아무 것도 안 함.                      */
    gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(SENSOR_I2C_SDA, GPIO_IN);
    gpio_pull_up(SENSOR_I2C_SDA);

    bool sda_stuck = !gpio_get(SENSOR_I2C_SDA);
    if (!sda_stuck) {
        /* 정상 — I2C 기능으로 복원 후 반환 */
        gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_I2C);
        return;
    }

    printf("[ST] bus stuck (SDA=LOW) — recovering\n");

    /* SCL 을 수동 출력으로 전환 */
    gpio_set_function(SENSOR_I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(SENSOR_I2C_SCL, GPIO_OUT);
    gpio_put(SENSOR_I2C_SCL, 1);
    sleep_us(10);

    /* 최대 36 clock 토글해 slave 를 해제 */
    for (int i = 0; i < 36; i++) {
        gpio_put(SENSOR_I2C_SCL, 0); sleep_us(10);
        gpio_put(SENSOR_I2C_SCL, 1); sleep_us(10);
        if (gpio_get(SENSOR_I2C_SDA)) break;
    }

    /* STOP 조건 (SDA LOW→HIGH while SCL HIGH) 을 수동 생성 */
    gpio_set_dir(SENSOR_I2C_SDA, GPIO_OUT);
    gpio_put(SENSOR_I2C_SDA, 0); sleep_us(10);
    gpio_put(SENSOR_I2C_SCL, 1); sleep_us(10);
    gpio_put(SENSOR_I2C_SDA, 1); sleep_us(10);

    /* I2C 기능으로 복원 + peripheral 재초기화 */
    gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_I2C_SDA);
    gpio_pull_up(SENSOR_I2C_SCL);
    i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_SPEED);
    sleep_ms(2);

    printf("[ST] bus recovered, SDA=%s\n", gpio_get(SENSOR_I2C_SDA) ? "HIGH" : "LOW(still stuck)");
}

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
        st_bus_recover_if_stuck();
        return 1;
    }
    int ret = i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, val, 1, false, ST_TIMEOUT_SMALL_US);
    if (ret != 1) {
        printf("[ST] rd_byte rd fail @0x%02X reg=0x%04X ret=%d\n", addr7, reg, ret);
        st_bus_recover_if_stuck();
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
        st_bus_recover_if_stuck();
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
            st_bus_recover_if_stuck();
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
        st_bus_recover_if_stuck();
        return 1;
    }
    uint32_t tmo = st_timeout_us(size);
    int ret = i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, data, size, false, tmo);
    if (ret != (int)size) {
        printf("[ST] rd_multi rd fail @0x%02X reg=0x%04X size=%u tmo_us=%u ret=%d\n",
               addr7, reg, (unsigned)size, (unsigned)tmo, ret);
        st_bus_recover_if_stuck();
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
