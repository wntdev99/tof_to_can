#include "sensor_manager.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

/* ── 저수준 I2C 헬퍼 ─────────────────────────────────────── */

static bool i2c_probe(uint8_t addr7) {
    uint8_t dummy;
    int ret = i2c_read_blocking(SENSOR_I2C_PORT, addr7, &dummy, 1, false);
    return (ret >= 0);
}

/* ST 센서: 16-bit 레지스터 주소, 1바이트 읽기 */
static bool st_read_reg16(uint8_t addr7, uint16_t reg, uint8_t *val) {
    uint8_t buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_blocking(SENSOR_I2C_PORT, addr7, buf, 2, true) != 2)
        return false;
    return (i2c_read_blocking(SENSOR_I2C_PORT, addr7, val, 1, false) == 1);
}

/* ── 제어 핀 초기화 ──────────────────────────────────────── */

static void ctrl_pins_init(void) {
    /* L7CX, L8CX LPn LOW → I2C 버스에서 숨김 */
    gpio_init(PIN_L7CX_CS); gpio_set_dir(PIN_L7CX_CS, GPIO_OUT); gpio_put(PIN_L7CX_CS, 0);
    gpio_init(PIN_L8CX_CS); gpio_set_dir(PIN_L8CX_CS, GPIO_OUT); gpio_put(PIN_L8CX_CS, 0);
    /* TMF8828 EN HIGH → 활성화 (active-high) */
    gpio_init(PIN_TMF_EN);  gpio_set_dir(PIN_TMF_EN,  GPIO_OUT); gpio_put(PIN_TMF_EN,  1);
    sleep_ms(10);
}

/* ── 0x52 센서 판별 ──────────────────────────────────────── */

sensor_type_t sensor_identify_at_52(void) {
    uint8_t addr7 = ADDR_DEFAULT_ST >> 1;   /* 0x52 → 7-bit 0x29 */

    if (!i2c_probe(addr7)) {
        printf("[SENSOR] 0x52 no response\n");
        return SENSOR_NONE;
    }

    uint8_t model_id = 0;
    if (!st_read_reg16(addr7, ST_REG_MODEL_ID, &model_id)) {
        printf("[SENSOR] 0x52 model_id read failed → possible dual conflict\n");
        return SENSOR_NONE;
    }

    printf("[SENSOR] 0x52 model_id = 0x%02X\n", model_id);

    switch (model_id) {
        case MODEL_ID_L5CX: return SENSOR_VL53L5CX;
        case MODEL_ID_L4CD: return SENSOR_VL53L4CD;
        default:
            printf("[SENSOR] 0x52 unknown model_id 0x%02X\n", model_id);
            return SENSOR_NONE;
    }
}

/* ── L7CX / L8CX 주소 재할당 ────────────────────────────── */

/* ST 센서 I2C 주소 변경 명령 (ULD 공통) */
static bool st_set_i2c_addr(uint8_t current_addr7, uint8_t new_addr7) {
    /* 레지스터 0x0001: 새 I2C 주소 (7-bit << 1) */
    uint8_t buf[3] = { 0x00, 0x01, (uint8_t)(new_addr7 << 1) };
    return (i2c_write_blocking(SENSOR_I2C_PORT, current_addr7, buf, 3, false) == 3);
}

static bool assign_lpn_sensor(uint8_t lpn_pin, uint8_t new_addr7,
                               sensor_info_t *info, sensor_type_t type) {
    uint8_t default7 = ADDR_DEFAULT_ST >> 1;

    gpio_put(lpn_pin, 1);
    sleep_ms(10);

    if (!i2c_probe(default7)) {
        printf("[SENSOR] LPn pin=%d: sensor not found at 0x%02X\n", lpn_pin, ADDR_DEFAULT_ST);
        gpio_put(lpn_pin, 0);
        return false;
    }

    if (!st_set_i2c_addr(default7, new_addr7)) {
        printf("[SENSOR] LPn pin=%d: addr reassign failed\n", lpn_pin);
        gpio_put(lpn_pin, 0);
        return false;
    }

    sleep_ms(2);

    if (!i2c_probe(new_addr7)) {
        printf("[SENSOR] LPn pin=%d: not found at new addr 0x%02X\n", lpn_pin, new_addr7 << 1);
        gpio_put(lpn_pin, 0);
        return false;
    }

    info->type      = type;
    info->i2c_addr  = new_addr7;
    info->present   = true;
    info->initialized = false;
    printf("[SENSOR] %s assigned to 0x%02X\n",
           type == SENSOR_VL53L7CX ? "L7CX" : "L8CX", new_addr7 << 1);
    return true;
}

/* ── 공개 API ────────────────────────────────────────────── */

bool sensor_manager_init(sensor_table_t *table) {
    memset(table, 0, sizeof(*table));

    /* I2C 초기화 */
    i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_SPEED);
    gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_I2C_SDA);
    gpio_pull_up(SENSOR_I2C_SCL);

    ctrl_pins_init();

    /* 1. 0x52 센서 판별 */
    sensor_type_t type52 = sensor_identify_at_52();
    if (type52 != SENSOR_NONE) {
        table->slot_52.type     = type52;
        table->slot_52.i2c_addr = ADDR_DEFAULT_ST >> 1;
        table->slot_52.present  = true;
        table->count++;
    }

    /* 2. TMF8828 확인 (0x41, EN=HIGH 상태) */
    uint8_t tmf7 = ADDR_TMF8828;   /* TMF8828 주소는 이미 7-bit */
    if (i2c_probe(tmf7)) {
        table->slot_tmf.type     = SENSOR_TMF8828;
        table->slot_tmf.i2c_addr = tmf7;
        table->slot_tmf.present  = true;
        table->count++;
        printf("[SENSOR] TMF8828 found at 0x%02X\n", ADDR_TMF8828);
    }

    /* 3. L7CX: LPn HIGH → 주소 재할당 */
    if (assign_lpn_sensor(PIN_L7CX_CS, ADDR_L7CX >> 1, &table->slot_l7cx, SENSOR_VL53L7CX))
        table->count++;

    /* 4. L8CX: LPn HIGH → 주소 재할당 */
    if (assign_lpn_sensor(PIN_L8CX_CS, ADDR_L8CX >> 1, &table->slot_l8cx, SENSOR_VL53L8CX))
        table->count++;

    printf("[SENSOR] Total sensors found: %d\n", table->count);
    return (table->count > 0);
}

bool sensor_assign_addresses(sensor_table_t *table) {
    (void)table;
    return true;
}
