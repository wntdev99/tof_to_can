#include "sensor_manager.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#define I2C_TIMEOUT_US  2000u

/* ── I2C 헬퍼 ────────────────────────────────────────────── */

static bool i2c_probe(uint8_t addr7) {
    uint8_t dummy;
    return (i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, &dummy, 1, false, I2C_TIMEOUT_US) >= 0);
}

static bool st_read_reg16(uint8_t addr7, uint16_t reg, uint8_t *val) {
    uint8_t buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_timeout_us(SENSOR_I2C_PORT, addr7, buf, 2, true, I2C_TIMEOUT_US) != 2)
        return false;
    return (i2c_read_timeout_us(SENSOR_I2C_PORT, addr7, val, 1, false, I2C_TIMEOUT_US) == 1);
}

static bool st_set_i2c_addr(uint8_t current_addr7, uint8_t new_addr7) {
    /* ST L5CX/L7CX/L8CX: reg[0x0004] = 7-bit 주소 (ULD set_i2c_address 동일 로직) */
    uint8_t buf[3] = { 0x00, 0x04, new_addr7 };
    return (i2c_write_timeout_us(SENSOR_I2C_PORT, current_addr7, buf, 3, false, I2C_TIMEOUT_US) == 3);
}

/* ── 버스 복구 ───────────────────────────────────────────── */

static void i2c_bus_recover(void) {
    gpio_init(SENSOR_I2C_SCL); gpio_set_dir(SENSOR_I2C_SCL, GPIO_OUT); gpio_put(SENSOR_I2C_SCL, 1);
    gpio_init(SENSOR_I2C_SDA); gpio_set_dir(SENSOR_I2C_SDA, GPIO_IN);  gpio_pull_up(SENSOR_I2C_SDA);
    sleep_us(200);

    bool sda_stuck = !gpio_get(SENSOR_I2C_SDA);
    printf("[I2C] bus recover: SDA=%s\n", sda_stuck ? "LOW(stuck)" : "HIGH(ok)");

    for (int i = 0; i < 36; i++) {
        gpio_put(SENSOR_I2C_SCL, 0); sleep_us(10);
        gpio_put(SENSOR_I2C_SCL, 1); sleep_us(10);
        if (gpio_get(SENSOR_I2C_SDA)) break;
    }
    gpio_set_dir(SENSOR_I2C_SDA, GPIO_OUT);
    gpio_put(SENSOR_I2C_SDA, 0); sleep_us(10);
    gpio_put(SENSOR_I2C_SCL, 1); sleep_us(10);
    gpio_put(SENSOR_I2C_SDA, 1); sleep_us(10);
    gpio_set_dir(SENSOR_I2C_SDA, GPIO_IN);
    sleep_us(200);
}

/* ── 전체 I2C 스캔 (진단용) ─────────────────────────────── */

static void i2c_full_scan(void) {
    printf("[I2C SCAN] 0x00-0x7F:\n");
    bool found = false;
    for (uint8_t a = 1; a <= 0x7E; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(SENSOR_I2C_PORT, a, &dummy, 1, false, I2C_TIMEOUT_US) >= 0) {
            printf("  7-bit=0x%02X (8-bit=0x%02X)\n", a, (uint8_t)(a << 1));
            found = true;
        }
    }
    if (!found) printf("  (없음)\n");
}

/* ── 제어 핀 초기화 ──────────────────────────────────────── */

static void ctrl_pins_init(void) {
    gpio_init(PIN_L7CX_CS); gpio_set_dir(PIN_L7CX_CS, GPIO_OUT); gpio_put(PIN_L7CX_CS, 0);
    gpio_init(PIN_L8CX_CS); gpio_set_dir(PIN_L8CX_CS, GPIO_OUT); gpio_put(PIN_L8CX_CS, 0);
    gpio_init(PIN_TMF_EN);  gpio_set_dir(PIN_TMF_EN,  GPIO_OUT); gpio_put(PIN_TMF_EN,  1);
    sleep_ms(10);
}

/* ── LPn 제어 센서(L7CX/L8CX) 공용 탐지 헬퍼 ────────────────
 *   호출 전제: 0x52 가 비어있어야 함 (caller 가 보장).
 *   동작: LPn=HIGH → 10 ms 대기 → 0x52 probe → 응답하면 target 주소로 이동.
 *   실패 시 LPn 을 LOW 로 되돌려 센서를 버스에서 격리.                   */
static bool detect_lpn_sensor(sensor_table_t *table, sensor_info_t *slot,
                                uint lpn_pin, uint8_t target_addr8,
                                sensor_type_t type, const char *name) {
    uint8_t default7 = ADDR_DEFAULT_ST >> 1;

    gpio_put(lpn_pin, 1);
    sleep_ms(10);

    if (!i2c_probe(default7)) {
        gpio_put(lpn_pin, 0);
        return false;
    }

    uint8_t new7 = target_addr8 >> 1;
    if (!st_set_i2c_addr(default7, new7) || !i2c_probe(new7)) {
        gpio_put(lpn_pin, 0);
        return false;
    }

    slot->type     = type;
    slot->i2c_addr = new7;
    slot->present  = true;
    table->count++;
    printf("[SENSOR] %s → 0x%02X\n", name, target_addr8);
    return true;
}

/* ── 공개 API ────────────────────────────────────────────── */

bool sensor_manager_init_phase1(sensor_table_t *table) {
    memset(table, 0, sizeof(*table));

    i2c_bus_recover();

    i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_SPEED);
    gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_I2C_SDA);
    gpio_pull_up(SENSOR_I2C_SCL);
    sleep_ms(2);

    ctrl_pins_init();
    i2c_full_scan();

    uint8_t default7 = ADDR_DEFAULT_ST >> 1;   /* 0x29 */

    /* 0x52 단일 읽기로 L5CX 판별.
     * L5CX는 page 레지스터(0x7FFF)가 있어 버스 복구 직후 page 상태가
     * 불명확하므로 page 0 강제 리셋 후 model_id(reg 0x0000)를 읽는다. */
    if (i2c_probe(default7)) {
        uint8_t page_buf[3] = { 0x7F, 0xFF, 0x00 };
        i2c_write_timeout_us(SENSOR_I2C_PORT, default7, page_buf, 3, false, I2C_TIMEOUT_US);
        sleep_us(200);

        uint8_t model_id = 0;
        bool read_ok = st_read_reg16(default7, ST_REG_MODEL_ID, &model_id);
        printf("[SENSOR] @ 0x%02X: read_ok=%d model_id=0x%02X\n",
               ADDR_DEFAULT_ST, read_ok, model_id);

        if (!read_ok) {
            /* nostop 실패 후 버스 상태 복구 */
            i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_SPEED);
            gpio_set_function(SENSOR_I2C_SDA, GPIO_FUNC_I2C);
            gpio_set_function(SENSOR_I2C_SCL, GPIO_FUNC_I2C);
            gpio_pull_up(SENSOR_I2C_SDA);
            gpio_pull_up(SENSOR_I2C_SCL);
            sleep_ms(2);
        }

        if (read_ok && model_id == MODEL_ID_L5CX) {
            /* 감지만. 주소 이동은 vl53l5cx_drv_init() 에서 수행 (펌웨어 로딩 후). */
            table->slot_l5cx.type     = SENSOR_VL53L5CX;
            table->slot_l5cx.i2c_addr = default7;   /* 0x29 (8-bit 0x52) */
            table->slot_l5cx.present  = true;
            table->count++;
            printf("[SENSOR] L5CX @ 0x%02X (driver init에서 0x%02X 이동 예정)\n",
                   ADDR_DEFAULT_ST, ADDR_L5CX_ASSIGNED);
        } else {
            /* L5CX 가 아니면 L4CD 로 확정하기 위해 reg 0x010F 확인.
             * (0x52 에 응답하는 미지의 ST 장치 오인식 방지) */
            uint8_t l4cd_id = 0;
            bool l4cd_ok = st_read_reg16(default7, L4CD_REG_MODEL_ID, &l4cd_id);
            if (l4cd_ok && l4cd_id == MODEL_ID_L4CD) {
                table->slot_l4cd.type     = SENSOR_VL53L4CD;
                table->slot_l4cd.i2c_addr = default7;
                table->slot_l4cd.present  = true;
                table->count++;
                printf("[SENSOR] L4CD @ 0x%02X (driver init에서 0x%02X 이동 예정)\n",
                       ADDR_DEFAULT_ST, ADDR_L4CD_ASSIGNED);
            } else {
                printf("[SENSOR] WARN: 0x%02X 에 알 수 없는 장치 "
                       "(reg[0]=0x%02X, reg[0x10F]=0x%02X ok=%d) — 등록 안 함\n",
                       ADDR_DEFAULT_ST, model_id, l4cd_id, l4cd_ok);
            }
        }
    } else {
        printf("[SENSOR] no device @ 0x%02X (L4CD/L5CX 핫플러그 대기)\n", ADDR_DEFAULT_ST);
    }

    return true;
}

bool sensor_manager_init_phase2(sensor_table_t *table) {
    uint8_t default7 = ADDR_DEFAULT_ST >> 1;

    /* phase2 안전 전제: 0x52 는 비어 있어야 한다.
     *   이 시점엔 L5CX / L4CD 모두 drv_init 에서 각자 할당 주소(0x50/0x54)
     *   로 이동이 완료됐어야 하며, 0x52 가 여전히 점유되어 있으면 이동이
     *   실패한 것. 0x52 를 건드리는 L7/L8 탐지는 위험하므로 생략.           */
    bool addr52_busy = i2c_probe(default7);
    if (addr52_busy) {
        bool l5_stuck = table->slot_l5cx.present && !table->slot_l5cx.initialized;
        bool l4_stuck = table->slot_l4cd.present && !table->slot_l4cd.initialized;
        if (l5_stuck) {
            printf("[SENSOR] WARN: L5CX 이동 실패(0x52 점유) — L7/L8 탐지 스킵\n");
        } else if (l4_stuck) {
            printf("[SENSOR] WARN: L4CD 이동 실패(0x52 점유) — L7/L8 탐지 스킵\n");
        } else {
            printf("[SENSOR] WARN: 0x52 미상 점유 — L7/L8 탐지 스킵\n");
        }
    } else if (table->slot_l5cx.present || table->slot_l4cd.present) {
        printf("[SENSOR] 0x52 free — 추가 핫플러그 대기중\n");
    }

    /* TMF8828 (0x41) — 0x52 와 무관하므로 항상 시도 */
    if (i2c_probe(ADDR_TMF8828)) {
        table->slot_tmf.type     = SENSOR_TMF8828;
        table->slot_tmf.i2c_addr = ADDR_TMF8828;
        table->slot_tmf.present  = true;
        table->count++;
        printf("[SENSOR] TMF8828 @ 0x%02X\n", (uint8_t)(ADDR_TMF8828 << 1));
    }

    /* L7CX / L8CX: LPn HIGH → 0x52 → 각자 할당 주소 (0x52 비어있을 때만) */
    if (!addr52_busy) {
        detect_lpn_sensor(table, &table->slot_l7cx,
                          PIN_L7CX_CS, ADDR_L7CX, SENSOR_VL53L7CX, "L7CX");
        detect_lpn_sensor(table, &table->slot_l8cx,
                          PIN_L8CX_CS, ADDR_L8CX, SENSOR_VL53L8CX, "L8CX");
    }

    printf("[SENSOR] Total: %d\n", table->count);
    return (table->count > 0);
}

/* ── 핫플러그 감지 (poll 루프에서 2초마다 호출) ────────────
 * L5CX가 0x50으로 이동한 뒤 0x52에 L4CD가 연결되면 감지.
 * 반환값: 새 센서가 감지되면 true (드라이버 초기화 필요)     */
bool sensor_manager_poll_hotplug(sensor_table_t *table) {
    bool registered = false;
    uint8_t default7 = ADDR_DEFAULT_ST >> 1;

    /* 1. TMF8828 (0x41) — 0x52 와 독립된 주소라 언제든 시도 가능 */
    if (!table->slot_tmf.present && i2c_probe(ADDR_TMF8828)) {
        table->slot_tmf.type     = SENSOR_TMF8828;
        table->slot_tmf.i2c_addr = ADDR_TMF8828;
        table->slot_tmf.present  = true;
        table->count++;
        printf("[SENSOR] TMF8828 핫플러그 감지 @ 0x%02X\n",
               (uint8_t)(ADDR_TMF8828 << 1));
        registered = true;
    }

    /* 2. L4CD @ 0x52 — model_id 검증 후 등록.
     *    L4CD 가 아직 0x52 에 있으면(이동 전) 아래의 L7/L8 LPn 토글과
     *    버스 충돌할 수 있으므로 여기서 early return.                     */
    if (!table->slot_l4cd.present && i2c_probe(default7)) {
        uint8_t l4cd_id = 0;
        if (!st_read_reg16(default7, L4CD_REG_MODEL_ID, &l4cd_id)) {
            /* read 실패 — 부팅 중으로 간주, 다음 cycle 재시도 */
            return registered;
        }
        if (l4cd_id == MODEL_ID_L4CD) {
            table->slot_l4cd.type     = SENSOR_VL53L4CD;
            table->slot_l4cd.i2c_addr = default7;
            table->slot_l4cd.present  = true;
            table->count++;
            printf("[SENSOR] L4CD 핫플러그 감지 @ 0x%02X\n", ADDR_DEFAULT_ST);
            registered = true;
        } else {
            printf("[SENSOR] WARN: 0x%02X 에 L4CD 아닌 장치 "
                   "(reg[0x10F]=0x%02X) — 무시\n", ADDR_DEFAULT_ST, l4cd_id);
        }
        /* L4CD drv_init 으로 0x52 가 비워지기 전까진 L7/L8 은 건들지 않음 */
        return registered;
    }

    /* 3. L7CX / L8CX — LPn↑ 후 0x52 probe → 주소 이동.
     *    0x52 가 비어있다는 조건은 바로 위 블록에서 확인됨.               */
    if (!table->slot_l7cx.present) {
        if (detect_lpn_sensor(table, &table->slot_l7cx,
                               PIN_L7CX_CS, ADDR_L7CX,
                               SENSOR_VL53L7CX, "L7CX 핫플러그")) {
            registered = true;
        }
    }
    if (!table->slot_l8cx.present) {
        if (detect_lpn_sensor(table, &table->slot_l8cx,
                               PIN_L8CX_CS, ADDR_L8CX,
                               SENSOR_VL53L8CX, "L8CX 핫플러그")) {
            registered = true;
        }
    }

    return registered;
}
