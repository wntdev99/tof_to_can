#pragma once
#include <stdint.h>
#include <stdbool.h>

/* I2C 설정 */
#define SENSOR_I2C_PORT     i2c1
#define SENSOR_I2C_SDA      2
#define SENSOR_I2C_SCL      3
#define SENSOR_I2C_SPEED    100000U

/* 제어 핀 */
#define PIN_L7CX_CS      10   /* VL53L7CX LPn */
#define PIN_L8CX_CS      11   /* VL53L8CX LPn */
#define PIN_TMF_EN       12   /* TMF8828  EN  */

/* I2C 주소 (8-bit)
 *   0x52 = 모든 ST ToF 의 기본 주소 → 부팅/핫플러그 시 모델별로 전용
 *   주소(0x50/0x54/0x56/0x58)로 이동해 0x52 를 항상 '탐지 슬롯'으로 비움. */
#define ADDR_DEFAULT_ST      0x52
#define ADDR_L5CX_ASSIGNED   0x50
#define ADDR_L4CD_ASSIGNED   0x54
#define ADDR_L7CX            0x56
#define ADDR_L8CX            0x58
#define ADDR_TMF8828         0x41

/* Model ID
 *   L5CX/L7CX/L8CX 는 동일 silicon 으로 reg[0x0000] = 0xF0 (구분은 LPn 로)
 *   L4CD 는 reg[0x010F] = 0xEB (상위 바이트)                              */
#define ST_REG_MODEL_ID      0x0000
#define L4CD_REG_MODEL_ID    0x010F
#define MODEL_ID_L5CX        0xF0
#define MODEL_ID_L4CD        0xEB

typedef enum {
    SENSOR_NONE = 0,
    SENSOR_VL53L5CX,
    SENSOR_VL53L4CD,
    SENSOR_VL53L7CX,
    SENSOR_VL53L8CX,
    SENSOR_TMF8828,
} sensor_type_t;

typedef struct {
    sensor_type_t type;
    uint8_t       i2c_addr;   /* 7-bit */
    bool          present;
    bool          initialized;
} sensor_info_t;

typedef struct {
    sensor_info_t slot_l5cx;
    sensor_info_t slot_l4cd;
    sensor_info_t slot_l7cx;
    sensor_info_t slot_l8cx;
    sensor_info_t slot_tmf;
    int           count;
} sensor_table_t;

/* 2단계 초기화
 *  phase1: I2C 버스 복구/스캔 + 0x52 에서 L5CX 또는 L4CD 감지 (LPn 건드리지 않음)
 *  phase2: TMF8828(0x41) + L7CX(LPn↑, 0x52→0x56) + L8CX(LPn↑, 0x52→0x58)
 *
 * 사용 순서:
 *   sensor_manager_init_phase1(&t);
 *   if (t.slot_l5cx.present) vl53l5cx_drv_init(...);   // ← 0x52→0x50 이동 실수행
 *   sensor_manager_init_phase2(&t);                    // 0x52 비어 있어야 안전
 *
 * phase2는 0x52 가 비어 있다는 전제를 요구한다. L5CX 가 감지되었으나 아직
 * initialized==false 이면(드라이버 init 실패) phase2 는 L7/L8 탐지를 생략해
 * L5CX 와의 버스 충돌을 회피한다.                                              */
bool sensor_manager_init_phase1(sensor_table_t *table);
bool sensor_manager_init_phase2(sensor_table_t *table);

/* poll 루프에서 주기적으로 호출 — L4CD 핫플러그 감지 */
bool sensor_manager_poll_hotplug(sensor_table_t *table);
