#pragma once
#include <stdint.h>
#include <stdbool.h>

/* I2C 설정 */
#define SENSOR_I2C_PORT     i2c1
#define SENSOR_I2C_SDA      2
#define SENSOR_I2C_SCL      3
#define SENSOR_I2C_SPEED    400000U

/* 제어 핀 */
#define PIN_L7CX_CS     10   /* VL53L7CX  LPn (CS in I2C mode) */
#define PIN_L8CX_CS     11   /* VL53L8CX  LPn (CS in I2C mode) */
#define PIN_TMF_EN      12   /* TMF8828   EN  */

/* 기본 I2C 주소 (8-bit) */
#define ADDR_DEFAULT_ST     0x52
#define ADDR_TMF8828        0x41

/* 재할당 주소 */
#define ADDR_L5CX_OR_L4CD   0x52   /* 제어핀 없음, 그대로 사용 */
#define ADDR_L7CX           0x56
#define ADDR_L8CX           0x58

/* ST 센서 Model ID 레지스터 (16-bit 주소) */
#define ST_REG_MODEL_ID     0x010F
#define MODEL_ID_L5CX       0xF0
#define MODEL_ID_L4CD       0xEB

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
    sensor_info_t slot_52;    /* L5CX 또는 L4CD */
    sensor_info_t slot_l7cx;
    sensor_info_t slot_l8cx;
    sensor_info_t slot_tmf;
    int           count;
} sensor_table_t;

/* 초기화 및 자동 감지 */
bool sensor_manager_init(sensor_table_t *table);

/* I2C 스캔 + Model ID 판별 */
sensor_type_t sensor_identify_at_52(void);

/* 각 센서 I2C 주소 재할당 */
bool sensor_assign_addresses(sensor_table_t *table);
