#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>

#include "drivers/mcp25625/mcp25625.h"
#include "drivers/vl53l5cx/vl53l5cx_driver.h"
#include "drivers/vl53l4cd/vl53l4cd_driver.h"
#include "drivers/vl53l7cx/vl53l7cx_driver.h"
#include "drivers/vl53l8cx/vl53l8cx_driver.h"
#include "drivers/tmf8828/tmf8828_driver.h"
#include "sensor/sensor_manager.h"
#include "can/can_tx.h"
#include "can/can_protocol.h"
#include "utils/ring_buffer.h"

/* ── 공유 자원 ────────────────────────────────────────────── */
static ring_buffer_t   g_rb;
static sensor_table_t  g_sensors;

/* ── 드라이버 핸들 (Configuration은 각 driver.c 내부 static 변수로 관리) */
static VL53L5CX_Drv  g_l5cx;
static VL53L4CD_Drv  g_l4cd;
static VL53L7CX_Drv  g_l7cx;
static VL53L8CX_Drv  g_l8cx;
static TMF8828_Drv   g_tmf;

/* ── Core1: CAN 전송 루프 ─────────────────────────────────── */
static void core1_entry(void) {
    while (true) {
        can_tx_task(&g_rb);
        sleep_us(100);
    }
}

/* ── 드라이버 초기화 (sensor_table 기반) ─────────────────── */
static void init_sensor_drivers(void) {
    sensor_info_t *s52  = &g_sensors.slot_52;
    sensor_info_t *sl7  = &g_sensors.slot_l7cx;
    sensor_info_t *sl8  = &g_sensors.slot_l8cx;
    sensor_info_t *stmf = &g_sensors.slot_tmf;

    /* 0x52 슬롯: L5CX 또는 L4CD */
    if (s52->present) {
        uint8_t addr8 = (uint8_t)(s52->i2c_addr << 1);  /* 7-bit → 8-bit */
        if (s52->type == SENSOR_VL53L5CX) {
            if (vl53l5cx_drv_init(&g_l5cx, addr8))
                vl53l5cx_drv_start(&g_l5cx);
        } else if (s52->type == SENSOR_VL53L4CD) {
            /* timing_budget=10ms, inter_meas=0(최대속도) */
            if (vl53l4cd_drv_init(&g_l4cd, addr8, 10, 0))
                vl53l4cd_drv_start(&g_l4cd);
        }
    }

    /* VL53L7CX @ 0x56 (재할당) */
    if (sl7->present) {
        uint8_t addr8 = (uint8_t)(sl7->i2c_addr << 1);
        if (vl53l7cx_drv_init(&g_l7cx, addr8))
            vl53l7cx_drv_start(&g_l7cx);
    }

    /* VL53L8CX @ 0x58 (재할당) */
    if (sl8->present) {
        uint8_t addr8 = (uint8_t)(sl8->i2c_addr << 1);
        if (vl53l8cx_drv_init(&g_l8cx, addr8))
            vl53l8cx_drv_start(&g_l8cx);
    }

    /* TMF8828 @ 0x41 */
    if (stmf->present) {
        if (tmf8828_drv_init(&g_tmf))
            tmf8828_drv_start(&g_tmf);
    }
}

/* ── Core0: 센서 폴링 루프 ───────────────────────────────── */
static void poll_all_sensors(void) {
    sensor_info_t *s52  = &g_sensors.slot_52;
    sensor_info_t *sl7  = &g_sensors.slot_l7cx;
    sensor_info_t *sl8  = &g_sensors.slot_l8cx;
    sensor_info_t *stmf = &g_sensors.slot_tmf;

    if (s52->present) {
        if (s52->type == SENSOR_VL53L5CX)
            vl53l5cx_drv_poll_and_push(&g_l5cx, &g_rb);
        else if (s52->type == SENSOR_VL53L4CD)
            vl53l4cd_drv_poll_and_push(&g_l4cd, &g_rb);
    }

    if (sl7->present)
        vl53l7cx_drv_poll_and_push(&g_l7cx, &g_rb);

    if (sl8->present)
        vl53l8cx_drv_poll_and_push(&g_l8cx, &g_rb);

    if (stmf->present)
        tmf8828_drv_poll_and_push(&g_tmf, &g_rb);
}

/* ── 진입점 ──────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();
    sleep_ms(500);   /* USB CDC 안정화 */

    printf("=== ToF-to-CAN firmware start ===\n");

    ring_buffer_init(&g_rb);

    /* MCP25625 초기화 */
    printf("[CAN] Initializing MCP25625...\n");
    if (!mcp25625_init()) {
        printf("[CAN] INIT FAILED\n");
        while (true) tight_loop_contents();
    }

    printf("[CAN] Loopback test...\n");
    if (!mcp25625_loopback_test()) {
        printf("[CAN] LOOPBACK FAILED\n");
        while (true) tight_loop_contents();
    }
    printf("[CAN] Loopback OK\n");

    if (!mcp25625_set_normal_mode()) {
        printf("[CAN] Cannot enter normal mode\n");
        while (true) tight_loop_contents();
    }

    /* 센서 자동 감지 및 주소 할당 */
    printf("[SENSOR] Scanning I2C bus...\n");
    if (!sensor_manager_init(&g_sensors))
        printf("[SENSOR] No sensors found — CAN TX only\n");

    /* 감지된 센서 드라이버 초기화 */
    init_sensor_drivers();

    /* Core1 시작 (CAN TX) */
    multicore_launch_core1(core1_entry);

    printf("[MAIN] Entering poll loop\n");
    while (true) {
        poll_all_sensors();
        sleep_ms(10);
    }
}
