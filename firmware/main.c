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

/* ── L5CX 선행 초기화: phase1 직후, phase2 이전에 호출.
 *   init 성공 시 ULD 내부에서 0x52 → 0x50 으로 주소 이동 → phase2 의
 *   L7/L8 탐지(LPn↑ 후 0x52 probe)와 충돌하지 않는다. */
static void init_l5cx_if_present(void) {
    sensor_info_t *sl5 = &g_sensors.slot_l5cx;
    if (!sl5->present) return;

    uint8_t addr8 = (uint8_t)(sl5->i2c_addr << 1);   /* 0x52 */
    if (vl53l5cx_drv_init(&g_l5cx, addr8)) {
        sl5->i2c_addr    = (uint8_t)(ADDR_L5CX_ASSIGNED >> 1);   /* 0x28 */
        sl5->initialized = true;
        vl53l5cx_drv_start(&g_l5cx);
    } else {
        printf("[SENSOR] L5CX driver init 실패 — slot 비활성화\n");
        sl5->present = false;
        if (g_sensors.count > 0) g_sensors.count--;
    }
}

/* ── L4CD 초기화: phase1 직후(부팅) 또는 hotplug 감지 직후에 호출.
 *   init 성공 시 ULD 에서 0x52 → 0x54 이동 → 0x52 를 다시 탐지 슬롯으로 해방.
 *   실패 시 slot 롤백하여 이후 핫플러그에서 재시도 가능.                   */
static void init_l4cd_if_present(void) {
    sensor_info_t *sl4 = &g_sensors.slot_l4cd;
    if (!sl4->present || sl4->initialized) return;

    uint8_t addr8 = (uint8_t)(sl4->i2c_addr << 1);   /* 0x52 */
    if (vl53l4cd_drv_init(&g_l4cd, addr8, 10, 0)) {
        sl4->i2c_addr    = (uint8_t)(ADDR_L4CD_ASSIGNED >> 1);   /* 0x2A */
        sl4->initialized = true;
        vl53l4cd_drv_start(&g_l4cd);
    } else {
        printf("[SENSOR] L4CD driver init 실패 — slot 롤백 (재핫플러그 가능)\n");
        sl4->present = false;
        if (g_sensors.count > 0) g_sensors.count--;
    }
}

/* ── 나머지 센서 드라이버 초기화 (phase2 이후) — L5CX/L4CD 는 제외 */
static void init_other_drivers(void) {
    sensor_info_t *sl7  = &g_sensors.slot_l7cx;
    sensor_info_t *sl8  = &g_sensors.slot_l8cx;
    sensor_info_t *stmf = &g_sensors.slot_tmf;

    /* VL53L7CX @ 0x56 (재할당) */
    if (sl7->present && !sl7->initialized) {
        uint8_t addr8 = (uint8_t)(sl7->i2c_addr << 1);
        if (vl53l7cx_drv_init(&g_l7cx, addr8)) {
            sl7->initialized = true;
            vl53l7cx_drv_start(&g_l7cx);
        }
    }

    /* VL53L8CX @ 0x58 (재할당) */
    if (sl8->present && !sl8->initialized) {
        uint8_t addr8 = (uint8_t)(sl8->i2c_addr << 1);
        if (vl53l8cx_drv_init(&g_l8cx, addr8)) {
            sl8->initialized = true;
            vl53l8cx_drv_start(&g_l8cx);
        }
    }

    /* TMF8828 @ 0x41 */
    if (stmf->present && !stmf->initialized) {
        if (tmf8828_drv_init(&g_tmf)) {
            stmf->initialized = true;
            tmf8828_drv_start(&g_tmf);
        }
    }
}

/* ── Core0: 센서 폴링 루프 ───────────────────────────────── */
static void poll_all_sensors(void) {
    if (g_sensors.slot_l5cx.present)
        vl53l5cx_drv_poll_and_push(&g_l5cx, &g_rb);

    if (g_sensors.slot_l4cd.present)
        vl53l4cd_drv_poll_and_push(&g_l4cd, &g_rb);

    if (g_sensors.slot_l7cx.present)
        vl53l7cx_drv_poll_and_push(&g_l7cx, &g_rb);

    if (g_sensors.slot_l8cx.present)
        vl53l8cx_drv_poll_and_push(&g_l8cx, &g_rb);

    if (g_sensors.slot_tmf.present)
        tmf8828_drv_poll_and_push(&g_tmf, &g_rb);
}

/* ── 진입점 ──────────────────────────────────────────────── */
int main(void) {
    /* LPn 핀을 가장 먼저 LOW — 부팅 직후 L7CX/L8CX 버스 충돌 방지 */
    gpio_init(PIN_L7CX_CS); gpio_set_dir(PIN_L7CX_CS, GPIO_OUT); gpio_put(PIN_L7CX_CS, 0);
    gpio_init(PIN_L8CX_CS); gpio_set_dir(PIN_L8CX_CS, GPIO_OUT); gpio_put(PIN_L8CX_CS, 0);

    stdio_init_all();

    /* USB CDC 연결 대기 (최대 5초). 미연결 시 standalone으로 계속 실행 */
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++)
        sleep_ms(100);

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

    /* 센서 자동 감지 — 2단계로 분할
     *   phase1: 0x52 probe (L5CX / L4CD 모델 판별)
     *   init_l5cx/l4cd: 각자 할당 주소(0x50/0x54)로 이동 → 0x52 항상 free
     *   phase2: TMF(0x41) / L7CX(0x52→0x56) / L8CX(0x52→0x58)                */
    printf("[SENSOR] Scanning I2C bus...\n");
    sensor_manager_init_phase1(&g_sensors);
    init_l5cx_if_present();
    init_l4cd_if_present();
    if (!sensor_manager_init_phase2(&g_sensors))
        printf("[SENSOR] No sensors found — CAN TX only\n");

    /* 남은 드라이버 초기화 (L5CX/L4CD 는 이미 시작됨) */
    init_other_drivers();

    /* Core1 시작 (CAN TX) */
    multicore_launch_core1(core1_entry);

    printf("[MAIN] Entering poll loop\n");
    uint32_t hotplug_timer = 0;
    while (true) {
        poll_all_sensors();

        /* 2초마다 L4CD 핫플러그 확인 */
        if (++hotplug_timer >= 200) {
            hotplug_timer = 0;
            if (sensor_manager_poll_hotplug(&g_sensors))
                init_l4cd_if_present();
        }

        sleep_ms(10);
    }
}
