#include "can_tx.h"
#include "../drivers/mcp25625/mcp25625.h"
#include "pico/stdlib.h"
#include <string.h>

void can_pack_single_zone(uint16_t base_id, uint16_t dist_mm,
                          uint8_t status, ring_buffer_t *rb) {
    can_frame_t f = {0};
    f.can_id    = base_id;
    f.dlc       = 3;
    f.data[0]   = (uint8_t)(dist_mm & 0xFF);
    f.data[1]   = (uint8_t)(dist_mm >> 8);
    f.data[2]   = status;
    ring_buffer_push(rb, &f);
}

void can_pack_multizone(uint16_t base_id, const uint16_t *dist_mm,
                        uint8_t zones, ring_buffer_t *rb) {
    /*
     * 프레임 0 (헤더):
     *   byte0: total_frames, byte1: zones, byte2~7: 첫 3존 (2byte each → 6byte)
     * 프레임 1~N:
     *   각 프레임에 4존씩 (8byte)
     */
    uint8_t zones_per_frame_body = 4;
    uint8_t total_frames = 1 + (uint8_t)((zones + zones_per_frame_body - 1) / zones_per_frame_body);

    /* 헤더 프레임 */
    can_frame_t hdr = {0};
    hdr.can_id  = CAN_FRAME_ID(base_id, 0);
    hdr.dlc     = 8;
    hdr.data[0] = total_frames;
    hdr.data[1] = zones;
    /* 첫 3존: byte2~7 */
    uint8_t first = (zones < 3) ? zones : 3;
    for (uint8_t i = 0; i < first; i++) {
        hdr.data[2 + i * 2]     = (uint8_t)(dist_mm[i] & 0xFF);
        hdr.data[2 + i * 2 + 1] = (uint8_t)(dist_mm[i] >> 8);
    }
    ring_buffer_push(rb, &hdr);

    /* 데이터 프레임 */
    uint8_t zone_idx = first;
    for (uint8_t fi = 1; fi < total_frames && zone_idx < zones; fi++) {
        can_frame_t f = {0};
        f.can_id = CAN_FRAME_ID(base_id, fi);
        f.dlc    = 0;
        for (uint8_t i = 0; i < 4 && zone_idx < zones; i++, zone_idx++) {
            f.data[i * 2]     = (uint8_t)(dist_mm[zone_idx] & 0xFF);
            f.data[i * 2 + 1] = (uint8_t)(dist_mm[zone_idx] >> 8);
            f.dlc += 2;
        }
        ring_buffer_push(rb, &f);
    }
}

void can_pack_tmf8828(const uint16_t *dist_mm, uint8_t zones,
                      ring_buffer_t *rb) {
    can_pack_multizone(CAN_ID_TMF8828_BASE, dist_mm, zones, rb);
}

/* Core1 루프: 링 버퍼 → MCP25625 전송 */
void can_tx_task(ring_buffer_t *rb) {
    can_frame_t f;
    while (ring_buffer_pop(rb, &f)) {
        mcp25625_send_frame(f.can_id, f.data, f.dlc);
        /* 프레임 간 최소 간격 (버스 부하 방지) */
        sleep_us(200);
    }
}
