"""
CAN 프레임 디코더 — host/can_receiver.py 의 로직을 ROS2 노드에서 재사용.

프레임 포맷 (firmware/can/can_protocol.h 참조):
  base_id 로 센서 식별, 하위 바이트로 frame_index (멀티프레임 조립).

  단일존 (L4CD, 0x200):
    [dist_lo, dist_hi, status, rsvd×5]
  다존 (L5CX=0x100, L7CX=0x300):
    frame 0: [total_frames, zones, dist0_lo, dist0_hi, dist1_lo, dist1_hi, dist2_lo, dist2_hi]
    frame N≥1: [dist × 4존 (2byte LE)]
"""

import struct


CAN_ID_L5CX_BASE = 0x100
CAN_ID_L4CD_BASE = 0x200
CAN_ID_L7CX_BASE = 0x300

BASE_MASK = 0xFF00

SENSOR_INFO = {
    CAN_ID_L5CX_BASE: {"name": "VL53L5CX", "zones": 64, "multiframe": True},
    CAN_ID_L4CD_BASE: {"name": "VL53L4CD", "zones":  1, "multiframe": False},
    CAN_ID_L7CX_BASE: {"name": "VL53L7CX", "zones": 64, "multiframe": True},
}


def split_can_id(can_id: int):
    return can_id & BASE_MASK, can_id & 0x00FF


def decode_single_zone(data: bytes) -> dict:
    dist_mm = struct.unpack_from("<H", data, 0)[0]
    status  = data[2]
    return {"distance_mm": dist_mm, "status": status, "zones": 1}


class MultiFrameAssembler:
    """base_id 별로 하나씩 생성. feed() 가 True 반환 시 get_result() 로 꺼낸다."""

    def __init__(self, sensor_name: str):
        self.sensor_name = sensor_name
        self._reset()

    def _reset(self):
        self.total_frames = 0
        self.zones        = 0
        self.distances    = []
        self._received    = set()

    def feed(self, frame_index: int, data: bytes) -> bool:
        if frame_index == 0:
            self._reset()
            self.total_frames = data[0]
            self.zones        = data[1]
            self.distances    = [0] * self.zones
            first = min(3, self.zones)
            for i in range(first):
                self.distances[i] = struct.unpack_from("<H", data, 2 + i * 2)[0]
            self._received.add(0)
        else:
            if self.total_frames == 0:
                return False
            zone_start = 3 + (frame_index - 1) * 4
            for i in range(4):
                z = zone_start + i
                if z >= self.zones:
                    break
                offset = i * 2
                if offset + 1 < len(data):
                    self.distances[z] = struct.unpack_from("<H", data, offset)[0]
            self._received.add(frame_index)

        return len(self._received) == self.total_frames

    def get_result(self) -> dict:
        return {
            "sensor":    self.sensor_name,
            "zones":     self.zones,
            "distances": list(self.distances),
        }
