"""
ToF-to-CAN 수신 파서
Ubuntu + SocketCAN (python-can)

설치:
    sudo apt install can-utils python3-pip
    pip3 install python-can

인터페이스 활성화 (예: PEAK USB-to-CAN):
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up

실행:
    python3 can_receiver.py [--interface can0]
"""

import can
import struct
import time
import argparse
from collections import defaultdict

# ── CAN ID 상수 ────────────────────────────────────────────
CAN_ID_L5CX_BASE    = 0x100
CAN_ID_L4CD_BASE    = 0x200
CAN_ID_L7CX_BASE    = 0x300
CAN_ID_L8CX_BASE    = 0x400
CAN_ID_TMF8828_BASE = 0x500

SENSOR_MAP = {
    CAN_ID_L5CX_BASE:    ("VL53L5CX",  64, True),   # (이름, zones, multiframe)
    CAN_ID_L4CD_BASE:    ("VL53L4CD",   1, False),
    CAN_ID_L7CX_BASE:    ("VL53L7CX",  64, True),
    CAN_ID_L8CX_BASE:    ("VL53L8CX",  64, True),
    CAN_ID_TMF8828_BASE: ("TMF8828",   75, True),
}

BASE_MASK = 0xFF00   # 상위 바이트로 센서 구분, 하위 바이트로 frame_index


# ── 단일존 프레임 디코딩 ───────────────────────────────────

def decode_single_zone(data: bytes) -> dict:
    """VL53L4CD: [dist_lo, dist_hi, status, ...]"""
    dist_mm = struct.unpack_from("<H", data, 0)[0]
    status  = data[2]
    return {"distance_mm": dist_mm, "status": status, "zones": 1}


# ── 멀티프레임 조립기 ──────────────────────────────────────

class MultiFrameAssembler:
    def __init__(self, sensor_name: str):
        self.sensor_name  = sensor_name
        self.total_frames = 0
        self.zones        = 0
        self.distances    = []
        self._received    = set()
        self._reset()

    def _reset(self):
        self.total_frames = 0
        self.zones        = 0
        self.distances    = []
        self._received    = set()

    def feed(self, frame_index: int, data: bytes) -> bool:
        """
        frame_index == 0: 헤더 프레임
          byte0 = total_frames, byte1 = zones, byte2-7 = dist[0..2]
        frame_index >= 1: 데이터 프레임
          byte0-7 = dist × 4존 (2byte each, LE)

        반환: True = 전체 조립 완료
        """
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
                return False   # 헤더를 아직 못 받음
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


# ── 수신기 ─────────────────────────────────────────────────

class ToFCanReceiver:
    def __init__(self, interface: str = "can0"):
        self.bus = can.interface.Bus(channel=interface,
                                     bustype="socketcan")
        self._assemblers: dict[int, MultiFrameAssembler] = {}

        for base_id, (name, _, multiframe) in SENSOR_MAP.items():
            if multiframe:
                self._assemblers[base_id] = MultiFrameAssembler(name)

    def _find_base(self, can_id: int):
        """CAN ID에서 base_id, frame_index 추출"""
        base = can_id & BASE_MASK
        frame_idx = can_id & 0x00FF
        return base, frame_idx

    def _on_result(self, result: dict):
        """완성된 측정 결과 처리 — 필요에 따라 오버라이드 또는 확장"""
        name = result.get("sensor", "?")
        if result.get("zones", 0) == 1:
            print(f"[{name}] dist={result['distance_mm']} mm  "
                  f"status={result['status']}")
        else:
            dists = result["distances"]
            zones = result["zones"]
            # 8×8 그리드로 출력 (L5CX/L7CX/L8CX)
            cols = 8 if zones == 64 else 9
            print(f"[{name}] {zones}존 거리 맵 (mm):")
            for row in range((zones + cols - 1) // cols):
                row_data = dists[row * cols: row * cols + cols]
                print("  " + "  ".join(f"{d:4d}" for d in row_data))

    def run(self):
        print(f"[RX] 수신 대기 중... (Ctrl+C로 종료)")
        try:
            while True:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                base_id, frame_idx = self._find_base(msg.arbitration_id)

                if base_id not in SENSOR_MAP:
                    continue

                name, zones, multiframe = SENSOR_MAP[base_id]

                if not multiframe:
                    # 단일존 (VL53L4CD)
                    result = decode_single_zone(msg.data)
                    result["sensor"] = name
                    self._on_result(result)
                else:
                    asm = self._assemblers.get(base_id)
                    if asm and asm.feed(frame_idx, msg.data):
                        self._on_result(asm.get_result())

        except KeyboardInterrupt:
            print("\n[RX] 종료")
        finally:
            self.bus.shutdown()


# ── CSV 로거 (선택적 사용) ─────────────────────────────────

class ToFCanLogger(ToFCanReceiver):
    def __init__(self, interface: str = "can0",
                 csv_path: str = "tof_log.csv"):
        super().__init__(interface)
        self._f = open(csv_path, "w")
        self._f.write("timestamp,sensor,zone,distance_mm\n")
        print(f"[LOG] CSV 저장 → {csv_path}")

    def _on_result(self, result: dict):
        ts = time.time()
        name = result.get("sensor", "?")
        if result.get("zones", 0) == 1:
            self._f.write(f"{ts:.3f},{name},0,{result['distance_mm']}\n")
        else:
            for i, d in enumerate(result["distances"]):
                self._f.write(f"{ts:.3f},{name},{i},{d}\n")
        self._f.flush()

    def run(self):
        try:
            super().run()
        finally:
            self._f.close()


# ── 진입점 ─────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ToF-to-CAN 수신 파서")
    parser.add_argument("--interface", default="can0",
                        help="SocketCAN 인터페이스 이름 (기본값: can0)")
    parser.add_argument("--log", metavar="FILE",
                        help="CSV 파일 경로 (지정 시 CSV 로깅 활성화)")
    args = parser.parse_args()

    if args.log:
        rx = ToFCanLogger(interface=args.interface, csv_path=args.log)
    else:
        rx = ToFCanReceiver(interface=args.interface)

    rx.run()
