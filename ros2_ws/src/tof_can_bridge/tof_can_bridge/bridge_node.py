"""
ToF-to-CAN bridge node.

SocketCAN (can0) 에서 프레임을 읽어:
  - VL53L4CD  → sensor_msgs/Range      on /tof/l4cd
  - VL53L5CX  → sensor_msgs/PointCloud2 on /tof/l5cx/points
  - VL53L7CX  → sensor_msgs/PointCloud2 on /tof/l7cx/points

실행:
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
  ros2 launch tof_can_bridge bringup.launch.py
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header

import can

from .can_decoder import (
    CAN_ID_L5CX_BASE,
    CAN_ID_L4CD_BASE,
    CAN_ID_L7CX_BASE,
    SENSOR_INFO,
    MultiFrameAssembler,
    decode_single_zone,
    split_can_id,
)
from .zone_projection import build_unit_rays, project_distances_mm


# ── 센서별 파라미터 ───────────────────────────────────────────

L4CD_FOV_DEG     = 18.0   # VL53L4CD datasheet — 18° typical
L4CD_MIN_RANGE_M = 0.01
L4CD_MAX_RANGE_M = 1.3

MULTIZONE_MIN_RANGE_M = 0.02
MULTIZONE_MAX_RANGE_M = 4.0


# ── PointCloud2 헬퍼 ──────────────────────────────────────────

_POINT_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]

def make_pointcloud2(header: Header, points) -> PointCloud2:
    """points: iterable of (x, y, z) floats (meter)."""
    n = len(points)
    buf = bytearray(n * 12)
    for i, (x, y, z) in enumerate(points):
        struct.pack_into('<fff', buf, i * 12, x, y, z)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = bytes(buf)
    return msg


# ── 노드 ──────────────────────────────────────────────────────

class TofCanBridge(Node):

    def __init__(self):
        super().__init__('tof_can_bridge')

        # --- parameters ---
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('frame_id_l4cd', 'l4cd_frame')
        self.declare_parameter('frame_id_l5cx', 'l5cx_optical_frame')
        self.declare_parameter('frame_id_l7cx', 'l7cx_optical_frame')
        self.declare_parameter('fov_axis_deg', 43.0)
        self.declare_parameter('flip_rows', False)
        self.declare_parameter('flip_cols', False)

        iface = self.get_parameter('can_interface').value
        self._frame_l4cd = self.get_parameter('frame_id_l4cd').value
        self._frame_l5cx = self.get_parameter('frame_id_l5cx').value
        self._frame_l7cx = self.get_parameter('frame_id_l7cx').value
        fov = float(self.get_parameter('fov_axis_deg').value)
        flip_r = bool(self.get_parameter('flip_rows').value)
        flip_c = bool(self.get_parameter('flip_cols').value)

        # --- publishers ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub_l4cd = self.create_publisher(Range,       '/tof/l4cd',        qos)
        self._pub_l5cx = self.create_publisher(PointCloud2, '/tof/l5cx/points', qos)
        self._pub_l7cx = self.create_publisher(PointCloud2, '/tof/l7cx/points', qos)

        # --- multiframe assemblers ---
        self._asm = {
            CAN_ID_L5CX_BASE: MultiFrameAssembler(SENSOR_INFO[CAN_ID_L5CX_BASE]['name']),
            CAN_ID_L7CX_BASE: MultiFrameAssembler(SENSOR_INFO[CAN_ID_L7CX_BASE]['name']),
        }

        # --- zone projection LUT (L5CX/L7CX 공통: 8×8) ---
        self._rays_8x8 = build_unit_rays(
            grid=8, fov_axis_deg=fov,
            flip_rows=flip_r, flip_cols=flip_c,
        )

        # --- CAN bus ---
        try:
            self._bus = can.interface.Bus(channel=iface, bustype='socketcan')
        except OSError as e:
            self.get_logger().error(f"cannot open CAN '{iface}': {e}")
            raise

        self.get_logger().info(
            f"listening on {iface} — "
            f"/tof/l4cd (Range), /tof/l5cx/points, /tof/l7cx/points (PointCloud2)"
        )

        # --- RX thread ---
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def destroy_node(self):
        self._stop.set()
        try:
            self._bus.shutdown()
        except Exception:
            pass
        super().destroy_node()

    # ── RX 루프 ───────────────────────────────────────────────

    def _rx_loop(self):
        while not self._stop.is_set():
            msg = self._bus.recv(timeout=0.5)
            if msg is None:
                continue

            base_id, frame_idx = split_can_id(msg.arbitration_id)
            info = SENSOR_INFO.get(base_id)
            if info is None:
                continue

            try:
                if not info['multiframe']:
                    self._handle_l4cd(msg.data)
                else:
                    asm = self._asm[base_id]
                    if asm.feed(frame_idx, msg.data):
                        self._handle_multizone(base_id, asm.get_result())
            except Exception as e:
                self.get_logger().warn(f"decode error on base=0x{base_id:03X}: {e}")

    # ── publish ───────────────────────────────────────────────

    def _now_header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _handle_l4cd(self, data: bytes):
        r = decode_single_zone(data)
        msg = Range()
        msg.header = self._now_header(self._frame_l4cd)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(L4CD_FOV_DEG) * 3.14159265358979 / 180.0
        msg.min_range = L4CD_MIN_RANGE_M
        msg.max_range = L4CD_MAX_RANGE_M
        msg.range = float(r['distance_mm']) / 1000.0
        self._pub_l4cd.publish(msg)

    def _handle_multizone(self, base_id: int, result: dict):
        distances = result['distances']
        zones = result['zones']
        if zones != 64:
            self.get_logger().warn(
                f"unexpected zone count {zones} for base=0x{base_id:03X}"
            )
            return

        points = project_distances_mm(
            distances, self._rays_8x8,
            min_range_m=MULTIZONE_MIN_RANGE_M,
            max_range_m=MULTIZONE_MAX_RANGE_M,
        )

        if base_id == CAN_ID_L5CX_BASE:
            pub, frame_id = self._pub_l5cx, self._frame_l5cx
        else:
            pub, frame_id = self._pub_l7cx, self._frame_l7cx

        pub.publish(make_pointcloud2(self._now_header(frame_id), points))


def main(args=None):
    rclpy.init(args=args)
    node = TofCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
