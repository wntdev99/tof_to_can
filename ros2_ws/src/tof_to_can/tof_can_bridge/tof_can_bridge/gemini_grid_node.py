"""
Gemini 336 PointCloud2 → 8×8 zone grid PointCloud2 변환 노드.

Gemini depth point cloud (full FOV)를 받아, TOF 센서와 동일한
43°×43° FOV의 8×8 grid로 분할하고 cell당 최근접 포인트를 publish.

좌표계: ROS optical frame — Z forward, X right, Y down (REP-103)
각 cell: FOV(43°) / 8 = 5.375° 단위 bin
대표값: cell 내 z(깊이) 최솟값 포인트
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_POINT_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]

GRID = 8


def _parse_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → Nx3 float32 ndarray. NaN/Inf 제거 포함."""
    field_map = {f.name: f.offset for f in msg.fields}
    ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
    step = msg.point_step
    n = msg.width * msg.height

    dt = np.dtype({
        'names':   ['x',        'y',        'z'       ],
        'formats': [np.float32, np.float32, np.float32],
        'offsets': [ox,         oy,         oz        ],
        'itemsize': step,
    })
    arr = np.frombuffer(bytes(msg.data), dtype=dt)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _make_pointcloud2(header: Header, points: list) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = _POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    buf = bytearray(12 * len(points))
    for i, (x, y, z) in enumerate(points):
        struct.pack_into('fff', buf, i * 12, x, y, z)
    msg.data = bytes(buf)
    return msg


class GeminiGridNode(Node):
    def __init__(self):
        super().__init__('gemini_grid_node')

        self.declare_parameter('fov_axis_deg',  43.0)
        self.declare_parameter('min_range_m',    0.1)
        self.declare_parameter('max_range_m',    4.0)
        self.declare_parameter('input_topic',  '/camera/camera/depth/color/points')
        self.declare_parameter('output_topic', '/tof/gemini/points')
        self.declare_parameter('frame_id',     'camera_link')

        fov          = self.get_parameter('fov_axis_deg').value
        self._half   = math.radians(fov / 2.0)
        self._cell   = math.radians(fov) / GRID
        self._min_r  = self.get_parameter('min_range_m').value
        self._max_r  = self.get_parameter('max_range_m').value
        self._fid    = self.get_parameter('frame_id').value
        in_topic     = self.get_parameter('input_topic').value
        out_topic    = self.get_parameter('output_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)
        self._sub = self.create_subscription(PointCloud2, in_topic, self._cb, qos)
        self.get_logger().info(
            f'gemini_grid_node ready  {in_topic} → {out_topic}  fov={fov}°  grid={GRID}×{GRID}'
        )

    def _cb(self, msg: PointCloud2) -> None:
        try:
            xyz = _parse_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f'parse error: {e}', throttle_duration_sec=5.0)
            return

        if len(xyz) == 0:
            return

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        # 거리 범위 필터
        mask = (z > self._min_r) & (z < self._max_r)
        x, y, z = x[mask], y[mask], z[mask]
        if len(z) == 0:
            return

        # 수평/수직 각도 (Z forward, Y down 기준)
        az = np.arctan2(x, z)
        el = np.arctan2(y, z)

        # 43°×43° FOV 필터
        half = self._half
        in_fov = (np.abs(az) <= half) & (np.abs(el) <= half)
        x, y, z = x[in_fov], y[in_fov], z[in_fov]
        az, el = az[in_fov], el[in_fov]
        if len(z) == 0:
            return

        # cell 인덱스 (0~GRID-1)
        col = np.clip(((az + half) / self._cell).astype(int), 0, GRID - 1)
        row = np.clip(((el + half) / self._cell).astype(int), 0, GRID - 1)
        idx = row * GRID + col

        # 각 cell 최근접(z 최솟값) 포인트 선택
        grid: dict[int, tuple] = {}
        for i in range(len(z)):
            ci = int(idx[i])
            if ci not in grid or z[i] < grid[ci][2]:
                grid[ci] = (float(x[i]), float(y[i]), float(z[i]))

        # row-major 순서로 출력 (빈 cell 제외)
        points = [grid[i] for i in range(GRID * GRID) if i in grid]
        if not points:
            return

        self._pub.publish(_make_pointcloud2(
            Header(stamp=msg.header.stamp, frame_id=self._fid),
            points,
        ))


def main(args=None):
    rclpy.init(args=args)
    node = GeminiGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
