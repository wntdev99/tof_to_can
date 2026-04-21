"""
tof_eval — PointCloud2 평가 노드.

단일 input_topic 구독 → RANSAC ground plane → working frame 변환 →
clustering → obstacle 매칭 → per-frame CSV append → 세션 종료 시 summary + index.

사용:
  ros2 launch tof_eval eval.launch.py \\
      input_topic:=/tof/l5cx/points \\
      params_file:=<path to eval_params.yaml>
"""

import math
import os
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2

from .clustering import euclidean_cluster, filter_above_ground
from .ground_plane import (
    build_working_frame,
    fit_ground_plane,
    normal_angle_deg,
    to_working_frame,
)
from .metrics import (
    ObstacleSpec,
    PerFrameMetrics,
    aggregate_session,
    forward_slab_density,
    match_obstacles,
)
from .pointcloud_io import pointcloud2_to_xyz
from .session_writer import SessionWriter


def _default_session_id() -> str:
    """session_id 미지정 시 fallback — 타임스탬프 기반."""
    from datetime import datetime
    return datetime.now().strftime('session_%Y%m%d_%H%M%S')


class TofEvalNode(Node):

    def __init__(self):
        super().__init__('tof_eval')

        # ── parameters ──────────────────────────────────────
        self.declare_parameter('input_topic', '/tof/l5cx/points')

        # Scene 메타데이터 (알고리즘 영향 없음, 기록용)
        self.declare_parameter('scene.session_id', '')
        self.declare_parameter('scene.sensor', 'UNKNOWN')
        self.declare_parameter('scene.floor_material', 'unspecified')
        self.declare_parameter('scene.floor_color', 'unspecified')
        self.declare_parameter('scene.ambient_light', 'unspecified')
        self.declare_parameter('scene.obstacle_surface', 'unspecified')
        self.declare_parameter('scene.notes', '')
        self.declare_parameter('scene.tags', [''])   # ROS param 빈 str array 취급 주의

        # Ground plane (Group B)
        self.declare_parameter('ground_removal.ransac_distance_threshold', 0.02)
        self.declare_parameter('ground_removal.ransac_max_iterations', 100)
        self.declare_parameter('ground_removal.min_inlier_ratio', 0.3)

        # Clustering (Group C)
        self.declare_parameter('clustering.min_height_above_ground', 0.03)
        self.declare_parameter('clustering.max_height_above_ground', 2.0)
        self.declare_parameter('clustering.euclidean_tolerance', 0.05)
        self.declare_parameter('clustering.min_cluster_points', 3)

        # A1 — expected point count (0 이면 NaN 으로 기록)
        self.declare_parameter('expected_point_count', 64)

        # A4 — forward slab density 파라미터
        self.declare_parameter('density_distances_m', [0.3, 0.5, 1.0, 1.5, 2.0])
        self.declare_parameter('density_slab_thickness_m', 0.2)
        self.declare_parameter('density_lateral_window_m', 2.0)

        # Obstacles — ROS2 param 은 dict array 직접 지원 X → index 기반 평탄화
        # obstacle_ids: [cube_a, cube_b] 같은 list 지정 후 각 id 별 sub-param 읽기
        self.declare_parameter('obstacle_ids', [''])

        # 출력
        self.declare_parameter('csv_output_dir', '/tmp/tof_eval')

        # ── load params ─────────────────────────────────────
        p = self.get_parameter

        input_topic = p('input_topic').value
        sid = p('scene.session_id').value or _default_session_id()
        self._scene_meta: Dict[str, Any] = {
            'session_id': sid,
            'sensor': p('scene.sensor').value,
            'floor_material': p('scene.floor_material').value,
            'floor_color': p('scene.floor_color').value,
            'ambient_light': p('scene.ambient_light').value,
            'obstacle_surface': p('scene.obstacle_surface').value,
            'notes': p('scene.notes').value,
            'tags': [t for t in (p('scene.tags').value or []) if t],
        }

        self._ransac_thresh = float(p('ground_removal.ransac_distance_threshold').value)
        self._ransac_iters = int(p('ground_removal.ransac_max_iterations').value)
        self._min_inlier = float(p('ground_removal.min_inlier_ratio').value)

        self._clust_min_h = float(p('clustering.min_height_above_ground').value)
        self._clust_max_h = float(p('clustering.max_height_above_ground').value)
        self._clust_tol = float(p('clustering.euclidean_tolerance').value)
        self._clust_min_pts = int(p('clustering.min_cluster_points').value)

        self._expected_count = int(p('expected_point_count').value)

        self._density_distances: List[float] = [
            float(x) for x in p('density_distances_m').value
        ]
        self._density_thickness = float(p('density_slab_thickness_m').value)
        self._density_lateral = float(p('density_lateral_window_m').value)

        # Obstacles 로드
        self._obstacles = self._load_obstacles()

        output_dir = str(p('csv_output_dir').value)
        os.makedirs(output_dir, exist_ok=True)

        # ── session writer ──────────────────────────────────
        algo_params_snapshot = {
            'ground_removal': {
                'ransac_distance_threshold': self._ransac_thresh,
                'ransac_max_iterations': self._ransac_iters,
                'min_inlier_ratio': self._min_inlier,
            },
            'clustering': {
                'min_height_above_ground': self._clust_min_h,
                'max_height_above_ground': self._clust_max_h,
                'euclidean_tolerance': self._clust_tol,
                'min_cluster_points': self._clust_min_pts,
            },
            'expected_point_count': self._expected_count,
            'density_slab_thickness_m': self._density_thickness,
            'density_lateral_window_m': self._density_lateral,
            'input_topic': input_topic,
            'obstacles': [
                {'id': o.obstacle_id,
                 'expected_xyz': list(o.expected_xyz),
                 'size_xyz': list(o.size_xyz),
                 'search_radius': o.search_radius}
                for o in self._obstacles
            ],
        }
        self._writer = SessionWriter(
            output_dir=output_dir,
            session_id=sid,
            density_distances=self._density_distances,
            meta=self._scene_meta,
            algo_params=algo_params_snapshot,
        )

        # ── state ───────────────────────────────────────────
        self._frame_idx = 0
        self._prev_time: Optional[float] = None
        self._prev_normal: Optional[np.ndarray] = None
        self._per_frame: List[PerFrameMetrics] = []
        self._per_obstacle: List = []   # PerObstacleMetrics
        self._fp_per_frame: List[int] = []
        self._closed = False

        # ── subscription ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos,
        )

        self.get_logger().info(
            f"[tof_eval] session_id={sid} "
            f"input={input_topic} output_dir={output_dir} "
            f"obstacles={len(self._obstacles)}"
        )

    # ── param 로드 헬퍼 ─────────────────────────────────────

    def _load_obstacles(self) -> List[ObstacleSpec]:
        ids_raw = self.get_parameter('obstacle_ids').value or []
        obstacle_ids = [i for i in ids_raw if i]

        # 각 id 에 대해 obstacles.<id>.{expected_xyz, size_xyz, search_radius} 읽기
        specs: List[ObstacleSpec] = []
        for oid in obstacle_ids:
            base = f'obstacles.{oid}'
            self.declare_parameter(f'{base}.expected_xyz', [0.0, 0.0, 0.0])
            self.declare_parameter(f'{base}.size_xyz', [0.1, 0.1, 0.1])
            self.declare_parameter(f'{base}.search_radius', 0.2)

            exp = [float(x) for x in self.get_parameter(f'{base}.expected_xyz').value]
            sz = [float(x) for x in self.get_parameter(f'{base}.size_xyz').value]
            sr = float(self.get_parameter(f'{base}.search_radius').value)

            if len(exp) != 3 or len(sz) != 3:
                self.get_logger().warn(
                    f"obstacle '{oid}' xyz/size must be 3-vec, skipped"
                )
                continue
            specs.append(ObstacleSpec(
                obstacle_id=oid,
                expected_xyz=(exp[0], exp[1], exp[2]),
                size_xyz=(sz[0], sz[1], sz[2]),
                search_radius=sr,
            ))
        return specs

    # ── callback ────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp
        t_now = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        dt = (t_now - self._prev_time) if self._prev_time is not None else float('nan')
        self._prev_time = t_now

        # 1) PointCloud2 → Nx3
        try:
            pts_raw = pointcloud2_to_xyz(msg)
        except ValueError as e:
            self.get_logger().warn(f"decode fail: {e}")
            return

        n_raw = int(msg.width * msg.height)
        n_valid = int(pts_raw.shape[0])
        valid_ratio = (n_valid / self._expected_count) if self._expected_count > 0 \
            else float('nan')

        # 2) RANSAC ground plane
        plane = fit_ground_plane(
            pts_raw,
            distance_threshold=self._ransac_thresh,
            max_iterations=self._ransac_iters,
            min_inlier_ratio=self._min_inlier,
        )

        if plane is None:
            # ground fit 실패 — C/A4 계산 불가, 최소 정보만 기록
            m = PerFrameMetrics(
                frame_idx=self._frame_idx,
                timestamp=t_now,
                dt_since_prev=dt,
                n_points_raw=n_raw,
                n_points_valid=n_valid,
                valid_zone_ratio=valid_ratio,
                ground_fit_ok=False,
                ground_inlier_ratio=float('nan'),
                ground_residual_std=float('nan'),
                plane_normal_angle_deg=float('nan'),
                fwd_densities={D: float('nan') for D in self._density_distances},
            )
            self._per_frame.append(m)
            self._fp_per_frame.append(0)
            self._writer.append_frame(m)
            self._frame_idx += 1
            return

        # 3) working frame
        wf = build_working_frame(plane)
        if wf is None:
            self.get_logger().warn("working frame degenerate (sensor ∥ normal)")
            self._frame_idx += 1
            return

        pts_wf = to_working_frame(pts_raw, wf)

        # 4) A4 density — inlier 만
        inlier_wf = pts_wf[plane.inlier_mask]
        densities = forward_slab_density(
            inlier_wf,
            distances=self._density_distances,
            slab_thickness=self._density_thickness,
            lateral_window=self._density_lateral,
        )

        # 5) obstacle clustering — above-ground 만
        above_mask = filter_above_ground(
            pts_wf, self._clust_min_h, self._clust_max_h,
        )
        above_wf = pts_wf[above_mask]
        clusters = euclidean_cluster(
            above_wf,
            tolerance=self._clust_tol,
            min_points=self._clust_min_pts,
        )

        # 6) 매칭
        obs_rows, fp_count = match_obstacles(clusters, self._obstacles, self._frame_idx)

        # 7) B3 — 직전 normal 과의 각도
        if self._prev_normal is not None:
            angle = normal_angle_deg(self._prev_normal, plane.normal)
        else:
            angle = float('nan')
        self._prev_normal = plane.normal

        # 8) record
        m = PerFrameMetrics(
            frame_idx=self._frame_idx,
            timestamp=t_now,
            dt_since_prev=dt,
            n_points_raw=n_raw,
            n_points_valid=n_valid,
            valid_zone_ratio=valid_ratio,
            ground_fit_ok=True,
            ground_inlier_ratio=plane.inlier_ratio,
            ground_residual_std=plane.residual_std,
            plane_normal_angle_deg=angle,
            fwd_densities=densities,
        )
        self._per_frame.append(m)
        self._per_obstacle.extend(obs_rows)
        self._fp_per_frame.append(fp_count)

        self._writer.append_frame(m)
        self._writer.append_obstacles(obs_rows)

        self._frame_idx += 1

    # ── close ───────────────────────────────────────────────

    def close_session(self, terminated_normally: bool) -> None:
        if self._closed:
            return
        self._closed = True
        summary = aggregate_session(
            per_frame=self._per_frame,
            per_obstacle=self._per_obstacle,
            fp_per_frame=self._fp_per_frame,
            obstacle_specs=self._obstacles,
            density_distances=self._density_distances,
        )
        self._writer.close(summary=summary, terminated_normally=terminated_normally)
        self.get_logger().info(
            f"[tof_eval] closed: n_frames={summary.get('n_frames',0)} "
            f"terminated_normally={terminated_normally}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TofEvalNode()
    normal = False
    try:
        rclpy.spin(node)
        normal = True
    except KeyboardInterrupt:
        # 사용자가 Ctrl-C → 정상 종료로 간주 (partial 아님)
        normal = True
    finally:
        try:
            node.close_session(terminated_normally=normal)
        except Exception as e:
            node.get_logger().error(f"close_session failed: {e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
