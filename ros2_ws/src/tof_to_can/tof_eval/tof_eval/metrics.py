"""
지표 계산 — Group A (센서 raw 품질), Group B (ground plane), Group C (obstacle).

Per-frame 지표는 PerFrameMetrics, 세션 aggregate 는 SessionSummary 로 분리.
Obstacle 관련은 per-frame × obstacle grain 으로 PerObstacleMetrics 리스트 발행.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .clustering import Cluster


# ── per-frame ──────────────────────────────────────────────────

@dataclass
class PerFrameMetrics:
    frame_idx: int
    timestamp: float                  # ROS time (sec, float)
    dt_since_prev: float              # 직전 frame 과의 간격 (s). 첫 프레임은 NaN

    # Group A
    n_points_raw: int                 # input cloud 의 원시 점 수 (NaN drop 전)
    n_points_valid: int               # 유효 점 수
    valid_zone_ratio: float           # A1: n_points_valid / expected_count. expected 미설정 시 NaN

    # Group B
    ground_fit_ok: bool               # B4 판정 (not failed)
    ground_inlier_ratio: float        # B1
    ground_residual_std: float        # B2 (m)
    plane_normal_angle_deg: float     # B3: 직전 frame normal 과의 각도. 첫 frame NaN

    # Group A4: forward distance 별 density (points/m²)
    fwd_densities: Dict[float, float] = field(default_factory=dict)


@dataclass
class PerObstacleMetrics:
    frame_idx: int
    obstacle_id: str
    detected: bool

    # 이하 detected=True 일 때만 유효
    centroid_fwd: float = float('nan')
    centroid_right: float = float('nan')
    centroid_up: float = float('nan')
    loc_err_m: float = float('nan')   # expected 와 centroid 거리

    aabb_fwd: float = float('nan')
    aabb_right: float = float('nan')
    aabb_up: float = float('nan')

    size_ratio_fwd: float = float('nan')
    size_ratio_right: float = float('nan')
    size_ratio_up: float = float('nan')

    point_count: int = 0

    # Frame 기준 false positive cluster 수 — obstacle row 대신 frame-level 로 갈까
    # 생각했으나, obstacle-specific 이 아니므로 별도 경로로 빠짐 (PerFrameMetrics 에는
    # 안 넣고 Eval 노드에서 따로 집계)


# ── A4 density 계산 ──────────────────────────────────────────────

def forward_slab_density(
    points_wf_inlier: np.ndarray,
    distances: List[float],
    slab_thickness: float,
    lateral_window: float,
) -> Dict[float, float]:
    """
    inlier 점 (ground 위) 을 working frame 으로 변환한 배열에서 forward 거리별 slab density.

    points_wf_inlier: Nx3 (fwd, right, up) — RANSAC inlier 만 포함
    distances: 샘플링할 전방 거리 (m)
    slab_thickness: fwd 슬래브 두께 (m)  — slab = [D - t/2, D + t/2]
    lateral_window: right 방향 수용 폭 (m) — right in [-w/2, w/2]
    """
    if points_wf_inlier.shape[0] == 0:
        return {d: 0.0 for d in distances}

    fwd = points_wf_inlier[:, 0]
    rgt = points_wf_inlier[:, 1]
    half_t = slab_thickness * 0.5
    half_w = lateral_window * 0.5
    lat_mask = (rgt >= -half_w) & (rgt <= half_w)
    slab_area = slab_thickness * lateral_window

    out: Dict[float, float] = {}
    for D in distances:
        mask = lat_mask & (fwd >= D - half_t) & (fwd <= D + half_t)
        count = int(mask.sum())
        out[D] = float(count) / slab_area
    return out


# ── Obstacle matching ───────────────────────────────────────────

@dataclass
class ObstacleSpec:
    """Launch-time 파라미터로 들어오는 obstacle 정의 (working frame 좌표)."""
    obstacle_id: str
    expected_xyz: Tuple[float, float, float]   # (fwd, right, up)
    size_xyz: Tuple[float, float, float]       # (fwd, right, up)
    search_radius: float


def match_obstacles(
    clusters: List[Cluster],
    obstacles: List[ObstacleSpec],
    frame_idx: int,
) -> Tuple[List[PerObstacleMetrics], int]:
    """
    각 obstacle 에 대해 search_radius 안 가장 가까운 cluster 를 매칭.
    반환: (per-obstacle 지표 리스트, false positive cluster 수)

    False positive: 어떤 obstacle 과도 매칭되지 않은 cluster.
    """
    claimed = np.zeros(len(clusters), dtype=bool)
    obstacle_results: List[PerObstacleMetrics] = []

    # 각 obstacle 별로 독립 매칭 (가까운 cluster claim)
    for spec in obstacles:
        best_idx = -1
        best_dist = float('inf')
        exp = np.asarray(spec.expected_xyz, dtype=np.float64)

        for ci, cl in enumerate(clusters):
            if claimed[ci]:
                continue
            d = float(np.linalg.norm(cl.centroid - exp))
            if d <= spec.search_radius and d < best_dist:
                best_dist = d
                best_idx = ci

        if best_idx < 0:
            obstacle_results.append(PerObstacleMetrics(
                frame_idx=frame_idx,
                obstacle_id=spec.obstacle_id,
                detected=False,
            ))
            continue

        claimed[best_idx] = True
        cl = clusters[best_idx]
        size = cl.size  # (fwd, right, up)
        ratio = np.where(
            np.asarray(spec.size_xyz) > 1e-6,
            size / np.asarray(spec.size_xyz),
            np.nan,
        )
        obstacle_results.append(PerObstacleMetrics(
            frame_idx=frame_idx,
            obstacle_id=spec.obstacle_id,
            detected=True,
            centroid_fwd=float(cl.centroid[0]),
            centroid_right=float(cl.centroid[1]),
            centroid_up=float(cl.centroid[2]),
            loc_err_m=best_dist,
            aabb_fwd=float(size[0]),
            aabb_right=float(size[1]),
            aabb_up=float(size[2]),
            size_ratio_fwd=float(ratio[0]),
            size_ratio_right=float(ratio[1]),
            size_ratio_up=float(ratio[2]),
            point_count=cl.point_count,
        ))

    fp_count = int(np.sum(~claimed))
    return obstacle_results, fp_count


# ── Session 집계 ─────────────────────────────────────────────────

def _nanmean(a) -> float:
    arr = np.asarray(a, dtype=np.float64)
    if arr.size == 0:
        return float('nan')
    with np.errstate(all='ignore'):
        return float(np.nanmean(arr))


def _nanstd(a) -> float:
    arr = np.asarray(a, dtype=np.float64)
    if arr.size == 0:
        return float('nan')
    with np.errstate(all='ignore'):
        return float(np.nanstd(arr))


def aggregate_session(
    per_frame: List[PerFrameMetrics],
    per_obstacle: List[PerObstacleMetrics],
    fp_per_frame: List[int],
    obstacle_specs: List[ObstacleSpec],
    density_distances: List[float],
) -> Dict[str, float]:
    """세션 전체 집계 — summary.csv 및 sessions_index.csv 의 한 행."""
    n_frames = len(per_frame)
    out: Dict[str, float] = {}
    out['n_frames'] = n_frames
    if n_frames == 0:
        return out

    # A 계열
    valid_counts = [m.n_points_valid for m in per_frame]
    out['valid_count_mean'] = _nanmean(valid_counts)
    out['valid_count_cv'] = (
        _nanstd(valid_counts) / out['valid_count_mean']
        if out['valid_count_mean'] > 0 else float('nan')
    )
    out['valid_zone_ratio_mean'] = _nanmean([m.valid_zone_ratio for m in per_frame])

    # A3: Hz
    dts = [m.dt_since_prev for m in per_frame if not np.isnan(m.dt_since_prev)]
    dt_mean = _nanmean(dts) if dts else float('nan')
    out['effective_hz'] = (1.0 / dt_mean) if dt_mean and dt_mean > 0 else float('nan')

    # A4: 거리별 density
    for D in density_distances:
        vals = [m.fwd_densities.get(D, float('nan')) for m in per_frame]
        out[f'fwd_density_{D:.1f}_mean'] = _nanmean(vals)
        out[f'fwd_density_{D:.1f}_std'] = _nanstd(vals)

    # B 계열
    out['ground_inlier_ratio_mean'] = _nanmean([m.ground_inlier_ratio for m in per_frame])
    out['ground_residual_std_mean'] = _nanmean([m.ground_residual_std for m in per_frame])
    out['plane_normal_jitter_std'] = _nanstd([
        m.plane_normal_angle_deg for m in per_frame
        if not np.isnan(m.plane_normal_angle_deg)
    ])
    fit_fail = sum(1 for m in per_frame if not m.ground_fit_ok)
    out['ground_fit_fail_rate'] = fit_fail / n_frames

    # C 계열 — per obstacle
    per_obs_by_id: Dict[str, List[PerObstacleMetrics]] = {}
    for m in per_obstacle:
        per_obs_by_id.setdefault(m.obstacle_id, []).append(m)

    for spec in obstacle_specs:
        rows = per_obs_by_id.get(spec.obstacle_id, [])
        prefix = f'obs_{spec.obstacle_id}'
        if not rows:
            out[f'{prefix}_detection_rate'] = float('nan')
            out[f'{prefix}_loc_err_mean'] = float('nan')
            out[f'{prefix}_size_ratio_fwd_mean'] = float('nan')
            continue
        detected = [r for r in rows if r.detected]
        out[f'{prefix}_detection_rate'] = len(detected) / len(rows)
        out[f'{prefix}_loc_err_mean'] = _nanmean([r.loc_err_m for r in detected])
        out[f'{prefix}_size_ratio_fwd_mean'] = _nanmean([r.size_ratio_fwd for r in detected])
        out[f'{prefix}_size_ratio_right_mean'] = _nanmean([r.size_ratio_right for r in detected])
        out[f'{prefix}_size_ratio_up_mean'] = _nanmean([r.size_ratio_up for r in detected])

    # C2: FP rate (per minute)
    hz = out.get('effective_hz', float('nan'))
    fp_mean = _nanmean(fp_per_frame)
    if not np.isnan(hz) and hz > 0:
        out['fp_per_minute'] = fp_mean * hz * 60.0
    else:
        out['fp_per_minute'] = float('nan')
    out['fp_per_frame_mean'] = fp_mean

    return out
