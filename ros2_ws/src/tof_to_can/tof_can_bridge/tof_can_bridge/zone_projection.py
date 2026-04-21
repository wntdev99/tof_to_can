"""
Multi-zone ToF (VL53L5CX / L7CX) zone 배열 → 3D point 투영.

좌표계: ROS optical frame 관례 — Z forward, X right, Y down (REP-103 camera).
근거:
  - VL53L5CX/L7CX 는 8×8 zone grid, zone-center-to-zone-center axis FoV ≈ 43°
    (ST community: "FoV and Zone Area for VL53L5CX")
  - 각 zone 은 자체 5-6° 콘이지만 PointCloud2 관례상 zone center 방향 단일
    ray 로 근사한다 (tof_imager_ros 와 동일 패턴).

투영 공식 (단위 ray):
  az = (c - 3.5) / 3.5 · (FoV_axis / 2)   [-21.5° … +21.5°]
  el = (r - 3.5) / 3.5 · (FoV_axis / 2)
  ray = (sin(az)·cos(el), sin(el), cos(az)·cos(el))   # Z forward
  point = distance · ray

zone index 규칙:
  ST ULD API 는 zone 0 을 top-left (센서 정면에서 봤을 때) 로 주는 경우가
  많으나, CAN 프레임은 firmware 측 배열 순서를 그대로 전송한다. 필요 시
  flip_rows / flip_cols 파라미터로 맞춘다.
"""

import math
from typing import List, Tuple

# ST datasheet / community Q&A 기준 zone-center axis FoV
DEFAULT_FOV_AXIS_DEG = 43.0
DEFAULT_GRID = 8  # 8×8


def build_unit_rays(
    grid: int = DEFAULT_GRID,
    fov_axis_deg: float = DEFAULT_FOV_AXIS_DEG,
    flip_rows: bool = False,
    flip_cols: bool = False,
) -> List[Tuple[float, float, float]]:
    """zone index (0..grid*grid-1) 순서대로 unit ray (x, y, z) 를 반환.

    row-major: idx = r * grid + c.
    """
    if grid < 2:
        raise ValueError("grid must be >= 2")

    half_fov = math.radians(fov_axis_deg) / 2.0
    step = 2.0 * half_fov / (grid - 1)   # zone-center 간격 (radian)
    rays: List[Tuple[float, float, float]] = []

    for r in range(grid):
        rr = (grid - 1 - r) if flip_rows else r
        el = -half_fov + rr * step        # row → elevation (+ = down)
        for c in range(grid):
            cc = (grid - 1 - c) if flip_cols else c
            az = -half_fov + cc * step    # col → azimuth (+ = right)
            x = math.sin(az) * math.cos(el)
            y = math.sin(el)
            z = math.cos(az) * math.cos(el)
            rays.append((x, y, z))
    return rays


def project_distances_mm(
    distances_mm: List[int],
    unit_rays: List[Tuple[float, float, float]],
    min_range_m: float = 0.02,
    max_range_m: float = 4.0,
) -> List[Tuple[float, float, float]]:
    """zone 별 distance_mm 를 unit ray 에 곱해 XYZ (meter) 를 반환.

    유효 범위 밖은 제외. distances_mm 와 unit_rays 길이가 맞아야 한다.
    """
    points = []
    for d_mm, (rx, ry, rz) in zip(distances_mm, unit_rays):
        if d_mm <= 0:
            continue
        d_m = d_mm / 1000.0
        if d_m < min_range_m or d_m > max_range_m:
            continue
        points.append((d_m * rx, d_m * ry, d_m * rz))
    return points
