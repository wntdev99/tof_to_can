"""
Multi-zone ToF (VL53L5CX / L7CX) zone 배열 → 3D point 투영.

좌표계: ROS optical frame 관례 — Z forward, X right, Y down (REP-103 camera).

distance_mm 해석:
  VL53L5CX/L7CX 는 내부적으로 radial → perpendicular 변환을 완료한 뒤
  z-depth (센서 평면에 수직인 방향의 거리) 를 반환한다.
  출처: ST Community, John E KVAM (ST Employee), 2022-02-17
        "We then translate that 'radial' measure to a perpendicular one."

  따라서 올바른 XYZ 변환:
    Zpos = distance_mm                          (z-depth 그대로)
    Hyp  = distance_mm / sin(Pitch)             (z-depth → radial 역산)
    Xpos = cos(Yaw) * cos(Pitch) * Hyp
    Ypos = sin(Yaw) * cos(Pitch) * Hyp  (ST Y-up → ROS Y-down 부호 반전 적용)

  unit ray 를 normalize(sx, sy, 1.0) 으로 정의하면:
    d_radial = d_z / rz
    point    = d_radial * (rx, ry, rz)  →  (d_z*rx/rz, d_z*ry/rz, d_z)

8×8 zone LUT:
  균일 각도 근사(fov_axis_deg) 대신 ST 제공 Pitch/Yaw LUT 를 사용해
  zone 별 실제 방향을 정확하게 반영한다.
  grid ≠ 8 인 경우에만 fov_axis_deg 파라미터 기반 균일 근사로 fallback.

zone index 규칙:
  ST ULD API 는 zone 0 을 top-left (센서 정면에서 봤을 때) 로 주는 경우가
  많으나, CAN 프레임은 firmware 측 배열 순서를 그대로 전송한다. 필요 시
  flip_rows / flip_cols 파라미터로 맞춘다.
"""

import math
from typing import List, Tuple

DEFAULT_FOV_AXIS_DEG = 43.0
DEFAULT_GRID = 8


# ── ST 공식 8×8 Pitch/Yaw LUT ────────────────────────────────────────────────
# 출처: ST Community — John E KVAM (ST Employee), 2022-02-17
# Pitch: xy-평면으로부터의 고도각 (= 90° - polar_angle).  sin(Pitch) = cos(θ)
# Yaw:   x-축 기준 방위각, ST 좌표계 기준 (Y-up)
# index = row * 8 + col,  row 0 = top,  col 0 = left  (센서 정면 기준)

_ST_PITCH_8X8: List[float] = [
    62.85, 66.50, 69.40, 71.08, 71.08, 69.40, 66.50, 62.85,
    66.50, 70.81, 75.05, 77.50, 77.50, 75.05, 70.81, 66.50,
    69.40, 75.05, 78.15, 81.76, 81.76, 78.15, 75.05, 69.40,
    71.08, 77.50, 81.76, 86.00, 86.00, 81.76, 77.50, 71.08,
    71.08, 77.50, 81.76, 86.00, 86.00, 81.76, 77.50, 71.08,
    69.40, 75.05, 78.15, 81.76, 81.76, 78.15, 75.05, 69.40,
    66.50, 70.81, 75.05, 77.50, 77.50, 75.05, 70.81, 66.50,
    62.85, 66.50, 69.40, 71.08, 71.08, 69.40, 66.50, 62.85,
]

_ST_YAW_8X8: List[float] = [
    135.00, 125.40, 113.20,  98.13,  81.87,  66.80,  54.60,  45.00,
    144.60, 135.00, 120.96, 101.31,  78.69,  59.04,  45.00,  35.40,
    156.80, 149.04, 135.00, 108.45,  71.55,  45.00,  30.96,  23.20,
    171.87, 168.69, 161.55, 135.00,  45.00,  18.45,  11.31,   8.13,
    188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
    203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
    203.20, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
    225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00,
]


# ── LUT 기반 8×8 unit ray 계산 ────────────────────────────────────────────────

def _build_unit_rays_st_lut(
    flip_rows: bool,
    flip_cols: bool,
) -> List[Tuple[float, float, float]]:
    rays: List[Tuple[float, float, float]] = []
    for r in range(8):
        rr = (7 - r) if flip_rows else r
        for c in range(8):
            cc = (7 - c) if flip_cols else c
            idx = rr * 8 + cc
            pitch = math.radians(_ST_PITCH_8X8[idx])
            yaw   = math.radians(_ST_YAW_8X8[idx])
            sp = math.sin(pitch)
            cp = math.cos(pitch)
            # z-depth 기준 스케일: point = d_z * (sx, sy, 1.0)
            # ST Y-up → ROS optical Y-down: sin(Yaw) 부호 반전
            sx =  math.cos(yaw) * cp / sp
            sy = -math.sin(yaw) * cp / sp
            sz = 1.0
            n = math.sqrt(sx * sx + sy * sy + sz * sz)
            rays.append((sx / n, sy / n, sz / n))
    return rays


# ── 균일 각도 근사 (grid ≠ 8 fallback) ───────────────────────────────────────

def _build_unit_rays_uniform(
    grid: int,
    fov_axis_deg: float,
    flip_rows: bool,
    flip_cols: bool,
) -> List[Tuple[float, float, float]]:
    half_fov = math.radians(fov_axis_deg) / 2.0
    step = 2.0 * half_fov / (grid - 1)
    rays: List[Tuple[float, float, float]] = []
    for r in range(grid):
        rr = (grid - 1 - r) if flip_rows else r
        el = -half_fov + rr * step
        for c in range(grid):
            cc = (grid - 1 - c) if flip_cols else c
            az = -half_fov + cc * step
            # distance_mm 가 z-depth 이므로: point = d_z * (tan(az), tan(el), 1.0)
            sx = math.tan(az)
            sy = math.tan(el)
            sz = 1.0
            n = math.sqrt(sx * sx + sy * sy + sz * sz)
            rays.append((sx / n, sy / n, sz / n))
    return rays


# ── 공개 인터페이스 ───────────────────────────────────────────────────────────

def build_unit_rays(
    grid: int = DEFAULT_GRID,
    fov_axis_deg: float = DEFAULT_FOV_AXIS_DEG,
    flip_rows: bool = False,
    flip_cols: bool = False,
) -> List[Tuple[float, float, float]]:
    """zone index (0..grid*grid-1) 순서대로 unit ray (x, y, z) 를 반환.

    grid=8 이면 ST 공식 Pitch/Yaw LUT 를 사용하고,
    그 외에는 fov_axis_deg 기반 균일 각도 근사로 fallback 한다.
    row-major: idx = r * grid + c.
    """
    if grid < 2:
        raise ValueError("grid must be >= 2")
    if grid == 8:
        return _build_unit_rays_st_lut(flip_rows, flip_cols)
    return _build_unit_rays_uniform(grid, fov_axis_deg, flip_rows, flip_cols)


def project_distances_mm(
    distances_mm: List[int],
    unit_rays: List[Tuple[float, float, float]],
    min_range_m: float = 0.02,
    max_range_m: float = 4.0,
) -> List[Tuple[float, float, float]]:
    """zone 별 distance_mm (z-depth) 를 unit ray 에 투영해 XYZ (meter) 를 반환.

    distance_mm 는 센서가 반환하는 z-depth (perpendicular distance) 이므로
    radial 로 역산한 뒤 unit ray 에 곱한다:
      d_radial = d_z / rz
      point    = d_radial * (rx, ry, rz)  →  (d_z*rx/rz, d_z*ry/rz, d_z)

    min_range_m / max_range_m 는 z-depth 기준으로 필터링한다.
    distances_mm 와 unit_rays 길이가 맞아야 한다.
    """
    points = []
    for d_mm, (rx, ry, rz) in zip(distances_mm, unit_rays):
        if d_mm <= 0:
            continue
        d_z = d_mm / 1000.0
        if d_z < min_range_m or d_z > max_range_m:
            continue
        d_r = d_z / rz
        points.append((d_r * rx, d_r * ry, d_r * rz))
    return points
