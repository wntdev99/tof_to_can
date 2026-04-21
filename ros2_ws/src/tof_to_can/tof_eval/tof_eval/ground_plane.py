"""
Ground plane RANSAC + working frame 산출.

TF 가 캘리브레이션되어 있지 않다는 전제에서, 매 프레임 센서가 보고 있는
바닥 평면을 RANSAC 으로 직접 추정하고 거기서 working frame (forward,
right, up) 을 만들어 센서 간 비교가 가능한 공통 좌표계를 확보한다.

working frame 정의:
  up      = ground plane normal (센서 방향을 양수로)
  forward = 센서 optical axis (+Z) 를 ground plane 에 투영한 단위벡터
  right   = up × forward   → (forward, right, up) 은 right-handed
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class GroundPlane:
    """RANSAC 결과.

    plane 식:  n · p = d  (n 은 단위벡터, 센서 origin 이 +n 측에 오도록 부호 정렬됨)
    """
    normal: np.ndarray          # shape (3,)
    d: float
    inlier_mask: np.ndarray     # bool (N,)
    inlier_ratio: float
    residual_std: float         # inlier 의 signed distance stddev (m)


@dataclass
class WorkingFrame:
    """working frame 기저벡터 (sensor frame 내 단위벡터)."""
    forward: np.ndarray         # (3,) 평면 위 vector
    right: np.ndarray           # (3,) 평면 위 vector
    up: np.ndarray              # (3,) = normal
    plane_d: float              # plane 식 d (up 계산용)


def fit_ground_plane(
    points: np.ndarray,
    distance_threshold: float = 0.02,
    max_iterations: int = 100,
    min_inlier_ratio: float = 0.3,
    rng: Optional[np.random.Generator] = None,
) -> Optional[GroundPlane]:
    """
    3-점 RANSAC 으로 dominant plane 을 찾아 반환. 실패 시 None.

    실패 조건:
      - 입력 점이 3 개 미만
      - 최대 inlier 비율이 min_inlier_ratio 미만
    """
    n = points.shape[0]
    if n < 3:
        return None

    if rng is None:
        rng = np.random.default_rng()

    best_mask: Optional[np.ndarray] = None
    best_inliers = 0
    best_normal: Optional[np.ndarray] = None
    best_d = 0.0

    for _ in range(max_iterations):
        # 3 개 서로 다른 점 샘플
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx[0]], points[idx[1]], points[idx[2]]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            continue  # colinear
        normal = normal / norm_len
        d = float(np.dot(normal, p0))

        # signed distance
        dist = np.abs(points @ normal - d)
        mask = dist < distance_threshold
        inliers = int(mask.sum())
        if inliers > best_inliers:
            best_inliers = inliers
            best_mask = mask
            best_normal = normal
            best_d = d

    if best_mask is None or best_inliers < 3:
        return None

    inlier_ratio = best_inliers / n
    if inlier_ratio < min_inlier_ratio:
        return None

    # Least-squares refinement on inliers (SVD of centered points)
    inlier_pts = points[best_mask]
    centroid = inlier_pts.mean(axis=0)
    centered = inlier_pts - centroid
    # plane normal = smallest singular vector
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    d = float(np.dot(normal, centroid))

    # 센서 origin (0,0,0) 이 +n 측에 오도록 부호 정렬
    # signed distance of origin = 0 - d = -d → 양수가 되도록 d < 0 으로
    if d > 0:
        normal = -normal
        d = -d

    # Refined residual std
    signed = points[best_mask] @ normal - d
    residual_std = float(np.std(signed))

    return GroundPlane(
        normal=normal,
        d=d,
        inlier_mask=best_mask,
        inlier_ratio=inlier_ratio,
        residual_std=residual_std,
    )


def build_working_frame(plane: GroundPlane) -> Optional[WorkingFrame]:
    """ground plane 에서 (forward, right, up) 기저 생성. 퇴화 시 None."""
    up = plane.normal.astype(np.float64)

    # 센서 optical axis (+Z) 를 plane 에 투영
    z_cam = np.array([0.0, 0.0, 1.0])
    z_proj = z_cam - np.dot(z_cam, up) * up
    if np.linalg.norm(z_proj) < 1e-6:
        # 센서가 plane normal 방향을 똑바로 바라봄 (거의 일어나지 않는 경우)
        # fallback: X 축으로 대체
        x_cam = np.array([1.0, 0.0, 0.0])
        z_proj = x_cam - np.dot(x_cam, up) * up
        if np.linalg.norm(z_proj) < 1e-6:
            return None
    forward = z_proj / np.linalg.norm(z_proj)
    right = np.cross(up, forward)
    right = right / np.linalg.norm(right)

    return WorkingFrame(
        forward=forward.astype(np.float64),
        right=right.astype(np.float64),
        up=up,
        plane_d=plane.d,
    )


def to_working_frame(points: np.ndarray, wf: WorkingFrame) -> np.ndarray:
    """
    Sensor-frame Nx3 → working-frame Nx3 (fwd, right, up_from_ground).

    forward/right 는 plane 에 평행하므로 normal-component 가 자동으로 빠진다.
    up = n·p - d  (센서 origin 이 +n 측에 있도록 부호 정렬되어 있음)
    """
    pts = points.astype(np.float64, copy=False)
    fwd = pts @ wf.forward
    rgt = pts @ wf.right
    up = pts @ wf.up - wf.plane_d
    return np.stack([fwd, rgt, up], axis=1)


def normal_angle_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """두 normal 사이 각도 (도). Jitter 계산용."""
    cos = float(np.clip(np.dot(n1, n2) / (np.linalg.norm(n1) * np.linalg.norm(n2)),
                        -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))
