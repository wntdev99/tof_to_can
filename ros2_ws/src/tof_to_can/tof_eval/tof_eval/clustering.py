"""
Euclidean clustering — KDTree 기반 BFS.

ToF 는 zone 이 sparse (최대 64) 하므로 무거운 라이브러리 대신 scipy.spatial.KDTree
위에 BFS 한 레이어 올려 구현한다.
"""

from dataclasses import dataclass
from typing import List

import numpy as np
from scipy.spatial import KDTree


@dataclass
class Cluster:
    """working frame 기준 cluster 특성."""
    indices: np.ndarray         # 원본 점 배열에서의 인덱스
    centroid: np.ndarray        # (3,) — fwd, right, up
    aabb_min: np.ndarray        # (3,)
    aabb_max: np.ndarray        # (3,)

    @property
    def size(self) -> np.ndarray:
        """AABB 크기 (fwd_ext, right_ext, up_ext)."""
        return self.aabb_max - self.aabb_min

    @property
    def point_count(self) -> int:
        return int(self.indices.shape[0])


def euclidean_cluster(
    points_wf: np.ndarray,
    tolerance: float = 0.05,
    min_points: int = 3,
    max_points: int = 10000,
) -> List[Cluster]:
    """
    points_wf: Nx3 working-frame 좌표.
    tolerance: BFS 이웃 반경 (m).
    min/max_points: cluster 크기 필터.
    """
    n = points_wf.shape[0]
    if n == 0:
        return []

    tree = KDTree(points_wf)
    visited = np.zeros(n, dtype=bool)
    clusters: List[Cluster] = []

    for seed in range(n):
        if visited[seed]:
            continue
        # BFS
        queue = [seed]
        member: List[int] = []
        visited[seed] = True
        while queue:
            idx = queue.pop()
            member.append(idx)
            neighbors = tree.query_ball_point(points_wf[idx], r=tolerance)
            for nb in neighbors:
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)

        if len(member) < min_points or len(member) > max_points:
            continue
        idx_arr = np.asarray(member, dtype=np.int64)
        pts = points_wf[idx_arr]
        clusters.append(Cluster(
            indices=idx_arr,
            centroid=pts.mean(axis=0),
            aabb_min=pts.min(axis=0),
            aabb_max=pts.max(axis=0),
        ))

    return clusters


def filter_above_ground(
    points_wf: np.ndarray,
    min_height: float,
    max_height: float = 2.0,
) -> np.ndarray:
    """
    up 성분 (index 2) 이 [min_height, max_height] 범위 안인 점의 bool mask.
    바닥·천장 제거용.
    """
    up = points_wf[:, 2]
    return (up >= min_height) & (up <= max_height)
