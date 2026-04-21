"""
세션 산출물 기록기.

디렉토리 구조:
  <csv_output_dir>/
    sessions_index.csv            (마스터, append)
    <session_id>/
      meta.json
      frames.csv
      obstacles.csv
      summary.csv

frames/obstacles 는 per-row 즉시 append 로 저장 (crash 시에도 최대한 데이터 보존).
summary/meta/index 는 세션 종료 시점에 일괄 기록.
"""

import csv
import json
import os
from dataclasses import asdict
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from .metrics import PerFrameMetrics, PerObstacleMetrics


FRAMES_BASE_HEADERS = [
    'frame_idx', 'timestamp', 'dt_since_prev',
    'n_points_raw', 'n_points_valid', 'valid_zone_ratio',
    'ground_fit_ok', 'ground_inlier_ratio', 'ground_residual_std',
    'plane_normal_angle_deg',
]

OBSTACLE_HEADERS = [
    'frame_idx', 'obstacle_id', 'detected',
    'centroid_fwd', 'centroid_right', 'centroid_up',
    'loc_err_m',
    'aabb_fwd', 'aabb_right', 'aabb_up',
    'size_ratio_fwd', 'size_ratio_right', 'size_ratio_up',
    'point_count',
]


class SessionWriter:
    def __init__(
        self,
        output_dir: str,
        session_id: str,
        density_distances: List[float],
        meta: Dict[str, Any],
        algo_params: Dict[str, Any],
    ):
        self.output_root = output_dir
        self.session_id = session_id
        self.session_dir = os.path.join(output_dir, session_id)
        os.makedirs(self.session_dir, exist_ok=True)

        # A4 density 는 distance 만큼 column 추가
        self._density_cols = [f'fwd_density_{D:.1f}' for D in density_distances]
        self._density_distances = list(density_distances)
        self._frames_headers = FRAMES_BASE_HEADERS + self._density_cols

        self._frames_path = os.path.join(self.session_dir, 'frames.csv')
        self._obstacles_path = os.path.join(self.session_dir, 'obstacles.csv')
        self._summary_path = os.path.join(self.session_dir, 'summary.csv')
        self._meta_path = os.path.join(self.session_dir, 'meta.json')
        self._index_path = os.path.join(output_dir, 'sessions_index.csv')

        # frames.csv / obstacles.csv 는 즉시 header 작성
        with open(self._frames_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(self._frames_headers)
        with open(self._obstacles_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(OBSTACLE_HEADERS)

        # meta.json 은 세션 시작 시 바로 기록 (crash 대비)
        self._meta_snapshot = {
            'session_id': session_id,
            'started_at_utc': datetime.now(timezone.utc).isoformat(),
            'ended_at_utc': None,
            'terminated_normally': False,      # close() 에서 true 로 갱신
            'scene': meta,
            'algo_params': algo_params,
            'density_distances_m': self._density_distances,
        }
        self._write_meta()

    # ── per-frame append ─────────────────────────────────────
    def append_frame(self, m: PerFrameMetrics) -> None:
        row = [
            m.frame_idx, f'{m.timestamp:.6f}', f'{m.dt_since_prev:.6f}',
            m.n_points_raw, m.n_points_valid, f'{m.valid_zone_ratio:.6f}',
            int(m.ground_fit_ok),
            f'{m.ground_inlier_ratio:.6f}', f'{m.ground_residual_std:.6f}',
            f'{m.plane_normal_angle_deg:.6f}',
        ]
        for D in self._density_distances:
            row.append(f'{m.fwd_densities.get(D, float("nan")):.3f}')
        with open(self._frames_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

    def append_obstacles(self, rows: List[PerObstacleMetrics]) -> None:
        if not rows:
            return
        with open(self._obstacles_path, 'a', newline='') as f:
            w = csv.writer(f)
            for r in rows:
                w.writerow([
                    r.frame_idx, r.obstacle_id, int(r.detected),
                    f'{r.centroid_fwd:.6f}', f'{r.centroid_right:.6f}', f'{r.centroid_up:.6f}',
                    f'{r.loc_err_m:.6f}',
                    f'{r.aabb_fwd:.6f}', f'{r.aabb_right:.6f}', f'{r.aabb_up:.6f}',
                    f'{r.size_ratio_fwd:.6f}', f'{r.size_ratio_right:.6f}', f'{r.size_ratio_up:.6f}',
                    r.point_count,
                ])

    # ── 세션 종료 ───────────────────────────────────────────
    def close(
        self,
        summary: Dict[str, Any],
        terminated_normally: bool,
    ) -> None:
        self._meta_snapshot['ended_at_utc'] = datetime.now(timezone.utc).isoformat()
        self._meta_snapshot['terminated_normally'] = terminated_normally
        self._write_meta()

        # summary.csv — 단일 행. scene 필드도 함께 embed 해서 세션 단독으로 해석 가능
        scene = self._meta_snapshot['scene'] or {}
        combined = {
            'session_id': self.session_id,
            'terminated_normally': int(terminated_normally),
            **{f'scene_{k}': _to_str(v) for k, v in scene.items()},
            **{k: _fmt(v) for k, v in summary.items()},
        }
        with open(self._summary_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(combined.keys()))
            w.writeheader()
            w.writerow(combined)

        # sessions_index.csv — master append (헤더 호환성 위해 unionize)
        self._append_to_master_index(combined)

    # ── 내부 ────────────────────────────────────────────────
    def _write_meta(self) -> None:
        with open(self._meta_path, 'w') as f:
            json.dump(self._meta_snapshot, f, indent=2, default=_json_default)

    def _append_to_master_index(self, row: Dict[str, Any]) -> None:
        """
        Master index 가 존재하면 헤더를 union 해서 재작성, 없으면 신규 생성.
        세션마다 새 scene field 가 등장해도 호환 유지.
        """
        existing_rows: List[Dict[str, Any]] = []
        existing_headers: List[str] = []
        if os.path.exists(self._index_path):
            with open(self._index_path, 'r', newline='') as f:
                r = csv.DictReader(f)
                existing_headers = r.fieldnames or []
                existing_rows = [dict(line) for line in r]

        # 신규 row 의 key 중 기존에 없는 것 추가
        merged_headers = list(existing_headers)
        for k in row.keys():
            if k not in merged_headers:
                merged_headers.append(k)

        # 기존 row 들 + 신규 row 를 같은 header 로 재기록
        existing_rows.append(row)
        with open(self._index_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=merged_headers, extrasaction='ignore')
            w.writeheader()
            for line in existing_rows:
                # 없는 컬럼은 빈 문자열로
                w.writerow({k: line.get(k, '') for k in merged_headers})


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f'{v:.6f}'
    return str(v)


def _to_str(v: Any) -> str:
    if isinstance(v, (list, tuple)):
        return ','.join(str(x) for x in v)
    return str(v)


def _json_default(o):
    # numpy types → python
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    raise TypeError(f'not serializable: {type(o)}')
