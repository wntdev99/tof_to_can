"""
PointCloud2 → numpy Nx3 변환 유틸리티.

센서별 field 레이아웃이 다를 수 있어 (tof_can_bridge 는 float32 xyz만 발행,
Gemini depth/points 는 rgb/intensity 등 동반) 동적으로 x/y/z offset·dtype 을
파싱해 numpy structured array 로 디코딩한다.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


# PointField.datatype → numpy dtype
_DTYPE_MAP = {
    PointField.INT8:    np.int8,
    PointField.UINT8:   np.uint8,
    PointField.INT16:   np.int16,
    PointField.UINT16:  np.uint16,
    PointField.INT32:   np.int32,
    PointField.UINT32:  np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    PointCloud2 → Nx3 float32 array (meter).

    - x/y/z field 이 없으면 ValueError
    - count != 1 인 field (vector field) 는 지원하지 않음
    - NaN / Inf 점은 drop (is_dense=False 인 경우 대비)
    """
    fields_by_name = {f.name: f for f in msg.fields}
    required = ('x', 'y', 'z')
    for name in required:
        if name not in fields_by_name:
            raise ValueError(f"PointCloud2 missing field '{name}'")

    n_points = msg.width * msg.height
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    point_step = msg.point_step
    if len(msg.data) < n_points * point_step:
        raise ValueError(
            f"data length {len(msg.data)} < expected {n_points * point_step}"
        )

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8,
                        count=n_points * point_step).reshape(n_points, point_step)

    out = np.empty((n_points, 3), dtype=np.float32)
    for i, name in enumerate(required):
        f = fields_by_name[name]
        dtype = _DTYPE_MAP.get(f.datatype)
        if dtype is None:
            raise ValueError(f"unsupported datatype {f.datatype} for field '{name}'")
        # Row 단위 slice → bytes view → dtype 해석
        # offset 은 point_step 내부 바이트 오프셋
        byte_count = np.dtype(dtype).itemsize
        col = buf[:, f.offset:f.offset + byte_count].copy().view(dtype).reshape(-1)
        out[:, i] = col.astype(np.float32, copy=False)

    # Drop non-finite (NaN/Inf) — Gemini 같은 센서는 invalid 를 NaN 으로 채움
    mask = np.isfinite(out).all(axis=1)
    if not mask.all():
        out = out[mask]

    return out


def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    """
    Voxel grid downsampling. leaf ≤ 0 이면 no-op.

    각 voxel 에서 첫 번째 점 1개를 유지한다 (centroid 보다 빠르고,
    RANSAC / BFS clustering 입력 목적으로 충분).
    """
    if leaf <= 0.0 or pts.shape[0] == 0:
        return pts
    inv_leaf = 1.0 / leaf
    voxel_idx = np.floor(pts * inv_leaf).astype(np.int64)
    _, first_occ = np.unique(voxel_idx, axis=0, return_index=True)
    return pts[first_occ]
