#!/usr/bin/env python3
"""
동일 scene 에서 센서별 대응 세션을 자동으로 찾아 side-by-side 비교표를 출력한다.

매칭 기준: scene_floor_material + scene_floor_color + scene_ambient_light +
         scene_obstacle_surface 4 필드 일치.

사용:
  python3 pair_sensors.py /tmp/tof_eval --sensors VL53L5CX,GEMINI_336
  python3 pair_sensors.py /tmp/tof_eval --sensors VL53L5CX,VL53L7CX,GEMINI_336 \\
                                         --metric obs_cube_a_detection_rate
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from typing import Dict, List, Tuple


SCENE_KEY = ('scene_floor_material', 'scene_floor_color',
             'scene_ambient_light', 'scene_obstacle_surface')


def main():
    ap = argparse.ArgumentParser(description='Pair sessions by scene across sensors')
    ap.add_argument('root', help='tof_eval 출력 루트')
    ap.add_argument('--sensors', required=True,
                    help='비교할 센서 (comma 분리, 예: VL53L5CX,GEMINI_336)')
    ap.add_argument('--metric', default='',
                    help='특정 metric 만 표시. 지정 안 하면 주요 지표 전체')
    args = ap.parse_args()

    sensors: List[str] = [s.strip() for s in args.sensors.split(',') if s.strip()]
    index_path = os.path.join(args.root, 'sessions_index.csv')
    if not os.path.exists(index_path):
        print(f"[err] not found: {index_path}", file=sys.stderr)
        return 1

    with open(index_path, 'r', newline='') as f:
        rows = [dict(r) for r in csv.DictReader(f)]

    # scene key → {sensor: row}
    buckets: Dict[Tuple, Dict[str, dict]] = defaultdict(dict)
    for r in rows:
        key = tuple(r.get(k, '') for k in SCENE_KEY)
        sensor = r.get('scene_sensor', '')
        if sensor in sensors:
            buckets[key][sensor] = r

    # 완전 매칭만 (모든 sensor 가 있는 scene)
    pairs = {k: v for k, v in buckets.items() if all(s in v for s in sensors)}

    if not pairs:
        print("[warn] no fully-paired scenes found", file=sys.stderr)
        return 0

    default_metrics = [
        'n_frames', 'effective_hz',
        'valid_count_mean', 'valid_count_cv',
        'ground_inlier_ratio_mean', 'ground_residual_std_mean',
        'plane_normal_jitter_std', 'ground_fit_fail_rate',
        'fp_per_minute',
    ]
    # obstacle detection rate 자동 포함
    obs_cols = set()
    for p in pairs.values():
        for r in p.values():
            obs_cols.update(k for k in r.keys()
                            if k.startswith('obs_') and k.endswith('_detection_rate'))
    default_metrics.extend(sorted(obs_cols))

    metrics = [args.metric] if args.metric else default_metrics

    for key, sensor_rows in pairs.items():
        print('\n=== Scene: ' + ' / '.join(f'{k}={v}' for k, v in zip(SCENE_KEY, key)) + ' ===')
        headers = ['metric'] + sensors
        widths = [max(len(h), 20) for h in headers]
        print('  '.join(f'{{:<{w}}}'.format(h) for h, w in zip(headers, widths)))
        for m in metrics:
            cells = [m]
            for s in sensors:
                cells.append(sensor_rows[s].get(m, '—'))
            print('  '.join(f'{{:<{w}}}'.format(c) for c, w in zip(cells, widths)))

    return 0


if __name__ == '__main__':
    sys.exit(main())
