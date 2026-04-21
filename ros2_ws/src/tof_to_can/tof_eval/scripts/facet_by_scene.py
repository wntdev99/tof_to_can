#!/usr/bin/env python3
"""
Scene 축 하나에 대해 지표를 grouping 해서 평균·표준편차 표로 출력한다.

사용:
  python3 facet_by_scene.py /tmp/tof_eval \\
      --group-by ambient_light --metric obs_cube_a_detection_rate

  python3 facet_by_scene.py /tmp/tof_eval \\
      --group-by floor_material --metric ground_inlier_ratio_mean \\
      --filter scene_sensor=VL53L5CX
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List


def parse_filter(filters: List[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for f in filters or []:
        if '=' in f:
            k, v = f.split('=', 1)
            out[k.strip()] = v.strip()
    return out


def _to_float(s: str) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def main():
    ap = argparse.ArgumentParser(description='Group sessions by scene axis, aggregate metric')
    ap.add_argument('root', help='tof_eval 출력 루트')
    ap.add_argument('--group-by', required=True,
                    help='grouping 축 (예: ambient_light, floor_material, scene_sensor)')
    ap.add_argument('--metric', required=True, help='집계할 지표 컬럼명')
    ap.add_argument('--filter', action='append', default=[],
                    help='key=value 필터 (여러 번 가능)')
    args = ap.parse_args()

    index_path = os.path.join(args.root, 'sessions_index.csv')
    if not os.path.exists(index_path):
        print(f"[err] not found: {index_path}", file=sys.stderr)
        return 1

    with open(index_path, 'r', newline='') as f:
        rows = [dict(r) for r in csv.DictReader(f)]

    group_key = args.group_by
    if group_key not in (rows[0].keys() if rows else []):
        # scene_ prefix 자동 보강
        alt = f'scene_{group_key}'
        if rows and alt in rows[0]:
            group_key = alt

    flt = parse_filter(args.filter)
    # scene_* prefix 자동 보강
    for k in list(flt.keys()):
        if rows and k not in rows[0] and f'scene_{k}' in rows[0]:
            flt[f'scene_{k}'] = flt.pop(k)

    rows = [r for r in rows if all(r.get(k, '') == v for k, v in flt.items())]

    groups: Dict[str, List[float]] = defaultdict(list)
    for r in rows:
        g = r.get(group_key, '')
        v = _to_float(r.get(args.metric, ''))
        if not math.isnan(v):
            groups[g].append(v)

    print(f'group_by: {group_key}   metric: {args.metric}   filter: {flt or "(none)"}\n')
    headers = ['group', 'n', 'mean', 'stddev', 'min', 'max']
    widths = [max(len(h), 10) for h in headers]
    print('  '.join(f'{{:<{w}}}'.format(h) for h, w in zip(headers, widths)))
    print('  '.join('-' * w for w in widths))
    for g in sorted(groups.keys()):
        vals = groups[g]
        n = len(vals)
        mean = sum(vals) / n
        var = sum((x - mean) ** 2 for x in vals) / n if n > 0 else 0.0
        std = math.sqrt(var)
        vmin = min(vals)
        vmax = max(vals)
        cells = [g, str(n), f'{mean:.4f}', f'{std:.4f}', f'{vmin:.4f}', f'{vmax:.4f}']
        print('  '.join(f'{{:<{w}}}'.format(c) for c, w in zip(cells, widths)))

    return 0


if __name__ == '__main__':
    sys.exit(main())
