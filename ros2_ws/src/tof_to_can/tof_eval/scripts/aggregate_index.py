#!/usr/bin/env python3
"""
sessions_index.csv 를 filter 해서 요약 테이블로 출력한다.

사용:
  python3 aggregate_index.py /tmp/tof_eval
  python3 aggregate_index.py /tmp/tof_eval --filter floor_material=matte_vinyl
  python3 aggregate_index.py /tmp/tof_eval --filter ambient_light=direct_sunlight \\
                                            --filter scene_sensor=VL53L5CX \\
                                            --columns obs_cube_a_detection_rate,ground_inlier_ratio_mean
"""

import argparse
import csv
import os
import sys
from typing import Dict, List


def parse_filter(filters: List[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for f in filters or []:
        if '=' not in f:
            print(f"[warn] ignoring filter '{f}' — expected key=value", file=sys.stderr)
            continue
        k, v = f.split('=', 1)
        out[k.strip()] = v.strip()
    return out


def main():
    ap = argparse.ArgumentParser(description='sessions_index.csv 집계/filter')
    ap.add_argument('root', help='tof_eval 출력 루트 (sessions_index.csv 포함)')
    ap.add_argument('--filter', action='append', default=[],
                    help='key=value 필터 (여러 번 가능, AND 결합)')
    ap.add_argument('--columns', default='',
                    help='출력할 컬럼 (comma 분리). 지정 안 하면 전체')
    args = ap.parse_args()

    index_path = os.path.join(args.root, 'sessions_index.csv')
    if not os.path.exists(index_path):
        print(f"[err] not found: {index_path}", file=sys.stderr)
        return 1

    with open(index_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        rows = [dict(r) for r in reader]
        all_headers = reader.fieldnames or []

    flt = parse_filter(args.filter)
    for k in flt:
        # scene_* prefix 없이 쓴 경우 자동 매핑 시도
        if k not in all_headers:
            alt = f'scene_{k}'
            if alt in all_headers:
                flt[alt] = flt.pop(k)

    def keep(r):
        return all(r.get(k, '') == v for k, v in flt.items())

    rows = [r for r in rows if keep(r)]

    if args.columns:
        cols = [c.strip() for c in args.columns.split(',') if c.strip()]
        # session_id 는 항상 맨 앞에
        if 'session_id' not in cols:
            cols = ['session_id'] + cols
    else:
        cols = all_headers

    w = csv.DictWriter(sys.stdout, fieldnames=cols, extrasaction='ignore')
    w.writeheader()
    for r in rows:
        w.writerow({k: r.get(k, '') for k in cols})

    print(f"\n[info] matched {len(rows)} session(s)", file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
