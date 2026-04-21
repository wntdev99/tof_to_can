#!/usr/bin/env python3
"""
세션 2 개 이상을 받아 summary 및 meta 비교 리포트를 출력한다.

사용:
  python3 compare_sessions.py /tmp/tof_eval/l5cx_001 /tmp/tof_eval/gemini_001
  python3 compare_sessions.py /tmp/tof_eval/*_matte_*   # wildcard 가능 (shell 확장)
"""

import argparse
import csv
import json
import os
import sys
from typing import Dict, List


def load_summary(session_dir: str) -> Dict[str, str]:
    path = os.path.join(session_dir, 'summary.csv')
    if not os.path.exists(path):
        return {}
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            return dict(row)
    return {}


def load_meta(session_dir: str) -> dict:
    path = os.path.join(session_dir, 'meta.json')
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return json.load(f)


def print_table(headers: List[str], rows: List[List[str]]):
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    fmt = '  '.join(f'{{:<{w}}}' for w in widths)
    print(fmt.format(*headers))
    print(fmt.format(*['-' * w for w in widths]))
    for r in rows:
        print(fmt.format(*r))


def main():
    ap = argparse.ArgumentParser(description='세션 nautical diff')
    ap.add_argument('session_dirs', nargs='+', help='세션 디렉토리 경로들')
    ap.add_argument('--algo-diff', action='store_true',
                    help='알고리즘 파라미터가 서로 다르면 경고')
    args = ap.parse_args()

    summaries = [load_summary(d) for d in args.session_dirs]
    metas = [load_meta(d) for d in args.session_dirs]

    # 수치 비교 가능한 컬럼만 추출 (숫자로 변환 가능한 것)
    union_keys: List[str] = []
    seen = set()
    for s in summaries:
        for k in s.keys():
            if k not in seen:
                seen.add(k)
                union_keys.append(k)

    print('=== Summary diff ===\n')
    session_ids = [s.get('session_id', os.path.basename(d))
                   for s, d in zip(summaries, args.session_dirs)]
    headers = ['metric'] + session_ids
    rows = []
    for k in union_keys:
        if k in ('session_id',):
            continue
        row = [k] + [s.get(k, '') for s in summaries]
        rows.append(row)
    print_table(headers, rows)

    # Algo params diff
    if args.algo_diff:
        print('\n=== Algo params diff ===\n')
        algos = [m.get('algo_params', {}) for m in metas]
        keys = set()
        for a in algos:
            keys.update(_flatten(a).keys())
        for k in sorted(keys):
            vals = [_flatten(a).get(k, '—') for a in algos]
            if len(set(str(v) for v in vals)) > 1:
                print(f'  {k}:')
                for sid, v in zip(session_ids, vals):
                    print(f'    {sid}: {v}')

    return 0


def _flatten(d: dict, prefix: str = '') -> dict:
    out = {}
    for k, v in d.items():
        nk = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            out.update(_flatten(v, nk))
        else:
            out[nk] = v
    return out


if __name__ == '__main__':
    sys.exit(main())
