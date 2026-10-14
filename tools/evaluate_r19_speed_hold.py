#!/usr/bin/env python3
"""Offline scaled-speed evaluation of a saved 3 s V3 paired-hold record.

Uses tools/analyze_r19_dual_hold.mjs for the window metrics, then applies the
200 ticks/s 3 s rules scaled by |target| (raw +/-10 %, filtered +/-15 %), with
raw SD <= 60 and pair gap <= 20 ticks/s kept absolute. Declared 13 Oct 2026 in
R19_RESPONSE_ID_PLAN_20261013.md before the 150/250 runs. No hardware access.
Usage: python3 tools/evaluate_r19_speed_hold.py record.json [--pair-limit N]
--pair-limit 30 applies the 14 Oct change (gap 20 -> 30); 20 is the original.
"""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = '''import(process.argv[2]).then(m=>{const fs=require("fs");const t=JSON.parse(fs.readFileSync(process.argv[1],"utf8"));
console.log(JSON.stringify(m.analyze(t.raw??t)))})'''


def evaluate(path, pair_limit=20):
    record = json.loads(Path(path).read_text())
    raw = record.get('raw', record)
    if raw.get('nominal_seconds') not in (3, 5):
        raise ValueError('3 s or 5 s records only')
    target = abs(raw['targets'][0])
    out = json.loads(subprocess.run(
        ['node', '-e', NODE, str(path), str(ROOT / 'tools/analyze_r19_dual_hold.mjs')],
        capture_output=True, text=True, check=True).stdout)
    sides = out['tail'].get('sides') or []
    criteria = {
        'cleanup_and_guard': out['criteria']['cleanup_and_guard'],
        'full_episode': out['criteria']['full_episode'],
        'window_complete': out['criteria']['window_complete'],
        'both_RUN': out['criteria']['both_RUN'],
        'raw_means': bool(sides) and all(abs(s['raw_mean'] - target) <= .10 * target for s in sides),
        'filtered_means': bool(sides) and all(abs(s['filtered_mean'] - target) <= .15 * target for s in sides),
        'raw_sds': bool(sides) and all(s['raw_sd'] <= 60 for s in sides),
        'pair': out['tail'].get('pair_mean_difference', 1e9) <= pair_limit,
    }
    if raw['nominal_seconds'] == 5:
        # 5 s: last 2 s window plus each 1 s block within +/-15 % (170-230 at 200).
        blocks = out.get('blocks') or []
        criteria['each_1s_block'] = len(blocks) == 2 and all(
            b['complete'] and all(abs(s['raw_mean'] - target) <= .15 * target for s in b['sides'])
            for b in blocks)
    return {'target': target,
            'last1s_raw': [round(s['raw_mean'], 1) for s in sides],
            'last1s_filtered': [round(s['filtered_mean'], 1) for s in sides],
            'raw_sd': [round(s['raw_sd'], 1) for s in sides],
            'pair': round(out['tail'].get('pair_mean_difference', 0), 1),
            'peak_filtered': out['peak_filtered'], 'max_pwm': out['max_pwm'],
            'pair_limit': pair_limit,
            'criteria': criteria, 'pass': all(criteria.values())}


if __name__ == '__main__':
    limit = 20
    if len(sys.argv) == 4 and sys.argv[2] == '--pair-limit':
        limit = float(sys.argv[3])
    print(json.dumps(evaluate(sys.argv[1], limit)))
