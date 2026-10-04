#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Section 16 table: live vs final per sequence for the stella_vio gait scale servo (live_study.py / run.py live --cfg results).
usage: study_report.py [--base base] [--new sv05] [--seqs indoor1,indoor2,...]  > runs/phone_pipeline/study16_tables.md
final = causal AUTO (gnss_fusion/c/gf_auto_run) on the odometry of the FINAL trajectory (auto_eval.py --src final), live canonical = section 15 (live_full/scores.json 'stream' auto),
base / new = study16/results.json entries of this section (same pipeline, servo off / on; several start frames = perturbations of the initialisation, mean [min..max])."""
import json, sys, argparse
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import auto_eval as AE
ROOT = Path(__file__).resolve().parent.parent
NAMES = dict(indoor1='Indoor-1', indoor2='Indoor-2', outdoor1='Outdoor-1', outdoor2='Outdoor-2', advio15='ADVIO-15', advio20='ADVIO-20')


def vals(res, tag, key='auto'):
    v = [r.get(key) for k, r in res.items() if k.split('|')[0] == tag and r.get(key) is not None]
    return np.array(v, float)


def fmt(v):
    return '-' if len(v) == 0 else f'{v.mean():.2f} [{v.min():.2f}..{v.max():.2f}]'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--base', default='base'); ap.add_argument('--new', default='sv05'); ap.add_argument('--seqs', default='indoor1,indoor2,outdoor1,outdoor2,advio15,advio20')
    a = ap.parse_args()
    print(f'| sequence | final traj. (causal) | live, section 15 (one run) | live, servo off (study, n starts) | live, servo `{a.new}` | live/final off | live/final servo | raw live map Sim3 ATE off / servo |')
    print('|---|---|---|---|---|---|---|---|')
    for s in a.seqs.split(','):
        sd = ROOT / f'runs/phone_pipeline/{s}'
        if not (sd / 'study16/results.json').exists(): continue
        res = json.loads((sd / 'study16/results.json').read_text()); sc = json.loads((sd / 'live_full/scores.json').read_text())
        fin = AE.run(s, 'final')['auto']['se3']; can = sc['stream']['auto']['se3']      # AUTO on the FINAL trajectory's odometry (section 15.4), same scorer as the live rows
        b, n = vals(res, a.base), vals(res, a.new); mb, mn = vals(res, a.base, 'map_sim3'), vals(res, a.new, 'map_sim3')
        print(f"| {NAMES[s]} | {fin:.2f} | {can:.2f} | {fmt(b)} (n={len(b)}) | {fmt(n)} (n={len(n)}) | {b.mean() / fin:.2f}x | **{n.mean() / fin:.2f}x** | {mb.mean():.2f} / {mn.mean():.2f} |")


if __name__ == '__main__':
    main()
