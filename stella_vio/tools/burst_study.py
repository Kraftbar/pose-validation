#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""burst_study.py <tag> [--starts 200,300,400,500] [--n 1200] [--extra "--set k=v"] [--imu] [--bin ..]: complex_environment started at several frames before the two rotation bursts
(frames 698 and 836-874), run for n frames, scored over the window: coverage, Lost frames, maps and the ATE of
 (a) one alignment of everything that is one map (SV_MAPCOL=8), (b) one alignment per segment (a bridged R-frame part is its own map, like a re-initialised map; SV_MAPCOL=10).
Prints one row per start and the mean/median."""
import sys, argparse, subprocess, os, re, json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import run_eval as R
ap = argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--starts', default='200,300,400,500'); ap.add_argument('--n', type=int, default=1200)
ap.add_argument('--extra', default=''); ap.add_argument('--imu', action='store_true'); ap.add_argument('--bin', default=str(R.ROOT / 'stella_vio/sv_run')); ap.add_argument('--workers', type=int, default=8)
ap.add_argument('--no-run', action='store_true')
a = ap.parse_args(); root = R.ROOT / 'runs/stella_vio/burst' / a.tag; starts = [int(x) for x in a.starts.split(',')]
d, fx, cam = R.SEQS['complex']; imu_args = []
if a.imu:
    f_, e_, to_, bg_ = R.IMU_CFG['complex']; imu_args = ['--imu', str(f_), '--imu-ext', str(e_), '--imu-toff', str(to_), '--imu-bg', bg_]
def one(s):
    out = root / f's{s}'; out.mkdir(parents=True, exist_ok=True)
    if not a.no_run:
        with open(out / 'log.txt', 'w') as f:
            subprocess.run([a.bin, str(R.VOCAB), str(d), str(fx), str(out), str(a.n), '--skip', str(s), '--no-snap', '--lean', '--camera', cam] + imu_args + a.extra.split(), stdout=f, stderr=subprocess.STDOUT)
    res = {}
    for name, col in (('one', 8), ('seg', 10)):
        R.MAPCOL = col; res[name] = R.score('complex', out, a.n, a.n / 20.0)
    log = (out / 'log.txt').read_text(errors='ignore'); m = re.search(r'rframes=(\d+) rframes_gyro=(\d+) rbridges=(\d+) rfail=(\d+)', log)
    res['rf'] = tuple(int(x) for x in m.groups()) if m else None
    return s, res
with ThreadPoolExecutor(a.workers) as ex: rows = list(ex.map(one, starts))
O, S, C, L = [], [], [], []
for s, r in rows:
    o, g = r['one'], r['seg']
    print(f"start {s:4d}: ATE one {o.get('ate', float('nan')):6.2f} seg {g.get('ate', float('nan')):6.2f} win60 {g.get('ate_win60', float('nan')):5.2f} cov {o['coverage']*100:3.0f}% lost {o.get('lost_frames','-')} maps {o.get('maps','-')} rf/gyro/bridge/fail {r['rf']}")
    O.append(o.get('ate', np.nan)); S.append(g.get('ate', np.nan)); C.append(o['coverage']); L.append(o.get('lost_frames', 0))
print(f"{a.tag}: ATE one mean {np.nanmean(O):.2f} median {np.nanmedian(O):.2f} | seg mean {np.nanmean(S):.2f} median {np.nanmedian(S):.2f} | cov {np.mean(C)*100:.0f}% lost {np.mean(L):.1f}")
