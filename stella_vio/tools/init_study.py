#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""init_study.py <tag> <seq> [--n 600] [--step 300] [--extra "--set k=v"] [--workers 8]: runs sv_run from many start frames (--skip) for N frames each
and scores every window (Sim3 ATE of that window, coverage). Measures initialisation / early-tracking robustness, not drift. Prints one row per start + mean / median / success rate
(success: coverage >= 80% and ATE < --ok m)."""
import sys, argparse, subprocess, json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import run_eval as R
ap = argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('seq'); ap.add_argument('--n', type=int, default=600); ap.add_argument('--step', type=int, default=300)
ap.add_argument('--extra', default=''); ap.add_argument('--workers', type=int, default=8); ap.add_argument('--ok', type=float, default=2.0); ap.add_argument('--cov', type=float, default=0.6)
ap.add_argument('--imu', action='store_true'); ap.add_argument('--bin', default=str(R.ROOT / 'stella_vio/sv_run')); ap.add_argument('--no-run', action='store_true')
a = ap.parse_args(); root = R.ROOT / 'runs/stella_vio' / a.tag; nf = len(list(R.SEQS[a.seq][1].glob('*.pgm'))); starts = list(range(0, nf - a.n + 1, a.step))
d, fx, cam = R.SEQS[a.seq]
imu_args = []
if a.imu and a.seq in R.IMU_CFG:
    f_, e_, to_, bg_ = R.IMU_CFG[a.seq]; imu_args = ['--imu', str(f_), '--imu-ext', str(e_), '--imu-toff', str(to_), '--imu-bg', bg_]
def one(s):
    out = root / f'{a.seq}_s{s}'; out.mkdir(parents=True, exist_ok=True)
    if not a.no_run:
        with open(out / 'log.txt', 'w') as f:
            subprocess.run([a.bin, str(R.VOCAB), str(d), str(fx), str(out), str(a.n), '--skip', str(s), '--no-snap', '--lean', '--camera', cam] + imu_args + a.extra.split(), stdout=f, stderr=subprocess.STDOUT)
    r = R.score(a.seq, out, a.n, 1.0)
    R.MAPCOL = 10; r['seg'] = R.score(a.seq, out, a.n, 1.0); R.MAPCOL = 8   # a bridged R-frame part scored as its own map (like a re-initialised map); identical to r without R-frames
    return s, r
with ThreadPoolExecutor(a.workers) as ex: res = list(ex.map(one, starts))
ates = []; ok = 0
for s, r in res:
    ate = r.get('ate'); ates.append(ate if ate is not None else np.nan)
    good = ate is not None and r['coverage'] >= a.cov and ate < a.ok; ok += good
    print(f"start {s:5d}: ATE {('%.2f' % ate) if ate is not None else '-':>6} cov {r['coverage']*100:3.0f}% lost {r.get('lost_frames','-')} maps {r.get('maps','-')} {'ok' if good else 'BAD'}")
ok_seg = sum(1 for s, r in res if r['seg'].get('ate') is not None and r['seg']['coverage'] >= a.cov and r['seg']['ate'] < a.ok)
v = np.array(ates, float); print(f"{a.tag} {a.seq}: success {ok}/{len(res)}  mean ATE {np.nanmean(v):.2f}  median {np.nanmedian(v):.2f}  (no pose counted as missing)  [segment-scored success {ok_seg}/{len(res)}]")
json.dump([dict(start=s, **{k: x for k, x in r.items() if k not in ('err30s', 'seg')}) for s, r in res], open(root / f'{a.seq}_study.json', 'w'), indent=1)
