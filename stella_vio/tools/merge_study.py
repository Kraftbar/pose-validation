#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""merge_study.py <tag> [--seqs fr1_xyz,fr1_desk,...] [--extra ".."] [--bin ..]: map-merge test. Each TUM sequence is run with a blanked stretch (flat gray frames, `--blank A-B`, 3 s) at
three places so that the tracker is Lost, re-initialises into a new map (reinit_sec 2) and later sees the old scene again. For every (sequence, blank position) the run is made twice,
default (old map dropped, per-map alignment) and with `--set merge=1` (old map kept, merged on place recognition, ONE alignment of everything that merged). Prints ATE, coverage, maps, merges."""
import sys, argparse, subprocess, re
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import run_eval as R
ap = argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--seqs', default='fr1_xyz,fr1_desk,fr1_floor,fr2_xyz,fr3_long_office'); ap.add_argument('--extra', default='')
ap.add_argument('--bin', default=str(R.ROOT / 'stella_vio/sv_run')); ap.add_argument('--workers', type=int, default=6); ap.add_argument('--fracs', default='0.25,0.5,0.7'); ap.add_argument('--len', type=int, default=90)
a = ap.parse_args(); root = R.ROOT / 'runs/stella_vio/merge' / a.tag
jobs = []
for sq in a.seqs.split(','):
    nf = R.nframes(sq)
    for fr in a.fracs.split(','):
        s0 = int(float(fr) * nf)
        for mode in ('drop', 'merge'): jobs.append((sq, s0, mode, nf))
def one(j):
    sq, s0, mode, nf = j; d, fx, cam = R.SEQS[sq]; out = root / f'{sq}_{s0}_{mode}'; out.mkdir(parents=True, exist_ok=True)
    ex = a.extra.split() + (['--set', 'merge=1'] if mode == 'merge' else [])
    with open(out / 'log.txt', 'w') as f:
        subprocess.run([a.bin, str(R.VOCAB), str(d), str(fx), str(out), '--no-snap', '--lean', '--camera', cam, '--blank', f'{s0}-{s0 + a.len}'] + ex, stdout=f, stderr=subprocess.STDOUT)
    dur = (lambda ts: ts[-1] - ts[0])([float(x.split()[0]) for x in open(d / 'rgb.txt') if not x.startswith('#')])
    r = R.score(sq, out, nf, dur); log = (out / 'log.txt').read_text(errors='ignore'); m = re.search(r'merges=(\d+)', log)
    r['merges'] = int(m.group(1)) if m else 0
    return j, r
with ThreadPoolExecutor(a.workers) as ex: res = list(ex.map(one, jobs))
print('| seq | blank at | mode | ATE m | main-map ATE | coverage | lost | maps | merges |\n|---|---|---|---|---|---|---|---|---|')
for (sq, s0, mode, nf), r in res:
    print(f"| {sq} | {s0} | {mode} | {r.get('ate', float('nan')):.3f} | {r.get('ate_main_map', float('nan')):.3f} | {r['coverage']*100:.0f}% | {r.get('lost_frames','-')} | {r.get('maps','-')} | {r['merges']} |")
