#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""reloc_study.py <tag> [--seqs ..] [--lens 8,20,45] [--fracs 0.2,0.4,0.6,0.8] [--variants "base=;pnp_lo=--set pnp_lo=1"] [--bin ..]: relocalization stress test. Each TUM sequence is run
with a blanked stretch (flat gray frames, --blank A-B, `len` frames) at several places so that tracking is Lost and has to relocalize (PnP RANSAC in sv_relocalizer.c) when the scene returns.
Per variant (name=extra args) prints ATE, coverage (poses/frames), Lost frames, maps; summed over all (sequence, place, length) cells."""
import sys, argparse, subprocess, re
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import run_eval as R
ap = argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--seqs', default='fr1_xyz,fr1_desk,fr1_floor,fr2_xyz,fr3_long_office'); ap.add_argument('--lens', default='8,20,45')
ap.add_argument('--fracs', default='0.2,0.4,0.6,0.8'); ap.add_argument('--variants', default='base=;pnp_lo=--set pnp_lo=1'); ap.add_argument('--bin', default=str(R.ROOT / 'stella_vio/sv_run')); ap.add_argument('--workers', type=int, default=12)
a = ap.parse_args(); root = R.ROOT / 'runs/stella_vio/reloc' / a.tag
variants = [v.split('=', 1) for v in a.variants.split(';')]
jobs = [(sq, int(float(fr) * R.nframes(sq)), int(ln), vn, vx) for sq in a.seqs.split(',') for fr in a.fracs.split(',') for ln in a.lens.split(',') for vn, vx in variants]
def one(j):
    sq, s0, ln, vn, vx = j; d, fx, cam = R.SEQS[sq]; out = root / f'{sq}_{s0}_{ln}_{vn}'; out.mkdir(parents=True, exist_ok=True)
    with open(out / 'log.txt', 'w') as f:
        subprocess.run([a.bin, str(R.VOCAB), str(d), str(fx), str(out), '--no-snap', '--lean', '--camera', cam, '--blank', f'{s0}-{s0 + ln - 1}'] + vx.split(), stdout=f, stderr=subprocess.STDOUT)
    dur = (lambda ts: ts[-1] - ts[0])([float(x.split()[0]) for x in open(d / 'rgb.txt') if not x.startswith('#')])
    return j, R.score(sq, out, R.nframes(sq), dur)
with ThreadPoolExecutor(a.workers) as ex: res = list(ex.map(one, jobs))
print('| variant | cells | mean ATE | median ATE | max ATE | mean coverage | Lost frames (sum) | cells with >1 map |\n|---|---|---|---|---|---|---|---|')
for vn, _ in variants:
    rs = [r for (j, r) in res if j[3] == vn]; at = np.array([r.get('ate', np.nan) for r in rs]); cov = np.array([r['coverage'] for r in rs])
    print(f"| {vn} | {len(rs)} | {np.nanmean(at):.4f} | {np.nanmedian(at):.4f} | {np.nanmax(at):.3f} | {cov.mean()*100:.2f}% | {sum(r.get('lost_frames', 0) for r in rs)} | {sum(1 for r in rs if r.get('maps', 1) > 1)} |")
print('\nper cell (seq, blank start, len): ' + '; '.join(f"{j[0]} {j[1]} {j[2]}" for j in jobs[::len(variants)]))
for vn, _ in variants:
    print(vn + ' ATE: ' + ' '.join(f"{r.get('ate', float('nan')):.3f}" for (j, r) in res if j[3] == vn))
    print(vn + ' cov: ' + ' '.join(f"{r['coverage']*100:.0f}" for (j, r) in res if j[3] == vn))
