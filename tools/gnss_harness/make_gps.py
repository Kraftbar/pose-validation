#!/usr/bin/env python3
"""Build gps0/data.csv variants (OKVIS2-X cartesian format: ns, x, y, z, hErr1, hErr2, vErr) from the receiver RTK epochs in gt_pvt.csv.
usage: make_gps.py <seq_dir> <out_dir> [--rate HZ] [--noise H,V] [--tau S] [--blackout a,b[,a2,b2]] [--seed N] [--rtk-only-hacc X]
 --noise H,V : simulated SPP/phone-grade error: AR(1) time-correlated noise (std H horizontal, V vertical, correlation time --tau s, default 30)
               plus 0.3*H white; reported errors are H, H, V.  Without --noise the receiver's RTK fixes are used as-is (sigma = max(h_acc, 0.02)).
 --blackout  : drop all fixes in the given [a,b] windows (seconds from sequence start)
 --rate HZ   : decimate to this rate (default 1)
NOTE: the RTK reference is also the evaluation GT, so the 'rtk' variant is an optimistic upper bound; the noisy variants are the honest smartphone/SPP test."""
import sys, argparse
from pathlib import Path
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument('seq'); ap.add_argument('out')
ap.add_argument('--rate', type=float, default=1.0); ap.add_argument('--noise', default=''); ap.add_argument('--tau', type=float, default=30.0)
ap.add_argument('--blackout', default=''); ap.add_argument('--seed', type=int, default=1); a = ap.parse_args()
g = np.loadtxt(Path(a.seq) / 'gt_pvt.csv', delimiter=',')
t0 = g[0, 0]
keep = [0]
for i in range(1, len(g)):
    if g[i, 0] - g[keep[-1], 0] >= 1.0 / a.rate - 1e-3: keep.append(i)
g = g[keep]
rng = np.random.default_rng(a.seed)
pos = g[:, 1:4].copy(); herr = np.maximum(g[:, 7], 0.02); verr = np.maximum(g[:, 8], 0.02)
if a.noise:
    H, V = [float(x) for x in a.noise.split(',')]
    dt = np.diff(g[:, 0], prepend=g[0, 0] - 1 / a.rate); ph = np.exp(-dt / a.tau)
    ar = np.zeros((len(g), 3))
    for i in range(len(g)):
        sig = np.array([H, H, V]) * 0.95
        ar[i] = (ar[i - 1] * ph[i] if i else 0) + np.sqrt(1 - ph[i] ** 2) * rng.standard_normal(3) * sig
    pos += ar + rng.standard_normal((len(g), 3)) * np.array([H, H, V]) * 0.3
    herr = np.full(len(g), H); verr = np.full(len(g), V)
m = np.ones(len(g), bool)
if a.blackout:
    b = [float(x) for x in a.blackout.split(',')]
    for k in range(0, len(b), 2): m &= ~((g[:, 0] - t0 >= b[k]) & (g[:, 0] - t0 <= b[k + 1]))
out = Path(a.out); (out / 'gps0').mkdir(parents=True, exist_ok=True)
with open(out / 'gps0' / 'data.csv', 'w') as f:
    f.write('timestamp, x, y, z, hErr1, hErr2, vErr\n')
    for i in np.where(m)[0]:
        f.write(f'{int(round(g[i, 0] * 1e9))},{pos[i,0]:.4f},{pos[i,1]:.4f},{pos[i,2]:.4f},{herr[i]:.3f},{herr[i]:.3f},{verr[i]:.3f}\n')
print('wrote', out / 'gps0' / 'data.csv', m.sum(), 'fixes')
