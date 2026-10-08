#!/usr/bin/env python3
"""Time-binned SE3-aligned error (one alignment over the whole scored run) of several trajectories of a sequence, for the live-vs-final analysis.
usage: binned.py <seq> <bin_s> label=path [label=path ...]   (path = pose file with t x y z qx qy qz qw [.. flags in column 9: aligned if &1])
Uses gf_cases.score_traj (gnss_eval.score), the same alignment as the study tables."""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent / 'gnss_fusion/tools'))
import run as R, score as S
from gf_cases import phone_case, score_traj
seq, bw = sys.argv[1], float(sys.argv[2])
case = phone_case(seq, str((R.OUT / seq / 'sv_full' / 'trajectory_maps.tum').resolve()), cam_only=False, label=seq)
ft = S.frame_times(seq); c = R.cfg_of(seq); iw = 30.0 if c['fixes'] else R.INIT_WAIT_NOFIX
cols = {}
for a in sys.argv[3:]:
    lab, p = a.split('=')
    tr = np.loadtxt(p, ndmin=2); tr = tr[np.argsort(tr[:, 0], kind='stable')]
    if tr.shape[1] > 8 and 'live.tum' not in p: tr = tr[(tr[:, 8].astype(int) & 1) == 1]
    f = ft[ft >= ft[0] + iw]; pp, ok = S.resample(tr, f)
    pose = np.c_[f[ok], pp, np.tile([0, 0, 0, 1.0], (ok.sum(), 1))]
    s = score_traj(case, pose)
    cols[lab] = (s['_t'], s['_err_se3'], s['ate_se3'])
t0 = ft[0]
print('bin(s from start)   ' + '  '.join(f'{k:>12}' for k in cols))
print('ATE SE3 (whole)     ' + '  '.join(f'{v[2]:12.3f}' for v in cols.values()))
for b in np.arange(iw, ft[-1] - t0, bw):
    row = []
    for k, (t, e, _) in cols.items():
        m = (t - t0 >= b) & (t - t0 < b + bw)
        row.append(f'{np.sqrt((e[m] ** 2).mean()):12.3f}' if m.sum() > 3 else f'{"-":>12}')
    print(f'{b:5.0f}-{b + bw:<5.0f}         ' + '  '.join(row))
