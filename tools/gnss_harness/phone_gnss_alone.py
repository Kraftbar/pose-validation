#!/usr/bin/env python3
"""GNSS fixes alone vs the sequence GT (SE3 and Sim3 alignment via gnss_eval.score), optionally restricted to the time span of a run's poses.
usage: phone_gnss_alone.py <seq> [<run_dir_name> ...]   (needs rob/<seq>/gnss_enu.txt from mobile_make_gps.py)"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import phone_score as ps
from gnss_eval import score, read_traj
seq = sys.argv[1]; si = ps.rs.SEQ[seq]; gt = si['gt']()
fx = np.loadtxt(ps.ROB / seq / 'gnss_enu.txt'); nf = ps.rs.n_frames(seq)
def rep(name, f):
    s = score(f, gt, [0, 0, 0], nf, si['dur'])
    print(f"{seq} GNSS alone {name}: n={len(f)} SE3 {s.get('ate_se3', float('nan')):.2f} Sim3 {s.get('ate_sim3', float('nan')):.2f} (scored {s.get('n_scored')})")
rep('all fixes', fx)
for r in sys.argv[2:]:
    tr = read_traj(ps.ROB / 'out' / r / 'traj.txt'); tr = tr[np.argsort(tr[:, 0])]
    k = np.clip(np.searchsorted(tr[:, 0], fx[:, 0]), 1, len(tr) - 1); near = np.minimum(abs(tr[k, 0] - fx[:, 0]), abs(tr[k - 1, 0] - fx[:, 0])) < 1.0
    rep(f'on the epochs covered by {r}', fx[near])
