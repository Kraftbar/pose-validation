#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""scale_probe.py <run_dir> <seq>: per-window ratio of estimated to GT path length (arbitrary global scale removed with the median) -> scale drift / init quality."""
import sys; from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import run_eval as R
d = Path(sys.argv[1]); seq = sys.argv[2]
a = R.read_maps(d); E, P, M, T = R.match(seq, a)
w = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
rows = []
for b in range(int((T.max() - T.min()) // w) + 1):
    k = (T - T.min() >= b * w) & (T - T.min() < (b + 1) * w)
    if k.sum() < 5: continue
    le = np.linalg.norm(np.diff(E[k], axis=0), axis=1).sum(); lg = np.linalg.norm(np.diff(P[k], axis=0), axis=1).sum()
    rows.append((b * w, int(M[k][0]), le / max(lg, 1e-9), lg))
med = np.median([r[2] for r in rows])
for t, m, r, lg in rows: print(f't={t:5.0f}s map={m} gt_path={lg:6.1f}m scale_ratio={r/med:5.2f}')
