#!/usr/bin/env python3
"""Score Outdoor-1 blow-up study runs (runs/phone_pipeline/outdoor1/blowup/<tag>/): causal AUTO / smoother ATE SE3 (as live_study.py), raw live map Sim3 ATE,
local map scale (GT metres per map unit, Sim3 of 12 s windows) before / after the blow-up window and its worst value.
usage: o1_score.py <tag> [<tag> ...] [--skip N]"""
import sys, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import run as R
import live_study as LS
import sim3util as SU

def score(tag, seq='outdoor1', skip=0):
    d = R.OUT / seq / 'blowup' / tag
    r = LS.score_dir(seq, d, skip)
    f = d / 'live.tum'
    if f.exists() and seq in SU.OFFS:
        SU.SEQ = seq
        rows = SU.table(str(f), 12, 6, 0, 130 if seq == 'indoor2' else 390)
        sc = np.array([(a, s) for a, s, *_ in rows if not np.isnan(s)])
        r['scale_min'] = float(sc[:, 1].min()); r['scale_max'] = float(sc[:, 1].max()); r['scale_med'] = float(np.median(sc[:, 1]))
        r['scale_ratio_maxmin'] = float(sc[:, 1].max() / sc[:, 1].min())
        if seq == 'outdoor1':
            r['scale_late'] = float(np.median(sc[sc[:, 0] >= 250, 1])) if (sc[:, 0] >= 250).any() else None
            r['scale_early'] = float(np.median(sc[(sc[:, 0] >= 24) & (sc[:, 0] < 190), 1]))
            r['late_over_early'] = r['scale_late'] / r['scale_early']
            w = sc[sc[:, 0] >= 24]; r['worst_step'] = float(max(w[:, 1].max() / w[:, 1].min(), 1.0))  # max/min of the local scale after the start-up
    return r

if __name__ == '__main__':
    skip = 0; tags = []; seq = 'outdoor1'
    a = sys.argv[1:]
    while a:
        x = a.pop(0)
        if x == '--skip': skip = int(a.pop(0))
        elif x == '--seq': seq = a.pop(0)
        else: tags.append(x)
    for t in tags:
        r = score(t, seq=seq, skip=skip)
        print(t, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}))
