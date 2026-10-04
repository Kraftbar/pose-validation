#!/usr/bin/env python3
"""Phone scoring for the 'more systems' study (section 10 of docs/gnss_vio_benchmark_20261001.md). Reuses robust_score / phone_score (gnss_eval.score -> benchmark.umeyama_alignment, not reimplemented).
Reads external/vio2/phone_out/<seq>_<system>[_tag]/{traj.txt,run.json,log.txt}; writes runs/gnss_compare/more_systems/{table.md,table.json,<run>/{metrics.json,traj.txt}}.
Sequences: indoor1 indoor2 outdoor1 outdoor2 advio15 advio20 (same GT / clock offsets as phone_more; outdoor1 uses the section-3.2 offset)."""
import sys, json
import numpy as np
from pathlib import Path
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/gnss_harness')
import robust_score as rs
import phone_score as ps   # sets rs.SEQ for the 5 new sequences, rs.OUT, rs.runs filter
orig_o1 = None
SEQ0 = dict(ps.rs.SEQ)
# outdoor1 from the original robust_score table
import importlib
rs2 = importlib.reload(rs)  # pristine SEQ with outdoor1/complex
o1 = rs2.SEQ['outdoor1']
rs2.SEQ = {s: SEQ0[s] for s in SEQ0}; rs2.SEQ['outdoor1'] = o1
IN = Path('/home/nybo/github/pose-validation/external/vio2/phone_out')
rs2.OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare/more_systems')
rs2.LABEL.update({
    'xrslam': ('XRSLAM mono-inertial (Apache-2.0)', 'cam+IMU', 'body', 'xrslam'),
    'vinsf': ('VINS-Fusion mono-inertial (GPL, ref)', 'cam+IMU', 'body', 'vins'),
    'dmvio': ('DM-VIO mono-inertial (GPL, ref)', 'cam+IMU', 'body', 'dmvio'),
})
rs2.PATS.update({'xrslam': dict(losses=r'state \d -> 2', inits=r'state -?\d -> 1'), 'vins': dict(losses=r'(?i)big (translation|IMU|z translation)|failure detection|system reboot'), 'dmvio': dict(losses=r'(?i)lost|REINIT')})
def runs():
    R = []
    for d in sorted(IN.glob('*')):
        if not (d / 'run.json').exists(): continue
        seq, rest = d.name.split('_', 1); R.append((d.name, seq, rest, d))
    return R
_load = rs2.load
def load(path):
    a = _load(path)
    if len(a): a = a[np.isfinite(a).all(1) & (np.linalg.norm(a[:, 4:8], axis=1) > 0.5)]   # drop XRSLAM's all-zero first pose / NaNs
    return a
rs2.load = load
rs2.runs = runs
if __name__ == '__main__': rs2.main()
