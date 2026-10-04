#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (own code)
"""Exploratory fusion variants on the existing sv_run outputs (extra gf_run keys). usage: explore.py <seq> <variant> <mode> <tag> key=value ..."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import run as R, score as S
seq, v, mode, tag = sys.argv[1:5]; extra = sys.argv[5:]
import os
R.run_fuse(seq, v, mode, extra=extra, tag=tag, fix_sigma_k=float(os.environ.get('FIXK', '1')))
import numpy as np
c = R.cfg_of(seq); base = R.OUT / seq; ft = S.frame_times(seq)
some = next(iter(sorted(base.glob('sv_*/trajectory_maps.tum'))))
from gf_cases import phone_case
case = phone_case(seq, str(some.resolve()), cam_only=False, label=seq)
r = S.fused(case, base / f'fuse_{v}_{tag}', ft, R.rp(c['fixes']) if c['fixes'] else None)
f = lambda x: '-' if x is None else f'{x:.2f}'
b, ca = r['batch'], r['causal']
print(f"{seq:9s} {v}|{mode}|{tag} {' '.join(extra)}: batch se3 {f(b.get('se3'))} sim3 {f(b.get('sim3'))} scale {f(b.get('scale'))} | causal se3 {f(ca.get('se3'))} sim3 {f(ca.get('sim3'))} cov {ca.get('coverage', 0)*100:.0f}%")
