#!/usr/bin/env python3
"""Compact phone summary (markdown) of the 2026-10-03 classical-candidate runs + the reference rows (XRSLAM, OKVIS2-X, ORB-SLAM3-MI quoted from docs/gnss_vio_benchmark_20261001.md sections 9-10)."""
import json
from pathlib import Path
R = Path('/home/nybo/github/pose-validation')
rows = json.loads((R / 'runs/gnss_compare/more_systems2/table.json').read_text())
cpu = {}
for l in (R / 'runs/gnss_compare/more_systems2/cpu.md').read_text().splitlines()[2:]:
    c = [x.strip() for x in l.strip('|').split('|')]; cpu[c[0]] = c
by = {r['id']: r for r in rows}
seqs = ['indoor1', 'indoor2', 'advio15', 'outdoor1', 'outdoor2', 'advio20']
variants = [('msceqf', 'MSCEqF'), ('msceqf_infl', 'MSCEqF infl'), ('rdvio', 'RD-VIO stock setting.yaml'), ('rdvio_xrsetting', 'RD-VIO XRSLAM settings'), ('rdvio_infl', 'RD-VIO infl'), ('eqvio', 'EqVIO (GPL ref)'), ('rovio', 'ROVIO')]
f = lambda v, n=2: '-' if v is None else (f'{v:.{n}f}' if abs(v) < 1e4 else f'{v:.1e}')
L = ['| seq | system | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage | CPU s |', '|---|---|---|---|---|---|---|']
for s in seqs:
    for v, name in variants:
        r = by.get(f'{s}_{v}')
        if r: L.append(f"| {s} | {name} | {f(r['ate_se3'])} | {f(r['ate_sim3'])} | {f(r['scale'], 3)} | {r['coverage']*100:.0f}% | {cpu.get(f'{s}_{v}', ['', '-'])[1]} |")
print('\n'.join(L))
