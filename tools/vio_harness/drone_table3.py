#!/usr/bin/env python3
"""runs/drone_compare/table_classical.md from the metrics.json that tools/drone_harness/score.py wrote: the 2026-10-03 classical candidates next to the reference systems."""
import json
from pathlib import Path
R = Path('/home/nybo/github/pose-validation/runs/drone_compare')
new = ['msceqf_mono', 'msceqf_mono_zvu', 'rdvio_mono', 'rdvio_mono_xrsetting', 'eqvio_mono', 'rovio_mono']
ref = ['okvis2_mono', 'okvis2_stereo', 'okvis2x_mono_nognss', 'okvis2x_mono_gnss', 'basalt_stereo', 'orb_stereo_inertial', 'orb_mono_inertial']
f = lambda v, n=2: '-' if v is None else f'{v:.{n}f}'
L = ['| seq | system | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage (all / ref window) | CPU s | exit |', '|---|---|---|---|---|---|---|---|']
for seq in ['m14', 'o1', 'of5']:
    for s in new + ref:
        p = R / seq / s / 'metrics.json'
        if not p.exists(): continue
        m = json.loads(p.read_text()); rj = json.loads((R / seq / s / 'run.json').read_text()) if (R / seq / s / 'run.json').exists() else {}
        L.append(f"| {seq} | {s}{' (NEW)' if s in new else ''} | {f(m.get('ate_se3'))} | {f(m.get('ate_sim3'))} | {f(m.get('scale'))} | {100*m.get('coverage',0):.0f}% / {100*m.get('coverage_gt_window',0):.0f}% | {f(rj.get('cpu_s'),0)} | {rj.get('exit_code','-')} |")
(R / 'table_classical.md').write_text('\n'.join(L) + '\n'); print('\n'.join(L))
