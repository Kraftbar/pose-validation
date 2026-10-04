# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Collect runs/phone_diag/init_rerun/*/summary.json into one table of the median scale ratio (estimate / Sim3 truth) per dataset, window length and variant."""
import json, sys
from pathlib import Path
import numpy as np
D = Path(__file__).resolve().parents[2] / 'runs/phone_diag/init_rerun'
var = [v for v in ['base', 'floors0.1', 'noacc', 'smooth0.3', 'smooth0.6', 'smooth1.0', 'match0.3', 'match0.6', 'match1.0', 'lens'] if (D / v / 'summary.json').exists()]
S = {v: json.loads((D / v / 'summary.json').read_text()) for v in var}
Ls = [int(a) for a in sys.argv[1:]] or [4, 8, 12, 16]
out = []
for L in Ls:
    out.append(f'\nwindow {L} s: median(scale estimate / Sim3 truth), within-20%% share in brackets')
    out.append('| dataset | ' + ' | '.join(var) + ' |'); out.append('|---|' + '---|' * len(var))
    for ds in S['base']:
        cells = []
        for v in var:
            r = [x for x in S[v].get(ds, []) if x['len'] == L]
            cells.append(f"{r[0]['med_ratio']:.2f} ({100*r[0]['within20']:.0f}%)" if r else '-')
        out.append(f'| {ds} | ' + ' | '.join(cells) + ' |')
print('\n'.join(out))
