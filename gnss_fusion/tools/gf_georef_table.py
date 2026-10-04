#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Section 14: effect of the slowly varying geo-referencing (gf_georef) on the cases of gf_table.py. The stream is the raw odometry of the case (gravity aligned,
no gaps filled; metric VIO: ridge scale prior 0.15, monocular: free scale), the fixes only fit one similarity (causal: all fixes up to now). Compared with the saved
gf_table values (work/table.json: robust preset, batch / causal30 / live). Scores as in gf_table (odometry sample times, causal from +30 s).
usage: gf_georef_table.py [--cases a,b] [--cfg 'key=val key=val'] [--out work/georef_table.json]"""
import sys, json, argparse, subprocess
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa
from gf_table import CASES, restrict, INIT_WAIT  # noqa
GEOREF = Path(__file__).resolve().parent.parent / 'c/gf_georef_run'


def eval_case(case, extra=()):
    raw = prepared(case); raw_t = raw[:, 0]; tmin_c = raw_t[0] + INIT_WAIT
    base = WORK / f'{case.name}_georef'
    write_odom(f'{base}.odom', raw); write_fixes(f'{base}.fix', case.gps)
    R = {}
    for mode in ('batch', 'causal'):
        cfg = ['scale_sigma=0.15' if case.metric else 'scale_sigma=100'] + list(extra)
        r = subprocess.run([str(GEOREF), '--stream', f'{base}.odom', '--fix', f'{base}.fix', '--out', f'{base}.{mode}', '--mode', mode] + cfg, capture_output=True, text=True)
        if r.returncode: raise RuntimeError(r.stderr + r.stdout)
        out = np.loadtxt(f'{base}.{mode}')
        if out.ndim < 2 or len(out) < 20: R[mode] = dict(se3=None, noalign=None, n=0); continue
        o = restrict(out[:, :8], raw_t, tmin_c if mode == 'causal' else None)
        s = score_traj(case, o)
        R[mode] = dict(se3=s.get('ate_se3'), noalign=s.get('ate_noalign'), n=s.get('n_scored'), cov=float(len(o) / max(1, int((raw_t >= (tmin_c if mode == 'causal' else -1)).sum()))))
        R[mode + '_stdout'] = r.stdout.strip()
        Path(f'{base}.{mode}').unlink(); 
    for e in ('odom', 'fix'): Path(f'{base}.{e}').unlink()
    return R


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--cases', default=''); ap.add_argument('--cfg', default=''); ap.add_argument('--out', default=str(WORK / 'georef_table.json'))
    a = ap.parse_args()
    ref = json.loads((WORK / 'table.json').read_text())
    names = a.cases.split(',') if a.cases else list(ref)
    res = {}
    f = lambda v: '-' if v is None else f'{v:.2f}'
    L = ['| sequence | GNSS alone | robust smoother batch / causal30 / live (gf_table) | georef batch / causal (coverage) | georef noalign batch / causal (geo-referenced cases) |', '|---|---|---|---|---|']
    for nm in names:
        case = CASES[nm](); R = eval_case(case, a.cfg.split()); res[nm] = R; r0 = ref[nm]
        g = lambda k, fld='se3': r0.get(k, {}).get(fld)
        L.append(f"| {nm} | {f(g('gnss'))} | {f(g('cn_batch'))} / {f(g('cn_causal'))} / {f(g('cn_live'))} | {f(R['batch']['se3'])} / {f(R['causal']['se3'])} ({R['causal'].get('cov', 0)*100:.0f}%) | {f(R['batch']['noalign'])} / {f(R['causal']['noalign'])} |")
        print(L[-1], flush=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=float))
    (WORK / 'georef_table.md').write_text('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
