#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Section 15: the automatic smoother <-> georef switch (gf_auto_run) on the cases of gf_table.py. One run of gf_auto_run per case writes the smoother live stream
(= gf_run causal live), the georef live stream (stream = raw odometry, scale_sigma 0.15 metric / 100 monocular) and the switched output, plus the signal log.
Scores as gf_table (odometry sample times, causal from +30 s, status bit 1). Output: work/auto_table.json (+ per-case signal logs work/auto/<case>.sig).
usage: gf_auto_table.py [--cases a,b] [--cfg 'key=val ...'] [--out work/auto_table.json] [--keep]"""
import sys, json, argparse, subprocess
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa
from gf_table import CASES, restrict, INIT_WAIT  # noqa
AUTO = Path(__file__).resolve().parent.parent / 'c/gf_auto_run'
NAMES = ['complex_rtk', 'complex_sim', 'complex_rtk_blk', 'complex_sim_blk', 'o1_okvis', 'o2_okvis', 'a15_okvis', 'a20_okvis', 'o1_orb3mono', 'o2_orb3mono', 'a15_orb3mono', 'a20_orb3mono',
         'o1_stella', 'o2_stella', 'm14_okvis', 'o1d_okvis', 'o1_xrslam', 'o2_xrslam', 'a20_xrslam']


def run_case(case, extra=(), keep=False):
    raw = prepared(case); raw_t = raw[:, 0]; tmin = raw_t[0] + INIT_WAIT
    d = WORK / 'auto'; d.mkdir(exist_ok=True)
    b = d / case.name
    write_odom(f'{b}.odom', raw); write_fixes(f'{b}.fix', case.gps)
    cfg = ['preset=robust', 'rsa=%.6f,%.6f,%.6f' % tuple(case.rsa), 'g.scale_sigma=%s' % ('0.15' if case.metric else '100'), 'stream=0']
    if not case.metric: cfg.append('metric=0')
    r = subprocess.run([str(AUTO), '--odom', f'{b}.odom', '--fix', f'{b}.fix', '--out', f'{b}.auto', '--out-sm', f'{b}.sm', '--out-geo', f'{b}.geo', '--sig', f'{b}.sig', '--timing'] + cfg + list(extra),
                       capture_output=True, text=True)
    if r.returncode: raise RuntimeError(r.stderr + r.stdout)
    R = {}
    for k in ('sm', 'geo', 'auto'):
        p = Path(f'{b}.{k}')
        a = np.loadtxt(p, ndmin=2) if p.stat().st_size else np.zeros((0, 10))
        if len(a) and k != 'geo': a = a[(a[:, 8].astype(int) & 1) == 1]
        o = restrict(a[:, :8], raw_t, tmin) if len(a) else a
        s = score_traj(case, o) if len(o) >= 20 else {}
        R[k] = dict(se3=s.get('ate_se3'), noalign=s.get('ate_noalign'), n=s.get('n_scored'), cov=float(len(o) / max(1, int((raw_t >= tmin).sum()))))
    R['stdout'] = r.stdout.strip()
    for e in ('odom', 'fix', 'sm', 'geo', 'auto'):
        if not keep: Path(f'{b}.{e}').unlink(missing_ok=True)
    return R


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--cases', default=''); ap.add_argument('--cfg', default=''); ap.add_argument('--out', default=str(WORK / 'auto_table.json')); ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()
    names = a.cases.split(',') if a.cases else NAMES
    ref = json.loads((WORK / 'table.json').read_text())
    res = {}
    f = lambda v: '-' if v is None else f'{v:.2f}'
    L = ['| case | GNSS alone | smoother live | georef | auto (switch) | auto vs best of the two | auto vs GNSS alone |', '|---|---|---|---|---|---|---|']
    for nm in names:
        case = CASES[nm](); R = run_case(case, a.cfg.split(), a.keep); res[nm] = R
        g = ref[nm]['gnss']['se3']; sm, ge, au = R['sm']['se3'], R['geo']['se3'], R['auto']['se3']
        best = min(x for x in (sm, ge) if x is not None)
        L.append(f"| {nm} | {f(g)} | {f(sm)} | {f(ge)} ({R['geo']['cov']*100:.0f}%) | {f(au)} | {au / best - 1:+.0%} | {au / g - 1:+.0%} |")
        print(L[-1], flush=True)
    Path(a.out).write_text(json.dumps(res, indent=1, default=float))
    (WORK / 'auto_table.md').write_text('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
