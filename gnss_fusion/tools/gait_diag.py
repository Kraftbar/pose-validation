#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Section 17 diagnosis: false-walking / missed-walking seconds of the gait detector against the GT speed, per sequence (6 phone sequences + 2 drone IMUs).
usage: gait_diag.py [--cfg KEY=VAL,...] [--dump SEQ t0 t1]      (needs make -C c; prints a table; GT = 0.5 s smoothed path speed over the 6 s window and the 6 s chord speed)"""
import sys, subprocess, argparse
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, '/home/nybo/github/pose-validation/tools/phone_diag')
import gait, common
from gf_cases import WORK, ROOT
REPO = Path('/home/nybo/github/pose-validation')
PHONE = ['outdoor1', 'outdoor2', 'indoor1', 'indoor2', 'advio15', 'advio20']
DRONE = {'drone_o1': REPO / 'external/drone/ins_o1', 'drone_m14': REPO / 'external/drone/ins_m14'}
W = 6.0


def seq_files(s):
    if s in DRONE:
        d = DRONE[s]; return d / 'mav0/imu0/data.csv', d / 'gt_enu.tum', 0.0
    return common.SEQ[s]


def load_gt(s):
    imu_f, gt_f, off = seq_files(s)
    im = np.loadtxt(imu_f, delimiter=',', comments='#', usecols=(0,)); t0 = im[0] * 1e-9
    g = np.loadtxt(gt_f); tg = g[:, 0] + off - t0; o = np.argsort(tg)
    return t0, tg[o], g[o, 1:4]


def gt_speeds(tg, p, t):
    """(path speed, chord speed) over [t-W, t]; nan if GT has gaps"""
    rate = 1.0 / np.median(np.diff(tg))
    m = (tg >= t - W) & (tg <= t)
    if m.sum() < max(5, int(0.6 * W * rate)) or np.diff(tg[m]).max() > 1.5: return np.nan, np.nan
    q = np.stack([gait.movavg(p[m, k], max(1, rate * 0.5)) for k in range(3)], 1)
    path = np.linalg.norm(np.diff(q, axis=0), axis=1).sum() * (W / (tg[m][-1] - tg[m][0]))
    chord = np.linalg.norm(q[-1] - q[0]) / (tg[m][-1] - tg[m][0])
    return path / W, chord


def run_c(s, tag, cfg=''):
    imu_f = seq_files(s)[0]; WORK.mkdir(exist_ok=True)
    ep = WORK / f'diag_{s}_{tag}.ep'
    cmd = [str(ROOT / 'c/gf_gait_run'), '--imu', str(imu_f), '--epochs', str(ep), '--epoch-dt', '3', '--window', '6']
    for kv in cfg.split(','):
        if kv: cmd += ['--cfg', kv]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    return np.loadtxt(ep, ndmin=2)


def epochs(s, cfg='', tag='base'):
    ep = run_c(s, tag, cfg); t0, tg, p = load_gt(s)
    gp = np.array([gt_speeds(tg, p, e[0] - t0) for e in ep])
    return ep, t0, gp


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--cfg', default=''); ap.add_argument('--dump', nargs=3); ap.add_argument('--seqs', default=','.join(PHONE + list(DRONE)))
    a = ap.parse_args()
    if a.dump:
        s, x0, x1 = a.dump[0], float(a.dump[1]), float(a.dump[2]); ep, t0, gp = epochs(s, a.cfg)
        print('t[s]  state  n  cad  v_det  gt_path gt_chord')
        for e, g in zip(ep, gp):
            if x0 <= e[0] - t0 <= x1: print(f'{e[0]-t0:6.1f} {int(e[3])} {int(e[1])} {e[2]:5.2f} {e[4]:5.2f}  {g[0]:5.2f} {g[1]:5.2f}')
        return
    print('| seq | epochs with GT | GT walking s (chord>0.8) | detected WALK s | false-walk s (WALK, v>0.6, GT chord<0.5) | over-speed s (WALK, v>1.35 GT path) | missed-walk s (GT chord>0.8, not WALK) | false-stationary s (STAT, GT path>0.5) |')
    print('|---|---|---|---|---|---|---|---|')
    for s in a.seqs.split(','):
        ep, t0, gp = epochs(s, a.cfg)
        ok = np.isfinite(gp[:, 0]); walk = (ep[:, 3] == 0) & (ep[:, 10] == 1)   # trusted walking: WALK and regular (reg = 2 keeps irregular windows as WALK with a wide sigma)
        stat = ep[:, 3] == 1; v = ep[:, 4]
        fw = ok & walk & (v > 0.6) & (gp[:, 1] < 0.5); ov = ok & walk & (v > 1.35 * gp[:, 0]) & (v > 0.6)
        mw = ok & ~walk & (gp[:, 1] > 0.8); fs = ok & stat & (gp[:, 0] > 0.5)
        row = [3 * fw.sum(), 3 * ov.sum(), 3 * mw.sum(), 3 * fs.sum()]
        print(f'| {s} | {ok.sum()} | {3*(ok&(gp[:,1]>0.8)).sum()} | {3*(ok&walk).sum()} | {row[0]} | {row[1]} | {row[2]} | {row[3]} |')


if __name__ == '__main__':
    main()
