#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""C gf_gait vs the python prototype tools/gait.py on the six phone IMU streams: step times, epoch cadence / speed / state, online calibration k.
usage: check_gait.py [seq ...]   (needs make -C c)"""
import sys, subprocess
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import gait
from gf_cases import WORK, ROOT, ROB, lf

SEQS = ['outdoor1', 'outdoor2', 'indoor1', 'indoor2', 'advio15', 'advio20']


def fixes_for(seq):
    g = lf.read_gps(ROB / seq / 'gps0' / 'data.csv')
    return [(r[0], r[1], r[2], r[4]) for r in g]


def run_c(seq, tag, model=None, online=False, fixes=None, window=0.0, edt=3.0):
    import common
    WORK.mkdir(exist_ok=True)
    ep, st = WORK / f'gait_{seq}_{tag}.ep', WORK / f'gait_{seq}_{tag}.steps'
    cmd = [str(ROOT / 'c' / 'gf_gait_run'), '--imu', str(common.SEQ[seq][0]), '--epochs', str(ep), '--steps', str(st), '--epoch-dt', str(edt)]
    if window: cmd += ['--window', str(window)]
    if model: cmd += ['--model', str(model)]
    if online:
        fx = WORK / f'gait_{seq}.fix'
        np.savetxt(fx, np.array(fixes), fmt='%.9f %.4f %.4f %.3f'); cmd += ['--online', '--fix', str(fx)]
    subprocess.run(cmd, check=True)
    return np.loadtxt(ep, ndmin=2), np.loadtxt(st, ndmin=1)


def run_py(seq, model=None, online=False, fixes=None, window=None, edt=3.0):
    t, a, w = gait.load_imu(seq)
    cfg = gait.Config(); cfg.online = 1 if online else 0
    g, ep = gait.run_stream(t, a, w, cfg=cfg, model=model, epoch_dt=edt, fixes=fixes, window=window)
    return g, ep


def main():
    seqs = sys.argv[1:] or SEQS
    worst = 0.0
    for s in seqs:
        for tag, kw in (('gen', {}), ('user', dict(model=0.35)), ('onl', dict(online=True))):
            kw = dict(kw)
            if kw.get('online'): kw['fixes'] = fixes_for(s)
            cep, cst = run_c(s, tag, **kw)
            g, pep = run_py(s, **kw)
            # python keeps only the last 64 steps: compare the step count via the epochs, and step times via full re-run
            ps = np.array([e['n_steps'] for e in pep]); cs = cep[:, 1].astype(int)
            n = min(len(pep), len(cep))
            d_state = int((np.array([e['state'] for e in pep])[:n] != cep[:n, 3].astype(int)).sum())
            d_n = int(np.abs(ps[:n] - cs[:n]).sum())
            d_sp = float(np.max(np.abs(np.array([e['speed'] for e in pep])[:n] - cep[:n, 4])))
            d_cad = float(np.max(np.abs(np.array([e['cadence'] for e in pep])[:n] - cep[:n, 2])))
            d_k = float(np.max(np.abs(np.array([e['k'] for e in pep])[:n] - cep[:n, 7])))
            d_hd = float(np.max(np.abs(np.array([e['heading'] for e in pep])[:n] - cep[:n, 9])))
            d_od = float(np.max(np.abs(np.array([e['odo'] for e in pep])[:n] - cep[:n, 8])))
            worst = max(worst, d_sp, d_cad, d_k, d_hd, d_od)
            print(f'{s:9s} {tag:4s} epochs {len(pep)}/{len(cep)}  state diffs {d_state}  step-count diffs {d_n}  max |d| speed {d_sp:.2e} cadence {d_cad:.2e} k {d_k:.2e} heading {d_hd:.2e} odometer {d_od:.2e}  (k_end {cep[-1,7]:.3f})')
    print('max abs difference over everything: %.3e' % worst)
    return 0 if worst < 1e-6 else 1


if __name__ == '__main__':
    sys.exit(main())
