#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""gf_georef checks on synthetic data: (1) a stream that is an exact similarity of the fixes is recovered (batch and, at the end, causal); (2) causality: the causal output
for the first half of the stream is byte-identical when the second half (stream and fixes) is cut away; (3) a burst of outlier fixes is down-weighted (Huber);
(4) holes in the stream (> max_gap_s) produce no pairs. usage: test_georef.py   (python with numpy)"""
import subprocess, sys, tempfile
import numpy as np
from pathlib import Path
EXE = Path(__file__).resolve().parent.parent / 'c/gf_georef_run'
rng = np.random.default_rng(3)
ok = True


def check(name, cond, msg=''):
    global ok
    ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name + ' ' + msg)


def run(d, stream, fixes, mode, *cfg):
    np.savetxt(d / 's.txt', stream, fmt='%.9f'); np.savetxt(d / 'f.txt', fixes, fmt='%.9f')
    r = subprocess.run([str(EXE), '--stream', str(d / 's.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'o.txt'), '--mode', mode, *cfg], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return np.loadtxt(d / 'o.txt', ndmin=2), r.stdout


with tempfile.TemporaryDirectory() as td:
    d = Path(td)
    t = np.arange(0, 600, 0.1)
    th = np.cumsum(rng.normal(0, 0.02, t.size)); p = np.c_[np.cumsum(1.3 * np.cos(th)) * 0.1, np.cumsum(1.3 * np.sin(th)) * 0.1, np.zeros(t.size)]
    psi, s, tr = 0.7, 1.08, np.array([100., -50., 3.])
    Rm = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
    q = np.tile([0, 0, 0, 1.0], (t.size, 1))
    stream = np.c_[t, p, q, np.ones(t.size)]
    tf = np.arange(1, 600, 1.0)
    pf = np.c_[np.interp(tf, t, p[:, 0]), np.interp(tf, t, p[:, 1]), np.zeros(tf.size)]
    z = np.c_[s * pf[:, :2] @ Rm.T + tr[:2], pf[:, 2] + tr[2]]
    fixes = np.c_[tf, z, np.full(tf.size, 5.0), np.full(tf.size, 8.0)]
    o, so = run(d, stream, fixes, 'batch', 'scale_sigma=100')
    pe = s * p[:, :2] @ Rm.T + tr[:2]
    err = np.abs(o[:, 1:3] - pe[: len(o)]).max()
    check('batch recovers the similarity', err < 1e-3, f'max err {err:.2e} m')
    o, so = run(d, stream, fixes, 'causal', 'scale_sigma=100')
    err = np.abs(o[-1, 1:3] - pe[len(stream) - 1 - (len(stream) - len(o)) + (len(o) - 1) - (len(o) - 1) + (len(o) - 1)][:2]).max() if False else np.abs(o[-1, 1:3] - (s * p[np.searchsorted(t, o[-1, 0]), :2] @ Rm.T + tr[:2])).max()
    check('causal recovers the similarity at the end', err < 1e-3, f'max err {err:.2e} m')
    # causality
    half = t.size // 2
    full, _ = run(d, stream, fixes, 'causal')
    cut, _ = run(d, stream[:half], fixes[fixes[:, 0] <= stream[half - 1, 0]], 'causal')
    n = len(cut)
    check('causal output of the first half does not depend on the second half', n > 100 and np.array_equal(full[:n], cut), f'{n} samples compared')
    # outlier burst: 30 fixes with +60 m offset
    f2 = fixes.copy(); f2[300:330, 1:3] += 60.0
    o_h, _ = run(d, stream, f2, 'batch', 'scale_sigma=100', 'huber_k=2.5'); o_n, _ = run(d, stream, f2, 'batch', 'scale_sigma=100', 'huber_k=0')
    e_h = np.abs(o_h[:, 1:3] - pe[: len(o_h)]).max(); e_n = np.abs(o_n[:, 1:3] - pe[: len(o_n)]).max()
    check('Huber re-weighting reduces the effect of a 30 s burst of outliers', e_h < 0.5 * e_n, f'max err {e_h:.2f} m vs {e_n:.2f} m without')
    # hole in the stream: fixes inside are not paired
    keep = (t < 250) | (t > 280)
    o_g, so = run(d, stream[keep], fixes, 'batch', 'scale_sigma=100')
    npairs = int(so.split('pairs')[-1])
    check('fixes inside a stream hole are not paired', npairs <= len(fixes) - 25, f'{npairs} pairs of {len(fixes)} fixes')
print('ALL PASS' if ok else 'FAILED'); sys.exit(0 if ok else 1)
