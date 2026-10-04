#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""gf_auto checks on synthetic data: (1) policy 0 output == gf_run causal live, policy 1 output == gf_georef_run causal (to print precision);
(2) causality: the AUTO output of the first half is byte-identical when the second half of odometry and fixes is cut away;
(3) auto picks the georef for poor correlated fixes + good stream, and the smoother for accurate fixes (sigma 0.05) with a drifting stream;
(4) a stream with an unrelated frame jump (flagged NEW_FRAME) at t=300 s never makes the output blow up. usage: test_auto.py"""
import subprocess, sys, tempfile
import numpy as np
from pathlib import Path
C = Path(__file__).resolve().parent.parent / 'c'
rng = np.random.default_rng(5)
ok = True


def check(name, cond, msg=''):
    global ok
    ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name + ' ' + msg)


def run(exe, args):
    r = subprocess.run([str(C / exe)] + args, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout


def scene(T=500, fix_sigma=5.0, corr=40.0, drift=0.0, rep=None):
    t = np.arange(0, T, 0.1)
    th = np.cumsum(rng.normal(0, 0.02, t.size))
    p = np.c_[np.cumsum(1.3 * np.cos(th)) * 0.1, np.cumsum(1.3 * np.sin(th)) * 0.1, np.zeros(t.size)]
    od = p * (1 + drift * t / T)[:, None]
    odom = np.c_[t, od, np.tile([0, 0, 0, 1.0], (t.size, 1)), np.zeros(t.size)]
    tf = np.arange(1, T, 1.0)
    # correlated (AR1, tau 40 s) fix error of std fix_sigma
    e = np.zeros((tf.size, 2)); a = np.exp(-1 / corr)
    for i in range(1, tf.size): e[i] = a * e[i - 1] + np.sqrt(1 - a * a) * rng.normal(0, fix_sigma, 2)
    z = np.c_[np.interp(tf, t, p[:, 0]) + e[:, 0], np.interp(tf, t, p[:, 1]) + e[:, 1], np.zeros(tf.size)]
    fixes = np.c_[tf, z, np.full(tf.size, rep or fix_sigma), np.full(tf.size, 2 * (rep or fix_sigma))]
    return odom, fixes, p


def ate(o, p, t):
    k = np.clip(np.searchsorted(t, o[:, 0]), 0, len(t) - 1)
    return np.sqrt(((o[:, 1:3] - p[k, :2]) ** 2).sum(1).mean())


with tempfile.TemporaryDirectory() as td:
    d = Path(td)
    odom, fixes, p = scene(rep=14.0)   # like the Mobile-GVIO phone: true error 5 m/axis (7 m rms), reported sigma a pessimistic constant
    np.savetxt(d / 'o.txt', odom, fmt='%.9f'); np.savetxt(d / 'f.txt', fixes, fmt='%.9f')
    cfg = ['preset=robust', 'g.scale_sigma=0.15']
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a0.txt'), '--out-geo', str(d / 'g.txt'), '--out-sm', str(d / 's.txt')] + cfg + ['policy=0'])
    run('gf_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'r.out'), '--out-live', str(d / 'r.live'), '--mode', 'causal', 'preset=robust'])
    a0 = np.loadtxt(d / 'a0.txt'); rl = np.loadtxt(d / 'r.live')
    check('policy 0 == gf_run causal live', len(a0) == len(rl) and np.abs(a0[:, 1:4] - rl[:, 1:4]).max() < 2e-6, f'{len(a0)} rows')
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a1.txt')] + cfg + ['policy=1'])
    np.savetxt(d / 'os.txt', odom[:, :8], fmt='%.9f')
    run('gf_georef_run', ['--stream', str(d / 'os.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'g1.txt'), '--mode', 'causal', 'scale_sigma=0.15'])
    a1 = np.loadtxt(d / 'a1.txt'); g1 = np.loadtxt(d / 'g1.txt')
    check('policy 1 == gf_georef_run causal', len(a1) == len(g1) and np.abs(a1[:, 1:4] - g1[:, 1:4]).max() < 2e-6, f'{len(a1)} rows')
    # causality
    n = len(odom) // 2
    np.savetxt(d / 'o2.txt', odom[:n], fmt='%.9f'); np.savetxt(d / 'f2.txt', fixes[fixes[:, 0] <= odom[n - 1, 0]], fmt='%.9f')
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a2.txt')] + cfg)
    run('gf_auto_run', ['--odom', str(d / 'o2.txt'), '--fix', str(d / 'f2.txt'), '--out', str(d / 'a3.txt')] + cfg)
    A = (d / 'a2.txt').read_text().splitlines(); B = (d / 'a3.txt').read_text().splitlines()
    check('causality: first half identical', A[:len(B) - 1] == B[:len(B) - 1], f'{len(B)} lines compared')
    # (3) poor correlated fixes, good stream -> georef weight high at the end and better than the smoother
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a4.txt')] + cfg)
    a4 = np.loadtxt(d / 'a4.txt'); 
    check('poor fixes + good stream: georef used', a4[-1, 9] > 0.9, f'w_geo end {a4[-1, 9]:.2f}')
    odom, fixes, p = scene(fix_sigma=0.05, drift=0.15)
    np.savetxt(d / 'o.txt', odom, fmt='%.9f'); np.savetxt(d / 'f.txt', fixes, fmt='%.9f')
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a5.txt')] + cfg)
    a5 = np.loadtxt(d / 'a5.txt')
    check('accurate fixes: smoother used', a5[:, 9].max() < 0.01, f'max w_geo {a5[:, 9].max():.2f}')
    # (4) frame jump
    odom, fixes, p = scene(fix_sigma=5.0)
    k = odom[:, 0] >= 300; odom[k, 1:3] += 400.0; odom[np.argmax(k), 8] = 1
    np.savetxt(d / 'o.txt', odom, fmt='%.9f'); np.savetxt(d / 'f.txt', fixes, fmt='%.9f')
    run('gf_auto_run', ['--odom', str(d / 'o.txt'), '--fix', str(d / 'f.txt'), '--out', str(d / 'a6.txt')] + cfg)
    a6 = np.loadtxt(d / 'a6.txt'); a6 = a6[(a6[:, 8].astype(int) & 1) == 1]
    err = np.abs(a6[a6[:, 0] > 330][:, 1:3] - p[np.clip(np.searchsorted(odom[:, 0], a6[a6[:, 0] > 330][:, 0]), 0, len(p) - 1), :2]).max()
    check('NEW_FRAME jump: output bounded', err < 100, f'max error after the jump {err:.1f} m')
sys.exit(0 if ok else 1)
