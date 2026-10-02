#!/usr/bin/env python3
"""Clock offset between a dataset's IMU (EuRoC csv) and a TUM ground-truth file by cross-correlating gyro-norm with the GT angular-speed norm
(frame-invariant). usage: estimate_offset.py imu0/data.csv gt.tum  -> prints dt such that t_imu = t_gt + dt  (apply GT time += dt)"""
import sys
import numpy as np
imu = np.loadtxt(sys.argv[1], delimiter=',', comments='#'); gt = np.loadtxt(sys.argv[2])
ti = imu[:, 0] * 1e-9; wi = np.linalg.norm(imu[:, 1:4], axis=1)
q = gt[:, 4:8]; q /= np.linalg.norm(q, axis=1, keepdims=True)
dots = np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)
wg = 2 * np.arccos(dots) / np.diff(gt[:, 0]); tg = 0.5 * (gt[1:, 0] + gt[:-1, 0])
def corr(dt, step=0.1):
    t = np.arange(max(ti[0], tg[0] + dt) + 1, min(ti[-1], tg[-1] + dt) - 1, step)
    if len(t) < 50: return -9
    a = np.interp(t, ti, wi); b = np.interp(t - dt, tg, wg)
    a = np.convolve(a, np.ones(5) / 5, 'same'); b = np.convolve(b, np.ones(5) / 5, 'same')
    return np.corrcoef(a, b)[0, 1]
c0 = (ti[0] - tg[0], ti[-1] - tg[-1])
cands = np.arange(min(c0) - 60, max(c0) + 60, 0.1)
cs = np.array([corr(d) for d in cands]); d0 = cands[cs.argmax()]
fine = np.arange(d0 - 0.3, d0 + 0.3, 0.01); cf = np.array([corr(d, 0.05) for d in fine])
print('offset t_imu = t_gt + %.3f s, corr %.3f (coarse best %.3f, second-best %.3f)' % (fine[cf.argmax()], cf.max(), cs.max(), np.sort(cs)[-50]))
