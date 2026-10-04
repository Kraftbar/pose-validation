# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Window-local accelerometer gain as the VI initialiser sees it, but with GT orientation and GT positions (no visual front end).
Per window [t0,t0+L]:   P_gt(t) = P0 + v0*tau + 0.5*g*tau^2 + k * I(t),   I(t) = double integral of R_wI(t) * f(t)   (IMU, GT orientation, zero start velocity)
Unknowns P0, v0, g (3 each) and the scalar k (or one k per IMU axis).  k = 1 -> the accelerometer contains exactly the GT motion; k ~ 0.5 -> it carries half.
Also returns the 'excitation' (rms of the detrended GT acceleration in the window) and the window path length.
usage: h1_window.py [seq ...] [--L 4] [--noacc]  -> runs/phone_diag/h1win_<seq>.json"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C
import h1_accel as A


def prep(d, gy, fscale=1.0, tshift=0.0, delay=0.0, lp=None):
    """IMU-rate world-frame acceleration-like signal R_wI f and its GT-time cumulative double integral sampler. tshift = extra GT time shift."""
    ti = d['ti']
    Rwb = A.slerp_R(d, ti - gy['tau'] - tshift)
    RwI = Rwb @ gy['R_bI']
    f = d['f'] * fscale
    if lp:                     # moving-average low-pass (s) applied to the accelerometer, causal-free
        n = max(1, int(round(lp / np.median(np.diff(ti)))))
        f = np.c_[[np.convolve(f[:, c], np.ones(n) / n, 'same') for c in range(3)]].T
    return ti, RwI, f


def windows(name, L=4.0, stride=2.0, axis_gain=False, fscale=1.0, tshift=0.0, lp=None, gy=None, d=None, imu_delay=0.0):
    d = d or C.load(name)
    gy = gy or A.fit_gyro(d)
    ti, RwI, f = prep(d, gy, fscale, tshift, lp=lp)
    if imu_delay:                                   # shift accelerometer in time (accel stamps delayed by imu_delay s)
        f = np.c_[[np.interp(ti - imu_delay, ti, f[:, c]) for c in range(3)]].T
    a_w = np.einsum('nij,nj->ni', RwI, f)           # includes gravity as -g
    a_wj = RwI * f[:, None, :]                      # (n,3 world,3 axes)  per-axis contributions
    dti = float(np.median(np.diff(ti)))
    tg, p = d['tg'], d['p']
    cont = C.gt_continuous_mask(tg, 0.35)
    # cumulative double integrals (trapezoid)
    def dint(x):
        v = np.vstack([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (x[1:] + x[:-1]) * dti, axis=0)])
        s = np.vstack([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (v[1:] + v[:-1]) * dti, axis=0)])
        return s
    S = dint(a_w); Sj = dint(a_wj)
    rows = []
    t_lo, t_hi = C.gt_window(d, 0.5)
    t0 = t_lo
    while t0 + L <= t_hi:
        sel = (tg >= t0) & (tg <= t0 + L)
        if sel.sum() < 0.8 * L / np.median(np.diff(tg)) or not cont[sel].all():
            t0 += stride; continue
        t = tg[sel]; tau = t - t[0]
        j0 = int(round((t[0] - ti[0]) / dti))
        i_t = np.c_[[np.interp(t, ti, S[:, c]) for c in range(3)]].T
        i_t = i_t - np.interp(t[0], ti, S[:, 0])[None, None][0] * 0 - np.array([np.interp(t[0], ti, S[:, c]) for c in range(3)])[None, :]
        # remove the start velocity part of the integral: I(t) starting at t[0] with zero velocity = S(t)-S(t0)-V(t0)*tau; V(t0) is a free (v0) term, absorbed
        P = p[sel]
        n = len(t)
        B = np.zeros((3 * n, 7)); y = P.reshape(-1)
        for c in range(3):
            B[c::3, c] = 1; B[c::3, 3 + 0] = 0
        # unknowns: P0(3), v0(3), g(3) -> 9, + k
        cols = np.zeros((3 * n, 9 + (3 if axis_gain else 1)))
        for c in range(3):
            cols[c::3, c] = 1.0; cols[c::3, 3 + c] = tau; cols[c::3, 6 + c] = 0.5 * tau ** 2
        if axis_gain:
            Ij = np.stack([np.c_[[np.interp(t, ti, Sj[:, c, j]) for c in range(3)]].T for j in range(3)], axis=2)   # (n,3,3)
            Ij = Ij - Ij[0:1]
            for j in range(3):
                cols[:, 9 + j] = Ij[:, :, j].reshape(-1)
        else:
            cols[:, 9] = (i_t - i_t[0]).reshape(-1)
        th, *_ = np.linalg.lstsq(cols, y, rcond=None)
        res = y - cols @ th
        # excitation: GT acceleration rms after quadratic detrend of the window (second difference based)
        dd = (P[2:] - 2 * P[1:-1] + P[:-2]) / np.median(np.diff(tg)) ** 2
        # path length
        pl = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
        # displacement explained by dynamic acceleration only: rms of k*I_dyn after quad detrend
        rows.append(dict(t0=float(t0), L=L, k=th[9:].tolist(), g=th[6:9].tolist(), gnorm=float(np.linalg.norm(th[6:9])), res_rms=float(np.sqrt((res ** 2).mean())),
                         path=pl, acc_rms=float(np.sqrt((dd ** 2).sum(1).mean())), v0=float(np.linalg.norm(th[3:6]))))
        t0 += stride
    return rows


def summarize(rows, key=0):
    if not rows: return {}
    k = np.array([r['k'][key] if len(r['k']) > key else np.nan for r in rows])
    return dict(n=len(rows), k_med=float(np.median(k)), k_q25=float(np.percentile(k, 25)), k_q75=float(np.percentile(k, 75)), g_med=float(np.median([r['gnorm'] for r in rows])),
                path_med=float(np.median([r['path'] for r in rows])), res_med=float(np.median([r['res_rms'] for r in rows])), acc_med=float(np.median([r['acc_rms'] for r in rows])))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    names = args or list(C.SEQ)
    for n in names:
        d = C.load(n); gy = A.fit_gyro(d)
        out = {}
        for L in (2.0, 4.0, 8.0):
            rows = windows(n, L, 1.0 if L < 8 else 2.0, d=d, gy=gy)
            s = summarize(rows); out[f'L{L:g}'] = dict(summary=s, rows=rows)
            print(f"{n:11s} L={L:3.0f}s n={s.get('n')} k med {s.get('k_med'):.3f} IQR [{s.get('k_q25'):.2f},{s.get('k_q75'):.2f}] |g|fit {s.get('g_med'):.2f} path {s.get('path_med'):.1f} m acc_rms {s.get('acc_med'):.2f} res {s.get('res_med'):.3f}")
        rows = windows(n, 4.0, 1.0, axis_gain=True, d=d, gy=gy)
        K = np.array([r['k'] for r in rows]); out['axis_L4'] = dict(k_med=np.median(K, axis=0).tolist())
        print(f"{'':11s} per-IMU-axis k (L=4): {np.round(np.median(K, axis=0),3)}")
        (C.OUT / f'h1win_{n}.json').write_text(json.dumps(out))
