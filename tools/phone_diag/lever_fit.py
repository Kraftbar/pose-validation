# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H2 lever arm: the Mobile-GVIO GT is the pose of the LiDAR rig, the phone (IMU/camera) sits at an unknown offset r (rig frame).  A rotating r adds R(t) r to the
position the phone sees (and w x w x r + alpha x r to its acceleration).  Fit r from the visual trajectory:  windows of L s, Sim3 (Umeyama) of the visual camera centres to
P_gt + R_wb r, residual summed over windows; r by Nelder-Mead-free coordinate descent (3 parameters).   Then recompute the visual-vs-GT error N with the corrected GT.
usage: lever_fit.py"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C
import h1_accel as A
import vis_scale as V
import vis_noise as VN
from benchmark import umeyama_alignment


def windows_data(seq, tk, L=8.0, stride=4.0):
    d = V.load_ds(seq, tk); g = C.load(seq); gy = A.fit_gyro(g)
    tr, gt = d['tr'], d['gt']
    seg = np.r_[0, np.cumsum(np.diff(tr[:, 0]) > 0.5)]
    # GT body->world at visual stamps (absolute time)
    gg = dict(g); gg['tg'] = g['tg'] + g['t0']
    out = []
    t = tr[0, 0] + 0.5
    while t + L < tr[-1, 0]:
        sel = (tr[:, 0] >= t) & (tr[:, 0] <= t + L)
        if sel.sum() > 6 and seg[sel].min() == seg[sel].max() and np.ptp(tr[sel, 0]) > 0.8 * L:
            tt = tr[sel, 0]; ok = (tt > gg['tg'][0]) & (tt < gg['tg'][-1])
            if ok.sum() > 6:
                Pg = np.c_[[np.interp(tt[ok], gg['tg'], g['p'][:, c]) for c in range(3)]].T
                Rwb = A.slerp_R(gg, tt[ok])
                if np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum() > 2.0:
                    out.append((tr[sel, 1:4][ok], Pg, Rwb, tt[ok]))
        t += stride
    return out


def cost(W, r):
    c = 0; n = 0
    for Pv, Pg, Rwb, _ in W:
        Q = Pg + Rwb @ r
        R, tx, s = umeyama_alignment(Pv, Q, with_scale=True)
        e = s * (R @ Pv.T).T + tx - Q
        c += (e ** 2).sum(); n += len(e)
    return np.sqrt(c / n)


def fit_r(W):
    r = np.zeros(3); best = cost(W, r)
    for step in (0.1, 0.05, 0.02, 0.01, 0.005):
        improved = True
        while improved:
            improved = False
            for k in range(3):
                for sgn in (+1, -1):
                    r2 = r.copy(); r2[k] += sgn * step
                    c = cost(W, r2)
                    if c < best - 1e-7: r, best, improved = r2, c, True
    return r, best


if __name__ == '__main__':
    res = {}
    for seq, tk in (('outdoor1', 'orb3'), ('outdoor2', 'orb3'), ('indoor1', 'orb3'), ('indoor1', 'stella'), ('indoor2', 'orb3'), ('indoor2', 'stella'), ('outdoor1', 'stella')):
        W = windows_data(seq, tk)
        r, c1 = fit_r(W); c0 = cost(W, np.zeros(3))
        res[f'{seq}_{tk}'] = dict(r=r.tolist(), rms_before_m=float(c0), rms_after_m=float(c1), n_windows=len(W))
        print(f'{seq}_{tk}: n={len(W)} lever r (rig frame) = {np.round(r,3)} m ({100*np.linalg.norm(r):.0f} cm), Sim3-residual rms (8 s windows) {1e3*c0:.0f} -> {1e3*c1:.0f} mm')
    (C.OUT / 'lever_fit.json').write_text(json.dumps(res))
