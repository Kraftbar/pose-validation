# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Errors-in-variables test: after the per-window quadratic detrend that the closed-form scale estimator applies implicitly (v0, g free), how big is the
VISUAL position error compared with the motion signal that remains?   For each window (L s): align visual to GT by Sim3, detrend (quadratic in time) both
the aligned visual positions and the GT positions; signal = rms of detrended GT, noise = rms of (detrended visual - detrended GT).  The scale estimator
regresses accelerometer-integrated displacement on the visual displacement, so its expected attenuation is  S/(S+N)  (variances).
usage: vis_noise.py <seq> <tracker>   (-> prints, and returns dict)"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C
import vis_scale as V
from benchmark import umeyama_alignment


def detrend(P, tau, deg=2):
    B = np.vander(tau, deg + 1)
    co, *_ = np.linalg.lstsq(B, P, rcond=None)
    return P - B @ co


def noise_signal(seq, tk, L, stride=2.0, d=None):
    d = d or V.load_ds(seq, tk)
    tr, gt = d['tr'], d['gt']
    out = []
    t = tr[0, 0] + 0.5
    seg_id = np.r_[0, np.cumsum(np.diff(tr[:, 0]) > 0.5)]
    while t + L < tr[-1, 0]:
        sel = (tr[:, 0] >= t) & (tr[:, 0] <= t + L)
        if sel.sum() < 4 or seg_id[sel].min() != seg_id[sel].max() or np.ptp(tr[sel, 0]) < 0.8 * L: t += stride; continue
        tt = tr[sel, 0]; Pv = tr[sel, 1:4]
        ok = (tt > gt[0, 0]) & (tt < gt[-1, 0])
        if ok.sum() < 4: t += stride; continue
        Pg = np.c_[[np.interp(tt[ok], gt[:, 0], gt[:, c]) for c in (1, 2, 3)]].T
        if np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum() < 0.5: t += stride; continue
        R, tx, s = umeyama_alignment(Pv[ok], Pg, with_scale=True)
        Pa = s * (R @ Pv[ok].T).T + tx
        tau = tt[ok] - tt[ok][0]
        dg = detrend(Pg, tau); da = detrend(Pa, tau)
        sig = float(np.sqrt((dg ** 2).sum(1).mean())); noi = float(np.sqrt(((da - dg) ** 2).sum(1).mean()))
        out.append((t, sig, noi))
        t += stride
    return np.array(out)


if __name__ == '__main__':
    seq, tk = sys.argv[1], sys.argv[2]
    d = V.load_ds(seq, tk)
    for L in (3.0, 4.0, 8.0, 12.0):
        o = noise_signal(seq, tk, L, d=d)
        S, N = o[:, 1], o[:, 2]
        pred = S ** 2 / (S ** 2 + N ** 2)
        rows = V.run(seq, tk, L, d=d, stride=2.0)
        r = np.array([x['ratio'] for x in rows])
        print(f"{seq}_{tk} L={L:g}: n={len(o)} detrended GT signal rms {1e3*np.median(S):.1f} mm, visual error rms {1e3*np.median(N):.1f} mm, predicted attenuation S2/(S2+N2) med {np.median(pred):.2f}, measured ratio med {np.median(r):.2f}")
