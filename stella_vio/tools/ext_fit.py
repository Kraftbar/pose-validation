#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""Rotation-only camera-IMU extrinsic and time-offset fit from a visual trajectory and the gyro (Kabsch on angular-velocity vectors).
Model: w_cam(t) = R_CB * (w_gyro(t + toff) - bg), IMU time = camera time + toff. Used to check / replace calibration files before the VI initialiser.
usage: ext_fit.py <dataset ...>   (names of imu_init_eval.datasets())   prints cfg residual vs fitted residual, fitted R_BC angle difference, toff."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import imu_init_eval as E  # noqa: E402


def rotvec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (0.5 if th < 1e-6 else th / (2 * np.sin(th)))


def pairs(tr, imu, toff):
    ti, gi = imu[:, 0], imu[:, 1:4]
    cum = np.vstack([np.zeros(3), np.cumsum(0.5 * (gi[1:] + gi[:-1]) * np.diff(ti)[:, None], axis=0)])
    I = lambda t: np.c_[[np.interp(t, ti, cum[:, k]) for k in range(3)]].T
    t0, t1 = tr[:-1, 0], tr[1:, 0]
    dt = t1 - t0
    ok = (dt > 0) & (dt < 0.2) & (t0 + toff > ti[0]) & (t1 + toff < ti[-1])
    wg = (I(t1[ok] + toff) - I(t0[ok] + toff)) / dt[ok][:, None]
    Rs = [E.q2R(q) for q in tr[:, 4:8]]
    wc = np.array([rotvec(Rs[i].T @ Rs[i + 1]) / dt[i] for i in np.where(ok)[0]])
    return wg, wc


def kabsch(wg, wc):
    U, _, Vt = np.linalg.svd(wc.T @ wg)
    D = np.diag([1, 1, np.linalg.det(U @ Vt)])
    return U @ D @ Vt   # R_CB: w_c = R_CB w_b


def resid(R_CB, wg, wc):
    d = wc - wg @ R_CB.T
    m = np.linalg.norm(wc, axis=1) > 0.15
    return float(np.sqrt((d[m] ** 2).sum(1).mean())), float(np.sqrt((wc[m] ** 2).sum(1).mean()))


def fit(imu, tr, cfg_RBC, scan=np.arange(-0.08, 0.0801, 0.005)):
    best = None
    for to in scan:
        wg, wc = pairs(tr, imu, to)
        R = kabsch(wg, wc)
        r, _ = resid(R, wg, wc)
        if best is None or r < best[0]:
            best = (r, to, R)
    wg, wc = pairs(tr, imu, 0.0)
    r_cfg, mag = resid(cfg_RBC.T, wg, wc)
    r_fit, to, R = best
    ang = np.degrees(np.arccos(np.clip((np.trace(R @ cfg_RBC) - 1) / 2, -1, 1)))   # R_CB(fit) * R_BC(cfg) = I when identical
    return dict(resid_cfg_rad_s=r_cfg, resid_fit_rad_s=r_fit, rms_w=mag, toff=float(to), R_CB=R, rot_diff_deg=float(ang))


if __name__ == '__main__':
    D = E.datasets()
    for ds in sys.argv[1:]:
        d = D[ds]
        imu = np.loadtxt(d['imu'], delimiter=',', comments='#'); imu[:, 0] *= 1e-9
        tr = E.read_tum(d['traj'])
        f = fit(imu, tr, d['T_BC'][:3, :3])
        print(f"{ds}: |w_vis| rms {f['rms_w']:.3f} rad/s; residual with cfg extrinsic {f['resid_cfg_rad_s']:.3f}, fitted {f['resid_fit_rad_s']:.3f} (toff {f['toff']*1e3:+.0f} ms); "
              f"fitted vs cfg rotation differs by {f['rot_diff_deg']:.1f} deg")
