#!/usr/bin/env python3
"""ATE of a Basalt TUM trajectory (IMU pose, seconds) against the EuRoC GT (state_groundtruth_estimate0, ns).
Reuses benchmark.ate_rmse / umeyama_alignment (no own Umeyama). Association: nearest GT sample within 5 ms (as in
basalt_port/reference/README.md); --interp uses linear GT interpolation instead (removes the up-to-2.5 ms association error).
Prints Sim3 (benchmark.ate_rmse), SE3 (umeyama, no scale) and 4-DoF (yaw + translation, gravity-aligned world) ATE.
Run with external/gnss/venv/bin/python.   usage: ate_eval.py est.tum <seq dir> [--interp] [--first S]"""
import argparse, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import benchmark as bm


def load_gt(seq):
    d = np.loadtxt(Path(seq) / "mav0/state_groundtruth_estimate0/data.csv", delimiter=",", comments="#")
    return d[:, 0] * 1e-9, d[:, 1:4]


def associate(t, p, gt_t, gt_p, interp):
    if interp:
        ok = (t >= gt_t[0]) & (t <= gt_t[-1])
        g = np.stack([np.interp(t[ok], gt_t, gt_p[:, k]) for k in range(3)], 1)
        return p[ok], g, t[ok]
    j = np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - t) < np.abs(gt_t[j] - t), j - 1, j)
    ok = np.abs(gt_t[j] - t) <= 5e-3
    return p[ok], gt_p[j[ok]], t[ok]


def yaw_align(src, dst):
    """4-DoF (rotation about z + translation) least squares alignment; returns aligned src."""
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    num = (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()
    den = (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]).sum()
    th = np.arctan2(num, den)
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    return (R @ a.T).T + md


def evaluate(est_file, seq, interp=False, first=None):
    e = np.loadtxt(est_file)
    gt_t, gt_p = load_gt(seq)
    p, g, t = associate(e[:, 0], e[:, 1:4], gt_t, gt_p, interp)
    if first:
        m = t - t[0] <= first
        p, g = p[m], g[m]
    sim3 = bm.ate_rmse(p, g)
    R, tt, s = bm.umeyama_alignment(p, g, with_scale=False)
    se3 = float(np.sqrt(((bm.apply_alignment(p, R, tt, 1.0) - g) ** 2).sum(1).mean()))
    y = float(np.sqrt(((yaw_align(p, g) - g) ** 2).sum(1).mean()))
    return {"n": len(p), "sim3": sim3["ate_rmse"], "scale": sim3["scale"], "se3": se3, "yaw4": y,
            "path_len": float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum())}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("est")
    ap.add_argument("seq")
    ap.add_argument("--interp", action="store_true")
    ap.add_argument("--first", type=float, default=None)
    a = ap.parse_args()
    r = evaluate(a.est, a.seq, a.interp, a.first)
    print("%s n=%d Sim3 %.4f (scale %.4f)  SE3 %.4f  yaw4 %.4f  gt_path %.1f m" % (Path(a.est).name, r["n"], r["sim3"], r["scale"], r["se3"], r["yaw4"], r["path_len"]))
