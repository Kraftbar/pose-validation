"""Scoring for GNSS-VIO runs against the dataset RTK reference (own code; alignment = benchmark.umeyama_alignment, never reimplemented).
Trajectory file: TUM-like rows `t x y z qx qy qz qw` (t in s or ns). Poses are IMU/body poses; scored at the GNSS antenna: p + R(q) r_SA.
GT = receiver RTK fix (gt_pvt.csv: t,E,N,U,fix,carr,nsv,h_acc,v_acc,pdop), only epochs with h_acc <= GT_HACC are used."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, '/home/nybo/github/pose-validation')
from benchmark import umeyama_alignment, apply_alignment  # noqa: E402

GT_HACC = 0.10  # m


def read_traj(p):
    rows = []
    for l in Path(p).read_text().splitlines():
        l = l.replace(',', ' ').strip()
        if not l or l[0] == '#': continue
        try: rows.append([float(x) for x in l.split()[:8]])  # 4 columns (t x y z) are accepted too
        except ValueError: continue
    w = max(len(r) for r in rows) if rows else 0
    a = np.array([r for r in rows if len(r) == w]) if rows else np.zeros((0, 8))
    if len(a) and a[0, 0] > 1e12: a[:, 0] *= 1e-9
    return a


def quat_to_R(q):  # q = x y z w, Nx4
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def antenna(traj, r_SA):
    if traj.shape[1] < 8: return traj[:, 1:4]  # positions already at the antenna / body point
    R = quat_to_R(traj[:, 4:8] / np.linalg.norm(traj[:, 4:8], axis=1, keepdims=True))
    return traj[:, 1:4] + R @ np.asarray(r_SA)


class GT:
    """reference positions: t[N], p[N,3], good[N] (bool quality gate)"""
    def __init__(self, t, p, good):
        self.t, self.p, self.good = np.asarray(t), np.asarray(p), np.asarray(good, bool)

    @staticmethod
    def from_pvt(seq, hacc=GT_HACC):
        g = np.loadtxt(Path(seq) / 'gt_pvt.csv', delimiter=',')
        return GT(g[:, 0], g[:, 1:4], g[:, 7] <= hacc)

    @staticmethod
    def from_tum(path, dt=0.0):
        g = np.loadtxt(path)
        return GT(g[:, 0] + dt, g[:, 1:4], np.ones(len(g), bool))

    def at(self, t):
        gp = np.c_[[np.interp(t, self.t, self.p[:, k]) for k in range(3)]].T
        i = np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1)
        ok = self.good[i - 1] & self.good[i] & ((self.t[i] - self.t[i - 1]) < 0.25) & (t >= self.t[0]) & (t <= self.t[-1])
        return gp, ok


def score(traj, gt, r_SA, n_frames, duration, geo=False, windows=None):
    """dict: n_pose, coverage, ate_se3 (SE3 align), ate_sim3, scale, [ate_noalign for geo-referenced output], win_<name> = RMSE inside time windows
    (windows: list of (t0, t1, name) in absolute seconds; error is the un-aligned error for geo runs, SE3-aligned error otherwise)"""
    A = antenna(traj, r_SA); t = traj[:, 0]
    out = {'n_pose': int(len(t)), 'coverage': len(t) / n_frames, 'span_s': float(t[-1] - t[0]) if len(t) else 0}
    if len(t) < 20: return out
    gp, ok = gt.at(t)
    if ok.sum() < 20: return out
    e, p = A[ok], gp[ok]
    R, tt, s = umeyama_alignment(e, p, with_scale=False)
    err = np.linalg.norm(apply_alignment(e, R, tt, s) - p, axis=1)
    out['ate_se3'] = float(np.sqrt((err ** 2).mean())); out['n_scored'] = int(ok.sum())
    Rs, ts, ss = umeyama_alignment(e, p, with_scale=True); out['scale'] = float(ss)
    out['ate_sim3'] = float(np.sqrt((np.linalg.norm(apply_alignment(e, Rs, ts, ss) - p, axis=1) ** 2).mean()))
    use = err
    if geo:
        raw = np.linalg.norm(e - p, axis=1)
        out['ate_noalign'] = float(np.sqrt((raw ** 2).mean())); out['noalign_median'] = float(np.median(raw)); out['noalign_max'] = float(raw.max())
        use = raw
    out['_t'] = t[ok]; out['_err'] = use; out['_err_se3'] = err
    if windows:
        for w0, w1, name in windows:
            m = (t[ok] >= w0) & (t[ok] <= w1)
            if m.sum() >= 5: out[f'win_{name}'] = float(np.sqrt((use[m] ** 2).mean()))
    return out
