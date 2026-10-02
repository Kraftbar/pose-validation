#!/usr/bin/env python3
"""Offline loosely-coupled GNSS + VIO smoother (own code, numpy only).

Input : a VIO trajectory in its own gravity-aligned local frame (TUM rows t x y z qx qy qz qw, IMU pose) and GNSS position fixes
        in a global ENU frame (t, x, y, z, sigma_h, sigma_v) at the antenna.
Output: the VIO trajectory re-expressed in the global frame, with drift corrected by the fixes (TUM rows, IMU pose).

Model (a 5-DoF-per-node pose graph; gravity/roll/pitch are trusted from the VIO):
  node i (every --node-dt seconds): yaw correction psi_i (global = Rz(psi_i) * local), position p_i (global IMU position), scale s_i
  odometry factor i->j : Rz(-psi_i)(p_j - p_i) - s_i * (pL_j - pL_i) ~ N(0, (sp + kp*|d|)^2)
  yaw random walk      : psi_j - psi_i ~ N(0, (qpsi*sqrt(dt))^2)
  scale random walk    : s_j - s_i   ~ N(0, (qs*sqrt(dt))^2),  weak prior s ~ 1
  GNSS factor          : p_i + Rz(psi_i) R_L,i r_SA - z_i ~ N(0, diag(sh, sh, sv)^2), Huber-robustified (IRLS)
Solved by Gauss-Newton on the block-tridiagonal normal equations (block Thomas algorithm, O(N)).
--causal W solves a sliding window of W seconds at every GNSS epoch (older nodes frozen) and keeps only the newest node estimate.
The VIO frame-rate output is the nearest earlier node's estimate propagated with the VIO relative motion.

usage: gnss_loose_fusion.py --traj vio.csv --gps gps0/data.csv --rsa -0.01,-0.03,-0.06 --out fused.tum [--causal 30] [--node-dt 1]
"""
import argparse, sys
from pathlib import Path
import numpy as np

# default noise model (set once from physical reasoning, not tuned on the test sequences)
SP, KP = 0.05, 0.02      # odometry position sigma: sp + kp*|d| [m]
QPSI = np.radians(0.5)   # yaw drift [rad/sqrt(s)]
QS = 0.003               # scale drift [1/sqrt(s)]
HUBER = 2.5              # Huber threshold in sigmas


def read_tum(p):
    rows = []
    for l in Path(p).read_text().splitlines():
        l = l.replace(',', ' ').strip()
        if not l or l[0] == '#': continue
        try: rows.append([float(x) for x in l.split()[:8]])
        except ValueError: continue
    a = np.array(rows)
    if len(a) and a[0, 0] > 1e12: a[:, 0] *= 1e-9
    return a


def read_gps(p):
    rows = []
    for l in Path(p).read_text().splitlines()[1:]:
        v = [float(x) for x in l.split(',')[:7]]
        rows.append([v[0] * 1e-9, v[1], v[2], v[3], max(v[4], 0.02), max(v[6], 0.02)])
    return np.array(rows)


def quat_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2; return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = np.argmax(np.diag(R)); j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = np.zeros(4); q[i] = s / 4; q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; q[3] = (R[k, j] - R[j, k]) / s
    return q


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def dRz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[-s, -c, 0], [c, -s, 0], [0, 0, 0.]])


class Graph:
    """nodes: times tn[N], local positions PL[N,3], local rotations RL[N,3,3]; gnss: per-node (z[3], sigma[3]) or None"""
    def __init__(self, tn, PL, RL, rsa, gnss):
        self.tn, self.PL, self.RL, self.gnss = tn, PL, RL, gnss
        self.A = np.array([RL[i] @ np.asarray(rsa) for i in range(len(tn))])  # antenna offset, local-world axes

    def solve(self, X, lo, hi, iters=12):
        """X[N,5] = (psi, px, py, pz, s); Gauss-Newton on nodes lo..hi (inclusive); node lo-1 (if any) is a frozen boundary."""
        n = hi - lo + 1
        for _ in range(iters):
            D = np.zeros((n, 5, 5)); O = np.zeros((max(n - 1, 0), 5, 5)); b = np.zeros((n, 5))
            for k in range(n):
                i = lo + k
                g = self.gnss[i]
                if g is not None:
                    z, sg = g
                    psi = X[i, 0]
                    r = X[i, 1:4] + Rz(psi) @ self.A[i] - z
                    J = np.zeros((3, 5)); J[:, 0] = dRz(psi) @ self.A[i]; J[:, 1:4] = np.eye(3)
                    for ax in range(3):
                        e = r[ax] / sg[ax]
                        w = 1.0 if abs(e) <= HUBER else HUBER / abs(e)
                        Ja = J[ax] / sg[ax]
                        D[k] += w * np.outer(Ja, Ja); b[k] -= w * Ja * e
                D[k, 4, 4] += 1 / 0.04; b[k, 4] -= (X[i, 4] - 1.0) / 0.04   # weak scale prior s ~ 1 (sigma 0.2)
            for i in range(max(lo - 1, 0), hi):
                j = i + 1
                dt = self.tn[j] - self.tn[i]
                dL = self.PL[j] - self.PL[i]
                sp = SP + KP * np.linalg.norm(dL)
                dp = X[j, 1:4] - X[i, 1:4]
                Ji = np.zeros((3, 5)); Jj = np.zeros((3, 5))
                Ji[:, 0] = dRz(X[i, 0]).T @ dp; Ji[:, 1:4] = -Rz(X[i, 0]).T; Ji[:, 4] = -dL; Jj[:, 1:4] = Rz(X[i, 0]).T
                r = Rz(X[i, 0]).T @ dp - X[i, 4] * dL
                sq, ss = QPSI * np.sqrt(dt), QS * np.sqrt(dt)
                ry = np.array([(X[j, 0] - X[i, 0]) / sq]); Jyi = np.zeros((1, 5)); Jyj = np.zeros((1, 5)); Jyi[0, 0] = -1 / sq; Jyj[0, 0] = 1 / sq
                rs = np.array([(X[j, 4] - X[i, 4]) / ss]); Jsi = np.zeros((1, 5)); Jsj = np.zeros((1, 5)); Jsi[0, 4] = -1 / ss; Jsj[0, 4] = 1 / ss
                for r_, Ji_, Jj_ in ((r / sp, Ji / sp, Jj / sp), (ry, Jyi, Jyj), (rs, Jsi, Jsj)):
                    ki, kj = i - lo, j - lo
                    D[kj] += Jj_.T @ Jj_; b[kj] -= Jj_.T @ r_
                    if ki >= 0:
                        D[ki] += Ji_.T @ Ji_; b[ki] -= Ji_.T @ r_; O[ki] += Ji_.T @ Jj_
            for k in range(n): D[k] += 1e-6 * np.eye(5)
            dx = self._thomas(D, O, b)
            X[lo:hi + 1] += dx
            if np.abs(dx).max() < 1e-6: break
        return X

    @staticmethod
    def _thomas(D, O, b):
        """solve block-tridiagonal H x = b, H[k,k]=D[k], H[k,k+1]=O[k], H[k+1,k]=O[k]^T"""
        n = len(D); Dp = D.copy(); bp = b.copy(); x = np.zeros_like(b)
        for k in range(1, n):
            Dp[k] -= O[k - 1].T @ np.linalg.solve(Dp[k - 1], O[k - 1])
            bp[k] -= O[k - 1].T @ np.linalg.solve(Dp[k - 1], bp[k - 1])
        x[-1] = np.linalg.solve(Dp[-1], bp[-1])
        for k in range(n - 2, -1, -1):
            x[k] = np.linalg.solve(Dp[k], bp[k] - O[k] @ x[k + 1])
        return x


def propagate(Xprev, PL, k):
    X = Xprev.copy(); X[1:4] = Xprev[1:4] + Rz(Xprev[0]) @ (Xprev[4] * (PL[k] - PL[k - 1])); return X


def fuse(traj, gps, rsa, node_dt=1.0, causal=0.0):
    t = traj[:, 0]
    idx = np.clip(np.searchsorted(t, np.arange(t[0], t[-1], node_dt)), 0, len(t) - 1)
    tn = t[idx]
    PL = traj[idx, 1:4]; RL = np.array([quat_R(q) for q in traj[idx, 4:8]])
    gnss = [None] * len(tn)
    if len(gps):
        j = np.clip(np.searchsorted(gps[:, 0], tn), 1, len(gps) - 1)
        j = np.where(np.abs(gps[j - 1, 0] - tn) < np.abs(gps[j, 0] - tn), j - 1, j)
        for i in range(len(tn)):
            if abs(gps[j[i], 0] - tn[i]) <= 0.5 * node_dt + 1e-6:
                g = gps[j[i]]; gnss[i] = (g[1:4].copy(), np.array([g[4], g[4], g[5]]))
    G = Graph(tn, PL, RL, rsa, gnss)
    # initial global alignment: yaw + scale + translation (2D Procrustes) on the first 30 s of fixes (causal) or all fixes (batch)
    have = [i for i in range(len(tn)) if gnss[i] is not None and (not causal or tn[i] - tn[0] <= 30.0)]
    if len(have) < 3: raise SystemExit('need >= 3 GNSS fixes for initialisation')
    ii = np.array(have); Zg = np.array([gnss[i][0] for i in have]); Pl = PL[ii] + G.A[ii]
    mz, ml = Zg.mean(0), Pl.mean(0); a, c = (Pl - ml)[:, :2], (Zg - mz)[:, :2]
    psi0 = np.arctan2((a[:, 0] * c[:, 1] - a[:, 1] * c[:, 0]).sum(), (a * c).sum())
    s0 = float(np.clip(np.sqrt((c ** 2).sum() / max((a ** 2).sum(), 1e-9)), 0.5, 1.5))
    X = np.zeros((len(tn), 5)); X[:, 0] = psi0; X[:, 4] = s0
    X[:, 1:4] = (Rz(psi0) @ (s0 * (PL + G.A - ml).T)).T + mz - (Rz(psi0) @ G.A.T).T
    if not causal:
        X = G.solve(X, 0, len(tn) - 1, iters=25)
        Xo = X
    else:
        W = max(int(causal / node_dt), 3)
        X = G.solve(X, 0, min(len(tn) - 1, 10), iters=10)   # settle the first nodes
        Xo = X.copy()
        for k in range(11, len(tn)):
            X[k] = propagate(X[k - 1], PL, k)
            lo = max(0, k - W)
            if any(gnss[i] is not None for i in range(lo, k + 1)):
                X = G.solve(X, lo, k, iters=4)
            Xo[k] = X[k]
    out = np.zeros((len(t), 8)); out[:, 0] = t
    node_of = np.clip(np.searchsorted(tn, t, side='right') - 1, 0, len(tn) - 1)
    for n_i in range(len(t)):
        k = node_of[n_i]; psi, s = Xo[k, 0], Xo[k, 4]
        out[n_i, 1:4] = Xo[k, 1:4] + Rz(psi) @ (s * (traj[n_i, 1:4] - PL[k]))
        out[n_i, 4:8] = R_quat(Rz(psi) @ quat_R(traj[n_i, 4:8]))
    return out, Xo, tn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', required=True); ap.add_argument('--gps', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--rsa', default='0,0,0'); ap.add_argument('--node-dt', type=float, default=1.0); ap.add_argument('--causal', type=float, default=0.0)
    a = ap.parse_args()
    traj = read_tum(a.traj); gps = read_gps(a.gps)
    out, X, tn = fuse(traj, gps, [float(x) for x in a.rsa.split(',')], a.node_dt, a.causal)
    np.savetxt(a.out, out, fmt=['%.9f'] + ['%.6f'] * 3 + ['%.8f'] * 4)
    print('wrote', a.out, len(out), 'poses; nodes', len(tn), 'scale median %.3f' % np.median(X[:, 4]))


if __name__ == '__main__':
    main()
