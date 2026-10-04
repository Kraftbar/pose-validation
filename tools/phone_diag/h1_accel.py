# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H1/H2/H3 core: GT-derived specific force vs raw accelerometer, plus gyro vs GT angular velocity (gain, delay, axis convention, lever arm).
usage: h1_accel.py [seq ...]   -> runs/phone_diag/h1_<seq>.json, prints a table.
Method: GT positions -> second difference at step m*h (h = GT period); the accelerometer is converted to a world-frame acceleration with the GT orientation
(R_wI = R_wb R_bI, R_bI fitted from gyro vs GT omega) and filtered with the SAME triangular kernel the second difference implies, so both sides see
the identical band-limit (no differentiation transfer-function correction needed).  Linear least squares for per-IMU-axis gain k, bias b, gravity g_w, lever arm r."""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C


def gt_omega_b(d):
    R = d['R']; tg = d['tg']
    w = np.zeros((len(tg) - 1, 3))
    for i in range(len(tg) - 1):
        w[i] = C.rotvec(R[i].T @ R[i + 1]) / (tg[i + 1] - tg[i])
    return w   # body-frame mean angular velocity over [tg_i, tg_i+1]


def gyro_pairs(d, tau):
    ti, gi = d['ti'], d['w']
    cum = np.vstack([np.zeros(3), np.cumsum(0.5 * (gi[1:] + gi[:-1]) * np.diff(ti)[:, None], axis=0)])
    I = lambda t: np.c_[[np.interp(t, ti, cum[:, k]) for k in range(3)]].T
    t0, t1 = d['tg'][:-1] + tau, d['tg'][1:] + tau          # GT interval shifted by tau on the IMU clock
    dt = np.diff(d['tg'])
    ok = (dt < 0.25) & (t0 > ti[0]) & (t1 < ti[-1])
    wg = (I(t1[ok]) - I(t0[ok])) / dt[ok][:, None]
    return ok, wg


def fit_gyro(d, scan=np.arange(-0.3, 0.3001, 0.005)):
    wb = gt_omega_b(d)
    best = None
    curve = []
    for tau in scan:
        ok, wg = gyro_pairs(d, tau)
        m = np.linalg.norm(wb[ok], axis=1) > 0.2
        R = C.kabsch(wg[m], wb[ok][m])            # wb = R_bI wg
        r = np.sqrt(((wb[ok][m] - wg[m] @ R.T) ** 2).sum(1).mean())
        curve.append((float(tau), float(r)))
        if best is None or r < best[0]:
            best = (r, tau, R)
    r, tau, R = best
    ok, wg = gyro_pairs(d, tau)
    m = np.linalg.norm(wb[ok], axis=1) > 0.2
    # general linear map wb = M wg  -> gains (singular values), isotropic gain
    M = np.linalg.lstsq(wg[m], wb[ok][m], rcond=None)[0].T
    sv = np.linalg.svd(M, compute_uv=False)
    k_iso = float((wb[ok][m] * (wg[m] @ R.T)).sum() / (wg[m] ** 2).sum())
    w_rms = float(np.sqrt((wb[ok][m] ** 2).sum(1).mean()))
    return dict(tau=float(tau), R_bI=R, resid=float(r), w_rms=w_rms, gain_iso=k_iso, gain_sv=sv.tolist(), curve=curve, n=int(m.sum()))


def slerp_R(d, t):
    """GT body->world at IMU times t by quaternion nlerp (GT rate 10 Hz, rotation per step < 0.1 rad)"""
    tg, q = d['tg'], d['q'].copy()
    for i in range(1, len(q)):
        if q[i] @ q[i - 1] < 0:
            q[i] = -q[i]
    qi = np.c_[[np.interp(t, tg, q[:, k]) for k in range(4)]].T
    return C.q2R_batch(qi)


def build(d, tau, R_bI, m, w_extra=None):
    """returns y (n,3), design columns per unknown (n,3,ncol) for GT samples k, with step m*h; mask of valid samples"""
    tg, p = d['tg'], d['p']
    h = float(np.median(np.diff(tg)))
    H = m * h
    cont = C.gt_continuous_mask(tg, 2.5 * h)
    n = len(tg)
    ks = np.arange(m, n - m)
    ok = np.array([cont[k - m:k + m + 1].all() and abs(tg[k + m] - tg[k - m] - 2 * H) < 0.2 * H for k in ks])
    ks = ks[ok]
    y = (p[ks + m] - 2 * p[ks] + p[ks - m]) / H ** 2
    ti, f, w = d['ti'], d['f'], d['w']
    dti = float(np.median(np.diff(ti)))
    # per IMU sample world rotation (at IMU stamp, GT shifted by tau): R_wI = R_wb(t - tau?) ...
    # GT time tg is already on the IMU clock; extra shift tau means GT(t) corresponds to IMU(t+tau): evaluate GT orientation at t_imu - tau
    Rwb = slerp_R(d, ti - tau)
    RwI = Rwb @ R_bI                       # R_bI: b = R_bI I ... careful: fit gives wb = R wg, i.e. R maps IMU->body
    # world-frame direction columns for each IMU axis j: RwI[:, :, j]
    nk = len(ks)
    cols = np.zeros((nk, 3, 9 + 3))
    # lever-arm terms need omega_b, alpha_b from the gyro (smoothed) rotated to body
    wb_i = w @ R_bI.T
    ker = int(round(0.1 / dti))
    box = np.ones(2 * ker + 1) / (2 * ker + 1)
    wbs = np.c_[[np.convolve(wb_i[:, c], box, 'same') for c in range(3)]].T
    alb = np.gradient(wbs, ti, axis=0)
    LA = np.zeros((len(ti), 3, 3))         # world-frame acceleration of a point at body offset e_c:  R_wb (alpha x e_c + w x (w x e_c))
    for c in range(3):
        e = np.zeros(3); e[c] = 1
        a_b = np.cross(alb, e) + np.cross(wb_i, np.cross(wb_i, e))
        LA[:, :, c] = np.einsum('nij,nj->ni', Rwb, a_b)
    nhalf = int(round(H / dti))
    s = np.arange(-nhalf, nhalf + 1) * dti
    wt = np.maximum(H - np.abs(s), 0) / H ** 2 * dti
    for r_, k in enumerate(ks):
        j0 = int(round((tg[k] - ti[0]) / dti))
        idx = np.clip(j0 + np.arange(-nhalf, nhalf + 1), 0, len(ti) - 1)
        W = wt[:, None, None]
        for j in range(3):
            cols[r_, :, j] = (wt[:, None] * RwI[idx][:, :, j] * f[idx][:, j:j + 1]).sum(0)      # gain k_j
            cols[r_, :, 3 + j] = (wt[:, None] * RwI[idx][:, :, j]).sum(0)                         # bias b_j
            cols[r_, :, 9 + j] = (wt[:, None] * LA[idx][:, :, j]).sum(0)                          # lever arm r_j (body)
        cols[r_, :, 6:9] = np.eye(3) * wt.sum()                                                    # gravity g_w
    t_k = tg[ks]
    return y, cols, ks, t_k, H


def solve(y, cols, use):
    A = cols[:, :, use].reshape(-1, len(use)); b = y.reshape(-1)
    th, *_ = np.linalg.lstsq(A, b, rcond=None)
    res = b - A @ th
    return th, res


def analyse(name, Hs=(0.2, 0.3, 0.5), verbose=True):
    d = C.load(name)
    h0 = float(np.median(np.diff(d['tg']))); ms = sorted({max(1, int(round(x / h0))) for x in Hs})
    gy = fit_gyro(d)
    out = dict(seq=name, gyro=dict(tau=gy['tau'], resid=gy['resid'], w_rms=gy['w_rms'], gain_iso=gy['gain_iso'], gain_sv=gy['gain_sv'], R_bI=gy['R_bI'].tolist(), n=gy['n']))
    # gyro dependence of the coarse tau on fine scan
    f_norm = np.linalg.norm(d['f'], axis=1)
    out['acc_norm_median'] = float(np.median(f_norm)); out['acc_norm_static_median'] = float(np.median(f_norm[np.linalg.norm(d['w'], axis=1) < 0.1]))
    res = {}
    for m in ms:
        y, cols, ks, t_k, H = build(d, gy['tau'], gy['R_bI'], m)
        out_m = {}
        # model A: isotropic gain, gravity free, no bias
        Aiso = np.concatenate([cols[:, :, 0:3].sum(2, keepdims=True), cols[:, :, 6:9]], axis=2)   # one gain k (sum over axes) + g_w
        th, r = solve(y, Aiso, [0, 1, 2, 3])
        out_m['iso_gain'] = float(th[0]); out_m['iso_g'] = th[1:].tolist(); out_m['iso_gnorm'] = float(np.linalg.norm(th[1:]))
        out_m['iso_resid_rms'] = float(np.sqrt((r ** 2).mean()))
        # model B: per-axis gains + g_w (no bias: bias and gain along gravity are collinear for a phone held at constant tilt)
        th, r = solve(y, cols, [0, 1, 2, 6, 7, 8])
        out_m['axis_gain'] = th[0:3].tolist(); out_m['bias'] = [0, 0, 0]; out_m['g_w'] = th[3:6].tolist(); out_m['g_norm'] = float(np.linalg.norm(th[3:6]))
        out_m['axis_resid_rms'] = float(np.sqrt((r ** 2).mean()))
        # model C: + lever arm
        th, r = solve(y, cols, [0, 1, 2, 6, 7, 8, 9, 10, 11])
        out_m['lever_r_body'] = th[6:9].tolist(); out_m['lever_gain'] = th[0:3].tolist(); out_m['lever_resid_rms'] = float(np.sqrt((r ** 2).mean()))
        # model D: gravity-free (high-passed) regression: subtract a 2 s running mean from y and from every column (gravity, slow tilt error and bias drop out)
        nb = max(3, int(round(2.0 / h0)) | 1); bx = np.ones(nb) / nb
        hp = lambda a: a - np.apply_along_axis(lambda v: np.convolve(np.pad(v, nb // 2, mode='edge'), bx, 'valid'), 0, a)
        yh = hp(y); ch = np.stack([hp(cols[:, :, j]) for j in range(3)], axis=2)
        th, r = solve(yh, ch, [0, 1, 2])
        out_m['hp_axis_gain'] = th.tolist()
        A1 = ch.sum(2, keepdims=True); th1, r1 = solve(yh, A1, [0])
        out_m['hp_iso_gain'] = float(th1[0]); out_m['hp_resid_rms'] = float(np.sqrt((r1 ** 2).mean())); out_m['hp_y_rms'] = float(np.sqrt((yh ** 2).sum(1).mean()))
        # lever arm in the high-passed model
        ca = np.concatenate([A1, np.stack([hp(cols[:, :, 9 + j]) for j in range(3)], axis=2)], axis=2)
        th2, r2 = solve(yh, ca, [0, 1, 2, 3]); out_m['hp_lever_gain'] = float(th2[0]); out_m['hp_lever_r'] = th2[1:].tolist(); out_m['hp_lever_resid_rms'] = float(np.sqrt((r2 ** 2).mean()))
        out_m['y_rms_dyn'] = float(np.sqrt(((y - y.mean(0)) ** 2).sum(1).mean()))
        out_m['n'] = int(len(ks)); out_m['H'] = H
        # gain with g_w fixed to unit-z-ish norm? keep free
        res[str(m)] = out_m
    out['by_step'] = res
    if verbose:
        print(f"== {name}: gyro tau {gy['tau']*1e3:+.0f} ms, resid {gy['resid']:.3f} of {gy['w_rms']:.3f} rad/s, gain_iso {gy['gain_iso']:.3f}, sv {np.round(gy['gain_sv'],3)}, "
              f"|f| median {out['acc_norm_median']:.2f} static {out['acc_norm_static_median']:.2f}")
        for m, o in res.items():
            print(f"  H={o['H']:.2f}s n={o['n']}: iso k={o['iso_gain']:.3f} |g|={o['iso_gnorm']:.2f} res {o['iso_resid_rms']:.2f} | axis k={np.round(o['axis_gain'],3)} bias={np.round(o['bias'],2)} |g|={o['g_norm']:.2f} res {o['axis_resid_rms']:.2f} | lever r={np.round(o['lever_r_body'],2)} res {o['lever_resid_rms']:.2f} | HP iso k={o['hp_iso_gain']:.3f} axis {np.round(o['hp_axis_gain'],2)} res {o['hp_resid_rms']:.2f}/{o['hp_y_rms']:.2f} lever r={np.round(o['hp_lever_r'],2)} k {o['hp_lever_gain']:.3f} res {o['hp_lever_resid_rms']:.2f}")
    return out


if __name__ == '__main__':
    names = sys.argv[1:] or list(C.SEQ)
    C.OUT.mkdir(parents=True, exist_ok=True)
    for n in names:
        o = analyse(n)
        (C.OUT / f'h1_{n}.json').write_text(json.dumps(o, indent=1))


def fit_matrix(name, Hs=0.3, d=None, gy=None):
    """free 3x3 map X (accelerometer axes -> GT-body axes) + g_w + bias-free; y = tri(R_wb X f) + g_w.  Returns X, its SVD (gains) and the nearest rotation vs the gyro-fitted R_bI."""
    d = d or C.load(name); gy = gy or fit_gyro(d)
    h = float(np.median(np.diff(d['tg'])))
    m = max(1, int(round(Hs / h)))
    tg, p, ti, f = d['tg'], d['p'], d['ti'], d['f']
    H = m * h; dti = float(np.median(np.diff(ti)))
    Rwb = slerp_R(d, ti - gy['tau'])
    cont = C.gt_continuous_mask(tg, 2.5 * h)
    ks = np.arange(m, len(tg) - m)
    ks = np.array([k for k in ks if cont[k - m:k + m + 1].all()])
    y = (p[ks + m] - 2 * p[ks] + p[ks - m]) / H ** 2
    nh = int(round(H / dti)); s = np.arange(-nh, nh + 1) * dti; wt = np.maximum(H - np.abs(s), 0) / H ** 2 * dti
    A = np.zeros((len(ks), 3, 12))
    for r_, k in enumerate(ks):
        j0 = int(round((tg[k] - ti[0]) / dti)); idx = np.clip(j0 + np.arange(-nh, nh + 1), 0, len(ti) - 1)
        # R_wb(t) X f(t): column (a,b): sum wt * Rwb[:, a] * f[b]   (X[a,b])
        for a in range(3):
            for b in range(3):
                A[r_, :, 3 * a + b] = (wt[:, None] * Rwb[idx][:, :, a] * f[idx][:, b:b + 1]).sum(0)
        A[r_, :, 9:12] = np.eye(3) * wt.sum()
    th, *_ = np.linalg.lstsq(A.reshape(-1, 12), y.reshape(-1), rcond=None)
    X = th[:9].reshape(3, 3); gw = th[9:]
    U, S, Vt = np.linalg.svd(X)
    Rn = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    dR = Rn @ gy['R_bI'].T
    ang = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
    res = y.reshape(-1) - A.reshape(-1, 12) @ th
    return dict(X=X.tolist(), sv=S.tolist(), det=float(np.linalg.det(X)), g_w=gw.tolist(), g_norm=float(np.linalg.norm(gw)), angle_vs_gyro_R_deg=ang,
                resid_rms=float(np.sqrt((res ** 2).mean())), y_rms=float(np.sqrt((y ** 2).sum(1).mean())), H=H)
