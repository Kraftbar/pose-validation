# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Independent (numpy) closed-form metric-scale estimator on the same inputs as imu_init_eval (visual trajectory + raw IMU), to separate 'the data' from 'the C initialiser'.
Per window:  s * P_vis(t) = P0 + v0*tau + 0.5*g*tau^2 + I(t),   I = double integral of R_wI f   (R_wI from the visual orientation and the camera-IMU extrinsic),
linear in (s, P0, v0, g).  Truth scale = Sim3 of P_vis to GT positions in the window (as imu_init_eval).  Variants: orientation source (visual / GT),
accelerometer transform (raw, gain, low-pass, delay), position source (visual / GT*1).
usage: vis_scale.py <seq> <tracker: stella|orb3> [--L 4]"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(C_ROOT := Path(__file__).resolve().parents[2]))
import common as C
import h1_accel as A
from benchmark import umeyama_alignment

IMU = C.ROOT / 'runs/stella_vio/imu'


def read_tum(p):
    a = np.loadtxt(p)
    return a[np.argsort(a[:, 0])]


def load_ds(seq, tk):
    ext = np.loadtxt(IMU / f'ext_{seq}_{tk}_fit.txt')
    T = np.eye(4); T[:3, :3] = ext[:9].reshape(3, 3); T[:3, 3] = ext[9:12]
    tr = read_tum(IMU / f'traj_{seq}_{tk}_fit.txt')      # camera stamps already shifted to the IMU clock
    imu_f, gt_f, off = C.SEQ[seq]
    im = np.loadtxt(imu_f, delimiter=',', comments='#'); im[:, 0] *= 1e-9
    g = np.loadtxt(gt_f); g[:, 0] += off
    return dict(T_BC=T, tr=tr, imu=im, gt=g)


def smooth_traj(tr, sig):
    """Gaussian low-pass (sigma sig seconds) of the visual positions, per tracking segment (edges handled by renormalised weights); orientation untouched."""
    out = tr.copy(); t = tr[:, 0]
    seg = np.r_[0, np.cumsum(np.diff(t) > 0.5)]
    for sid in np.unique(seg):
        ix = np.where(seg == sid)[0]; ts = t[ix]
        W = np.exp(-0.5 * ((ts[:, None] - ts[None, :]) / sig) ** 2); W /= W.sum(1, keepdims=True)
        out[ix, 1:4] = W @ tr[ix, 1:4]
    return out


def dint(x, ti):
    dt = np.diff(ti)[:, None] if x.ndim == 2 else np.diff(ti)[:, None, None]
    v = np.vstack([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (x[1:] + x[:-1]) * dt, axis=0)])
    return np.vstack([np.zeros((1,) + x.shape[1:]), np.cumsum(0.5 * (v[1:] + v[:-1]) * dt, axis=0)])


def run(seq, tk, L=4.0, stride=4.0, gain=1.0, lp=None, delay=0.0, ori='vis', fscale=1.0, d=None, bias_mode=None, imu_override=None, gyfit=None, gpin=None, accgain=None, smooth=None, match=None):
    d = d or load_ds(seq, tk)
    if ori == 'gt' and '_dgt' not in d:
        d['_dgt'] = C.load(seq); d['_dgt']['ti'] = d['_dgt']['ti'] + d['_dgt']['t0']; d['_dgt']['tg'] = d['_dgt']['tg'] + d['_dgt']['t0']
        d['_gyfit'] = A.fit_gyro(C.load(seq))
    gyfit = d.get('_gyfit')
    if smooth or match:
        d = dict(d); d['tr'] = smooth_traj(d['tr'], smooth or match)
    tr, im, gt = d['tr'], (imu_override if imu_override is not None else d['imu']), d['gt']
    ti = im[:, 0]; f = im[:, 4:7] * gain * fscale
    if accgain == 'cal':
        f = f * (9.81 / np.median(np.linalg.norm(f, axis=1)))
    if lp:
        n = max(1, int(round(lp / np.median(np.diff(ti)))))
        f = np.c_[[np.convolve(f[:, c], np.ones(n) / n, 'same') for c in range(3)]].T
    if delay:
        f = np.c_[[np.interp(ti - delay, ti, f[:, c]) for c in range(3)]].T
    R_cB = d['T_BC'][:3, :3].T
    # visual orientation at IMU times (nlerp); camera -> visual world
    q = tr[:, 4:8].copy()
    for i in range(1, len(q)):
        if q[i] @ q[i - 1] < 0: q[i] = -q[i]
    qi = np.c_[[np.interp(ti, tr[:, 0], q[:, k]) for k in range(4)]].T
    Rwc = C.q2R_batch(qi)
    RwI = Rwc @ R_cB
    a_w = np.einsum('nij,nj->ni', RwI, f)
    S = dint(a_w, ti)
    if match:     # matched Gaussian low-pass of the accelerometer double integral (same sigma as the visual positions)
        dti_ = float(np.median(np.diff(ti))); kk = np.arange(-int(4 * match / dti_), int(4 * match / dti_) + 1) * dti_
        gk = np.exp(-0.5 * (kk / match) ** 2); gk /= gk.sum()
        S = np.c_[[np.convolve(np.pad(S[:, c], len(kk) // 2, mode='edge'), gk, 'valid') for c in range(3)]].T
    segs = np.where(np.diff(tr[:, 0]) > 0.5)[0]
    seg_id = np.r_[0, np.cumsum(np.diff(tr[:, 0]) > 0.5)]
    rows = []
    t = tr[0, 0] + 0.5
    while t + L < tr[-1, 0]:
        sel = (tr[:, 0] >= t) & (tr[:, 0] <= t + L)
        if sel.sum() < 4 or seg_id[sel].min() != seg_id[sel].max() or tr[sel, 0].max() - tr[sel, 0].min() < 0.8 * L:
            t += stride; continue
        tt = tr[sel, 0]; Pv = tr[sel, 1:4]
        ok = (tt > gt[0, 0]) & (tt < gt[-1, 0])
        if ok.sum() < 4: t += stride; continue
        Pg = np.c_[[np.interp(tt[ok], gt[:, 0], gt[:, c]) for c in (1, 2, 3)]].T
        if np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum() < 0.5: t += stride; continue
        R, tx, s_gt = umeyama_alignment(Pv[ok], Pg, with_scale=True)
        tau = tt - tt[0]; n = len(tt)
        if ori == 'gt':     # GT orientation (rotated into the visual world by the window Sim3 rotation R: p_g = s R p_v + t)
            j = (ti >= tt[0] - 0.01) & (ti <= tt[-1] + 0.01)
            dd = d['_dgt']
            Rg = A.slerp_R(dd, ti[j] - gyfit['tau']) @ gyfit['R_bI']
            aw = np.einsum('ij,njk,nk->ni', R.T, Rg, f[j])
            Sw = dint(aw, ti[j])
            I = np.c_[[np.interp(tt, ti[j], Sw[:, c]) for c in range(3)]].T
        else:
            I = np.c_[[np.interp(tt, ti, S[:, c]) for c in range(3)]].T
        I = I - I[0]
        cols = np.zeros((3 * n, 10)); y = I.reshape(-1)
        for c in range(3):
            cols[c::3, 0] = Pv[:, c] - Pv[0, c]       # s * (P - P0): P0 is absorbed by subtracting the first sample
            cols[c::3, 1 + c] = -tau                   # v0
            cols[c::3, 4 + c] = -0.5 * tau ** 2        # g
        if gpin:     # gravity vector pinned: direction = minus the mean world-frame specific force over +-gpin/2 s around the window centre (visual orientation), norm 9.81
            tc = 0.5 * (tt[0] + tt[-1]); m = (ti > tc - gpin / 2) & (ti < tc + gpin / 2)
            mv = a_w[m].mean(0); gv = -mv / np.linalg.norm(mv) * 9.81
            yy = y + 0.5 * (tau ** 2)[:, None].repeat(3, 1).reshape(-1) * np.tile(gv, n)
            cc = cols[:, :4].copy()
            th4, *_ = np.linalg.lstsq(cc, yy, rcond=None)
            th = np.r_[th4, gv]; res = yy - cc @ th4
        else:
            th, *_ = np.linalg.lstsq(cols[:, :7], y, rcond=None)
            res = y - cols[:, :7] @ th
        rows.append(dict(t0=float(t), s_est=float(th[0]), s_gt=float(s_gt), ratio=float(th[0] / s_gt), g=float(np.linalg.norm(th[4:7])), res=float(np.sqrt((res ** 2).mean())), path=float(np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum())))
        t += stride
    return rows


def summ(rows):
    r = np.array([x['ratio'] for x in rows])
    return dict(n=len(r), med=float(np.median(r)), q25=float(np.percentile(r, 25)), q75=float(np.percentile(r, 75)), g_med=float(np.median([x['g'] for x in rows])))


if __name__ == '__main__':
    seq, tk = sys.argv[1], sys.argv[2]
    d = load_ds(seq, tk)
    for lab, kw in (('free g', {}), ('smooth 0.3s (unmatched)', dict(smooth=0.3)), ('matched 0.3s', dict(match=0.3)), ('matched 0.6s', dict(match=0.6)), ('matched 1.0s', dict(match=1.0))):
        for L in (3.0, 4.0, 8.0, 12.0):
            s = summ(run(seq, tk, L, d=d, **kw))
            print(f"{seq}_{tk} [{lab}] L={L:g}: n={s['n']} scale ratio med {s['med']:.3f} IQR [{s['q25']:.2f},{s['q75']:.2f}]")
