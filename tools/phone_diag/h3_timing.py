# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H3 timestamps.  (a) IMU / camera sample-interval statistics, duplicates, PSD tail (resampling / low-pass signature).  (b) IMU-vs-GT clock offset per 30 s window
(gyro integral vs GT angular velocity, rotation fixed at the global fit).  (c) camera-vs-IMU offset per 30 s window (visual angular velocity of the ORB-SLAM3 mono
trajectory vs gyro, extrinsic fixed at the shipped calibration).   usage: h3_timing.py [seq ...]  -> runs/phone_diag/h3_timing.json"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C
import h1_accel as A

CAM = {'outdoor1': C.G / 'seq/outdoor1/cam0/data.csv'}
TRAJ = {s: C.ROOT / f'runs/gnss_compare/{sub}/{s}_orb3_mono/traj.txt' for s, sub in
        [('outdoor1', 'robustness'), ('outdoor2', 'phone_more'), ('indoor1', 'phone_more'), ('indoor2', 'phone_more'), ('advio15', 'phone_more'), ('advio20', 'phone_more')]}


def imu_stats(d):
    ti = d['ti']; dt = np.diff(ti)
    dup = np.all(np.diff(d['f'], axis=0) == 0, axis=1) & np.all(np.diff(d['w'], axis=0) == 0, axis=1)
    out = dict(rate_hz=float(1 / np.median(dt)), dt_std_ms=float(1e3 * dt.std()), dt_max_ms=float(1e3 * dt.max()), n_gap=int((dt > 1.5 * np.median(dt)).sum()), dup_frac=float(dup.mean()))
    # PSD tail: accel noise floor flatness: power in 0.7..0.9 Nyquist vs 0.1..0.3 Nyquist of the high-passed signal
    for nm, x in (('acc', d['f']), ('gyr', d['w'])):
        fs = 1 / np.median(dt); n = 256
        P = 0
        for a in range(0, len(x) - n, n):
            P = P + (np.abs(np.fft.rfft((x[a:a + n] - x[a:a + n].mean(0)) * np.hanning(n)[:, None], axis=0)) ** 2).sum(1)
        fr = np.fft.rfftfreq(n, 1 / fs); ny = fs / 2
        hi = P[(fr > 0.7 * ny) & (fr < 0.95 * ny)].mean(); mid = P[(fr > 0.2 * ny) & (fr < 0.4 * ny)].mean(); lo = P[(fr > 2) & (fr < 0.1 * ny)].mean()
        out[f'{nm}_psd_hi_over_mid'] = float(hi / mid); out[f'{nm}_psd_mid_over_low'] = float(mid / lo)
    return out


def gt_offset_windows(d, gy, win=30.0):
    wb = A.gt_omega_b(d)
    res = []
    t = d['tg'][0]
    while t + win < d['tg'][-1]:
        best = None
        for tau in np.arange(-0.1, 0.1001, 0.0025):
            ok, wg = A.gyro_pairs(d, tau)
            tm = 0.5 * (d['tg'][:-1] + d['tg'][1:])[ok]
            m = (tm >= t) & (tm < t + win) & (np.linalg.norm(wb[ok], axis=1) > 0.2)
            if m.sum() < 30: best = None; break
            r = np.sqrt(((wb[ok][m] - wg[m] @ gy['R_bI'].T) ** 2).sum(1).mean())
            if best is None or r < best[0]: best = (r, tau)
        if best: res.append((t, best[1], best[0]))
        t += win
    return res


def cam_offset_windows(name, d, win=30.0):
    import re
    tr = np.loadtxt(TRAJ[name]); tr = tr[np.argsort(tr[:, 0])]
    if tr[0, 0] > 1e12: tr[:, 0] *= 1e-9
    tr[:, 0] -= d['t0']
    yaml = (C.ROOT / f'tools/gnss_harness/robust_cfg/{name}/okvis_default.yaml').read_text()
    T = np.array([float(x) for x in re.sub(r'#[^\n]*', '', re.search(r'T_SC:[^\[]*\[([^\]]+)\]', yaml).group(1)).replace('\n', ' ').split(',') if x.strip()]).reshape(4, 4)
    R_cB = T[:3, :3].T
    Rs = C.q2R_batch(tr[:, 4:8]); dtv = np.diff(tr[:, 0])
    okp = (dtv > 0) & (dtv < 0.2)
    wc = np.array([C.rotvec(Rs[i].T @ Rs[i + 1]) / dtv[i] if okp[i] else np.full(3, np.nan) for i in range(len(dtv))])
    ti, gi = d['ti'], d['w']
    cum = np.vstack([np.zeros(3), np.cumsum(0.5 * (gi[1:] + gi[:-1]) * np.diff(ti)[:, None], axis=0)])
    I = lambda t: np.c_[[np.interp(t, ti, cum[:, k]) for k in range(3)]].T
    scan = np.arange(-0.5, 0.5001, 0.01) if name.startswith('adv') else np.arange(-0.1, 0.1001, 0.005)
    res = []; t = tr[0, 0]
    while t + win < tr[-1, 0]:
        best = None
        tm = 0.5 * (tr[:-1, 0] + tr[1:, 0]); sel = okp & (tm >= t) & (tm < t + win) & (np.linalg.norm(np.nan_to_num(wc), axis=1) > 0.2)
        if sel.sum() < 40: t += win; continue
        for tau in scan:
            wg = (I(tr[1:, 0][sel] + tau) - I(tr[:-1, 0][sel] + tau)) / dtv[sel][:, None]
            r = np.sqrt(((wc[sel] - wg @ R_cB.T) ** 2).sum(1).mean())
            if best is None or r < best[0]: best = (r, tau)
        res.append((t, best[1], best[0], int(sel.sum())))
        t += win
    return res


if __name__ == '__main__':
    names = sys.argv[1:] or ['outdoor1', 'outdoor2', 'indoor1', 'indoor2', 'advio15', 'advio20', 'euroc_MH01']
    out = {}
    for n in names:
        d = C.load(n); gy = A.fit_gyro(d)
        o = dict(imu=imu_stats(d))
        g = gt_offset_windows(d, gy)
        o['imu_vs_gt_ms'] = [[round(a, 1), round(1e3 * b, 1), round(c, 3)] for a, b, c in g]
        if n in TRAJ:
            c = cam_offset_windows(n, d)
            o['cam_vs_imu_ms'] = [[round(a, 1), round(1e3 * b, 1), round(r, 3), k] for a, b, r, k in c]
        out[n] = o
        gv = np.array([x[1] for x in g]) * 1e3
        s = f"{n:11s} imu {o['imu']['rate_hz']:.1f} Hz dt std {o['imu']['dt_std_ms']:.3f} ms max {o['imu']['dt_max_ms']:.1f} ms gaps {o['imu']['n_gap']} dup {o['imu']['dup_frac']:.4f} | acc PSD hi/mid {o['imu']['acc_psd_hi_over_mid']:.2f} gyr {o['imu']['gyr_psd_hi_over_mid']:.2f} | IMU-GT offset per 30 s: median {np.median(gv):+.1f} ms, range [{gv.min():+.1f},{gv.max():+.1f}], slope {np.polyfit(np.arange(len(gv)), gv, 1)[0] if len(gv) > 1 else 0:+.2f} ms/window"
        if 'cam_vs_imu_ms' in o:
            cv = np.array([x[1] for x in o["cam_vs_imu_ms"]])
            s += f" | cam-IMU per 30 s: median {np.median(cv):+.0f} ms, range [{cv.min():+.0f},{cv.max():+.0f}], n={len(cv)}"
        print(s)
    (C.OUT / 'h3_timing.json').write_text(json.dumps(out))
