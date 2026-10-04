# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H5: does the scale collapse of a camera+IMU system coincide with low excitation?
Per 20 s bin: local metric scale of each system = (path length of the GT, sampled at 1 s) / (path length of the system trajectory, same epochs)  = metres per system unit; 1 = metric  [frame independent, no alignment],
and excitation measures from GT + IMU: (S) rms of the quadratically detrended 4 s GT displacement [mm]  (= the signal the scale estimator can use),
(a) rms of GT acceleration low-passed at 0.5 s [m/s^2], (v) mean speed, (w) rms yaw rate from the gyro, (nk) fraction of the bin with speed < 0.3 m/s.
Prints bin tables and the rank correlation between log(local scale) and the excitation measures, per system.
usage: h5_excitation.py"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C
import vis_noise as VN

R = C.ROOT / 'runs/gnss_compare'
SYS = {
    'outdoor1': {'xrslam': 'more_systems/outdoor1_xrslam', 'okvis': 'robustness/outdoor1_okvis_default', 'orb3_mi': 'robustness/outdoor1_orb3_mi_ds'},
    'outdoor2': {'xrslam': 'more_systems/outdoor2_xrslam', 'okvis': 'phone_more/outdoor2_okvis_default', 'orb3_mi': 'phone_more/outdoor2_orb3_mi_ds'},
    'advio20': {'xrslam': 'more_systems/advio20_xrslam', 'okvis': 'phone_more/advio20_okvis_default', 'orb3_mi': 'phone_more/advio20_orb3_mi_ds'},
    'indoor1': {'xrslam': 'more_systems/indoor1_xrslam', 'okvis': 'phone_more/indoor1_okvis_default'},
    'indoor2': {'xrslam': 'more_systems/indoor2_xrslam', 'okvis': 'phone_more/indoor2_okvis_default'},
    'advio15': {'xrslam': 'more_systems/advio15_xrslam', 'okvis': 'phone_more/advio15_okvis_default'},
}


def rank(a):
    return np.argsort(np.argsort(a)).astype(float)


def spearman(a, b):
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 5: return float('nan')
    return float(np.corrcoef(rank(a[ok]), rank(b[ok]))[0, 1])


def path_len(tt, P, t0, t1):
    t = np.arange(t0, t1 + 1e-9, 1.0)
    ok = (t >= tt[0]) & (t <= tt[-1])
    if ok.sum() < 0.8 * len(t): return None
    Q = np.c_[[np.interp(t, tt, P[:, c]) for c in range(3)]].T
    # require no hole in the system trajectory
    i = np.searchsorted(tt, [t0, t1]); 
    if i[1] > i[0] and np.max(np.diff(tt[max(i[0] - 1, 0):i[1] + 1])) > 2.5: return None
    return float(np.linalg.norm(np.diff(Q, axis=0), axis=1).sum())


def analyse(seq, bin_s=20.0, verbose=True):
    d = C.load(seq)
    tg = d['tg'] + d['t0']; pg = d['p']
    ti = d['ti'] + d['t0']
    out = {}
    # excitation per bin
    h = float(np.median(np.diff(tg))); k = max(3, int(round(0.5 / h))); bx = np.ones(k) / k
    acc = (pg[2:] - 2 * pg[1:-1] + pg[:-2]) / h ** 2
    acc = np.c_[[np.convolve(acc[:, c], bx, 'same') for c in range(3)]].T; ta = tg[1:-1]
    vel = np.gradient(pg, tg, axis=0); vel = np.c_[[np.convolve(vel[:, c], bx, 'same') for c in range(3)]].T; spd = np.linalg.norm(vel, axis=1)
    # detrended 4 s displacement signal on the GT (as vis_noise), 2 s stride
    sig = []
    t = tg[0] + 0.5
    while t + 4 < tg[-1]:
        sel = (tg >= t) & (tg <= t + 4)
        if sel.sum() > 0.8 * 4 / h and np.max(np.diff(tg[sel])) < 0.35:
            tau = tg[sel] - tg[sel][0]; sig.append((t + 2, np.sqrt((VN.detrend(pg[sel], tau) ** 2).sum(1).mean())))
        t += 2
    sig = np.array(sig)
    t_start = tg[0]
    bins = np.arange(t_start, tg[-1] - bin_s + 1e-9, bin_s)
    rows = []
    for b0 in bins:
        b1 = b0 + bin_s
        m = (ta >= b0) & (ta < b1); sm = (sig[:, 0] >= b0) & (sig[:, 0] < b1); mv = (tg >= b0) & (tg < b1); mi = (ti >= b0) & (ti < b1)
        if m.sum() < 10 or sm.sum() < 2: continue
        gl = path_len(tg, pg, b0, b1)
        if gl is None or gl < 3: continue
        rows.append(dict(t=b0 - d['t0'], S_mm=1e3 * float(np.median(sig[sm, 1])), acc_rms=float(np.sqrt((acc[m] ** 2).sum(1).mean())), speed=float(spd[mv].mean()),
                         w_rms=float(np.sqrt((d['w'][mi] ** 2).sum(1).mean())), slow_frac=float((spd[mv] < 0.3).mean()), gt_len=gl, _b=(b0, b1)))
    for name, rel in SYS[seq].items():
        if not (R / rel / 'traj.txt').exists(): continue
        tr = np.loadtxt(R / rel / 'traj.txt'); tr = tr[np.argsort(tr[:, 0])]
        for r in rows:
            L = path_len(tr[:, 0], tr[:, 1:4], *r['_b'])
            r[f'sc_{name}'] = float('nan') if (L is None or L < 1e-6) else r['gt_len'] / L      # metric scale factor GT/system: 1 = metric
        sc = np.array([r[f'sc_{name}'] for r in rows])
        bad = [r['t'] for r in rows if np.isfinite(r[f'sc_{name}']) and not (0.5 < r[f'sc_{name}'] < 2.0)]
        good = [r['t'] for r in rows if np.isfinite(r[f'sc_{name}']) and (0.5 < r[f'sc_{name}'] < 2.0)]
        Sg = np.array([r['S_mm'] for r in rows])
        out[name] = dict(n=int(np.isfinite(sc).sum()), median_scale=float(np.nanmedian(sc)) if np.isfinite(sc).any() else None, first_bad_t=bad[0] if bad else None, n_good=len(good),
                         S_mm_p10_med_p90=[float(np.percentile(Sg, 10)), float(np.median(Sg)), float(np.percentile(Sg, 90))],
                         **{f'rho_{k}': spearman(np.log(np.clip(sc, 1e-4, None)), np.array([r[k] for r in rows])) for k in ('S_mm', 'acc_rms', 'speed', 'w_rms', 'slow_frac')})
    if verbose:
        names = list(out)
        print(f'== {seq} (bins of {bin_s:.0f} s; local scale = path length ratio system/GT)')
        print('  t[s]  S[mm] acc[m/s2] v[m/s] w[rad/s] slow  ' + ' '.join(f'{n:>8s}' for n in names))
        for r in rows:
            print(f"  {r['t']:5.0f} {r['S_mm']:6.1f} {r['acc_rms']:8.2f} {r['speed']:6.2f} {r['w_rms']:6.2f} {r['slow_frac']:5.2f} " + ' '.join(f"{r[f'sc_{n}']:8.3f}" for n in names))
        for n, o in out.items():
            print(f"  {n}: bins {o['n']}, in [0.5,2]: {o['n_good']}, first bin outside at t={o['first_bad_t']}, median local scale {o['median_scale']:.3g}, S p10/med/p90 {np.round(o['S_mm_p10_med_p90'],1)} mm, Spearman(log scale, S) {o['rho_S_mm']:.2f}, (acc) {o['rho_acc_rms']:.2f}, (speed) {o['rho_speed']:.2f}, (yaw-rate w) {o['rho_w_rms']:.2f}, (slow) {o['rho_slow_frac']:.2f}")
    for r in rows: r.pop('_b')
    return dict(bins=rows, corr=out)


if __name__ == '__main__':
    names = sys.argv[1:] or list(SYS)
    res = {n: analyse(n) for n in names}
    (C.OUT / 'h5_excitation.json').write_text(json.dumps(res, indent=1))
