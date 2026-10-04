# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H1 bandwidth: gain and coherence vs frequency between the GT-derived world acceleration (second difference of GT position, step m*h) and the raw accelerometer
converted to world acceleration with GT orientation (+ gravity removed by the per-window mean, a high-pass) and filtered with the identical triangular kernel.
Gain bounds: lo = Sxy/Sxx (biased low by GT noise), hi = Syy/Re(Sxy) (biased high by accelerometer noise).  usage: h1_spectral.py [seq ...]"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C
import h1_accel as A

BANDS = [(0.1, 0.3), (0.3, 0.7), (0.7, 1.5), (1.5, 2.5), (2.5, 4.0)]


def series(d, gy, m):
    """returns segments of (t, x=GT accel (n,3 world), y=measured tri-filtered world accel) at the GT rate"""
    tg, p = d['tg'], d['p']; h = float(np.median(np.diff(tg))); H = m * h
    ti, f = d['ti'], d['f']; dti = float(np.median(np.diff(ti)))
    Rwb = A.slerp_R(d, ti - gy['tau']); RwI = Rwb @ gy['R_bI']
    aw = np.einsum('nij,nj->ni', RwI, f)
    nh = int(round(H / dti)); s = np.arange(-nh, nh + 1) * dti; wt = np.maximum(H - np.abs(s), 0) / H ** 2 * dti
    cont = C.gt_continuous_mask(tg, 2.5 * h)
    ks = np.arange(m, len(tg) - m)
    ok = np.array([cont[k - m:k + m + 1].all() and abs(tg[k + m] - tg[k - m] - 2 * H) < 0.2 * H for k in ks])
    # contiguous runs
    runs = []; cur = []
    for k, o in zip(ks, ok):
        if o and (not cur or k == cur[-1] + 1): cur.append(k)
        else:
            if len(cur) > 400: runs.append(cur)
            cur = [k] if o else []
    if len(cur) > 400: runs.append(cur)
    segs = []
    for r in runs:
        r = np.array(r)
        x = (p[r + m] - 2 * p[r] + p[r - m]) / H ** 2
        y = np.zeros_like(x)
        for i, k in enumerate(r):
            j0 = int(round((tg[k] - ti[0]) / dti)); idx = np.clip(j0 + np.arange(-nh, nh + 1), 0, len(ti) - 1)
            y[i] = (wt[:, None] * aw[idx]).sum(0)
        segs.append((tg[r], x, y))
    return segs, h


def welch(segs, h, nper=None):
    nper = nper or int(round(6.4 / h)) // 2 * 2
    fr = np.fft.rfftfreq(nper, h)
    Sxx = np.zeros((len(fr), 3)); Syy = np.zeros_like(Sxx); Sxy = np.zeros((len(fr), 3), complex); cnt = 0
    w = np.hanning(nper)
    for t, x, y in segs:
        for a in range(0, len(t) - nper + 1, nper // 2):
            X = np.fft.rfft((x[a:a + nper] - x[a:a + nper].mean(0)) * w[:, None], axis=0)
            Y = np.fft.rfft((y[a:a + nper] - y[a:a + nper].mean(0)) * w[:, None], axis=0)
            Sxx += np.abs(X) ** 2; Syy += np.abs(Y) ** 2; Sxy += X.conj() * Y; cnt += 1
    return fr, Sxx, Syy, Sxy, cnt


def band_table(fr, Sxx, Syy, Sxy, axes_sel):
    out = []
    for lo, hi in BANDS:
        b = (fr >= lo) & (fr < hi)
        sxx = Sxx[b][:, axes_sel].sum(); syy = Syy[b][:, axes_sel].sum(); sxy = Sxy[b][:, axes_sel].sum()
        coh = abs(sxy) ** 2 / (sxx * syy)
        out.append(dict(band=[lo, hi], gain_lo=float(sxy.real / sxx), gain_hi=float(syy / sxy.real) if sxy.real > 0 else None, coh=float(coh),
                        power_gt=float(sxx), power_acc=float(syy)))
    return out


if __name__ == '__main__':
    names = sys.argv[1:] or ['euroc_MH01', 'outdoor1', 'outdoor2', 'indoor1', 'indoor2']
    res = {}
    for n in names:
        d = C.load(n); gy = A.fit_gyro(d)
        # world z axis = direction of mean specific force in the GT world
        Rwb = A.slerp_R(d, d['ti'] - gy['tau']); up = np.einsum('nij,nj->ni', Rwb @ gy['R_bI'], d['f']).mean(0); up /= np.linalg.norm(up)
        res[n] = {}
        h0 = float(np.median(np.diff(d['tg'])))
        for m in sorted({max(1, int(round(0.1 / h0))), max(1, int(round(0.2 / h0)))}):
            segs, h = series(d, gy, m)
            fr, Sxx, Syy, Sxy, cnt = welch(segs, h)
            # project onto vertical (up) and horizontal via rotating the 3 spectra is not valid for cross terms; use per-world-axis selection of the dominant-up axis
            ax = int(np.argmax(abs(up))); hz = [i for i in range(3) if i != ax]
            res[n][f'm{m}'] = dict(n_seg=cnt, vertical=band_table(fr, Sxx, Syy, Sxy, [ax]), horizontal=band_table(fr, Sxx, Syy, Sxy, hz))
            for lab in ('vertical', 'horizontal'):
                print(f"{n:11s} H={m*h:.2f}s {lab:10s} " + ' | '.join(f"{b['band'][0]}-{b['band'][1]}Hz G[{b['gain_lo']:.2f},{b['gain_hi'] if b['gain_hi'] is None else round(b['gain_hi'],2)}] coh {b['coh']:.2f}" for b in res[n][f'm{m}'][lab]))
    (C.OUT / 'h1_spectral.json').write_text(json.dumps(res, indent=1))
