# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""Per-frequency-band SNR of the visual trajectory as a scale regressor: PSD of the GT position (motion signal, windowed Hann, mean/linear removed) vs PSD of the visual error
(Sim3-aligned visual - GT, per 30 s block aligned separately) on the 10 Hz GT grid.  Bands where signal/error > 1 are where a scale estimate can come from.
usage: vis_band_snr.py"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C
import vis_scale as V
from benchmark import umeyama_alignment

BANDS = [(0.05, 0.15), (0.15, 0.4), (0.4, 0.8), (0.8, 1.5), (1.5, 3.0)]


def run(name, tr, gt, block=30.0, step=0.1):
    seg = np.r_[0, np.cumsum(np.diff(tr[:, 0]) > 0.5)]
    nper = int(round(8.0 / step)); w = np.hanning(nper); fr = np.fft.rfftfreq(nper, step)
    Ps = np.zeros(len(fr)); Pe = np.zeros(len(fr)); cnt = 0
    for sid in np.unique(seg):
        ix = np.where(seg == sid)[0]
        if tr[ix[-1], 0] - tr[ix[0], 0] < block: continue
        t = np.arange(tr[ix[0], 0] + 0.2, tr[ix[-1], 0] - 0.2, step)
        ok = (t > gt[0, 0]) & (t < gt[-1, 0]); t = t[ok]
        Pv = np.c_[[np.interp(t, tr[ix, 0], tr[ix, 1 + c]) for c in range(3)]].T
        Pg = np.c_[[np.interp(t, gt[:, 0], gt[:, 1 + c]) for c in range(3)]].T
        nb = int(block / step)
        for a in range(0, len(t) - nb, nb):
            R, tx, s = umeyama_alignment(Pv[a:a + nb], Pg[a:a + nb], with_scale=True)
            Pa = s * (R @ Pv[a:a + nb].T).T + tx; G = Pg[a:a + nb]; E = Pa - G
            for b in range(0, nb - nper + 1, nper // 2):
                g_ = G[b:b + nper]; e_ = E[b:b + nper]
                tt = np.arange(nper)[:, None] / nper
                dt_ = lambda x: x - (x[0] + (x[-1] - x[0]) * tt)           # remove linear trend
                Ps += (np.abs(np.fft.rfft(dt_(g_) * w[:, None], axis=0)) ** 2).sum(1); Pe += (np.abs(np.fft.rfft(dt_(e_) * w[:, None], axis=0)) ** 2).sum(1); cnt += 1
    res = []
    for lo, hi in BANDS:
        m = (fr >= lo) & (fr < hi)
        res.append(float(Ps[m].sum() / Pe[m].sum()))
    print(f'{name:18s} ' + ' | '.join(f'{lo}-{hi}Hz {r:.2f}' for (lo, hi), r in zip(BANDS, res)) + '   (signal/error power)')
    return dict(bands=BANDS, snr=res, blocks=cnt)


if __name__ == '__main__':
    out = {}
    tr = np.loadtxt(C.ROOT / 'runs/vio_compare/stella_mono/MH_01_easy/trajectory.tum'); tr = tr[np.argsort(tr[:, 0])]
    gt = np.loadtxt(C.ROOT / 'runs/vio_compare/gt/MH_01_easy_cam0.tum')
    out['euroc_stella'] = run('euroc_stella', tr, gt)
    for seq, tk in (('outdoor1', 'orb3'), ('outdoor1', 'stella'), ('outdoor2', 'orb3'), ('outdoor2', 'stella'), ('indoor1', 'stella'), ('indoor2', 'stella'), ('indoor2', 'orb3'), ('advio20', 'orb3'), ('advio20', 'stella')):
        d = V.load_ds(seq, tk)
        out[f'{seq}_{tk}'] = run(f'{seq}_{tk}', d['tr'], d['gt'])
    (C.OUT / 'vis_band_snr.json').write_text(json.dumps(out))
