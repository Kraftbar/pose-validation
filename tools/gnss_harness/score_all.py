#!/usr/bin/env python3
"""Score every GNSS-VIO run under external/gnss/out/ and write runs/gnss_compare/{table.md,table.json,<run>/...}.
Alignment: benchmark.umeyama_alignment (via gnss_eval). 'noalign' = error of the geo-referenced output with no alignment at all.
Run registry is the RUNS list below (id, system, sequence, GNSS input, loader, geo flag)."""
import sys, json, shutil
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from gnss_eval import GT, read_traj, score  # noqa: E402
sys.path.insert(0, str(Path(__file__).parent.parent))
import gnss_loose_fusion as lf  # noqa: E402

G = Path('/home/nybo/github/pose-validation/external/gnss')
OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare')
RSA_OKVIS = [-0.01, -0.03, -0.06]   # antenna in IMU frame (from OKVIS2-X's shipped GVINS-dataset config)

A_, F_ = 6378137.0, 1 / 298.257223563; E2 = F_ * (2 - F_)


def ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon); N = A_ / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.c_[(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon), (N * (1 - E2) + h) * np.sin(lat)]


def lla_to_enu(lla, lla0):
    lat0, lon0 = np.radians(lla0[0]), np.radians(lla0[1])
    R = np.array([[-np.sin(lon0), np.cos(lon0), 0], [-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)],
                  [np.cos(lat0) * np.cos(lon0), np.cos(lat0) * np.sin(lon0), np.sin(lat0)]])
    return (R @ (ecef(lla[:, 0], lla[:, 1], lla[:, 2]) - ecef(*lla0)[0]).T).T


def load_okvis(d, kind):
    f = {'local': 'okvis2-vio-final_trajectory.csv', 'global': 'okvis2-vio-global-final_trajectory.csv', 'causal': 'okvis2-vio_trajectory.csv'}[kind]
    return read_traj(d / f)


def load_gvins_fused(d, lla0, off=18.0262):
    f = np.loadtxt(d / 'fused_lla.csv', delimiter=',')
    return np.c_[f[:, 0] - off, lla_to_enu(f[:, 1:4], lla0)]


def load_gvins_odom(d):
    return np.loadtxt(d / 'odom_local.csv', delimiter=',')


SEQS = {}


def seq_complex():
    S = G / 'seq' / 'complex'
    o = json.load(open(S / 'origin.json'))
    n_fr = len((S / 'cam0' / 'data.csv').read_text().splitlines()) - 1
    return dict(dir=S, lla0=o['lla0'], t0=o['t0'], n_frames=n_fr, duration=435.65, gt=GT.from_pvt(S), geo=True)


def seq_outdoor1():
    S = G / 'seq' / 'outdoor1'
    info = json.load(open(S / 'info.json'))
    return dict(dir=S, lla0=None, t0=info['t0'], n_frames=info['frames'], duration=393.99, gt=GT.from_tum(S / 'gt.tum', dt=-292.887), geo=False)


def loose_mobile(causal, sig=None):
    tr = load_okvis(G / 'out' / 'okvis2x_mobile_nogps_outdoor1', 'local')
    gps = lf.read_gps(G / 'seq' / 'outdoor1' / 'gps0' / 'data.csv')
    if sig: gps[:, 4] = sig; gps[:, 5] = 2 * sig
    return lf.fuse(tr, gps, [0, 0, 0], 1.0, causal)[0]


def run_spec():
    """(id, system, seq, gnss_input, loader(lambda ctx -> traj array), geo, rsa)"""
    R = []
    ok = lambda tag, kind: (lambda c: load_okvis(G / 'out' / tag, kind))
    R += [
        ('okvis2x_mono_vio', 'OKVIS2-X mono (BSD-3)', 'complex', 'none (VIO baseline)', ok('okvis2x_mono_nogps_complex', 'local'), False, RSA_OKVIS),
        ('okvis2x_mono_rtk', 'OKVIS2-X mono', 'complex', 'RTK 10 Hz', ok('okvis2x_mono_rtk_complex', 'global'), True, [0, 0, 0]),
        ('okvis2x_mono_sim', 'OKVIS2-X mono', 'complex', 'simulated SPP-grade 1 Hz (1.5 m h / 3 m v)', ok('okvis2x_mono_sim_complex', 'global'), True, [0, 0, 0]),
        ('okvis2x_mono_rtk_blk', 'OKVIS2-X mono', 'complex', 'RTK, blackout 100-220 s', ok('okvis2x_mono_rtk_blk_complex', 'global'), True, [0, 0, 0]),
        ('okvis2x_mono_sim_blk', 'OKVIS2-X mono', 'complex', 'sim SPP, blackout 100-220 s', ok('okvis2x_mono_sim_blk_complex', 'global'), True, [0, 0, 0]),
        ('gvins_vio', 'GVINS (GPL-3) GNSS off', 'complex', 'none (VIO baseline)', lambda c: load_gvins_odom(G / 'out' / 'gvins_complex_off'), False, RSA_OKVIS),
        ('gvins_gnss', 'GVINS (GPL-3)', 'complex', 'raw pseudorange+Doppler 10 Hz', lambda c: load_gvins_fused(G / 'out' / 'gvins_complex_on', c['lla0']), True, [0, 0, 0]),
    ]
    R += [
        ('mobile_okvis2x_vio', 'OKVIS2-X mono (BSD-3)', 'outdoor1', 'none (VIO baseline)', ok('okvis2x_mobile_nogps_outdoor1', 'local'), False, [0, 0, 0]),
        ('mobile_okvis2x_gnss', 'OKVIS2-X mono', 'outdoor1', 'iPhone GNSS 1 Hz (reported sigma 14 m)', ok('okvis2x_mobile_gps_outdoor1', 'local'), False, [0, 0, 0]),
        ('mobile_loose_batch', 'own loose fusion (batch) on OKVIS2 mono VIO', 'outdoor1', 'iPhone GNSS 1 Hz', (lambda c: loose_mobile(0.0)), False, [0, 0, 0]),
        ('mobile_loose_batch_sig5', 'own loose fusion (batch), GNSS sigma set to 5 m (post-hoc sensitivity)', 'outdoor1', 'iPhone GNSS 1 Hz', (lambda c: loose_mobile(0.0, 5.0)), False, [0, 0, 0]),
        ('mobile_loose_causal30_sig5', 'own loose fusion (causal30), GNSS sigma 5 m (post-hoc)', 'outdoor1', 'iPhone GNSS 1 Hz', (lambda c: loose_mobile(30.0, 5.0)), False, [0, 0, 0]),
        ('mobile_gnss_only', 'iPhone GNSS fixes alone (no vision)', 'outdoor1', 'iPhone GNSS 1 Hz', (lambda c: np.loadtxt(G / 'seq' / 'outdoor1' / 'gnss_enu.txt')), False, [0, 0, 0]),
        ('mobile_loose_causal30', 'own loose fusion (causal30) on OKVIS2 mono VIO', 'outdoor1', 'iPhone GNSS 1 Hz', (lambda c: loose_mobile(30.0)), False, [0, 0, 0]),
    ]
    for v, name in (('rtk', 'RTK 10 Hz'), ('sim', 'simulated SPP-grade 1 Hz'), ('rtk_blk', 'RTK, blackout 100-220 s'), ('sim_blk', 'sim SPP, blackout 100-220 s')):
        for causal, tag in ((0.0, 'batch'), (30.0, 'causal30')):
            R.append((f'loose_{tag}_{v}', f'own loose fusion ({tag}) on OKVIS2 mono VIO', 'complex', name,
                      (lambda c, v=v, causal=causal: loose(c, v, causal)), True, RSA_OKVIS))
    return R


def loose(c, v, causal):
    tr = load_okvis(G / 'out' / 'okvis2x_mono_nogps_complex', 'local')
    gps = lf.read_gps(G / 'seq' / f'complex_{v}' / 'gps0' / 'data.csv')
    out, X, tn = lf.fuse(tr, gps, RSA_OKVIS, 1.0, causal)
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    seqs = {'complex': seq_complex(), 'outdoor1': seq_outdoor1()}
    rows = []
    for rid, system, seq, gin, loader, geo, rsa in run_spec():
        c = seqs[seq]
        try:
            tr = loader(c)
        except Exception as e:  # missing run
            print('skip', rid, e); continue
        t0 = c['t0']
        wins = [(t0 + 100, t0 + 220, 'blk'), (t0 + 220, t0 + 260, 'after'), (t0 + 240, t0 + 330, 'float')]
        r = score(tr, c['gt'], rsa, c['n_frames'], c['duration'], geo=geo, windows=wins)
        d = OUT / rid; d.mkdir(exist_ok=True)
        np.savetxt(d / 'trajectory.tum', tr[:, :8] if tr.shape[1] >= 8 else tr[:, :4], fmt='%.6f')
        rj = {k: v for k, v in r.items() if not k.startswith('_')}
        rj.update(id=rid, system=system, seq=seq, gnss=gin, geo=geo)
        runj = G / 'out'
        rows.append(rj)
        json.dump(rj, open(d / 'metrics.json', 'w'), indent=1)
    json.dump(rows, open(OUT / 'table.json', 'w'), indent=1)
    f = lambda v, n=3: '-' if v is None else f'{v:.{n}f}'
    L = ['| run | system | GNSS input | ATE SE3 (m) | ATE no-align (m) | rate-cov | window blk / after / float (m) |', '|---|---|---|---|---|---|---|']
    for r in rows:
        L.append(f"| {r['id']} | {r['system']} | {r['gnss']} | {f(r.get('ate_se3'))} | {f(r.get('ate_noalign'))} | {r['coverage']*100:.0f}% | "
                 f"{f(r.get('win_blk'),2)} / {f(r.get('win_after'),2)} / {f(r.get('win_float'),2)} |")
    (OUT / 'table.md').write_text('\n'.join(L) + '\n'); print('\n'.join(L))


if __name__ == '__main__':
    main()
