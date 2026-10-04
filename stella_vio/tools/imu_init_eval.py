#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""Evaluate stella_vio/c/sv_imu_init (check_sv_imu_init driver) on real data: metric scale and gravity recovered from an up-to-scale mono
trajectory + raw IMU, per window length. Needs only IMU csv, a visual trajectory, and position GT (no images).

usage: imu_init_eval.py [dataset ...]      (default: all available)    results -> runs/stella_vio/imu/init_<dataset>.{csv,json}, init_summary.md
Scale truth  = Sim3 (benchmark.umeyama_alignment, with_scale=True) of the visual camera centres in the window to the GT positions.
Gravity truth = EuRoC: GT orientation (R_WV from GT camera orientation vs visual orientation, g_V = R_WV^T g_W).
                phones: reference = -mean over the window of R_VB(t) f(t) (accelerometer mean rotated by the VISUAL orientation; exact when the mean
                world acceleration vanishes, error ~ |mean a|/g, a few tenths of a degree for walking windows >= 4 s). Independent of the estimator.
"""
import json, os, re, subprocess, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from benchmark import umeyama_alignment  # noqa: E402

BIN = ROOT / 'runs/stella_vio/imu/bin/check_sv_imu_init'
OUT = ROOT / 'runs/stella_vio/imu'
G = ROOT / 'external/gnss'
RC = ROOT / 'tools/gnss_harness/robust_cfg'
OFFS = json.loads((ROOT / 'tools/gnss_harness/phone_offsets.json').read_text())
LENS = [2, 3, 4, 6, 8, 12, 16]


def q2R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_tum(p):
    rows = []
    for l in Path(p).read_text().splitlines():
        v = l.replace(',', ' ').split()
        if not v or v[0][0] == '#':
            continue
        try:
            rows.append([float(x) for x in v[:8]])
        except ValueError:
            pass
    a = np.array(rows)
    if len(a) and a[0, 0] > 1e12:
        a[:, 0] *= 1e-9
    return a[np.argsort(a[:, 0])]


def okvis_tsc(cfg):
    m = re.search(r'T_SC:[^\[]*\[([^\]]+)\]', Path(cfg).read_text())
    T = np.array([float(x) for x in re.sub(r'#[^\n]*', '', m.group(1)).replace('\n', ' ').split(',') if x.strip()]).reshape(4, 4)
    return T


def euroc_tbs(yaml):
    m = re.search(r'data:\s*\[([^\]]+)\]', Path(yaml).read_text())
    return np.array([float(x) for x in re.sub(r'#[^\n]*', '', m.group(1)).replace('\n', ' ').split(',') if x.strip()]).reshape(4, 4)


def pvt_gt(seq):
    g = np.loadtxt(Path(seq) / 'gt_pvt.csv', delimiter=',')
    g = g[g[:, 7] <= 0.05]
    return np.c_[g[:, 0], g[:, 1:4]]


def calibrated_imu(seq, imu):
    """accelerometer rescaled so that the median specific-force norm in low-rotation samples is 9.81 (the Mobile-GVIO phones read 5-7% high);
    written to runs/stella_vio/imu/imu_<seq>_cal.csv"""
    out = OUT / f'imu_{seq}_cal.csv'
    a = np.loadtxt(imu, delimiter=',', comments='#')
    n = np.linalg.norm(a[:, 4:7], axis=1)
    lo = np.linalg.norm(a[:, 1:4], axis=1) < 0.1
    k = 9.81 / np.median(n[lo] if lo.sum() > 100 else n)
    a[:, 4:7] *= k
    with open(out, 'w') as f:
        f.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
        for r in a:
            f.write('%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n' % (r[0], *r[1:]))
    print(f'  {seq}: accelerometer scale k = {k:.4f}')
    return out


def fitted_variant(seq, tk, imu, traj, gtf, dt0, ph):
    """camera-IMU time offset (and rotation) fitted from the visual trajectory + gyro (ext_fit.py), GT clock offset re-estimated by Sim3 residual."""
    import ext_fit as X
    T = okvis_tsc(RC / seq / 'okvis_default.yaml')
    im = np.loadtxt(imu, delimiter=',', comments='#'); im[:, 0] *= 1e-9
    tr = read_tum(traj)
    f = X.fit(im, tr, T[:3, :3], scan=np.arange(-0.5, 0.5001, 0.01) if seq[:3] == 'adv' else np.arange(-0.06, 0.0601, 0.005))
    T2 = T.copy(); T2[:3, :3] = f['R_CB'].T
    tr2 = tr.copy(); tr2[:, 0] += f['toff']            # camera stamps -> IMU clock
    out = OUT / f'traj_{seq}_{tk}_fit.txt'
    np.savetxt(out, tr2, fmt='%.6f')
    g = read_tum(gtf)
    best = None
    seg = tr2[(tr2[:, 0] < tr2[0, 0] + 60)]
    for d in np.arange(dt0 - 0.6, dt0 + 0.6001, 0.02):
        t = seg[:, 0]; ok = (t > g[0, 0] + d) & (t < g[-1, 0] + d)
        if ok.sum() < 50:
            continue
        P = np.c_[[np.interp(t[ok] - d, g[:, 0], g[:, k]) for k in (1, 2, 3)]].T
        R, tt, sc = umeyama_alignment(seg[ok, 1:4], P, with_scale=True)
        r = np.sqrt(((sc * (R @ seg[ok, 1:4].T).T + tt - P) ** 2).sum(1).mean()) / max(sc, 1e-9)
        if best is None or r < best[0]:
            best = (r, d)
    print(f'  {seq}_{tk}_fit: toff {f["toff"]*1e3:+.0f} ms, extrinsic change {f["rot_diff_deg"]:.1f} deg, GT dt {dt0:+.3f} -> {best[1]:+.3f}')
    return dict(ph, imu=imu, traj=out, T_BC=T2, gt=('pos', gtf, best[1]))


def datasets():
    D = {}
    mh = ROOT / 'external/vio/data'
    Tbs = euroc_tbs(mh / 'MH_01_easy/mav0/cam0/sensor.yaml')
    for s in ['MH_01_easy', 'MH_03_medium', 'V1_02_medium', 'V2_02_medium']:
        imu = mh / s / 'mav0/imu0/data.csv'
        if not imu.exists():
            continue
        D['euroc_' + s[:5]] = dict(imu=imu, traj=ROOT / f'runs/vio_compare/stella_mono/{s}/trajectory.tum', T_BC=Tbs, noise=(1.7e-4, 2e-3),
                                   gt=('euroc', ROOT / f'runs/vio_compare/gt/{s}_cam0.tum'), kf_dt=0.25)
    ph = dict(noise=(1e-2, 1e-1), kf_dt=0.5)
    gts = {'advio15': (G / 'rob/advio15/gt.tum', OFFS['advio15']), 'advio20': (G / 'rob/advio20/gt.tum', OFFS['advio20']),
           'outdoor1': (G / 'seq/outdoor1/gt.tum', -292.887), 'outdoor2': (G / 'rob/outdoor2/gt.tum', OFFS['outdoor2']),
           'indoor1': (G / 'rob/indoor1/gt.tum', OFFS['indoor1']), 'indoor2': (G / 'rob/indoor2/gt.tum', OFFS['indoor2'])}
    trk = {'stella': 'stella_up', 'orb3': 'orb3_mono'}
    for seq, (gtf, dt) in gts.items():
        for tk, run in trk.items():
            cands = [ROOT / f'runs/gnss_compare/{sub}/{seq}_{r}/traj.txt' for sub in ('robustness', 'phone_more') for r in ([run, run + '_lowfast'] if tk == 'stella' else [run])]
            tj = next((c for c in cands if c.exists() and len(c.read_text().splitlines()) > 300), None)
            if tj is None:
                continue
            for cal in ([False, True] if seq[:3] in ('out', 'ind') else [False]):
                imu = G / f'rob/{seq}/imu0/data.csv'
                if cal:
                    imu = calibrated_imu(seq, imu)
                D[f'{seq}_{tk}' + ('_cal' if cal else '')] = dict(ph, imu=imu, traj=tj, T_BC=okvis_tsc(RC / seq / 'okvis_default.yaml'), gt=('pos', gtf, dt))
                if cal or seq[:3] == 'adv':
                    D[f'{seq}_{tk}_fit'] = fitted_variant(seq, tk, imu, tj, gtf, dt, ph)
    if (ROOT / 'runs/gnss_compare/robustness/complex_orb3_mono/traj.txt').exists():
        D['complex_orb3'] = dict(imu=G / 'rob/complex/imu0/data.csv', traj=ROOT / 'runs/gnss_compare/robustness/complex_orb3_mono/traj.txt',
                                 T_BC=okvis_tsc(RC / 'complex/okvis_default.yaml'), noise=(8e-3, 4e-2), kf_dt=0.5, gt=('pvt', G / 'seq/complex'))
    return D


def segments(t, gap=0.5):
    br = np.where(np.diff(t) > gap)[0]
    s = np.r_[0, br + 1]; e = np.r_[br, len(t) - 1]
    return [(t[a], t[b]) for a, b in zip(s, e)]


def run_c(ds, d, starts):
    ext = d['T_BC']
    ef = OUT / f'ext_{ds}.txt'
    ef.write_text(' '.join(f'{x:.9f}' for x in list(ext[:3, :3].ravel()) + list(ext[:3, 3])) + '\n')
    cmd = [str(BIN), str(d['imu']), str(d['traj']), str(ef), str(d['kf_dt']), str(d['noise'][0]), str(d['noise'][1]),
           '--starts', ','.join(f'{x:.3f}' for x in starts), '--lens', ','.join(str(l) for l in LENS)]
    import os
    cmd += os.environ.get('IMU_INIT_EXTRA', '').split()
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    recs = []
    for l in out.splitlines():
        r = dict(kv.split('=') for kv in l.split())
        recs.append(r)
    return recs


def window_truth(d, tr, imu, t0, L, g_ref_mode):
    """scale truth and gravity reference for [t0, t0+L]"""
    sel = (tr[:, 0] >= t0) & (tr[:, 0] <= t0 + L)
    if sel.sum() < 5:
        return None
    t = tr[sel, 0]
    out = {}
    gt = d['_gt']
    if gt['kind'] == 'euroc':
        gtd = gt['data']
    else:
        gtd = gt['data']
    ok = (t >= gtd[0, 0]) & (t <= gtd[-1, 0])
    if ok.sum() >= 5:
        P = np.c_[[np.interp(t[ok], gtd[:, 0], gtd[:, k]) for k in (1, 2, 3)]].T
        # require GT continuity (no >0.5 s hole) around the window
        i = np.searchsorted(gtd[:, 0], [t[ok][0], t[ok][-1]])
        if np.max(np.diff(gtd[i[0]:i[1] + 1, 0])) < 0.5 if i[1] > i[0] else False:
            R, tt, s = umeyama_alignment(tr[sel, 1:4][ok], P, with_scale=True)
            out['scale_gt'] = s
            res = np.linalg.norm((s * (R @ tr[sel, 1:4][ok].T).T + tt) - P, axis=1)
            out['align_rms'] = float(np.sqrt((res ** 2).mean()))
            out['path_len_gt'] = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    if gt['kind'] == 'euroc':
        gd = gt['data']
        Rs = []
        for tt_, row in zip(t, tr[sel]):
            j = np.searchsorted(gd[:, 0], tt_)
            if 0 < j < len(gd):
                Rs.append(q2R(gd[j, 4:8]) @ q2R(row[4:8]).T)
        if Rs:
            M = np.sum(Rs, axis=0)
            U, _, Vt = np.linalg.svd(M)
            Rwv = U @ Vt
            out['g_true'] = Rwv.T @ np.array([0, 0, -1.0])
    # accelerometer-mean reference
    m = (imu[:, 0] >= t0) & (imu[:, 0] <= t0 + L)
    if m.sum() > 10:
        ti = imu[m, 0]
        idx = np.clip(np.searchsorted(tr[:, 0], ti), 0, len(tr) - 1)
        Rb = np.array([q2R(tr[j, 4:8]) @ d['_R_CB'] for j in idx])
        a = np.einsum('nij,nj->ni', Rb, imu[m, 4:7]).mean(axis=0)
        out['g_acc'] = -a / np.linalg.norm(a)
    return out


def ang(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def evaluate(ds, d, stride=8.0, max_starts=40):
    import os
    if os.environ.get('IMU_NOISE'):
        d['noise'] = tuple(float(x) for x in os.environ['IMU_NOISE'].split(','))
    tr = read_tum(d['traj'])
    imu = np.loadtxt(d['imu'], delimiter=',', comments='#')
    imu = np.c_[imu[:, 0] * 1e-9, imu[:, 1:]]
    d['_R_CB'] = d['T_BC'][:3, :3].T
    kind = d['gt'][0]
    if kind == 'euroc':
        d['_gt'] = dict(kind='euroc', data=read_tum(d['gt'][1]))
    elif kind == 'pos':
        g = read_tum(d['gt'][1]); g[:, 0] += d['gt'][2]
        d['_gt'] = dict(kind='pos', data=g)
    else:
        g = pvt_gt(d['gt'][1]); d['_gt'] = dict(kind='pos', data=g)
    starts = []
    for a, b in segments(tr[:, 0]):
        x = a + 1.0
        while x + LENS[0] < b:
            starts.append(x)
            x += stride
    if len(starts) > max_starts:
        starts = list(np.array(starts)[np.linspace(0, len(starts) - 1, max_starts).astype(int)])
    recs = run_c(ds, d, starts)
    rows = []
    for r in recs:
        if r.get('status') in ('gap', 'fail'):
            continue
        t0, L = float(r['start']), float(r['len'])
        tru = window_truth(d, tr, imu, t0, L, None)
        if not tru or 'scale_gt' not in tru:
            continue
        gE = np.array([float(r['gx']), float(r['gy']), float(r['gz'])])
        row = dict(start=t0, len=L, status=r['status'], n_kf=int(r['n_kf']), scale=float(r['scale']), scale_lin=float(r['scale_lin']), scale_gt=tru['scale_gt'],
                   ratio=float(r['scale']) / tru['scale_gt'], sigma_logS=float(r['sigma_logS']), sigma_g_deg=float(r['sigma_g_deg']),
                   align_rms=tru['align_rms'], path_len_gt=tru['path_len_gt'], rot_before=float(r['rot_before']), rot_after=float(r['rot_after']),
                   bg=float(np.linalg.norm([float(r['bgx']), float(r['bgy']), float(r['bgz'])])),
                   ba=float(np.linalg.norm([float(r['bax']), float(r['bay']), float(r['baz'])])), chi2dof=float(r['chi2dof']), rc=int(r['rc']))
        if 'g_true' in tru:
            row['grav_err_gt'] = ang(gE, tru['g_true'])
        if 'g_acc' in tru:
            row['grav_err_acc'] = ang(gE, tru['g_acc'])
        if 'g_true' in tru and 'g_acc' in tru:
            row['accref_vs_gt'] = ang(tru['g_acc'], tru['g_true'])
        rows.append(row)
    return rows


def summarize(ds, rows):
    lines = []
    for L in LENS:
        R = [r for r in rows if r['len'] == L and r['path_len_gt'] >= 0.5]   # windows with < 0.5 m of motion: GT scale undefined
        if not R:
            continue
        rat = np.array([r['ratio'] for r in R])
        lr = np.abs(np.log(np.clip(rat, 1e-3, 1e3)))
        acc = [r for r in R if r['status'] == 'ok']
        ge = [r.get('grav_err_gt', r.get('grav_err_acc')) for r in R if ('grav_err_gt' in r or 'grav_err_acc' in r)]
        gea = [r.get('grav_err_gt', r.get('grav_err_acc')) for r in acc if ('grav_err_gt' in r or 'grav_err_acc' in r)]
        lra = np.abs(np.log(np.clip(np.array([r['ratio'] for r in acc]), 1e-3, 1e3))) if acc else np.array([])
        lines.append(dict(len=L, n=len(R), med_ratio=float(np.median(rat)), med_abs_err_pct=float(100 * (np.exp(np.median(lr)) - 1)),
                          p90_abs_err_pct=float(100 * (np.exp(np.percentile(lr, 90)) - 1)),
                          within20=float(np.mean(lr < np.log(1.2))), n_ok=len(acc), n_static=len([r for r in rows if r['len'] == L and r['path_len_gt'] < 0.5]), n_static_accepted=len([r for r in rows if r['len'] == L and r['path_len_gt'] < 0.5 and r['status'] == 'ok']), within20_ok=float(np.mean(lra < np.log(1.2))) if len(acc) else None,
                          med_abs_err_pct_ok=float(100 * (np.exp(np.median(lra)) - 1)) if len(acc) else None,
                          grav_med=float(np.median(ge)) if ge else None, grav_p90=float(np.percentile(ge, 90)) if ge else None,
                          grav_med_ok=float(np.median(gea)) if gea else None,
                          med_sigma_logS=float(np.median([r['sigma_logS'] for r in R])), med_path=float(np.median([r['path_len_gt'] for r in R]))))
    return lines


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    D = datasets()
    names = sys.argv[1:] or list(D)
    md = []
    for ds in names:
        d = D[ds]
        rows = evaluate(ds, d)
        (OUT / f'init_{ds}.json').write_text(json.dumps(rows, indent=0))
        S = summarize(ds, rows)
        (OUT / f'init_{ds}_summary.json').write_text(json.dumps(S, indent=1))
        md.append(f'### {ds}  ({len(rows)} window results)\n')
        md.append('| win [s] | n | median scale err [%] | p90 [%] | within 20% | gate-accepted | acc. within 20% | grav err med/p90 [deg] | med sigma_logS | med GT path [m] | static windows (accepted) |')
        md.append('|---|---|---|---|---|---|---|---|---|---|---|')
        for s in S:
            f = lambda x, p=1: 'n/a' if x is None else f'{x:.{p}f}'
            md.append(f"| {s['len']} | {s['n']} | {s['med_abs_err_pct']:.1f} | {s['p90_abs_err_pct']:.1f} | {100*s['within20']:.0f}% | {s['n_ok']} | "
                      f"{'n/a' if s['within20_ok'] is None else format(100*s['within20_ok'], '.0f')+'%'} | {f(s['grav_med'])} / {f(s['grav_p90'])} | {s['med_sigma_logS']:.2f} | {s['med_path']:.1f} | {s['n_static']} ({s['n_static_accepted']}) |")
        md.append('')
        print('\n'.join(md[-(len(S) + 4):]))
    if not os.environ.get('IMU_TAG'):
        (OUT / 'init_summary.md').write_text('\n'.join(md))


if __name__ == '__main__':
    main()
