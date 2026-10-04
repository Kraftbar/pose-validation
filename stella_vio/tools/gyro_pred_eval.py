#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""Gyro-aided rotation prediction vs visual (or GT) rotation, per sequence. Driver: runs/stella_vio/imu/bin/sv_imu_gyro_run.
usage: gyro_pred_eval.py [case ...]      results: runs/stella_vio/imu/gyro_pred.md / .json
Per consecutive trajectory pair: error (deg) of (i) identity prediction (= visual rotation magnitude), (ii) constant-angular-velocity prediction from the
previous visual pair, (iii) gyro prediction (bias from a visual fit over the first continuous <= 60 s, i.e. what an initialiser would have).
Also reports the gyro error with the camera-IMU time offset scanned (-60..+60 ms) to show the timing sensitivity.
"""
import json, subprocess, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import imu_init_eval as E  # noqa: E402

BIN = ROOT / 'runs/stella_vio/imu/bin/sv_imu_gyro_run'
OUT = ROOT / 'runs/stella_vio/imu'
G = ROOT / 'external/gnss'
CT0 = 1610581021.787963   # GVINS complex_environment sequence origin (seconds), burst at +34 s


def write_ext(name, T):
    p = OUT / f'ext_{name}.txt'
    p.write_text(' '.join(f'{x:.9f}' for x in list(T[:3, :3].ravel()) + list(T[:3, 3])) + '\n')
    return p


def gt_at_cam_times(gt, camcsv, out):
    cam = np.loadtxt(camcsv, delimiter=',', comments='#', usecols=0) * 1e-9
    g = E.read_tum(gt)
    rows = []
    for t in cam:
        j = np.searchsorted(g[:, 0], t)
        if 0 < j < len(g):
            rows.append([t] + list(g[j if abs(g[j, 0] - t) < abs(g[j - 1, 0] - t) else j - 1, 1:8]))
    np.savetxt(out, np.array(rows), fmt='%.9f')
    return out


def run(imu, traj, ext, fit=True, toff=0.0, bg=None, max_gap=0.3):
    cmd = [str(BIN), str(imu), str(traj), str(ext), '--max-gap', str(max_gap), '--toff', str(toff)]
    cmd += ['--fit-bg'] if fit else (['--bg', ','.join(map(str, bg))] if bg is not None else [])
    r = subprocess.run(cmd, capture_output=True, text=True, check=True)
    a = np.array([[float(x) for x in l.split()] for l in r.stdout.splitlines()]) if r.stdout.strip() else np.zeros((0, 6))
    return a, r.stderr.strip()


def stats(a, sel=None):
    if sel is not None:
        a = a[sel(a)]
    if len(a) < 3:
        return None
    ok = ~np.isnan(a[:, 5])
    q = lambda x: [float(np.median(x)), float(np.percentile(x, 95)), float(np.max(x))]
    return dict(n=int(len(a)), ang_vis=q(a[:, 3]), gyro=q(a[:, 4]), cv=q(a[ok, 5]) if ok.sum() > 2 else None, w_med=float(np.median(a[:, 2])), w_max=float(a[:, 2].max()))


def cases():
    C = {}
    mh = ROOT / 'external/vio/data'
    Tbs = E.euroc_tbs(mh / 'MH_01_easy/mav0/cam0/sensor.yaml')
    for s in ['MH_01_easy', 'V1_02_medium']:
        d = mh / s / 'mav0'
        if not (d / 'imu0/data.csv').exists():
            continue
        gtc = gt_at_cam_times(ROOT / f'runs/vio_compare/gt/{s}_cam0.tum', d / 'cam0/data.csv', OUT / f'gt_cam_{s}.tum') if (d / 'cam0/data.csv').exists() else None
        C[f'euroc_{s[:5]}_GT'] = dict(imu=d / 'imu0/data.csv', traj=gtc or ROOT / f'runs/vio_compare/gt/{s}_cam0.tum', T=Tbs)
        C[f'euroc_{s[:5]}_stella'] = dict(imu=d / 'imu0/data.csv', traj=ROOT / f'runs/vio_compare/stella_mono/{s}/trajectory.tum', T=Tbs)
    D = E.datasets()
    cx = E.okvis_tsc(E.RC / 'complex/okvis_default.yaml')
    for nm, p in [('orb3', 'complex_orb3_mono'), ('stella_up', 'complex_stella_up'), ('stella_port', 'complex_stella_port')]:
        C[f'complex_{nm}'] = dict(imu=G / 'rob/complex/imu0/data.csv', traj=ROOT / f'runs/gnss_compare/robustness/{p}/traj.txt', T=cx, burst=(CT0 + 33.0, CT0 + 37.0))
    for k, d in D.items():
        if k.startswith(('advio', 'outdoor', 'indoor')) and not k.endswith('_cal'):
            C['phone_' + k] = dict(imu=d['imu'], traj=d['traj'], T=d['T_BC'])
    return C


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    C = cases()
    names = sys.argv[1:] or list(C)
    res = {}
    md = ['| case | pairs | median frame rotation [deg] | p95 | identity median / p95 [deg] (=rotation) | const-velocity median / p95 | gyro median / p95 / max | fast frames (|w|>1 rad/s) n, identity p95, gyro p95 | bg fit |', '|---|---|---|---|---|---|---|---|---|']
    for nm in names:
        c = C[nm]
        ext = write_ext(nm, c['T'])
        a, info = run(c['imu'], c['traj'], ext)
        if not len(a):
            continue
        s_all = stats(a)
        s_fast = stats(a, lambda x: x[:, 2] > 1.0)
        res[nm] = dict(all=s_all, fast=s_fast, info=info)
        f = lambda v: 'n/a' if v is None else f'{v[0]:.2f} / {v[1]:.2f}'
        md.append(f"| {nm} | {s_all['n']} | {np.median(a[:,3]):.2f} | {np.percentile(a[:,3],95):.2f} | {f(s_all['ang_vis'][1:2] and [s_all['ang_vis'][0], s_all['ang_vis'][1]])} | "
                  f"{f(s_all['cv'])} | {s_all['gyro'][0]:.3f} / {s_all['gyro'][1]:.3f} / {s_all['gyro'][2]:.3f} | "
                  f"{('%d, %.2f, %.3f' % (s_fast['n'], s_fast['ang_vis'][1], s_fast['gyro'][1])) if s_fast else 'n/a'} | {info.split('(')[0].replace('fit bg = ','')[:40] if info else '-'} |")
        # timing scan
        scan = {}
        for to in (-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06):
            b, _ = run(c['imu'], c['traj'], ext, toff=to)
            scan[f'{to:+.2f}'] = float(np.median(b[:, 4])) if len(b) else None
        res[nm]['toff_scan_median_err_deg'] = scan
        if 'burst' in c:
            b0, b1 = c['burst']
            sb = stats(a, lambda x: (x[:, 0] >= b0) & (x[:, 1] <= b1))
            res[nm]['burst_33_37s'] = sb
            if sb:
                md.append(f"| {nm} burst t=33..37 s | {sb['n']} | w median {sb['w_med']:.2f}, max {sb['w_max']:.2f} rad/s | | identity {sb['ang_vis'][0]:.2f} / {sb['ang_vis'][1]:.2f} / max {sb['ang_vis'][2]:.2f} | cv {sb['cv'][0]:.2f} / {sb['cv'][1]:.2f} / max {sb['cv'][2]:.2f} | "
                          f"{sb['gyro'][0]:.3f} / {sb['gyro'][1]:.3f} / {sb['gyro'][2]:.3f} | | |" if sb['cv'] else f"| {nm} burst | {sb['n']} | | | | | {sb['gyro']} | | |")
        print(md[-1] if 'burst_33_37s' not in res[nm] or not res[nm]['burst_33_37s'] else '\n'.join(md[-2:]), flush=True)
    md += ['', 'camera-IMU time offset scan (median gyro error [deg] vs IMU time = camera time + offset):', '']
    for nm, r in res.items():
        md.append(f"- {nm}: " + ', '.join(f'{k}s {v:.3f}' for k, v in r['toff_scan_median_err_deg'].items()))
    (OUT / 'gyro_pred.md').write_text('\n'.join(md))
    (OUT / 'gyro_pred.json').write_text(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
