#!/usr/bin/env python3
"""Score drone-benchmark runs (own glue; alignment is benchmark.umeyama_alignment via tools/gnss_harness/gnss_eval.py).
usage: score.py <seq_name> <seq_dir> [--runs runs/drone_compare]
Reference: insane -> <seq_dir>/gt_enu.tum (dual-RTK midpoint, 7 Hz, cm-level); fpv -> <seq_dir>/gt_imu.tum (IMU pose, Leica/Vicon-class, only the flight part of the sequence); mars -> <seq_dir>/gt_rtk.csv (DJI RTK antenna position, 5 Hz) + lever.json.
Writes <runs>/<seq_name>/<system>/metrics.json and <runs>/<seq_name>/table.md."""
import sys, json, re
from pathlib import Path
import numpy as np
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/gnss_harness')
from gnss_eval import read_traj, GT, score

name, seqd = sys.argv[1], Path(sys.argv[2]); runs = Path(sys.argv[sys.argv.index('--runs') + 1]) if '--runs' in sys.argv else Path('/home/nybo/github/pose-validation/runs/drone_compare')
cam = np.loadtxt(seqd / 'mav0' / 'cam0' / 'data.csv', delimiter=',', usecols=0, comments='#') * 1e-9
if (seqd / 'gt_imu.tum').exists():
    gt = GT.from_tum(seqd / 'gt_imu.tum'); r_SA = [0, 0, 0]; kind = 'fpv'
elif (seqd / 'gt_enu.tum').exists():  # INSANE: dual-RTK midpoint (vehicle centre) in the GNSS ENU frame, lever from the IMU to the vehicle centre
    gt = GT.from_tum(seqd / 'gt_enu.tum'); r_SA = json.load(open(seqd.parent / (seqd.name + '_cfg') / 'lever.json'))['r_RTK']; kind = 'insane'
else:
    g = np.loadtxt(seqd / 'gt_rtk.csv', delimiter=','); gt = GT(g[:, 0], g[:, 1:4], np.ones(len(g), bool)); kind = 'mars'
    lev = json.load(open(seqd.parent / (seqd.name + '_cfg') / 'lever.json')); r_SA = lev['r_RTK']
gt_span = (gt.t[0], gt.t[-1]); nwin = int(((cam >= gt_span[0]) & (cam <= gt_span[1])).sum())
dur = cam[-1] - cam[0]
rows = []
for d in sorted((runs / name).iterdir()):
    if not (d / 'run.json').exists(): continue
    rj = json.load(open(d / 'run.json')); m = {'system': d.name, 'wall_s': rj['wall_s'], 'rtf': rj['wall_s'] / dur, 'exit_code': rj['exit_code']}
    # losses = tracking-lost / map-reset events found in the log (system specific, indicative)
    log = (d / 'log.txt').read_text(errors='ignore') if (d / 'log.txt').exists() else ''
    if d.name.startswith('orb'): m['lost_events'] = len(re.findall(r'Fail to track local map', log)) + max(len(re.findall(r'New Map created', log)) - 1, 0)  # failed tracks + extra maps
    elif d.name.startswith('stella'): m['lost_events'] = len(re.findall(r'tracking lost|Lost|lost', log))
    elif d.name.startswith('basalt'): m['lost_events'] = len(re.findall(r'lost|Lost|LOST', log))
    else: m['lost_events'] = None  # OKVIS2 has no loss concept (it always outputs a pose)
    tp = d / 'trajectory.tum'
    if tp.exists() and tp.stat().st_size:
        tr = read_traj(tp)
        # camera-only systems report the camera pose: use the camera position (r_SA irrelevant for the shape; they are SE3/Sim3 scored at the cam)
        if len(tr) and 1e10 < tr[0, 0] < 1e12: tr[:, 0] *= 1e-9  # ORB-SLAM3 EuRoC writer stamps in ns
        tr = tr[np.argsort(tr[:, 0])]
        ra = [0, 0, 0] if d.name.startswith('stella') else r_SA
        res = score(tr, gt, ra, len(cam), dur)
        res.pop('_t', None); res.pop('_err', None); res.pop('_err_se3', None)
        inw = ((tr[:, 0] >= gt_span[0]) & (tr[:, 0] <= gt_span[1])).sum()
        res['coverage_gt_window'] = float(inw) / max(nwin, 1); m.update(res)
        gp = next((p for p in [d / 'global_final.csv', d / 'global_final_raw.csv', *d.glob('okvis2-*-global-final_trajectory.csv')] if p.exists()), None)
        if gp is not None:
            g2 = read_traj(gp); g2 = g2[np.argsort(g2[:, 0])]
            gres = score(g2, gt, r_SA, len(cam), dur, geo=True)
            for k in ('ate_noalign', 'noalign_median', 'noalign_max', 'ate_se3', 'ate_sim3', 'scale', 'n_scored'): m['geo_' + k] = gres.get(k)
    json.dump(m, open(d / 'metrics.json', 'w'), indent=1); rows.append(m)
if (seqd / 'gnss_px4.csv').exists():  # raw receiver fixes alone (no vision): the GNSS baseline every fused system must beat
    g = np.loadtxt(seqd / 'gnss_px4.csv', delimiter=',', comments='#'); tr = np.c_[g[:, 0], g[:, 1:4]]
    res = score(tr, gt, [0, 0, 0], len(g), g[-1, 0] - g[0, 0], geo=True)
    m = {'system': 'gnss_alone(px4 receiver)', 'wall_s': 0, 'rtf': 0, 'exit_code': 0, 'lost_events': None, 'coverage_gt_window': 1.0}
    m.update({k: v for k, v in res.items() if not k.startswith('_')}); m['geo_ate_noalign'] = res.get('ate_noalign'); rows.append(m)
f = lambda v, p=3: '-' if v is None else f'{v:.{p}f}'
T = [f'### {name} (duration {dur:.0f} s, {len(cam)} frames, reference window {gt_span[1] - gt_span[0]:.1f} s)\n',
     '| system | ATE SE3 (m) | ATE Sim3 (m) | scale | geo-ref err, no align (m) | coverage (all / ref window) | lost events | wall (s) | RTF |', '|---|---|---|---|---|---|---|---|---|']
for m in rows:
    T.append(f"| {m['system']} | {f(m.get('ate_se3'))} | {f(m.get('ate_sim3'))} | {f(m.get('scale'), 2)} | {f(m.get('geo_ate_noalign'))} | "
             f"{100 * m.get('coverage', 0):.0f}% / {100 * m.get('coverage_gt_window', 0):.0f}% | {'-' if m['lost_events'] is None else m['lost_events']} | {m['wall_s']:.0f} | {m['rtf']:.2f} |")
(runs / name / 'table.md').write_text('\n'.join(T) + '\n'); print('\n'.join(T))
