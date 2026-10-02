#!/usr/bin/env python3
"""Image/motion diagnostics per 30 s bin for the phone/handheld study: gyro norm (rad/s), sharpness (variance of Laplacian), FAST(thr 20 and 10) corner counts,
mean brightness, plus the coverage gaps (> 1 s) of every run's trajectory. usage: robust_diag.py <outdoor1|complex> -> runs/gnss_compare/robustness/diag_<seq>.json"""
import sys, json
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).parent))
from gnss_eval import read_traj
ROB = Path('/home/nybo/github/pose-validation/external/gnss/rob'); OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare/robustness')
seq = sys.argv[1]; d = ROB / seq
cam = [l.split(',') for l in (d / 'cam0' / 'data.csv').read_text().splitlines()[1:]]
tc = np.array([int(c[0]) * 1e-9 for c in cam]); t0 = tc[0]
imu = np.loadtxt(d / 'imu0' / 'data.csv', delimiter=',', comments='#'); ti = (imu[:, 0] * 1e-9 - t0); wn = np.linalg.norm(imu[:, 1:4], axis=1)
f20 = cv2.FastFeatureDetector_create(20, True); f10 = cv2.FastFeatureDetector_create(10, True)
rows = []
for k in range(0, len(cam), 15):
    im = cv2.imread(str(d / 'cam0' / 'data' / cam[k][1].strip()), 0)
    rows.append((tc[k] - t0, cv2.Laplacian(im, cv2.CV_64F).var(), len(f20.detect(im)), len(f10.detect(im)), float(im.mean())))
r = np.array(rows); res = []
for b in range(int((tc[-1] - t0) // 30) + 1):
    m = (r[:, 0] >= 30 * b) & (r[:, 0] < 30 * b + 30); mi = (ti >= 30 * b) & (ti < 30 * b + 30)
    if m.sum(): res.append(dict(t=30 * b, gyro_mean=round(float(wn[mi].mean()), 2), gyro_p95=round(float(np.percentile(wn[mi], 95)), 2), lapvar=round(float(r[m, 1].mean()), 1),
                               fast20=int(r[m, 2].mean()), fast10=int(r[m, 3].mean()), bright=round(float(r[m, 4].mean()), 0)))
gaps = {}
for rd in sorted((ROB / 'out').glob(f'{seq}_*')):
    p = rd / 'traj.txt'
    if not p.exists() or p.stat().st_size == 0: continue
    a = np.sort(read_traj(p)[:, 0]) - t0
    g = np.where(np.diff(a) > 1.0)[0]
    gaps[rd.name] = dict(first=round(float(a[0]), 1), last=round(float(a[-1]), 1), gaps=[(round(float(a[i]), 1), round(float(a[i + 1]), 1)) for i in g][:40], n_gaps=len(g))
json.dump(dict(bins=res, gaps=gaps), open(OUT / f'diag_{seq}.json', 'w'), indent=1)
print('overall gyro mean %.2f p95 %.2f; lapvar mean %.1f; fast20 mean %d' % (wn.mean(), np.percentile(wn, 95), r[:, 1].mean(), r[:, 2].mean()))
for x in res: print(x)
