#!/usr/bin/env python3
"""Per-sequence descriptive stats for section 9 (duration, frame/IMU timing regularity, gyro and GT path speed).  usage: phone_stats.py [seq ...]"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import phone_score as ps
for s in (sys.argv[1:] or list(ps.OFFSET)):
    d = ps.ROB / s
    cam = np.array([int(l.split(',')[0]) * 1e-9 for l in (d / 'cam0' / 'data.csv').read_text().splitlines()[1:] if l.strip()])
    imu = np.loadtxt(d / 'imu0' / 'data.csv', delimiter=',', comments='#'); ti = imu[:, 0] * 1e-9
    w = np.linalg.norm(imu[:, 1:4], axis=1); dc = np.diff(cam); di = np.diff(ti)
    gt = np.loadtxt(d / 'gt.tum'); g = gt[(gt[:, 0] + ps.OFFSET[s] >= cam[0]) & (gt[:, 0] + ps.OFFSET[s] <= cam[-1])]
    path = np.linalg.norm(np.diff(g[:, 1:4], axis=0), axis=1).sum(); sp = np.linalg.norm(np.diff(g[:, 1:4], axis=0), axis=1) / np.diff(g[:, 0])
    print(f"{s}: dur {cam[-1]-cam[0]:.0f} s, frames {len(cam)} @ {1/np.median(dc):.1f} Hz (dt std {dc.std()*1e3:.1f} ms, max {dc.max()*1e3:.0f} ms), IMU {len(ti)} @ {1/np.median(di):.1f} Hz (dt std {di.std()*1e3:.1f} ms, max {di.max()*1e3:.0f} ms), "
          f"gyro mean {w.mean():.2f} p95 {np.percentile(w,95):.2f} max {w.max():.2f} rad/s, |a| mean {np.linalg.norm(imu[:,4:7],axis=1).mean():.2f}, GT path {path:.0f} m, speed mean {np.median(sp):.2f} (p95 {np.percentile(sp,95):.2f}) m/s")
