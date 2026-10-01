#!/usr/bin/env python3
"""Convert an EuRoC ASL sequence's ground truth to TUM text files under runs/vio_compare/gt/.
  <seq>_body.tum : GT body (= IMU) frame pose, for VIO systems that report the IMU pose
  <seq>_cam0.tum : GT pose of cam0 (T_WB * T_BC0), for camera-only baselines that report the camera pose
Also writes <seq>_info.json (frame count, duration) used for coverage / real-time factor.
usage: vio_prep_gt.py <mav0 dir> <seq name>
"""
import json, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot

mav0, seq = Path(sys.argv[1]), sys.argv[2]
out = Path(__file__).resolve().parent.parent / 'runs' / 'vio_compare' / 'gt'
out.mkdir(parents=True, exist_ok=True)
import re
txt = (mav0 / 'cam0' / 'sensor.yaml').read_text()
T = np.array([float(x) for x in re.search(r'data:\s*\[([^\]]*)\]', txt).group(1).replace('\n', ' ').split(',')]).reshape(4, 4)  # T_BS (cam0)
rows = np.loadtxt(mav0 / 'state_groundtruth_estimate0' / 'data.csv', delimiter=',', comments='#')
ts = rows[:, 0] * 1e-9
p = rows[:, 1:4]
q = rows[:, [5, 6, 7, 4]]  # x y z w
R = Rot.from_quat(q).as_matrix()
pc = p + R @ T[:3, 3]
Rc = R @ T[:3, :3]
qc = Rot.from_matrix(Rc).as_quat()
def w(name, pp, qq):
    with open(out / name, 'w') as f:
        for t, a, b in zip(ts, pp, qq):
            f.write(f'{t:.9f} {a[0]:.6f} {a[1]:.6f} {a[2]:.6f} {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}\n')
w(f'{seq}_body.tum', p, q)
w(f'{seq}_cam0.tum', pc, qc)
cam = np.loadtxt(mav0 / 'cam0' / 'data.csv', delimiter=',', comments='#', usecols=0)
(out / f'{seq}_info.json').write_text(json.dumps({'frames': int(len(cam)), 'duration_s': float((cam[-1] - cam[0]) * 1e-9), 'cam_t0': float(cam[0] * 1e-9), 'cam_t1': float(cam[-1] * 1e-9)}))
print(seq, len(cam), 'frames', (cam[-1] - cam[0]) * 1e-9, 's')
