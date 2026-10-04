#!/usr/bin/env python3
"""UZH-FPV Snapdragon ASL-ish folder (groundtruth.txt, imu.txt, left_images.txt, right_images.txt, img/) -> EuRoC layout (in place, files are moved).
usage: fpv_to_euroc.py <seq_dir> [cam_shift_s=-0.008]   (t_imu = t_cam + shift, Kalibr timeshift_cam_imu of the dataset calibration, OpenVINS config)
Writes <seq_dir>/mav0/{cam0,cam1,imu0}, <seq_dir>/gt_imu.tum (IMU/body pose, TUM rows).  Dataset: UZH-FPV, CC BY-NC-SA 3.0."""
import sys, os
from pathlib import Path
d = Path(sys.argv[1]); shift = float(sys.argv[2]) if len(sys.argv) > 2 else -0.008
mav = d / 'mav0'
for k, name in ((0, 'left'), (1, 'right')):
    (mav / f'cam{k}' / 'data').mkdir(parents=True, exist_ok=True)
    with open(mav / f'cam{k}' / 'data.csv', 'w') as f:
        f.write('#timestamp [ns], filename\n')
        for l in (d / f'{name}_images.txt').read_text().splitlines():
            if l.startswith('#') or not l.strip(): continue
            i, t, fn = l.split()[:3]; tn = int(round((float(t) + shift) * 1e9))
            src = d / fn
            if src.exists(): os.rename(src, mav / f'cam{k}' / 'data' / f'{tn}.png')
            f.write(f'{tn},{tn}.png\n')
(mav / 'imu0').mkdir(parents=True, exist_ok=True)
with open(mav / 'imu0' / 'data.csv', 'w') as f:
    f.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
    for l in (d / 'imu.txt').read_text().splitlines():
        if l.startswith('#') or not l.strip(): continue
        p = l.split(); f.write(f'{int(round(float(p[1]) * 1e9))},' + ','.join(p[2:8]) + '\n')
if (d / 'groundtruth.txt').exists():
    with open(d / 'gt_imu.tum', 'w') as f:
        for l in (d / 'groundtruth.txt').read_text().splitlines():
            if l.startswith('#') or not l.strip(): continue
            f.write(l.replace('\t', ' ') + '\n')
print('ok')
