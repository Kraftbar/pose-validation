#!/usr/bin/env python3
"""kimera_prep.py <seq> <params_dir> <mav0_dir>: rewrite Kimera EurocMono LeftCameraParams/ImuParams (and BackendParams autoInitialize) for the sequence; writes the dummy / real GT csv Kimera's EuRoC provider insists on."""
import sys, re
import numpy as np
from pathlib import Path
seq, P, M = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
R = Path('/home/nybo/github/pose-validation')
hdr = '#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n'
cam = [int(l.split(',')[0]) for l in (M / 'cam0/data.csv').read_text().splitlines()[1:] if l.strip()]
rate = round(1e9 / np.median(np.diff(cam)))
if seq.startswith(('MH_', 'V1_', 'V2_')):
    gt = [l.split() for l in open(R / 'runs/vio_compare/gt' / f'{seq}_body.tum')]
    with open(M / 'state_groundtruth_estimate0/data.csv', 'w') as f:
        f.write(hdr)
        for t, x, y, z, qx, qy, qz, qw in gt: f.write(f'{int(round(float(t)*1e9))},{x},{y},{z},{qw},{qx},{qy},{qz},0,0,0,0,0,0,0,0,0\n')
    rate_imu = 200
else:
    t = (R / 'tools/gnss_harness/robust_cfg' / seq / 'okvis_default.yaml').read_text()
    arr = lambda k: [float(x) for x in re.search(k + r':[^\[]*\[([^\]]*)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    T = arr('T_SC'); W, H = map(int, arr('image_dimension')); fx, fy = arr('focal_length'); cx, cy = arr('principal_point'); dc = arr('distortion_coefficients')
    imu = [int(l.split(',')[0]) for l in (M / 'imu0/data.csv').read_text().splitlines()[1:] if l.strip()]
    rate_imu = round(1e9 / np.median(np.diff(imu)))
    with open(M / 'state_groundtruth_estimate0/data.csv', 'w') as f:
        f.write(hdr); f.write(f'{imu[0]},0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0\n'); f.write(f'{imu[-1]},0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0\n')
    (P / 'LeftCameraParams.yaml').write_text(f"""%YAML:1.0
camera_id: left_cam
T_BS:
  cols: 4
  rows: 4
  data: [{', '.join(repr(x) for x in T)}]
rate_hz: {rate}
resolution: [{W}, {H}]
camera_model: pinhole
intrinsics: [{fx}, {fy}, {cx}, {cy}]
distortion_model: radial-tangential
distortion_coefficients: [{dc[0]}, {dc[1]}, {dc[2]}, {dc[3]}]
""")
    s = (P / 'ImuParams.yaml').read_text()
    s = re.sub(r'rate_hz:.*', f'rate_hz: {rate_imu}', s)
    s = re.sub(r'gyroscope_noise_density:.*', 'gyroscope_noise_density: 1.0e-2', s); s = re.sub(r'gyroscope_random_walk:.*', 'gyroscope_random_walk: 1.0e-4', s)
    s = re.sub(r'accelerometer_noise_density:.*', 'accelerometer_noise_density: 1.0e-1', s); s = re.sub(r'accelerometer_random_walk:.*', 'accelerometer_random_walk: 1.0e-3', s)
    (P / 'ImuParams.yaml').write_text(s)
    b = (P / 'BackendParams.yaml').read_text(); b = re.sub(r'autoInitialize:.*', 'autoInitialize: 1', b); (P / 'BackendParams.yaml').write_text(b)
print('kimera prep', seq, 'cam rate', rate)
