#!/usr/bin/env python3
"""XRSLAM device-config generator (benchmark glue).
usage: make_cfg.py <seq> <out_sensor.yaml> [noise=default|infl]
 seq: indoor1 indoor2 outdoor1 outdoor2 advio15 advio20 (phones, from tools/gnss_harness/robust_cfg/<seq>/okvis_default.yaml: T_SC, intrinsics, radtan)
Camera->body (body == IMU) = T_SC; images are undistorted by the player reader (camera_distortion_flag 1) and run through the pinhole model.
IMU noise: 'default' = the XRSLAM iphone12.yaml continuous covariances; 'infl' = OKVIS-tuned phone values (1e-2 / 1e-1 densities)."""
import sys, re
import numpy as np
from pathlib import Path
def mat2quat(R):  # x y z w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2; return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, s/4])
    i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
    s = np.sqrt(1 + R[i,i] - R[j,j] - R[k,k]) * 2; q = np.zeros(4)
    q[i] = s/4; q[j] = (R[j,i]+R[i,j])/s; q[k] = (R[k,i]+R[i,k])/s; q[3] = (R[k,j]-R[j,k])/s
    return q
seq, out = sys.argv[1], sys.argv[2]; noise = sys.argv[3] if len(sys.argv) > 3 else 'default'
src = Path('/home/nybo/github/pose-validation/tools/gnss_harness/robust_cfg') / seq / 'okvis_default.yaml'
t = src.read_text()
def arr(key):
    m = re.search(key + r':[^\[]*\[([^\]]*)\]', t, re.S); return [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
T = np.array(arr('T_SC')).reshape(4, 4); dim = arr('image_dimension'); fl = arr('focal_length'); pp = arr('principal_point'); dc = arr('distortion_coefficients')
q = [float(v) for v in mat2quat(T[:3, :3])]; p = [float(v) for v in T[:3, 3]]
if noise == 'infl': g, a, bg, ba = 1e-4, 1e-2, 1e-8, 1e-6
else: g, a, bg, ba = 2.8791302399999997e-08, 4.0e-6, 3.7608844899999997e-10, 9.0e-6
dg = lambda v: f'[{v}, 0.0, 0.0, 0.0, {v}, 0.0, 0.0, 0.0, {v}]'
Ts = ', '.join(f'{x:.9f}' for x in T.flatten())
open(out, 'w').write(f"""%YAML:1.0
imu:
  gyroscope_noise_density: 0.01
  gyroscope_random_walk: 0.0001
  accelerometer_noise_density: 0.1
  accelerometer_random_walk: 0.001
  accelerometer_bias: [0.0, 0.0, 0.0]
  gyroscope_bias: [0.0, 0.0, 0.0]
  extrinsic:
    q_bi: [ 0.0, 0.0, 0.0, 1.0 ]
    p_bi: [ 0.0, 0.0, 0.0 ]
  noise:
    cov_g: {dg(g)}
    cov_a: {dg(a)}
    cov_bg: {dg(bg)}
    cov_ba: {dg(ba)}
cam0:
  T_BS:
    cols: 4
    rows: 4
    data: [{Ts}]
  resolution: [{int(dim[0])}, {int(dim[1])}]
  camera_model: pinhole
  distortion_model: radtan
  intrinsics: [{fl[0]}, {fl[1]}, {pp[0]}, {pp[1]}]
  camera_distortion_flag: 1
  distortion: [{dc[0]}, {dc[1]}, {dc[2]}, {dc[3]}]
  camera_readout_time: 0.0
  time_offset: 0.0
  extrinsic:
    q_bc: [ {q[0]!r}, {q[1]!r}, {q[2]!r}, {q[3]!r} ]
    p_bc: [ {p[0]!r}, {p[1]!r}, {p[2]!r} ]
  noise: [
    0.5, 0.0,
    0.0, 0.5]
""")
