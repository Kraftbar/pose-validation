#!/usr/bin/env python3
"""MSCEqF phone config generator (benchmark glue). usage: make_cfg.py <seq> <out.yaml> [noise=default|infl]
Camera/IMU geometry from tools/gnss_harness/robust_cfg/<seq>/okvis_default.yaml (T_SC == MSCEqF T_imu_cam, intrinsics, radtan).
Everything else = the upstream EuRoC example config (examples/euroc/config/config.yaml) with zero_velocity_update disabled (see docs: the ZVU-enabled init path
crashes on MH_01 in this build) because phones do not start static. IMU noise: 'default' = EuRoC example numbers (about the XRSLAM iPhone12 values);
'infl' = OKVIS-tuned phone densities (1e-2 / 1e-1, walks 1e-4 / 1e-3)."""
import sys, re
from pathlib import Path
seq, out = sys.argv[1], sys.argv[2]; noise = sys.argv[3] if len(sys.argv) > 3 else 'default'
R = Path('/home/nybo/github/pose-validation')
sys.path.insert(0, str(R / 'tools/vio_harness')); import okvis_cfg
c = okvis_cfg.load(seq); T = c['T']; dim = c['dim']; fl = c['fl']; pp = c['pp']; dc = c['dc']
import os
cfg = (R / 'tools/vio_harness/msceqf/cfg/euroc_zvu_off.yaml').read_text()
if os.environ.get('MSCEQF_ZVU'): cfg = cfg.replace('zero_velocity_update: disabled', 'zero_velocity_update: enabled')   # upstream stock setting (static start)
lines = []
skip = False
for l in cfg.splitlines():
    k = l.split(':')[0].strip()
    if k in ('distortion_coeffs', 'distortion_model', 'resolution', 'intrinsics', 'T_imu_cam'): skip = (k == 'T_imu_cam'); continue
    if skip and l.startswith(' '): continue
    skip = False
    lines.append(l)
body = '\n'.join(lines)
if noise == 'okvis':   # IMU densities of the OKVIS yaml (drones: calib.py values)
    for k, v in (('accelerometer_noise_density', c['imu']['a']), ('accelerometer_random_walk', c['imu']['ba']), ('gyroscope_noise_density', c['imu']['g']), ('gyroscope_random_walk', c['imu']['bg'])):
        body = re.sub(rf'^{k}:.*$', f'{k}: {v}', body, flags=re.M)
if noise == 'infl':
    for k, v in (('accelerometer_noise_density', 1e-1), ('accelerometer_random_walk', 1e-3), ('gyroscope_noise_density', 1e-2), ('gyroscope_random_walk', 1e-4)):
        body = re.sub(rf'^{k}:.*$', f'{k}: {v}', body, flags=re.M)
add = f"""
distortion_model: {c['model']}
distortion_coeffs: [{dc[0]}, {dc[1]}, {dc[2]}, {dc[3]}]
resolution: [{dim[0]}, {dim[1]}]
intrinsics: [{fl[0]}, {fl[1]}, {pp[0]}, {pp[1]}]
T_imu_cam:
 - [{T[0]}, {T[1]}, {T[2]}, {T[3]}]
 - [{T[4]}, {T[5]}, {T[6]}, {T[7]}]
 - [{T[8]}, {T[9]}, {T[10]}, {T[11]}]
 - [0.0, 0.0, 0.0, 1.0]
"""
Path(out).write_text(body + '\n' + add)
