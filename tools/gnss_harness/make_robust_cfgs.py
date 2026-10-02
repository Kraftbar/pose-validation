#!/usr/bin/env python3
"""Per-dataset configs for the phone/handheld robustness study (stella upstream, ORB-SLAM3 mono / mono-inertial, OpenVINS mono, stella-port --camera string).
Writes tools/gnss_harness/robust_cfg/<seq>/{stella.yaml, orb3_mono.yaml, orb3_mi.yaml, ov/*.yaml, port_camera.txt}. Calibration sources:
 outdoor1 = Mobile-GVIO calib/orbslam3.yaml (Honor phone, pinhole+radtan 1280x720, T_b_c1 = cam->IMU), complex = OKVIS2-X config/gvins/okvis2.yaml for the /cam0 stream (the 'right' camera)."""
import os, re
from pathlib import Path
R = Path('/home/nybo/github/pose-validation')
OUT = R / 'tools/gnss_harness/robust_cfg'
ORB_EUROC = (R / 'external/vio/orbslam3/Examples/Monocular-Inertial/EuRoC.yaml').read_text()
S = {
 'outdoor1': dict(W=1280, H=720, fps=15, fx=996.4948765107108, fy=995.8706613911386, cx=651.7912725410048, cy=364.86680915422824,
                  d=[0.06513753258976974, -0.23698184370758457, 0.0007685090700979932, 0.005825777305547826],
                  T=[[0.01916709, -0.99980408, -0.00494212, 0.03133239], [-0.99955275, -0.01904832, -0.02305337, 0.00009802],
                     [0.02295471, 0.00538177, -0.99972202, 0.00177312], [0, 0, 0, 1]],
                  ng=1e-2, na=1e-1, gw=1e-4, aw=1e-3, hz=100.1, nfeat=4000, ini=10, mn=7),
 'complex': dict(W=752, H=480, fps=20, fx=477.11460200, fy=477.01905851, cx=377.85700939, cy=238.45983428,
                 d=[-0.27190160, 0.06572811, 0.00004263, 0.00001058],
                 T=[[0.999893235367105, -0.0138839964192605, 0.00455549234800658, 0.0350751246820374],
                    [0.0138335898016083, 0.999844724419467, 0.0109159902785527, -0.00322315785169526],
                    [-0.00470634256122773, -0.0108518060243712, 0.999930041875783, -0.0205095767896131], [0, 0, 0, 1]],
                 ng=4e-3, na=8e-2, gw=2e-6, aw=4e-5, hz=200.0, nfeat=1000, ini=20, mn=7),
}
def sub(txt, key, val):
    return re.sub(rf'^({re.escape(key)}:)[^\n]*', lambda m: f'{m.group(1)} {val}', txt, flags=re.M)
h = dict(S['outdoor1']); sx = 0.5
h.update(W=640, H=360, fx=h['fx'] * sx, fy=h['fy'] * sx, cx=h['cx'] * sx, cy=h['cy'] * sx)  # half-resolution copy (INTER_AREA) for the stella C port, which cannot take 1280x720
S['o1half'] = h
for name, c in S.items():
    d = OUT / name; (d / 'ov').mkdir(parents=True, exist_ok=True)
    # --- ORB-SLAM3
    t = ORB_EUROC
    for k, v in dict(**{'Camera1.fx': c['fx'], 'Camera1.fy': c['fy'], 'Camera1.cx': c['cx'], 'Camera1.cy': c['cy'], 'Camera1.k1': c['d'][0], 'Camera1.k2': c['d'][1],
                        'Camera1.p1': c['d'][2], 'Camera1.p2': c['d'][3], 'Camera.width': c['W'], 'Camera.height': c['H'], 'Camera.fps': c['fps'],
                        'Camera.newWidth': c['W'], 'Camera.newHeight': c['H'], 'IMU.NoiseGyro': c['ng'], 'IMU.NoiseAcc': c['na'], 'IMU.GyroWalk': c['gw'],
                        'IMU.AccWalk': c['aw'], 'IMU.Frequency': c['hz'], 'ORBextractor.nFeatures': c['nfeat'], 'ORBextractor.iniThFAST': c['ini'],
                        'ORBextractor.minThFAST': c['mn']}).items():
        t = sub(t, k, v)
    flat = ', '.join(f'{x}' for r in c['T'] for x in r)
    t = re.sub(r'data: \[.*?\]', f'data: [{flat}]', t, flags=re.S, count=1)
    (d / 'orb3_mi.yaml').write_text(t); (d / 'orb3_mono.yaml').write_text(t)
    # --- stella upstream
    s = (R / 'external/candidates/stella_vslam/example/euroc/EuRoC_mono.yaml').read_text()
    for k, v in dict(fx=c['fx'], fy=c['fy'], cx=c['cx'], cy=c['cy'], k1=c['d'][0], k2=c['d'][1], p1=c['d'][2], p2=c['d'][3], fps=float(c['fps']), cols=c['W'], rows=c['H']).items():
        s = re.sub(rf'^(  {k}:)[^\n]*', lambda m: f'{m.group(1)} {v}', s, count=1, flags=re.M)
    s = s.replace('ini_fast_threshold: 20', f'ini_fast_threshold: {c["ini"] if name=="complex" else 20}')
    (d / 'stella.yaml').write_text(s)
    (d / 'port_camera.txt').write_text(','.join(str(x) for x in [c['fx'], c['fy'], c['cx'], c['cy'], c['d'][0], c['d'][1], c['d'][2], c['d'][3], 0.0]) + '\n')
    # --- OpenVINS mono
    o = d / 'ov'
    est = (R / 'external/vio/ov_cfg/mono/estimator_config.yaml').read_text()
    (o / 'estimator_config.yaml').write_text(est.replace('init_dyn_use: false', 'init_dyn_use: true'))  # static init never fires on handheld motion
    rows = '\n'.join(f'    - [{", ".join(str(x) for x in r)}]' for r in c['T'])
    (o / 'kalibr_imucam_chain.yaml').write_text(f'''%YAML:1.0

cam0:
  T_imu_cam:
{rows}
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: {c["d"]}
  distortion_model: radtan
  intrinsics: [{c["fx"]}, {c["fy"]}, {c["cx"]}, {c["cy"]}]
  resolution: [{c["W"]}, {c["H"]}]
  rostopic: /cam0/image_raw
''')
    imu = (R / 'external/vio/ov_cfg/mono/kalibr_imu_chain.yaml').read_text()
    imu = re.sub(r'accelerometer_noise_density:[^\n#]*', f'accelerometer_noise_density: {c["na"]}  ', imu)
    imu = re.sub(r'accelerometer_random_walk:[^\n#]*', f'accelerometer_random_walk: {c["aw"]}  ', imu)
    imu = re.sub(r'gyroscope_noise_density:[^\n#]*', f'gyroscope_noise_density: {c["ng"]}  ', imu)
    imu = re.sub(r'gyroscope_random_walk:[^\n#]*', f'gyroscope_random_walk: {c["gw"]}  ', imu)
    imu = re.sub(r'update_rate:[^\n#]*', f'update_rate: {c["hz"]}  ', imu)
    (o / 'kalibr_imu_chain.yaml').write_text(imu)
print('ok')
