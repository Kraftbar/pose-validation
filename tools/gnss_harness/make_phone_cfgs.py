#!/usr/bin/env python3
"""Per-sequence configs for the 'more phone sequences' study (section 9): stella upstream, ORB-SLAM3 mono / mono-inertial (+ dataset-style 'orb3_mi_ds'), OpenVINS mono, OKVIS2-X mono default.
 Mobile-GVIO indoor1 / indoor2 / outdoor2: same phone and calibration as outdoor1 -> robust_cfg/outdoor1 is copied (run make_robust_cfgs.py first).
 ADVIO advio15 (calibration batch 13-17) / advio20 (batch 20-23): iPhone, 720x1280 portrait, 30 fps (every 2nd frame), IMU 100 Hz, Kalibr T_cam_imu and noise from the ADVIO calibration README.
 usage: make_phone_cfgs.py   -> robust_cfg/<seq>/ (okvis_default.yaml stella.yaml orb3_mono.yaml orb3_mi.yaml orb3_mi_ds.yaml ov/ port_camera.txt)"""
import re, shutil
from pathlib import Path
import numpy as np
R = Path('/home/nybo/github/pose-validation'); OUT = R / 'tools/gnss_harness/robust_cfg'
for s in ('indoor1', 'indoor2', 'outdoor2'):
    if (OUT / s).exists(): shutil.rmtree(OUT / s)
    shutil.copytree(OUT / 'outdoor1', OUT / s)
T_CI = np.array([[0.9999763379093255, -0.004079205042965442, -0.005539287650170447, -0.008977668364731128],   # ADVIO Kalibr T_cam_imu (imu -> cam)
                 [-0.004066386342107199, -0.9999890330121858, 0.0023234365646622014, 0.07557012320238939],
                 [-0.00554870467502187, -0.0023008567036498766, -0.9999819588046867, -0.005545773942541918], [0, 0, 0, 1.0]])
T_IC = np.linalg.inv(T_CI)   # cam -> imu (what ORB-SLAM3 IMU.T_b_c1, OKVIS T_SC and OpenVINS T_imu_cam expect)
S = {'advio15': dict(fx=1082.4, fy=1084.4, cx=364.6778, cy=643.3080, d=[0.0366, 0.0803, 0.000783, -0.000215]),
     'advio20': dict(fx=1081.1, fy=1082.1, cx=359.59, cy=640.79, d=[0.0556, -0.0454, 0.0009, -0.0018])}
for c in S.values(): c.update(W=720, H=1280, fps=30, T=T_IC.tolist(), ng=2.4e-3, na=4.8e-3, gw=5.1e-5, aw=2.1e-4, hz=100.0, nfeat=1500, ini=20, mn=7)
ORB_EUROC = (R / 'external/vio/orbslam3/Examples/Monocular-Inertial/EuRoC.yaml').read_text()
def sub(txt, key, val): return re.sub(rf'^({re.escape(key)}:)[^\n]*', lambda m: f'{m.group(1)} {val}', txt, flags=re.M)
for name, c in S.items():
    d = OUT / name; (d / 'ov').mkdir(parents=True, exist_ok=True)
    t = ORB_EUROC
    for k, v in {'Camera1.fx': c['fx'], 'Camera1.fy': c['fy'], 'Camera1.cx': c['cx'], 'Camera1.cy': c['cy'], 'Camera1.k1': c['d'][0], 'Camera1.k2': c['d'][1], 'Camera1.p1': c['d'][2], 'Camera1.p2': c['d'][3],
                 'Camera.width': c['W'], 'Camera.height': c['H'], 'Camera.fps': c['fps'], 'Camera.newWidth': c['W'], 'Camera.newHeight': c['H'], 'IMU.NoiseGyro': c['ng'], 'IMU.NoiseAcc': c['na'],
                 'IMU.GyroWalk': c['gw'], 'IMU.AccWalk': c['aw'], 'IMU.Frequency': c['hz'], 'ORBextractor.nFeatures': c['nfeat'], 'ORBextractor.iniThFAST': c['ini'], 'ORBextractor.minThFAST': c['mn']}.items():
        t = sub(t, k, v)
    flat = ', '.join(f'{x}' for r in c['T'] for x in r)
    t = re.sub(r'data: \[.*?\]', f'data: [{flat}]', t, flags=re.S, count=1)
    (d / 'orb3_mono.yaml').write_text(t); (d / 'orb3_mi_ds.yaml').write_text(t)   # orb3_mi_ds = ADVIO's own noise values (the only ones the dataset gives)
    for k, v in {'IMU.NoiseGyro': 1e-2, 'IMU.NoiseAcc': 1e-1, 'IMU.GyroWalk': 1e-4, 'IMU.AccWalk': 1e-3}.items(): t = sub(t, k, v)
    (d / 'orb3_mi.yaml').write_text(t)   # orb3_mi = noise inflated like the phone OKVIS config (1e-2 / 1e-1)
    s = (R / 'external/candidates/stella_vslam/example/euroc/EuRoC_mono.yaml').read_text()
    for k, v in dict(fx=c['fx'], fy=c['fy'], cx=c['cx'], cy=c['cy'], k1=c['d'][0], k2=c['d'][1], p1=c['d'][2], p2=c['d'][3], fps=float(c['fps']), cols=c['W'], rows=c['H']).items():
        s = re.sub(rf'^(  {k}:)[^\n]*', lambda m: f'{m.group(1)} {v}', s, count=1, flags=re.M)
    (d / 'stella.yaml').write_text(s)
    (d / 'port_camera.txt').write_text(','.join(str(x) for x in [c['fx'], c['fy'], c['cx'], c['cy'], *c['d'], 0.0]) + '\n')
    # OKVIS2-X: take the outdoor1 default config, swap camera block + extrinsics, keep the phone IMU noise (1e-2 / 1e-1, 1e-4 / 1e-3)
    o = (OUT / 'outdoor1' / 'okvis_default.yaml').read_text()
    a = o.index('     - {T_SC:'); b = o.index('camera_parameters:')
    Tf = ',\n          '.join(', '.join(f'{x:.8f}' for x in r) for r in c['T'])
    cam = f'''     - {{T_SC: # ADVIO iPhone camera
        [ {Tf} ],
        image_dimension: [{c['W']}, {c['H']}],
        distortion_coefficients: {c['d']},
        distortion_type: radialtangential,
        focal_length: [{c['fx']}, {c['fy']}],
        principal_point: [{c['cx']}, {c['cy']}],
        cam_model: pinhole,
        camera_type: gray,
        mapping: false,
        mapping_rectification: false,
        slam_use: okvis}}

'''
    (d / 'okvis_default.yaml').write_text(o[:a] + cam + o[b:])
    ov = d / 'ov'
    for f in ('estimator_config.yaml',): shutil.copy(OUT / 'outdoor1' / 'ov' / f, ov / f)
    rows = '\n'.join(f'    - [{", ".join(str(x) for x in r)}]' for r in c['T'])
    (ov / 'kalibr_imucam_chain.yaml').write_text(f'''%YAML:1.0

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
    imu = (OUT / 'outdoor1' / 'ov' / 'kalibr_imu_chain.yaml').read_text()
    for k, v in dict(accelerometer_noise_density=c['na'], accelerometer_random_walk=c['aw'], gyroscope_noise_density=c['ng'], gyroscope_random_walk=c['gw'], update_rate=c['hz']).items():
        imu = re.sub(rf'{k}:[^\n#]*', f'{k}: {v}  ', imu)
    (ov / 'kalibr_imu_chain.yaml').write_text(imu)
print('ok')
for name in ('indoor1', 'indoor2', 'outdoor2', 'advio15', 'advio20'):   # stella variant with lower FAST thresholds (initialisation on low-texture corridors)
    (OUT / name / 'stella_lowfast.yaml').write_text((OUT / name / 'stella.yaml').read_text().replace('ini_fast_threshold: 20', 'ini_fast_threshold: 10').replace('min_fast_threshold: 7', 'min_fast_threshold: 4'))
