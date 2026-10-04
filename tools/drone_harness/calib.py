#!/usr/bin/env python3
"""Per-sequence calibration (UZH-FPV Snapdragon outdoor, MARS-LVIG) and config writers for every system in the drone benchmark. Own code.
usage: calib.py <fpv|mars> <seq_dir> <cfg_out_dir> [--gnss]
All systems consume the EuRoC layout <seq_dir>/mav0/{cam0,cam1,imu0,gps0}; body frame = IMU frame; trajectories are scored at the IMU origin
(plus the GNSS antenna lever arm where the reference is an antenna)."""
import sys, json
from pathlib import Path
import numpy as np

# ---------------------------------------------------------------------------------------------- UZH-FPV (OpenVINS' copy of the dataset's Kalibr result, outdoor forward, Snapdragon)
def fpv():
    Tci = [np.array([[-0.03179778293757218, -0.9994933985910031, -0.001359107523862424, 0.021115239798621798],
                     [0.012827844120885779, 0.0009515801497960164, -0.9999172670328424, -0.0008992998316121829],
                     [0.9994120008362244, -0.03181258663210035, 0.012791087377928778, -0.009491094814035777], [0, 0, 0, 1]]),
           np.array([[-0.011450159873389598, -0.9998746482793399, -0.010935335712288774, -0.05828448770624624],
                     [0.009171247533644289, 0.010830579777447058, -0.9998992883087583, -0.0002362068202437068],
                     [0.999892385238307, -0.01154929737910465, 0.009046086032012068, -0.00947464531803495], [0, 0, 0, 1]])]
    cams = [dict(T_BC=np.linalg.inv(Tci[0]), K=[277.4786896484645, 277.42548548840034, 320.1052053576385, 242.10083077857894], model='equidistant',
                 dist=[-0.005719912631104124, 0.004742449009601135, 0.0012060658036136048, -0.001580292679344826], size=[640, 480]),
            dict(T_BC=np.linalg.inv(Tci[1]), K=[276.78679780974477, 276.79332134030807, 314.2862327340746, 236.51313088043128], model='equidistant',
                 dist=[-0.009025009906076716, 0.009967427035376123, -0.0029538969814842117, -0.0003503551771748748], size=[640, 480])]
    # Snapdragon IMU: OpenVINS' commented "inflated" values (the active ones are far too optimistic for this MEMS part)
    imu = dict(sg=0.05, sa=0.1, sgw=4e-5, saw=2e-3, rate=500.0, a_max=176.0, g_max=34.0)
    return dict(cams=cams, imu=imu, g=9.80665, fps=30.0, r_SA=None, fast=15, fast_ini=20)

# ---------------------------------------------------------------------------------------------- MARS-LVIG (HKU MaRS): camera 2448x2048 intrinsics calibrated here
# (9 chessboard views of the dataset's raw calibration set, cv2.calibrateCamera, rms 0.28 px), images are decoded at half resolution (1224x1024).
def mars(intr_json='/home/nybo/github/pose-validation/external/drone/mars/cam_intrinsics_full.json'):
    c = json.load(open(intr_json)); K = np.array(c['K']); d = c['dist']
    # half-resolution intrinsics (pixel centres at integer coords): c' = (c + 0.5)/2 - 0.5
    fx, fy, cx, cy = K[0, 0] / 2, K[1, 1] / 2, (K[0, 2] + 0.5) / 2 - 0.5, (K[1, 2] + 0.5) / 2 - 0.5
    R_cl = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float); t_cl = np.array([-0.03, 0.0, -0.058])  # dataset CAD_extrinsic.yaml: T_cam_lidar
    t_li = np.array([-0.011, -0.02329, 0.04412])  # Livox Avia manual: IMU position in the lidar frame (IMU axes = lidar axes)
    T_ci = np.eye(4); T_ci[:3, :3] = R_cl; T_ci[:3, 3] = R_cl @ t_li + t_cl
    cams = [dict(T_BC=np.linalg.inv(T_ci), K=[fx, fy, cx, cy], model='radtan', dist=[d[0], d[1], d[2], d[3]], size=[1224, 1024])]
    imu = dict(sg=2.0e-3, sa=2.0e-2, sgw=5e-5, saw=1e-3, rate=200.0, a_max=156.0, g_max=34.0)  # BMI088 datasheet noise x ~5 margin
    r_SA = np.array([-0.3501, 0.1631, 0.0]) - t_li   # u-blox F9P antenna in the IMU frame (CAD_extrinsic.yaml, lidar frame -> IMU)
    r_RTK = np.array([-0.1471, -0.3049, 0.3448]) - t_li  # DJI RTK antenna (reference) in the IMU frame
    return dict(cams=cams, imu=imu, g=9.7877, fps=10.0, r_SA=r_SA.tolist(), r_RTK=r_RTK.tolist(), fast=34, fast_ini=20)

def mat_rows(T): return ',\n          '.join(', '.join(f'{x:.12g}' for x in T[i]) for i in range(4))

# ---------------------------------------------------------------------------------------------- OKVIS2 / OKVIS2-X
def okvis_cfg(C, ncam, gps, loop=True, final_ba=True, kp=1000, thr=34.0):
    s = '%YAML:1.0\ncameras:\n'
    for c in C['cams'][:ncam]:
        dt = 'equidistant' if c['model'] == 'equidistant' else 'radialtangential'
        s += (f"     - {{T_SC:\n        [ {mat_rows(c['T_BC'])}],\n        image_dimension: {c['size']},\n        distortion_coefficients: {c['dist']},\n"
              f"        distortion_type: {dt},\n        focal_length: {c['K'][:2]},\n        principal_point: {c['K'][2:]},\n        cam_model: pinhole,\n"
              f"        camera_type: gray,\n        mapping: false,\n        mapping_rectification: false,\n        slam_use: okvis}}\n\n")
    im = C['imu']
    s += f"""camera_parameters:
    timestamp_tolerance: 0.005
    sync_cameras: {list(range(ncam))}
    image_delay: 0.0
    online_calibration:
        do_extrinsics: false
        do_extrinsics_final_ba: false
        sigma_r: 0.01
        sigma_alpha: 0.1
        sigma_r_final_ba: 0.03
        sigma_alpha_final_ba: 0.3
    deep_stereo_indices: {[0, 1] if ncam == 2 else [0]}
    fov_scale: 1.0

imu_parameters:
    use: true
    a_max: {im['a_max']}
    g_max: {im['g_max']}
    sigma_g_c: {im['sg']}
    sigma_a_c: {im['sa']}
    sigma_bg: 0.03
    sigma_ba: 0.1
    sigma_gw_c: {im['sgw']}
    sigma_aw_c: {im['saw']}
    g: {C['g']}
    g0: [0.0, 0.0, 0.0]
    a0: [0.0, 0.0, 0.0]
    s_a: [1.0, 1.0, 1.0]
    T_BS:
        [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]

frontend_parameters:
    detection_threshold: {thr}
    absolute_threshold: 4.0
    matching_threshold: 60.0
    octaves: 0
    max_num_keypoints: {kp}
    keyframe_overlap: 0.6
    use_cnn: false
    parallelise_detection: true
    num_matching_threads: 4

estimator_parameters:
    num_keyframes: 5
    num_loop_closure_frames: 3
    num_imu_frames: 3
    do_loop_closures: {str(loop).lower()}
    do_final_ba: {str(final_ba).lower()}
    enforce_realtime: false
    realtime_min_iterations: 3
    realtime_max_iterations: 10
    realtime_time_limit: 0.035
    realtime_num_threads: 3
    full_graph_iterations: 50
    full_graph_num_threads: 3
    p_dbow: 0.55
    drift_percentage_heuristic: 5.5

output_parameters:
    display_topview: false
    display_matches: false
    display_overhead: false
    enable_submapping: false
"""
    if gps:
        s += f"\ngps_parameters:\n    data_type: cartesian\n    r_SA: {list(C['r_SA'])}\n    yaw_error_threshold: 1.0\n    robust_gps_init: true\n"
    return s

# ---------------------------------------------------------------------------------------------- Basalt (stereo, kb4 only)
def q_xyzw(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2; return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    i = np.argmax(np.diag(R)); j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2; q = [0] * 4; q[i] = s / 4; q[3] = (R[k, j] - R[j, k]) / s
    q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; return q

def basalt_calib(C):
    im = C['imu']; Tl, Il = [], []
    for c in C['cams']:
        T = c['T_BC']; q = q_xyzw(T[:3, :3])
        Tl.append(dict(px=T[0, 3], py=T[1, 3], pz=T[2, 3], qx=q[0], qy=q[1], qz=q[2], qw=q[3]))
        fx, fy, cx, cy = c['K']; k = c['dist']
        Il.append(dict(camera_type='kb4', intrinsics=dict(fx=fx, fy=fy, cx=cx, cy=cy, k1=k[0], k2=k[1], k3=k[2], k4=k[3])))
    v = dict(T_imu_cam=Tl, intrinsics=Il, resolution=[c['size'] for c in C['cams']],
             calib_accel_bias=[0.0] * 9, calib_gyro_bias=[0.0] * 12, imu_update_rate=im['rate'],
             accel_noise_std=[im['sa']] * 3, gyro_noise_std=[im['sg']] * 3, accel_bias_std=[im['saw']] * 3, gyro_bias_std=[im['sgw']] * 3,
             T_mocap_world=dict(px=0, py=0, pz=0, qx=0, qy=0, qz=0, qw=1), T_imu_marker=dict(px=0, py=0, pz=0, qx=0, qy=0, qz=0, qw=1),
             mocap_time_offset_ns=0, mocap_to_imu_offset_ns=0, cam_time_offset_ns=0,
             vignette=[dict(value0=0, value1=50000000000, value2=[[1.0]] * 10)] * len(C['cams']))
    return json.dumps(dict(value0=v), indent=1)

# ---------------------------------------------------------------------------------------------- ORB-SLAM3 (reference only, GPL)
def orb_cfg(C, stereo, nfeat=1200):
    c0 = C['cams'][0]; im = C['imu']; fish = c0['model'] == 'equidistant'
    s = '%YAML:1.0\nFile.version: "1.0"\n'
    s += f'Camera.type: "{"KannalaBrandt8" if fish else "PinHole"}"\n'
    for i, c in enumerate(C['cams'][:2 if stereo else 1], 1):
        fx, fy, cx, cy = c['K']; d = c['dist']
        s += f'Camera{i}.fx: {fx}\nCamera{i}.fy: {fy}\nCamera{i}.cx: {cx}\nCamera{i}.cy: {cy}\n'
        if fish: s += ''.join(f'Camera{i}.k{j + 1}: {d[j]}\n' for j in range(4))
        else: s += f'Camera{i}.k1: {d[0]}\nCamera{i}.k2: {d[1]}\nCamera{i}.p1: {d[2]}\nCamera{i}.p2: {d[3]}\n'
    s += f"Camera.width: {c0['size'][0]}\nCamera.height: {c0['size'][1]}\nCamera.fps: {int(C['fps'])}\nCamera.RGB: 1\n"
    if stereo:
        T = np.linalg.inv(C['cams'][0]['T_BC']) @ C['cams'][1]['T_BC']  # T_c1_c2: points of cam2 -> cam1
        s += 'Stereo.ThDepth: 40.0\nStereo.T_c1_c2: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [' + ', '.join(f'{x:.12g}' for x in T.ravel()) + ']\n'
        if fish: s += f"Camera1.overlappingBegin: 0\nCamera1.overlappingEnd: {c0['size'][0] - 1}\nCamera2.overlappingBegin: 0\nCamera2.overlappingEnd: {c0['size'][0] - 1}\n"
    s += 'IMU.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [' + ', '.join(f'{x:.12g}' for x in c0['T_BC'].ravel()) + ']\n'
    s += f"IMU.NoiseGyro: {im['sg']}\nIMU.NoiseAcc: {im['sa']}\nIMU.GyroWalk: {im['sgw']}\nIMU.AccWalk: {im['saw']}\nIMU.Frequency: {im['rate']}\n"
    s += (f"ORBextractor.nFeatures: {nfeat}\nORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\nORBextractor.iniThFAST: {C['fast_ini']}\nORBextractor.minThFAST: 7\n"
          "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\nViewer.GraphLineWidth: 0.9\nViewer.PointSize: 2.0\nViewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
          "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -1.8\nViewer.ViewpointF: 500.0\nViewer.imageViewScale: 1.0\n")
    return s

# ---------------------------------------------------------------------------------------------- stella_vslam (mono)
def stella_cfg(C, nfeat=None):
    c = C['cams'][0]; fish = c['model'] == 'equidistant'; fx, fy, cx, cy = c['K']; d = c['dist']
    s = f'Camera:\n  name: "drone"\n  setup: "monocular"\n  model: "{"fisheye" if fish else "perspective"}"\n  fx: {fx}\n  fy: {fy}\n  cx: {cx}\n  cy: {cy}\n'
    if fish: s += ''.join(f'  k{j + 1}: {d[j]}\n' for j in range(4))
    else: s += f'  k1: {d[0]}\n  k2: {d[1]}\n  p1: {d[2]}\n  p2: {d[3]}\n  k3: 0.0\n'
    s += f"  fps: {C['fps']}\n  cols: {c['size'][0]}\n  rows: {c['size'][1]}\n  color_order: \"Gray\"\n"
    s += 'Preprocessing:\n  min_size: 800\nFeature:\n  name: "default"\n  scale_factor: 1.2\n  num_levels: 8\n  ini_fast_threshold: 20\n  min_fast_threshold: 7\n'
    s += ('Mapping:\n  backend: "g2o"\n  baseline_dist_thr_ratio: 0.02\n  redundant_obs_ratio_thr: 0.9\n  num_covisibilities_for_landmark_generation: 20\n'
          '  num_covisibilities_for_landmark_fusion: 20\n  erase_temporal_keyframes: false\n  num_temporal_keyframes: 15\nTracking:\n  backend: "g2o"\n'
          '  enable_temporal_keyframe_only_tracking: false\nKeyframeInserter:\n  wait_for_local_bundle_adjustment: false\nRelocalizer:\n  search_neighbor: true\n'
          'LoopDetector:\n  backend: "g2o"\nSystem:\n  map_format: "msgpack"\n  num_grid_cols: 32\n  num_grid_rows: 24\n')
    return s

# ---------------------------------------------------------------------------------------------- INSANE (Univ. Klagenfurt) nav cam, PX4 IMU, Kalibr result "nav_cam_radtan" of the dataset (halved to 1028x771)
INSANE = {'mars': dict(K=[1134.1962336714, 1135.509432443764, 1034.9252935429292, 750.9855619769147], d=[-0.2789624778746687, 0.08326114116558676, -0.0002778237201036228, -0.0003520215650091775],
                       T=[[-0.9995462687822135, 0.009026648725449518, -0.028736321552042587, -0.2548635985803006], [0.005770977572923451, 0.9937561519216068, 0.11142444227283936, 0.05189940496536139],
                          [0.029562685625765826, 0.11120804885775055, -0.9933573463199658, -0.0129615438247558], [0, 0, 0, 1]]),
          'klu1': dict(K=[1140.568807287164, 1138.229332477909, 1050.4263115571923, 731.6675708950979], d=[-0.24953155709879407, 0.052177152480177444, 4.248459864565281e-05, -0.00017620193709646758],
                       T=[[-0.9994755028106611, -0.022518362304292116, 0.023273217235888752, -0.2527399835555562], [-0.02275197225831357, 0.9996928897440455, -0.00982211553366561, 0.045387794112330376],
                          [-0.023044891836004854, -0.010346475454587485, -0.9996808907876226, -0.019307258157050512], [0, 0, 0, 1]])}
def insane(which):
    c = INSANE[which]; fx, fy, cx, cy = c['K']
    cam = dict(T_BC=np.linalg.inv(np.array(c['T'])), K=[fx / 2, fy / 2, (cx + 0.5) / 2 - 0.5, (cy + 0.5) / 2 - 0.5], model='radtan', dist=c['d'], size=[1028, 771])
    imu = dict(sg=4e-3, sa=4e-2, sgw=2e-4, saw=2e-3, rate=200.0, a_max=156.0, g_max=34.0)  # Pixhawk-class MEMS, generous (no datasheet tuning)
    return dict(cams=[cam], imu=imu, g=9.80665, fps=15.0, r_SA=[0.0, 0.0, 0.0], r_RTK=[-0.06, 0.0, 0.0], fast=34, fast_ini=20)  # GNSS antenna position in the IMU frame unknown -> 0; GT = vehicle centre, 6 cm behind the IMU

def build(which):
    if which.startswith('insane_'): return insane(which[7:])
    return fpv() if which == 'fpv' else mars()

if __name__ == '__main__':
    which, out = sys.argv[1], Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True); C = build(which)
    stereo = len(C['cams']) == 2
    if stereo:
        (out / 'okvis2_stereo.yaml').write_text(okvis_cfg(C, 2, False)); (out / 'basalt_calib.json').write_text(basalt_calib(C))
        (out / 'orb_stereo_inertial.yaml').write_text(orb_cfg(C, True))
    (out / 'okvis2_mono.yaml').write_text(okvis_cfg(C, 1, False)); (out / 'orb_mono_inertial.yaml').write_text(orb_cfg(C, False))
    (out / 'stella_mono.yaml').write_text(stella_cfg(C))
    if C['r_SA'] is not None:
        (out / 'okvis2x_mono_gnss.yaml').write_text(okvis_cfg(C, 1, True, loop=False)); (out / 'okvis2x_mono_nognss.yaml').write_text(okvis_cfg(C, 1, False, loop=False))
    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in dict(r_SA=C['r_SA'], r_RTK=C.get('r_RTK')).items()}, open(out / 'lever.json', 'w'))
    print('wrote', sorted(p.name for p in out.iterdir()))
