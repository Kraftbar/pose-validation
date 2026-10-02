#!/usr/bin/env python3
"""OKVIS2-X mono config for the Mobile-GVIO phone camera/IMU (calibration from the dataset's calib/orbslam3.yaml, T_b_c1 = camera->IMU).
usage: make_okvis2x_cfg_mobile.py out.yaml gps|nogps"""
import re, sys
src = open('/home/nybo/github/pose-validation/external/gnss/cfg/okvis2x_mono_gps.yaml').read()
T = '''[ 0.01916709, -0.99980408, -0.00494212, 0.03133239,
          -0.99955275, -0.01904832, -0.02305337, 0.00009802,
           0.02295471,  0.00538177, -0.99972202, 0.00177312,
           0.0, 0.0, 0.0, 1.0]'''
a = src.index('     - {T_SC:'); b = src.index('# additional camera parameters')
cam = f'''     - {{T_SC: # phone camera
        {T},
        image_dimension: [1280, 720],
        distortion_coefficients: [0.06513753258976974, -0.23698184370758457, 0.0007685090700979932, 0.005825777305547826],
        distortion_type: radialtangential,
        focal_length: [996.4948765107108, 995.8706613911386],
        principal_point: [651.7912725410048, 364.86680915422824],
        cam_model: pinhole,
        camera_type: gray,
        mapping: false,
        mapping_rectification: false,
        slam_use: okvis}}

'''
s = src[:a] + cam + src[b:]
for k, v in dict(sigma_g_c='1.0e-2', sigma_a_c='1.0e-1', sigma_gw_c='1.0e-4', sigma_aw_c='1.0e-3', a_max='176.0', g_max='7.8', g='9.80655').items():
    s = re.sub(rf'^(\s*{k}:)\s*\S+', rf'\1 {v}', s, flags=re.M)
s = s.replace('num_matching_threads: 4', 'num_matching_threads: 4')
if sys.argv[2] == 'gps':
    s = re.sub(r'r_SA:.*', 'r_SA: [0.0, 0.0, 0.0]', s)
else:
    s = s[:s.index('gps_parameters:')]
open(sys.argv[1], 'w').write(s)
