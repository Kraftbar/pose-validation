#!/usr/bin/env python3
"""Derive mono GVINS-dataset configs for OKVIS2-X (BSD-3) from its shipped config/gvins/okvis2.yaml (text edit, no YAML lib).
usage: make_okvis2x_cfg.py <out.yaml> <gps|nogps> [mono|stereo]"""
import re, sys
SRC = '/home/nybo/github/pose-validation/external/gnss/OKVIS2-X/config/gvins/okvis2.yaml'
out, gps = sys.argv[1], sys.argv[2] == 'gps'
mode = sys.argv[3] if len(sys.argv) > 3 else 'mono'
s = open(SRC).read()
if mode == 'mono':
    a = s.index('     - {T_SC: # Left camera'); b = s.index('# additional camera parameters')
    s = s[:a] + '\n' + s[b:]
    s = s.replace('sync_cameras: [0, 1]', 'sync_cameras: [0]').replace('deep_stereo_indices: [0,1]', 'deep_stereo_indices: [0]')
# IMU frame == body frame (output pose is the IMU pose)
s = re.sub(r'T_BS: *\n[^\n]*\n[^\n]*\n[^\n]*\n[^\n]*\n',
           'T_BS:\n        [1.0, 0.0, 0.0, 0.0,\n         0.0, 1.0, 0.0, 0.0,\n         0.0, 0.0, 1.0, 0.0,\n         0.0, 0.0, 0.0, 1.0]\n', s)
if not gps:
    s = s[:s.index('gps_parameters:')]
open(out, 'w').write(s)
