#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""gf_geo.c vs the numpy WGS-84 conversion used by the GNSS-VIO harness (tools/gnss_harness/score_all.py): LLA -> ECEF/ENU, ENU -> LLA round trip."""
import sys, subprocess
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from gf_cases import *  # noqa
import score_all as sa  # noqa

rng = np.random.default_rng(3)
lla0 = [22.3350016, 114.2639816, 130.814]
pts = np.c_[rng.uniform(-80, 80, 400), rng.uniform(-180, 180, 400), rng.uniform(-100, 9000, 400)]
pts = np.r_[pts, [[22.34, 114.27, 200], [lla0[0] + 1e-4, lla0[1] + 1e-4, lla0[2]], [89.9, 10, 0], [0, 0, 0]]]
pts = np.c_[np.round(pts[:, 0], 10), np.round(pts[:, 1], 10), np.round(pts[:, 2], 4)]
txt = '\n'.join('%.10f %.10f %.4f' % tuple(p) for p in pts)
r = subprocess.run([str(GF_RUN), '--geo', '%.7f,%.7f,%.3f' % tuple(lla0)], input=txt, capture_output=True, text=True, check=True)
o = np.array([[float(x) for x in l.split()] for l in r.stdout.splitlines()])
enu_ref = sa.lla_to_enu(pts, lla0)
ecef_ref = sa.ecef(pts[:, 0], pts[:, 1], pts[:, 2])
d_enu = np.abs(o[:, :3] - enu_ref).max(); d_ecef = np.abs(o[:, 6:9] - ecef_ref).max()
d_rt = np.abs(o[:, 3:5] - pts[:, :2]).max(); d_rth = np.abs(o[:, 5] - pts[:, 2]).max()
ok = np.where(np.abs(pts[:, 0]) < 89)[0]
print(f'LLA->ENU max diff vs numpy: {d_enu:.2e} m; ECEF: {d_ecef:.2e} m; ENU->LLA round trip: lat/lon {d_rt:.2e} deg ({d_rt * 111e3:.2e} m), h {d_rth:.2e} m')
assert d_enu < 1e-5 and d_ecef < 1e-4 and d_rth < 1e-5 and d_rt * 111e3 < 1e-5, 'geodetic mismatch'
print('PASS')
