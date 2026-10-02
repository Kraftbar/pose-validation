#!/usr/bin/env python3
"""Mobile-GVIO gnss_lla.csv (iPhone fixes, sensor clock) -> OKVIS2-X gps0/data.csv in a local ENU frame at the first fix (errors = reported covariance, as given).
usage: mobile_make_gps.py seq_dir [--blackout a,b]  (also writes enu_origin.json)"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from score_all import lla_to_enu
d = Path(sys.argv[1]); g = np.loadtxt(d / 'gnss_lla.csv', delimiter=',', skiprows=1)
lla0 = g[0, 1:4]; enu = lla_to_enu(g[:, 1:4], lla0)
bl = [float(x) for x in sys.argv[sys.argv.index('--blackout') + 1].split(',')] if '--blackout' in sys.argv else []
(d / 'gps0').mkdir(exist_ok=True)
with open(d / 'gps0' / 'data.csv', 'w') as f:
    f.write('timestamp, x, y, z, hErr1, hErr2, vErr\n')
    for r, p in zip(g, enu):
        if any(bl[k] <= r[0] - g[0, 0] <= bl[k + 1] for k in range(0, len(bl), 2)): continue
        f.write(f'{int(round(r[0]*1e9))},{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},{np.sqrt(r[4]):.2f},{np.sqrt(r[5]):.2f},{np.sqrt(r[6]):.2f}\n')
json.dump({'lla0': lla0.tolist()}, open(d / 'enu_origin.json', 'w'))
np.savetxt(d / 'gnss_enu.txt', np.c_[g[:, 0], enu], fmt='%.4f')
