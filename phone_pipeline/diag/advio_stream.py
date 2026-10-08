#!/usr/bin/env python3
"""ADVIO (CC BY-NC 4.0, Zenodo 1476931) iPhone sequence -> the EuRoC-style fetch layout used by make_layout.py / run_rob.sh.
usage: advio_stream.py <advio-NN> <out_dir> [--every N]   (keeps every N-th video frame, default 2 = 30 fps; 720x1280 portrait, gray JPEG q95)
Writes cam0/data/*.jpg + data.csv, imu0/data.csv (ns, gyro, accel; accel linearly interpolated to the gyro stamps, m/s^2 as stored),
gt.tum (t x y z qx qy qz qw, ADVIO camera-pose ground truth, same clock as the sensors: no offset), gnss_lla.csv (CoreLocation fixes; cov = hAcc^2, hAcc^2, vAcc^2), info.json.
Calibration: iPhone camera by batch (sequences 1-12 / 13-17 / 18-19 / 20-23), see make_phone_cfgs.py."""
import sys, json, zipfile, io
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/gnss_harness'))
from http_zip import HTTPFile
nm, out = sys.argv[1], Path(sys.argv[2])   # out should be on /dev/shm to keep the disk free (the .mov is 1-2 GB)
BASE = 1.6e9   # ADVIO stamps are seconds since boot (0..300); added to every stamp so ns stamps exceed 1e12 and read_traj()/gnss_eval treat them as ns (pure offset, no effect on any system)
every = int(sys.argv[sys.argv.index('--every') + 1]) if '--every' in sys.argv else 2
z = zipfile.ZipFile(HTTPFile(f'https://zenodo.org/records/1476931/files/{nm}.zip?download=1'))   # remote zip by HTTP ranges, nothing but the output is stored
rd = lambda n: np.loadtxt(io.StringIO(z.read(f'{nm}/{n}').decode()), delimiter=',')
(out / 'cam0' / 'data').mkdir(parents=True, exist_ok=True); (out / 'imu0').mkdir(exist_ok=True)
g, a = rd('iphone/gyro.csv'), rd('iphone/accelerometer.csv')
g[:, 0] += BASE; a[:, 0] += BASE
ai = np.c_[[np.interp(g[:, 0], a[:, 0], a[:, k]) for k in (1, 2, 3)]].T
with open(out / 'imu0' / 'data.csv', 'w') as f:
    f.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
    for t, w, ac in zip(g[:, 0], g[:, 1:4], ai): f.write(f'{int(round(t*1e9))},{w[0]:.9f},{w[1]:.9f},{w[2]:.9f},{ac[0]:.9f},{ac[1]:.9f},{ac[2]:.9f}\n')
p = rd('ground-truth/pose.csv'); p[:, 0] += BASE   # t x y z qw qx qy qz
np.savetxt(out / 'gt.tum', np.c_[p[:, 0:4], p[:, 5:8], p[:, 4]], fmt='%.9f')
loc = np.atleast_2d(rd('iphone/platform-locations.csv')); loc[:, 0] += BASE   # columns: t, lat, lon, hAcc, alt, vAcc
with open(out / 'gnss_lla.csv', 'w') as f:
    f.write('t,lat,lon,alt,cov_xx,cov_yy,cov_zz,status\n')
    for r in np.atleast_2d(loc): f.write(f'{r[0]:.6f},{r[1]:.9f},{r[2]:.9f},{r[4]:.3f},{r[3]**2:.3f},{r[3]**2:.3f},{r[5]**2:.3f},0\n')
ft = np.loadtxt(io.StringIO(z.read(f'{nm}/iphone/frames.csv').decode()), delimiter=','); ft[:, 0] += BASE
tmp = out / '_frames.mov'; tmp.write_bytes(z.read(f'{nm}/iphone/frames.mov'))
cap = cv2.VideoCapture(str(tmp)); fc = open(out / 'cam0' / 'data.csv', 'w'); fc.write('#timestamp [ns], filename\n'); n = 0; k = 0
while True:
    ok, im = cap.read()
    if not ok: break
    if k % every == 0 and k < len(ft):
        tn = int(round(ft[k, 0] * 1e9)); cv2.imwrite(str(out / 'cam0' / 'data' / f'{tn}.jpg'), cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), [cv2.IMWRITE_JPEG_QUALITY, 95])
        fc.write(f'{tn},{tn}.jpg\n'); n += 1
    k += 1
fc.close(); tmp.unlink()
json.dump({'frames': n, 'video_frames': k, 'every': every, 't0': float(ft[0, 0]), 'base_s': BASE, 'duration': float(ft[-1, 0] - ft[0, 0])}, open(out / 'info.json', 'w'))
print('done', n, 'frames of', k, 'duration', ft[-1, 0] - ft[0, 0])
