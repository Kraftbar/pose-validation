#!/usr/bin/env python3
"""MARS-LVIG (HKU MaRS, CC BY-NC-SA 4.0) *_GNSS bag head -> EuRoC-style folder. Own code, no ROS.
usage: mars_bag_to_euroc.py <bag> <out_dir> [--max-s S] [--scale 2] [--dcam 0.0] [--drtk 0.306] [--skip-img]
Writes out_dir/mav0/{cam0/data/*.png, cam0/data.csv, imu0/data.csv, gps0/data.csv}, out_dir/gt_rtk.csv (t,E,N,U), out_dir/origin.json.
Clock: everything is put on the IMU/GNSS clock (Livox header stamp; GNSS PVT time = GPS time - 18 s). camera = header + dcam, RTK reference = header + drtk
(measured, see docs). Livox IMU accel is in g (x 9.80665). GNSS input = u-blox F9P receiver_pvt (standalone fix, independent of the DJI RTK reference).
IMU body frame = Livox IMU = lidar frame."""
import sys, os, json, argparse
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/gnss_harness')
from bag_head import iter_msgs

A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)
def lla2ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon); N = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.array([(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon), (N * (1 - E2) + h) * np.sin(lat)])
def enu_mat(lat, lon):
    lat, lon = np.radians(lat), np.radians(lon)
    return np.array([[-np.sin(lon), np.cos(lon), 0], [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
                     [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]])
G = 9.80665

ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('out')
ap.add_argument('--max-s', type=float, default=1e9); ap.add_argument('--scale', type=int, default=2)
ap.add_argument('--dcam', type=float, default=0.0); ap.add_argument('--drtk', type=float, default=0.306); ap.add_argument('--skip-img', action='store_true')
a = ap.parse_args()
out = Path(a.out); mav = out / 'mav0'
for d in ['cam0/data', 'imu0', 'gps0']: (mav / d).mkdir(parents=True, exist_ok=True)
fc = open(mav / 'cam0' / 'data.csv', 'w'); fc.write('#timestamp [ns], filename\n')
fi = open(mav / 'imu0' / 'data.csv', 'w'); fi.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
pvt, rtk = [], []; t0 = None; n = 0
topics = {'/left_camera/image/compressed', '/livox/imu', '/ublox_driver/receiver_pvt', '/dji_osdk_ros/rtk_position'}
size = os.path.getsize(a.bag)
flag = cv2.IMREAD_REDUCED_GRAYSCALE_2 if a.scale == 2 else cv2.IMREAD_GRAYSCALE
for topic, tb, m in iter_msgs(a.bag, topics, size=size):
    if topic == '/ublox_driver/receiver_pvt':
        pvt.append([315964800.0 + m.time.week * 604800 + m.time.tow - 18.0, m.latitude, m.longitude, m.altitude, m.fix_type, m.carr_soln, m.num_sv, m.h_acc, m.v_acc]); continue
    if topic == '/dji_osdk_ros/rtk_position':
        rtk.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + a.drtk, m.latitude, m.longitude, m.altitude]); continue
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if topic == '/livox/imu':
        if t0 is None: t0 = t
        if t - t0 > a.max_s: break
        w, ac = m.angular_velocity, m.linear_acceleration
        fi.write(f'{int(round(t * 1e9))},{w.x!r},{w.y!r},{w.z!r},{ac.x * G!r},{ac.y * G!r},{ac.z * G!r}\n')
    else:
        t += a.dcam; tn = int(round(t * 1e9))
        if a.skip_img: continue
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), flag)
        cv2.imwrite(str(mav / 'cam0' / 'data' / f'{tn}.png'), img, [cv2.IMWRITE_PNG_COMPRESSION, 1]); fc.write(f'{tn},{tn}.png\n'); n += 1
        if n % 200 == 0: print('img', n, flush=True)
fc.close(); fi.close()
P = np.array(pvt); R_ = np.array(rtk)
tend = (t0 + a.max_s)
P = P[(P[:, 0] >= t0 - 1) & (P[:, 0] <= tend + 1)]; R_ = R_[(R_[:, 0] >= t0 - 1) & (R_[:, 0] <= tend + 1)]
lla0 = P[0, 1:4]; Rm = enu_mat(*lla0[:2]); e0 = lla2ecef(*lla0)
enu = lambda X: np.array([Rm @ (lla2ecef(*r[1:4]) - e0) for r in X])
Pe, Re = enu(P), enu(R_)
np.savetxt(out / 'gt_rtk.csv', np.c_[R_[:, 0], Re], delimiter=',', fmt='%.6f', header='t_s,E,N,U')
np.savetxt(out / 'gnss_pvt.csv', np.c_[P[:, 0], Pe, P[:, 4:9]], delimiter=',', fmt='%.6f', header='t_s,E,N,U,fix,carr,nsv,h_acc,v_acc')
with open(mav / 'gps0' / 'data.csv', 'w') as g:
    g.write('timestamp, x, y, z, hErr1, hErr2, vErr\n')
    for r, p in zip(P, Pe): g.write(f'{int(round(r[0] * 1e9))},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{r[7]:.3f},{r[7]:.3f},{r[8]:.3f}\n')
json.dump({'lla0': lla0.tolist(), 't0': t0, 'n_img': n, 'dcam': a.dcam, 'drtk': a.drtk}, open(out / 'origin.json', 'w'))
print('done', n, 'imgs; pvt', len(P), 'rtk', len(R_))
