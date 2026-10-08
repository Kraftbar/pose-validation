#!/usr/bin/env python3
"""GVINS-Dataset bag (full or truncated head) -> EuRoC-style folder + GNSS/GT csv. Own code, no ROS.
usage: gvins_bag_to_euroc.py <bag file or https URL (streamed)> <out_dir> [max_seconds] [--stereo]
Writes: cam0/data/*.png + cam0/data.csv, imu0/data.csv, gps0/data.csv (cartesian ENU, ns, UTC-aligned),
        gt_pvt.csv (t_s, E, N, U, fix_type, carr_soln, num_sv, h_acc, v_acc), origin.json (lla0).
Time: sensor stamps are UTC unix; GNSS stamps (PVT week/tow) are GPS time = UTC + 18 s (leap seconds, Jan 2021) -> subtract 18."""
import sys, json
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).parent))
from bag_head import iter_msgs

LEAP = 18.0262  # GPS-UTC leap (18 s) + measured local-clock offset 26.2 ms (GVINS PPS sync, see docs); was 18.0
_LEAP_OLD = 18.0
GPS_EPOCH_UNIX = 315964800.0
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def lla2ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon)
    N = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.array([(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon), (N * (1 - E2) + h) * np.sin(lat)])


def ecef2enu_mat(lat, lon):
    lat, lon = np.radians(lat), np.radians(lon)
    return np.array([[-np.sin(lon), np.cos(lon), 0],
                     [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
                     [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]])


def main():
    bag, out = sys.argv[1], Path(sys.argv[2])
    maxs = float(sys.argv[3]) if len(sys.argv) > 3 and not sys.argv[3].startswith('-') else 1e9
    stereo = '--stereo' in sys.argv
    cams = ['cam0'] + (['cam1'] if stereo else [])
    for c in cams: (out / c / 'data').mkdir(parents=True, exist_ok=True)
    (out / 'imu0').mkdir(parents=True, exist_ok=True); (out / 'gps0').mkdir(exist_ok=True)
    camcsv = {c: open(out / c / 'data.csv', 'w') for c in cams}
    for f in camcsv.values(): f.write('#timestamp [ns], filename\n')
    imu = open(out / 'imu0' / 'data.csv', 'w'); imu.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
    pvt = []; t0 = None
    sz = None
    if str(bag).startswith('http'):  # stream from the URL (HTTP range requests), nothing but the output is stored
        import io; from http_zip import HTTPFile
        raw = HTTPFile(bag); bag = io.BufferedReader(raw, buffer_size=1 << 20); sz = raw.size
    for topic, tbag, m in iter_msgs(bag, {'/cam0/image_raw', '/cam1/image_raw', '/imu0', '/ublox_driver/receiver_pvt'}, size=sz):
        if topic == '/ublox_driver/receiver_pvt':
            t = GPS_EPOCH_UNIX + m.time.week * 604800 + m.time.tow - LEAP
            pvt.append([t, m.latitude, m.longitude, m.altitude, m.fix_type, m.carr_soln, m.num_sv, m.h_acc, m.v_acc, m.p_dop, m.valid_fix])
            continue
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t0 is None: t0 = t
        if t - t0 > maxs: break
        tn = int(round(t * 1e9))
        if topic == '/imu0':
            a, w = m.linear_acceleration, m.angular_velocity
            imu.write(f'{tn},{w.x!r},{w.y!r},{w.z!r},{a.x!r},{a.y!r},{a.z!r}\n')
        else:
            c = topic[1:5]
            if c not in cams: continue
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
            cv2.imwrite(str(out / c / 'data' / f'{tn}.png'), img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            camcsv[c].write(f'{tn},{tn}.png\n')
    for f in camcsv.values(): f.close()
    imu.close()
    P = np.array(pvt, float)
    P = P[(P[:, 0] >= t0 - 1) & (P[:, 0] <= t0 + min(maxs, 1e9) + 1)]
    P = P[P[:, 10] > 0]  # valid fixes only
    lla0 = P[0, 1:4]; R = ecef2enu_mat(*lla0[:2]); e0 = lla2ecef(*lla0)
    enu = np.array([R @ (lla2ecef(*r[1:4]) - e0) for r in P])
    np.savetxt(out / 'gt_pvt.csv', np.c_[P[:, 0], enu, P[:, 4:10]], delimiter=',', fmt='%.6f',
               header='t_s,E,N,U,fix_type,carr_soln,num_sv,h_acc,v_acc,pdop')
    with open(out / 'gps0' / 'data.csv', 'w') as g:
        g.write('timestamp, x, y, z, hErr1, hErr2, vErr\n')
        for t, p, r in zip(P[:, 0], enu, P):
            g.write(f'{int(round(t * 1e9))},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{r[7]:.3f},{r[7]:.3f},{r[8]:.3f}\n')
    json.dump({'lla0': lla0.tolist(), 'bag': str(bag), 't0': t0, 'n_pvt': len(P)}, open(out / 'origin.json', 'w'))
    print('done', out, 'span_s', (P[-1, 0] - P[0, 0]), 'pvt', len(P))

main()
