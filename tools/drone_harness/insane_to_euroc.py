#!/usr/bin/env python3
"""INSANE dataset (Univ. Klagenfurt CNS, BSD-2 + no-selling condition) sequence -> EuRoC layout; the nav-cam images are streamed out of the remote zip
(HTTP range requests, stored PNG members), decoded, halved to 1028x771 and written as PNG; nothing else is kept.
usage: insane_to_euroc.py <seq e.g. mars_14|outdoor_1|mars_9> <out_dir> [--sensors-dir D] [--max-s S] [--skip-img]
Writes out_dir/mav0/{cam0,imu0,gps0}, out_dir/gt_enu.tum (vehicle-centre pose in the GNSS ENU frame; fitted from the dual-RTK antenna track), out_dir/gt_local.tum,
out_dir/gnss_px4.csv. IMU = PX4 IMU (the calibration reference frame); GNSS input = PX4 GPS (consumer receiver, reported sigma 3-9 m)."""
import sys, io, json, argparse, zipfile
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/gnss_harness'); sys.path.insert(0, '/home/nybo/github/pose-validation')
from http_zip import HTTPFile
from benchmark import umeyama_alignment

ap = argparse.ArgumentParser(); ap.add_argument('seq'); ap.add_argument('out'); ap.add_argument('--sensors-dir', default=None)
ap.add_argument('--max-s', type=float, default=1e9); ap.add_argument('--skip-img', action='store_true'); a = ap.parse_args()
sd = Path(a.sensors_dir or f'/home/nybo/github/pose-validation/external/drone/insane/{a.seq}_sensors'); out = Path(a.out); mav = out / 'mav0'
for d in ['cam0/data', 'imu0', 'gps0']: (mav / d).mkdir(parents=True, exist_ok=True)
ref = [float(x) for x in open(sd / 'README.txt').read().split('=')[1].split('\n')[0].strip(' []').split(',')]
A, F = 6378137.0, 1 / 298.257223563; E2 = F * (2 - F)
def lla2ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon); N = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.array([(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon), (N * (1 - E2) + h) * np.sin(lat)])
lat0, lon0 = np.radians(ref[0]), np.radians(ref[1])
Ren = np.array([[-np.sin(lon0), np.cos(lon0), 0], [-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)], [np.cos(lat0) * np.cos(lon0), np.cos(lat0) * np.sin(lon0), np.sin(lat0)]])
e0 = lla2ecef(*ref)
enu = lambda lla: np.array([Ren @ (lla2ecef(*r) - e0) for r in lla])
imu = np.loadtxt(sd / 'px4_imu.csv', delimiter=',', skiprows=1)
gt = np.loadtxt(sd / 'ground_truth' / 'ground_truth_80hz.csv', delimiter=',', skiprows=1)
t0 = max(imu[0, 0], gt[0, 0]); t1 = min(imu[-1, 0], gt[-1, 0], t0 + a.max_s)
imu = imu[(imu[:, 0] >= t0 - 0.5) & (imu[:, 0] <= t1 + 0.5)]
with open(mav / 'imu0' / 'data.csv', 'w') as f:
    f.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
    for r in imu: f.write(f'{int(round(r[0] * 1e9))},{r[4]:.9g},{r[5]:.9g},{r[6]:.9g},{r[1]:.9g},{r[2]:.9g},{r[3]:.9g}\n')
# GNSS (PX4 receiver)
g = np.loadtxt(sd / 'px4_gps.csv', delimiter=',', skiprows=1); g = g[(g[:, 0] >= t0 - 1) & (g[:, 0] <= t1 + 1)]
pe = enu(g[:, 1:4])
np.savetxt(out / 'gnss_px4.csv', np.c_[g[:, 0], pe, g[:, 7:10]], delimiter=',', fmt='%.5f', header='t_s,E,N,U,cov_x,cov_y,cov_z')
with open(mav / 'gps0' / 'data.csv', 'w') as f:
    f.write('timestamp, x, y, z, hErr1, hErr2, vErr\n')
    for r, p in zip(g, pe): f.write(f'{int(round(r[0] * 1e9))},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{np.sqrt(r[7]):.3f},{np.sqrt(r[8]):.3f},{np.sqrt(r[9]):.3f}\n')
# reference: fit the GT local frame to the global ENU frame with the dual-RTK antenna midpoint (the GT is derived from the same RTK pair)
r1 = np.loadtxt(sd / 'ground_truth' / 'rtk_gps1_data_revised.csv', delimiter=','); r2 = np.loadtxt(sd / 'ground_truth' / 'rtk_gps2_data_revised.csv', delimiter=',')  # GT time base (lat, lon, alt)
P1, P2 = enu(r1[:, 1:4]), enu(r2[:, 1:4]); mid_t = r1[:, 0]; p2 = np.array([np.interp(mid_t, r2[:, 0], P2[:, k]) for k in range(3)]).T
mid = 0.5 * (P1 + p2); ok = (mid_t > gt[0, 0] + 1) & (mid_t < gt[-1, 0] - 1)
gi = np.array([np.interp(mid_t[ok], gt[:, 0], gt[:, 1 + k]) for k in range(3)]).T
R, tt, s = umeyama_alignment(gi, mid[ok], with_scale=False)
res = np.linalg.norm(gi @ R.T + tt - mid[ok], axis=1); print('GT local->ENU fit: yaw', np.degrees(np.arctan2(R[1, 0], R[0, 0])), 'rms', np.sqrt((res ** 2).mean()), 'tilt', np.degrees(np.arccos(R[2, 2])))
def qmat(q):
    w, x, y, z = q; return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def mq(Rm):
    t = np.trace(Rm)
    if t > 0: s = np.sqrt(t + 1) * 2; return [(Rm[2, 1] - Rm[1, 2]) / s, (Rm[0, 2] - Rm[2, 0]) / s, (Rm[1, 0] - Rm[0, 1]) / s, s / 4]
    i = np.argmax(np.diag(Rm)); j, k = (i + 1) % 3, (i + 2) % 3; s = np.sqrt(1 + Rm[i, i] - Rm[j, j] - Rm[k, k]) * 2; q = [0] * 4; q[i] = s / 4; q[3] = (Rm[k, j] - Rm[j, k]) / s; q[j] = (Rm[j, i] + Rm[i, j]) / s; q[k] = (Rm[k, i] + Rm[i, k]) / s; return q
with open(out / 'gt_enu.tum', 'w') as f:  # position-only reference: dual-RTK midpoint (= vehicle centre), 7 Hz, ENU of the GNSS reference point
    for tt_, p in zip(mid_t, mid):
        if t0 <= tt_ <= t1: f.write(f'{tt_:.6f} {p[0]:.5f} {p[1]:.5f} {p[2]:.5f} 0 0 0 1\n')
with open(out / 'gt_local.tum', 'w') as f2:  # 80 Hz dataset GT in its own local frame (diagnostics only)
    for r in gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]: f2.write(f'{r[0]:.6f} {r[1]:.5f} {r[2]:.5f} {r[3]:.5f} {r[5]:.6f} {r[6]:.6f} {r[7]:.6f} {r[4]:.6f}\n')
json.dump({'ref_lla': ref, 't0': t0, 't1': t1, 'fit_rms': float(np.sqrt((res ** 2).mean()))}, open(out / 'origin.json', 'w'))
# images: parallel per-member range requests (the server gives ~0.5 MB/s per connection, so many connections)
if not a.skip_img:
    import struct, time, urllib.request, concurrent.futures as cf
    URL = f'https://cns-data.aau.at/insane-dataset/{a.seq}_nav_cam.zip'
    z = zipfile.ZipFile(HTTPFile(URL, chunk=8 << 20))
    ts = [l.split(',') for l in z.read(f'{a.seq}_nav_cam/nav_cam_timestamps.csv').decode().splitlines() if l and l[0] != '#']
    info = {i.filename: i for i in z.infolist()}
    sel = [(float(t), fn.strip()) for _, t, fn in ts if t0 <= float(t) <= t1]
    def work(item):
        t, fn = item; zi = info[f'{a.seq}_nav_cam/img/{fn}.png']; tn = int(round(t * 1e9)); p = mav / 'cam0' / 'data' / f'{tn}.png'
        if p.exists(): return tn
        for k in range(30):
            try:
                n = zi.file_size + 30 + len(zi.filename) + 400
                r = urllib.request.urlopen(urllib.request.Request(URL, headers={'Range': f'bytes={zi.header_offset}-{zi.header_offset + n - 1}'}), timeout=120).read()
                nl, el = struct.unpack('<HH', r[26:30]); raw = r[30 + nl + el:30 + nl + el + zi.file_size]
                if len(raw) != zi.file_size: raise RuntimeError('short')
                im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
                im = cv2.resize(im, (im.shape[1] // 2, im.shape[0] // 2), interpolation=cv2.INTER_AREA); cv2.imwrite(str(p), im, [cv2.IMWRITE_PNG_COMPRESSION, 1]); return tn
            except Exception as e:
                time.sleep(3)
        raise RuntimeError('failed ' + fn)
    with cf.ThreadPoolExecutor(32) as ex: tns = list(ex.map(work, sel))
    with open(mav / 'cam0' / 'data.csv', 'w') as fc:
        fc.write('#timestamp [ns], filename\n')
        for tn in tns: fc.write(f'{tn},{tn}.png\n')
    print('images', len(tns))
