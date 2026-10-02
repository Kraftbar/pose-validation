#!/usr/bin/env python3
"""Mobile-GVIO (Zenodo 20525157, CC-BY-4.0) sequence zip -> EuRoC-style folder by streaming the zip over HTTP (nothing but the output is stored).
usage: mobilegvio_to_euroc.py <Outdoor-1> <out_dir> [max_seconds] [--every N]   (keeps every N-th camera frame, JPEG q95)
Writes cam0/data/*.jpg + data.csv, imu0/data.csv, gnss_lla.csv (t, lat, lon, alt, cov_xx, cov_yy, cov_zz, status), gt.tum (copied), info.json"""
import sys, json, zipfile
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).parent))
from http_zip import HTTPFile, ZipMemberStream
from bag_head import iter_msgs
seq, out = sys.argv[1], Path(sys.argv[2])
maxs = float(sys.argv[3]) if len(sys.argv) > 3 and not sys.argv[3].startswith('-') else 1e9
every = int(sys.argv[sys.argv.index('--every') + 1]) if '--every' in sys.argv else 1
url = f'https://zenodo.org/api/records/20525157/files/{seq}.zip/content'
z = zipfile.ZipFile(HTTPFile(url))
bagi = [i for i in z.infolist() if i.filename.endswith('.bag')][0]
(out / 'cam0' / 'data').mkdir(parents=True, exist_ok=True); (out / 'imu0').mkdir(exist_ok=True)
(out / 'gt.tum').write_bytes(z.read([i for i in z.infolist() if i.filename.endswith('ground_truth.txt')][0]))
st = ZipMemberStream(url, bagi.header_offset)
fc = open(out / 'cam0' / 'data.csv', 'w'); fc.write('#timestamp [ns], filename\n')
fi = open(out / 'imu0' / 'data.csv', 'w'); fi.write('#timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z\n')
fg = open(out / 'gnss_lla.csv', 'w'); fg.write('t,lat,lon,alt,cov_xx,cov_yy,cov_zz,status\n')
t0 = None; n = 0; k = 0
for topic, tb, m in iter_msgs(st, {'/cam0/image_raw', '/imu0', '/gnss0'}, size=bagi.file_size):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None: t0 = t
    if t - t0 > maxs: break
    tn = int(round(t * 1e9))
    if topic == '/imu0':
        a, w = m.linear_acceleration, m.angular_velocity
        fi.write(f'{tn},{w.x:.9f},{w.y:.9f},{w.z:.9f},{a.x:.9f},{a.y:.9f},{a.z:.9f}\n')
    elif topic == '/gnss0':
        c = m.position_covariance
        fg.write(f'{t:.6f},{m.latitude:.9f},{m.longitude:.9f},{m.altitude:.4f},{c[0]},{c[4]},{c[8]},{m.status.status}\n')
    else:
        k += 1
        if (k - 1) % every: continue
        enc = m.encoding
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
        if enc in ('bgr8', 'rgb8'): img = cv2.cvtColor(img.reshape(m.height, m.width, 3), cv2.COLOR_BGR2GRAY if enc == 'bgr8' else cv2.COLOR_RGB2GRAY)
        else: img = img[:, :m.width]
        cv2.imwrite(str(out / 'cam0' / 'data' / f'{tn}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 95]); fc.write(f'{tn},{tn}.jpg\n'); n += 1
for f in (fc, fi, fg): f.close()
json.dump({'t0': t0, 'frames': n, 'every': every, 'enc': enc}, open(out / 'info.json', 'w'))
print('done', n, 'frames', 'duration', t - t0)
