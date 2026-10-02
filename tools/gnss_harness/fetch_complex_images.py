#!/usr/bin/env python3
"""Stream the first N seconds of the GVINS-Dataset complex_environment bag over HTTP range requests (nothing stored but the output) and write only
cam0 as PNG + data.csv (same names/timestamps as gvins_bag_to_euroc.py). usage: fetch_complex_images.py <out_dir> [max_seconds=436]"""
import sys
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).parent))
from http_zip import HTTPFile
from bag_head import iter_msgs
out = Path(sys.argv[1]); maxs = float(sys.argv[2]) if len(sys.argv) > 2 else 436.0
url = 'https://huggingface.co/datasets/Shawn202606/GVINS-Dataset/resolve/main/complex_environment.bag'
import io
raw = HTTPFile(url); f = io.BufferedReader(raw, buffer_size=1 << 20)
(out / 'cam0' / 'data').mkdir(parents=True, exist_ok=True)
fc = open(out / 'cam0' / 'data.csv', 'w'); fc.write('#timestamp [ns], filename\n')
t0 = None; n = 0
for topic, tb, m in iter_msgs(f, {'/cam0/image_raw'}, size=raw.size):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None: t0 = t
    if t - t0 > maxs: break
    tn = int(round(t * 1e9))
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
    cv2.imwrite(str(out / 'cam0' / 'data' / f'{tn}.png'), img, [cv2.IMWRITE_PNG_COMPRESSION, 1]); fc.write(f'{tn},{tn}.png\n'); n += 1
    if n % 500 == 0: print(n, t - t0, flush=True)
fc.close(); print('done', n, 't0', t0)
