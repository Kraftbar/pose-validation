#!/usr/bin/env python3
"""Build the folders the study's drivers expect from a fetched sequence (external/gnss/rob/<seq>/{cam0,imu0}) -- symlinks only, no image copies.
 mav0/cam0/data.csv + data/<ts>.png (symlink to the real file; jpg content behind a .png name is fine for cv::imread), mav0/imu0/data.csv,
 times.txt (ORB-SLAM3 timestamp list), tum/{rgb,depth}.txt (for sv_run, 'timestamp file' format), usage: make_layout.py <seq_dir> [imu_csv]"""
import sys, os
from pathlib import Path
d = Path(sys.argv[1]).resolve(); imu = Path(sys.argv[2]) if len(sys.argv) > 2 else d / 'imu0' / 'data.csv'
(d / 'mav0' / 'cam0' / 'data').mkdir(parents=True, exist_ok=True); (d / 'mav0' / 'imu0').mkdir(exist_ok=True); (d / 'tum').mkdir(exist_ok=True)
rows = [l.strip().split(',') for l in (d / 'cam0' / 'data.csv').read_text().splitlines()[1:] if l.strip()]
with open(d / 'mav0' / 'cam0' / 'data.csv', 'w') as f, open(d / 'times.txt', 'w') as ft, open(d / 'tum' / 'rgb.txt', 'w') as fr, open(d / 'tum' / 'depth.txt', 'w') as fd:
    f.write('#timestamp [ns], filename\n')
    for h in (fr, fd): h.write('# TUM-style index\n# (sv_run skips the first 3 lines of rgb.txt / depth.txt, like the TUM files)\n# timestamp filename\n')
    for ts, fn in rows:
        fn = fn.strip(); l = d / 'mav0' / 'cam0' / 'data' / f'{ts}.png'
        if not l.exists() and not l.is_symlink(): os.symlink(d / 'cam0' / 'data' / fn, l)
        f.write(f'{ts},{ts}.png\n'); ft.write(f'{ts}\n'); t = int(ts) * 1e-9
        fr.write(f'{t:.9f} rgb/{ts}.png\n'); fd.write(f'{t:.9f} depth/{ts}.png\n')
dst = d / 'mav0' / 'imu0' / 'data.csv'
if not dst.exists(): os.symlink(imu.resolve(), dst)
print('layout', d, len(rows), 'frames')
