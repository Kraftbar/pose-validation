#!/usr/bin/env python3
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
"""[--keep] [--tum] Gray PGM fixtures for sv_run from a fetched sequence (external/gnss/rob/<seq>/cam0/{data.csv,data/*}); frame k -> <out>/%06d.pgm in data.csv order.
Raw images are deleted as they are converted (disk). usage: make_fixtures.py <seq_dir> <out_dir>"""
import sys, os
from pathlib import Path
import cv2
args = [a for a in sys.argv[1:] if not a.startswith('--')]; keep = '--keep' in sys.argv or '--tum' in sys.argv
d, out = Path(args[0]), Path(args[1]); out.mkdir(parents=True, exist_ok=True)
if '--tum' in sys.argv:  # TUM layout: rgb.txt + rgb/*.png (never deleted)
    rd = lambda n: [l.split() for l in (d / n).read_text().splitlines() if l.strip() and l[0] != '#']
    dt = [float(r[0]) for r in rd('depth.txt')]; names = []
    for r in rd('rgb.txt'):  # same association as sv_run: nearest depth frame, kept only if within 0.1 s (frames are numbered after this filter)
        t = float(r[0]); md = min(abs(t - x) for x in dt)
        if md <= 0.1: names.append(r[1])
    import shutil  # the exact fixtures of the port are the C++ OpenCV gray conversions in <seq>/pgm_cache/<timestamp>.pgm (cv2 5.x converts differently)
    for i, nme in enumerate(names):
        c = d / 'pgm_cache' / (Path(nme).stem + '.pgm')
        if c.exists(): shutil.copyfile(c, out / f'{i:06d}.pgm')
        else: img = cv2.imread(str(d / nme), cv2.IMREAD_GRAYSCALE); cv2.imwrite(str(out / f'{i:06d}.pgm'), img)
    print('fixtures', len(names)); sys.exit(0)
rows = [l.split(',') for l in (d / 'cam0' / 'data.csv').read_text().splitlines()[1:] if l.strip()]
for i, r in enumerate(rows):
    p = d / 'cam0' / 'data' / r[1].strip()
    if not p.exists(): print('missing', p); continue
    img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
    cv2.imwrite(str(out / f'{i:06d}.pgm'), img)
    if not keep: os.unlink(p)
print('fixtures', len(rows), img.shape)
