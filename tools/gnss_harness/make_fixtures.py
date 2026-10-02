#!/usr/bin/env python3
"""Gray PGM fixtures for stella_port sv_run (frame k -> <out>/%06d.pgm, order of <seq>/tum/rgb.txt). usage: make_fixtures.py <seq_dir> <out_dir>
(upstream's fixtures are the exact cvtColor(GRAY) frames; these data sets are 8-bit mono already, decoded with cv2.IMREAD_GRAYSCALE)"""
import sys
from pathlib import Path
import cv2
d, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
for i, l in enumerate([x for x in (d / 'tum' / 'rgb.txt').read_text().splitlines() if not x.startswith('#')]):
    ts = l.split()[1].split('/')[1][:-4]
    img = cv2.imread(str(d / 'mav0' / 'cam0' / 'data' / f'{ts}.png'), cv2.IMREAD_GRAYSCALE)
    cv2.imwrite(str(out / f'{i:06d}.pgm'), img)
print('fixtures', i + 1)
