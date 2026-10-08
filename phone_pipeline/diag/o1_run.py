#!/usr/bin/env python3
"""Outdoor-1 blow-up study driver (docs/phone_outdoor1_scale_20261008.md): one sv_run / pp_live lock-step run over the JPEG fetch dir with just-in-time gray PGMs.
usage: o1_run.py <tag> [--frames N] [--live] [--variant full] [--stream DIR] [--set k=v ...] [--sv-extra "..."]  -> runs/phone_pipeline/outdoor1/blowup/<tag>/
Several tags can run at once (each has its own fixture dir; PGMs are written ahead of the process and deleted behind it)."""
import sys, os, re, time, subprocess, argparse, threading
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run as R

ap = argparse.ArgumentParser()
ap.add_argument('tag'); ap.add_argument('--seq', default='outdoor1'); ap.add_argument('--frames', type=int, default=-1)
ap.add_argument('--live', action='store_true'); ap.add_argument('--variant', default='full')
ap.add_argument('--stream', default=str(R.ROOT / 'runs/phone_pipeline/_fetch_o1/outdoor1'))
ap.add_argument('--set', action='append', default=[]); ap.add_argument('--pp', default='')
ap.add_argument('--ahead', type=int, default=250); ap.add_argument('--skip', type=int, default=0)
a = ap.parse_args()
c = R.cfg_of(a.seq); out = R.OUT / a.seq / 'blowup' / a.tag; out.mkdir(parents=True, exist_ok=True)
fx = out / '_fx'; fx.mkdir(exist_ok=True)
names = [l.split(',')[1].strip() for l in (Path(a.stream) / 'cam0/data.csv').read_text().splitlines()[1:] if l.strip()]
nmax = len(names) if a.frames < 0 else min(a.frames, len(names))
cmd = [str(R.PP_LIVE if a.live else R.SV_RUN), str(R.VOCAB), R.rp(c['rgb_dir']), str(fx), str(out)]
if a.frames >= 0: cmd += [str(a.frames)]
if a.skip: cmd += ['--skip', str(a.skip)]
cmd += ['--no-snap', '--lean', '--diag-log', str(out / 'diag.log'), '--wait-fixtures', '--size', c['size'], '--camera', c['camera'], '--imu', R.rp(c['imu']), '--imu-ext', str(out / 'ext.txt'),
        '--imu-toff', str(c['imu_toff']), '--imu-bg', ','.join(str(x) for x in c['imu_bg'])]
(out / 'ext.txt').write_text(' '.join(repr(float(x)) for x in c['imu_ext']) + '\n')
for s in R.SV_VARIANTS[a.variant] + a.set: cmd += ['--set', s]
if a.live:
    cmd += ['--live-out', str(out / 'live.tum'), '--servo-log', str(out / 'servo.log'), '--pp-out', str(out / 'pp'), '--pp-speed-out', str(out / 'pp.speed')]
    if c['fixes']: cmd += ['--pp-fix', R.rp(c['fixes'])]
    if c['gait']['mode'] == 'user': cmd += ['--pp-gait-c', repr(c['gait']['c'])]
    cmd += a.pp.split()
log = out / 'log.txt'
f = open(log, 'w'); p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT)
done = False
def feed():
    deleted = a.skip
    for i in range(a.skip, nmax if a.frames < 0 else min(nmax, a.skip + a.frames)):
        while True:
            if p.poll() is not None: return
            prog = R.last_frame(log)
            if prog < 0: prog = a.skip
            for j in range(deleted, max(deleted, prog - 3)): (fx / f'{j:06d}.pgm').unlink(missing_ok=True); deleted = j + 1
            if i - prog <= a.ahead: break
            time.sleep(0.2)
        img = cv2.imread(str(Path(a.stream) / 'cam0/data' / names[i]), cv2.IMREAD_GRAYSCALE)
        R.write_pgm(fx / f'{i:06d}.pgm', img)
th = threading.Thread(target=feed, daemon=True); th.start()
p.wait(); f.close()
import shutil; shutil.rmtree(fx, ignore_errors=True)
print(a.tag, 'rc', p.returncode, flush=True)
