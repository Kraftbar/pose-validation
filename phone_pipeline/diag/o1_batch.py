#!/usr/bin/env python3
"""Run o1_run.py for a grid of (config, start) pairs with N workers. usage: o1_batch.py --workers 4 --skips 0,15,30 --cfg "tag|set1 set2|pp opts" ..."""
import sys, subprocess, argparse
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument('--workers', type=int, default=4); ap.add_argument('--skips', default='0'); ap.add_argument('--cfg', action='append', required=True)
ap.add_argument('--seq', default='outdoor1'); ap.add_argument('--stream', default=None); ap.add_argument('--variant', default='full')
a = ap.parse_args()
jobs = []
for sk in [int(x) for x in a.skips.split(',')]:
    for c in a.cfg:
        tag, sets, pps = (c.split('|') + ['', ''])[:3]
        t = f'{tag}_s{sk}'
        cmd = [sys.executable, str(HERE / 'o1_run.py'), t, '--live', '--seq', a.seq, '--variant', a.variant, '--skip', str(sk)]
        if a.stream: cmd += ['--stream', a.stream]
        for s in sets.split(): cmd += ['--set', s]
        if pps.strip(): cmd += ['--pp', pps]
        jobs.append(cmd)
def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True); print(cmd[2], r.stdout.strip().splitlines()[-1:] , flush=True)
with ThreadPoolExecutor(a.workers) as ex: list(ex.map(run, jobs))
