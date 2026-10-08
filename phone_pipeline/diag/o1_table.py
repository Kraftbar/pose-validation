#!/usr/bin/env python3
"""Paired table over starts for the Outdoor-1 blow-up study. usage: o1_table.py [--seq outdoor1] tag1 tag2 ...  (dirs runs/phone_pipeline/<seq>/blowup/<tag>_s<start>)"""
import sys, json, glob, re
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import run as R
import o1_score as OS
a = sys.argv[1:]; seq = 'outdoor1'
if a and a[0] == '--seq': seq = a[1]; a = a[2:]
tags = a
starts = sorted({int(re.search(r'_s(\d+)$', p).group(1)) for t in tags for p in glob.glob(str(R.OUT / seq / 'blowup' / f'{t}_s*')) if (Path(p) / 'pp.auto').exists() and (Path(p) / 'pp.timing').exists()})
res = {}
cache = R.OUT / seq / 'blowup' / 'table_cache.json'
C = json.loads(cache.read_text()) if cache.exists() else {}
for t in tags:
    for s in starts:
        d = R.OUT / seq / 'blowup' / f'{t}_s{s}'
        if not (d / 'pp.timing').exists(): continue
        k = f'{t}_s{s}'
        if k not in C: C[k] = OS.score(f'{t}_s{s}', seq=seq, skip=s)
        res[(t, s)] = C[k]
cache.write_text(json.dumps(C))
def f(x, n=2): return '  -  ' if x is None else f'{x:.{n}f}'
print(f'{seq}: causal AUTO ATE SE3 [m] / raw live map Sim3 ATE [m] / worst-step (max/min of the 12 s Sim3 scale from 24 s on) per start')
print('start  ' + ' | '.join(f'{t:>26}' for t in tags))
for s in starts:
    print(f'{s:5d}  ' + ' | '.join(f'{f(res.get((t, s), {}).get("auto")):>6} {f(res.get((t, s), {}).get("map_sim3"), 1):>6} {f(res.get((t, s), {}).get("worst_step"), 1):>6}   ' for t in tags))
print('mean   ' + ' | '.join(f'{np.mean([res[(t, s)]["auto"] for s in starts if (t, s) in res and res[(t, s)].get("auto") is not None]):6.2f} {np.mean([res[(t, s)]["map_sim3"] for s in starts if (t, s) in res and res[(t, s)].get("map_sim3") is not None]):6.1f} {np.mean([res[(t, s)]["worst_step"] for s in starts if (t, s) in res and "worst_step" in res[(t, s)]]) if any("worst_step" in res.get((t, s), {}) for s in starts) else float("nan"):6.1f}   ' for t in tags))
print('median ' + ' | '.join(f'{np.median([res[(t, s)]["auto"] for s in starts if (t, s) in res and res[(t, s)].get("auto") is not None]):6.2f} {np.median([res[(t, s)]["map_sim3"] for s in starts if (t, s) in res and res[(t, s)].get("map_sim3") is not None]):6.1f}          ' for t in tags))
b = tags[0]
for t in tags[1:]:
    ds = [(res[(t, s)]['auto'] - res[(b, s)]['auto']) for s in starts if (t, s) in res and (b, s) in res and res[(t, s)].get('auto') is not None and res[(b, s)].get('auto') is not None]
    print(f'{t} vs {b}: better (>0.02 m) in {sum(d < -0.02 for d in ds)}, worse in {sum(d > 0.02 for d in ds)}, equal {sum(abs(d) <= 0.02 for d in ds)} of {len(ds)} starts; mean diff {np.mean(ds):+.2f} m')
