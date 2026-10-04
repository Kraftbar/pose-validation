#!/usr/bin/env python3
"""Aggregate runs/blocks/poselib/res.csv (blocks_eval output) into markdown tables. usage: blocks_report.py res.csv"""
import sys, csv, collections
import numpy as np
rows = list(csv.DictReader(open(sys.argv[1])))
BASE = {}
_cur = None
for l in open(sys.argv[2] if len(sys.argv) > 2 else '/home/nybo/github/pose-validation/runs/blocks/poselib/pairs.txt'):
    if l.startswith('PAIR'): _cur = tuple(l.split()[1:4])
    elif l.startswith('GT'): v = l.split(); BASE[_cur] = float(np.linalg.norm([float(x) for x in v[10:13]]))
for r in rows: r['base'] = BASE.get((r['seq'], r['i'], r['j']), float('nan'))
def f(r, k): return float(r[k])
def agg(sel, label_fn, task):
    D = collections.defaultdict(list)
    for r in rows:
        if r['task'] != task or not sel(r): continue
        D[(label_fn(r), r['solver'])].append(r)
    return D
def table_rel(sel, title):
    D = agg(sel, lambda r: 'all', 'rel'); out = [f'**{title}**', '', '| solver | pairs | returned a pose | accepted (stella-style check) | rot<2deg of all pairs | rot<2deg and dir<20deg of all pairs | accepted AND rot<2 AND dir<20 | rot err median (deg, returned) | lateral t err median (cm @ GT baseline) | median time (ms) |', '|---|---|---|---|---|---|---|---|---|---|']
    for (lab, s), rs in sorted(D.items(), key=lambda kv: kv[0][1]):
        n = len(rs); ok = [r for r in rs if int(r['ok'])]; acc = [r for r in rs if int(r['ok']) and int(r['accepted'])]
        good = [r for r in ok if f(r, 'rot_err') < 2]; good2 = [r for r in good if f(r, 'dir_err') < 20]; gacc = [r for r in good2 if int(r['accepted'])]
        re = np.median([f(r, 'rot_err') for r in ok]) if ok else float('nan')
        out.append(f"| {s} | {n} | {len(ok)/n:.0%} | {len(acc)/n:.0%} | {len(good)/n:.0%} | {len(good2)/n:.0%} | {len(gacc)/n:.0%} | {re:.2f} | - | {np.median([f(r,'us') for r in rs])/1000:.1f} |")
    return '\n'.join(out)
def table_gap():
    D = collections.defaultdict(list)
    for r in rows:
        if r['task'] == 'rel': D[(int(r['gap']), r['solver'])].append(r)
    gaps = sorted({g for g, _ in D}); solvers = sorted({s for _, s in D})
    out = ['| solver | ' + ' | '.join(f'gap {g}' for g in gaps) + ' |', '|---|' + '---|' * len(gaps)]
    for s in solvers:
        cells = []
        for g in gaps:
            rs = D[(g, s)]; good = [r for r in rs if int(r['ok']) and int(r['accepted']) and f(r, 'rot_err') < 2 and f(r, 'dir_err') < 20]
            med = np.median([f(r, 'rot_err') for r in rs if int(r['ok'])]) if any(int(r['ok']) for r in rs) else float('nan')
            cells.append(f'{len(good)/len(rs):.0%} ({med:.2f})')
        out.append(f'| {s} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(out)
def table_pnp():
    D = collections.defaultdict(list)
    for r in rows:
        if r['task'] == 'pnp': D[(float(r['param']), r['solver'])].append(r)
    fr = sorted({p for p, _ in D}); solvers = sorted({s for _, s in D})
    out = ['| solver | ' + ' | '.join(f'+{int(p*100)}% outliers' for p in fr) + ' | median time ms (0% / 85%) |', '|---|' + '---|' * len(fr) + '---|']
    for s in solvers:
        cells = []
        for p in fr:
            rs = D[(p, s)]; good = [r for r in rs if int(r['ok']) and f(r, 'rot_err') < 2 and f(r, 'pos_err') < 0.05]
            cells.append(f'{len(good)/len(rs):.0%}')
        t0 = np.median([f(r, 'us') for r in D[(fr[0], s)]]) / 1000; t1 = np.median([f(r, 'us') for r in D[(fr[-1], s)]]) / 1000
        out.append(f'| {s} | ' + ' | '.join(cells) + f' | {t0:.1f} / {t1:.1f} |')
    return '\n'.join(out)
def table_pnp_err():
    D = collections.defaultdict(list)
    for r in rows:
        if r['task'] == 'pnp' and float(r['param']) == 0.0 and int(r['ok']): D[r['solver']].append(r)
    out = ['| solver | median rot err (deg) | median camera-centre err (cm) | p95 centre err (cm) |', '|---|---|---|---|']
    for s, rs in sorted(D.items()):
        out.append(f"| {s} | {np.median([f(r,'rot_err') for r in rs]):.2f} | {100*np.median([f(r,'pos_err') for r in rs]):.2f} | {100*np.percentile([f(r,'pos_err') for r in rs],95):.1f} |")
    return '\n'.join(out)
def table_base():
    bk = [(0, .05, '<5 cm'), (.05, .15, '5-15 cm'), (.15, .30, '15-30 cm'), (.30, 9, '>30 cm')]
    solvers = sorted({r['solver'] for r in rows if r['task'] == 'rel'})
    out = ['| solver | ' + ' | '.join(f'{b[2]} (n={len({(r["seq"],r["i"],r["j"]) for r in rows if r["task"]=="rel" and b[0]<=r["base"]<b[1]})})' for b in bk) + ' |', '|---|' + '---|' * len(bk)]
    for s_ in solvers:
        cells = []
        for lo, hi, _ in bk:
            rs = [r for r in rows if r['task'] == 'rel' and r['solver'] == s_ and lo <= r['base'] < hi]
            acc = [r for r in rs if int(r['ok']) and int(r['accepted'])]; good = [r for r in acc if f(r, 'rot_err') < 2]
            dirs = [f(r, 'dir_err') for r in good]
            cells.append(f"{len(good)/max(1,len(rs)):.0%} / acc {len(acc)/max(1,len(rs)):.0%} / dir {np.median(dirs) if dirs else float('nan'):.0f}")
        out.append(f'| {s_} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(out)
print(table_rel(lambda r: True, 'Relative pose / init, all pairs with >= 50 matches'))
print(); print('**By GT baseline: success (accepted AND rot<2deg) / accepted / median direction error (deg) of those**'); print(); print(table_base())
print(); print('**Success rate by frame gap (accepted AND rot<2deg AND dir<20deg; in brackets: median rotation error deg of returned poses)**'); print(); print(table_gap())
print(); print('**PnP / relocalisation success (rot<2deg and camera centre<5cm) vs injected outlier fraction (on top of the natural ORB outliers)**'); print(); print(table_pnp())
print(); print('**PnP accuracy on the naturally contaminated set (0% injected)**'); print(); print(table_pnp_err())
