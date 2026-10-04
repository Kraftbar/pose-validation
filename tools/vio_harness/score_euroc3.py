#!/usr/bin/env python3
"""Score the 2026-10-03 classical-candidate EuRoC runs (runs/vio_compare/<system>/<seq>) with tools/vio_eval.score (benchmark.umeyama_alignment) -> runs/vio_compare/table_classical.md.
usage: score_euroc3.py sys1 sys2 ..."""
import sys, json
sys.path.insert(0, '/home/nybo/github/pose-validation/tools')
import vio_eval as ve
SEQS = ['MH_01_easy', 'MH_03_medium', 'V1_02_medium', 'V2_02_medium']
f = lambda v, n=3: '-' if v is None else f'{v:.{n}f}'
rows = []
L = ['| system | seq | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage | span | CPU s | wall s | notes |', '|---|---|---|---|---|---|---|---|---|---|']
for s in sys.argv[1:]:
    for q in SEQS:
        tp = ve.OUT / s / q / 'trajectory.tum'   # drop a truncated final line of a crashed/unflushed run
        if tp.exists():
            ls = tp.read_text().splitlines(); ls = [l for l in ls if len(l.split()) == 8]; tp.write_text('\n'.join(ls) + '\n')
        try: r = ve.score(s, q)
        except Exception as e:
            L.append(f'| {s} | {q} | - | - | - | - | - | - | - | score error: {type(e).__name__} (partial/crashed trajectory) |'); continue
        if not r: continue
        run = json.loads((ve.OUT / s / q / 'run.json').read_text()); r['cpu_s'] = run.get('cpu_s'); rows.append(r)
        L.append(f"| {s} | {q} | {f(r['ate_se3'])} | {f(r['ate_sim3'])} | {f(r['scale'])} | {r['coverage']*100:.0f}% | {r.get('span_cov', 0)*100:.0f}% | {f(r['cpu_s'],0)} | {f(r['wall_s'],0)} | {r['notes']} (exit {r['exit_code']}) |")
open(ve.OUT / 'table_classical.md', 'w').write('\n'.join(L) + '\n'); json.dump(rows, open(ve.OUT / 'table_classical.json', 'w'), indent=1)
print('\n'.join(L))
