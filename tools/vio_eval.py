#!/usr/bin/env python3
"""Score VIO / camera-only runs on EuRoC against ground truth.

Layout: runs/vio_compare/<system>/<seq>/{trajectory.tum,run.json}; GT in runs/vio_compare/gt/ (tools/vio_prep_gt.py).
run.json: {"wall_s":..., "exit_code":..., "frame":"body"|"cam0", "notes":...}
Alignment is done by benchmark.umeyama_alignment / apply_alignment / ate_rmse (never reimplemented here):
  ate_sim3 = benchmark.ate_rmse (Sim3, scale free)       -- comparable to the mono-camera tables
  ate_se3  = same Umeyama with with_scale=False           -- metric-scale check for VIO
  scale    = the Sim3 scale factor (1.0 == perfect metric scale)
Metrics: coverage = estimated poses / input frames; trajectory time span vs sequence span (span_cov);
rtf = wall_s / sequence duration.  Writes runs/vio_compare/table.md + table.json.
"""
import json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from benchmark import ate_rmse, umeyama_alignment, apply_alignment  # noqa: E402

OUT = ROOT / 'runs' / 'vio_compare'
ASSOC = 0.005


def read_tum(p):
    rows = []
    for l in Path(p).read_text().splitlines():
        l = l.replace(',', ' ').strip()
        if not l or l[0] == '#':
            continue
        v = l.split()
        try:
            rows.append([float(x) for x in v[:8]])
        except ValueError:
            continue
    a = np.array(rows)
    if len(a) and a[0, 0] > 1e12:  # nanoseconds
        a[:, 0] *= 1e-9
    return a


def associate(est, gt):
    gts = gt[:, 0]
    idx = np.searchsorted(gts, est[:, 0])
    idx = np.clip(idx, 1, len(gts) - 1)
    left = np.abs(gts[idx - 1] - est[:, 0]) < np.abs(gts[idx] - est[:, 0])
    j = np.where(left, idx - 1, idx)
    ok = np.abs(gts[j] - est[:, 0]) <= ASSOC
    return est[ok, 1:4], gt[j[ok], 1:4]


def score(system, seq):
    d = OUT / system / seq
    if not (d / 'run.json').exists():
        return None
    run = json.loads((d / 'run.json').read_text())
    info = json.loads((OUT / 'gt' / f'{seq}_info.json').read_text())
    row = {'system': system, 'seq': seq, 'wall_s': run.get('wall_s'), 'exit_code': run.get('exit_code'), 'notes': run.get('notes', '')}
    row['rtf'] = run['wall_s'] / info['duration_s'] if run.get('wall_s') else None
    tp = d / 'trajectory.tum'
    est = read_tum(tp) if tp.exists() and tp.stat().st_size else np.zeros((0, 8))
    row['n_pose'] = int(len(est))
    row['coverage'] = len(est) / info['frames']
    row['ate_sim3'] = row['ate_se3'] = row['scale'] = None
    if len(est) >= 10:
        row['span_cov'] = float((est[-1, 0] - est[0, 0]) / info['duration_s'])
        gt = read_tum(OUT / 'gt' / f"{seq}_{run.get('frame', 'body')}.tum")
        e, g = associate(est, gt)
        row['n_assoc'] = int(len(e))
        if len(e) >= 10:
            r = ate_rmse(e, g)
            row['ate_sim3'], row['scale'] = r['ate_rmse'], r['scale']
            R, t, s = umeyama_alignment(e, g, with_scale=False)
            row['ate_se3'] = float(np.sqrt((np.linalg.norm(apply_alignment(e, R, t, s) - g, axis=1) ** 2).mean()))
    return row


def main():
    rows = []
    for sd in sorted(p for p in OUT.iterdir() if p.is_dir() and p.name != 'gt'):
        for qd in sorted(p for p in sd.iterdir() if p.is_dir()):
            r = score(sd.name, qd.name)
            if r:
                rows.append(r)
    f = lambda v, n=3: '-' if v is None else f'{v:.{n}f}'
    lines = ['| system | seq | ATE SE3 (m) | ATE Sim3 (m) | Sim3 scale | coverage | wall (s) | RTF | notes |', '|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f"| {r['system']} | {r['seq']} | {f(r['ate_se3'])} | {f(r['ate_sim3'])} | {f(r['scale'])} | {r['coverage']*100:.0f}% | {f(r['wall_s'],1)} | {f(r['rtf'],2)} | {r['notes']} |")
    (OUT / 'table.md').write_text('\n'.join(lines) + '\n')
    (OUT / 'table.json').write_text(json.dumps(rows, indent=1))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
