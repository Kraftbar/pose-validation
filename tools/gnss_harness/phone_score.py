#!/usr/bin/env python3
"""Scoring for the 'more phone sequences' study (section 9). Reuses robust_score.py (gnss_eval.score on benchmark.umeyama_alignment) with the new sequences swapped in.
Reads external/gnss/rob/out/<seq>_<system>[_tag]/ for seq in indoor1 indoor2 outdoor2 advio15 advio20; writes runs/gnss_compare/phone_more/{table.md,table.json,<run>/{metrics.json,traj.txt}}.
GT: Mobile-GVIO gt.tum with the per-sequence clock offset from estimate_offset.py (OFFSET below; LiDAR-rig frame, so SE3 only is meaningful for IMU systems); ADVIO gt.tum with offset -0.315 s (advio20, estimate_offset.py)."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import robust_score as rs
from gnss_eval import GT
ROB = rs.ROB
OFFSET = json.loads((Path(__file__).parent / 'phone_offsets.json').read_text())   # {"indoor1": -282.522, ...}
def seqinfo(s):
    cam = [int(l.split(',')[0]) * 1e-9 for l in (ROB / s / 'cam0' / 'data.csv').read_text().splitlines()[1:] if l.strip()]
    return dict(dir=ROB / s, gt=lambda s=s: GT.from_tum(ROB / s / 'gt.tum', dt=OFFSET[s]), dur=cam[-1] - cam[0], rsa=[0, 0, 0])
rs.SEQ = {s: seqinfo(s) for s in OFFSET}
rs.OUT = Path('/home/nybo/github/pose-validation/runs/gnss_compare/phone_more')
_runs = rs.runs
rs.runs = lambda: [r for r in _runs() if r[1] in rs.SEQ and not r[2].startswith('fuse_')] + [r for r in _runs() if r[1] in rs.SEQ and r[2].startswith('fuse_')]
if __name__ == '__main__': rs.main()
