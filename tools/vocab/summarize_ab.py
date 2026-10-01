#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Re-score all 30 complete runs without modifying canonical benchmark files."""
import json,sys,statistics
from common import ROOT,OUT
sys.path.insert(0,str(ROOT/'tools'))
import tum_eval
base=OUT/'ab';tum_eval.COMPARE_ROOT=base;rows=[];summary=[]
a=json.loads((base/'provenance_original.json').read_text());b=json.loads((base/'provenance_own.json').read_text())
for key in ['binary_sha256','libraries','configs','threads','workers','loop_closing']:
 if a[key]!=b[key]:raise RuntimeError('A/B configuration mismatch: '+key)
for seq in tum_eval.ALL_SEQS:
 data=tum_eval.DATA_ROOT/tum_eval.SEQ_DATA_DIRS[seq]
 rgb=tum_eval.read_rgb_frame_timestamps(seq);depth=[float(r[0]) for r in tum_eval.read_tum_list(data/'depth.txt')]
 expected=sum(min(abs(float(t)-d) for d in depth)<=0.1 for t in rgb)
 for variant in ['original','own']:
  runs=[]
  for rep in range(1,4):
   system=f'{variant}_run{rep}';folder=base/system/seq;info=json.loads((folder/'run.json').read_text())
   # This fixes the initial completeness check, which counted all RGB images
   # before accounting for the native example's RGB/depth pairing filter.
   info.update(loader_frames=expected,complete=info['processed_frames']==expected)
   (folder/'run.json').write_text(json.dumps(info,indent=2)+'\n')
   if info['exit_code']!=0 or not info['complete']:raise RuntimeError('incomplete '+str(folder))
   score=tum_eval.score_pair(system,seq);score.update(variant=variant,rep=rep,complete=True,loader_frames=expected)
   (folder/'score.json').write_text(json.dumps(score,indent=2)+'\n');rows.append(score);runs.append(score)
  valid=[r for r in runs if r['ate_tracked_m'] is not None]
  # Do not hide initialization failures behind a median of successful runs.
  chosen=sorted(valid,key=lambda x:x['ate_tracked_m'])[1] if len(valid)==3 else None
  summary.append(dict(sequence=seq,variant=variant,successful_runs=len(valid),median_ate_run=chosen,ate_range=[min(r['ate_tracked_m'] for r in valid),max(r['ate_tracked_m'] for r in valid)] if valid else None,coverage_range=[min(r['coverage'] for r in runs),max(r['coverage'] for r in runs)],wall_median=statistics.median(r['wall_s'] for r in runs)))
report=dict(runs=rows,summary=summary,selection='median-ATE run of three; coverage from that same run; range includes all three',mean_ate={v:statistics.mean(s['median_ate_run']['ate_tracked_m'] for s in summary if s['variant']==v) if all(s['median_ate_run'] for s in summary if s['variant']==v) else None for v in ['original','own']})
(base/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
lines=['| Sequence | Original ATE m / coverage | Own ATE m / coverage | Original ATE range | Own ATE range |','|---|---:|---:|---:|---:|']
for seq in tum_eval.ALL_SEQS:
 a,b=[next(s for s in summary if s['sequence']==seq and s['variant']==v) for v in ['original','own']]
 def value(s):
  r=s['median_ate_run'];return f"{r['ate_tracked_m']:.4f} / {100*r['coverage']:.1f}%" if r else 'failed run(s)'
 def spread(s):return '–'.join(f'{v:.4f}' for v in s['ate_range']) if s['ate_range'] else 'none'
 lines.append(f'| {seq} | {value(a)} | {value(b)} | {spread(a)} | {spread(b)} |')
lines+=['',f"Mean of per-sequence median ATEs: {report['mean_ate']}"]
(base/'table.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
