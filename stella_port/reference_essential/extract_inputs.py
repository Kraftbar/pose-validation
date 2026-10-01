#!/usr/bin/env python3
"""Extract only teacher-forced robust fallback inputs; no reference mutation."""
import csv,json,itertools,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/stella_port/reference_essential/fixtures'
def rows(p):
 with p.open() as f:yield from csv.DictReader(f,delimiter='\t')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for seq in ['fr1_xyz','fr1_desk']:
 base=ROOT/'runs/stella_port/reference_dumps_force_bow'/seq
 dest=OUT/seq;dest.mkdir(parents=True,exist_ok=False)
 traces={int(r['frame_idx']):r for r in rows(base/'frame_trace.tsv') if r['track_path']=='robust_match'}
 pre={int(r['frame_idx']):r for r in rows(base/'track_pre.tsv')}
 time_to_frame={r['timestamp_hex']:int(r['frame_idx']) for r in pre.values()}
 kf_frame={int(r['kf_id']):time_to_frame[r['timestamp_hex']] for r in rows(base/'keyframe_meta.tsv')}
 refs={t:int(pre[t]['last_frm_ref_keyfrm_id']) for t in traces}
 needed=set(traces)|{kf_frame[k] for k in refs.values()}
 points={t:[] for t in needed};desc={t:[] for t in needed}
 for r in rows(base/'keypoints.tsv'):
  t=int(r['frame_idx'])
  if t in needed:
   assert int(r['kp_idx'])==len(points[t]);points[t].append([r['x_hex'],r['y_hex'],r['angle_hex']])
 for r in rows(base/'descriptors.tsv'):
  t=int(r['frame_idx'])
  if t in needed:
   assert int(r['kp_idx'])==len(desc[t]);desc[t].append(r['descriptor_hex'])
 attachments={t:{} for t in traces}
 for r in rows(base/'landmarks.tsv'):
  t=int(r['frame_idx'])+1
  if t not in traces:continue
  for item in r['observations'].split(','):
   if not item:continue
   k,i=map(int,item.split(':'))
   if k==refs[t]:attachments[t][i]=int(r['lm_id'])
 for t in traces:
  kf=kf_frame[refs[t]]
  with (dest/f'{t}.input').open('w') as out:
   out.write(f'{t} {len(points[t])} {len(points[kf])}\n')
   for side,fid in enumerate([t,kf]):
    for i,p in enumerate(points[fid]):
     lm=attachments[t].get(i,-1) if side else -1
     out.write(' '.join([*p,desc[fid][i],str(lm)])+'\n')
 names=['frame_trace.tsv','track_pre.tsv','keyframe_meta.tsv','keypoints.tsv','descriptors.tsv','landmarks.tsv']
 provenance={'frames':sorted(traces),'source_sha256':{str(base/n):sha(base/n) for n in names},'input_sha256':{p.name:sha(p) for p in dest.glob('*.input')}}
 (dest/'inputs.json').write_text(json.dumps(provenance,indent=2)+'\n')
 print(seq,len(traces),flush=True)
