#!/usr/bin/env python3
"""Immutable teacher-forced map inputs from existing native tracking snapshots."""
import csv,json,struct,hashlib
from pathlib import Path
from build import ROOT,OUT,sha

def rows(p):
 with p.open() as f:yield from csv.DictReader(f,delimiter='\t')
def pairs(s):return [tuple(map(int,v.split(':'))) for v in s.split(',') if v]
def hexes(s):return [float.fromhex(v) for v in s.split(',')]
for seq,frames in [('fr1_xyz',[80,150,250,400,550,700]),('fr1_desk',[80,110,150,250,400,550])]:
 src=ROOT/'runs/stella_port/reference_dumps'/seq;dest=OUT/'fixtures'/seq;dest.mkdir(parents=True,exist_ok=False)
 pre={int(r['frame_idx']):r for r in rows(src/'track_pre.tsv')}
 bytime={r['timestamp_hex']:i for i,r in pre.items()}
 kfsrc={int(r['kf_id']):bytime[r['timestamp_hex']] for r in rows(src/'keyframe_meta.tsv')}
 keys={t:[] for t in frames};lms={t:[] for t in frames}
 for name,out in [('keyframes.tsv',keys),('landmarks.tsv',lms)]:
  for r in rows(src/name):
   t=int(r['frame_idx'])+1
   if t in out:out[t].append(r)
 needed=set(frames)|{kfsrc[int(k['kf_id'])] for ks in keys.values() for k in ks}
 kp={i:[] for i in needed};desc={i:[] for i in needed}
 for r in rows(src/'keypoints.tsv'):
  i=int(r['frame_idx'])
  if i in kp:
   assert len(kp[i])==int(r['kp_idx']);kp[i].append((float.fromhex(r['x_hex']),float.fromhex(r['y_hex']),float.fromhex(r['angle_hex']),int(r['octave'])))
 for r in rows(src/'descriptors.tsv'):
  i=int(r['frame_idx'])
  if i in desc:
   assert len(desc[i])==int(r['kp_idx']);desc[i].append(bytes.fromhex(r['descriptor_hex']))
 digests={}
 for t in frames:
  with (dest/f'{t}.map').open('wb') as f:
   def pack(fmt,*args):f.write(struct.pack('<'+fmt,*args))
   def obs(i):
    assert len(kp[i])==len(desc[i]);pack('I',len(kp[i]))
    for point,d in zip(kp[i],desc[i]):pack('3fi',*point);f.write(d)
   f.write(b'SVRELOC1');pack('IdiII',t,float.fromhex(pre[t]['timestamp_hex']),int(pre[t]['last_frm_ref_keyfrm_id']),len(keys[t]),len(lms[t]));obs(t)
   for k in keys[t]:
    id=int(k['kf_id']);source=kfsrc[id];covis=pairs(k['covisibilities']);children=[int(v) for v in k['spanning_children'].split(',') if v]
    pack('Id',id,float.fromhex(pre[source]['timestamp_hex']));pack('16d',*hexes(k['pose_cw_hex']));obs(source)
    pack('I',len(covis))
    for a,b in covis:pack('II',a,b)
    pack('iI',int(k['spanning_parent']),len(children))
    for a in children:pack('I',a)
   for lm in lms[t]:
    obslist=pairs(lm['observations']);pack('I',int(lm['lm_id']));pack('3d',*hexes(lm['pos_w_hex']));pack('3d',*hexes(lm['mean_normal_hex']));pack('ff',float(lm['min_valid_dist_9g']),float(lm['max_valid_dist_9g']));f.write(bytes.fromhex(lm['descriptor_hex']));pack('iI',int(lm['ref_keyfrm_id']),len(obslist))
    for a,b in obslist:pack('II',a,b)
  digests[str(t)]=sha(dest/f'{t}.map');print(seq,t,'extracted',flush=True)
 (dest/'frames.txt').write_text(''.join(f'{t}\n' for t in frames))
 (dest/'inputs.json').write_text(json.dumps({'frames':frames,'maps':digests,'sources':{str(src/n):sha(src/n) for n in ['track_pre.tsv','keyframe_meta.tsv','keyframes.tsv','landmarks.tsv','keypoints.tsv','descriptors.tsv']}},indent=2)+'\n')
