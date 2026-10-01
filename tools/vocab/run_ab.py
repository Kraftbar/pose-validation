#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Isolated full-sequence A/B. Existing scorer reused; canonical results untouched."""
import argparse,json,subprocess,sys,time,concurrent.futures
from common import ROOT,OUT,sha,env
sys.path.insert(0,str(ROOT/'tools'))
import tum_eval
ap=argparse.ArgumentParser();ap.add_argument('--variant',choices=['original','own','both'],default='both');ap.add_argument('--workers',type=int,default=2);args=ap.parse_args()
BIN=ROOT/'external/candidates/stella_vslam_examples/build/run_tum_rgbd_slam'
VOCABS={'original':ROOT/'external/candidates/orb_vocab.fbow','own':ROOT/'stella_port/vocab/own_orb_v1.fbow'}
base=OUT/'ab';base.mkdir(exist_ok=True);tum_eval.COMPARE_ROOT=base
configs={s:ROOT/f'external/candidates/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_{s[2]}.yaml' for s in tum_eval.ALL_SEQS}
ldd=subprocess.check_output(['ldd',str(BIN)],env=env(True),text=True)
if 'not found' in ldd:raise RuntimeError(ldd)
libs={p:sha(p) for line in ldd.splitlines() for p in line.split() if p.startswith('/') and __import__('pathlib').Path(p).is_file()}
manifest=dict(binary=str(BIN),binary_sha256=sha(BIN),libraries=libs,configs={s:sha(p) for s,p in configs.items()},repetitions=3,threads={'OMP_NUM_THREADS':1,'OPENBLAS_NUM_THREADS':1},workers=args.workers,loop_closing=True,timeout_seconds=1800)
(base/f'provenance_{args.variant}.json').write_text(json.dumps(manifest,indent=2)+'\n')
variants=list(VOCABS) if args.variant=='both' else [args.variant]
def run(job):
 variant,seq,rep=job;system=f'{variant}_run{rep}';dest=base/system/seq;dest.mkdir(parents=True,exist_ok=True)
 vocab=VOCABS[variant];vhash=sha(vocab);cfg=configs[seq];data=tum_eval.DATA_ROOT/tum_eval.SEQ_DATA_DIRS[seq]
 cmd=[str(BIN),'-v',str(vocab),'-d',str(data),'-c',str(cfg),'--no-sleep','--auto-term','--eval-log-dir',str(dest),'--log-level','warn']
 old=dest/'run.json'
 if old.exists():
  cached=json.loads(old.read_text())
  if cached.get('vocab_sha256')!=vhash or cached.get('binary_sha256')!=manifest['binary_sha256'] or cached.get('config_sha256')!=sha(cfg):raise RuntimeError('stale run '+str(dest))
  if cached['exit_code']==0 and cached['complete']:return variant,seq,rep,'cached'
  raise RuntimeError('preserve failed run before retry: '+str(dest))
 started=time.monotonic()
 with (dest/'stdout.log').open('w') as log:
  try:p=subprocess.run(cmd,env=env(True),stdout=log,stderr=subprocess.STDOUT,timeout=1800);code=p.returncode
  except subprocess.TimeoutExpired:code=124
 wall=time.monotonic()-started
 frames=len(tum_eval.read_rgb_frame_timestamps(seq))
 # The native example pairs RGB/depth within 0.1s even in monocular mode.
 # Keep coverage denominator as all RGB frames, but verify the actual loader count.
 rgb=tum_eval.read_rgb_frame_timestamps(seq)
 depths=[float(r[0]) for r in tum_eval.read_tum_list(data/'depth.txt')]
 expected=sum(min(abs(float(t)-d) for d in depths)<=0.1 for t in rgb)
 times=dest/'track_times.txt'
 processed=len(times.read_text().splitlines()) if times.exists() else 0
 trajectory=dest/'frame_trajectory.txt'
 if trajectory.exists():(dest/'trajectory.tum').write_bytes(trajectory.read_bytes())
 else:(dest/'trajectory.tum').write_text('')
 info=dict(wall_s=wall,frames_in=frames,processed_frames=processed,loader_frames=expected,complete=processed==expected,exit_code=code,command=cmd,vocab_sha256=vhash,binary_sha256=manifest['binary_sha256'],config_sha256=sha(cfg),keyframe_trajectory='keyframe_trajectory.txt',notes='Full original PNG sequence; LC enabled; isolated vocabulary A/B')
 old.write_text(json.dumps(info,indent=2)+'\n')
 score=tum_eval.score_pair(system,seq);score['complete']=info['complete'];(dest/'score.json').write_text(json.dumps(score,indent=2)+'\n')
 return variant,seq,rep,code,processed,frames,score['ate_tracked_m'],score['coverage'],round(wall,2)
jobs=[(v,s,r) for r in range(1,4) for s in tum_eval.ALL_SEQS for v in variants]
with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
 for result in pool.map(run,jobs):print(result,flush=True)
