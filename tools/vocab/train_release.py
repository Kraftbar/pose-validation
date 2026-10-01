#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Produce the first fixed candidate before looking at its A/B scores."""
import json,subprocess,time
from common import ROOT,OUT,sha,env
subprocess.run(['python3',str(ROOT/'tools/vocab/select_training.py')],check=True)
images=OUT/'training_images.txt';desc=OUT/'training.desc'
for rep in [1,2]:
 target=desc if rep==1 else OUT/'training_repeat.desc'
 with (OUT/f'extract{rep}.log').open('w') as log:subprocess.run([str(OUT/'extract'),str(target),str(images)],env=env(),stdout=log,stderr=subprocess.STDOUT,check=True)
assert sha(desc)==sha(OUT/'training_repeat.desc'),'extractor not deterministic'
release=ROOT/'stella_port/vocab/own_orb_v1.fbow'
if release.exists():raise RuntimeError('release already exists; preserve it before retraining')
for rep in [1,2]:
 target=release if rep==1 else OUT/'own_orb_v1_repeat.fbow'
 command=[str(OUT/'train'),str(desc),str(target),'10','6','15','20260930',str(OUT/f'train{rep}.json')]
 start=time.monotonic()
 with (OUT/f'train{rep}.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
 print('trained',rep,round(time.monotonic()-start,2),'seconds',flush=True)
assert sha(release)==sha(OUT/'own_orb_v1_repeat.fbow'),'trainer not deterministic'
p=subprocess.run([str(OUT/'check_compat'),str(release),str(desc)],env=env(),capture_output=True,text=True)
(OUT/'compatibility.log').write_text(p.stdout+p.stderr)
if p.returncode:raise RuntimeError('native/C FBoW check failed: '+p.stdout+p.stderr)
record=dict(name='own_orb_v1',artifact_sha256=sha(release),artifact_bytes=release.stat().st_size,license='CC-BY-4.0',training_sources=json.loads((OUT/'training/sources.json').read_text()),training_image_manifest_sha256=sha(OUT/'training_images.json'),descriptor_sha256=sha(desc),extractor_binary_sha256=sha(OUT/'extract'),trainer_binary_sha256=sha(OUT/'train'),trainer_source_sha256=sha(ROOT/'tools/vocab/train.c'),parameters=json.loads((OUT/'train1.json').read_text()),two_process_extraction_identical=True,two_process_training_identical=True,compatibility=p.stdout,eval_exclusion=['fr1_xyz','fr1_desk','fr1_floor','fr2_xyz','fr3_long_office'],selection='single predeclared candidate; no selection using comparison ATE')
(ROOT/'stella_port/vocab/own_orb_v1.json').write_text(json.dumps(record,indent=2)+'\n')
print(p.stdout,flush=True)
