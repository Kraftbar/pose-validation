#!/usr/bin/env python3
"""Two-process, immutable relocalization references on 12 recorded maps."""
import sys,json,subprocess
from pathlib import Path
from build import ROOT,OUT,env,sha
for seq in sys.argv[1:] or ['fr1_xyz','fr1_desk']:
 folder=OUT/'fixtures'/seq
 dest=folder/'relocalization';dest.mkdir(exist_ok=False)
 inputs=json.loads((folder/'inputs.json').read_text());records=[]
 for frame in inputs['frames']:
  # Snapshot 549 retains graph links to absent keyframe 3; cannot reconstruct faithfully.
  if seq=='fr1_desk' and frame==550:continue
  for mode in range(8):
   name=f'{frame}_{mode}'
   for rep in [1,2]:
    prefix=dest/f'{name}.pass{rep}'
    cmd=[OUT/'dump_reloc',folder/f'{frame}.map',ROOT/'external/candidates/orb_vocab.fbow',prefix,str(mode)]
    p=subprocess.run(list(map(str,cmd)),env=env(),capture_output=True,text=True)
    (dest/f'{name}.pass{rep}.log').write_text(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError(f'{name} pass{rep}: {p.stdout} {p.stderr}')
   trace1=dest/f'{name}.pass1.trace';trace2=dest/f'{name}.pass2.trace'
   if sha(trace1)!=sha(trace2):raise RuntimeError(f'nondeterministic {name}')
   pnp1=sorted(dest.glob(f'{name}.pass1.pnp*.input'));pnp2=sorted(dest.glob(f'{name}.pass2.pnp*.input'))
   assert len(pnp1)==len(pnp2)
   for a,b in zip(pnp1,pnp2):assert sha(a)==sha(b),(a,b)
   records.append({'name':name,'frame':frame,'mode':mode,'trace':trace1.name,'sha256':sha(trace1),'pnp_inputs':{p.name:sha(p) for p in pnp1}})
   (dest/'reference.json').write_text(json.dumps({'binary_sha256':sha(OUT/'dump_reloc'),'cases':records,'excluded_frames':([{'frame':550,'reason':'snapshot549 links to absent keyframe3'}] if seq=='fr1_desk' else [])},indent=2)+'\n')
   print(seq,name,(dest/f'{name}.pass1.log').read_text().strip(),'deterministic',flush=True)
