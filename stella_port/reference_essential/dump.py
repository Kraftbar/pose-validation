#!/usr/bin/env python3
import sys,json,subprocess,hashlib
from pathlib import Path
from build import ROOT,OUT,env,sha
binary=OUT/'dump_reference'
for seq in sys.argv[1:] or ['fr1_xyz','fr1_desk']:
 folder=OUT/'fixtures'/seq
 inputs=json.loads((folder/'inputs.json').read_text());records={}
 for frame in inputs['frames']:
  for rep in [1,2]:
   dest=folder/f'{frame}.pass{rep}';assert not dest.exists(),dest
   p=subprocess.run([str(binary),str(folder/f'{frame}.input'),str(dest)],env=env(),capture_output=True,text=True)
   (folder/f'{frame}.pass{rep}.log').write_text(p.stdout+p.stderr);p.check_returncode()
  assert sha(folder/f'{frame}.pass1')==sha(folder/f'{frame}.pass2'),frame
  (folder/f'{frame}.trace').write_bytes((folder/f'{frame}.pass1').read_bytes())
  records[frame]=sha(folder/f'{frame}.trace');print(seq,frame,'deterministic',flush=True)
 (folder/'frames.txt').write_text(''.join(f'{i}\n' for i in inputs['frames']))
 (folder/'reference.json').write_text(json.dumps({'binary_sha256':sha(binary),'traces':records},indent=2)+'\n')
