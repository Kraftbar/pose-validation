#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import json,random,struct,subprocess,os
from common import ROOT,OUT,sha,env
folder=OUT/'tests';folder.mkdir(exist_ok=True);rng=random.Random(20260930)
inputs={'random':[(i//100,rng.randbytes(32)) for i in range(2000)],'identical':[(i//10,b'\x5a'*32) for i in range(100)],'few_unique':[(i//20,bytes([i%3])*32) for i in range(200)]}
results=[]
for label,records in inputs.items():
 path=folder/(label+'.desc');path.write_bytes(b'SVORBD01'+struct.pack('<II',records[-1][0]+1,len(records))+b''.join(struct.pack('<I',d)+b for d,b in records))
 for rep in [1,2]:
  subprocess.run([str(OUT/'train'),str(path),str(folder/f'{label}{rep}.fbow'),'10','6','15','20260930',str(folder/f'{label}{rep}.json')],check=True)
 assert sha(folder/f'{label}1.fbow')==sha(folder/f'{label}2.fbow')
 stats=json.loads((folder/f'{label}1.json').read_text());assert stats['zero_df_words']==0
 if label=='identical':assert stats['words']==1
 p=subprocess.run([str(OUT/'check_compat'),str(folder/f'{label}1.fbow'),str(path)],env=env(),capture_output=True,text=True);print(label,p.stdout,p.stderr,flush=True);assert p.returncode==0
 results.append(dict(case=label,stats=stats,comparison=p.stdout,sha256=sha(folder/f'{label}1.fbow')))
# Invalid input must fail before creating a vocabulary.
for label,blob in [('empty',b''),('truncated',(folder/'random.desc').read_bytes()[:-1]),('bad_doc',b'SVORBD01'+struct.pack('<III',1,1,9)+bytes(32))]:
 p=folder/(label+'.desc');p.write_bytes(blob)
 run=subprocess.run([str(OUT/'train'),str(p),str(folder/(label+'.fbow')),'10','6','15','1',str(folder/(label+'.json'))],capture_output=True,text=True);assert run.returncode!=0
 results.append(dict(case=label,returncode=run.returncode))
(folder/'results.json').write_text(json.dumps(results,indent=2)+'\n')
