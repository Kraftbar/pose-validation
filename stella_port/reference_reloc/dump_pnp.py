#!/usr/bin/env python3
import json,subprocess,sys,struct,math
from build import ROOT,OUT,env,sha
for seq in sys.argv[1:] or ['fr1_xyz','fr1_desk']:
 folder=OUT/'fixtures'/seq;dest=folder/'pnp';dest.mkdir(exist_ok=False)
 sources={}
 for p in sorted((folder/'relocalization').glob('*.pass1.pnp*.input')):sources.setdefault(sha(p),p)
 for h,p in sources.items():(dest/(h[:16]+'.input')).write_bytes(p.read_bytes())
 # No-match, below minimum, exact minimum, collinear/identical controls.
 scales=[1.0]
 for i in range(7):scales.append(struct.unpack('f',struct.pack('f',scales[-1]*struct.unpack('f',struct.pack('f',1.2))[0]))[0])
 for n in [0,3,10,12]:
  blob=struct.pack('<II8f',n,8,*scales)
  for i in range(n):
   p=(0.,0.,3.) if n==12 else ((i%3)*.1,(i//3)*.1,3.+(i%2)*.2)
   c=(p[0]+.2,p[1]-.1,p[2]+.3);norm=math.sqrt(sum(x*x for x in c));b=tuple(x/norm for x in c);blob+=struct.pack('<6di',*b,*p,i%8)
  (dest/f'edge_{n}.input').write_bytes(blob)
 records=[]
 for input in sorted(dest.glob('*.input')):
  for recompute in [0,1]:
   name=input.stem+f'_r{recompute}'
   for rep in [1,2]:
    trace=dest/f'{name}.pass{rep}.trace';p=subprocess.run([str(OUT/'dump_pnp'),str(input),str(trace),str(recompute)],env=env(),capture_output=True,text=True)
    (dest/f'{name}.pass{rep}.log').write_text(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError(name+': '+p.stderr)
   assert sha(dest/f'{name}.pass1.trace')==sha(dest/f'{name}.pass2.trace'),name
   records.append({'name':name,'input':input.name,'input_sha256':sha(input),'recompute':recompute,'trace':f'{name}.pass1.trace','sha256':sha(dest/f'{name}.pass1.trace')})
   print(seq,name,'deterministic',flush=True)
 (dest/'reference.json').write_text(json.dumps({'binary_sha256':sha(OUT/'dump_pnp'),'sources':{str(p):h for h,p in sources.items()},'cases':records},indent=2)+'\n')
