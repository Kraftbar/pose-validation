#!/usr/bin/env python3
"""Isolated leaf build; reuse shared reference flags read-only."""
import json,subprocess,shlex,hashlib,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/stella_port/reference_reloc'
REF=ROOT/'runs/stella_port/reference_build'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def env():
 e=dict(os.environ);e['LD_LIBRARY_PATH']=':'.join(map(str,[REF/'install/lib',ROOT/'external/candidates/deps/root/usr/lib',ROOT/'external/candidates/deps/root/usr/lib/x86_64-linux-gnu',Path('/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu')]));return e
def build(name,sources):
 OUT.mkdir(parents=True,exist_ok=True)
 target=REF/'frame_bow_tool_build/CMakeFiles/dump_frame_bow.dir'
 flags=[]
 for line in (target/'flags.make').read_text().splitlines():
  if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
 objs=[];commands=[];dependencies=set()
 previous=json.loads((OUT/(name+'.build.json')).read_text()) if (OUT/(name+'.build.json')).exists() else {}
 for source in sources:
  p=ROOT/source;obj=OUT/(p.stem+'.o');objs.append(str(obj))
  cmd=(['gcc','-std=c99','-O2','-g','-ffp-contract=off','-fno-fast-math'] if p.suffix=='.c' else ['c++',*flags])+['-I'+str(ROOT/'stella_port/c'),'-I'+str(ROOT/'stella_port/reference_reloc'),'-I'+str(OUT/'instrumented'),'-MD','-MF',str(obj)+'.d','-c',str(p),'-o',str(obj)]
  can_reuse=False
  depfile=Path(str(obj)+'.d')
  if obj.exists() and depfile.exists() and cmd in previous.get('commands',[]):
   olddeps=shlex.split(depfile.read_text().replace('\\\n',' ').split(':',1)[1])
   can_reuse=all(Path(x).exists() and previous.get('inputs',{}).get(str(Path(x)))==sha(Path(x)) for x in olddeps)
  if not can_reuse:subprocess.run(cmd,check=True)
  commands.append(cmd)
  deptext=Path(str(obj)+'.d').read_text().replace('\\\n',' ')
  dependencies.update(Path(x) for x in shlex.split(deptext.split(':',1)[1]))
 link=[s for s in shlex.split((target/'link.txt').read_text()) if not s.endswith('.o') and not s.startswith('-Wl,--dependency-file=')]
 link[link.index('-o')+1]=str(OUT/name);link[1:1]=objs
 subprocess.run(link,check=True);commands.append(link)
 dependencies.update(ROOT/s for s in sources)
 dependencies.update([target/'flags.make',target/'link.txt'])
 dependencies.update(Path(x) for x in link if x.startswith('/') and Path(x).is_file())
 # Include actual resolved shared libraries, including -l dependencies.
 ldd=subprocess.run(['ldd',str(OUT/name)],env=env(),capture_output=True,text=True,check=True)
 for line in ldd.stdout.splitlines():
  for item in line.split():
   if item.startswith('/') and Path(item).is_file():dependencies.add(Path(item))
 inputs={str(p):sha(p) for p in sorted(dependencies)}
 (OUT/(name+'.build.json')).write_text(json.dumps({'commands':commands,'inputs':inputs,'binary_sha256':sha(OUT/name)},indent=2)+'\n')
 return OUT/name
