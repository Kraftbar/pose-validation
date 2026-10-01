#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build only in the reserved vocabulary tree, reusing native flags read-only."""
import json,shlex,subprocess
from pathlib import Path
from common import ROOT,OUT,sha,env
OUT.mkdir(parents=True,exist_ok=True)
def native(name,sources):
 target=ROOT/'runs/stella_port/reference_build/frame_bow_tool_build/CMakeFiles/dump_frame_bow.dir'
 flags=[]
 for line in (target/'flags.make').read_text().splitlines():
  if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
 objects=[];commands=[];deps=set()
 for source in sources:
  p=ROOT/source;o=OUT/(p.stem+'.o');objects.append(str(o))
  cmd=(['gcc','-std=c99','-O2','-ffp-contract=off'] if p.suffix=='.c' else ['c++',*flags])+['-MD','-MF',str(o)+'.d','-c',str(p),'-o',str(o)]
  subprocess.run(cmd,check=True);commands.append(cmd)
  deps.update(Path(x) for x in shlex.split(Path(str(o)+'.d').read_text().replace('\\\n',' ').split(':',1)[1]))
 link=[x for x in shlex.split((target/'link.txt').read_text()) if not x.endswith('.o') and not x.startswith('-Wl,--dependency-file=')]
 link[link.index('-o')+1]=str(OUT/name);link[1:1]=objects;subprocess.run(link,check=True);commands.append(link)
 for line in subprocess.check_output(['ldd',str(OUT/name)],env=env(),text=True).splitlines():
  deps.update(Path(x) for x in line.split() if x.startswith('/') and Path(x).is_file())
 (OUT/(name+'.build.json')).write_text(json.dumps(dict(commands=commands,inputs={str(p):sha(p) for p in sorted(deps)},binary_sha256=sha(OUT/name)),indent=2)+'\n')
if __name__=='__main__':
 native('extract',['tools/vocab/extract.cc'])
 native('check_compat',['tools/vocab/check_compat.cc','stella_port/c/sv_bow.c'])
 cmd=['gcc','-std=c99','-O3','-mpopcnt','-ffp-contract=off','-Wall','-Wextra','tools/vocab/train.c','-lm','-o',str(OUT/'train')]
 subprocess.run(cmd,check=True)
 (OUT/'train.build.json').write_text(json.dumps(dict(command=cmd,source_sha256=sha(ROOT/'tools/vocab/train.c'),binary_sha256=sha(OUT/'train')),indent=2)+'\n')
