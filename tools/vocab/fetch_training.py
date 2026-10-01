#!/usr/bin/env python3
"""Fetch only explicitly chosen CC-BY-4.0 training sequences, never eval data."""
import hashlib,json,urllib.request,tarfile,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'runs/stella_port/vocab';DATA=OUT/'training';DATA.mkdir(exist_ok=True)
SEQS=[('freiburg2','rgbd_dataset_freiburg2_large_no_loop'),('freiburg3','rgbd_dataset_freiburg3_teddy')]
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  while b:=f.read(1<<20):h.update(b)
 return h.hexdigest()
records=[]
for group,name in SEQS:
 url=f'https://cvg.cit.tum.de/rgbd/dataset/{group}/{name}.tgz'
 archive=DATA/(name+'.tgz');part=archive.with_suffix('.part')
 if not archive.exists():
  print('fetch',url,flush=True)
  with urllib.request.urlopen(url,timeout=60) as src,part.open('wb') as dst:shutil.copyfileobj(src,dst,1<<20)
  part.rename(archive)
 target=DATA/name;target.mkdir(exist_ok=True)
 with tarfile.open(archive,'r|gz') as tar:
  for m in tar:
   pieces=Path(m.name).parts
   if not m.isfile() or not pieces or pieces[0]!=name:continue
   rel=Path(*pieces[1:])
   if '..' in rel.parts or rel.is_absolute():raise ValueError(m.name)
   if rel.name!='rgb.txt' and not (len(rel.parts)==2 and rel.parts[0]=='rgb' and rel.suffix=='.png'):continue
   dest=target/rel;dest.parent.mkdir(exist_ok=True)
   if not dest.exists():
    with tar.extractfile(m) as src,dest.open('wb') as dst:shutil.copyfileobj(src,dst)
 records.append(dict(sequence=name,url=url,archive_sha256=digest(archive),rgb_list_sha256=digest(target/'rgb.txt'),license='CC-BY-4.0',license_source='https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license'))
 print('ready',name,flush=True)
(DATA/'sources.json').write_text(json.dumps(records,indent=2)+'\n')
