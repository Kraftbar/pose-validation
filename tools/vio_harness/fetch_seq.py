#!/usr/bin/env python3
"""fetch_seq.py MH_01_easy -> data/MH_01_easy/mav0 (ASL). Streams nested zip from ETH Research Collection via HTTP ranges,
writes the zip to data/_tmp.zip, extracts, deletes zip."""
import io, os, sys, shutil, zipfile
sys.path.insert(0, os.path.dirname(__file__))
from remotezip import HTTPFile
B='https://www.research-collection.ethz.ch/server/api/core/bitstreams/'
OUT={'machine_hall':B+'7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/content',
     'vicon_room1':B+'02ecda9a-298f-498b-970c-b7c44334d880/content',
     'vicon_room2':B+'ea12bc01-3677-4b4c-853d-87c7870b8c44/content'}
seq=sys.argv[1]
grp={'MH':'machine_hall','V1':'vicon_room1','V2':'vicon_room2'}[seq[:2]]
here=os.path.dirname(os.path.abspath(__file__)); data=os.path.join(here,'data'); tmp=os.path.join(data,'_tmp.zip')
z=zipfile.ZipFile(io.BufferedReader(HTTPFile(OUT[grp]),buffer_size=1<<22))
name=f'{grp}/{seq}/{seq}.zip'
with z.open(name) as f, open(tmp,'wb') as o: shutil.copyfileobj(f,o,1<<22)
zz=zipfile.ZipFile(tmp); zz.extractall(os.path.join(data,seq)); os.remove(tmp)
print('done',seq)
