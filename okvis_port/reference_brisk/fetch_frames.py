#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fetch only selected EuRoC images via the ETH archive used by fetch_seq.py.
Non-commercial evaluation data: output must stay under gitignored runs/.
Stored outer ZIP members use byte ranges into the inner archive. Deflated
members use bounded streaming local-entry decoding, including trailing-size
ZIP descriptors, and stop after the requested cam0 window. Full archives
are never stored. The URLs are read from tools/vio_harness/fetch_seq.py.
"""
import ast, hashlib, io, json, struct, urllib.request, urllib.error, zipfile, time, zlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/okvis_port/reference_brisk/frames'
# Read the fetcher's source constants without running its full-sequence download.
tree=ast.parse((ROOT/'tools/vio_harness/fetch_seq.py').read_text())
env={}
for stmt in tree.body:
    if isinstance(stmt,ast.Assign) and isinstance(stmt.targets[0],ast.Name) and stmt.targets[0].id in ('B','OUT'):
        exec(compile(ast.Module(body=[stmt],type_ignores=[]),'<fetch_seq constants>','exec'),env)
class Remote(io.RawIOBase):
    def __init__(self,url,base=0,size=None):
        self.url,self.base,self.pos=url,base,0
        if size is None:
            _,total=self.request(0,0);size=total
        self.size=size
    def request(self,a,b):
        req=urllib.request.Request(self.url,headers={'Range':f'bytes={a}-{b}','User-Agent':'Mozilla/5.0'})
        for attempt in range(8):
            try:
                with urllib.request.urlopen(req,timeout=45) as r:
                    cr=r.headers.get('Content-Range','');assert cr.startswith(f'bytes {a}-'),(r.status,cr)
                    data=r.read();assert len(data)==b-a+1,(len(data),b-a+1)
                    return data,int(cr.split('/')[1])
            except urllib.error.HTTPError as e:
                if e.code not in (429,500,502,503,504) or attempt==7:raise
                print('retry',e,flush=True);time.sleep(15+attempt*5)
    def readable(self):return True
    def seekable(self):return True
    def tell(self):return self.pos
    def seek(self,n,w=0):
        self.pos=n if w==0 else self.pos+n if w==1 else self.size+n
        return self.pos
    def read(self,n=-1):
        n=min(self.size-self.pos,n if n>=0 else self.size)
        if n<=0:return b''
        b,_=self.request(self.base+self.pos,self.base+self.pos+n-1);self.pos+=len(b);return b
    def readinto(self,b):
        d=self.read(len(b));b[:len(d)]=d;return len(d)
def main():
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--seq',default='MH_01_easy');ap.add_argument('--indices',default='0,1,50,200,500,1000,2000');args=ap.parse_args()
    grp={'MH':'machine_hall','V1':'vicon_room1','V2':'vicon_room2'}[args.seq[:2]]
    url=env['OUT'][grp];f=Remote(url);z=zipfile.ZipFile(io.BufferedReader(f,buffer_size=4<<20));e=z.getinfo(f'{grp}/{args.seq}/{args.seq}.zip')
    if e.compress_type != zipfile.ZIP_STORED:
        # Stream local ZIP entries from the deflated outer member. Never store
        # a full archive; stop once the selected cam0 window is available.
        indices=set(map(int,args.indices.split(',')));last=max(indices)
        OUT.mkdir(parents=True,exist_ok=True);records=[];frame=-1
        with io.BufferedReader(z.open(e),buffer_size=1<<17) as stream:
            while True:
                h=stream.read(30)
                if len(h)!=30 or h[:4]!=b'PK\x03\x04':break
                _,flags,method=struct.unpack_from('<HHH',h,4)
                crc=struct.unpack_from('<I',h,14)[0]
                cs,us,nl,xl=struct.unpack_from('<IIHH',h,18)
                assert not flags&1,'Encrypted ZIP unsupported'
                name=stream.read(nl).decode();stream.read(xl)
                wanted=False
                if 'mav0/cam0/data/' in name and name.endswith('.png'):
                    frame+=1;wanted=frame in indices
                data=bytearray()
                if flags&8 and method==8:
                    dec=zlib.decompressobj(-15);actual=0
                    while not dec.eof:
                        block=stream.peek(1<<16);assert block
                        raw=dec.decompress(block);used=len(block)-len(dec.unused_data)
                        stream.read(used);actual+=len(raw)
                        if wanted:data.extend(raw)
                    desc=stream.read(4)
                    if desc==b'PK\x07\x08':desc=stream.read(4)
                    crc,cs,us=struct.unpack('<III',desc+stream.read(8))
                    assert us==actual
                    raw=bytes(data)
                else:
                    left=cs
                    while left:
                        block=stream.read(min(left,1<<20));assert block
                        if wanted:data.extend(block)
                        left-=len(block)
                    if flags&8:
                        desc=stream.read(4)
                        if desc==b'PK\x07\x08':desc=stream.read(4)
                        crc,cs,us=struct.unpack('<III',desc+stream.read(8))
                    if wanted:raw=zlib.decompress(data,-15) if method==8 else bytes(data)
                if wanted:
                    assert len(raw)==us and zlib.crc32(raw)==crc
                    dest=OUT/f'{args.seq}_cam0_{frame:06d}.png';dest.write_bytes(raw)
                    records.append({'path':str(dest.relative_to(ROOT)),'member':name,'index':frame,'url':url,'sha256':hashlib.sha256(raw).hexdigest()})
                    print(dest.name,len(raw),flush=True)
                if frame>=last:break
        assert len(records)==len(indices),'Incomplete selected frame window'
        (OUT/f'{args.seq}.json').write_text(json.dumps(records,indent=2)+'\n')
        return
    f.seek(e.header_offset);h=f.read(30);nl,xl=struct.unpack_from('<HH',h,26)
    inner=zipfile.ZipFile(Remote(url,e.header_offset+30+nl+xl,e.file_size))
    OUT.mkdir(parents=True,exist_ok=True);records=[]
    for cam in ('cam0','cam1'):
        names=sorted(n for n in inner.namelist() if f'mav0/{cam}/data/' in n and n.endswith('.png'))
        for idx in map(int,args.indices.split(',')):
            if idx>=len(names):continue
            n=names[idx];dest=OUT/f'{args.seq}_{cam}_{idx:06d}.png'
            if not dest.exists():dest.write_bytes(inner.read(n))
            records.append({'path':str(dest.relative_to(ROOT)),'member':n,'index':idx,'url':url,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
            print(dest.name,dest.stat().st_size,flush=True)
    (OUT/f'{args.seq}.json').write_text(json.dumps(records,indent=2)+'\n')
if __name__=='__main__':main()
