#!/usr/bin/env python3
"""Controlled observation clouds; calls the real native landmark method twice."""
import argparse
import csv
import itertools
import json
from pathlib import Path
import random
import subprocess
from build_stella_landmark_descriptor import ROOT, OUT, build, runtime_env, sha

def case(out,qid,rows):
    out.write(f'Q {qid} {len(rows)}\n')
    for ident,erased,desc in rows:out.write(f'{ident} {erased} {desc}\n')

def synthetic(path):
    rng=random.Random(31861);qid=0
    def bits(n):return ((1<<n)-1).to_bytes(32,'little').hex()
    with path.open('w') as out:
        for n in [1,2,3,4,5,8,15,16,31,32,63,64,127]:
            for pattern in range(24):
                ids=rng.sample(range(100000),n)
                ids[0]=4294967295
                rows=[]
                for i,ident in enumerate(ids):
                    if pattern%4==0:desc=bits(0)
                    elif pattern%4==1:desc=bits(256*(i%2))
                    elif pattern%4==2:desc=bits(rng.choice([0,1,30,49,50,51,128,255,256]))
                    else:desc=rng.getrandbits(256).to_bytes(32,'little').hex()
                    erased=int(i>0 and pattern%3==0 and i%3!=0)
                    rows.append((ident,erased,desc))
                rng.shuffle(rows);case(out,qid,rows);qid+=1
        # Two-row ties select the smaller ID even when its descriptor is far
        # away; erased lowest-ID rows must not participate in the tie.
        for ds in [[0,256],[0,1,2,256],[0,30,31,32,256],[0,0,256,256]]:
            for erased in [False,True]:
                rows=[(100+i,0,bits(d)) for i,d in enumerate(ds)]
                if erased:rows.append((0,1,bits(17)))
                rows.reverse();case(out,qid,rows);qid+=1
    return {'cases':qid}

def groups(path):
    with path.open() as f:
        for k,rs in itertools.groupby(csv.reader(itertools.islice(f,1,None),delimiter='\t'),lambda r:int(r[0])):
            yield k,list(rs)

def real(path,seq):
    desc=ROOT/'runs/stella_port/reference_dumps'/seq/'descriptors.tsv'
    feat=ROOT/'runs/stella_port/reference_frame_bow'/seq/'bow_feat.tsv'
    hashes={str(p.relative_to(ROOT)):sha(p) for p in [desc,feat]}
    frames=[]
    for d,b in itertools.zip_longest(groups(desc),groups(feat)):
        assert d and b and d[0]==b[0]==len(frames)
        nodes={int(row[1]):d[1][int(row[2].split(',')[0])][2] for row in b[1]}
        frames.append(nodes)
    rng=random.Random(877);qid=0
    with path.open('w') as out:
        for ident,nodes in enumerate(frames):
            # Eight anchor nodes spread through the actual BoW node list.
            keys=sorted(nodes)
            for node in keys[::max(1,len(keys)//8)][:8]:
                rows=[]
                for frame_id in range(ident,min(ident+8,len(frames))):
                    if node in frames[frame_id]:
                        rows.append((frame_id,int(frame_id!=ident and frame_id%11==0),frames[frame_id][node]))
                rng.shuffle(rows);case(out,qid,rows);qid+=1
    assert all(sha(ROOT/p)==h for p,h in hashes.items())
    return {'cases':qid,'frames':len(frames),'input_sha256':hashes}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=OUT/'fixtures')
    ap.add_argument('--skip-build',action='store_true');args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    binary=OUT/'build/dump_landmark_descriptor' if args.skip_build else build()
    provenance={'binary_sha256':sha(binary),'generator_sha256':sha(Path(__file__)),'cases':{}}
    for name in ['synthetic','fr1_xyz','fr1_desk']:
        folder=args.out/name;folder.mkdir();commands=folder/'commands.txt'
        info=synthetic(commands) if name=='synthetic' else real(commands,name)
        for i in [1,2]:
            p=subprocess.run([str(binary),str(commands),str(folder/f'pass{i}.tsv')],env=runtime_env(),cwd=ROOT,capture_output=True,text=True)
            (folder/f'pass{i}.log').write_text(p.stdout+p.stderr)
            p.check_returncode()
        assert sha(folder/'pass1.tsv')==sha(folder/'pass2.tsv')
        (folder/'expected.tsv').write_bytes((folder/'pass1.tsv').read_bytes())
        info.update(commands_sha256=sha(commands),expected_sha256=sha(folder/'expected.tsv'),deterministic=True)
        provenance['cases'][name]=info
        print(name,info['cases'],flush=True)
    assert provenance['binary_sha256']==sha(binary)
    (args.out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')

if __name__=='__main__':main()
