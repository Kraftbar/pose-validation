#!/usr/bin/env python3
"""Capture real bow_tree outputs on immutable frame inputs and synthetic maps.
Landmark attachments are controlled fixtures, not continuous SLAM history.
"""
import argparse
import csv
import itertools
import json
from pathlib import Path
import random
import struct
import subprocess
from build_stella_match_bow import ROOT, OUT, build, runtime_env, sha


def frame(out, ident, rows, nodes):
    out.write(f'F {ident} {len(rows)}\n')
    for angle, desc, token, erased in rows:
        out.write(f'{float(angle).hex()} {desc} {token} {erased}\n')
    out.write(f'{len(nodes)}\n')
    for node, indices in nodes:
        out.write(f'{node} {len(indices)} ' + ' '.join(map(str, indices)) + '\n')


def synthetic(path):
    rng = random.Random(9041)
    qid = 0
    def queries(out, a, b):
        nonlocal qid
        for mode, orient, ratio in itertools.product([0, 1], [0, 1], [0., .6, .8, 1.]):
            out.write(f'Q {qid} {mode} {a} {b} {ratio} {orient}\n'); qid += 1
    def bits(n):
        return ((1 << n)-1).to_bytes(32, 'little').hex()
    with path.open('w') as out:
        frame(out, 0, [], []); queries(out, 0, 0)
        angles = [0, 30, struct.unpack('f',struct.pack('I',0x41efffff))[0],
                  struct.unpack('f',struct.pack('I',0x41f00001))[0], 330, 360, -30, 180, -180]
        # One descriptor at distances straddling the Hamming and orientation
        # boundaries; nonzero ID 2^32 exercises the UINT32_MAX native ID.
        fid = 1
        for d, angle in itertools.product([0, 1, 49, 50, 51, 256], angles):
            frame(out, fid, [(0, bits(0), 4294967296, 0)], [(9,[0])])
            frame(out, fid+1, [(angle, bits(d), 100, 0)], [(9,[0])])
            queries(out,fid,fid+1); queries(out,fid+1,fid); fid += 2
        # Exact ratio equality: best=30, second=50 with ratio .6.
        frame(out,fid,[(0,bits(0),1,0)],[(7,[0])])
        frame(out,fid+1,[(0,bits(30),2,0),(0,bits(50),3,0)],[(7,[0,1])])
        queries(out,fid,fid+1); fid += 2
        # Ties, reverse node-vector order, duplicates in landmark identity,
        # absent/erased landmarks, empty nodes, and disjoint words.
        for case in range(180):
            for side in range(2):
                rows=[]; buckets={0:[], 5:[], 4294967295:[]}
                for i in range(rng.randrange(0,24)):
                    d=rng.choice([0,0,1,20,30,49,50,51,100,256])
                    token = (i//2+1) if case % 3 == 0 else i+1
                    state = token % 5
                    if state == 0: token=0
                    rows.append((rng.choice(angles),bits(d),token,int(state==4)))
                    buckets[rng.choice(list(buckets))].append(i)
                for indices in buckets.values(): rng.shuffle(indices)
                frame(out,fid+side,rows,sorted(buckets.items()))
            queries(out,fid,fid+1); queries(out,fid+1,fid)
            queries(out,0,fid); queries(out,fid,0); fid+=2
    return {'frames':fid,'queries':qid}


def groups(path):
    with path.open() as f:
        for key, rows in itertools.groupby(csv.reader(itertools.islice(f,1,None),delimiter='\t'), lambda r:int(r[0])):
            yield key, list(rows)


def real(path, seq):
    base=ROOT/'runs/stella_port/reference_dumps'/seq
    bow=ROOT/'runs/stella_port/reference_frame_bow'/seq/'bow_feat.tsv'
    paths=[base/'keypoints.tsv',base/'descriptors.tsv',bow]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in paths}
    ids=[]; qid=0
    with path.open('w') as out:
        for kp,ds,bs in itertools.zip_longest(*(groups(p) for p in paths)):
            assert kp and ds and bs and kp[0]==ds[0]==bs[0]
            ident=kp[0]; ids.append(ident)
            assert len(kp[1])==len(ds[1])
            rows=[]
            for i,(k,d) in enumerate(zip(kp[1],ds[1])):
                assert int(k[1])==int(d[1])==i
                # Mixed live, absent and pending-erasure landmarks.
                token=ident*2048+i+1
                rows.append((float.fromhex(k[8]),d[2],0 if i%11==0 else token,int(i%13==0)))
            nodes=[(int(r[1]),[int(i) for i in r[2].split(',')]) for r in bs[1]]
            frame(out,ident,rows,nodes)
        assert ids==list(range(len(ids)))
        for gap in [1,50]:
            for a in ids[:-gap]:
                for mode,orient in itertools.product([0,1],[0,1]):
                    ratio=[.6,.75,.8][a%3]
                    out.write(f'Q {qid} {mode} {a} {a+gap} {ratio} {orient}\n');qid+=1
        # Self matches exercise identity and large exact-match populations.
        for a in ids[::25]:
            for mode in [0,1]:
                out.write(f'Q {qid} {mode} {a} {a} 0.6 1\n');qid+=1
    assert all(sha(ROOT/p)==h for p,h in hashes.items())
    return {'frames':len(ids),'queries':qid,'input_sha256':hashes}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,default=OUT/'fixtures')
    ap.add_argument('--skip-build',action='store_true')
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    binary=OUT/'build/dump_match_bow' if args.skip_build else build()
    provenance={'binary_sha256':sha(binary),'cases':{}}
    for name in ['synthetic','fr1_xyz','fr1_desk']:
        folder=args.out/name;folder.mkdir()
        commands=folder/'commands.txt'
        info=synthetic(commands) if name=='synthetic' else real(commands,name)
        for n in [1,2]:
            p=subprocess.run([str(binary),str(commands),str(folder/f'pass{n}.tsv')],
                env=runtime_env(),cwd=ROOT,capture_output=True,text=True,check=True)
            (folder/f'pass{n}.log').write_text(p.stdout+p.stderr)
        assert sha(folder/'pass1.tsv')==sha(folder/'pass2.tsv'), name
        (folder/'expected.tsv').write_bytes((folder/'pass1.tsv').read_bytes())
        info.update(commands_sha256=sha(commands),expected_sha256=sha(folder/'expected.tsv'),deterministic=True)
        provenance['cases'][name]=info
        print(name,info['frames'],info['queries'],flush=True)
    assert provenance['binary_sha256']==sha(binary)
    (args.out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')


if __name__=='__main__':main()
