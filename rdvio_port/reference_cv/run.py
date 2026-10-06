#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate/compress M7 fixtures, or replay them at tolerance zero.
All artifacts stay in runs/rdvio_port/reference_cv. No external dataset writes.
"""
import argparse, gzip, hashlib, json, os, re, shutil, subprocess, sys
from pathlib import Path
from build import ROOT,OUT,PORT,FLAGS,build,run

def expected_names():
    sizes=[(752,480),(640,480),(753,479),(751,480),(640,481),(31,27),(1,1),(43,43)]
    return ([f'synthetic_{w}x{h}_{k}.bin.gz' for w,h in sizes for k in range(5)]
            +[f'undistorted_{b}_{g}.bin.gz' for b in (0,100,700,1800,3000) for g in range(1,6)]
            +[f'raw_0_{g}.bin.gz' for g in range(1,6)])
def hash_file(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--reuse',action='store_true');ap.add_argument('--sanitize',action='store_true');a=ap.parse_args()
    exe=build(a.sanitize);fixtures=OUT/'fixtures';fixtures.mkdir(exist_ok=True)
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OPENCV_FOR_THREADS_NUM='1',ASAN_OPTIONS='detect_leaks=0')
    if not a.reuse:
        if shutil.disk_usage(OUT).free<19_000_000_000:raise RuntimeError('need 19 GB free before generating <1 GB of uncompressed fixtures')
        with (OUT/'dump.log').open('w') as log:
            subprocess.run([str(OUT/'dump_main'),str(ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray'),str(fixtures)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        for p in sorted(fixtures.glob('*.bin')):
            with p.open('rb') as src,gzip.open(str(p)+'.gz','wb',compresslevel=1) as dst:shutil.copyfileobj(src,dst)
            p.unlink()
    files=[fixtures/n for n in expected_names()]
    missing=[p.name for p in files if not p.is_file()]
    if missing:raise RuntimeError(f'missing {len(missing)} expected fixtures; regenerate without --reuse')
    mode='san' if a.sanitize else 'normal'
    groups={g:{'cases':0,'bytes':0,'mismatches':0,'records':0} for g in ('synthetic','raw','undistorted')}
    manifests=[]
    with (OUT/f'check_{mode}.log').open('w') as log,(OUT/f'check_{mode}.err').open('w') as err:
        for p in files:
            with gzip.open(p,'rb') as f: payload=f.read()
            r=subprocess.run([str(exe),'-'],input=payload,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env)
            text=r.stdout.decode();log.write('FILE '+p.name+'\n'+text);err.write(r.stderr.decode())
            m=re.search(r'^m7: (\d+)/(\d+)$',text,re.M);records=re.search(r'records=(\d+)',text)
            if not m or not records:raise RuntimeError(f'no complete result: {p.name}, {r.stderr.decode()}')
            g=groups[p.name.split('_')[0]];g['cases']+=1;g['bytes']+=int(m[2]);g['mismatches']+=int(m[1]);g['records']+=int(records[1])
            manifests.append({'file':p.name,'uncompressed_sha256':hashlib.sha256(payload).hexdigest(),'uncompressed_bytes':len(payload)})
            if r.returncode:raise RuntimeError(f'fixture failed: {p.name}; see check_{mode}.log/.err')
    api=OUT/('check_rd_cv_api_'+mode)
    run(['cc','-std=c99',*FLAGS,'-Wall','-Wextra',*(['-fsanitize=address,undefined','-g','-fno-omit-frame-pointer','-no-pie'] if a.sanitize else []),
         PORT/'c/check_rd_cv_api.c',PORT/'c/rd_cv.c',PORT/'c/rd_cv_lk.c',PORT/'c/rd_cv_gftt.c','-lm','-o',api])
    r=subprocess.run([str(api)],env=env,capture_output=True,text=True);(OUT/f'api_{mode}.log').write_text(r.stdout+r.stderr)
    if r.returncode:raise RuntimeError('API checks failed')
    # Fixture sensitivity: corrupt one expected byte, truncate, append surplus.
    with gzip.open(files[0],'rb') as f: payload=f.read()
    # Third record is expected CLAHE, after the two raw input records.
    off=20
    for _ in range(2):off+=36+int.from_bytes(payload[off+32:off+36],'little')
    flipped=bytearray(payload);flipped[off+36]^=1
    for name,data in [('corrupt',flipped),('truncated',payload[:-1]),('surplus',payload+b'!')]:
        r=subprocess.run([str(exe),'-'],input=data,env=env,capture_output=True)
        (OUT/f'negative_{mode}_{name}.log').write_bytes(r.stdout+r.stderr)
        if r.returncode!=1:raise RuntimeError(f'negative fixture {name}: expected rejection, got {r.returncode}')
    summary={'mode':mode,'groups':groups,'total':{key:sum(g[key] for g in groups.values()) for key in ('cases','bytes','mismatches','records')},'api_checks':18,'negative_fixtures':3}
    (OUT/f'validation_{mode}.json').write_text(json.dumps(summary,indent=2)+'\n')
    (OUT/'fixtures_manifest.json').write_text(json.dumps(manifests,indent=2)+'\n')
    print(json.dumps(summary,indent=2));print(f"m7: {summary['total']['mismatches']}/{summary['total']['bytes']}")
if __name__=='__main__':main()
