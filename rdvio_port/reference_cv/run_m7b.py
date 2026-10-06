#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate or replay M7b byte-exact fixtures, including the full gray pack."""
import argparse, hashlib, json, os, re, shutil, struct, subprocess
from build_m7b import ROOT, PORT, OUT, build

RAW=ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray'
REAL=ROOT/'runs/rdvio_port/m7b_reference_run/pnp_real.bin'
PACK=ROOT/'runs/rdvio_port/system/MH_01_easy.undist.gray'

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--reuse',action='store_true');p.add_argument('--sanitize',action='store_true');a=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(OUT).free<15_000_000_000:raise RuntimeError('less than 15 GB free')
    build(a.sanitize)
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OPENCV_FOR_THREADS_NUM='1',
             ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1')
    env.pop('OPENCV_CPU_DISABLE',None)
    if not a.reuse:
        for name,args in [('undist',[RAW,OUT/'undist.bin']),('pnp',[OUT/'pnp.bin','1000'])]:
            with (OUT/('dump_'+name+'.log')).open('w') as log:
                subprocess.run([str(OUT/('dump_'+name)),*map(str,args)],env=env,check=True,stdout=log,stderr=subprocess.STDOUT)
    mode='san' if a.sanitize else 'normal';suffix='_san' if a.sanitize else ''
    groups={}
    for label,piece,args,count in [('undist','undist',[OUT/'undist.bin'],104),
                                  ('pnp','pnp',[OUT/'pnp.bin'],18000),
                                  ('pnp_real','pnp',[REAL],127057),
                                  ('pack','undist',['--pack',RAW,PACK],3682)]:
        proc=subprocess.run([str(OUT/('check_'+piece+suffix)),*map(str,args)],env=env,capture_output=True,text=True)
        (OUT/(label+'_'+mode+'.log')).write_text(proc.stdout)
        (OUT/(label+'_'+mode+'.err')).write_text(proc.stderr)
        m=re.search(r'^m7b_\w+: (\d+)/(\d+)$',proc.stdout,re.M);n=re.search(r'cases=(\d+)',proc.stdout)
        if proc.returncode or not m or not n or int(n[1])!=count:
            raise RuntimeError(f'{label} failed/incomplete: {proc.stdout} {proc.stderr}')
        groups[label]={'cases':int(n[1]),'bytes':int(m[2]),'mismatches':int(m[1])}
        print(label,groups[label],flush=True)
    # Check rejection of damaged, truncated, surplus, and empty fixtures.
    for piece,header,end in [('pnp',8,'T'),('undist',16,'remap')]:
        with (OUT/(piece+'.bin')).open('rb') as f:
            first=bytearray(f.read(8+header))
            while True:
                h=f.read(36);z=struct.unpack('<I',h[32:])[0];first+=h+f.read(z)
                if h[:32].rstrip(b'\0').decode()==end:break
        bad=bytearray(first);bad[-1]^=1
        for label,payload in [('corrupt',bad),('truncated',first[:-1]),('surplus',first+b'!'),('empty',first[:8])]:
            path=OUT/'negative.bin';path.write_bytes(payload)
            proc=subprocess.run([str(OUT/('check_'+piece+suffix)),str(path)],env=env,capture_output=True,text=True)
            path.unlink()
            (OUT/(f'negative_{piece}_{mode}_{label}.log')).write_text(proc.stdout+proc.stderr)
            if proc.returncode!=1:raise RuntimeError(f'{piece} did not reject {label}')
    result={'mode':mode,'groups':groups,'negative_fixtures':8,
            'total':{k:sum(g[k] for g in groups.values()) for k in ('cases','bytes','mismatches')}}
    (OUT/('validation_'+mode+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    manifest=[{'file':str(f.relative_to(ROOT)),'bytes':f.stat().st_size,'sha256':digest(f)} for f in (OUT/'undist.bin',OUT/'pnp.bin',REAL)]
    (OUT/'m7b_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f"m7b: {result['total']['mismatches']}/{result['total']['bytes']}")

if __name__=='__main__':main()
