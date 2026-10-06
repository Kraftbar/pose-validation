#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Identify dispatch and compare two small native fixture sets across CPU settings."""
import collections, hashlib, json, os, struct, subprocess
from pathlib import Path
from build import ROOT,OUT,PORT,CV,FLAGS,run
DISABLED='AVX2,FMA3,AVX,AVX512-SKX,SSE4.2,SSE4.1,SSSE3,POPCNT,SSE3'
OLD='AVX2,FMA3,AVX,AVX512_*,SSE4_2,SSE4_1,SSSE3,POPCNT,SSE3'
def records(path):
    with path.open('rb') as f:
        if f.read(8)!=b'RDCV001\0':raise RuntimeError('fixture magic')
        f.read(12)
        while name:=f.read(32):
            n=struct.unpack('<I',f.read(4))[0];b=f.read(n)
            if len(b)!=n:raise RuntimeError('truncated fixture')
            yield name.rstrip(b'\0').decode(),b

def main():
    run(['c++','-std=c++17',*FLAGS,'-I'+str(CV/'include/opencv4'),PORT/'reference_cv/dispatch.cpp','-L'+str(CV/'lib'),'-Wl,-rpath,'+str(CV/'lib'),'-lopencv_core','-o',OUT/'dispatch'])
    base=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OPENCV_FOR_THREADS_NUM='1')
    base.pop('OPENCV_CPU_DISABLE',None)
    for name,disabled in [('default',''),('disabled',DISABLED),('original_env',OLD)]:
        e=dict(base)
        if disabled:e['OPENCV_CPU_DISABLE']=disabled
        r=subprocess.run([str(OUT/'dispatch')],env=e,capture_output=True,text=True,check=True)
        (OUT/f'dispatch_{name}.txt').write_text(r.stdout+r.stderr)
        if name=='original_env':continue
        d=OUT/('quick' if name=='default' else 'disabled');d.mkdir(exist_ok=True)
        r=subprocess.run([str(OUT/'dump_main'),str(ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray'),str(d),'quick'],env=e,capture_output=True,text=True,check=True)
        (OUT/f'dispatch_{name}_dump.log').write_text(r.stdout+r.stderr)
    summary=collections.defaultdict(lambda:[0,0]);cases=0
    for p in sorted((OUT/'quick').glob('*.bin')):
        a=list(records(p));b=list(records(OUT/'disabled'/p.name))
        if len(a)!=len(b):raise RuntimeError('record count')
        for (name,x),(name2,y) in zip(a,b):
            if name!=name2 or len(x)!=len(y):raise RuntimeError('record shape')
            summary[name][0]+=sum(c!=d for c,d in zip(x,y));summary[name][1]+=len(x)
        cases+=1
        p.unlink();(OUT/'disabled'/p.name).unlink()
    (OUT/'dispatch_comparison.json').write_text(json.dumps({'cases':cases,'disabled':DISABLED,'records_mismatches_bytes':summary},indent=2)+'\n')
    # Symbol list and disassembly are evidence of compiled kernels, independent
    # of the CPU-support report. Keep the LK symbol's complete instruction body.
    video=CV/'lib/libopencv_video.so.4.6.0'
    syms=subprocess.check_output(['nm','-S','-C',str(video)],text=True)
    (OUT/'video_dispatch_symbols.txt').write_text('\n'.join(s for s in syms.splitlines() if any(k in s for k in ['LKTracker','Scharr','opt_AVX','opt_SSE']))+'\n')
    match=[s.split()[:2] for s in syms.splitlines() if 'LKTrackerInvoker::operator()(cv::Range const&) const' in s and 'clone' not in s and '__cv_trace' not in s]
    if len(match)!=1:raise RuntimeError('ambiguous LK symbol')
    address,size=(int(v,16) for v in match[0])
    asm=subprocess.check_output(['objdump','-d',f'--start-address={address}',f'--stop-address={address+size}',str(video)],text=True)
    (OUT/'lk_disassembly.txt').write_text(asm)
    imgsyms=subprocess.check_output(['nm','-C',str(CV/'lib/libopencv_imgproc.so.4.6.0')],text=True)
    (OUT/'imgproc_dispatch_symbols.txt').write_text('\n'.join(s for s in imgsyms.splitlines() if any(k in s for k in ['calcHarrisLine_AVX','RowVec_8u32f','SymmColumnSmallVec_32f','getLinearRowFilter']))+'\n')
    print(json.dumps({'cases':cases,'mismatches_bytes':summary},indent=2))
if __name__=='__main__':main()
