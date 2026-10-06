#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Preserve compiled-kernel evidence and compare native CPU-dispatch paths."""
import collections, json, os, shutil, struct, subprocess
from build_m7b import ROOT, PORT, OUT, CV, FLAGS, run

def records(path,piece):
    with path.open('rb') as f:
        magic=f.read(8)
        if magic!=(b'RDUND01\0' if piece=='undist' else b'RDPNP01\0'):raise RuntimeError('magic')
        while h:=f.read(16 if piece=='undist' else 8):
            branch=('fisheye' if struct.unpack('<4I',h)[2] else 'radtan') if piece=='undist' else 'epnp'
            while True:
                name=f.read(32).rstrip(b'\0').decode();n=struct.unpack('<I',f.read(4))[0];data=f.read(n)
                if len(data)!=n:raise RuntimeError('truncated')
                if name not in ('K','D','input','map_input1','map_input2','X','x'):yield branch+'.'+name,data
                if name==('remap' if piece=='undist' else 'T'):break

def main():
    run(['c++','-std=c++17',*FLAGS,'-I'+str(CV/'include/opencv4'),PORT/'reference_cv/dispatch.cpp',
         '-L'+str(CV/'lib'),'-Wl,-rpath,'+str(CV/'lib'),'-lopencv_core','-o',OUT/'dispatch'])
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OPENCV_FOR_THREADS_NUM='1')
    env.pop('OPENCV_CPU_DISABLE',None)
    disabled='AVX2,FMA3,AVX,AVX512-SKX,SSE4.2,SSE4.1,SSSE3,POPCNT'
    for mode,value in [('default',''),('disabled',disabled)]:
        e=dict(env)
        if value:e['OPENCV_CPU_DISABLE']=value
        r=subprocess.run([str(OUT/'dispatch')],env=e,capture_output=True,text=True,check=True)
        (OUT/('dispatch_'+mode+'.txt')).write_text(r.stdout+r.stderr)
    env['OPENCV_CPU_DISABLE']=disabled
    result={}
    for piece,args in [('undist',[ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray',OUT/'disabled.bin']),
                       ('pnp',[OUT/'disabled.bin','1000'])]:
        run([OUT/('dump_'+piece),*args],env=env)
        summary=collections.defaultdict(lambda:{'records':0,'bytes':0,'mismatches':0})
        from itertools import zip_longest
        for a,b in zip_longest(records(OUT/(piece+'.bin'),piece),records(OUT/'disabled.bin',piece)):
            if a is None or b is None or a[0]!=b[0] or len(a[1])!=len(b[1]):raise RuntimeError('shape')
            g=summary[a[0]];g['records']+=1;g['bytes']+=len(a[1]);g['mismatches']+=sum(x!=y for x,y in zip(a[1],b[1]))
        result[piece]=dict(summary);(OUT/'disabled.bin').unlink()
    for lib,keys in [('calib3d',['initUndistortRectifyMapComputer::operator()']),
                     ('core',['JacobiSVDImpl_<double>','MulTransposedR<double, double>']),
                     ('imgproc',['remapBilinear<cv::FixedPtCast<int, unsigned char, 15>','RemapInvoker::operator()'])]:
        path=CV/('lib/libopencv_'+lib+'.so.4.6.0')
        syms=subprocess.check_output(['nm','-S','-C',str(path)],text=True)
        matches=[s for s in syms.splitlines() if any(k in s for k in keys)]
        (OUT/(lib+'_symbols.txt')).write_text('\n'.join(matches)+'\n')
        with (OUT/(lib+'_disassembly.txt')).open('w') as f:
            for s in matches:
                fields=s.split()
                if len(fields)<4 or 'cold' in s:continue
                address,size=int(fields[0],16),int(fields[1],16)
                subprocess.run(['objdump','-d','-C',f'--start-address={address}',f'--stop-address={address+size}',str(path)],stdout=f,check=True)
    # On this host ptrace is available. These breakpoints establish execution,
    # independently of matching float outputs or merely finding a symbol.
    if shutil.which('gdb'):
        mapfn='::(anonymous namespace)::initUndistortRectifyMapComputer::operator()(cv::Range const&) const'
        probes=[('map_avx2','undist','cv::opt_AVX2'+mapfn,False),
                ('map_baseline','undist','cv::cpu_baseline'+mapfn,True),
                ('remap','undist','void cv::remapBilinear<cv::FixedPtCast<int, unsigned char, 15>, cv::RemapVec_8u, short>(cv::Mat const&, cv::Mat&, cv::Mat const&, cv::Mat const&, void const*, int, cv::Scalar_<double> const&)',False)]
        core=(OUT/'core_symbols.txt').read_text().splitlines()
        for label,key in [('svd','JacobiSVDImpl_<double>'),('mtm','MulTransposedR<double, double>')]:
            sym=next(s.split(' ',3)[3] for s in core if key in s and 'cold' not in s)
            probes.append((label,'pnp',sym,False))
        for label,piece,sym,disable in probes:
            e=dict(env)
            if not disable:e.pop('OPENCV_CPU_DISABLE',None)
            path=OUT/'dispatch_probe.bin'
            args=([ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray',path] if piece=='undist' else [path,'1'])
            cmd=['gdb','-q','-batch','-ex','set pagination off','-ex','set confirm off','-ex','set breakpoint pending on',
                 '-ex',"break '"+sym+"'",'-ex','run','-ex','bt 5','--args',OUT/('dump_'+piece),*args]
            r=subprocess.run(list(map(str,cmd)),env=e,capture_output=True,text=True)
            (OUT/('dispatch_gdb_'+label+'.log')).write_text(r.stdout+r.stderr)
            if path.exists():path.unlink()
            if 'Breakpoint 1,' not in r.stdout:raise RuntimeError('dispatch probe failed: '+label)
    result['disabled']=disabled
    (OUT/'dispatch_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
