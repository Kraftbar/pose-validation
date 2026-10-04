#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate twice with the real reference, compare exact C traces, run sanitizers.
All artifacts/data go to gitignored runs/okvis_port/reference_brisk (<1.5 GB).
"""
import argparse,hashlib,json,os,subprocess,sys,struct,zlib
from pathlib import Path
R=Path(__file__).resolve().parents[2];O=R/'runs/okvis_port/reference_brisk'
def run(cmd,env=None,log=None):
    print('+',' '.join(map(str,cmd[:8])),f'... ({len(cmd)} arguments)' if len(cmd)>8 else '',flush=True)
    p=subprocess.run(list(map(str,cmd)),env=env,capture_output=True,text=True)
    if log:Path(log).write_text(p.stdout+p.stderr)
    if p.returncode:print((p.stdout+p.stderr)[-5000:]);raise RuntimeError(f'exit {p.returncode}: {cmd[0]}')
    return p.stdout

def png(path,data,w=752,h=480):
    def chunk(tag,b):return struct.pack('>I',len(b))+tag+b+struct.pack('>I',zlib.crc32(tag+b))
    raw=b''.join(b'\0'+data[y*w:(y+1)*w] for y in range(h))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--reuse',action='store_true',help='Reuse complete existing reference dumps');args=ap.parse_args()
    env=dict(os.environ,LD_LIBRARY_PATH=':'.join(str(R/p) for p in ['external/vio/deps/opencv/lib','external/vio/deps/root/usr/lib/x86_64-linux-gnu','external/candidates/deps/root/usr/lib/x86_64-linux-gnu']))
    if not (O/'build/dump_brisk').exists():run([sys.executable,R/'okvis_port/reference_brisk/build.py'],env,O/'build.log')
    frames=sorted((O/'frames').glob('*.png'));assert frames,'Fetch frames first'
    synth=O/'synthetic';synth.mkdir(exist_ok=True)
    w,h=752,480
    cases={'black':bytes(w*h),'white':bytes([255])*(w*h),'checker':bytes(255*((x//8+y//8)%2) for y in range(h) for x in range(w)),'ramp':bytes((x+3*y)%256 for y in range(h) for x in range(w))}
    state=0x12345678;noise=bytearray()
    for i in range(w*h):state=(state*1664525+1013904223)&0xffffffff;noise.append(state>>24)
    cases['noise']=noise
    for name,data in cases.items():png(synth/(name+'.png'),data)
    images=frames+sorted(synth.glob('*.png'))
    assert len(images)<=24,'Refusing oversized fixture set; keep <1.5 GB'
    results=[];hashes={}
    for cam in (0,1):
        dirs=[O/f'validation_{c}/cam{cam}' for c in ('a','b')]
        modes=(0,1,2) if cam==0 else (1,2)
        cam_images=images if cam==0 else frames
        for out in dirs:
            out.mkdir(parents=True,exist_ok=True)
            if not args.reuse:
                for mode in modes:run([O/'build/dump_brisk',out,cam,mode]+cam_images,env,out/f'dump_m{mode}.log')
        for p in sorted(dirs[0].glob('*')):
            if p.suffix not in ('.bin','.maps'):continue
            other=dirs[1]/p.name
            assert p.read_bytes()==other.read_bytes(),f'Nondeterministic {p}'
            hashes[str(p.relative_to(O))]=hashlib.sha256(p.read_bytes()).hexdigest()
        fixtures=sorted(p for p in dirs[0].glob('*.bin') if p.name!='pattern.bin')
        assert len(fixtures)==len(cam_images)*len(modes),'Incomplete fixture set'
        sources=[R/'okvis_port/c'/s for s in ('check_ok_brisk.c','ok_brisk_detector.c','ok_brisk_descriptor.c','ok_brisk_camera.c')]
        for sanitized in (False,True):
            binary=O/('check_ok_brisk_san' if sanitized else 'check_ok_brisk')
            flags=['-std=c99','-O1' if sanitized else '-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra']
            if sanitized:flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-no-pie']
            run(['gcc']+flags+sources+['-lm','-o',binary],log=O/('build_san.log' if sanitized else 'build_c.log'))
            checkenv=dict(os.environ,ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
            result=run([binary,dirs[0]]+fixtures,checkenv,O/f'check_cam{cam}_{"san" if sanitized else "normal"}.log')
            print(result.splitlines()[-1],flush=True);results.append({'camera':cam,'sanitizer':sanitized,'result':result.splitlines()[-1]})
            if cam==0:
                api_binary=O/('check_api_san' if sanitized else 'check_api')
                run(['gcc']+flags+[R/'okvis_port/c/check_ok_brisk_api.c']+sources[1:]+['-lm','-o',api_binary],log=O/f'build_api_{sanitized}.log')
                api_result=run([api_binary],checkenv,O/f'api_{sanitized}.log');print(api_result.strip())
                neg=O/'negative';neg.mkdir(exist_ok=True)
                source=fixtures[0].read_bytes()
                for label,data in [('truncated',source[:100]),('trailing',source+b'bad'),('mismatch',source[:32]+bytes([source[32]^255])+source[33:])]:
                    badfile=neg/(label+'.bin');badfile.write_bytes(data)
                    p=subprocess.run([str(binary),str(dirs[0]),str(badfile)],env=checkenv,capture_output=True,text=True)
                    (neg/f'{label}_{sanitized}.log').write_text(p.stdout+p.stderr)
                    assert p.returncode in (1,2,3),f'Bad fixture accepted/crashed: {label} ({p.returncode})'
                    assert 'AddressSanitizer' not in p.stderr and 'runtime error:' not in p.stderr,p.stderr

    usage=sum(p.stat().st_size for p in O.rglob('*') if p.is_file());assert usage<1500000000,usage
    report={'results':results,'deterministic_file_hashes':hashes,'artifact_bytes':usage,'image_sha256':{str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest() for p in images},'real_image_count':len(frames),'synthetic_image_count':len(cases),'note':'Camera-1 calibration tested on the same cam0 image inputs; gravity vectors are controlled poses, not full VIO replay. LeakSanitizer disabled because the sandbox uses ptrace.'}
    (O/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print('Artifact bytes',usage)
if __name__=='__main__':main()
