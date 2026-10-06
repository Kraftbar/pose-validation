#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build the native M7 oracle and standalone C99 checker, without shared builds."""
import argparse, hashlib, json, os, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/rdvio_port/reference_cv'
CV=ROOT/'external/vio/deps/opencv'
PORT=ROOT/'rdvio_port'
FLAGS=['-O2','-DNDEBUG','-ffp-contract=off','-fno-fast-math']
def run(cmd):
    print('+',' '.join(map(str,cmd)),flush=True)
    subprocess.run(list(map(str,cmd)),check=True)
def build(sanitize=False):
    OUT.mkdir(parents=True,exist_ok=True)
    run(['c++','-std=c++17',*FLAGS,'-I'+str(CV/'include/opencv4'),PORT/'reference_cv/dump_main.cpp',
         '-L'+str(CV/'lib'),'-Wl,-rpath,'+str(CV/'lib'),'-lopencv_video','-lopencv_features2d','-lopencv_calib3d','-lopencv_imgproc','-lopencv_core','-o',OUT/'dump_main'])
    exe=OUT/('check_rd_cv_san' if sanitize else 'check_rd_cv')
    run(['cc','-std=c99',*FLAGS,'-Wall','-Wextra',*(['-fsanitize=address,undefined','-g','-fno-omit-frame-pointer','-no-pie'] if sanitize else []),
         PORT/'c/check_rd_cv.c',PORT/'c/rd_cv.c',PORT/'c/rd_cv_lk.c',PORT/'c/rd_cv_gftt.c','-lm','-o',exe])
    def sha(path):
        h=hashlib.sha256()
        with path.open('rb') as f:
            for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
        return h.hexdigest()
    inputs=[*sorted((PORT/'c').glob('rd_cv*')),*sorted((PORT/'c').glob('check_rd_cv*.c')),
            PORT/'reference_cv/dump_main.cpp']
    libraries=[CV/'lib'/f'libopencv_{m}.so.4.6.0' for m in ('core','imgproc','video','features2d','calib3d')]
    provenance={'compiler':subprocess.check_output(['cc','--version'],text=True).splitlines()[0],
                'flags':FLAGS,'sanitize':sanitize,'opencv':'4.6.0, installed native libraries (their own Release flags are in dispatch_default.txt)',
                'sources':{str(p.relative_to(ROOT)):sha(p) for p in inputs},
                'libraries':{str(p.relative_to(ROOT)):sha(p) for p in libraries},
                'harness_sha256':sha(exe),'oracle_sha256':sha(OUT/'dump_main')}
    (OUT/('provenance_san.json' if sanitize else 'provenance_normal.json')).write_text(json.dumps(provenance,indent=2)+'\n')
    return exe
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');a=p.parse_args();build(a.sanitize)
