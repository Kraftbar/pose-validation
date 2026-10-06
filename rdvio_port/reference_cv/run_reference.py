#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Stream a complete tagged reference run through the C99 M7 checker."""
import argparse, json, os, subprocess, sys
from pathlib import Path
from build import ROOT,OUT,PORT,FLAGS,run
p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');p.add_argument('--tag',default='m7_stream');p.add_argument('--max-seconds');a=p.parse_args()
tag=ROOT/'runs/rdvio_port'/a.tag;tag.mkdir(parents=True,exist_ok=True)
exe=OUT/('check_rd_cv_stream_san' if a.sanitize else 'check_rd_cv_stream')
run(['cc','-std=c99',*FLAGS,'-Wall','-Wextra',*(['-fsanitize=address,undefined','-g','-fno-omit-frame-pointer','-no-pie'] if a.sanitize else []),PORT/'c/check_rd_cv_stream.c',PORT/'c/rd_cv.c',PORT/'c/rd_cv_lk.c',PORT/'c/rd_cv_gftt.c','-lm','-o',exe])
fifo=tag/'image.fifo'
if fifo.exists():
    if not fifo.is_fifo():raise RuntimeError('refusing to replace non-FIFO')
    fifo.unlink()
os.mkfifo(fifo)
env=dict(os.environ,RDVIO_CV_GRAY=str(ROOT/'external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray'),
         RDVIO_CV_STREAM=str(fifo),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENCV_FOR_THREADS_NUM='1',ASAN_OPTIONS='detect_leaks=0')
cmd=[sys.executable,str(ROOT/'tools/run_rdvio_reference.py'),'MH_01_easy','--tag',a.tag,'--stock-binary',str(ROOT/'runs/rdvio_port/m7_reference_build/build/rdvio_ref_driver')]
if a.max_seconds:cmd+=['--max-seconds',a.max_seconds]
try:
    with (tag/'check.log').open('w') as log,(tag/'check.err').open('w') as err:
        checker=subprocess.Popen([str(exe),str(fifo)],env=env,stdout=log,stderr=err)
        result=subprocess.run(cmd,env=env)
        meta=json.loads((tag/'MH_01_easy/run.json').read_text())
        if result.returncode or meta['exit']:
            checker.terminate()
        rc=checker.wait(timeout=60)
    meta=json.loads((tag/'MH_01_easy/run.json').read_text())
    print((tag/'check.log').read_text().splitlines()[-5:])
    if result.returncode or meta['exit'] or rc:raise RuntimeError('reference or checker failed')
    if not a.max_seconds and meta['traj_sha256']!='f0d60a3e03c1243d750da735a5f5da1ec232237a6d84c4e9a35f15747377a0fe':raise RuntimeError('trajectory changed')
finally:
    if 'checker' in locals() and checker.poll() is None:
        checker.terminate()
        checker.wait(timeout=10)
    fifo.unlink(missing_ok=True)
