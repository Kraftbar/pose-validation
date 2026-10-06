#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""System variant (b), replacing only its OpenCV callback with native EPnP."""
import argparse, hashlib, importlib.util, json, os, subprocess
from build_m7b import ROOT, PORT, OUT, FLAGS, run

def main():
    p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');a=p.parse_args()
    spec=importlib.util.spec_from_file_location('checks',ROOT/'tools/check_rdvio_port.py')
    checks=importlib.util.module_from_spec(spec);spec.loader.exec_module(checks)
    od=OUT/'system';od.mkdir(parents=True,exist_ok=True)
    mode='san' if a.sanitize else 'normal'
    exe=od/('rdvio_c_'+mode)
    sources=[PORT/'c'/s for s in checks.SYS_SRC]
    sources += [checks.OK/s for s in ('ok_kin.c','ok_blas.c','ok_sparse.c','ok_amd.c','ok_eigen.c','ok_dense.c')]
    sources += [checks.STELLA/s for s in ('sv_eigen_svd.c','sv_eigen_qr.c','sv_eigen_eigensolver.c')]
    sources += [PORT/'c/rd_cv_pnp.c',PORT/'c/rd_cv_pnp_math.c']
    san=['-fsanitize=address,undefined','-g','-fno-omit-frame-pointer','-no-pie'] if a.sanitize else []
    with (od/('build_'+mode+'.log')).open('w') as log:
        run(['cc','-std=c99',*FLAGS,*san,'-I'+str(checks.OK),'-DRD_PNP_OPENCV',
             '-Drd_pnp6_opencv=rd_cv_pnp6',*sources,'-lm','-o',exe],stdout=log,stderr=subprocess.STDOUT)
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',ASAN_OPTIONS='detect_leaks=0',UBSAN_OPTIONS='halt_on_error=1')
    traj=od/('traj_'+mode+'.tum')
    with (od/('run_'+mode+'.log')).open('w') as log:
        run([exe,PORT/'reference/configs/euroc_sensor.yaml',PORT/'reference/configs/setting.yaml',
             ROOT/'runs/rdvio_port/data/MH_01_easy/mav0',traj,'--gray',
             ROOT/'runs/rdvio_port/system/MH_01_easy.undist.gray'],env=env,stdout=log,stderr=subprocess.STDOUT)
    ref=ROOT/'runs/rdvio_port/m7b_reference_run/MH_01_easy/traj.tum'
    expected=ref.read_bytes();actual=traj.read_bytes()
    canonical='f0d60a3e03c1243d750da735a5f5da1ec232237a6d84c4e9a35f15747377a0fe'
    if hashlib.sha256(expected).hexdigest()!=canonical:raise RuntimeError('reference is not canonical')
    mismatches=sum(a!=b for a,b in zip(actual,expected))+abs(len(actual)-len(expected))
    result={'cases':len(expected.splitlines()),'bytes':len(expected),'mismatches':mismatches,
            'sha256':hashlib.sha256(actual).hexdigest(),'mode':mode}
    (od/('validation_'+mode+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));print(f'm7b_sys: {mismatches}/{len(expected)}')
    if mismatches:raise SystemExit(1)

if __name__=='__main__':main()
