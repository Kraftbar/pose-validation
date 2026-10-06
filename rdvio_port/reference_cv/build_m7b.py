#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import argparse,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];PORT=ROOT/'rdvio_port';OUT=ROOT/'runs/rdvio_port/reference_cv_m7b';CV=ROOT/'external/vio/deps/opencv'
FLAGS=['-O2','-DNDEBUG','-ffp-contract=off','-fno-fast-math']
def run(cmd,**kw):
 print('+',' '.join(map(str,cmd)),flush=True);return subprocess.run(list(map(str,cmd)),check=True,**kw)
def build(san=False):
 OUT.mkdir(exist_ok=True,parents=True)
 run([sys.executable,"-B",PORT/"reference_cv/prepare_pnp_trace.py"])
 for piece in ['undist','pnp']:
  src=PORT/f'reference_cv/dump_{piece}.cpp'
  if not src.exists():continue
  extras=[OUT/'epnp_trace.cpp'] if piece=='pnp' else []
  run(['c++','-std=c++17',*FLAGS,'-I'+str(CV/'include/opencv4'),'-I'+str(OUT),'-I'+str(PORT/'reference_cv'),src,*extras,'-L'+str(CV/'lib'),'-Wl,-rpath,'+str(CV/'lib'),'-lopencv_calib3d','-lopencv_imgproc','-lopencv_core','-o',OUT/f'dump_{piece}'])
  sources=[PORT/f'c/rd_cv_{piece}.c']+([PORT/'c/rd_cv_pnp_math.c'] if piece=='pnp' else [])
  run(['cc','-std=c99',*FLAGS,'-Wall','-Wextra',*(['-fsanitize=address,undefined','-g','-fno-omit-frame-pointer','-no-pie'] if san else []),PORT/f'c/check_rd_cv_{piece}.c',*sources,'-lm','-o',OUT/('check_'+piece+('_san' if san else ''))])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');a=p.parse_args();build(a.sanitize)
