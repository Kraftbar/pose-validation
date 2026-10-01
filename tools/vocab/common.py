# SPDX-License-Identifier: MIT
import os,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/stella_port/vocab'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  while b:=f.read(1<<20):h.update(b)
 return h.hexdigest()
def env(native=False):
 e=dict(os.environ);base=ROOT/'external/candidates/deps/root/usr'
 libs=[OUT/'deps/usr/lib',OUT/'deps/usr/lib/x86_64-linux-gnu',base/'lib',base/'lib/x86_64-linux-gnu',Path('/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu')]
 if not native:libs.insert(0,ROOT/'runs/stella_port/reference_build/install/lib')
 e['LD_LIBRARY_PATH']=':'.join(map(str,libs));e['OMP_NUM_THREADS']='1';e['OPENBLAS_NUM_THREADS']='1';return e
