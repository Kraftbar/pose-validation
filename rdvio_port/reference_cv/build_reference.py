#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the existing reference builder in an owned M7 tree, with a gray-stream driver.
The original reference tree and driver are never edited. Imread is replaced by
reading the user-provided, byte-exact decoded stream; remap remains identical.
"""
import importlib.util,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('rd_build',ROOT/'tools/build_rdvio_reference.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
# optional: --root <name under runs/rdvio_port> (default m7_reference_build; e.g. m6_reference_build for the M6 map log)
name='m7_reference_build'
if '--root' in sys.argv: name=sys.argv[sys.argv.index('--root')+1]
b.ROOT=ROOT/'runs/rdvio_port'/name;b.SRC=b.ROOT/'src';b.BUILD=b.ROOT/'build'
b.ROOT.mkdir(parents=True,exist_ok=True)
s=b.DRIVER.read_text()
s=s.replace('  string calib = argv[1]', '  cv::setNumThreads(1);\n  string calib = argv[1]')
s=s.replace('  double t0 = min(imus.front().t, cams.front().first);','''  const char *graypath = getenv("RDVIO_CV_GRAY");
  if (!graypath) return 2;
  FILE *gray = fopen(graypath, "rb");
  char magic[8]; uint32_t gh[3];
  if (!gray || fread(magic,1,8,gray)!=8 || memcmp(magic,"OKGRAY1\\0",8) || fread(gh,4,3,gray)!=3 ||
      gh[0]!=(uint32_t)res[0] || gh[1]!=(uint32_t)res[1] || gh[2]!=cams.size()) return 3;
  double t0 = min(imus.front().t, cams.front().first);''')
s=s.replace('    cv::Mat raw = cv::imread(d + "/cam0/data/" + c.second, cv::IMREAD_GRAYSCALE);','''    uint64_t graystamp;
    cv::Mat raw(res[1],res[0],CV_8UC1);
    if (fread(&graystamp,8,1,gray)!=1 || graystamp != stoull(c.second.substr(0,c.second.find('.'))) ||
        fread(raw.data,1,raw.total(),gray)!=raw.total()) return 4;''')
b.DRIVER=b.ROOT/'rdvio_gray_driver.cpp';b.DRIVER.write_text(s)
sys.argv=[str(ROOT/'tools/build_rdvio_reference.py'),'--jobs','4']
b.main()
