#!/usr/bin/env python3
"""Define failed EPnP output when all three candidate errors are non-finite."""
from pathlib import Path
import difflib,re
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/'runs/stella_port/reference_reloc/instrumented/pnp_solver.cc'
s=p.read_text()
needle='    // EPnP: An AccurateO(n)Solution to the PnP Problem'
guard='''    // [reference-port] All-invalid candidates must not expose uninitialized
    // output to the RANSAC inlier test.
    rot_cw.setConstant(std::numeric_limits<double>::quiet_NaN());
    trans_cw.setConstant(std::numeric_limits<double>::quiet_NaN());
'''
if guard in s:raise SystemExit(0)
assert needle in s
new=s.replace(needle,guard+needle)
patch='Define all-invalid EPnP output. Intentionally changes undefined degenerate behavior; real-input public results are checked separately.\n\n'+''.join(difflib.unified_diff(s.splitlines(True),new.splitlines(True),fromfile='a/src/stella_vslam/solve/pnp_solver.cc',tofile='b/src/stella_vslam/solve/pnp_solver.cc'))
folder=ROOT/'stella_port/reference_reloc/patches';old=list(folder.glob('*pnp-empty-guard.patch'))
if old:
 assert len(old)==1 and old[0].read_text()==patch
else:
 numbers=[int(p.name[:4]) for p in (ROOT/'stella_port').rglob('*.patch') if re.match(r'^\d{4}-',p.name)]
 with (folder/f'{max(numbers)+1:04d}-pnp-empty-guard.patch').open('x') as f:f.write(patch)
p.write_text(new)
