#!/usr/bin/env python3
"""Deterministic rejection of an Eigen NumericalIssue in EPnP alignment."""
from pathlib import Path
import difflib,re
ROOT=Path(__file__).resolve().parents[2];p=ROOT/'runs/stella_port/reference_reloc/instrumented/pnp_solver.cc'
s=p.read_text();needle='    Eigen::JacobiSVD<MatX_t> SVD(CM, Eigen::ComputeFullV | Eigen::ComputeFullU);'
guard='''
    // [reference-port] Degenerate beta candidates can contain NaNs. Eigen
    // returns NumericalIssue without initializing U/V; do not read them.
    if (SVD.info() != Eigen::Success) {
        reloc_trace::mat("CM", CM);
        reloc_trace::scalar("CM_failed", 1);
        rot.setConstant(std::numeric_limits<double>::quiet_NaN());
        trans.setConstant(std::numeric_limits<double>::quiet_NaN());
        return;
    }'''
if guard in s:raise SystemExit(0)
assert needle in s
new=s.replace(needle,needle+guard)
patch='EPnP deterministic rejected-candidate guard: avoid uninitialized Eigen U/V after NumericalIssue.\nPublic real-input results are checked against installed stella. Undefined fully degenerate cases need the separate all-invalid-output guard.\n\n'+''.join(difflib.unified_diff(s.splitlines(True),new.splitlines(True),fromfile='a/src/stella_vslam/solve/pnp_solver.cc',tofile='b/src/stella_vslam/solve/pnp_solver.cc'))
patchdir=ROOT/'stella_port/reference_reloc/patches'
old=list(patchdir.glob('*pnp-degenerate-guard.patch'))
if old:
 assert len(old)==1 and old[0].read_text()==patch
 target=old[0]
else:
 numbers=[int(p.name[:4]) for p in (ROOT/'stella_port').rglob('*.patch') if re.match(r'^\d{4}-',p.name)]
 target=patchdir/f'{max(numbers,default=0)+1:04d}-pnp-degenerate-guard.patch'
 with target.open('x') as f:f.write(patch)
p.write_text(new);print(target.relative_to(ROOT))
