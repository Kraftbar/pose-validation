#!/usr/bin/env python3
import subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
PNP='sv_pnp.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c sv_linalg.c'.split()
RELOC=PNP+'sv_relocalizer.c sv_bow_db.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c'.split()
def build(name,san=False):
 sources=[ROOT/'stella_port/reference_reloc'/f'{name}.c',ROOT/'stella_port/reference_reloc/c/sv_eigen_pnp.c']+[ROOT/'stella_port/c'/s for s in RELOC]
 binary=ROOT/'runs/stella_port/reference_reloc'/(name+('_san' if san else ''))
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if san else []
 subprocess.run(['gcc','-std=c99','-O2','-g','-ffp-contract=off','-fno-fast-math',*flags,'-I'+str(ROOT/'stella_port/c'),'-I'+str(ROOT/'stella_port/reference_reloc'),*map(str,sources),'-lm','-o',str(binary)],check=True)
 return binary
if __name__=='__main__':build('check_reloc')
