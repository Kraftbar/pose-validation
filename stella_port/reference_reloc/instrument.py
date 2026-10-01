#!/usr/bin/env python3
"""Isolate upstream PnP and add trace observers without changing its math."""
from pathlib import Path
import difflib, json, hashlib, re
ROOT=Path(__file__).resolve().parents[2]
SRC=ROOT/'external/candidates/stella_vslam/src/stella_vslam/solve'
OUT=ROOT/'runs/stella_port/reference_reloc/instrumented'
OUT.mkdir(parents=True,exist_ok=True)
original={name:(SRC/name).read_text() for name in ['pnp_solver.h','pnp_solver.cc']}
output={}
output['pnp_solver.h']=original['pnp_solver.h'].replace('STELLA_VSLAM_SOLVE_PNP_SOLVER_H','SV_OBSERVED_PNP_SOLVER_H').replace('namespace solve {','namespace reloc_reference {')
s=original['pnp_solver.cc'].replace('"stella_vslam/solve/pnp_solver.h"','"pnp_solver.h"\n#include "pnp_trace.hpp"').replace('namespace solve {','namespace reloc_reference {')
def after(old,new):
 global s
 assert old in s,old
 s=s.replace(old,old+'\n'+new)
after('    const eigen_alloc_vector<Vec3_t> control_points = choose_control_points(pos_ws);','    reloc_trace::vecs("controls", control_points);')
after('    const eigen_alloc_vector<Vec4_t> alphas = compute_barycentric_coordinates(control_points, pos_ws);','    reloc_trace::vecs("alphas", alphas);')
after('    const MatX_t M = compute_M(bearing_vectors, alphas);','    reloc_trace::mat("M", M);')
after('    const MatRC_t<12, 12> MtM = M.transpose() * M;','    reloc_trace::mat("MtM", MtM);')
after('    const MatRC_t<12, 12> U = SVD.matrixU();','    reloc_trace::mat("U12", U);')
after('    const MatRC_t<6, 10> L_6x10 = compute_L_6x10(U);','    reloc_trace::mat("L", L_6x10);')
after('    const MatRC_t<6, 1> Rho = compute_rho(control_points);','    reloc_trace::mat("rho", Rho);')
after('        const Vec4_t betas = find_initial_betas(L_6x10, Rho, N);','        reloc_trace::mat("betas", betas);')
after('        const Vec4_t refined_betas = gauss_newton(L_6x10, Rho, betas, num_iter);','        reloc_trace::mat("refined", refined_betas);')
after('        const eigen_alloc_vector<Vec3_t> ccs = compute_ccs(refined_betas, U);','        reloc_trace::vecs("ccs", ccs);')
after('        const eigen_alloc_vector<Vec3_t> pcs = compute_pcs(alphas, ccs, bearing_z_sign);','        reloc_trace::vecs("pcs", pcs);')
after('        estimate_R_and_t(pos_ws, pcs, rot_cand, trans_cand);','        reloc_trace::mat("rotation", rot_cand);\n        reloc_trace::mat("translation", trans_cand);')
after('        const auto reproj_error = reprojection_error(pos_ws, bearing_vectors, rot_cand, trans_cand);','        reloc_trace::scalar("error", reproj_error);')
after('    const MatX_t PW0tPW0 = PW0.transpose() * PW0;','    reloc_trace::mat("PW0", PW0);\n    reloc_trace::mat("PW0tPW0", PW0tPW0);')
after('    const MatX_t U = SVD.matrixU();','    reloc_trace::mat("control_U", U);\n    reloc_trace::mat("control_D", D);')
after('    const Mat33_t CC_inv = svd.matrixV() * S * svd.matrixU().transpose();','    reloc_trace::mat("CC_inv", CC_inv);')
after('    const MatX_t& CM_vt = SVD.matrixV().transpose();','    reloc_trace::mat("CM", CM);\n    reloc_trace::mat("CM_U", CM_u);\n    reloc_trace::mat("CM_Vt", CM_vt);')
after('        compute_A_and_b_for_gauss_newton(L_6x10, Rho, betas, A, B);','        reloc_trace::mat("GN_A", A);\n        reloc_trace::mat("GN_B", B);')
after('        betas += A.householderQr().solve(B);','        reloc_trace::mat("GN_betas", betas);')
for count in [3,4,5]:
 after(f'    const Vec{count}_t b{count} = SVD.solve(Rho);',f'    reloc_trace::mat("beta_solution", b{count});')
after('        assert(random_indices.size() == min_set_size);','        for(auto idx : random_indices) reloc_trace::scalar("sample", idx);')
after('        const auto num_inliers = check_inliers(rot_cw_in_sac, trans_cw_in_sac, is_inlier_match_in_sac, cost);','        reloc_trace::scalar("cost", cost);\n        reloc_trace::scalar("inliers", num_inliers);\n        for(bool inlier : is_inlier_match_in_sac) reloc_trace::scalar("mask", inlier);')
after('    solution_is_valid_ = min_cost < std::numeric_limits<double>::max();','    reloc_trace::scalar("valid", solution_is_valid_);')
output['pnp_solver.cc']=s
patch='Isolated observer copy of stella_vslam e445b545 PnP; namespace renamed to avoid interposition.\n\n'
for name,new in output.items():
 (OUT/name).write_text(new)
 patch+=''.join(difflib.unified_diff(original[name].splitlines(True),new.splitlines(True),fromfile='a/src/stella_vslam/solve/'+name,tofile='b/src/stella_vslam/solve/'+name))
patchdir=ROOT/'stella_port/reference_reloc/patches';patchdir.mkdir(exist_ok=True)
existing=list(patchdir.glob('*pnp-trace.patch'))
if existing:
 assert len(existing)==1
 assert existing[0].read_text()==patch,'Trace changed: create a new reviewed patch'
 target=existing[0]
else:
 # Re-check ALL patch namespaces immediately before exclusively creating ours.
 numbers=[int(p.name[:4]) for p in (ROOT/'stella_port').rglob('*.patch') if re.match(r'^\d{4}-',p.name)]
 target=patchdir/f'{max(numbers,default=0)+1:04d}-pnp-trace.patch'
 with target.open('x') as f:f.write(patch)
(OUT/'sources.json').write_text(json.dumps({str(SRC/n):hashlib.sha256((SRC/n).read_bytes()).hexdigest() for n in original},indent=2)+'\n')
print(target.relative_to(ROOT))
