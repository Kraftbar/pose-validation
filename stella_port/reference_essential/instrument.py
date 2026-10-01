#!/usr/bin/env python3
"""Generate an isolated upstream copy plus a reviewable trace-only diff."""
from pathlib import Path
import difflib,json,hashlib
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'runs/stella_port/reference_essential/instrumented';OUT.mkdir(parents=True,exist_ok=True)
SRC=ROOT/'external/candidates/stella_vslam/src/stella_vslam/solve'
outputs={}
s=(SRC/'essential_solver.h').read_text().replace('STELLA_VSLAM_SOLVE_ESSENTIAL_SOLVER_H','SV_OBSERVED_ESSENTIAL_SOLVER_H').replace('namespace solve {','namespace essential_reference {')
(OUT/'essential_solver.h').write_text(s);outputs['essential_solver.h']=s
s=(SRC/'essential_5pt.h').read_text().replace('namespace stella_vslam {','namespace stella_vslam {\nnamespace essential_reference {').replace('} // namespace stella_vslam','} // namespace essential_reference\n} // namespace stella_vslam')
s=s.replace('#include "stella_vslam/type.h"','#include "stella_vslam/type.h"\n#include "trace_hooks.h"')
s=s.replace('const Eigen::FullPivLU<MatX_t> lu(epipolar_constraint);','const Eigen::FullPivLU<MatX_t> lu(epipolar_constraint);\n        essential_trace::matrix(essential_trace::minimal.constraint, epipolar_constraint);\n        essential_trace::minimal.rank = lu.rank();')
(OUT/'essential_5pt.h').write_text(s);outputs['essential_5pt.h']=s
s=(SRC/'essential_solver.cc').read_text().replace('namespace solve {','namespace essential_reference {').replace('"stella_vslam/solve/essential_5pt.h"','"essential_5pt.h"').replace('"stella_vslam/solve/essential_solver.h"','"essential_solver.h"')
s=s.replace('const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1, random_engine_);','const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1, random_engine_);\n        essential_trace::begin(indices);')
s=s.replace('    solution_is_valid_ = best_cost_', '    solution_is_valid_ = best_cost_',1)
s=s.replace('        }\n    }\n\n    solution_is_valid_', '        }\n        essential_trace::best(best_cost_, best_num_inliers, best_E_21_);\n    }\n\n    solution_is_valid_',1)
s=s.replace('    return E_mats;', '    essential_trace::finish_minimal(E_mats);\n    return E_mats;')
s=s.replace('    // Use the epipolar constaints', '    essential_trace::matrix(essential_trace::minimal.basis, E_basis);\n\n    // Use the epipolar constaints')
s=s.replace('    // Step 3: Apply', '    essential_trace::matrix(essential_trace::minimal.polynomial, constraint_matrix);\n\n    // Step 3: Apply')
s=s.replace('    // Solving the eliminated matrix', '    essential_trace::matrix(essential_trace::minimal.eliminated, eliminated_matrix);\n\n    // Solving the eliminated matrix')
s=s.replace('    // Get the solutions to', '    essential_trace::matrix(essential_trace::minimal.action, action_matrix);\n\n    // Get the solutions to')
s=s.replace('    // Build essential matrices by', '    essential_trace::matrix(essential_trace::minimal.vectors_real, eig_vecs.real());\n    essential_trace::matrix(essential_trace::minimal.vectors_imag, eig_vecs.imag());\n    for(int i=0;i<10;i++){essential_trace::minimal.eigen_real[i]=eig_vals(i).real();essential_trace::minimal.eigen_imag[i]=eig_vals(i).imag();}\n\n    // Build essential matrices by')
s=s.replace('    return num_inliers;', '    essential_trace::inliers(cost, num_inliers, is_inlier_match);\n    return num_inliers;')
(OUT/'essential_solver.cc').write_text(s);outputs['essential_solver.cc']=s
patch=''.join(''.join(difflib.unified_diff((SRC/name).read_text().splitlines(True),s.splitlines(True),fromfile='a/'+name,tofile='b/'+name)) for name,s in outputs.items())
(ROOT/'stella_port/reference_essential/trace.patch').write_text(patch)
(OUT/'sources.json').write_text(json.dumps({str(SRC/name):hashlib.sha256((SRC/name).read_bytes()).hexdigest() for name in outputs},indent=2)+'\n')
