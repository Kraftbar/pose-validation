/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_SOLVE_ESSENTIAL_H
#define SV_SOLVE_ESSENTIAL_H

/* Port of stella_vslam's solve/essential_solver.cc::decompose() -- BSD-2
 * (AIST 2019 + stella-cv 2022, see sv_rng.h). SVD via sv_eigen_svd.h
 * (MPL-2.0, used not reproduced). Column-major 3x3/Vec3 (sv_linalg.h). */
void sv_solve_essential_decompose(const double E21[9], double rots[4][9], double transes[4][3]);

#endif /* SV_SOLVE_ESSENTIAL_H */
