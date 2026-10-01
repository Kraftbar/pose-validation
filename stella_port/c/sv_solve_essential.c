/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_solve_essential.h (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_solve_essential.h"
#include "sv_linalg.h"
#include "sv_eigen_svd.h"
#include <string.h>

void sv_solve_essential_decompose(const double E21[9], double rots[4][9], double transes[4][3]) {
    double U[9], V[9], Vt[9], sv[3];
    double trans[3], nrm;
    double W[9], Wt[9];
    double UW[9], rot1[9], UWt[9], rot2[9];

    sv_eigen_jacobisvd_3x3(E21, U, V, sv);
    sv_mat3_transpose(V, Vt);

    /* trans = U.col(2), normalized */
    trans[0] = U[2 * 3 + 0];
    trans[1] = U[2 * 3 + 1];
    trans[2] = U[2 * 3 + 2];
    nrm = sv_vec3_norm(trans);
    trans[0] /= nrm; trans[1] /= nrm; trans[2] /= nrm;

    memset(W, 0, sizeof(W));
    W[1 * 3 + 0] = -1.0; /* (0,1) */
    W[0 * 3 + 1] = 1.0;  /* (1,0) */
    W[2 * 3 + 2] = 1.0;  /* (2,2) */
    sv_mat3_transpose(W, Wt);

    sv_mat3_mul(U, W, UW);
    sv_mat3_mul(UW, Vt, rot1);
    if (sv_mat3_det(rot1) < 0) {
        int k;
        for (k = 0; k < 9; ++k) rot1[k] = -rot1[k];
    }

    sv_mat3_mul(U, Wt, UWt);
    sv_mat3_mul(UWt, Vt, rot2);
    if (sv_mat3_det(rot2) < 0) {
        int k;
        for (k = 0; k < 9; ++k) rot2[k] = -rot2[k];
    }

    memcpy(rots[0], rot1, sizeof(rot1));
    memcpy(rots[1], rot1, sizeof(rot1));
    memcpy(rots[2], rot2, sizeof(rot2));
    memcpy(rots[3], rot2, sizeof(rot2));

    transes[0][0] = trans[0]; transes[0][1] = trans[1]; transes[0][2] = trans[2];
    transes[1][0] = -trans[0]; transes[1][1] = -trans[1]; transes[1][2] = -trans[2];
    transes[2][0] = trans[0]; transes[2][1] = trans[1]; transes[2][2] = trans[2];
    transes[3][0] = -trans[0]; transes[3][1] = -trans[1]; transes[3][2] = -trans[2];
}
