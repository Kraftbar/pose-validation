/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_eigen_lu3.h (MPL-2.0, Eigen-derived). */
#include "sv_eigen_lu3.h"
#include <math.h>

#define LU(i, j) lu[(j) * 3 + (i)]

void sv_lu3_solve(const double w[9], const double b[3], double x[3]) {
    double lu[9];
    int perm[3] = {0, 1, 2}; /* row_transpositions applied to the rhs */
    int trans[3];
    int k, i, j;
    for (i = 0; i < 9; ++i) {
        lu[i] = w[i];
    }
    /* unblocked_lu: size 3 compile-time -> endk = size-1, last entry handled separately */
    for (k = 0; k < 2; ++k) {
        int best = k;
        double biggest = fabs(LU(k, k));
        for (i = k + 1; i < 3; ++i) { /* maxCoeff: first maximum */
            const double v = fabs(LU(i, k));
            if (v > biggest) {
                biggest = v;
                best = i;
            }
        }
        trans[k] = best;
        if (biggest != 0.0) {
            if (best != k) {
                for (j = 0; j < 3; ++j) {
                    const double tmp = LU(k, j);
                    LU(k, j) = LU(best, j);
                    LU(best, j) = tmp;
                }
            }
            for (i = k + 1; i < 3; ++i) {
                LU(i, k) = LU(i, k) / LU(k, k);
            }
        }
        for (j = k + 1; j < 3; ++j) {
            for (i = k + 1; i < 3; ++i) {
                LU(i, j) = LU(i, j) - LU(i, k) * LU(k, j);
            }
        }
    }
    trans[2] = 2;
    /* dst = P * rhs : apply the transpositions in order */
    {
        double y[3];
        y[0] = b[0];
        y[1] = b[1];
        y[2] = b[2];
        for (k = 0; k < 3; ++k) {
            const double tmp = y[k];
            y[k] = y[trans[k]];
            y[trans[k]] = tmp;
        }
        (void)perm;
        /* unit lower / upper, complete meta-unrolling (SolveTriangular.h
         * triangular_solver_unroller: rhs(i) -= row(i).segment.cwiseProduct(rhs.segment).sum(); rhs(i) /= diag).
         * A two-term sum is p0 + p1. */
        y[1] = y[1] - LU(1, 0) * y[0];
        y[2] = y[2] - (LU(2, 0) * y[0] + LU(2, 1) * y[1]);
        y[2] = y[2] / LU(2, 2);
        y[1] = y[1] - LU(1, 2) * y[2];
        y[1] = y[1] / LU(1, 1);
        y[0] = y[0] - (LU(0, 1) * y[1] + LU(0, 2) * y[2]);
        y[0] = y[0] / LU(0, 0);
        x[0] = y[0];
        x[1] = y[1];
        x[2] = y[2];
    }
}
