/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_eigen_mat4.h (MPL-2.0, Eigen-derived evaluation order). */
#include "sv_eigen_mat4.h"

#define A4(i, j) a[(j) * 4 + (i)]
#define B4(i, j) b[(j) * 4 + (i)]

/* A 4-row ColMajor destination is two full SSE2 2-wide double packets, so
 * every row is accumulated left-to-right/k-ascending by the packet pmadd
 * chain: ((a0*b0)+a1*b1)+a2*b2)+a3*b3 -- no scalar remainder row (unlike the
 * 3x3 case, whose third row goes through the scalar redux path). */
void sv_mat4_mul(const double a[16], const double b[16], double out[16]) {
    double res[16];
    int i, j;
    for (j = 0; j < 4; ++j) {
        for (i = 0; i < 4; ++i) {
            double s = A4(i, 0) * B4(0, j);
            s = s + A4(i, 1) * B4(1, j);
            s = s + A4(i, 2) * B4(2, j);
            s = s + A4(i, 3) * B4(3, j);
            res[j * 4 + i] = s;
        }
    }
    for (i = 0; i < 16; ++i) {
        out[i] = res[i];
    }
}
