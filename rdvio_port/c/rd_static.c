/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* See rd_static.h. */
#include "rd_static.h"

#define ST(p, i, v, op) do { if ((op) > 0) (p)[i] += (v); else if ((op) < 0) (p)[i] -= (v); else (p)[i] = (v); } while (0)

void rd_st_mtm_2_3(const double* A, const double* B, double* C, int cs, int op) {
    int i, j;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            const double v = A[i] * B[j] + A[3 + i] * B[3 + j];
            ST(C, i * cs + j, v, op);
        }
}
void rd_st_mtm_3_3(const double* A, const double* B, double* C, int cs, int op) {
    int i, j;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            const double v = A[i] * B[j] + (A[3 + i] * B[3 + j] + A[6 + i] * B[6 + j]);
            ST(C, i * cs + j, v, op);
        }
}
void rd_st_mmm_3_3(const double* A, const double* B, double* C, int cs, int op) {
    int i, j;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            /* row-major lhs, row-major rhs: the destination columns 0 and 1 form one packet (left fold over k), column 2 is the scalar
             * tail (halving tree) */
            const double v = j < 2 ? (A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j]) + A[i * 3 + 2] * B[6 + j]
                                   : A[i * 3] * B[j] + (A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]);
            ST(C, i * cs + j, v, op);
        }
}
/* InvertPSDMatrix<3> with assume_full_rank: kSize in (0, 5) takes `m.inverse()`: Eigen's closed-form 3x3 cofactor inverse (compute_inverse
 * <_,_,3>) on the full row-major matrix (both triangles are read). Same arithmetic as rd_inverse3 (column-major Matrix3d) except the
 * determinant: (cofactors_col0 .* matrix.col(0)).sum() reads a STRIDED column of the row-major matrix, so it is not vectorised and the
 * redux is the unrolled halving tree c0 m00 + (c1 m10 + c2 m20). */
#define RM(i, j) m[(i) * 3 + (j)]
static double cof(const double* m, int i, int j) {
    const int i1 = (i + 1) % 3, i2 = (i + 2) % 3, j1 = (j + 1) % 3, j2 = (j + 2) % 3;
    return RM(i1, j1) * RM(i2, j2) - RM(i1, j2) * RM(i2, j1);
}
void rd_st_invert_psd3(const double* m, double* out) {
    const double c0 = cof(m, 0, 0), c1 = cof(m, 1, 0), c2 = cof(m, 2, 0);
    const double det = c0 * RM(0, 0) + (c1 * RM(1, 0) + c2 * RM(2, 0));
    const double invdet = 1.0 / det;
    out[0] = c0 * invdet; out[1] = c1 * invdet; out[2] = c2 * invdet;            /* result(0, j) = cofactor(j, 0) */
    out[3] = cof(m, 0, 1) * invdet; out[4] = cof(m, 1, 1) * invdet; out[5] = cof(m, 2, 1) * invdet;
    out[6] = cof(m, 0, 2) * invdet; out[7] = cof(m, 1, 2) * invdet; out[8] = cof(m, 2, 2) * invdet;
}
#undef RM
void rd_st_inv_times_y(const double* inv, const double* y, double* out) {
    int i;
    for (i = 0; i < 3; ++i) out[i] = (inv[i * 3] * y[0] + inv[i * 3 + 1] * y[1]) + inv[i * 3 + 2] * y[2];   /* left fold (measured) */
}
