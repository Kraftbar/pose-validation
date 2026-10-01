/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_linalg.h (MPL-2.0, Eigen-derived formulas). */
#include "sv_linalg.h"
#include <math.h>

#define A(i, j) a[(j) * 3 + (i)]
#define B(i, j) b[(j) * 3 + (i)]
#define O(i, j) out[(j) * 3 + (i)]

/* Eigen's Matrix3d*Matrix3d (and Matrix3d*Vector3d) go through
 * CoeffBasedProductMode's packet-vectorized evaluator
 * (ProductEvaluators.h etor_product_packet_impl<ColMajor,...>): a
 * dense_assignment_loop over a 3-row ColMajor destination processes rows
 * [0,2) as one SSE2 2-wide double packet -- accumulated
 * left-to-right/k-ascending via pmadd chains, i.e. ((a0*b0)+a1*b1)+a2*b2
 * for rows 0 and 1 simultaneously -- and the leftover row 2 through the
 * plain scalar coeff() path, `(lhs.row(2).cwiseProduct(rhs.col(col))).sum()`,
 * which reduces through Redux.h's redux_novec_unroller
 * (HalfLength=Length/2 split -> right-associative for 3 terms:
 * a0 + (a1+a2)). So rows 0-1 and row 2 use different associativity. */
void sv_mat3_mul(const double a[9], const double b[9], double out[9]) {
    int i, j;
    double res[9];
    for (j = 0; j < 3; ++j) {
        for (i = 0; i < 2; ++i) {
            double s = A(i, 0) * B(0, j);
            s = s + A(i, 1) * B(1, j);
            s = s + A(i, 2) * B(2, j);
            res[j * 3 + i] = s;
        }
        res[j * 3 + 2] = A(2, 0) * B(0, j) + (A(2, 1) * B(1, j) + A(2, 2) * B(2, j));
    }
    for (i = 0; i < 9; ++i) {
        out[i] = res[i];
    }
}

void sv_mat3_mulv(const double a[9], const double v[3], double out[3]) {
    int i;
    double res[3];
    for (i = 0; i < 2; ++i) {
        double s = A(i, 0) * v[0];
        s = s + A(i, 1) * v[1];
        s = s + A(i, 2) * v[2];
        res[i] = s;
    }
    res[2] = A(2, 0) * v[0] + (A(2, 1) * v[1] + A(2, 2) * v[2]);
    out[0] = res[0];
    out[1] = res[1];
    out[2] = res[2];
}

void sv_mat3_transpose(const double a[9], double out[9]) {
    double res[9];
    int i, j;
    for (j = 0; j < 3; ++j) {
        for (i = 0; i < 3; ++i) {
            res[i * 3 + j] = A(i, j);
        }
    }
    for (i = 0; i < 9; ++i) {
        out[i] = res[i];
    }
}

/* bruteforce_det3_helper(m,a,b,c) = m(0,a) * (m(1,b)*m(2,c) - m(1,c)*m(2,b)) */
static double bruteforce_det3(const double m[9], int a, int b, int c) {
#define M(i, j) m[(j) * 3 + (i)]
    return M(0, a) * (M(1, b) * M(2, c) - M(1, c) * M(2, b));
#undef M
}

/* determinant(): row-0 cofactor expansion, left-associative (measured,
 * HANDOVER.md "Eigen 3x3 double evaluation-order rules"). */
double sv_mat3_det(const double m[9]) {
    return (bruteforce_det3(m, 0, 1, 2) - bruteforce_det3(m, 1, 0, 2)) + bruteforce_det3(m, 2, 0, 1);
}

/* cofactor_3x3<i,j>(m) = m(i1,j1)*m(i2,j2) - m(i1,j2)*m(i2,j1), i1=(i+1)%3, i2=(i+2)%3 (same for j). */
static double cofactor3(const double m[9], int i, int j) {
    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
    int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
#define M(r, c) m[(c) * 3 + (r)]
    return M(i1, j1) * M(i2, j2) - M(i1, j2) * M(i2, j1);
#undef M
}

void sv_mat3_inverse(const double m[9], double out[9]) {
    double cofactors_col0[3];
    double det, invdet;
    double c01, c11, c02;
    cofactors_col0[0] = cofactor3(m, 0, 0);
    cofactors_col0[1] = cofactor3(m, 1, 0);
    cofactors_col0[2] = cofactor3(m, 2, 0);
    /* det = L(cof<0,0>*m00, cof<1,0>*m10, cof<2,0>*m20) (measured,
     * HANDOVER.md). */
    det = (cofactors_col0[0] * m[0 * 3 + 0] + cofactors_col0[1] * m[0 * 3 + 1]) + cofactors_col0[2] * m[0 * 3 + 2];
    invdet = 1.0 / det;

    c01 = cofactor3(m, 0, 1) * invdet;
    c11 = cofactor3(m, 1, 1) * invdet;
    c02 = cofactor3(m, 0, 2) * invdet;
    out[2 * 3 + 1] = cofactor3(m, 2, 1) * invdet; /* result(1,2) */
    out[1 * 3 + 2] = cofactor3(m, 1, 2) * invdet; /* result(2,1) */
    out[2 * 3 + 2] = cofactor3(m, 2, 2) * invdet; /* result(2,2) */
    out[0 * 3 + 1] = c01; /* result(1,0) */
    out[1 * 3 + 1] = c11; /* result(1,1) */
    out[0 * 3 + 2] = c02; /* result(2,0) */
    out[0 * 3 + 0] = cofactors_col0[0] * invdet; /* result(0,0) */
    out[1 * 3 + 0] = cofactors_col0[1] * invdet; /* result(0,1) */
    out[2 * 3 + 0] = cofactors_col0[2] * invdet; /* result(0,2) */
}

double sv_vec3_dot(const double a[3], const double b[3]) {
    double s = a[0] * b[0];
    s = s + a[1] * b[1];
    s = s + a[2] * b[2];
    return s;
}

void sv_vec3_cross(const double a[3], const double b[3], double out[3]) {
    double r0 = a[1] * b[2] - a[2] * b[1];
    double r1 = a[2] * b[0] - a[0] * b[2];
    double r2 = a[0] * b[1] - a[1] * b[0];
    out[0] = r0;
    out[1] = r1;
    out[2] = r2;
}

double sv_vec3_norm(const double a[3]) {
    return sqrt(sv_vec3_dot(a, a));
}

/* `A.transpose() * v` (e.g. `rot_21.transpose() * bearing_2` in
 * triangulator::triangulate / base::triangulate): measured (HANDOVER.md)
 * as ALL THREE rows left-associative -- unlike `Matrix3d*Matrix3d`/
 * `Matrix3d*Vector3d` (rows 0-1 L, row 2 R) and unlike
 * `Matrix3d*Matrix3d.transpose()` (which uses that SAME mixed rule, not a
 * different one -- so plain sv_mat3_mul is correct there, no separate
 * "rhs_transposed" variant is needed for the matrix*matrix case). */
void sv_mat3_mulv_lhs_transposed(const double a[9], const double v[3], double out[3]) {
    int i;
    double res[3];
    double at[9];
    sv_mat3_transpose(a, at);
    for (i = 0; i < 3; ++i) {
#define AT(r, c) at[(c) * 3 + (r)]
        double s = AT(i, 0) * v[0];
        s = s + AT(i, 1) * v[1];
        s = s + AT(i, 2) * v[2];
        res[i] = s;
#undef AT
    }
    out[0] = res[0];
    out[1] = res[1];
    out[2] = res[2];
}

/* `A.transpose() * B` (matrix-matrix, LHS transpose used inline) --
 * measured (bisected against real Eigen via
 * stella_port/reference_tools/debug_decompose.cc on
 * `cam_matrix_2.transpose() * F_21`): ALL rows left-associative, same
 * pattern as `A.transpose() * v` (sv_mat3_mulv_lhs_transposed), NOT the
 * mixed rows-0-1-L/row-2-R rule that `A*B` and `A*B.transpose()` use. */
void sv_mat3_mul_lhs_transposed(const double a[9], const double b[9], double out[9]) {
    int i, j;
    double res[9];
    double at[9];
    sv_mat3_transpose(a, at);
    for (j = 0; j < 3; ++j) {
        for (i = 0; i < 3; ++i) {
#define AT(r, c) at[(c) * 3 + (r)]
            double s = AT(i, 0) * B(0, j);
            s = s + AT(i, 1) * B(1, j);
            s = s + AT(i, 2) * B(2, j);
            res[j * 3 + i] = s;
#undef AT
        }
    }
    for (i = 0; i < 9; ++i) {
        out[i] = res[i];
    }
}
