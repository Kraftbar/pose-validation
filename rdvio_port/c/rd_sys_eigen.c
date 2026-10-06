/* SPDX-License-Identifier: MPL-2.0 */
/* RD-VIO pure-C port: Eigen 3.4.0 models for the initializer. See rd_sys_eigen.h. */
#include "rd_sys_eigen.h"
#include "../../stella_port/c/sv_eigen_svd.h"
#include <float.h>
#include <math.h>

int rd_svd3_solve(const double A[9], const double b[3], double x[3]) {
    double U[9], V[9], sv[3], t[3], thr;
    int nonzero = 0, r, i, k;
    sv_eigen_jacobisvd_3x3(A, U, V, sv);
    while (nonzero < 3 && sv[nonzero] != 0.0) nonzero++;   /* m_nonzeroSingularValues */
    thr = sv[0] * (3.0 * DBL_EPSILON);
    if (thr < DBL_MIN) thr = DBL_MIN;                      /* numext::maxi(s0 * threshold(), min()) */
    r = nonzero;
    while (r > 0 && sv[r - 1] < thr) --r;
    for (i = 0; i < r; ++i) {                              /* tmp = U.leftCols(r).adjoint() * b */
        double s = U[3 * i] * b[0];
        s += U[3 * i + 1] * b[1];
        s += U[3 * i + 2] * b[2];
        t[i] = s;
    }
    for (i = 0; i < r; ++i) t[i] = (1.0 / sv[i]) * t[i];  /* asDiagonal().inverse() * tmp */
    for (k = 0; k < 3; ++k) {                              /* x = V.leftCols(r) * tmp */
        double s = 0.0;
        if (r > 0) { s = V[k] * t[0]; for (i = 1; i < r; ++i) s += V[k + 3 * i] * t[i]; }
        x[k] = s;
    }
    return r;
}

int rd_quat_from_two_vectors(const double a[3], const double b[3], ok_quat* q) {
    double v0[3], v1[3], c, axis[3], s, invs;
    v0[0] = a[0]; v0[1] = a[1]; v0[2] = a[2]; ok_v3_normalized(a, v0);
    v1[0] = b[0]; v1[1] = b[1]; v1[2] = b[2]; ok_v3_normalized(b, v1);
    c = (v1[0] * v0[0] + v1[1] * v0[1]) + v1[2] * v0[2];
    if (c < -1.0 + 1e-12) return 0;                         /* dummy_precision<double> */
    axis[0] = v0[1] * v1[2] - v0[2] * v1[1];
    axis[1] = v0[2] * v1[0] - v0[0] * v1[2];
    axis[2] = v0[0] * v1[1] - v0[1] * v1[0];
    s = sqrt((1.0 + c) * 2.0);
    invs = 1.0 / s;
    q->x = axis[0] * invs; q->y = axis[1] * invs; q->z = axis[2] * invs;
    q->w = s * 0.5;
    return 1;
}
