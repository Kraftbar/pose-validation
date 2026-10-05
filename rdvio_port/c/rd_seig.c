/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * RD-VIO port, module M5: Eigen 3.4.0 SelfAdjointEigenSolver<MatrixXd> (dynamic size, n up to RD_EIG_MAX = 300, the marginalisation prior of a
 * 12-frame window is 165 x 165) as a bit-exact evaluation-order model. Generalised copy of okvis_port/c/ok_eigen.c `ok_selfadjoint_eig`
 * (Eigen's Tridiagonalization.h / SelfadjointMatrixVector.h / SelfAdjointEigenSolver.h / HouseholderSequence.h / Jacobi.h models; MPL-2.0,
 * Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors) with heap storage for the matrix and the scratch limits raised.
 * See rd_seig.h and rdvio_port/NOTICE.
 */
#include "rd_seig.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ------------------------ SelfAdjointEigenSolver<Matrix<double,n,n>> ------------------------ */
/* <float.h> is not in the allowed header set */
#define RD_DBL_MIN 2.2250738585072014e-308
#define RD_DBL_EPSILON 2.220446049250313e-16
#define EM(a, r, c) (a)[(r) + n * (c)]

/* Eigen redux (sum) over a contiguous dynamic-size expression without DirectAccess: packet size 2, two
 * 2-lane accumulators, stella_port/c/sv_eigen_eigensolver.c `redux` (measured bit-exact there). */
static double rd_redux(const double* v, int n) {
    int end, end4, i;
    double a, b, c, d, sum;
    if (n == 0) return 0.0;
    end = n / 2 * 2;
    end4 = n / 4 * 4;
    if (end) {
        a = v[0]; b = v[1];
        if (end > 2) {
            c = v[2]; d = v[3];
            for (i = 4; i < end4; i += 4) { a += v[i]; b += v[i + 1]; c += v[i + 2]; d += v[i + 3]; }
            a += c; b += d;
            if (end > end4) { a += v[end4]; b += v[end4 + 1]; }
        }
        sum = a + b;
        for (i = end; i < n; i++) sum += v[i];
        return sum;
    }
    return v[0];
}

/* selfadjoint_matrix_vector_product<double,long,ColMajor,Lower,false,false>::run
 * (SelfadjointMatrixVector.h) with the SSE2 Packet2d loop emulated lane by lane.
 * lhs(row,col) = L[row + lda*col], res indexed from `resbase` (absolute parity of resbase+idx decides
 * first_default_aligned). For size <= 8 (bound = 0) only the scalar single-column loop runs. */
static void rd_symv_lower(int size, const double* L, long lda, const double* rhs, double* res, int res_parity, double alpha) {
    const int bound = (size - 8 > 0 ? size - 8 : 0) & ~1;
    int j, i;
    for (j = 0; j < bound; j += 2) {
        const double* A0 = L + (long)j * lda;
        const double* A1 = L + (long)(j + 1) * lda;
        const double t0 = alpha * rhs[j];
        const double t1 = alpha * rhs[j + 1];
        double t2 = 0.0, t3 = 0.0;
        double p2[2] = {0.0, 0.0}, p3[2] = {0.0, 0.0};
        const int starti = j + 2, endi = size;
        int first_aligned = ((res_parity + starti) & 1) ? 1 : 0;
        if (first_aligned > endi - starti) first_aligned = endi - starti;
        {
            const int alignedStart = starti + first_aligned;
            const int alignedEnd = alignedStart + ((endi - alignedStart) / 2) * 2;
            res[j] += A0[j] * t0;
            res[j + 1] += A1[j + 1] * t1;
            res[j + 1] += A0[j + 1] * t0;
            t2 += A0[j + 1] * rhs[j + 1];
            for (i = starti; i < alignedStart; ++i) {
                res[i] += A0[i] * t0 + A1[i] * t1;
                t2 += A0[i] * rhs[i];
                t3 += A1[i] * rhs[i];
            }
            for (i = alignedStart; i < alignedEnd; i += 2) {
                int l;
                for (l = 0; l < 2; ++l) {
                    const double x = res[i + l];
                    const double inner = A1[i + l] * t1 + x;
                    res[i + l] = A0[i + l] * t0 + inner;
                    p2[l] = A0[i + l] * rhs[i + l] + p2[l];
                    p3[l] = A1[i + l] * rhs[i + l] + p3[l];
                }
            }
            for (i = alignedEnd; i < endi; i++) {
                res[i] += A0[i] * t0 + A1[i] * t1;
                t2 += A0[i] * rhs[i];
                t3 += A1[i] * rhs[i];
            }
            res[j] += alpha * (t2 + (p2[0] + p2[1]));
            res[j + 1] += alpha * (t3 + (p3[0] + p3[1]));
        }
    }
    for (j = bound; j < size; j++) {
        const double* A0 = L + (long)j * lda;
        const double t1 = alpha * rhs[j];
        double t2 = 0.0;
        res[j] += A0[j] * t1;
        for (i = j + 1; i < size; i++) {
            res[i] += A0[i] * t1;
            t2 += A0[i] * rhs[i];
        }
        res[j] += alpha * t2;
    }
}

/* tridiagonalization_inplace(matA, hCoeffs) for an n x n lower-referenced matrix (Tridiagonalization.h), n != 3.
 * hc_parity: alignment parity (in doubles, 0 = 16-byte aligned) of hCoeffs[0]; it only matters for n > 8. */
static void rd_tridiagonalize(int n, double* A, double* hC, int hc_parity) {
    int i, k, ii;
    for (i = 0; i < n - 1; ++i) {
        const int r = n - i - 1;
        double* x = &EM(A, i + 1, i);
        double h, beta, c0 = x[0], tail;
        tail = 0.0;
        if (r > 1) {
            double sq[RD_EIG_MAX];
            for (k = 1; k < r; ++k) sq[k - 1] = x[k] * x[k];
            tail = rd_redux(sq, r - 1);
        }
        if (tail <= RD_DBL_MIN) {
            h = 0.0; beta = c0;
            for (k = 1; k < r; ++k) x[k] = 0.0;
        } else {
            beta = sqrt(c0 * c0 + tail);
            if (c0 >= 0.0) beta = -beta;
            {
                const double den = c0 - beta;
                for (k = 1; k < r; ++k) x[k] = x[k] / den;
            }
            h = (beta - c0) / beta;
        }
        x[0] = 1.0;
        for (k = 0; k < r; ++k) hC[i + k] = 0.0;
        rd_symv_lower(r, &EM(A, i + 1, i + 1), n, x, hC + i, hc_parity + i, h);
        {
            double prod[RD_EIG_MAX], d, s;
            for (k = 0; k < r; ++k) prod[k] = hC[i + k] * x[k];
            d = rd_redux(prod, r);
            s = (h * -0.5) * d;
            for (k = 0; k < r; ++k) hC[i + k] += s * x[k];
        }
        for (ii = 0; ii < r; ++ii) {
            const double a1 = -1.0 * x[ii];
            const double a2 = -1.0 * hC[i + ii];
            for (k = 0; k < r - ii; ++k) {
                double* m = &EM(A, i + 1 + ii + k, i + 1 + ii);
                *m += (a1 * hC[i + ii + k]) + (a2 * x[ii + k]);
            }
        }
        x[0] = beta;
        hC[i] = h;
    }
}

/* tridiagonalization_inplace_selector<MatrixType,3,false>::run: the closed-form 3x3 tridiagonalisation (lower triangle
 * read); diag/sub out, Q (extractQ) written into A. */
static void rd_tridiagonalize3(double* A, double* diag, double* sub) {
    const int n = 3;
    const double tol = RD_DBL_MIN;
    const double m00 = EM(A, 0, 0), m10 = EM(A, 1, 0), m20 = EM(A, 2, 0), m11 = EM(A, 1, 1), m21 = EM(A, 2, 1), m22 = EM(A, 2, 2);
    const double v1norm2 = m20 * m20;
    diag[0] = m00;
    if (v1norm2 <= tol) {
        diag[1] = m11;
        diag[2] = m22;
        sub[0] = m10;
        sub[1] = m21;
        EM(A, 0, 0) = 1.0; EM(A, 1, 0) = 0.0; EM(A, 2, 0) = 0.0;
        EM(A, 0, 1) = 0.0; EM(A, 1, 1) = 1.0; EM(A, 2, 1) = 0.0;
        EM(A, 0, 2) = 0.0; EM(A, 1, 2) = 0.0; EM(A, 2, 2) = 1.0;
    } else {
        const double beta = sqrt(m10 * m10 + v1norm2);
        const double invBeta = 1.0 / beta;
        const double m01 = m10 * invBeta;
        const double m02 = m20 * invBeta;
        const double q = 2.0 * m01 * m21 + m02 * (m22 - m11);
        diag[1] = m11 + m02 * q;
        diag[2] = m22 - m02 * q;
        sub[0] = beta;
        sub[1] = m21 - m01 * q;
        EM(A, 0, 0) = 1.0; EM(A, 1, 0) = 0.0; EM(A, 2, 0) = 0.0;
        EM(A, 0, 1) = 0.0; EM(A, 1, 1) = m01; EM(A, 2, 1) = m02;
        EM(A, 0, 2) = 0.0; EM(A, 1, 2) = m02; EM(A, 2, 2) = -m01;
    }
}

/* mat = HouseholderSequence(mat, hC).setLength(n-1).setShift(1) evaluated in place (HouseholderSequence.h
 * evalTo, is_same_dense branch) -> Q in mat. */
static void rd_householder_q(int n, double* A, const double* hC) {
    int i, j, k;
    for (j = 0; j < n; ++j) {
        EM(A, j, j) = 1.0;
        for (i = 0; i < j; ++i) EM(A, i, j) = 0.0;
    }
    for (k = n - 2; k >= 0; --k) {
        const int c = n - k - 1; /* corner size */
        const double tau = hC[k];
        const double* e = &EM(A, k + 2, k); /* essential, length c-1 (read before column k is cleared) */
        double ecopy[RD_EIG_MAX];
        for (i = 0; i < c - 1; ++i) ecopy[i] = e[i];
        if (c == 1) {
            EM(A, k + 1, k + 1) *= (1.0 - tau);
        } else if (tau != 0.0) {
            double tmp[RD_EIG_MAX], se[RD_EIG_MAX];
            const int depth = c - 1;
            for (j = 0; j < c; ++j) {
                double l0 = 0.0, l1 = 0.0, cc;
                int p = 0;
                for (; p + 2 <= depth; p += 2) {
                    l0 = EM(A, k + 2 + p, k + 1 + j) * ecopy[p] + l0;
                    l1 = EM(A, k + 2 + p + 1, k + 1 + j) * ecopy[p + 1] + l1;
                }
                cc = l0 + l1;
                for (; p < depth; ++p) cc += EM(A, k + 2 + p, k + 1 + j) * ecopy[p];
                tmp[j] = 0.0 + 1.0 * cc;
            }
            for (j = 0; j < c; ++j) tmp[j] += EM(A, k + 1, k + 1 + j);
            for (j = 0; j < c; ++j) EM(A, k + 1, k + 1 + j) -= tau * tmp[j];
            for (i = 0; i < depth; ++i) se[i] = tau * ecopy[i];
            for (j = 0; j < c; ++j)
                for (i = 0; i < depth; ++i) EM(A, k + 2 + i, k + 1 + j) -= tmp[j] * se[i];
        }
        for (i = k + 1; i < n; ++i) EM(A, i, k) = 0.0;
    }
}

/* numext::hypot -> positive_real_hypot (MathFunctionsImpl.h): p*sqrt(1+(min/p)^2), NOT libm hypot */
static double rd_hypot(double x, double y) {
    double p, qp;
    x = fabs(x); y = fabs(y);
    p = x > y ? x : y;
    if (p == 0.0) return 0.0;
    qp = (y < x ? y : x) / p;
    return p * sqrt(1.0 + qp * qp);
}

static void rd_givens(double p, double q, double* c, double* s) {
    if (q == 0.0) { *c = p < 0.0 ? -1.0 : 1.0; *s = 0.0; }
    else if (p == 0.0) { *c = 0.0; *s = q < 0.0 ? 1.0 : -1.0; }
    else if (fabs(p) > fabs(q)) { double t = q / p, u = sqrt(1.0 + t * t); if (p < 0.0) u = -u; *c = 1.0 / u; *s = -t * (*c); }
    else { double t = p / q, u = sqrt(1.0 + t * t); if (q < 0.0) u = -u; *s = -1.0 / u; *c = -t * (*s); }
}

/* internal::tridiagonal_qr_step + computeFromTridiagonal_impl (SelfAdjointEigenSolver.h). */
static int rd_tridiag_qr(int n, double* diag, double* sub, double* Q) {
    int end = n - 1, start = 0, iter = 0, i, k;
    const double considerAsZero = RD_DBL_MIN, precision_inv = 1.0 / RD_DBL_EPSILON;
    while (end > 0) {
        for (i = start; i < end; ++i) {
            if (fabs(sub[i]) < considerAsZero) {
                sub[i] = 0.0;
            } else {
                const double scaled = precision_inv * sub[i];
                if (scaled * scaled <= (fabs(diag[i]) + fabs(diag[i + 1]))) sub[i] = 0.0;
            }
        }
        while (end > 0 && sub[end - 1] == 0.0) end--;
        if (end <= 0) break;
        iter++;
        if (iter > 30 * n) break; /* m_maxIterations = 30 */
        start = end - 1;
        while (start > 0 && sub[start - 1] != 0.0) start--;
        {
            double td = (diag[end - 1] - diag[end]) * 0.5;
            double e = sub[end - 1];
            double mu = diag[end];
            double x, z;
            if (td == 0.0) {
                mu -= fabs(e);
            } else if (e != 0.0) {
                const double e2 = e * e;
                const double hh = rd_hypot(td, e);
                if (e2 == 0.0) mu -= e / ((td + (td > 0.0 ? hh : -hh)) / e);
                else mu -= e2 / (td + (td > 0.0 ? hh : -hh));
            }
            x = diag[start] - mu;
            z = sub[start];
            for (k = start; k < end && z != 0.0; ++k) {
                double c, s, sdk, dkp1;
                rd_givens(x, z, &c, &s);
                sdk = s * diag[k] + c * sub[k];
                dkp1 = s * sub[k] + c * diag[k + 1];
                diag[k] = c * (c * diag[k] - s * sub[k]) - s * (c * sub[k] - s * diag[k + 1]);
                diag[k + 1] = s * sdk + c * dkp1;
                sub[k] = c * sdk - s * dkp1;
                if (k > start) sub[k - 1] = c * sub[k - 1] - s * z;
                x = sub[k];
                if (k < end - 1) {
                    z = -s * sub[k + 1];
                    sub[k + 1] = c * sub[k + 1];
                }
                /* q.applyOnTheRight(k, k+1, rot): x' = c*x + (-s)*y ; y' = s*x + c*y (apply_rotation_in_the_plane) */
                {
                    double* cx = &EM(Q, 0, k);
                    double* cy = &EM(Q, 0, k + 1);
                    int r;
                    for (r = 0; r < n; ++r) {
                        const double xi = cx[r], yi = cy[r];
                        cx[r] = c * xi + (-s) * yi;
                        cy[r] = (-(-s)) * xi + c * yi;
                    }
                }
            }
        }
    }
    if (iter > 30 * n) return 1;
    for (i = 0; i < n - 1; ++i) { /* selection sort, ascending, first minimum */
        int kmin = 0, q;
        for (q = 1; q < n - i; ++q) if (diag[i + q] < diag[i + kmin]) kmin = q;
        if (kmin > 0) {
            double t = diag[i]; int r;
            diag[i] = diag[kmin + i]; diag[kmin + i] = t;
            for (r = 0; r < n; ++r) { t = EM(Q, r, i); EM(Q, r, i) = EM(Q, r, kmin + i); EM(Q, r, kmin + i) = t; }
        }
    }
    return 0;
}

int rd_selfadjoint_eig(int n, const double* a, double* evals, double* evecs, int hc_parity) {
    double hC[RD_EIG_MAX], sub[RD_EIG_MAX], scale = 0.0;
    double* mat;
    int i, j, info;
    if (n < 1 || n > RD_EIG_MAX) return 2;
    mat = (double*)malloc(sizeof(double) * (size_t)n * (size_t)n);
    if (!mat) return 2;
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) EM(mat, i, j) = (i >= j) ? EM(a, i, j) : 0.0;
    for (i = 0; i < n * n; ++i) if (fabs(mat[i]) > scale) scale = fabs(mat[i]);
    if (scale == 0.0) scale = 1.0;
    for (j = 0; j < n; ++j)
        for (i = j; i < n; ++i) EM(mat, i, j) = EM(mat, i, j) / scale;
    if (n == 3) {
        rd_tridiagonalize3(mat, evals, sub);
    } else if (n == 1) {
        evals[0] = EM(mat, 0, 0);
        EM(mat, 0, 0) = 1.0;
    } else {
        rd_tridiagonalize(n, mat, hC, hc_parity);
        for (i = 0; i < n; ++i) evals[i] = EM(mat, i, i);
        for (i = 0; i < n - 1; ++i) sub[i] = EM(mat, i + 1, i);
        rd_householder_q(n, mat, hC);
    }
    info = rd_tridiag_qr(n, evals, sub, mat);
    for (i = 0; i < n; ++i) evals[i] *= scale;
    memcpy(evecs, mat, sizeof(double) * (size_t)(n * n));
    free(mat);
    return info;
}
