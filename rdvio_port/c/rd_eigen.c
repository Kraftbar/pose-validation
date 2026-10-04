/* SPDX-License-Identifier: MPL-2.0 */
/* See rd_eigen.h. */
#include "rd_eigen.h"
#include "../../okvis_port/c/ok_dense.h"
#include <math.h>
#include <string.h>

#define NMAX 16

/* triangular_solve_matrix<double,Index,OnTheLeft,Mode,false,ColMajor,ColMajor>::run for a fixed-size problem whose gemm_blocking_space
 * is the static one (kc = size, mc = size): one k2 chunk, SmallPanelWidth = max(mr, nr) = 4, a single column chunk (subcols >= cols). */
static void trsm_left(int size, int cols, const double* tri, int ld, double* oth, int lower, int unit) {
    const int kc = size;
    const int SPW = 4;
    int k1, k, j, i3;
    const int k2 = lower ? 0 : size;
    const int actual_kc = lower ? (size - k2 < kc ? size - k2 : kc) : (k2 < kc ? k2 : kc);
#define T(i, j) tri[(i) + (long)ld * (j)]
#define O(i, j) oth[(i) + (long)ld * (j)]
    for (k1 = 0; k1 < actual_kc; k1 += SPW) {
        const int apw = (actual_kc - k1 < SPW) ? actual_kc - k1 : SPW;
        for (k = 0; k < apw; ++k) {
            const int i = lower ? k2 + k1 + k : k2 - k1 - k - 1;
            const int rs = apw - k - 1;
            const int s = lower ? i + 1 : i - rs;
            const double a = unit ? 1.0 : 1.0 / T(i, i);
            for (j = 0; j < cols; ++j) {
                O(i, j) = O(i, j) * a;
                {
                    const double b = O(i, j);
                    for (i3 = 0; i3 < rs; ++i3) O(s + i3, j) = O(s + i3, j) - b * T(s + i3, i);
                }
            }
        }
        {
            const int lengthTarget = actual_kc - k1 - apw;
            const int startBlock = lower ? k2 + k1 : k2 - k1 - apw;
            if (lengthTarget > 0) {
                const int startTarget = lower ? k2 + k1 + apw : k2 - actual_kc;
                /* gebp: other(startTarget.., :) += -1 * tri(startTarget.., startBlock..) * other(startBlock.., :) */
                ok_gebp(lengthTarget, cols, apw, &T(startTarget, startBlock), 1, ld, &O(startBlock, 0), 1, ld, -1.0,
                        &O(startTarget, 0), 1, ld);
            }
        }
    }
#undef T
#undef O
}

int rd_inverse_ppl(int n, const double* a, double* inv) {
    double lu[NMAX * NMAX];
    int trans[NMAX];
    int first_zero = -1, k, i, j;
    memcpy(lu, a, sizeof(double) * n * n);
#define L(i, j) lu[(i) + n * (j)]
    for (k = 0; k < n - 1; ++k) {
        int rb = k;
        double big = fabs(L(k, k));
        for (i = k + 1; i < n; ++i)
            if (fabs(L(i, k)) > big) { big = fabs(L(i, k)); rb = i; }
        trans[k] = rb;
        if (big != 0.0) {
            if (rb != k)
                for (j = 0; j < n; ++j) { double t = L(k, j); L(k, j) = L(rb, j); L(rb, j) = t; }
            for (i = k + 1; i < n; ++i) L(i, k) = L(i, k) / L(k, k);
        } else if (first_zero == -1) {
            first_zero = k;
        }
        for (j = k + 1; j < n; ++j)
            for (i = k + 1; i < n; ++i) L(i, j) = L(i, j) - L(k, j) * L(i, k);
    }
    trans[n - 1] = n - 1;
    if (L(n - 1, n - 1) == 0.0 && first_zero == -1) first_zero = n - 1;
    /* dst = P * Identity: apply the row transpositions in order */
    memset(inv, 0, sizeof(double) * n * n);
    for (i = 0; i < n; ++i) inv[i + n * i] = 1.0;
    for (k = 0; k < n; ++k) {
        if (trans[k] != k)
            for (j = 0; j < n; ++j) { double t = inv[k + n * j]; inv[k + n * j] = inv[trans[k] + n * j]; inv[trans[k] + n * j] = t; }
    }
    trsm_left(n, n, lu, n, inv, 1, 1);  /* UnitLower */
    trsm_left(n, n, lu, n, inv, 0, 0);  /* Upper */
#undef L
    return first_zero;
}

int rd_llt_sqrt_info(int n, const double* info, double* out) {
    double m[NMAX * NMAX];
    int i, j, r;
    memcpy(m, info, sizeof(double) * n * n);
    r = ok_llt_lower(n, m, n);
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) out[i + n * j] = (i <= j) ? m[j + n * i] : 0.0;
    return r;
}
