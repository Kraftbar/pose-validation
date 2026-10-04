/* SPDX-License-Identifier: MPL-2.0 */
/* See ok_sparse.h (Eigen 3.4.0 SimplicialLDLT, MPL-2.0, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen
 * authors; LDL algorithm by Timothy A. Davis as adapted in Eigen). */
#include <stdlib.h>
#include <string.h>
#include "ok_sparse.h"

void ok_ldlt_analyze(ok_ldlt* f, int n, const int* Ap, const int* Ai) {
    int k, p;
    int* tags;
    memset(f, 0, sizeof *f);
    f->n = n;
    f->parent = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
    f->nz_per_col = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
    tags = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
    /* analyzePattern_preordered(ap, doLDLT = true) */
    for (k = 0; k < n; ++k) {
        f->parent[k] = -1;
        tags[k] = k;
        f->nz_per_col[k] = 0;
        for (p = Ap[k]; p < Ap[k + 1]; ++p) {
            int i = Ai[p];
            if (i < k) {
                for (; tags[i] != k; i = f->parent[i]) {
                    if (f->parent[i] == -1) f->parent[i] = k;
                    f->nz_per_col[i]++;
                    tags[i] = k;
                }
            }
        }
    }
    free(tags);
    f->Lp = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    f->Lp[0] = 0;
    for (k = 0; k < n; ++k) f->Lp[k + 1] = f->Lp[k] + f->nz_per_col[k];  /* LDLT: no stored diagonal */
    f->nnz_l = f->Lp[n];
    f->Li = (int*)malloc(sizeof(int) * (size_t)(f->nnz_l > 0 ? f->nnz_l : 1));
    f->Lx = (double*)malloc(sizeof(double) * (size_t)(f->nnz_l > 0 ? f->nnz_l : 1));
    f->D = (double*)malloc(sizeof(double) * (size_t)(n ? n : 1));
    f->y = (double*)malloc(sizeof(double) * (size_t)(n ? n : 1));
    f->pattern = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
    f->tags = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
}

int ok_ldlt_factorize(ok_ldlt* f, const int* Ap, const int* Ai, const double* Ax) {
    const int n = f->n;
    const int* Lp = f->Lp;
    int* Li = f->Li;
    double* Lx = f->Lx;
    double* y = f->y;
    int* pattern = f->pattern;
    int* tags = f->tags;
    int k, p, ok = 1;
    for (k = 0; k < n; ++k) {
        int top = n;
        y[k] = 0.0;
        tags[k] = k;
        f->nz_per_col[k] = 0;
        for (p = Ap[k]; p < Ap[k + 1]; ++p) {
            int i = Ai[p];
            if (i <= k) {
                int len;
                y[i] += Ax[p];  /* scatter A(i,k) into Y (sum duplicates) */
                for (len = 0; tags[i] != k; i = f->parent[i]) {
                    pattern[len++] = i;
                    tags[i] = k;
                }
                while (len > 0) pattern[--top] = pattern[--len];
            }
        }
        {
            double d = y[k] * 1.0 + 0.0;  /* m_shiftScale = 1, m_shiftOffset = 0 */
            y[k] = 0.0;
            for (; top < n; ++top) {
                const int i = pattern[top];
                const double yi = y[i];
                double l_ki;
                int p2;
                y[i] = 0.0;
                l_ki = yi / f->D[i];
                p2 = Lp[i] + f->nz_per_col[i];
                for (p = Lp[i]; p < p2; ++p) y[Li[p]] -= Lx[p] * yi;
                d -= l_ki * yi;
                Li[p2] = k;
                Lx[p2] = l_ki;
                ++f->nz_per_col[i];
            }
            f->D[k] = d;
            if (d == 0.0) { ok = 0; break; }
        }
    }
    f->ok = ok;
    return ok;
}

void ok_ldlt_solve(const ok_ldlt* f, const double* b, double* x) {
    const int n = f->n;
    int i, p;
    memcpy(x, b, sizeof(double) * (size_t)n);
    /* matrixL().solveInPlace: UnitLower, ColMajor (no stored diagonal) */
    if (f->nnz_l > 0) {
        for (i = 0; i < n; ++i) {
            const double tmp = x[i];
            if (tmp != 0.0)
                for (p = f->Lp[i]; p < f->Lp[i] + f->nz_per_col[i]; ++p) x[f->Li[p]] -= tmp * f->Lx[p];
        }
    }
    /* dest = m_diag.asDiagonal().inverse() * dest */
    for (i = 0; i < n; ++i) x[i] = (1.0 / f->D[i]) * x[i];
    /* matrixU().solveInPlace: UnitUpper, RowMajor view of L^T */
    if (f->nnz_l > 0) {
        for (i = n - 1; i >= 0; --i) {
            double tmp = x[i];
            for (p = f->Lp[i]; p < f->Lp[i] + f->nz_per_col[i]; ++p) tmp -= f->Lx[p] * x[f->Li[p]];
            x[i] = tmp;
        }
    }
}

void ok_ldlt_free(ok_ldlt* f) {
    free(f->parent); free(f->nz_per_col); free(f->Lp); free(f->Li); free(f->Lx); free(f->D); free(f->y);
    free(f->pattern); free(f->tags);
    memset(f, 0, sizeof *f);
}
