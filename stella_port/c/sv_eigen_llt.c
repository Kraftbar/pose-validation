/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_eigen_llt.h (MPL-2.0, Eigen-derived). */
#include "sv_eigen_llt.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* SimplicialCholeskyBase::factorize_preordered<DoLDLT=false>, specialized
 * to a fully-dense NxN pattern under the identity permutation, reading
 * H's UPPER triangle (row<=col) as the source values -- see header. */
void sv_llt6_factorize(const double H[SV_LLT_MAXN][SV_LLT_MAXN], int n, sv_llt6* out) {
    double y[SV_LLT_MAXN];
    int i, k;

    out->n = n;
    out->ok = 1;

    for (k = 0; k < n; ++k) {
        /* y[0..k] = H(0..k, k), reading the upper-triangle entry
         * H[row][k] for row<=k (fillCCS(upperTriangle=true)'s
         * convention -- NOT H[k][row]). */
        for (i = 0; i <= k; ++i) {
            y[i] = H[i][k];
        }

        double d = y[k];
        y[k] = 0.0;

        for (i = 0; i < k; ++i) {
            double yi = y[i];
            y[i] = 0.0;

            /* DoLDLT=false branch of factorize_preordered: `yi = l_ki =
             * yi / Lx[Lp[i]]` -- yi is REASSIGNED to the divided value
             * before the scatter/diagonal-update use it (unlike the
             * LDLT branch, which keeps the undivided accumulator). Using
             * the undivided yi here (as the LDLT algorithm does) blows
             * up numerically -- this was a real bug, not just a
             * different rounding, caught by sv_llt6_factorize returning
             * ok=0 on real captured data where real Eigen succeeds; see
             * stella_port/HANDOVER.md module-4b "bit-exact closure". */
            double l_ki = yi / out->L[i][i];

            int r;
            for (r = i + 1; r < k; ++r) {
                y[r] -= out->L[r][i] * l_ki;
            }
            d -= l_ki * l_ki;

            out->L[k][i] = l_ki;
        }

        if (d <= 0.0) {
            out->ok = 0;
        }
        out->L[k][k] = sqrt(d);
    }
}

void sv_llt6_solve(const sv_llt6* f, const double b[SV_LLT_MAXN], double x[SV_LLT_MAXN]) {
    int n = f->n;
    double y[SV_LLT_MAXN];
    int i, k;

    /* matrixL().solveInPlace: real (non-unit) lower triangular forward
     * substitution -- divides by L(i,i), unlike the LDLT case. */
    for (i = 0; i < n; ++i) {
        double s = b[i];
        for (k = 0; k < i; ++k) {
            s -= f->L[i][k] * y[k];
        }
        y[i] = s / f->L[i][i];
    }

    /* matrixU().solveInPlace: real (non-unit) upper triangular (L^T) back
     * substitution -- no separate diagonal step (m_diag is empty for
     * DoLDLT=false, see SimplicialCholesky.h's _solve_impl). */
    for (i = n - 1; i >= 0; --i) {
        double s = y[i];
        for (k = i + 1; k < n; ++k) {
            s -= f->L[k][i] * x[k];
        }
        x[i] = s / f->L[i][i];
    }
}

/* ------------------------------------------------------------------------
 * General sparse SimplicialLLT (see header)
 * ------------------------------------------------------------------------ */

/* permute_symm_to_symm<Upper,Upper>(mat, dest, perm), ColMajor->ColMajor. */
static void permute_upper(int n, const int* a_p, const int* a_i, const double* a_x, const int* perm,
                          int* d_p, int* d_i, double* d_x) {
    int j, p;
    int* count = (int*)calloc((size_t)n, sizeof(int));
    for (j = 0; j < n; ++j) {
        const int jp = perm[j];
        for (p = a_p[j]; p < a_p[j + 1]; ++p) {
            const int i = a_i[p];
            if (i > j) {
                continue;
            }
            const int ip = perm[i];
            count[ip > jp ? ip : jp]++;
        }
    }
    d_p[0] = 0;
    for (j = 0; j < n; ++j) {
        d_p[j + 1] = d_p[j] + count[j];
    }
    for (j = 0; j < n; ++j) {
        count[j] = d_p[j];
    }
    for (j = 0; j < n; ++j) {
        const int jp = perm[j];
        for (p = a_p[j]; p < a_p[j + 1]; ++p) {
            const int i = a_i[p];
            if (i > j) {
                continue;
            }
            const int ip = perm[i];
            const int hi = ip > jp ? ip : jp;
            const int lo = ip < jp ? ip : jp;
            const int k = count[hi]++;
            d_i[k] = lo;
            if (d_x) {
                d_x[k] = a_x[p]; /* real scalar: conj is a no-op */
            }
        }
    }
    free(count);
}

void sv_sllt_analyze(sv_sllt* f, int n, const int* a_p, const int* a_i, const int* scalar_perm) {
    int k, p;
    memset(f, 0, sizeof(*f));
    f->n = n;
    f->nnz_a = a_p[n];
    f->Pinv = (int*)malloc(sizeof(int) * (size_t)n);
    f->P = (int*)malloc(sizeof(int) * (size_t)n);
    for (k = 0; k < n; ++k) {
        f->Pinv[k] = scalar_perm[k];
    }
    for (k = 0; k < n; ++k) {
        f->P[scalar_perm[k]] = k; /* m_P = permutation.inverse() */
    }
    f->ap_p = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    f->ap_i = (int*)malloc(sizeof(int) * (size_t)(f->nnz_a > 0 ? f->nnz_a : 1));
    f->ap_x = (double*)malloc(sizeof(double) * (size_t)(f->nnz_a > 0 ? f->nnz_a : 1));
    permute_upper(n, a_p, a_i, NULL, f->P, f->ap_p, f->ap_i, NULL);

    /* analyzePattern_preordered(ap, doLDLT=false) */
    f->parent = (int*)malloc(sizeof(int) * (size_t)n);
    f->nz_per_col = (int*)malloc(sizeof(int) * (size_t)n);
    int* tags = (int*)malloc(sizeof(int) * (size_t)n);
    for (k = 0; k < n; ++k) {
        f->parent[k] = -1; /* parent of k is not yet known */
        tags[k] = k;       /* mark node k as visited */
        f->nz_per_col[k] = 0;
        for (p = f->ap_p[k]; p < f->ap_p[k + 1]; ++p) {
            int i = f->ap_i[p];
            if (i < k) {
                /* follow path from i to root of etree, stop at flagged node */
                for (; tags[i] != k; i = f->parent[i]) {
                    if (f->parent[i] == -1) {
                        f->parent[i] = k;
                    }
                    f->nz_per_col[i]++; /* L (k,i) is nonzero */
                    tags[i] = k;        /* mark i as visited */
                }
            }
        }
    }
    free(tags);
    f->Lp = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    f->Lp[0] = 0;
    for (k = 0; k < n; ++k) {
        f->Lp[k + 1] = f->Lp[k] + f->nz_per_col[k] + 1; /* LLT: +1 for the diagonal */
    }
    f->nnz_l = f->Lp[n];
    f->Li = (int*)malloc(sizeof(int) * (size_t)(f->nnz_l > 0 ? f->nnz_l : 1));
    f->Lx = (double*)malloc(sizeof(double) * (size_t)(f->nnz_l > 0 ? f->nnz_l : 1));
    f->y = (double*)malloc(sizeof(double) * (size_t)n);
    f->pattern = (int*)malloc(sizeof(int) * (size_t)n);
    f->tags = (int*)malloc(sizeof(int) * (size_t)n);
    f->work = (double*)malloc(sizeof(double) * (size_t)n);
}

int sv_sllt_factorize(sv_sllt* f, const double* a_x, const int* a_p, const int* a_i) {
    const int n = f->n;
    int k, p;
    /* tmp.selfadjointView<Upper>() = a.selfadjointView<Upper>().twistedBy(m_P) */
    permute_upper(n, a_p, a_i, a_x, f->P, f->ap_p, f->ap_i, f->ap_x);

    const int* Lp = f->Lp;
    int* Li = f->Li;
    double* Lx = f->Lx;
    double* y = f->y;
    int* pattern = f->pattern;
    int* tags = f->tags;
    int ok = 1;

    for (k = 0; k < n; ++k) {
        /* compute nonzero pattern of kth row of L, in topological order */
        y[k] = 0.0;
        int top = n; /* stack for pattern is empty */
        tags[k] = k; /* mark node k as visited */
        f->nz_per_col[k] = 0;
        for (p = f->ap_p[k]; p < f->ap_p[k + 1]; ++p) {
            int i = f->ap_i[p];
            if (i <= k) {
                y[i] += f->ap_x[p]; /* scatter A(i,k) into Y (sum duplicates) */
                int len;
                for (len = 0; tags[i] != k; i = f->parent[i]) {
                    pattern[len++] = i; /* L(k,i) is nonzero */
                    tags[i] = k;        /* mark i as visited */
                }
                while (len > 0) {
                    pattern[--top] = pattern[--len];
                }
            }
        }

        /* compute numerical values kth row of L (a sparse triangular solve);
         * m_shiftScale == 1, m_shiftOffset == 0 in SimplicialLLT. */
        double d = y[k] * 1.0 + 0.0;
        y[k] = 0.0;
        for (; top < n; ++top) {
            const int i = pattern[top]; /* pattern[top:n-1] is pattern of L(:,k) */
            double yi = y[i];           /* get and clear Y(i) */
            y[i] = 0.0;

            /* the nonzero entry L(k,i); DoLDLT=false: yi is REASSIGNED */
            double l_ki;
            yi = l_ki = yi / Lx[Lp[i]];

            const int p2 = Lp[i] + f->nz_per_col[i];
            for (p = Lp[i] + 1; p < p2; ++p) {
                y[Li[p]] -= Lx[p] * yi;
            }
            d -= l_ki * yi;
            Li[p] = k; /* store L(k,i) in column form of L */
            Lx[p] = l_ki;
            ++f->nz_per_col[i]; /* increment count of nonzeros in col i */
        }
        p = Lp[k] + f->nz_per_col[k]++;
        Li[p] = k; /* store L(k,k) = sqrt (d) in column k */
        if (d <= 0.0) {
            ok = 0; /* failure, matrix is not positive definite */
            break;
        }
        Lx[p] = sqrt(d);
    }
    f->ok = ok;
    return ok;
}

void sv_sllt_solve(const sv_sllt* f, const double* b, double* x) {
    const int n = f->n;
    double* dest = f->work;
    int i, p;
    for (i = 0; i < n; ++i) {
        dest[f->P[i]] = b[i]; /* dest = m_P * b */
    }
    /* matrixL().solveInPlace(dest): Lower, ColMajor */
    for (i = 0; i < n; ++i) {
        double* tmp = &dest[i];
        if (*tmp != 0.0) {
            p = f->Lp[i]; /* diagonal first (index == i) */
            *tmp /= f->Lx[p];
            ++p;
            for (; p < f->Lp[i] + f->nz_per_col[i]; ++p) {
                dest[f->Li[p]] -= *tmp * f->Lx[p];
            }
        }
    }
    /* matrixU().solveInPlace(dest): U = L^T, RowMajor Upper */
    for (i = n - 1; i >= 0; --i) {
        double tmp = dest[i];
        p = f->Lp[i];
        const double l_ii = f->Lx[p];
        ++p;
        for (; p < f->Lp[i] + f->nz_per_col[i]; ++p) {
            tmp -= f->Lx[p] * dest[f->Li[p]];
        }
        dest[i] = tmp / l_ii;
    }
    for (i = 0; i < n; ++i) {
        x[f->Pinv[i]] = dest[i]; /* dest = m_Pinv * dest */
    }
}

void sv_sllt_free(sv_sllt* f) {
    free(f->P);
    free(f->Pinv);
    free(f->ap_p);
    free(f->ap_i);
    free(f->ap_x);
    free(f->parent);
    free(f->nz_per_col);
    free(f->Lp);
    free(f->Li);
    free(f->Lx);
    free(f->y);
    free(f->pattern);
    free(f->tags);
    free(f->work);
    memset(f, 0, sizeof(*f));
}
