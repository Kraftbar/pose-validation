/* SPDX-License-Identifier: BSD-3-Clause */
/* See ok_blas.h (Ceres Solver 2.2.0 small_blas.h / small_blas_generic.h, BSD-3-Clause, Copyright 2023 Google Inc.). */
#include "ok_blas.h"

#define STORE1(p, idx, v)                 \
    do {                                  \
        if (op > 0) (p)[idx] += (v);      \
        else if (op < 0) (p)[idx] -= (v); \
        else (p)[idx] = (v);              \
    } while (0)
#define STORE2(p, idx, v1, v2)                                   \
    do {                                                         \
        if (op > 0) { (p)[idx] += (v1); (p)[(idx) + 1] += (v2); } \
        else if (op < 0) { (p)[idx] -= (v1); (p)[(idx) + 1] -= (v2); } \
        else { (p)[idx] = (v1); (p)[(idx) + 1] = (v2); }          \
    } while (0)
#define STORE4(c, v)                                                                   \
    do {                                                                               \
        if (op > 0) { c[0] += v[0]; c[1] += v[1]; c[2] += v[2]; c[3] += v[3]; }        \
        else if (op < 0) { c[0] -= v[0]; c[1] -= v[1]; c[2] -= v[2]; c[3] -= v[3]; }   \
        else { c[0] = v[0]; c[1] = v[1]; c[2] = v[2]; c[3] = v[3]; }                   \
    } while (0)

/* MMM_mat1x4: one row of A (col_a entries) times four columns of B (row stride col_stride_b) */
static void mmm_mat1x4(int col_a, const double* a, const double* b, int col_stride_b, double* c, int op) {
    double cvec4[4] = {0.0, 0.0, 0.0, 0.0};
    int k, bi = 0;
    for (k = 0; k < col_a; ++k) {
        const double av = a[k];
        const double* pb = b + bi;
        cvec4[0] += av * pb[0];
        cvec4[1] += av * pb[1];
        cvec4[2] += av * pb[2];
        cvec4[3] += av * pb[3];
        bi += col_stride_b;
    }
    STORE4(c, cvec4);
}
/* MTM_mat1x4: one column of A (stride col_stride_a) times four columns of B */
static void mtm_mat1x4(int col_a, const double* a, int col_stride_a, const double* b, int col_stride_b, double* c,
                       int op) {
    double cvec4[4] = {0.0, 0.0, 0.0, 0.0};
    int k, ai = 0, bi = 0;
    for (k = 0; k < col_a; ++k) {
        const double av = a[ai];
        const double* pb = b + bi;
        cvec4[0] += av * pb[0];
        cvec4[1] += av * pb[1];
        cvec4[2] += av * pb[2];
        cvec4[3] += av * pb[3];
        ai += col_stride_a;
        bi += col_stride_b;
    }
    STORE4(c, cvec4);
}
/* MVM_mat4x1: four rows of A times the vector b */
static void mvm_mat4x1(int col_a, const double* a, int col_stride_a, const double* b, double* c, int op) {
    double cvec4[4] = {0.0, 0.0, 0.0, 0.0};
    int k;
    for (k = 0; k < col_a; ++k) {
        const double bv = b[k];
        cvec4[0] += a[k] * bv;
        cvec4[1] += a[k + col_stride_a] * bv;
        cvec4[2] += a[k + col_stride_a * 2] * bv;
        cvec4[3] += a[k + col_stride_a * 3] * bv;
    }
    STORE4(c, cvec4);
}
/* MTV_mat4x1: four columns of A (row stride col_stride_a) times the vector b */
static void mtv_mat4x1(int col_a, const double* a, int col_stride_a, const double* b, double* c, int op) {
    double cvec4[4] = {0.0, 0.0, 0.0, 0.0};
    int k;
    const double* pa = a;
    for (k = 0; k < col_a; ++k) {
        const double bv = b[k];
        cvec4[0] += pa[0] * bv;
        cvec4[1] += pa[1] * bv;
        cvec4[2] += pa[2] * bv;
        cvec4[3] += pa[3] * bv;
        pa += col_stride_a;
    }
    STORE4(c, cvec4);
}

void ok_mmm(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int start_row_c,
            int start_col_c, int col_stride_c, int op) {
    const int NUM_ROW_C = ra, NUM_COL_C = cb, NUM_COL_A = ca, NUM_COL_B = cb;
    int row, col, k, col_m;
    (void)rb;
    if (NUM_COL_C & 1) {
        col = NUM_COL_C - 1;
        for (row = 0; row < NUM_ROW_C; ++row) {
            const double* pa = A + row * NUM_COL_A;
            const double* pb = B + col;
            double tmp = 0.0;
            for (k = 0; k < NUM_COL_A; ++k, pb += NUM_COL_B) tmp += pa[k] * pb[0];
            STORE1(C, (row + start_row_c) * col_stride_c + start_col_c + col, tmp);
        }
        if (NUM_COL_C == 1) return;
    }
    if (NUM_COL_C & 2) {
        col = NUM_COL_C & ~3;
        for (row = 0; row < NUM_ROW_C; ++row) {
            const double* pa = A + row * NUM_COL_A;
            const double* pb = B + col;
            double tmp1 = 0.0, tmp2 = 0.0;
            for (k = 0; k < NUM_COL_A; ++k, pb += NUM_COL_B) {
                const double av = pa[k];
                tmp1 += av * pb[0];
                tmp2 += av * pb[1];
            }
            STORE2(C, (row + start_row_c) * col_stride_c + start_col_c + col, tmp1, tmp2);
        }
        if (NUM_COL_C < 4) return;
    }
    col_m = NUM_COL_C & ~3;
    for (col = 0; col < col_m; col += 4)
        for (row = 0; row < NUM_ROW_C; ++row)
            mmm_mat1x4(NUM_COL_A, A + row * NUM_COL_A, B + col, NUM_COL_B,
                       C + (row + start_row_c) * col_stride_c + start_col_c + col, op);
}

void ok_mtm(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int start_row_c,
            int start_col_c, int col_stride_c, int op) {
    const int NUM_ROW_C = ca, NUM_COL_C = cb, NUM_ROW_A = ra, NUM_COL_A = ca, NUM_COL_B = cb;
    int row, col, k, col_m;
    (void)rb;
    if (NUM_COL_C & 1) {
        col = NUM_COL_C - 1;
        for (row = 0; row < NUM_ROW_C; ++row) {
            const double* pa = A + row;
            const double* pb = B + col;
            double tmp = 0.0;
            for (k = 0; k < NUM_ROW_A; ++k) {
                tmp += pa[0] * pb[0];
                pa += NUM_COL_A;
                pb += NUM_COL_B;
            }
            STORE1(C, (row + start_row_c) * col_stride_c + start_col_c + col, tmp);
        }
        if (NUM_COL_C == 1) return;
    }
    if (NUM_COL_C & 2) {
        col = NUM_COL_C & ~3;
        for (row = 0; row < NUM_ROW_C; ++row) {
            const double* pa = A + row;
            const double* pb = B + col;
            double tmp1 = 0.0, tmp2 = 0.0;
            for (k = 0; k < NUM_ROW_A; ++k) {
                const double av = *pa;
                tmp1 += av * pb[0];
                tmp2 += av * pb[1];
                pa += NUM_COL_A;
                pb += NUM_COL_B;
            }
            STORE2(C, (row + start_row_c) * col_stride_c + start_col_c + col, tmp1, tmp2);
        }
        if (NUM_COL_C < 4) return;
    }
    col_m = NUM_COL_C & ~3;
    for (col = 0; col < col_m; col += 4)
        for (row = 0; row < NUM_ROW_C; ++row)
            mtm_mat1x4(NUM_ROW_A, A + row, NUM_COL_A, B + col, NUM_COL_B,
                       C + (row + start_row_c) * col_stride_c + start_col_c + col, op);
}

void ok_mv(const double* A, int ra, int ca, const double* b, double* c, int op) {
    const int NUM_ROW_A = ra, NUM_COL_A = ca;
    int row, col, row_m;
    if (NUM_ROW_A & 1) {
        row = NUM_ROW_A - 1;
        {
            const double* pa = A + row * NUM_COL_A;
            double tmp = 0.0;
            for (col = 0; col < NUM_COL_A; ++col) tmp += pa[col] * b[col];
            STORE1(c, row, tmp);
        }
        if (NUM_ROW_A == 1) return;
    }
    if (NUM_ROW_A & 2) {
        row = NUM_ROW_A & ~3;
        {
            const double* pa1 = A + row * NUM_COL_A;
            const double* pa2 = pa1 + NUM_COL_A;
            double tmp1 = 0.0, tmp2 = 0.0;
            for (col = 0; col < NUM_COL_A; ++col) {
                const double bv = b[col];
                tmp1 += pa1[col] * bv;
                tmp2 += pa2[col] * bv;
            }
            STORE2(c, row, tmp1, tmp2);
        }
        if (NUM_ROW_A < 4) return;
    }
    row_m = NUM_ROW_A & ~3;
    for (row = 0; row < row_m; row += 4) mvm_mat4x1(NUM_COL_A, A + row * NUM_COL_A, NUM_COL_A, b, c + row, op);
}

void ok_mtv(const double* A, int ra, int ca, const double* b, double* c, int op) {
    const int NUM_ROW_A = ra, NUM_COL_A = ca;
    int row, col, row_m;
    if (NUM_COL_A & 1) {
        row = NUM_COL_A - 1;
        {
            const double* pa = A + row;
            double tmp = 0.0;
            for (col = 0; col < NUM_ROW_A; ++col) {
                tmp += *pa * b[col];
                pa += NUM_COL_A;
            }
            STORE1(c, row, tmp);
        }
        if (NUM_COL_A == 1) return;
    }
    if (NUM_COL_A & 2) {
        row = NUM_COL_A & ~3;
        {
            const double* pa = A + row;
            double tmp1 = 0.0, tmp2 = 0.0;
            for (col = 0; col < NUM_ROW_A; ++col) {
                const double bv = b[col];
                tmp1 += pa[0] * bv;
                tmp2 += pa[1] * bv;
                pa += NUM_COL_A;
            }
            STORE2(c, row, tmp1, tmp2);
        }
        if (NUM_COL_A < 4) return;
    }
    row_m = NUM_COL_A & ~3;
    for (row = 0; row < row_m; row += 4) mtv_mat4x1(NUM_ROW_A, A + row, NUM_COL_A, b, c + row, op);
}
