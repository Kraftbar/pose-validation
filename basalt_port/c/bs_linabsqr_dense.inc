/* Dense Eigen 3.4.0 float kernels (SSE2, packet 4, no FMA) as used by the ABS_QR linearisation: dynamic GEMM (GEBP incl. the blocking
 * heuristic with the cache sizes of the reference machine) and GEMV (column-major and row-major kernel). Included by bs_linabsqr.c. */

#ifdef __SSE2__
#include <emmintrin.h>
#endif

/* Eigen queryCacheSizes() on the reference machine (AMD Ryzen 7 3700X: l1 32 KiB, l2 512 KiB, l3 32 MiB). The GEMM blocking of products with
 * max(m, n, k) >= 48 depends on them; a machine with other cache sizes gives other row/column blocks and in rare shapes other bits. */
#ifndef BS_L1
#define BS_L1 32768
#define BS_L2 524288
#define BS_L3 33554432
#endif

/* evaluateProductBlockingSizesHeuristic<float,float,1>(k, m, n, num_threads = 1): in/out sizes are kc, mc, nc */
static void la_blocking(int* pk, int* pm, int* pn) {
    long k = *pk, m = *pm, n = *pn;
    const long mr = 8, nr = 4, sz = 4, l1 = BS_L1, l2 = BS_L2, l3 = BS_L3;
    if ((k > m ? (k > n ? k : n) : (m > n ? m : n)) < 48) return;
    {
        const long k_peeling = 8, k_div = (mr * sz + nr * sz), k_sub = mr * nr * sz;
        long max_kc = ((l1 - k_sub) / k_div) & (~(k_peeling - 1));
        long old_k = k, actual_l2 = 1572864, max_nc, nc, lhs_bytes, remaining_l1;
        if (max_kc < 1) max_kc = 1;
        if (k > max_kc) k = (k % max_kc) == 0 ? max_kc : max_kc - k_peeling * ((max_kc - 1 - (k % max_kc)) / (k_peeling * (k / max_kc + 1)));
        lhs_bytes = m * k * sz;
        remaining_l1 = l1 - k_sub - lhs_bytes;
        if (remaining_l1 >= nr * sz * k) max_nc = remaining_l1 / (k * sz);
        else max_nc = (3 * actual_l2) / (2 * 2 * max_kc * sz);
        nc = (actual_l2 / (2 * k * sz) < max_nc ? actual_l2 / (2 * k * sz) : max_nc) & (~(nr - 1));
        if (n > nc) {
            n = (n % nc) == 0 ? nc : (nc - nr * ((nc - (n % nc)) / (nr * (n / nc + 1))));
        } else if (old_k == k) {
            long problem_size = k * n * sz, actual_lm = actual_l2, max_mc = m, mc;
            if (problem_size <= 1024) {
                actual_lm = l1;
            } else if (l3 != 0 && problem_size <= 32768) {
                actual_lm = l2;
                max_mc = max_mc < 576 ? max_mc : 576;
            }
            mc = actual_lm / (3 * k * sz) < max_mc ? actual_lm / (3 * k * sz) : max_mc;
            if (mc > mr) mc -= mc % mr;
            else if (mc == 0) { *pk = (int)k; *pm = (int)m; *pn = (int)n; return; }
            m = (m % mc) == 0 ? mc : (mc - mr * ((mc - (m % mc)) / (mr * (m / mc + 1))));
        }
    }
    *pk = (int)k; *pm = (int)m; *pn = (int)n;
}

/* one GEBP call on a block: rows x depth x cols, C(i,j) += acc(i,j). Eigen 3.4.0 float, mr = 8, nr = 4, LhsProgress = 4 (no half packets):
 *   rows [0, 8*(rows/8))         2-packet kernel: one left fold over k;  leftover columns (j >= 4*(cols/4)) too
 *   next rows in steps of 4      1-packet kernel, 4-column panels: even k -> C, odd k -> D for k < (depth & ~7), acc = C + D, then the
 *                                leftover k sequentially into acc;  leftover columns: one left fold
 *   remaining rows, 1 at a time  swapped path in 4-column panels: 4 accumulators over k & 3 for k < (depth & ~3), acc = (C0+C1)+(C2+C3), then the
 *                                leftover k sequentially;  leftover columns: one left fold */
/* Same per-element evaluation order as the one-element-at-a-time model; speed only comes from computing several independent elements per k
 * sweep (4x2 / 4x1 tiles of independent accumulators, no reassociation inside any element). */
#define LA_A(r) A[(long)(i + (r)) * ars + (long)k * acs]
#define LA_B(c) B[(long)k * brs + (long)(j + (c)) * bcs]
#define LA_CUP(r, c, v) C[(i + (r)) + (long)(j + (c)) * ldc] = C[(i + (r)) + (long)(j + (c)) * ldc] + (v) * 1.0f   /* res + alpha * acc */

/* Zero-skipping.  If every entry of A and B is finite, an A row i (or B column j) that is exactly zero over the whole depth gives acc = +0 for all
 * its elements (0 * finite = +-0 and +0 + +-0 = +0 in every fold order), and C + 0 = C unless C is -0 (never the case for the zero-initialised
 * accumulators of the callers; the test hook bs_la_t_gemm turns the skipping off).  Tiles whose rows or columns are all zero are skipped.
 * la_ra / la_ca (per block-relative row of A / column of B, NULL = no skipping) are set by la_gemm. */
static const unsigned char* la_ra;
static const unsigned char* la_ca;
static int la_skip_enable = 1;
static int la_anyf(const unsigned char* f, int i, int n) { int t = 0; while (n--) t |= f[i++]; return t; }
#define LA_SKIP(i, ni, j, nj) (la_ra && (!la_anyf(la_ra, (i), (ni)) || !la_anyf(la_ca, (j), (nj))))

/* left fold over k (2-packet kernel rows and leftover columns) on rows [i0,i1) x cols [j0,j1) */
static void la_gebp_fold_s(const float* A, long ars, long acs, const float* B, long brs, long bcs, float* C, long ldc, int i0, int i1, int j0, int j1, int depth) {
    int i, j, k;
    for (j = j0; j + 2 <= j1; j += 2) {
        for (i = i0; i + 4 <= i1; i += 4) {
            float c00 = 0.0f, c10 = 0.0f, c20 = 0.0f, c30 = 0.0f, c01 = 0.0f, c11 = 0.0f, c21 = 0.0f, c31 = 0.0f;
            if (LA_SKIP(i, 4, j, 2)) continue;
            for (k = 0; k < depth; ++k) {
                const float a0 = LA_A(0), a1 = LA_A(1), a2 = LA_A(2), a3 = LA_A(3), b0 = LA_B(0), b1 = LA_B(1);
                c00 = c00 + a0 * b0; c10 = c10 + a1 * b0; c20 = c20 + a2 * b0; c30 = c30 + a3 * b0;
                c01 = c01 + a0 * b1; c11 = c11 + a1 * b1; c21 = c21 + a2 * b1; c31 = c31 + a3 * b1;
            }
            LA_CUP(0, 0, c00); LA_CUP(1, 0, c10); LA_CUP(2, 0, c20); LA_CUP(3, 0, c30);
            LA_CUP(0, 1, c01); LA_CUP(1, 1, c11); LA_CUP(2, 1, c21); LA_CUP(3, 1, c31);
        }
        for (; i < i1; ++i) {
            float c0 = 0.0f, c1 = 0.0f;
            if (LA_SKIP(i, 1, j, 2)) continue;
            for (k = 0; k < depth; ++k) { const float a0 = LA_A(0); c0 = c0 + a0 * LA_B(0); c1 = c1 + a0 * LA_B(1); }
            LA_CUP(0, 0, c0); LA_CUP(0, 1, c1);
        }
    }
    for (; j < j1; ++j) {
        for (i = i0; i + 4 <= i1; i += 4) {
            float c0 = 0.0f, c1 = 0.0f, c2 = 0.0f, c3 = 0.0f;
            if (LA_SKIP(i, 4, j, 1)) continue;
            for (k = 0; k < depth; ++k) {
                const float b0 = LA_B(0);
                c0 = c0 + LA_A(0) * b0; c1 = c1 + LA_A(1) * b0; c2 = c2 + LA_A(2) * b0; c3 = c3 + LA_A(3) * b0;
            }
            LA_CUP(0, 0, c0); LA_CUP(1, 0, c1); LA_CUP(2, 0, c2); LA_CUP(3, 0, c3);
        }
        for (; i < i1; ++i) {
            float c0 = 0.0f;
            if (LA_SKIP(i, 1, j, 1)) continue;
            for (k = 0; k < depth; ++k) c0 = c0 + LA_A(0) * LA_B(0);
            LA_CUP(0, 0, c0);
        }
    }
}

static void la_gebp_fold(const float* A, long ars, long acs, const float* B, long brs, long bcs, float* C, long ldc, int i0, int i1, int j0, int j1, int depth) {
    int i, j, k;
#ifdef __SSE2__
    if (ars == 1) {   /* lanes = 4 consecutive rows; _mm_mul_ps / _mm_add_ps are the plain IEEE single ops (never fused), each lane keeps its own left fold */
        for (j = j0; j + 4 <= j1; j += 4) {
            for (i = i0; i + 8 <= i1; i += 8) {
                __m128 l0 = _mm_setzero_ps(), h0 = l0, l1 = l0, h1 = l0, l2 = l0, h2 = l0, l3 = l0, h3 = l0;
                if (LA_SKIP(i, 8, j, 4)) continue;
                for (k = 0; k < depth; ++k) {
                    const float* ap = A + i + (long)k * acs;
                    const float* bp = B + (long)k * brs + (long)j * bcs;
                    const __m128 al = _mm_loadu_ps(ap), ah = _mm_loadu_ps(ap + 4);
                    __m128 b = _mm_set1_ps(bp[0]);
                    l0 = _mm_add_ps(l0, _mm_mul_ps(al, b)); h0 = _mm_add_ps(h0, _mm_mul_ps(ah, b));
                    b = _mm_set1_ps(bp[bcs]);
                    l1 = _mm_add_ps(l1, _mm_mul_ps(al, b)); h1 = _mm_add_ps(h1, _mm_mul_ps(ah, b));
                    b = _mm_set1_ps(bp[2 * bcs]);
                    l2 = _mm_add_ps(l2, _mm_mul_ps(al, b)); h2 = _mm_add_ps(h2, _mm_mul_ps(ah, b));
                    b = _mm_set1_ps(bp[3 * bcs]);
                    l3 = _mm_add_ps(l3, _mm_mul_ps(al, b)); h3 = _mm_add_ps(h3, _mm_mul_ps(ah, b));
                }
                {
                    float* cp = C + i + (long)j * ldc;   /* res + alpha * acc, alpha = 1 (x * 1.0f == x) */
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l0)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h0)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l1)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h1)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l2)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h2)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l3)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h3));
                }
            }
            /* leftover rows of this 4-column panel */
            if (i < i1) la_gebp_fold_s(A, ars, acs, B, brs, bcs, C, ldc, i, i1, j, j + 4, depth);
        }
        if (j < j1) la_gebp_fold_s(A, ars, acs, B, brs, bcs, C, ldc, i0, i1, j, j1, depth);
        return;
    }
#endif
    la_gebp_fold_s(A, ars, acs, B, brs, bcs, C, ldc, i0, i1, j0, j1, depth);
}

#ifdef __SSE2__
/* Left-fold square [0,n8)^2 of C += A^T-type products where B(k,j) == A(j,k) (A == B with swapped strides, ars == 1): element (j,i) is the same
 * sequence of products as (i,j) (a*b == b*a bitwise), so it is evaluated once for i >= j and added to both C(i,j) and C(j,i). */
static void la_gebp_sym(const float* A, long acs, float* C, long ldc, int n8, int depth) {
    int i, j, k, c, r;
    for (j = 0; j < n8; j += 4)
        for (i = j & ~7; i < n8; i += 8) {
            __m128 l0 = _mm_setzero_ps(), h0 = l0, l1 = l0, h1 = l0, l2 = l0, h2 = l0, l3 = l0, h3 = l0;
            if (LA_SKIP(i, 8, j, 4)) continue;
            for (k = 0; k < depth; ++k) {
                const float* ap = A + i + (long)k * acs;
                const float* bp = A + j + (long)k * acs;   /* B(k,j+c) = A(j+c,k) */
                const __m128 al = _mm_loadu_ps(ap), ah = _mm_loadu_ps(ap + 4);
                __m128 b = _mm_set1_ps(bp[0]);
                l0 = _mm_add_ps(l0, _mm_mul_ps(al, b)); h0 = _mm_add_ps(h0, _mm_mul_ps(ah, b));
                b = _mm_set1_ps(bp[1]);
                l1 = _mm_add_ps(l1, _mm_mul_ps(al, b)); h1 = _mm_add_ps(h1, _mm_mul_ps(ah, b));
                b = _mm_set1_ps(bp[2]);
                l2 = _mm_add_ps(l2, _mm_mul_ps(al, b)); h2 = _mm_add_ps(h2, _mm_mul_ps(ah, b));
                b = _mm_set1_ps(bp[3]);
                l3 = _mm_add_ps(l3, _mm_mul_ps(al, b)); h3 = _mm_add_ps(h3, _mm_mul_ps(ah, b));
            }
            if (i >= j + 4) {   /* strictly below the diagonal: vector store of the tile, transposed store of its mirror */
                float* cp = C + i + (long)j * ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l0)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h0)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l1)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h1)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l2)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h2)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l3)); _mm_storeu_ps(cp + 4, _mm_add_ps(_mm_loadu_ps(cp + 4), h3));
                _MM_TRANSPOSE4_PS(l0, l1, l2, l3);   /* l_r = (acc(i+r, j+0..3)) */
                _MM_TRANSPOSE4_PS(h0, h1, h2, h3);
                cp = C + j + (long)i * ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l0)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l1)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l2)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), l3)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), h0)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), h1)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), h2)); cp += ldc;
                _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), h3));
            } else {            /* tile touches the diagonal: element by element */
                float t[8][4];
                _mm_storeu_ps(t[0], l0); _mm_storeu_ps(t[1], h0); _mm_storeu_ps(t[2], l1); _mm_storeu_ps(t[3], h1);
                _mm_storeu_ps(t[4], l2); _mm_storeu_ps(t[5], h2); _mm_storeu_ps(t[6], l3); _mm_storeu_ps(t[7], h3);
                for (c = 0; c < 4; ++c)
                    for (r = 0; r < 8; ++r) {
                        const int row = i + r, col = j + c;
                        const float v = t[2 * c + r / 4][r & 3];
                        if (row >= col) C[row + (long)col * ldc] = C[row + (long)col * ldc] + v;
                        if (row > col) C[col + (long)row * ldc] = C[col + (long)row * ldc] + v;
                    }
            }
        }
}
#endif

static void la_gebp(const float* A, int ars_, int acs_, const float* B, int brs_, int bcs_, float* C, int ldc_, int rows, int depth, int cols) {
    const long ars = ars_, acs = acs_, brs = brs_, bcs = bcs_, ldc = ldc_;
    const int peeled_mc2 = (rows / 8) * 8;
    const int peeled_mc1 = peeled_mc2 + ((rows - peeled_mc2) / 4) * 4;
    const int packet_cols4 = (cols / 4) * 4;
    const int peeled_kc = depth & ~7;
    const int endk4 = depth & ~3;
    int i, j, k;
#ifdef __SSE2__
    if (A == B && ars == 1 && bcs == 1 && acs == brs && rows == cols && peeled_mc2 > 0) {   /* symmetric product (J^T J, H^T H) */
        la_gebp_sym(A, acs, C, ldc, peeled_mc2, depth);
        la_gebp_fold(A, ars, acs, B, brs, bcs, C, ldc, 0, peeled_mc2, peeled_mc2, cols, depth);
    } else
#endif
    la_gebp_fold(A, ars, acs, B, brs, bcs, C, ldc, 0, peeled_mc2, 0, cols, depth);
    la_gebp_fold(A, ars, acs, B, brs, bcs, C, ldc, peeled_mc2, rows, packet_cols4, cols, depth);
    /* 4-row, 4-column-panel kernel rows: even k -> e, odd k -> o for k < peeled_kc, acc = e + o, leftover k sequential */
#ifdef __SSE2__
    if (bcs == 1) {   /* lanes = 4 consecutive columns of B (contiguous), one vector accumulator pair per row */
        for (j = 0; j < packet_cols4; j += 4)
            for (i = peeled_mc2; i < peeled_mc1; i += 4) {
                __m128 e0 = _mm_setzero_ps(), e1 = e0, e2 = e0, e3 = e0, o0 = e0, o1 = e0, o2 = e0, o3 = e0;
                const float* a0p = A + (long)i * ars;
                const float* a1p = a0p + ars;
                const float* a2p = a1p + ars;
                const float* a3p = a2p + ars;
                if (LA_SKIP(i, 4, j, 4)) continue;
                for (k = 0; k < peeled_kc; k += 2) {
                    const __m128 bv0 = _mm_loadu_ps(B + (long)k * brs + j), bv1 = _mm_loadu_ps(B + (long)(k + 1) * brs + j);
                    const long k0 = (long)k * acs, k1 = (long)(k + 1) * acs;
                    e0 = _mm_add_ps(e0, _mm_mul_ps(_mm_set1_ps(a0p[k0]), bv0)); o0 = _mm_add_ps(o0, _mm_mul_ps(_mm_set1_ps(a0p[k1]), bv1));
                    e1 = _mm_add_ps(e1, _mm_mul_ps(_mm_set1_ps(a1p[k0]), bv0)); o1 = _mm_add_ps(o1, _mm_mul_ps(_mm_set1_ps(a1p[k1]), bv1));
                    e2 = _mm_add_ps(e2, _mm_mul_ps(_mm_set1_ps(a2p[k0]), bv0)); o2 = _mm_add_ps(o2, _mm_mul_ps(_mm_set1_ps(a2p[k1]), bv1));
                    e3 = _mm_add_ps(e3, _mm_mul_ps(_mm_set1_ps(a3p[k0]), bv0)); o3 = _mm_add_ps(o3, _mm_mul_ps(_mm_set1_ps(a3p[k1]), bv1));
                }
                e0 = _mm_add_ps(e0, o0); e1 = _mm_add_ps(e1, o1); e2 = _mm_add_ps(e2, o2); e3 = _mm_add_ps(e3, o3);
                for (k = peeled_kc; k < depth; ++k) {
                    const __m128 bv0 = _mm_loadu_ps(B + (long)k * brs + j);
                    const long k0 = (long)k * acs;
                    e0 = _mm_add_ps(e0, _mm_mul_ps(_mm_set1_ps(a0p[k0]), bv0));
                    e1 = _mm_add_ps(e1, _mm_mul_ps(_mm_set1_ps(a1p[k0]), bv0));
                    e2 = _mm_add_ps(e2, _mm_mul_ps(_mm_set1_ps(a2p[k0]), bv0));
                    e3 = _mm_add_ps(e3, _mm_mul_ps(_mm_set1_ps(a3p[k0]), bv0));
                }
                _MM_TRANSPOSE4_PS(e0, e1, e2, e3);   /* e_c = acc(i..i+3, j+c) */
                {
                    float* cp = C + i + (long)j * ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), e0)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), e1)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), e2)); cp += ldc;
                    _mm_storeu_ps(cp, _mm_add_ps(_mm_loadu_ps(cp), e3));
                }
            }
    } else
#endif
    for (j = 0; j < packet_cols4; ++j)
        for (i = peeled_mc2; i < peeled_mc1; i += 4) {
            float e0 = 0.0f, e1 = 0.0f, e2 = 0.0f, e3 = 0.0f, o0 = 0.0f, o1 = 0.0f, o2 = 0.0f, o3 = 0.0f;
            if (LA_SKIP(i, 4, j, 1)) continue;
            for (k = 0; k < peeled_kc; k += 2) {
                const float b0 = LA_B(0);
                const float b1 = B[(long)(k + 1) * brs + (long)j * bcs];
                e0 = e0 + LA_A(0) * b0; e1 = e1 + LA_A(1) * b0; e2 = e2 + LA_A(2) * b0; e3 = e3 + LA_A(3) * b0;
                o0 = o0 + A[(long)(i + 0) * ars + (long)(k + 1) * acs] * b1; o1 = o1 + A[(long)(i + 1) * ars + (long)(k + 1) * acs] * b1;
                o2 = o2 + A[(long)(i + 2) * ars + (long)(k + 1) * acs] * b1; o3 = o3 + A[(long)(i + 3) * ars + (long)(k + 1) * acs] * b1;
            }
            e0 = e0 + o0; e1 = e1 + o1; e2 = e2 + o2; e3 = e3 + o3;
            for (k = peeled_kc; k < depth; ++k) {
                const float b0 = LA_B(0);
                e0 = e0 + LA_A(0) * b0; e1 = e1 + LA_A(1) * b0; e2 = e2 + LA_A(2) * b0; e3 = e3 + LA_A(3) * b0;
            }
            LA_CUP(0, 0, e0); LA_CUP(1, 0, e1); LA_CUP(2, 0, e2); LA_CUP(3, 0, e3);
        }
    /* remaining rows, one at a time, swapped path: 4 accumulators over k & 3 for k < endk4, acc = (c0+c1)+(c2+c3), then leftover k sequential */
#ifdef __SSE2__
    if (bcs == 1) {   /* lanes = 4 consecutive columns of B */
        for (j = 0; j < packet_cols4; j += 4)
            for (i = peeled_mc1; i < rows; ++i) {
                __m128 c0 = _mm_setzero_ps(), c1 = c0, c2 = c0, c3 = c0, acc;
                const float* a = A + (long)i * ars;
                float t[4];
                int q;
                if (LA_SKIP(i, 1, j, 4)) continue;
                for (k = 0; k < endk4; k += 4) {
                    c0 = _mm_add_ps(c0, _mm_mul_ps(_mm_set1_ps(a[(long)k * acs]), _mm_loadu_ps(B + (long)k * brs + j)));
                    c1 = _mm_add_ps(c1, _mm_mul_ps(_mm_set1_ps(a[(long)(k + 1) * acs]), _mm_loadu_ps(B + (long)(k + 1) * brs + j)));
                    c2 = _mm_add_ps(c2, _mm_mul_ps(_mm_set1_ps(a[(long)(k + 2) * acs]), _mm_loadu_ps(B + (long)(k + 2) * brs + j)));
                    c3 = _mm_add_ps(c3, _mm_mul_ps(_mm_set1_ps(a[(long)(k + 3) * acs]), _mm_loadu_ps(B + (long)(k + 3) * brs + j)));
                }
                acc = _mm_add_ps(_mm_add_ps(c0, c1), _mm_add_ps(c2, c3));
                for (k = endk4; k < depth; ++k) acc = _mm_add_ps(acc, _mm_mul_ps(_mm_set1_ps(a[(long)k * acs]), _mm_loadu_ps(B + (long)k * brs + j)));
                _mm_storeu_ps(t, acc);
                for (q = 0; q < 4; ++q) C[i + (long)(j + q) * ldc] = C[i + (long)(j + q) * ldc] + t[q];
            }
    } else
#endif
    for (j = 0; j < packet_cols4; ++j)
        for (i = peeled_mc1; i < rows; ++i) {
            float c0 = 0.0f, c1 = 0.0f, c2 = 0.0f, c3 = 0.0f, acc;
            const float* a = A + (long)i * ars;
            const float* b = B + (long)j * bcs;
            if (LA_SKIP(i, 1, j, 1)) continue;
            for (k = 0; k < endk4; k += 4) {
                c0 = c0 + a[(long)k * acs] * b[(long)k * brs];
                c1 = c1 + a[(long)(k + 1) * acs] * b[(long)(k + 1) * brs];
                c2 = c2 + a[(long)(k + 2) * acs] * b[(long)(k + 2) * brs];
                c3 = c3 + a[(long)(k + 3) * acs] * b[(long)(k + 3) * brs];
            }
            acc = (c0 + c1) + (c2 + c3);
            for (k = endk4; k < depth; ++k) acc = acc + a[(long)k * acs] * b[(long)k * brs];
            LA_CUP(0, 0, acc);
        }
}
#undef LA_A
#undef LA_B
#undef LA_CUP

/* general_matrix_matrix_product (column-major C, alpha = 1): C(m x n, ldc) += A * B, A(i,k) = a[i*ars + k*acs], B(k,j) = b[k*brs + j*bcs] */
static void la_gemm(int m, int n, int k, const float* a, int ars, int acs, const float* b, int brs, int bcs, float* c, int ldc) {
    unsigned char ra_buf[512], ca_buf[512];
    int kc = k, mc = m, nc = n, i2, k2, j2;
    if (m == 0 || n == 0 || k == 0) return;
    la_blocking(&kc, &mc, &nc);
    if (mc > m) mc = m;
    const unsigned char *base_ra = NULL, *base_ca = NULL;
    if (la_skip_enable && m <= 512 && n <= 512) {
        int q, any_zero = 0, fin = 1;
        const int sym = (a == b && ars == 1 && bcs == 1 && acs == brs && m == n);
        memset(ra_buf, 0, (size_t)m); memset(ca_buf, 0, (size_t)n);
        for (q = 0; q < k; ++q) {
            for (i2 = 0; i2 < m; ++i2) { const float v = a[(long)i2 * ars + (long)q * acs]; ra_buf[i2] |= (unsigned char)(v != 0.0f); fin &= (v - v == 0.0f); }
            if (!sym) for (j2 = 0; j2 < n; ++j2) { const float v = b[(long)q * brs + (long)j2 * bcs]; ca_buf[j2] |= (unsigned char)(v != 0.0f); fin &= (v - v == 0.0f); }
        }
        if (sym) memcpy(ca_buf, ra_buf, (size_t)n);
        for (i2 = 0; i2 < m; ++i2) any_zero |= !ra_buf[i2];
        for (j2 = 0; j2 < n; ++j2) any_zero |= !ca_buf[j2];
        if (fin && any_zero) { base_ra = ra_buf; base_ca = ca_buf; }
    }
    for (i2 = 0; i2 < m; i2 += mc) {
        int amc = (i2 + mc < m ? i2 + mc : m) - i2;
        for (k2 = 0; k2 < k; k2 += kc) {
            int akc = (k2 + kc < k ? k2 + kc : k) - k2;
            for (j2 = 0; j2 < n; j2 += nc) {
                int anc = (j2 + nc < n ? j2 + nc : n) - j2;
                la_ra = base_ra ? base_ra + i2 : NULL; la_ca = base_ca ? base_ca + j2 : NULL;
                la_gebp(a + (long)i2 * ars + (long)k2 * acs, ars, acs, b + (long)k2 * brs + (long)j2 * bcs, brs, bcs,
                        c + i2 + (long)j2 * ldc, ldc, amc, akc, anc);
            }
        }
    }
    la_ra = la_ca = NULL;
}

/* general_matrix_vector_product, y(m) += alpha * (A x) with A(i,j) = a[i*ars + j*acs].
 * acs == 1: Eigen's row-major kernel (lane sums over packets of 4, predux = (l0+l2)+(l1+l3), scalar tail);
 * otherwise (ars == 1) the column-major kernel (per row one left fold from 0 over the columns, in blocks of block_cols). */
static void la_gemv(int m, int n, const float* a, int ars, int acs, const float* x, int xinc, float* y, float alpha) {
    int i, j;
    if (acs == 1) {
        for (i = 0; i < m; ++i) {
            float l[4] = {0.0f, 0.0f, 0.0f, 0.0f}, cc;
#ifdef __SSE2__
            if (xinc == 1) {   /* the 4 lane sums are one packet accumulator: separate IEEE mul and add per lane */
                const float* ar = a + (long)i * ars;
                __m128 acc = _mm_setzero_ps();
                for (j = 0; j + 4 <= n; j += 4) acc = _mm_add_ps(acc, _mm_mul_ps(_mm_loadu_ps(ar + j), _mm_loadu_ps(x + j)));
                _mm_storeu_ps(l, acc);
            } else
#endif
            for (j = 0; j + 4 <= n; j += 4) {
                int t;
                for (t = 0; t < 4; ++t) l[t] = l[t] + a[(long)i * ars + j + t] * x[(long)(j + t) * xinc];
            }
            cc = (l[0] + l[2]) + (l[1] + l[3]);
            for (; j < n; ++j) cc = cc + a[(long)i * ars + j] * x[(long)j * xinc];
            y[i] = y[i] + alpha * cc;
        }
    } else {
        const long lhs_stride = acs;
        const int block_cols = n < 128 ? n : ((long)lhs_stride * 4 < 32000 ? 16 : 4);
        int j2;
        for (j2 = 0; j2 < n; j2 += block_cols) {
            int jend = j2 + block_cols < n ? j2 + block_cols : n;
            i = 0;
#ifdef __SSE2__
            for (; i + 4 <= m; i += 4) {   /* 4 rows per vector, each lane its own left fold over the columns */
                __m128 c0 = _mm_setzero_ps();
                for (j = j2; j < jend; ++j) c0 = _mm_add_ps(c0, _mm_mul_ps(_mm_loadu_ps(a + i + (long)j * acs), _mm_set1_ps(x[(long)j * xinc])));
                _mm_storeu_ps(y + i, _mm_add_ps(_mm_loadu_ps(y + i), _mm_mul_ps(_mm_set1_ps(alpha), c0)));
            }
#endif
            for (; i < m; ++i) {
                float c0 = 0.0f;
                for (j = j2; j < jend; ++j) c0 = c0 + a[i + (long)j * acs] * x[(long)j * xinc];
                y[i] = y[i] + alpha * c0;
            }
        }
    }
}
