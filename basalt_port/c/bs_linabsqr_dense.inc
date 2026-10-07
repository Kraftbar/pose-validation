/* Dense Eigen 3.4.0 float kernels (SSE2, packet 4, no FMA) as used by the ABS_QR linearisation: dynamic GEMM (GEBP incl. the blocking
 * heuristic with the cache sizes of the reference machine) and GEMV (column-major and row-major kernel). Included by bs_linabsqr.c. */

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
static void la_gebp(const float* A, int ars, int acs, const float* B, int brs, int bcs, float* C, int ldc, int rows, int depth, int cols) {
    const int peeled_mc2 = (rows / 8) * 8;
    const int peeled_mc1 = peeled_mc2 + ((rows - peeled_mc2) / 4) * 4;
    const int packet_cols4 = (cols / 4) * 4;
    const int peeled_kc = depth & ~7;
    int i, j, k;
    for (j = 0; j < cols; ++j)
        for (i = 0; i < rows; ++i) {
            float acc;
            if (i < peeled_mc2 || j >= packet_cols4) {
                acc = 0.0f;
                for (k = 0; k < depth; ++k) acc = acc + A[i * ars + k * acs] * B[k * brs + j * bcs];
            } else if (i < peeled_mc1) {
                float ce = 0.0f, co = 0.0f;
                for (k = 0; k < peeled_kc; ++k) {
                    float p = A[i * ars + k * acs] * B[k * brs + j * bcs];
                    if (k & 1) co = co + p; else ce = ce + p;
                }
                acc = ce + co;
                for (k = peeled_kc; k < depth; ++k) acc = acc + A[i * ars + k * acs] * B[k * brs + j * bcs];
            } else {
                float c[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                const int endk4 = depth & ~3;
                for (k = 0; k < endk4; ++k) c[k & 3] = c[k & 3] + A[i * ars + k * acs] * B[k * brs + j * bcs];
                acc = (c[0] + c[1]) + (c[2] + c[3]);
                for (k = endk4; k < depth; ++k) acc = acc + A[i * ars + k * acs] * B[k * brs + j * bcs];
            }
            C[i + (long)j * ldc] = C[i + (long)j * ldc] + acc * 1.0f;   /* res + alpha * acc */
        }
}

/* general_matrix_matrix_product (column-major C, alpha = 1): C(m x n, ldc) += A * B, A(i,k) = a[i*ars + k*acs], B(k,j) = b[k*brs + j*bcs] */
static void la_gemm(int m, int n, int k, const float* a, int ars, int acs, const float* b, int brs, int bcs, float* c, int ldc) {
    int kc = k, mc = m, nc = n, i2, k2, j2;
    if (m == 0 || n == 0 || k == 0) return;
    la_blocking(&kc, &mc, &nc);
    if (mc > m) mc = m;
    for (i2 = 0; i2 < m; i2 += mc) {
        int amc = (i2 + mc < m ? i2 + mc : m) - i2;
        for (k2 = 0; k2 < k; k2 += kc) {
            int akc = (k2 + kc < k ? k2 + kc : k) - k2;
            for (j2 = 0; j2 < n; j2 += nc) {
                int anc = (j2 + nc < n ? j2 + nc : n) - j2;
                la_gebp(a + (long)i2 * ars + (long)k2 * acs, ars, acs, b + (long)k2 * brs + (long)j2 * bcs, brs, bcs,
                        c + i2 + (long)j2 * ldc, ldc, amc, akc, anc);
            }
        }
    }
}

/* general_matrix_vector_product, y(m) += alpha * (A x) with A(i,j) = a[i*ars + j*acs].
 * acs == 1: Eigen's row-major kernel (lane sums over packets of 4, predux = (l0+l2)+(l1+l3), scalar tail);
 * otherwise (ars == 1) the column-major kernel (per row one left fold from 0 over the columns, in blocks of block_cols). */
static void la_gemv(int m, int n, const float* a, int ars, int acs, const float* x, int xinc, float* y, float alpha) {
    int i, j;
    if (acs == 1) {
        for (i = 0; i < m; ++i) {
            float l[4] = {0.0f, 0.0f, 0.0f, 0.0f}, cc;
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
            for (i = 0; i < m; ++i) {
                float c0 = 0.0f;
                for (j = j2; j < jend; ++j) c0 = c0 + a[i + (long)j * acs] * x[(long)j * xinc];
                y[i] = y[i] + alpha * c0;
            }
        }
    }
}
