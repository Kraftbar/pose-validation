/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Module M7: the Levenberg-Marquardt loop of SqrtKeypointVioEstimator<float>::optimize() and the dense LDLT solve, see bs_vio_opt.h.
 *
 * Eigen 3.4.0 evaluation orders used by the LDLT (all measured against the real LDLT<Ref<MatX>> by bs_vio_opt_test.cc `ldlt`):
 *   factor (ldlt_inplace<Lower>::unblocked, the only algorithm LDLT has in 3.4): pivot = first maximal |diag| in the trailing block,
 *     temp = diag(0..k) .* A10^T (elementwise products), A(k,k) -= left fold (A10 strided: plain fold from the first product),
 *     A21 -= A20 * temp  (colmajor GEMV, alpha -1), A21 /= A(k,k) (true division);
 *   solve: P b, unit-lower solve (triangular_solve_vector<ColMajor>, panel width 8: in-panel axpy, then GEMV alpha -1), D^-1 (|d| > FLT_MIN
 *     else 0), unit-upper solve with L^T (triangular_solve_vector<RowMajor>: row GEMV alpha -1 for the rows above the panel, in-panel
 *     dot product as a 2-packet redux), P^T. */
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bs_vio_opt.h"

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "bs_linabsqr_dense.inc"

long bs_ldlt_stats[4] = {0, 0, 0, 0};

static float vo_fold(const float* p, int n) {
    float s = p[0];
    int i;
    for (i = 1; i < n; ++i) s = s + p[i];
    return s;
}

/* LinearVectorizedTraversal redux over an expression (alignedStart 0), SSE packet 4: see bs_linabsqr.c vec_sum */
static float vo_vec_sum(const float* p, int n) {
    const int size2 = (n / 8) * 8, size1 = (n / 4) * 4;
    float r0[4], r1[4], res;
    int i, idx;
    if (size1 == 0) return vo_fold(p, n);
    for (i = 0; i < 4; ++i) r0[i] = p[i];
    if (size1 > 4) {
        for (i = 0; i < 4; ++i) r1[i] = p[4 + i];
        for (idx = 8; idx < size2; idx += 8)
            for (i = 0; i < 4; ++i) { r0[i] = r0[i] + p[idx + i]; r1[i] = r1[i] + p[idx + 4 + i]; }
        for (i = 0; i < 4; ++i) r0[i] = r0[i] + r1[i];
        if (size1 > size2)
            for (i = 0; i < 4; ++i) r0[i] = r0[i] + p[size2 + i];
    }
    res = (r0[0] + r0[2]) + (r0[1] + r0[3]);
    for (idx = size1; idx < n; ++idx) res = res + p[idx];
    return res;
}

/* ------------------------------------------------------------------------------------------------ LDLT */

void bs_ldlt_factor(float* A, int n, int* trans) {
    float* temp;
    int k, i;
    bs_ldlt_stats[0]++;
    if (n <= 1) { for (k = 0; k < n; ++k) trans[k] = k; return; }
    temp = (float*)malloc(sizeof(float) * (size_t)n);
    for (k = 0; k < n; ++k) {
        int big = k, rs, s;
        float best = fabsf(A[k + (size_t)k * n]), akk;
        for (i = k + 1; i < n; ++i) {
            const float a = fabsf(A[i + (size_t)i * n]);
            if (a > best) { best = a; big = i; }                  /* max_coeff_visitor: strict > keeps the first maximum */
        }
        trans[k] = big;
        if (k != big) {
            float t;
            s = n - big - 1;
            for (i = 0; i < k; ++i) { t = A[k + (size_t)i * n]; A[k + (size_t)i * n] = A[big + (size_t)i * n]; A[big + (size_t)i * n] = t; }
            for (i = 0; i < s; ++i) { t = A[(n - s + i) + (size_t)k * n]; A[(n - s + i) + (size_t)k * n] = A[(n - s + i) + (size_t)big * n]; A[(n - s + i) + (size_t)big * n] = t; }
            t = A[k + (size_t)k * n]; A[k + (size_t)k * n] = A[big + (size_t)big * n]; A[big + (size_t)big * n] = t;
            for (i = k + 1; i < big; ++i) { t = A[i + (size_t)k * n]; A[i + (size_t)k * n] = A[big + (size_t)i * n]; A[big + (size_t)i * n] = t; }
        }
        rs = n - k - 1;
        if (k > 0) {
            float v;
            for (i = 0; i < k; ++i) temp[i] = A[i + (size_t)i * n] * A[k + (size_t)i * n];
            v = A[k] * temp[0];                                    /* A10 * temp: strided operand, DefaultTraversal left fold */
            for (i = 1; i < k; ++i) v = v + A[k + (size_t)i * n] * temp[i];
            A[k + (size_t)k * n] = A[k + (size_t)k * n] - v;
            if (rs == 1) {                                         /* GemvProduct with lhs.rows() == 1 && rhs.cols() == 1: dst += alpha * lhs.row(0).dot(rhs) (strided: left fold) */
                float d = A[(k + 1)] * temp[0];
                for (i = 1; i < k; ++i) d = d + A[(k + 1) + (size_t)i * n] * temp[i];
                A[(k + 1) + (size_t)k * n] = A[(k + 1) + (size_t)k * n] + (-1.0f) * d;
            } else if (rs > 1) la_gemv(rs, k, &A[k + 1], 1, n, temp, 1, &A[(k + 1) + (size_t)k * n], -1.0f);
        }
        akk = A[k + (size_t)k * n];
        if (k == 0 && !(fabsf(akk) > 0.0f)) {                      /* the entire diagonal is zero: identity transpositions, nothing else */
            int j;
            for (j = 0; j < n; ++j) trans[j] = j;
            bs_ldlt_stats[1]++;
            free(temp);
            return;
        }
        if (rs > 0 && fabsf(akk) > 0.0f)
            for (i = 0; i < rs; ++i) A[(k + 1 + i) + (size_t)k * n] = A[(k + 1 + i) + (size_t)k * n] / akk;
    }
    for (k = 0; k < n; ++k) if (trans[k] != k) { bs_ldlt_stats[3]++; break; }
    free(temp);
}

void bs_ldlt_solve(const float* A, int n, const int* trans, const float* b, float* x) {
#define PW 8
    int i, k, pi;
    if (x != b) memcpy(x, b, sizeof(float) * (size_t)n);
    /* dst = P b */
    for (k = 0; k < n; ++k) { const int j = trans[k]; if (j != k) { float t = x[k]; x[k] = x[j]; x[j] = t; } }
    /* L^-1, unit lower, column-major kernel */
    for (pi = 0; pi < n; pi += PW) {
        const int apw = (n - pi < PW) ? n - pi : PW;
        const int startBlock = pi, endBlock = pi + apw;
        int r;
        for (k = 0; k < apw; ++k) {
            const int ii = pi + k;
            if (x[ii] != 0.0f) {
                const int rr = apw - k - 1, s = ii + 1;
                for (i = 0; i < rr; ++i) x[s + i] = x[s + i] - x[ii] * A[(s + i) + (size_t)ii * n];
            }
        }
        r = n - endBlock;
        if (r > 0) la_gemv(r, apw, &A[endBlock + (size_t)startBlock * n], 1, n, x + startBlock, 1, x + endBlock, -1.0f);
    }
    /* D^-1 (pseudo inverse, tolerance = FLT_MIN) */
    for (i = 0; i < n; ++i) {
        const float d = A[i + (size_t)i * n];
        if (fabsf(d) > FLT_MIN) x[i] = x[i] / d; else x[i] = 0.0f;
    }
    /* L^-T, unit upper, row-major kernel on the transposed storage: (L^T)(i,j) = A[j + i*n] = L(j,i) -> row-major element (i,j) at A[i*n + j]?
     * no: Transpose<Ref> keeps the pointer and outer stride: element (i,j) of L^T is L(j,i) = A[j + i*n], i.e. row-major with row stride n. */
    for (pi = n; pi > 0; pi -= PW) {
        const int apw = (pi < PW) ? pi : PW;
        const int r = n - pi;
        if (r > 0) {
            const int startRow = pi - apw, startCol = pi;
            la_gemv(apw, r, &A[(size_t)startRow * n + startCol], n, 1, x + startCol, 1, x + startRow, -1.0f);
        }
        for (k = 0; k < apw; ++k) {
            const int ii = pi - k - 1, s = ii + 1;
            if (k > 0) {
                float t[PW];
                int m;
                for (m = 0; m < k; ++m) t[m] = A[(size_t)ii * n + (s + m)] * x[s + m];
                x[ii] = x[ii] - vo_vec_sum(t, k);
            }
        }
    }
    /* dst = P^T dst */
    for (k = n - 1; k >= 0; --k) { const int j = trans[k]; if (j != k) { float t = x[k]; x[k] = x[j]; x[j] = t; } }
#undef PW
}

/* ------------------------------------------------------------------------------------------------ state container */

void bs_vio_init(bs_vio* v) {
    memset(v, 0, sizeof(*v));
    bs_ba_init(&v->ba);
    v->max_states = 3; v->max_kfs = 7;
    v->kf_marg_feature_ratio = 0.1;
    v->lm_lambda_initial = 1e-4; v->lambda = (float)1e-4; v->min_lambda = (float)1e-6; v->max_lambda = (float)1e2; v->lambda_vee = 2.0f;
    v->max_iterations = 7;
    v->take_kf = 1;
}

void bs_vio_destroy(bs_vio* v) {
    free(v->ba.poses); free(v->ba.states); free(v->imu);
    free(v->marg.item); free(v->marg.H); free(v->marg.b);
    free(v->kf_ids); free(v->num_points_kf);
    bs_ba_destroy(&v->ba);
    memset(v, 0, sizeof(*v));
}

#define BS_GROW(ptr, n, cap) do { if ((n) + 1 > (cap)) { (cap) = (cap) ? 2 * (cap) : 16; (ptr) = realloc((ptr), sizeof(*(ptr)) * (size_t)(cap)); } } while (0)

bs_frame_pose* bs_vio_pose_insert(bs_vio* v, int64_t t_ns) {
    int lo = 0, hi = v->ba.n_poses;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.poses[m].t_ns < t_ns) lo = m + 1; else hi = m; }
    if (lo < v->ba.n_poses && v->ba.poses[lo].t_ns == t_ns) return NULL;
    BS_GROW(v->ba.poses, v->ba.n_poses, v->cap_poses);
    memmove(&v->ba.poses[lo + 1], &v->ba.poses[lo], sizeof(bs_frame_pose) * (size_t)(v->ba.n_poses - lo));
    v->ba.n_poses++;
    memset(&v->ba.poses[lo], 0, sizeof(bs_frame_pose));
    v->ba.poses[lo].t_ns = t_ns;
    return &v->ba.poses[lo];
}

bs_frame_state* bs_vio_state_insert(bs_vio* v, int64_t t_ns) {
    int lo = 0, hi = v->ba.n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.states[m].t_ns < t_ns) lo = m + 1; else hi = m; }
    if (lo < v->ba.n_states && v->ba.states[lo].t_ns == t_ns) return NULL;
    BS_GROW(v->ba.states, v->ba.n_states, v->cap_states);
    memmove(&v->ba.states[lo + 1], &v->ba.states[lo], sizeof(bs_frame_state) * (size_t)(v->ba.n_states - lo));
    v->ba.n_states++;
    memset(&v->ba.states[lo], 0, sizeof(bs_frame_state));
    v->ba.states[lo].t_ns = t_ns;
    return &v->ba.states[lo];
}

bs_imu_meas* bs_vio_imu_insert(bs_vio* v, int64_t start_t_ns) {
    int lo = 0, hi = v->n_imu;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->imu[m].start_t_ns < start_t_ns) lo = m + 1; else hi = m; }
    if (lo < v->n_imu && v->imu[lo].start_t_ns == start_t_ns) return NULL;
    BS_GROW(v->imu, v->n_imu, v->cap_imu);
    memmove(&v->imu[lo + 1], &v->imu[lo], sizeof(bs_imu_meas) * (size_t)(v->n_imu - lo));
    v->n_imu++;
    memset(&v->imu[lo], 0, sizeof(bs_imu_meas));
    v->imu[lo].start_t_ns = start_t_ns;
    return &v->imu[lo];
}

void bs_vio_kf_insert(bs_vio* v, int64_t t_ns) {
    int lo = 0, hi = v->n_kf;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->kf_ids[m] < t_ns) lo = m + 1; else hi = m; }
    if (lo < v->n_kf && v->kf_ids[lo] == t_ns) return;
    BS_GROW(v->kf_ids, v->n_kf, v->cap_kf);
    memmove(&v->kf_ids[lo + 1], &v->kf_ids[lo], sizeof(int64_t) * (size_t)(v->n_kf - lo));
    v->kf_ids[lo] = t_ns;
    v->n_kf++;
}

void bs_vio_npk_set(bs_vio* v, int64_t t_ns, int n) {
    int lo = 0, hi = v->n_npk;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->num_points_kf[m].t_ns < t_ns) lo = m + 1; else hi = m; }
    if (lo < v->n_npk && v->num_points_kf[lo].t_ns == t_ns) { v->num_points_kf[lo].n = n; return; }
    BS_GROW(v->num_points_kf, v->n_npk, v->cap_npk);
    memmove(&v->num_points_kf[lo + 1], &v->num_points_kf[lo], sizeof(bs_kf_count) * (size_t)(v->n_npk - lo));
    v->num_points_kf[lo].t_ns = t_ns; v->num_points_kf[lo].n = n;
    v->n_npk++;
}

int bs_vio_npk_get(const bs_vio* v, int64_t t_ns, int* n) {
    int lo = 0, hi = v->n_npk;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->num_points_kf[m].t_ns < t_ns) lo = m + 1; else hi = m; }
    if (lo < v->n_npk && v->num_points_kf[lo].t_ns == t_ns) { *n = v->num_points_kf[lo].n; return 1; }
    return 0;
}

void bs_marg_data_set(bs_marg_data* m, const bs_aom_item* items, int n, int total, int rows, int cols, const float* H, const float* b) {
    free(m->item); free(m->H); free(m->b);
    m->item = (bs_aom_item*)malloc(sizeof(bs_aom_item) * (size_t)(n > 0 ? n : 1));
    if (n > 0) memcpy(m->item, items, sizeof(bs_aom_item) * (size_t)n);
    m->n = n; m->total = total; m->rows = rows; m->cols = cols;
    m->H = (float*)malloc(sizeof(float) * ((size_t)rows * cols > 0 ? (size_t)rows * cols : 1));
    m->b = (float*)malloc(sizeof(float) * (size_t)(rows > 0 ? rows : 1));
    if ((size_t)rows * (size_t)cols > 0) memcpy(m->H, H, sizeof(float) * (size_t)rows * cols);
    if (rows) memcpy(m->b, b, sizeof(float) * (size_t)rows);
}

void bs_vio_init_marg_prior(bs_vio* v, int64_t t_ns, double init_pose_weight, double init_ba_weight, double init_bg_weight) {
    float H[15 * 15], b[15];
    bs_aom_item it;
    int i;
    memset(H, 0, sizeof H); memset(b, 0, sizeof b);
    for (i = 0; i < 3; ++i) H[i + 15 * i] = sqrtf((float)init_pose_weight);
    H[5 + 15 * 5] = sqrtf((float)init_pose_weight);
    for (i = 9; i < 12; ++i) H[i + 15 * i] = sqrtf((float)init_ba_weight);
    for (i = 12; i < 15; ++i) H[i + 15 * i] = sqrtf((float)init_bg_weight);
    it.t_ns = t_ns; it.start = 0; it.size = 15;
    bs_marg_data_set(&v->marg, &it, 1, 15, 15, 15, H, b);
}

void bs_pose_from_state(const bs_frame_state* s, bs_frame_pose* p) {
    float d6[6];
    int i;
    for (i = 0; i < 6; ++i) d6[i] = s->delta[i];
    p->t_ns = s->t_ns;
    p->linearized = s->s.linearized;
    p->lin.so3.x = s->s.lin.s.q[0]; p->lin.so3.y = s->s.lin.s.q[1]; p->lin.so3.z = s->s.lin.s.q[2]; p->lin.so3.w = s->s.lin.s.q[3];
    for (i = 0; i < 3; ++i) p->lin.t[i] = s->s.lin.s.p[i];
    p->cur = p->lin;
    bs_inc_posef(d6, &p->cur);
    for (i = 0; i < 6; ++i) p->delta[i] = d6[i];
}

void bs_vio_build_aom(const bs_vio* v, bs_aom_item** items, bs_aom* aom) {
    const int n = v->ba.n_poses + v->ba.n_states;
    bs_aom_item* it = (bs_aom_item*)malloc(sizeof(bs_aom_item) * (size_t)(n > 0 ? n : 1));
    int i, total = 0, m = 0, a, b2;
    for (i = 0; i < v->ba.n_poses; ++i) { it[m].t_ns = v->ba.poses[i].t_ns; it[m].start = total; it[m].size = 6; total += 6; ++m; }
    for (i = 0; i < v->ba.n_states; ++i) { it[m].t_ns = v->ba.states[i].t_ns; it[m].start = total; it[m].size = 15; total += 15; ++m; }
    /* abs_order_map is a std::map keyed by time: iteration / lookup order is by t_ns (insertion sort, offsets stay) */
    for (a = 1; a < n; ++a) { bs_aom_item x = it[a]; b2 = a - 1; while (b2 >= 0 && it[b2].t_ns > x.t_ns) { it[b2 + 1] = it[b2]; --b2; } it[b2 + 1] = x; }
    *items = it;
    aom->item = it; aom->n = n; aom->total_size = total;
}

/* ------------------------------------------------------------------------------------------------ optimize() */

static const bs_aom_item* find_item(const bs_aom_item* it, int n, int64_t t) {
    int lo = 0, hi = n;
    while (lo < hi) { int m = (lo + hi) / 2; if (it[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < n && it[lo].t_ns == t) ? &it[lo] : NULL;
}

void bs_vio_optimize(bs_vio* v, bs_opt_info* info, bs_opt_cb cb, void* ctx) {
    bs_opt_info dummy;
    bs_aom_item* items;
    bs_aom aom;
    bs_marg_lin mg;
    bs_imu_lin ild;
    bs_imu_meas** imup;
    bs_linabsqr* la;
    float *H, *Hc, *b, *inc, gw[3], aw[3];
    int* trans;
    int n, i, it = 0, it_rejected = 0, terminated = 0, converged = 0;
    const float vee_factor = 2.0f, initial_vee = 2.0f;
    if (!info) info = &dummy;
    memset(info, 0, sizeof(*info));
    if (!(v->opt_started || v->ba.n_states > 4)) return;
    v->opt_started = 1;
    info->ran = 1;

    bs_vio_build_aom(v, &items, &aom);
    n = aom.total_size;
    /* BASALT_ASSERT: the marginalisation prior order is the same as the aom for the common prefix */
    {
        int cnt = 0;
        for (i = 0; i < v->ba.n_poses; ++i) {
            const bs_aom_item* a = find_item(aom.item, aom.n, v->ba.poses[i].t_ns);
            const bs_aom_item* m = find_item(v->marg.item, v->marg.n, v->ba.poses[i].t_ns);
            if (!m || m->start != a->start || m->size != a->size) info->layout_error = 1;
            ++cnt;
        }
        for (i = 0; i < v->ba.n_states; ++i) {
            if (cnt < v->marg.n) {
                const bs_aom_item* a = find_item(aom.item, aom.n, v->ba.states[i].t_ns);
                const bs_aom_item* m = find_item(v->marg.item, v->marg.n, v->ba.states[i].t_ns);
                if (!m || m->start != a->start || m->size != a->size) info->layout_error = 1;
            }
            ++cnt;
        }
    }
    v->lambda = (float)v->lm_lambda_initial;

    mg.order.item = v->marg.item; mg.order.n = v->marg.n; mg.order.total_size = v->marg.total;
    mg.rows = v->marg.rows; mg.cols = v->marg.cols; mg.H = v->marg.H; mg.b = v->marg.b;
    imup = (bs_imu_meas**)malloc(sizeof(bs_imu_meas*) * (size_t)(v->n_imu > 0 ? v->n_imu : 1));
    for (i = 0; i < v->n_imu; ++i) imup[i] = &v->imu[i];
    ild.n = v->n_imu; ild.meas = imup;
    for (i = 0; i < 3; ++i) { ild.g[i] = v->g[i]; ild.gyro_bias_weight_sqrt[i] = v->gyro_bias_sqrt_weight[i]; ild.accel_bias_weight_sqrt[i] = v->accel_bias_sqrt_weight[i]; }
    for (i = 0; i < 3; ++i) { gw[i] = v->gyro_bias_sqrt_weight[i] * v->gyro_bias_sqrt_weight[i]; aw[i] = v->accel_bias_sqrt_weight[i] * v->accel_bias_sqrt_weight[i]; }
    la = bs_la_create(&v->ba, &aom, &mg, &ild, NULL, -1, NULL, -1);

    H = (float*)malloc(sizeof(float) * (size_t)n * n);
    Hc = (float*)malloc(sizeof(float) * (size_t)n * n);
    b = (float*)malloc(sizeof(float) * (size_t)n);
    inc = (float*)malloc(sizeof(float) * (size_t)n);
    trans = (int*)malloc(sizeof(int) * (size_t)n);

    for (; it <= v->max_iterations && !terminated;) {
        float error_total;
        int valid = 1, j;
        error_total = bs_la_linearize_problem(la, &valid);
        if (!valid) { info->invalid_linearization = 1; break; }
        bs_la_perform_qr(la);
        for (j = 0; it <= v->max_iterations && !terminated; j++) {
            const float lambda0 = v->lambda;
            float l_diff, step_norminf, after_marg = 0.0f, after_vi = 0.0f, after_total, f_diff, relative_decrease = 0.0f;
            int step_valid, step_ok, iter = 0, inc_valid = 0, p;
            bs_frame_pose* poses0;
            bs_frame_state* states0;
            bs_opt_step st;

            bs_la_get_dense_H_b(la, H, b);
            while (iter < 3 && !inc_valid) {
                int fin = 1;
                memcpy(Hc, H, sizeof(float) * (size_t)n * n);
                for (i = 0; i < n; ++i) {
                    const float a = H[i + (size_t)i * n] * v->lambda;
                    const float hd = (a < v->min_lambda) ? v->min_lambda : a;      /* cwiseMax(min_lambda): scalar_max_op, (a < b) ? b : a */
                    Hc[i + (size_t)i * n] = Hc[i + (size_t)i * n] + hd;
                }
                bs_ldlt_factor(Hc, n, trans);
                bs_ldlt_solve(Hc, n, trans, b, inc);
                for (i = 0; i < n; ++i) if (!isfinite(inc[i])) { fin = 0; break; }
                if (!fin) { v->lambda = v->lambda_vee * v->lambda; v->lambda_vee *= vee_factor; info->retries++; bs_ldlt_stats[2]++; }
                else inc_valid = 1;
                iter++;
            }
            if (!inc_valid) { info->nonfinite_increment = 1; goto done; }   /* the C++ prints "Still invalid inc", continues, and Sophus' SO3::exp(NaN) aborts in applyInc */
            if (cb) { memset(&st, 0, sizeof st); st.phase = 0; st.it = it; st.j = j; st.lambda_before = lambda0; st.error_total = error_total; st.n = n; st.H = H; st.b = b; st.inc = inc; st.vio = v; cb(ctx, &st); }

            /* backup() */
            poses0 = (bs_frame_pose*)malloc(sizeof(bs_frame_pose) * (size_t)(v->ba.n_poses > 0 ? v->ba.n_poses : 1));
            states0 = (bs_frame_state*)malloc(sizeof(bs_frame_state) * (size_t)(v->ba.n_states > 0 ? v->ba.n_states : 1));
            if (v->ba.n_poses > 0) memcpy(poses0, v->ba.poses, sizeof(bs_frame_pose) * (size_t)v->ba.n_poses);
            if (v->ba.n_states > 0) memcpy(states0, v->ba.states, sizeof(bs_frame_state) * (size_t)v->ba.n_states);
            bs_lmdb_backup(&v->ba.lmdb);

            for (i = 0; i < n; ++i) inc[i] = -inc[i];
            l_diff = bs_la_back_substitute(la, inc);
            for (p = 0; p < v->ba.n_poses; ++p) bs_pose_apply_inc(&v->ba.poses[p], &inc[find_item(aom.item, aom.n, v->ba.poses[p].t_ns)->start]);
            for (p = 0; p < v->ba.n_states; ++p) bs_state_apply_inc(&v->ba.states[p], &inc[find_item(aom.item, aom.n, v->ba.states[p].t_ns)->start]);
            step_norminf = fabsf(inc[0]);
            for (i = 1; i < n; ++i) { const float a = fabsf(inc[i]); if (step_norminf < a) step_norminf = a; }

            {
                float ie, be, ae;
                after_vi = bs_ba_compute_error(&v->ba);
                after_marg = bs_ba_marg_prior_error(&v->ba, &mg);
                bs_ba_compute_imu_error(&v->ba, &aom, imup, v->n_imu, v->g, gw, aw, &ie, &be, &ae);
                after_vi += ie + be + ae;
            }
            after_total = after_vi + after_marg;
            f_diff = error_total - after_total;
            relative_decrease = f_diff / l_diff;
            step_valid = l_diff > 0.0f;
            step_ok = step_valid && relative_decrease > 0.0f;
            info->steps++;

            if (step_ok) {
                const float third = 1.0f / 3;
                const float x = (float)(1 - pow((double)(2 * relative_decrease - 1), 3.0));
                v->lambda *= (third < x) ? x : third;                      /* std::max<Scalar>(1/3, 1 - std::pow<Scalar>(2 rd - 1, 3)); pow(double, double) */
                v->lambda = (v->min_lambda < v->lambda) ? v->lambda : v->min_lambda;
                v->lambda_vee = initial_vee;
                it++;
                if ((f_diff > 0.0f && f_diff < 1e-6f) || step_norminf < 1e-4f) { converged = 1; terminated = 1; }
            } else {
                v->lambda = v->lambda_vee * v->lambda;
                v->lambda_vee *= vee_factor;
            }
            if (cb) {
                memset(&st, 0, sizeof st);
                st.phase = 1; st.it = it - (step_ok ? 1 : 0); st.j = j; st.lambda_before = lambda0; st.error_total = error_total; st.n = n; st.H = H; st.b = b; st.inc = inc;
                st.l_diff = l_diff; st.f_diff = f_diff; st.relative_decrease = relative_decrease; st.step_norminf = step_norminf; st.after_vi = after_vi; st.after_marg = after_marg;
                st.accepted = step_ok; st.step_valid = step_valid; st.lambda_after = v->lambda; st.vio = v;
                cb(ctx, &st);
            }
            if (!step_ok) {
                /* restore() */
                if (v->ba.n_poses > 0) memcpy(v->ba.poses, poses0, sizeof(bs_frame_pose) * (size_t)v->ba.n_poses);
                if (v->ba.n_states > 0) memcpy(v->ba.states, states0, sizeof(bs_frame_state) * (size_t)v->ba.n_states);
                bs_lmdb_restore(&v->ba.lmdb);
                it++;
                it_rejected++;
                if (v->lambda > v->max_lambda) terminated = 1;
            }
            free(poses0); free(states0);
            if (step_ok) break;
        }
    }
done:
    info->it = it; info->it_rejected = it_rejected; info->converged = converged; info->terminated = terminated;
    bs_la_destroy(la);
    free(H); free(Hc); free(b); free(inc); free(trans); free(imup); free(items);
}
