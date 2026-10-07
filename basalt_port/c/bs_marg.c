/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Module M8: marginalize() and marginalizeHelperSqrtToSqrt, see bs_marg.h.
 *
 * Eigen 3.4.0 evaluation orders used (all measured against the real MargHelper<float> / estimator by bs_marg_test.cc):
 *   makeHouseholderInPlace on a contiguous column tail: tailSqNorm = squaredNorm of an expression, alignedStart 0, two-packet linear vectorised redux;
 *     beta = -sign(c0) sqrt(c0^2 + tailSqNorm), essential = tail / (c0 - beta) (true division), tau = (beta - c0) / beta;
 *   applyHouseholderOnTheLeft: rows == 1: block *= (1 - tau); else tmp = essential^T * bottom (1 column: inner product = vec_sum redux; otherwise the
 *     RowMajor GEMV kernel of bs_linabsqr_dense.inc), tmp += row(0), row(0) -= tau * tmp, bottom(i,j) -= tmp_j * (tau * essential_i);
 *   marg_data.b -= marg_data.H * delta: the product is evaluated into a temporary (column-major GEMV, alpha 1), then subtracted. */
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bs_marg.h"
#include "bs_eigenf.h"

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "bs_linabsqr_dense.inc"

long bs_marg_stats[4] = {0, 0, 0, 0};   /* marginalizations; kfs removed by the feature-ratio criterion; kfs removed by the DSO score; zero-rank pivots (|beta| <= threshold) */
int bs_marg_oob = 0;
void (*bs_marg_dbg_hook)(const float* Q2Jp, int rows, int cols, const float* Q2r, const int* keep, int nkeep, const int* marg, int nmarg) = 0;   /* test hook: the helper's inputs */   /* counts calls where the C++ would read Q2Jp / Q2r out of bounds (marg_rank + keep_valid_rows > rows): undefined behaviour, never on the executed path */

static float mg_fold(const float* p, int n) {
    float s = p[0];
    int i;
    for (i = 1; i < n; ++i) s = s + p[i];
    return s;
}

static float mg_vec_sum(const float* p, int n) {      /* LinearVectorizedTraversal redux over an expression (alignedStart 0), SSE packet 4 */
    const int size2 = (n / 8) * 8, size1 = (n / 4) * 4;
    float r0[4], r1[4], res;
    int i, idx;
    if (size1 == 0) return mg_fold(p, n);
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

/* B (m rows x c cols, leading dimension ld) <- (I - tau [1 e]^T [1 e]) B;  e has m-1 entries;  tmp has c entries */
static void apply_householder_left(float* B, int ld, int m, int c, const float* e, float tau, float* tmp) {
    int i, j;
    if (m == 1) {
        for (j = 0; j < c; ++j) B[(size_t)j * ld] = B[(size_t)j * ld] * (1.0f - tau);
        return;
    }
    if (tau != 0.0f) {
        const int ne = m - 1;
        float* te = (float*)malloc(sizeof(float) * (size_t)ne);
        for (j = 0; j < c; ++j) tmp[j] = 0.0f;
        if (c == 1) {
            for (i = 0; i < ne; ++i) te[i] = e[i] * B[1 + i];
            tmp[0] = tmp[0] + 1.0f * mg_vec_sum(te, ne);                   /* dst += alpha * dot */
        } else if (c > 1) {
            la_gemv(c, ne, &B[1], ld, 1, e, 1, tmp, 1.0f);                   /* (B^T) e, RowMajor kernel on the transposed block */
        }
        for (j = 0; j < c; ++j) tmp[j] = tmp[j] + B[(size_t)j * ld];         /* tmp += row(0) */
        for (j = 0; j < c; ++j) B[(size_t)j * ld] = B[(size_t)j * ld] - tau * tmp[j];
        for (i = 0; i < ne; ++i) te[i] = tau * e[i];                          /* tau * essential, evaluated first */
        for (j = 0; j < c; ++j)
            for (i = 0; i < ne; ++i) B[1 + i + (size_t)j * ld] = B[1 + i + (size_t)j * ld] - tmp[j] * te[i];
        free(te);
    }
}

void bs_marg_helper_sqrt_to_sqrt(float* Q2Jp, int rows, int cols, float* Q2r, const int* keep, int nkeep, const int* marg, int nmarg,
                                 float** H_out, int* H_rows, float** b_out) {
    const int keep_size = nkeep, marg_size = nmarg;
    const float rank_threshold = sqrtf(FLT_EPSILON);
    float* perm = (float*)malloc(sizeof(float) * ((size_t)rows * cols > 0 ? (size_t)rows * cols : 1));
    float* temp = (float*)malloc(sizeof(float) * (size_t)(cols + 1));
    int k, j, i, total_rank = 0, marg_rank = 0, kvr;
    /* Q2Jp.applyOnTheRight(p): column j <- old column indices[j], indices = [marg..., keep...] */
    for (j = 0; j < marg_size + keep_size; ++j) {
        const int src = (j < marg_size) ? marg[j] : keep[j - marg_size];
        memcpy(&perm[(size_t)j * rows], &Q2Jp[(size_t)src * rows], sizeof(float) * (size_t)rows);
    }
    memcpy(Q2Jp, perm, sizeof(float) * (size_t)rows * cols);
    free(perm);

    for (k = 0; k < cols && total_rank < rows; ++k) {
        const int remainingRows = rows - total_rank, remainingCols = cols - k - 1;
        float* x = &Q2Jp[(size_t)k * rows + total_rank];                    /* Q2Jp.col(k).tail(remainingRows) */
        float tau, beta, c0 = x[0];
        const int ne = remainingRows - 1;
        float tailSqNorm = 0.0f;
        if (remainingRows != 1) {
            float* sq = (float*)malloc(sizeof(float) * (size_t)ne);
            for (i = 0; i < ne; ++i) sq[i] = x[1 + i] * x[1 + i];
            tailSqNorm = mg_vec_sum(sq, ne);
            free(sq);
        }
        if (tailSqNorm <= FLT_MIN) {
            tau = 0.0f; beta = c0;
            for (i = 0; i < ne; ++i) x[1 + i] = 0.0f;
        } else {
            beta = sqrtf(c0 * c0 + tailSqNorm);
            if (c0 >= 0.0f) beta = -beta;
            { const float den = c0 - beta; for (i = 0; i < ne; ++i) x[1 + i] = x[1 + i] / den; }
            tau = (beta - c0) / beta;
        }
        if (fabsf(beta) > rank_threshold) {
            Q2Jp[(size_t)k * rows + total_rank] = beta;
            apply_householder_left(&Q2Jp[(size_t)(k + 1) * rows + total_rank], rows, remainingRows, remainingCols, x + 1, tau, temp + k + 1);
            apply_householder_left(&Q2r[total_rank], remainingRows, remainingRows, 1, x + 1, tau, temp + cols);
            total_rank++;
        } else {
            Q2Jp[(size_t)k * rows + total_rank] = 0.0f;
            bs_marg_stats[3]++;
        }
        for (i = 0; i < ne; ++i) x[1 + i] = 0.0f;                          /* overwrite householder vectors with 0 */
        if (k == marg_size - 1) marg_rank = total_rank;
    }
    kvr = total_rank - marg_rank;
    if (kvr < 1) kvr = 1;
    if (marg_rank + kvr > rows) bs_marg_oob++;
    *H_rows = kvr;
    *H_out = (float*)malloc(sizeof(float) * (size_t)kvr * (keep_size > 0 ? keep_size : 1));
    *b_out = (float*)malloc(sizeof(float) * (size_t)kvr);
    for (j = 0; j < keep_size; ++j)
        for (i = 0; i < kvr; ++i) {
            const int r = marg_rank + i;
            (*H_out)[i + (size_t)j * kvr] = (r < rows) ? Q2Jp[r + (size_t)(marg_size + j) * rows] : 0.0f;   /* r >= rows reads out of bounds in the C++ (never on the path) */
        }
    for (i = 0; i < kvr; ++i) (*b_out)[i] = (marg_rank + i < rows) ? Q2r[marg_rank + i] : 0.0f;
    free(temp);
}

/* ------------------------------------------------------------------------------------------------ marginalize() */

static int cmp_i64(const void* a, const void* b) { const int64_t x = *(const int64_t*)a, y = *(const int64_t*)b; return (x > y) - (x < y); }

typedef struct i64set { int64_t* v; int n, cap; } i64set;
static int set_find(const i64set* s, int64_t t) {
    int lo = 0, hi = s->n;
    while (lo < hi) { int m = (lo + hi) / 2; if (s->v[m] < t) lo = m + 1; else hi = m; }
    return (lo < s->n && s->v[lo] == t) ? lo : -1;
}
static void set_emplace(i64set* s, int64_t t) {
    int lo = 0, hi = s->n;
    while (lo < hi) { int m = (lo + hi) / 2; if (s->v[m] < t) lo = m + 1; else hi = m; }
    if (lo < s->n && s->v[lo] == t) return;
    if (s->n + 1 > s->cap) { s->cap = s->cap ? 2 * s->cap : 16; s->v = (int64_t*)realloc(s->v, sizeof(int64_t) * (size_t)s->cap); }
    memmove(&s->v[lo + 1], &s->v[lo], sizeof(int64_t) * (size_t)(s->n - lo));
    s->v[lo] = t; s->n++;
}
static void set_erase(i64set* s, int64_t t) {
    const int i = set_find(s, t);
    if (i < 0) return;
    memmove(&s->v[i], &s->v[i + 1], sizeof(int64_t) * (size_t)(s->n - i - 1));
    s->n--;
}

static const bs_frame_pose* pose_at(const bs_vio* v, int64_t t) {
    int lo = 0, hi = v->ba.n_poses;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.poses[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < v->ba.n_poses && v->ba.poses[lo].t_ns == t) ? &v->ba.poses[lo] : NULL;
}
static bs_frame_state* state_at(bs_vio* v, int64_t t) {
    int lo = 0, hi = v->ba.n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.states[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < v->ba.n_states && v->ba.states[lo].t_ns == t) ? &v->ba.states[lo] : NULL;
}
static const float* pose_trans(const bs_frame_pose* p) { return p->linearized ? p->cur.t : p->lin.t; }
static const float* state_trans(const bs_frame_state* s) { return s->s.linearized ? s->s.cur.s.p : s->s.lin.s.p; }

static void erase_state(bs_vio* v, int64_t t) {
    bs_frame_state* s = state_at(v, t);
    if (!s) return;
    memmove(s, s + 1, sizeof(bs_frame_state) * (size_t)(v->ba.states + v->ba.n_states - (s + 1)));
    v->ba.n_states--;
}
static void erase_pose(bs_vio* v, int64_t t) {
    bs_frame_pose* p = (bs_frame_pose*)pose_at(v, t);
    if (!p) return;
    memmove(p, p + 1, sizeof(bs_frame_pose) * (size_t)(v->ba.poses + v->ba.n_poses - (p + 1)));
    v->ba.n_poses--;
}
static void erase_imu(bs_vio* v, int64_t t) {
    int lo = 0, hi = v->n_imu;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->imu[m].start_t_ns < t) lo = m + 1; else hi = m; }
    if (lo < v->n_imu && v->imu[lo].start_t_ns == t) {
        memmove(&v->imu[lo], &v->imu[lo + 1], sizeof(bs_imu_meas) * (size_t)(v->n_imu - lo - 1));
        v->n_imu--;
    }
}
static const bs_aom_item* find_item(const bs_aom_item* it, int n, int64_t t) {
    int lo = 0, hi = n;
    while (lo < hi) { int m = (lo + hi) / 2; if (it[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < n && it[lo].t_ns == t) ? &it[lo] : NULL;
}

void bs_marg_result_free(bs_marg_result* r) {
    free(r->aom); free(r->kf_ids_all); free(r->kfs_to_marg); free(r->poses_to_marg); free(r->states_to_marg_all); free(r->states_to_marg_vel_bias);
    free(r->idx_to_keep); free(r->idx_to_marg); free(r->b_new);
    memset(r, 0, sizeof(*r));
}

void bs_vio_marginalize(bs_vio* v, const bs_kf_count* npc, int n_conn, const int64_t* lost, int n_lost, bs_marg_result* res) {
    int i, j, states_to_remove, np, ns;
    int64_t last_state_to_marg;
    i64set poses_to_marg = {0, 0, 0}, st_vb = {0, 0, 0}, st_all = {0, 0, 0}, kfs_to_marg = {0, 0, 0};
    bs_aom_item* aom_items;
    int aom_n = 0, total = 0;
    bs_marg_result dummy;
    if (!res) res = &dummy;
    memset(res, 0, sizeof(*res));
    if (!v->opt_started) return;
    np = v->ba.n_poses; ns = v->ba.n_states;
    if (!((size_t)np > (size_t)v->max_kfs || (size_t)ns >= (size_t)v->max_states)) return;
    res->marginalized = 1;
    bs_marg_stats[0]++;

    states_to_remove = ns - v->max_states + 1;
    if (ns == 0) { res->layout_error = 1; return; }
    { int idx = 0; for (i = 0; i < states_to_remove; i++) idx++; if (idx >= ns) { res->layout_error = 1; return; } last_state_to_marg = v->ba.states[idx].t_ns; }
    res->last_state_to_marg = last_state_to_marg;

    aom_items = (bs_aom_item*)malloc(sizeof(bs_aom_item) * (size_t)(np + ns + 1));
    for (i = 0; i < np; ++i) {
        const int64_t t = v->ba.poses[i].t_ns;
        aom_items[aom_n].t_ns = t; aom_items[aom_n].start = total; aom_items[aom_n].size = 6; aom_n++;
        if (set_find(&(i64set){v->kf_ids, v->n_kf, v->n_kf}, t) < 0) set_emplace(&poses_to_marg, t);
        { const bs_aom_item* m = find_item(v->marg.item, v->marg.n, t); if (!m || m->start != total || m->size != 6) res->layout_error = 1; }
        total += 6;
    }
    for (i = 0; i < ns; ++i) {
        const int64_t t = v->ba.states[i].t_ns;
        if (t > last_state_to_marg) break;
        if (t != last_state_to_marg) {
            if (set_find(&(i64set){v->kf_ids, v->n_kf, v->n_kf}, t) >= 0) set_emplace(&st_vb, t); else set_emplace(&st_all, t);
        }
        aom_items[aom_n].t_ns = t; aom_items[aom_n].start = total; aom_items[aom_n].size = 15; aom_n++;
        if ((size_t)aom_n - 1 < (size_t)v->marg.n) {      /* aom.items < marg_data.order.abs_order_map.size() (items counted before the increment) */
            const bs_aom_item* m = find_item(v->marg.item, v->marg.n, t);
            if (!m || m->start != total || m->size != 15) res->layout_error = 1;
        }
        total += 15;
    }

    /* kf selection */
    {
        i64set kf = {(int64_t*)malloc(sizeof(int64_t) * (size_t)(v->n_kf + 1)), v->n_kf, v->n_kf + 1};
        res->kf_ids_all = (int64_t*)malloc(sizeof(int64_t) * (size_t)(v->n_kf + 1));
        memcpy(kf.v, v->kf_ids, sizeof(int64_t) * (size_t)v->n_kf);
        memcpy(res->kf_ids_all, v->kf_ids, sizeof(int64_t) * (size_t)v->n_kf);
        res->n_kf_all = v->n_kf;
        while ((size_t)kf.n > (size_t)v->max_kfs && st_vb.n > 0) {
            int64_t id_to_marg = -1;
            int found = 0;
            if (kf.n > 2) {
                const int end_minus_2 = kf.n - 2;
                for (i = 0; i < end_minus_2; ++i) {
                    int conn = 0, has = 0, kn = 0;
                    for (j = 0; j < n_conn; ++j) if (npc[j].t_ns == kf.v[i]) { has = 1; conn = npc[j].n; break; }
                    if (!has) { id_to_marg = kf.v[i]; found = 1; break; }
                    if (!bs_vio_npk_get(v, kf.v[i], &kn)) { res->layout_error = 1; found = 0; break; }
                    if ((double)((float)conn / (float)kn) < v->kf_marg_feature_ratio) { id_to_marg = kf.v[i]; found = 1; break; }
                }
                if (res->layout_error) break;
            }
            if (kf.n > 2 && found) bs_marg_stats[1]++;
            if (kf.n > 2 && !found) {
                const int end_minus_2 = kf.n - 2;
                const int64_t last_kf = kf.v[kf.n - 1];
                float min_score = FLT_MAX;
                int64_t min_score_id = -1;
                const bs_frame_state* lk = state_at(v, last_kf);
                if (!lk) { res->layout_error = 1; break; }
                for (i = 0; i < end_minus_2; ++i) {
                    float denom = 0.0f, score;
                    const bs_frame_pose* p1 = pose_at(v, kf.v[i]);
                    float d[3];
                    if (!p1) { res->layout_error = 1; break; }
                    for (j = 0; j < end_minus_2; ++j) {
                        const bs_frame_pose* p2 = pose_at(v, kf.v[j]);
                        if (!p2) { res->layout_error = 1; break; }
                        { const float *a = pose_trans(p1), *b = pose_trans(p2); d[0] = a[0] - b[0]; d[1] = a[1] - b[1]; d[2] = a[2] - b[2]; }
                        denom += 1 / (sqrtf(bs_v3f_sqn(d)) + (float)1e-5);
                    }
                    if (res->layout_error) break;
                    { const float *a = pose_trans(p1), *b = state_trans(lk); d[0] = a[0] - b[0]; d[1] = a[1] - b[1]; d[2] = a[2] - b[2]; }
                    score = sqrtf(sqrtf(bs_v3f_sqn(d))) * denom;
                    if (score < min_score) { min_score_id = kf.v[i]; min_score = score; }
                }
                if (res->layout_error) break;
                id_to_marg = min_score_id;
                bs_marg_stats[2]++;
            }
            if (id_to_marg < 0) { res->layout_error = 1; break; }    /* BASALT_ASSERT(id_to_marg >= 0) */
            set_emplace(&kfs_to_marg, id_to_marg);
            set_emplace(&poses_to_marg, id_to_marg);
            set_erase(&kf, id_to_marg);
        }
        /* kf_ids = kf */
        if (kf.n > v->cap_kf) { v->cap_kf = kf.n; v->kf_ids = (int64_t*)realloc(v->kf_ids, sizeof(int64_t) * (size_t)(v->cap_kf ? v->cap_kf : 1)); }
        memcpy(v->kf_ids, kf.v, sizeof(int64_t) * (size_t)kf.n);
        v->n_kf = kf.n;
        free(kf.v);
    }
    res->aom = aom_items; res->aom_n = aom_n; res->aom_total = total;
    res->kfs_to_marg = kfs_to_marg.v; res->n_kfs = kfs_to_marg.n;
    res->poses_to_marg = poses_to_marg.v; res->n_poses_to_marg = poses_to_marg.n;
    res->states_to_marg_all = st_all.v; res->n_states_all = st_all.n;
    res->states_to_marg_vel_bias = st_vb.v; res->n_states_vb = st_vb.n;
    if (res->layout_error) return;

    /* LandmarkBlockAbsDynamic::allocateLandmark asserts host_kf_id in aom for every landmark the linearization selects (host in kfs_to_marg, or lost) */
    {
        const bs_hnode* n;
        for (n = v->ba.lmdb.kpts.before_begin.next; n; n = n->next) {
            const bs_keypoint* k = (const bs_keypoint*)n->val;
            int sel = set_find(&kfs_to_marg, k->host.frame_id) >= 0;
            if (!sel) for (i = 0; i < n_lost; ++i) if (lost[i] == k->id) { sel = 1; break; }
            if (sel && !find_item(aom_items, aom_n, k->host.frame_id)) { res->layout_error = 1; return; }
        }
    }

    /* linearize + Q2Jp / Q2r */
    {
        bs_aom aom; bs_marg_lin mg; bs_imu_lin ild; bs_imu_meas** imup;
        bs_linabsqr* la;
        float *Q2Jp, *Q2r, *Hn, *bn, *delta, *tmp;
        int n_ild = 0, valid = 1, rows, nkeep = 0, nmarg = 0, Hn_rows, c;
        int* keep = (int*)malloc(sizeof(int) * (size_t)(total + 1));
        int* mrg = (int*)malloc(sizeof(int) * (size_t)(total + 1));
        aom.item = aom_items; aom.n = aom_n; aom.total_size = total;
        mg.order.item = v->marg.item; mg.order.n = v->marg.n; mg.order.total_size = v->marg.total; mg.rows = v->marg.rows; mg.cols = v->marg.cols; mg.H = v->marg.H; mg.b = v->marg.b;
        imup = (bs_imu_meas**)malloc(sizeof(bs_imu_meas*) * (size_t)(v->n_imu + 1));
        for (i = 0; i < v->n_imu; ++i) {
            const int64_t st = v->imu[i].start_t_ns, en = v->imu[i].start_t_ns + v->imu[i].delta.t_ns;
            if (!find_item(aom_items, aom_n, st) || !find_item(aom_items, aom_n, en)) continue;
            imup[n_ild++] = &v->imu[i];
        }
        ild.n = n_ild; ild.meas = imup;
        for (i = 0; i < 3; ++i) { ild.g[i] = v->g[i]; ild.gyro_bias_weight_sqrt[i] = v->gyro_bias_sqrt_weight[i]; ild.accel_bias_weight_sqrt[i] = v->accel_bias_sqrt_weight[i]; }
        la = bs_la_create(&v->ba, &aom, &mg, &ild, kfs_to_marg.v, kfs_to_marg.n, lost, n_lost);
        bs_la_linearize_problem(la, &valid);
        bs_la_perform_qr(la);
        rows = bs_la_dense_Q2_rows(la);
        res->q2_rows = rows;
        Q2Jp = (float*)malloc(sizeof(float) * ((size_t)rows * total > 0 ? (size_t)rows * total : 1));
        Q2r = (float*)malloc(sizeof(float) * (size_t)(rows > 0 ? rows : 1));
        bs_la_get_dense_Q2Jp_Q2r(la, Q2Jp, Q2r);
        bs_la_destroy(la);
        free(imup);

        /* idx_to_keep / idx_to_marg (std::set<int>: ascending) */
        for (i = 0; i < aom_n; ++i) {
            const bs_aom_item* it = &aom_items[i];
            const int64_t t = it->t_ns;
            if (it->size == 6) {
                if (set_find(&poses_to_marg, t) < 0) { for (c = 0; c < 6; ++c) keep[nkeep++] = it->start + c; }
                else { for (c = 0; c < 6; ++c) mrg[nmarg++] = it->start + c; }
            } else {
                if (set_find(&st_all, t) >= 0) { for (c = 0; c < 15; ++c) mrg[nmarg++] = it->start + c; }
                else if (set_find(&st_vb, t) >= 0) { for (c = 0; c < 6; ++c) keep[nkeep++] = it->start + c; for (c = 6; c < 15; ++c) mrg[nmarg++] = it->start + c; }
                else { if (t != last_state_to_marg) res->layout_error = 1; for (c = 0; c < 15; ++c) keep[nkeep++] = it->start + c; }
            }
        }
        res->idx_to_keep = keep; res->n_keep = nkeep; res->idx_to_marg = mrg; res->n_marg = nmarg;

        if (bs_marg_dbg_hook) bs_marg_dbg_hook(Q2Jp, rows, total, Q2r, keep, nkeep, mrg, nmarg);
        bs_marg_helper_sqrt_to_sqrt(Q2Jp, rows, total, Q2r, keep, nkeep, mrg, nmarg, &Hn, &Hn_rows, &bn);
        free(Q2Jp); free(Q2r);

        /* state updates */
        {
            bs_frame_state* ls = state_at(v, last_state_to_marg);
            if (!ls || ls->s.linearized) res->layout_error = 1;       /* BASALT_ASSERT(isLinearized() == false) */
            if (ls) { ls->s.linearized = 1; ls->s.cur = ls->s.lin; }  /* setLinTrue */
        }
        for (i = 0; i < st_all.n; ++i) erase_state(v, st_all.v[i]), erase_imu(v, st_all.v[i]);
        for (i = 0; i < st_vb.n; ++i) {
            const int64_t id = st_vb.v[i];
            bs_frame_state* s = state_at(v, id);
            bs_frame_pose pose;
            if (!s) { res->layout_error = 1; continue; }
            bs_pose_from_state(s, &pose);
            { bs_frame_pose* slot = bs_vio_pose_insert(v, id); if (!slot) slot = (bs_frame_pose*)pose_at(v, id); *slot = pose; }
            erase_state(v, id); erase_imu(v, id);
        }
        for (i = 0; i < poses_to_marg.n; ++i) erase_pose(v, poses_to_marg.v[i]);
        bs_lmdb_remove_keyframes(&v->ba.lmdb, kfs_to_marg.v, kfs_to_marg.n, poses_to_marg.v, poses_to_marg.n, st_all.v, st_all.n);
        for (i = 0; i < n_lost; ++i) bs_lmdb_remove_landmark(&v->ba.lmdb, lost[i]);

        /* new prior order: remaining poses, then last_state_to_marg */
        {
            bs_aom_item* ni = (bs_aom_item*)malloc(sizeof(bs_aom_item) * (size_t)(v->ba.n_poses + 1));
            int nt = 0, nn = 0;
            for (i = 0; i < v->ba.n_poses; ++i) { ni[nn].t_ns = v->ba.poses[i].t_ns; ni[nn].start = nt; ni[nn].size = 6; nt += 6; nn++; }
            ni[nn].t_ns = last_state_to_marg; ni[nn].start = nt; ni[nn].size = 15; nt += 15; nn++;
            /* abs_order_map is a std::map keyed by time */
            for (i = 1; i < nn; ++i) { bs_aom_item x = ni[i]; j = i - 1; while (j >= 0 && ni[j].t_ns > x.t_ns) { ni[j + 1] = ni[j]; --j; } ni[j + 1] = x; }
            free(v->marg.item); free(v->marg.H); free(v->marg.b);
            v->marg.item = ni; v->marg.n = nn; v->marg.total = nt; v->marg.rows = Hn_rows; v->marg.cols = nkeep; v->marg.H = Hn; v->marg.b = bn;
            if (nkeep != nt) res->layout_error = 1;                   /* BASALT_ASSERT(size_t(marg_data.H.cols()) == marg_data.order.total_size) */
        }
        res->b_new = (float*)malloc(sizeof(float) * (size_t)(v->marg.rows + 1)); res->n_b_new = v->marg.rows;
        if (v->marg.rows) memcpy(res->b_new, v->marg.b, sizeof(float) * (size_t)v->marg.rows);
        /* marg_data.b -= marg_data.H * delta */
        {
            bs_aom ord; ord.item = v->marg.item; ord.n = v->marg.n; ord.total_size = v->marg.total;
            delta = (float*)malloc(sizeof(float) * (size_t)(v->marg.total + 1));
            tmp = (float*)malloc(sizeof(float) * (size_t)(v->marg.rows + 1));
            for (i = 0; i < v->ba.n_poses; ++i) if (!v->ba.poses[i].linearized) res->layout_error = 1;   /* BASALT_ASSERT(frame_poses.at(t).isLinearized()) in computeDelta */
            bs_ba_compute_delta(&v->ba, &ord, delta);
            for (i = 0; i < v->marg.rows; ++i) tmp[i] = 0.0f;
            la_gemv(v->marg.rows, v->marg.cols, v->marg.H, 1, v->marg.rows, delta, 1, tmp, 1.0f);
            for (i = 0; i < v->marg.rows; ++i) v->marg.b[i] = v->marg.b[i] - tmp[i];
            free(delta); free(tmp);
        }
    }
}
