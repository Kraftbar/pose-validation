/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * LinearizationAbsQR<float, 6> and the estimator-side error / marginalisation-prior evaluation, see bs_linabsqr.h.
 * Port of linearization_abs_qr.cpp, landmark_block_abs_dynamic.hpp (performQRHouseholder, backSubstitute), imu_block.hpp, ba_base.cpp,
 * ba_utils.h linearizePoint, stereographic_param.hpp (unproject), sc_ba_base.cpp computeImuError. Every expression keeps the C++ association;
 * the Eigen evaluation orders applied (all measured against the real classes by basalt_port/reference_tools/bs_linabsqr_test.cc):
 *   - fixed small products (rows < 4: no packets): every coefficient is a binary tree of the products, (x0+x1)+(x2+x3), x0+(x1+x2);
 *   - Matrix4f * Vector4f: packet over the 4 rows, left fold over the columns;
 *   - dynamic float GEMM (Eigen GEBP, mr 8, nr 4, with the cache-size dependent blocking) and GEMV (both kernels): bs_linabsqr_dense.inc;
 *   - dynamic redux of strided data: plain left fold; of direct aligned data: two-packet linear vectorised redux (vec_sum);
 *   - Householder: makeHouseholder / applyHouseholderOnTheLeft as Eigen 3.4.0 (essential = tail / (c0 - beta), tmp = e^T * bottom via GEMV,
 *     rank-1 update row by row);
 *   - TBB parallel_reduce / parallel_for at parallelism 1: serial index order, each landmark block's value added to the running sum. */
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bs_linabsqr.h"
#include "bs_linabsqr_dense.inc"

/* Statement forms that Eigen evaluates through the coefficient-based product for tiny sizes (rows + dst.rows + dst.cols < 20, e.g. a marginalisation
 * prior with 1..few rows) are not modelled; they are counted here and computed with the GEBP model (never reached on the executed path: the prior
 * has >= 15 columns). */
int bs_la_unsupported = 0;
const long bs_la_cache_sizes[3] = {BS_L1, BS_L2, BS_L3};   /* what the GEMM blocking model assumes (bs_linabsqr_dense.inc) */

/* ------------------------------------------------------------------------------------------------ small numeric helpers */

static float tree_sum(const float* p, int n) {            /* Eigen redux_novec_unroller: sum(0..n/2) + sum(n/2..n) */
    int h;
    if (n == 1) return p[0];
    h = n / 2;
    return tree_sum(p, h) + tree_sum(p + h, n - h);
}

static float fold_sum(const float* p, int n) {            /* DefaultTraversal, NoUnrolling: res = c0; res = res + c_i */
    float s = p[0];
    int i;
    for (i = 1; i < n; ++i) s = s + p[i];
    return s;
}

/* LinearVectorizedTraversal redux, alignedStart = 0, dynamic or fixed size n >= 1 of float terms (SSE: packet 4) */
static float vec_sum(const float* p, int n) {
    const int size2 = (n / 8) * 8, size1 = (n / 4) * 4;
    float r0[4], r1[4], res;
    int i, idx;
    if (size1 == 0) return fold_sum(p, n);
    for (i = 0; i < 4; ++i) r0[i] = p[i];
    if (size1 > 4) {
        for (i = 0; i < 4; ++i) r1[i] = p[4 + i];
        for (idx = 8; idx < size2; idx += 8)
            for (i = 0; i < 4; ++i) { r0[i] = r0[i] + p[idx + i]; r1[i] = r1[i] + p[idx + 4 + i]; }
        for (i = 0; i < 4; ++i) r0[i] = r0[i] + r1[i];
        if (size1 > size2)
            for (i = 0; i < 4; ++i) r0[i] = r0[i] + p[size2 + i];
    }
    res = (r0[0] + r0[2]) + (r0[1] + r0[3]);              /* predux(Packet4f) */
    for (idx = size1; idx < n; ++idx) res = res + p[idx];
    return res;
}

static int is_finite_f(float x) { return isfinite(x) != 0; }

/* ------------------------------------------------------------------------------------------------ stereographic parametrisation */

void bs_stereo_unproject_f(const float proj[2], float res[4], float* d_r_d_p) {
    const float x2 = proj[0] * proj[0];
    const float y2 = proj[1] * proj[1];
    const float r2 = x2 + y2;
    const float norm_inv = 2.0f / (1.0f + r2);
    res[0] = proj[0] * norm_inv;
    res[1] = proj[1] * norm_inv;
    res[2] = norm_inv - 1.0f;
    res[3] = 0.0f;
    if (d_r_d_p) {
        const float norm_inv2 = norm_inv * norm_inv;
        const float xy = proj[0] * proj[1];
        d_r_d_p[0 + 4 * 0] = (norm_inv - x2 * norm_inv2);
        d_r_d_p[0 + 4 * 1] = -xy * norm_inv2;
        d_r_d_p[1 + 4 * 0] = -xy * norm_inv2;
        d_r_d_p[1 + 4 * 1] = (norm_inv - y2 * norm_inv2);
        d_r_d_p[2 + 4 * 0] = -proj[0] * norm_inv2;
        d_r_d_p[2 + 4 * 1] = -proj[1] * norm_inv2;
        d_r_d_p[3 + 4 * 0] = 0.0f;
        d_r_d_p[3 + 4 * 1] = 0.0f;
    }
}

/* ------------------------------------------------------------------------------------------------ linearizePoint (ba_utils.h) */

int bs_la_linearize_point(const float obs[2], const bs_keypoint* kp, const float T[16], const bs_ds_f* cam, float res[2], float* d_res_d_xi,
                          float* d_res_d_p) {
    float Jup[8], p_h[4], p_t[4], Jp[8];
    int valid, i, r, c, k;
    bs_stereo_unproject_f(kp->direction, p_h, Jup);
    p_h[3] = kp->inv_dist;
    for (i = 0; i < 4; ++i) p_t[i] = ((T[i] * p_h[0] + T[i + 4] * p_h[1]) + T[i + 8] * p_h[2]) + T[i + 12] * p_h[3];   /* Mat4 * Vec4: packet, left fold */
    valid = bs_ds_project_f(cam, p_t, res, Jp, NULL);
    valid &= (is_finite_f(res[0]) && is_finite_f(res[1]));
    if (!valid) return 0;
    res[0] = res[0] - obs[0];
    res[1] = res[1] - obs[1];
    if (d_res_d_xi) {
        float dp[24], h[9], t[4];
        const float ex[3] = {p_t[0], p_t[1], p_t[2]};
        h[0] = 0.0f;    h[3] = -ex[2];  h[6] = ex[1];       /* SO3::hat, column-major */
        h[1] = ex[2];   h[4] = 0.0f;    h[7] = -ex[0];
        h[2] = -ex[1];  h[5] = ex[0];   h[8] = 0.0f;
        for (c = 0; c < 6; ++c) for (r = 0; r < 4; ++r) dp[r + 4 * c] = 0.0f;
        for (c = 0; c < 3; ++c)
            for (r = 0; r < 3; ++r) {
                dp[r + 4 * c] = (r == c ? 1.0f : 0.0f) * kp->inv_dist;   /* Identity * inv_dist */
                dp[r + 4 * (3 + c)] = -h[r + 3 * c];                      /* -hat(p_t) */
            }
        for (c = 0; c < 6; ++c)
            for (r = 0; r < 2; ++r) {
                for (k = 0; k < 4; ++k) t[k] = Jp[r + 2 * k] * dp[k + 4 * c];
                d_res_d_xi[r + 2 * c] = tree_sum(t, 4);
            }
    }
    if (d_res_d_p) {
        float Jpp[12], t[4];
        for (i = 0; i < 12; ++i) Jpp[i] = 0.0f;
        for (c = 0; c < 2; ++c)
            for (r = 0; r < 3; ++r) {
                for (k = 0; k < 4; ++k) t[k] = T[r + 4 * k] * Jup[k + 4 * c];   /* T_t_h.topLeftCorner<3,4>() * Jup */
                Jpp[r + 4 * c] = tree_sum(t, 4);
            }
        for (i = 0; i < 4; ++i) Jpp[i + 8] = T[i + 12];                         /* Jpp.col(2) = T_t_h.col(3) */
        for (c = 0; c < 3; ++c)
            for (r = 0; r < 2; ++r) {
                for (k = 0; k < 4; ++k) t[k] = Jp[r + 2 * k] * Jpp[k + 4 * c];
                d_res_d_p[r + 2 * c] = tree_sum(t, 4);
            }
    }
    return 1;
}

/* ------------------------------------------------------------------------------------------------ state lookups */

typedef struct pose_view { int linearized; bs_se3f lin; bs_se3f cur; } pose_view;   /* PoseStateWithLin as getPoseStateWithLin returns it */

static const bs_se3f* pv_pose(const pose_view* p) { return p->linearized ? &p->cur : &p->lin; }   /* getPose() */

static const bs_aom_item* aom_find(const bs_aom* a, int64_t t) {
    int lo = 0, hi = a->n;
    while (lo < hi) { int m = (lo + hi) / 2; if (a->item[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < a->n && a->item[lo].t_ns == t) ? &a->item[lo] : NULL;
}

static int get_pose_view(const bs_ba* ba, int64_t t, pose_view* out) {
    int lo = 0, hi = ba->n_poses;
    while (lo < hi) { int m = (lo + hi) / 2; if (ba->poses[m].t_ns < t) lo = m + 1; else hi = m; }
    if (lo < ba->n_poses && ba->poses[lo].t_ns == t) {
        out->linearized = ba->poses[lo].linearized;
        out->lin = ba->poses[lo].lin;
        out->cur = ba->poses[lo].cur;
        return 1;
    }
    lo = 0; hi = ba->n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (ba->states[m].t_ns < t) lo = m + 1; else hi = m; }
    if (lo < ba->n_states && ba->states[lo].t_ns == t) {          /* PoseStateWithLin(const PoseVelBiasStateWithLin&) */
        const bs_frame_state* s = &ba->states[lo];
        float d6[6];
        int i;
        for (i = 0; i < 6; ++i) d6[i] = s->delta[i];
        out->linearized = s->s.linearized;
        out->lin = (bs_se3f){{s->s.lin.s.q[0], s->s.lin.s.q[1], s->s.lin.s.q[2], s->s.lin.s.q[3]}, {s->s.lin.s.p[0], s->s.lin.s.p[1], s->s.lin.s.p[2]}};
        out->cur = out->lin;
        bs_inc_posef(d6, &out->cur);
        return 1;
    }
    return 0;
}

static const bs_frame_state* find_state(const bs_ba* ba, int64_t t) {
    int lo = 0, hi = ba->n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (ba->states[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < ba->n_states && ba->states[lo].t_ns == t) ? &ba->states[lo] : NULL;
}

void bs_ba_init(bs_ba* ba) { memset(ba, 0, sizeof(*ba)); bs_lmdb_init(&ba->lmdb); }
void bs_ba_destroy(bs_ba* ba) { bs_lmdb_destroy(&ba->lmdb); }

/* ------------------------------------------------------------------------------------------------ BundleAdjustmentBase evaluation */

static const bs_obs* kp_obs_at(const bs_keypoint* k, bs_tcid t) {
    int lo = 0, hi = k->nobs;
    while (lo < hi) { int m = (lo + hi) / 2; if (bs_tcid_cmp(k->obs[m].t, t) < 0) lo = m + 1; else hi = m; }
    return (lo < k->nobs && bs_tcid_cmp(k->obs[lo].t, t) == 0) ? &k->obs[lo] : NULL;
}

float bs_ba_compute_error(const bs_ba* ba) {
    float error = 0.0f;
    const bs_hnode* hn;
    for (hn = ba->lmdb.observations.before_begin.next; hn; hn = hn->next) {
        const bs_host* h = (const bs_host*)hn->val;
        const bs_tcid tcid_h = {hn->k0, (uint64_t)hn->k1};
        int ti;
        for (ti = 0; ti < h->n; ++ti) {
            const bs_tcid tcid_t = h->tgt[ti].t;
            float T[16];
            int ii;
            if (bs_tcid_cmp(tcid_h, tcid_t) != 0) {
                pose_view sh, st;
                bs_se3f rel;
                get_pose_view(ba, tcid_h.frame_id, &sh);
                get_pose_view(ba, tcid_t.frame_id, &st);
                bs_compute_rel_posef(pv_pose(&sh), &ba->T_i_c[tcid_h.cam_id], pv_pose(&st), &ba->T_i_c[tcid_t.cam_id], NULL, NULL, &rel);
                bs_se3f_matrix(&rel, T);
            } else {
                int i;
                for (i = 0; i < 16; ++i) T[i] = ((i % 5) == 0) ? 1.0f : 0.0f;
            }
            for (ii = 0; ii < h->tgt[ti].n; ++ii) {
                const bs_keypoint* kp = bs_lmdb_get_landmark(&ba->lmdb, h->tgt[ti].ids[ii]);
                const bs_obs* o = kp_obs_at(kp, tcid_t);
                float res[2];
                if (bs_la_linearize_point(o->pos, kp, T, &ba->cam[tcid_t.cam_id], res, NULL, NULL)) {
                    const float e = sqrtf(res[0] * res[0] + res[1] * res[1]);
                    const float huber_weight = e < ba->huber_thresh ? 1.0f : ba->huber_thresh / e;
                    const float obs_weight = huber_weight / (ba->obs_std_dev * ba->obs_std_dev);
                    const float c = (0.5f * (2.0f - huber_weight)) * obs_weight;
                    const float t0 = (c * res[0]) * res[0], t1 = (c * res[1]) * res[1];
                    error = error + (t0 + t1);
                }
            }
        }
    }
    return error;
}

void bs_ba_compute_delta(const bs_ba* ba, const bs_aom* order, float* delta) {
    int i, j;
    for (i = 0; i < order->total_size; ++i) delta[i] = 0.0f;
    for (i = 0; i < order->n; ++i) {
        const bs_aom_item* it = &order->item[i];
        if (it->size == 6) {
            int lo = 0, hi = ba->n_poses;
            while (lo < hi) { int m = (lo + hi) / 2; if (ba->poses[m].t_ns < it->t_ns) lo = m + 1; else hi = m; }
            for (j = 0; j < 6; ++j) delta[it->start + j] = ba->poses[lo].delta[j];
        } else {
            const bs_frame_state* s = find_state(ba, it->t_ns);
            for (j = 0; j < 15; ++j) delta[it->start + j] = s->delta[j];
        }
    }
}

/* y = H * x (H rows x cols col-major) evaluated as a temporary: dst.setZero() + GEMV, scaled by alpha */
static void mat_vec(const float* H, int rows, int cols, const float* x, float* y, float alpha) {
    int i;
    for (i = 0; i < rows; ++i) y[i] = 0.0f;
    la_gemv(rows, cols, H, 1, rows, x, 1, y, alpha);
}

float bs_ba_marg_prior_error(const bs_ba* ba, const bs_marg_lin* mld) {
    const int rows = mld->rows, cols = mld->cols;
    float* delta = (float*)malloc(sizeof(float) * (cols > 0 ? cols : 1));
    float* z = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float* u = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float* t = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float out;
    int i;
    bs_ba_compute_delta(ba, &mld->order, delta);
    /* delta^T * H^T * (0.5 * H * delta + b) = ((delta^T H^T) * (0.5*H*delta + b)):  z = GEMV, u = 0.5*(H delta) + b, inner product */
    mat_vec(mld->H, rows, cols, delta, z, 1.0f);
    mat_vec(mld->H, rows, cols, delta, u, 0.5f);
    for (i = 0; i < rows; ++i) { u[i] = u[i] + mld->b[i]; t[i] = z[i] * u[i]; }
    out = vec_sum(t, rows);
    free(delta); free(z); free(u); free(t);
    return out;
}

float bs_ba_marg_prior_model_cost_change(const bs_ba* ba, const bs_marg_lin* mld, const float* marg_pose_inc) {
    const int rows = mld->rows, cols = mld->cols;
    float* delta = (float*)malloc(sizeof(float) * (cols > 0 ? cols : 1));
    float* bj = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float* jinc = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float* t = (float*)malloc(sizeof(float) * (rows > 0 ? rows : 1));
    float out;
    int i;
    bs_ba_compute_delta(ba, &mld->order, delta);
    mat_vec(mld->H, rows, cols, delta, bj, 1.0f);                                  /* H * delta + b */
    for (i = 0; i < rows; ++i) bj[i] = bj[i] + mld->b[i];
    mat_vec(mld->H, rows, cols, marg_pose_inc, jinc, 1.0f);                        /* J_inc = H * J_inc */
    for (i = 0; i < rows; ++i) t[i] = (-jinc[i]) * (bj[i] + 0.5f * jinc[i]);       /* -J_inc^T * (b_Jdelta + 0.5 J_inc) */
    out = vec_sum(t, rows);
    free(delta); free(bj); free(jinc); free(t);
    return out;
}

void bs_ba_compute_imu_error(const bs_ba* ba, const bs_aom* aom, bs_imu_meas* const* meas, int n_meas, const float g[3],
                             const float gyro_bias_weight[3], const float accel_bias_weight[3], float* imu_error, float* bg_error, float* ba_error) {
    int mi;
    *imu_error = 0.0f; *bg_error = 0.0f; *ba_error = 0.0f;
    for (mi = 0; mi < n_meas; ++mi) {
        bs_imu_meas* m = meas[mi];
        int64_t dt_ns = m->delta.t_ns;
        if (dt_ns != 0) {
            const int64_t start_t = m->start_t_ns, end_t = m->start_t_ns + dt_ns;
            const bs_frame_state *ss, *es;
            const bs_pvbstate *st, *en;
            float res[9], Cinv[81], z[9], tt[9], dt, gw[3], aw[3], rb[3], q[3], term;
            const float* S;
            int i, j;
            if (!aom_find(aom, start_t) || !aom_find(aom, end_t)) continue;
            ss = find_state(ba, start_t);
            es = find_state(ba, end_t);
            st = ss->s.linearized ? &ss->s.cur : &ss->s.lin;
            en = es->s.linearized ? &es->s.cur : &es->s.lin;
            bs_imu_residual(m, &st->s, g, &en->s, st->bg, st->ba, res, NULL, NULL, NULL, NULL);
            S = bs_imu_sqrt_cov_inv(m);
            for (i = 0; i < 81; ++i) Cinv[i] = 0.0f;
            la_gemm(9, 9, 9, S, 9, 1, S, 1, 9, Cinv, 9);                           /* get_cov_inv() = S^T S */
            for (i = 0; i < 9; ++i) z[i] = 0.0f;
            la_gemv(9, 9, Cinv, 9, 1, res, 1, z, 0.5f);                            /* (0.5 * res^T) * cov_inv : row-major GEMV */
            for (i = 0; i < 9; ++i) tt[i] = z[i] * res[i];
            *imu_error = *imu_error + vec_sum(tt, 9);
            dt = (float)dt_ns * (float)1e-9;
            for (i = 0; i < 3; ++i) gw[i] = gyro_bias_weight[i] / dt;
            for (i = 0; i < 3; ++i) rb[i] = st->bg[i] - en->bg[i];
            for (j = 0; j < 3; ++j) q[j] = ((0.5f * rb[j]) * gw[j]) * rb[j];
            term = tree_sum(q, 3);
            *bg_error = *bg_error + term;
            for (i = 0; i < 3; ++i) aw[i] = accel_bias_weight[i] / dt;
            for (i = 0; i < 3; ++i) rb[i] = st->ba[i] - en->ba[i];
            for (j = 0; j < 3; ++j) q[j] = ((0.5f * rb[j]) * aw[j]) * rb[j];
            term = tree_sum(q, 3);
            *ba_error = *ba_error + term;
        }
    }
}

/* ------------------------------------------------------------------------------------------------ landmark block */

typedef struct rel_pose_lin { float T_t_h[16]; float d_rel_d_h[36]; float d_rel_d_t[36]; } rel_pose_lin;

enum { ST_UNINIT = 0, ST_ALLOCATED, ST_NUMFAIL, ST_LINEARIZED, ST_MARGINALIZED };

typedef struct lblock {
    bs_keypoint* lm;
    int nobs;
    const rel_pose_lin** pl;      /* pose_lin_vec (NULL = observation dropped for marginalisation) */
    int num_rows, num_cols, padding_idx, padding_size, lm_idx, res_idx;
    float* st;                    /* row-major storage [Jp | pad | Jl | res] */
    int state;
} lblock;

struct bs_linabsqr {
    bs_ba* ba;
    const bs_aom* aom;
    const bs_marg_lin* marg;
    const bs_imu_lin* imu;
    float huber, obs_std;
    /* relative pose lin: one entry per (host, target) pair of lmdb.observations, in iteration order */
    int n_hosts;
    const bs_hnode** host_node;
    int* host_base;
    rel_pose_lin* rel;
    bs_htab host_to_idx;          /* val = (void*)(size_t)(host index + 1) */
    int nlm;
    int64_t* landmark_ids;
    lblock* blocks;
    int* block_row;               /* landmark_block_idx */
    int num_rows_Q2r;
    int n_imu;
    float (*imu_Jp)[450];
    float (*imu_r)[15];
};

static int tgt_index(const bs_host* h, bs_tcid t) {
    int lo = 0, hi = h->n;
    while (lo < hi) { int m = (lo + hi) / 2; if (bs_tcid_cmp(h->tgt[m].t, t) < 0) lo = m + 1; else hi = m; }
    return lo;
}

static const rel_pose_lin* rel_lookup(const bs_linabsqr* la, bs_tcid host, bs_tcid target) {
    bs_hnode* hn = bs_htab_find(&la->host_to_idx, host.frame_id, (int64_t)host.cam_id);
    int hi, ti;
    if (!hn) return NULL;
    hi = (int)(size_t)hn->val - 1;
    ti = tgt_index((const bs_host*)la->host_node[hi]->val, target);
    return &la->rel[la->host_base[hi] + ti];
}

static void lb_allocate(const bs_linabsqr* la, lblock* b, bs_keypoint* lm) {
    int i, pad;
    b->lm = lm;
    b->nobs = lm->nobs;
    b->pl = (const rel_pose_lin**)malloc(sizeof(rel_pose_lin*) * (lm->nobs > 0 ? lm->nobs : 1));
    for (i = 0; i < lm->nobs; ++i) {
        const rel_pose_lin* it = rel_lookup(la, lm->host, lm->obs[i].t);
        if (aom_find(la->aom, lm->obs[i].t.frame_id)) b->pl[i] = it;
        else b->pl[i] = NULL;                                           /* observation dropped for marginalisation */
    }
    b->padding_idx = la->aom->total_size;
    b->num_rows = lm->nobs * 2 + 3;
    b->padding_size = 0;
    pad = b->padding_idx % 4;
    if (pad != 0) b->padding_size = 4 - pad;
    b->lm_idx = b->padding_idx + b->padding_size;
    b->res_idx = b->lm_idx + 3;
    b->num_cols = b->res_idx + 1;
    b->st = (float*)calloc((size_t)b->num_rows * (size_t)b->num_cols, sizeof(float));
    b->state = ST_ALLOCATED;
}

static void lb_compute_error_weight(const bs_linabsqr* la, float res_squared, float* error, float* weight) {
    if (la->huber > 0) {
        const float hw = res_squared <= la->huber * la->huber ? 1.0f : la->huber / sqrtf(res_squared);
        *error = ((0.5f * (2.0f - hw)) * hw) * res_squared;
        *weight = hw;
    } else {
        *error = 0.5f * res_squared;
        *weight = 1.0f;
    }
}

static float lb_linearize(const bs_linabsqr* la, lblock* b) {
    const int nc = b->num_cols;
    int numerically_valid = 1, i, k;
    float error_sum = 0.0f;
    const bs_aom_item* host_it = aom_find(la->aom, b->lm->host.frame_id);
    memset(b->st, 0, sizeof(float) * (size_t)b->num_rows * (size_t)nc);
    for (i = 0; i < b->nobs; ++i) {
        if (b->pl[i]) {
            const int obs_idx = i * 2;
            const int abs_h_idx = host_it->start;
            const int abs_t_idx = aom_find(la->aom, b->lm->obs[i].t.frame_id)->start;
            float res[2], d_res_d_xi[12], d_res_d_p[6];
            const bs_tcid tt = b->lm->obs[i].t;
            const int valid = bs_la_linearize_point(b->lm->obs[i].pos, b->lm, b->pl[i]->T_t_h, &la->ba->cam[tt.cam_id], res, d_res_d_xi, d_res_d_p);
            if (valid) {                                                   /* use_valid_projections_only */
                float res_squared, we, w, sqrt_weight, tmp[12];
                int r, c;
                for (k = 0; k < 12; ++k) if (!is_finite_f(d_res_d_xi[k])) numerically_valid = 0;
                for (k = 0; k < 6; ++k) if (!is_finite_f(d_res_d_p[k])) numerically_valid = 0;
                res_squared = res[0] * res[0] + res[1] * res[1];
                lb_compute_error_weight(la, res_squared, &we, &w);
                sqrt_weight = sqrtf(w) / la->obs_std;
                error_sum = error_sum + we / (la->obs_std * la->obs_std);
                for (c = 0; c < 3; ++c) for (r = 0; r < 2; ++r) b->st[(obs_idx + r) * nc + b->lm_idx + c] = sqrt_weight * d_res_d_p[r + 2 * c];
                for (r = 0; r < 2; ++r) b->st[(obs_idx + r) * nc + b->res_idx] = sqrt_weight * res[r];
                for (k = 0; k < 12; ++k) d_res_d_xi[k] = d_res_d_xi[k] * sqrt_weight;
                for (c = 0; c < 6; ++c)                                    /* block<2,6>(obs_idx, abs_h_idx) += d_res_d_xi * d_rel_d_h */
                    for (r = 0; r < 2; ++r) {
                        float t[6];
                        for (k = 0; k < 6; ++k) t[k] = d_res_d_xi[r + 2 * k] * b->pl[i]->d_rel_d_h[k + 6 * c];
                        tmp[r + 2 * c] = tree_sum(t, 6);
                    }
                for (c = 0; c < 6; ++c) for (r = 0; r < 2; ++r) b->st[(obs_idx + r) * nc + abs_h_idx + c] = b->st[(obs_idx + r) * nc + abs_h_idx + c] + tmp[r + 2 * c];
                for (c = 0; c < 6; ++c)
                    for (r = 0; r < 2; ++r) {
                        float t[6];
                        for (k = 0; k < 6; ++k) t[k] = d_res_d_xi[r + 2 * k] * b->pl[i]->d_rel_d_t[k + 6 * c];
                        tmp[r + 2 * c] = tree_sum(t, 6);
                    }
                for (c = 0; c < 6; ++c) for (r = 0; r < 2; ++r) b->st[(obs_idx + r) * nc + abs_t_idx + c] = b->st[(obs_idx + r) * nc + abs_t_idx + c] + tmp[r + 2 * c];
            }
        }
    }
    b->state = numerically_valid ? ST_LINEARIZED : ST_NUMFAIL;
    return error_sum;
}

/* performQRHouseholder */
static void lb_perform_qr(lblock* b) {
    const int nc = b->num_cols;
    float* ess = (float*)malloc(sizeof(float) * (size_t)b->num_rows);
    float* tmp = (float*)malloc(sizeof(float) * (size_t)nc);
    int k, i, j;
    for (k = 0; k < 3; ++k) {
        const int rem = b->num_rows - k - 3;
        float tau, beta, c0, tail_sq;
        const float* col = &b->st[(size_t)k * nc + b->lm_idx + k];   /* storage.col(lm_idx + k).segment(k, rem), stride nc */
        float* top = &b->st[(size_t)k * nc];
        if (rem == 1) {
            tail_sq = 0.0f;
        } else {
            tail_sq = col[nc] * col[nc];                            /* tail.squaredNorm(): strided, left fold */
            for (i = 2; i < rem; ++i) tail_sq = tail_sq + col[(size_t)i * nc] * col[(size_t)i * nc];
        }
        c0 = col[0];
        if (tail_sq <= FLT_MIN) {                                   /* abs2(imag(c0)) = 0 <= tol */
            tau = 0.0f; beta = c0;
            for (i = 0; i < rem - 1; ++i) ess[i] = 0.0f;
        } else {
            const float denom_beta = sqrtf(c0 * c0 + tail_sq);
            beta = (c0 >= 0.0f) ? -denom_beta : denom_beta;
            for (i = 0; i < rem - 1; ++i) ess[i] = col[(size_t)(i + 1) * nc] / (c0 - beta);
            tau = (beta - c0) / beta;
        }
        /* storage.block(k, 0, rem, num_cols).applyHouseholderOnTheLeft(ess, tau, workspace) */
        if (rem == 1) {
            for (j = 0; j < nc; ++j) top[j] = top[j] * (1.0f - tau);
        } else if (tau != 0.0f) {
            float* bottom = top + nc;
            const int nb = rem - 1;
            for (j = 0; j < nc; ++j) tmp[j] = 0.0f;
            la_gemv(nc, nb, bottom, 1, nc, ess, 1, tmp, 1.0f);      /* tmp.noalias() = essential^T * bottom */
            for (j = 0; j < nc; ++j) tmp[j] = tmp[j] + top[j];      /* tmp += row(0) */
            for (j = 0; j < nc; ++j) top[j] = top[j] - tau * tmp[j];/* row(0) -= tau * tmp */
            for (i = 0; i < nb; ++i) {                              /* bottom.noalias() -= (tau * essential) * tmp, row by row */
                const float te = tau * ess[i];
                float* row = bottom + (size_t)i * nc;
                for (j = 0; j < nc; ++j) row[j] = row[j] - te * tmp[j];
            }
        }
    }
    free(ess); free(tmp);
    b->state = ST_MARGINALIZED;
}

/* inc = -Q1Jl.solve(Q1Jr + Q1Jp * pose_inc): triangular_solver_unroller (Upper, 3x3), the right-hand side from a row-major GEMV */
static void lb_solve_inc(const lblock* b, const float* pose_inc, float inc[3]) {
    const int nc = b->num_cols, P = b->padding_idx;
    float y[3] = {0.0f, 0.0f, 0.0f}, rhs[3], x[3];
    int i;
#define U(i_, j_) b->st[(size_t)(i_) * nc + b->lm_idx + (j_)]
    la_gemv(3, P, b->st, nc, 1, pose_inc, 1, y, 1.0f);
    for (i = 0; i < 3; ++i) rhs[i] = b->st[(size_t)i * nc + b->res_idx] + y[i];
    x[2] = rhs[2] / U(2, 2);
    x[1] = rhs[1] - (U(1, 2) * x[2]);
    x[1] = x[1] / U(1, 1);
    x[0] = rhs[0] - (U(0, 1) * x[1] + U(0, 2) * x[2]);
    x[0] = x[0] / U(0, 0);
    for (i = 0; i < 3; ++i) inc[i] = -x[i];
}

/* Q1Jl * inc: triangular_matrix_vector_product (RowMajor, Upper, one panel): res_i = 0 + 1 * left fold of the row segment */
static void lb_trmv(const lblock* b, const float inc[3], float t3[3]) {
    const int nc = b->num_cols;
    t3[0] = 0.0f + 1.0f * ((U(0, 0) * inc[0] + U(0, 1) * inc[1]) + U(0, 2) * inc[2]);
    t3[1] = 0.0f + 1.0f * (U(1, 1) * inc[1] + U(1, 2) * inc[2]);
    t3[2] = 0.0f + 1.0f * (U(2, 2) * inc[2]);
}
#undef U

static void lb_back_substitute_ex(lblock* b, const float* pose_inc, float* l_diff, float* inc_out, float* QJinc_head3) {
    const int nc = b->num_cols, P = b->padding_idx, nq = b->num_rows - 3;
    float inc[3], t3[3];
    float* QJinc = (float*)malloc(sizeof(float) * (size_t)nq);
    float* terms = (float*)malloc(sizeof(float) * (size_t)(nq > 0 ? nq : 1));
    int i;
    lb_solve_inc(b, pose_inc, inc);
    for (i = 0; i < nq; ++i) QJinc[i] = 0.0f;
    la_gemv(nq, P, b->st, nc, 1, pose_inc, 1, QJinc, 1.0f);          /* storage.topLeftCorner(num_rows - 3, padding_idx) * pose_inc */
    lb_trmv(b, inc, t3);
    for (i = 0; i < 3; ++i) QJinc[i] = QJinc[i] + t3[i];             /* QJinc.head<3>() += Q1Jl * inc */
    for (i = 0; i < nq; ++i) terms[i] = QJinc[i] * (0.5f * QJinc[i] + b->st[(size_t)i * nc + b->res_idx]);   /* QJinc^T (0.5 QJinc + Qr), Qr strided */
    if (nq > 0) *l_diff = *l_diff - fold_sum(terms, nq);
    if (inc_out) { inc_out[0] = inc[0]; inc_out[1] = inc[1]; inc_out[2] = inc[2]; }
    if (QJinc_head3) { QJinc_head3[0] = QJinc[0]; QJinc_head3[1] = QJinc[1]; QJinc_head3[2] = QJinc[2]; }
    b->lm->direction[0] = b->lm->direction[0] + inc[0];
    b->lm->direction[1] = b->lm->direction[1] + inc[1];
    {
        const float v = b->lm->inv_dist + inc[2];
        b->lm->inv_dist = (0.0f < v) ? v : 0.0f;                      /* std::max(Scalar(0), inv_dist + inc[2]) */
    }
    free(QJinc); free(terms);
}

static void lb_back_substitute(lblock* b, const float* pose_inc, float* l_diff) { lb_back_substitute_ex(b, pose_inc, l_diff, NULL, NULL); }

/* ------------------------------------------------------------------------------------------------ LinearizationAbsQR */

static int in_set(const int64_t* a, int n, int64_t v) { int i; for (i = 0; i < n; ++i) if (a[i] == v) return 1; return 0; }

bs_linabsqr* bs_la_create(bs_ba* ba, const bs_aom* aom, const bs_marg_lin* marg, const bs_imu_lin* imu, const int64_t* used_frames, int n_used,
                          const int64_t* lost_landmarks, int n_lost) {
    bs_linabsqr* la = (bs_linabsqr*)calloc(1, sizeof(bs_linabsqr));
    const bs_hnode* hn;
    int nh = 0, total = 0, i, cap;
    la->ba = ba; la->aom = aom; la->marg = marg; la->imu = imu;
    la->huber = ba->huber_thresh; la->obs_std = ba->obs_std_dev;
    bs_htab_init(&la->host_to_idx, BS_HK_TCID);
    for (hn = ba->lmdb.observations.before_begin.next; hn; hn = hn->next) nh++;
    la->n_hosts = nh;
    la->host_node = (const bs_hnode**)malloc(sizeof(bs_hnode*) * (nh > 0 ? nh : 1));
    la->host_base = (int*)malloc(sizeof(int) * (nh > 0 ? nh : 1));
    nh = 0;
    for (hn = ba->lmdb.observations.before_begin.next; hn; hn = hn->next) {
        int ins;
        bs_hnode* e = bs_htab_insert(&la->host_to_idx, hn->k0, hn->k1, &ins);
        e->val = (void*)(size_t)(nh + 1);
        la->host_node[nh] = hn;
        la->host_base[nh] = total;
        total += ((const bs_host*)hn->val)->n;
        nh++;
    }
    la->rel = (rel_pose_lin*)calloc((size_t)(total > 0 ? total : 1), sizeof(rel_pose_lin));
    /* landmark_ids: iterate kpts in node order */
    cap = (int)ba->lmdb.kpts.nelem;
    la->landmark_ids = (int64_t*)malloc(sizeof(int64_t) * (cap > 0 ? cap : 1));
    for (hn = ba->lmdb.kpts.before_begin.next; hn; hn = hn->next) {
        const bs_keypoint* v = (const bs_keypoint*)hn->val;
        const int64_t k = hn->k0;
        if (n_used >= 0 || n_lost >= 0) {
            if (n_used >= 0 && in_set(used_frames, n_used, v->host.frame_id)) la->landmark_ids[la->nlm++] = k;
            else if (n_lost >= 0 && in_set(lost_landmarks, n_lost, k)) la->landmark_ids[la->nlm++] = k;
        } else {
            la->landmark_ids[la->nlm++] = k;
        }
    }
    la->blocks = (lblock*)calloc((size_t)(la->nlm > 0 ? la->nlm : 1), sizeof(lblock));
    la->block_row = (int*)malloc(sizeof(int) * (la->nlm > 0 ? la->nlm : 1));
    for (i = 0; i < la->nlm; ++i) lb_allocate(la, &la->blocks[i], bs_lmdb_get_landmark(&ba->lmdb, la->landmark_ids[i]));
    la->num_rows_Q2r = 0;
    for (i = 0; i < la->nlm; ++i) {
        la->block_row[i] = la->num_rows_Q2r;
        la->num_rows_Q2r += la->blocks[i].num_rows - 3;
    }
    if (imu) {
        la->n_imu = imu->n;
        la->imu_Jp = (float(*)[450])calloc((size_t)(imu->n > 0 ? imu->n : 1), sizeof(float[450]));
        la->imu_r = (float(*)[15])calloc((size_t)(imu->n > 0 ? imu->n : 1), sizeof(float[15]));
    }
    return la;
}

void bs_la_destroy(bs_linabsqr* la) {
    int i;
    if (!la) return;
    for (i = 0; i < la->nlm; ++i) { free(la->blocks[i].st); free(la->blocks[i].pl); }
    free(la->blocks); free(la->block_row); free(la->landmark_ids); free(la->rel); free(la->host_node); free(la->host_base);
    free(la->imu_Jp); free(la->imu_r);
    bs_htab_destroy(&la->host_to_idx, NULL);
    free(la);
}

int bs_la_num_landmark_blocks(const bs_linabsqr* la) { return la->nlm; }
int64_t bs_la_landmark_id(const bs_linabsqr* la, int i) { return la->landmark_ids[i]; }
int bs_la_dense_Q2_rows(const bs_linabsqr* la) {
    int total = la->num_rows_Q2r;
    if (la->imu) total += la->imu->n * 15;
    if (la->marg) total += la->marg->rows;
    return total;
}

float bs_la_linearize_problem(bs_linabsqr* la, int* numerically_valid) {
    const bs_ba* ba = la->ba;
    float error;
    int ok = 1, hi, ti, i;
    for (hi = 0; hi < la->n_hosts; ++hi) {
        const bs_hnode* hn = la->host_node[hi];
        const bs_host* h = (const bs_host*)hn->val;
        const bs_tcid tcid_h = {hn->k0, (uint64_t)hn->k1};
        for (ti = 0; ti < h->n; ++ti) {
            const bs_tcid tcid_t = h->tgt[ti].t;
            rel_pose_lin* rpl = &la->rel[la->host_base[hi] + ti];
            if (bs_tcid_cmp(tcid_h, tcid_t) != 0) {
                pose_view sh, st;
                bs_se3f rel;
                get_pose_view(ba, tcid_h.frame_id, &sh);
                get_pose_view(ba, tcid_t.frame_id, &st);
                bs_compute_rel_posef(&sh.lin, &ba->T_i_c[tcid_h.cam_id], &st.lin, &ba->T_i_c[tcid_t.cam_id], rpl->d_rel_d_h, rpl->d_rel_d_t, &rel);
                if (sh.linearized || st.linearized)
                    bs_compute_rel_posef(pv_pose(&sh), &ba->T_i_c[tcid_h.cam_id], pv_pose(&st), &ba->T_i_c[tcid_t.cam_id], NULL, NULL, &rel);
                bs_se3f_matrix(&rel, rpl->T_t_h);
            } else {
                for (i = 0; i < 16; ++i) rpl->T_t_h[i] = ((i % 5) == 0) ? 1.0f : 0.0f;
                for (i = 0; i < 36; ++i) { rpl->d_rel_d_h[i] = 0.0f; rpl->d_rel_d_t[i] = 0.0f; }
            }
        }
    }
    error = 0.0f;
    for (i = 0; i < la->nlm; ++i) {
        error = error + lb_linearize(la, &la->blocks[i]);
        ok = ok && (la->blocks[i].state != ST_NUMFAIL);
    }
    if (numerically_valid) *numerically_valid = ok;
    if (la->imu) {
        for (i = 0; i < la->imu->n; ++i) {
            bs_imu_meas* m = la->imu->meas[i];
            const bs_frame_state* ss = find_state(ba, m->start_t_ns);
            const bs_frame_state* es = find_state(ba, m->start_t_ns + m->delta.t_ns);
            error = error + bs_imu_linearize(m, la->imu->g, la->imu->gyro_bias_weight_sqrt, la->imu->accel_bias_weight_sqrt, &ss->s, &es->s,
                                             la->imu_Jp[i], la->imu_r[i]);
        }
    }
    if (la->marg) error = error + bs_ba_marg_prior_error(ba, la->marg);
    return error;
}

void bs_la_perform_qr(bs_linabsqr* la) {
    int i;
    for (i = 0; i < la->nlm; ++i) lb_perform_qr(&la->blocks[i]);
}

float bs_la_back_substitute(bs_linabsqr* la, const float* pose_inc) {
    float l_diff = 0.0f;
    int i;
    for (i = 0; i < la->nlm; ++i) lb_back_substitute(&la->blocks[i], pose_inc, &l_diff);
    if (la->imu) {
        for (i = 0; i < la->imu->n; ++i) {
            const bs_imu_meas* m = la->imu->meas[i];
            const bs_aom_item* si = aom_find(la->aom, m->start_t_ns);
            const bs_aom_item* ei = aom_find(la->aom, m->start_t_ns + m->delta.t_ns);
            float inc[30], Jinc[15], terms[15];
            int j;
            for (j = 0; j < 15; ++j) { inc[j] = pose_inc[si->start + j]; inc[15 + j] = pose_inc[ei->start + j]; }
            for (j = 0; j < 15; ++j) Jinc[j] = 0.0f;
            la_gemv(15, 30, la->imu_Jp[i], 1, 15, inc, 1, Jinc, 1.0f);       /* Jp * pose_inc_reduced (col-major GEMV) */
            for (j = 0; j < 15; ++j) terms[j] = Jinc[j] * (0.5f * Jinc[j] + la->imu_r[i][j]);
            l_diff = l_diff - vec_sum(terms, 15);
        }
    }
    if (la->marg) {
        const int marg_size = la->marg->cols;
        l_diff = l_diff + bs_ba_marg_prior_model_cost_change(la->ba, la->marg, pose_inc);
        (void)marg_size;
    }
    return l_diff;
}

void bs_la_get_dense_H_b(const bs_linabsqr* la, float* H, float* b) {
    const int n = la->aom->total_size;
    int i, j, bi;
    for (i = 0; i < n * n; ++i) H[i] = 0.0f;
    for (i = 0; i < n; ++i) b[i] = 0.0f;
    for (bi = 0; bi < la->nlm; ++bi) {
        const lblock* lb = &la->blocks[bi];
        const int nc = lb->num_cols, P = lb->padding_idx, k = lb->num_rows - 3;
        const float* J = lb->st + (size_t)3 * nc;
        la_gemm(P, P, k, J, 1, nc, J, nc, 1, H, n);                          /* H.noalias() += J^T * J */
        la_gemv(P, k, J, 1, nc, J + lb->res_idx, nc, b, 1.0f);               /* b.noalias() += J^T * r (r = col(res_idx).tail) */
    }
    if (la->imu && la->imu->n > 0) {                                         /* add_dense_H_b_imu(H, b) through a DenseAccumulator */
        float* aH = (float*)calloc((size_t)n * n, sizeof(float));
        float* ab = (float*)calloc((size_t)n, sizeof(float));
        for (bi = 0; bi < la->imu->n; ++bi) {
            const bs_imu_meas* m = la->imu->meas[bi];
            const bs_aom_item* si = aom_find(la->aom, m->start_t_ns);
            const bs_aom_item* ei = aom_find(la->aom, m->start_t_ns + m->delta.t_ns);
            float Hb[900], bb[30];
            const float* Jp = la->imu_Jp[bi];
            const float* r = la->imu_r[bi];
            const int s = si->start, e = ei->start;
            for (i = 0; i < 900; ++i) Hb[i] = 0.0f;
            for (i = 0; i < 30; ++i) bb[i] = 0.0f;
            la_gemm(30, 30, 15, Jp, 15, 1, Jp, 1, 15, Hb, 30);               /* const MatX H = Jp^T * Jp */
            la_gemv(30, 15, Jp, 15, 1, r, 1, bb, 1.0f);                      /* const VecX b = Jp^T * r */
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                aH[(s + i) + (size_t)n * (s + j)] = aH[(s + i) + (size_t)n * (s + j)] + Hb[i + 30 * j];
            }
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                aH[(e + i) + (size_t)n * (s + j)] = aH[(e + i) + (size_t)n * (s + j)] + Hb[(15 + i) + 30 * j];
            }
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                aH[(s + i) + (size_t)n * (e + j)] = aH[(s + i) + (size_t)n * (e + j)] + Hb[i + 30 * (15 + j)];
            }
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                aH[(e + i) + (size_t)n * (e + j)] = aH[(e + i) + (size_t)n * (e + j)] + Hb[(15 + i) + 30 * (15 + j)];
            }
            for (i = 0; i < 15; ++i) ab[s + i] = ab[s + i] + bb[i];
            for (i = 0; i < 15; ++i) ab[e + i] = ab[e + i] + bb[15 + i];
        }
        for (i = 0; i < n * n; ++i) H[i] = H[i] + aH[i];
        for (i = 0; i < n; ++i) b[i] = b[i] + ab[i];
        free(aH); free(ab);
    }
    if (la->marg) {                                                          /* linearizeMargPrior (is_sqrt) */
        const bs_marg_lin* mld = la->marg;
        const int rows = mld->rows, cols = mld->cols;
        float* delta = (float*)malloc(sizeof(float) * (size_t)cols);
        float* tmpH = (float*)calloc((size_t)cols * cols, sizeof(float));
        float* hd = (float*)malloc(sizeof(float) * (size_t)rows);
        float* t2 = (float*)malloc(sizeof(float) * (size_t)rows);
        float* t3 = (float*)calloc((size_t)cols, sizeof(float));
        bs_ba_compute_delta(la->ba, &mld->order, delta);
        if (rows + 2 * cols < 20) bs_la_unsupported++;
        la_gemm(cols, cols, rows, mld->H, rows, 1, mld->H, 1, rows, tmpH, cols);   /* abs_H.topLeftCorner += H^T * H (temporary) */
        for (j = 0; j < cols; ++j) for (i = 0; i < cols; ++i) H[i + (size_t)n * j] = H[i + (size_t)n * j] + tmpH[i + (size_t)cols * j];
        mat_vec(mld->H, rows, cols, delta, hd, 1.0f);                        /* mld.b + mld.H * delta */
        for (i = 0; i < rows; ++i) t2[i] = mld->b[i] + hd[i];
        la_gemv(cols, rows, mld->H, rows, 1, t2, 1, t3, 1.0f);               /* mld.H^T * (...): row-major GEMV into a temporary */
        for (i = 0; i < cols; ++i) b[i] = b[i] + t3[i];
        free(delta); free(tmpH); free(hd); free(t2); free(t3);
    }
}

void bs_la_get_dense_Q2Jp_Q2r(const bs_linabsqr* la, float* Q2Jp, float* Q2r) {
    const int n = la->aom->total_size;
    const int total = bs_la_dense_Q2_rows(la);
    int imu_start = la->num_rows_Q2r, marg_start = imu_start + (la->imu ? la->imu->n * 15 : 0);
    int i, j, bi;
    for (i = 0; i < total * n; ++i) Q2Jp[i] = 0.0f;
    for (i = 0; i < total; ++i) Q2r[i] = 0.0f;
    for (bi = 0; bi < la->nlm; ++bi) {
        const lblock* lb = &la->blocks[bi];
        const int nc = lb->num_cols, P = lb->padding_idx, k = lb->num_rows - 3, start = la->block_row[bi];
        for (i = 0; i < k; ++i) Q2r[start + i] = lb->st[(size_t)(3 + i) * nc + lb->res_idx];
        for (j = 0; j < P; ++j) for (i = 0; i < k; ++i) Q2Jp[(start + i) + (size_t)total * j] = lb->st[(size_t)(3 + i) * nc + j];
    }
    if (la->imu) {
        for (bi = 0; bi < la->imu->n; ++bi) {
            const bs_imu_meas* m = la->imu->meas[bi];
            const bs_aom_item* si = aom_find(la->aom, m->start_t_ns);
            const bs_aom_item* ei = aom_find(la->aom, m->start_t_ns + m->delta.t_ns);
            const float* Jp = la->imu_Jp[bi];
            const int rs = imu_start + bi * 15;
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                Q2Jp[(rs + i) + (size_t)total * (si->start + j)] = Q2Jp[(rs + i) + (size_t)total * (si->start + j)] + Jp[i + 15 * j];
            }
            for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) {
                Q2Jp[(rs + i) + (size_t)total * (ei->start + j)] = Q2Jp[(rs + i) + (size_t)total * (ei->start + j)] + Jp[i + 15 * (15 + j)];
            }
            for (i = 0; i < 15; ++i) Q2r[rs + i] = Q2r[rs + i] + la->imu_r[bi][i];
        }
    }
    if (la->marg) {
        const bs_marg_lin* mld = la->marg;
        const int rows = mld->rows, cols = mld->cols;
        float* delta = (float*)malloc(sizeof(float) * (size_t)cols);
        float* hd = (float*)malloc(sizeof(float) * (size_t)rows);
        bs_ba_compute_delta(la->ba, &mld->order, delta);
        for (j = 0; j < cols; ++j) for (i = 0; i < rows; ++i) Q2Jp[(marg_start + i) + (size_t)total * j] = mld->H[i + (size_t)rows * j];
        mat_vec(mld->H, rows, cols, delta, hd, 1.0f);
        for (i = 0; i < rows; ++i) Q2r[marg_start + i] = hd[i] + mld->b[i];
        free(delta); free(hd);
    }
}

/* ------------------------------------------------------------------------------------------------ state increments (PoseStateWithLin / PoseVelBiasStateWithLin::applyInc) */

static void pvb_apply_inc(bs_pvbstate* st, const float inc[15]) {   /* PoseVelBiasState::applyInc */
    bs_se3f T;
    int i;
    T.so3.x = st->s.q[0]; T.so3.y = st->s.q[1]; T.so3.z = st->s.q[2]; T.so3.w = st->s.q[3];
    for (i = 0; i < 3; ++i) T.t[i] = st->s.p[i];
    bs_inc_posef(inc, &T);                                          /* t += inc.head<3>(); so3 = exp(inc.tail<3>()) * so3 */
    st->s.q[0] = T.so3.x; st->s.q[1] = T.so3.y; st->s.q[2] = T.so3.z; st->s.q[3] = T.so3.w;
    for (i = 0; i < 3; ++i) st->s.p[i] = T.t[i];
    for (i = 0; i < 3; ++i) st->s.v[i] = st->s.v[i] + inc[6 + i];
    for (i = 0; i < 3; ++i) st->bg[i] = st->bg[i] + inc[9 + i];
    for (i = 0; i < 3; ++i) st->ba[i] = st->ba[i] + inc[12 + i];
}

void bs_pose_apply_inc(bs_frame_pose* p, const float inc[6]) {
    int i;
    if (!p->linearized) {
        bs_inc_posef(inc, &p->lin);
    } else {
        for (i = 0; i < 6; ++i) p->delta[i] = p->delta[i] + inc[i];
        p->cur = p->lin;
        bs_inc_posef(p->delta, &p->cur);
    }
}

void bs_state_apply_inc(bs_frame_state* s, const float inc[15]) {
    int i;
    if (!s->s.linearized) {
        pvb_apply_inc(&s->s.lin, inc);
    } else {
        for (i = 0; i < 15; ++i) s->delta[i] = s->delta[i] + inc[i];
        s->s.cur = s->s.lin;
        pvb_apply_inc(&s->s.cur, s->delta);
    }
}

/* ------------------------------------------------------------------------------------------------ test hooks */

/* LandmarkBlockAbsDynamic::backSubstitute on an arbitrary row-major storage (num_rows x num_cols, [Jp | pad | Jl | res]) */
float bs_la_t_back_substitute(float* st, int num_rows, int num_cols, int padding_idx, int lm_idx, const float* pose_inc, float l_diff_in, float direction[2],
                              float* inv_dist, float inc_out[3], float QJinc_head3[3]) {
    lblock b;
    bs_keypoint kp;
    float l = l_diff_in;
    memset(&b, 0, sizeof b);
    memset(&kp, 0, sizeof kp);
    kp.direction[0] = direction[0]; kp.direction[1] = direction[1]; kp.inv_dist = *inv_dist;
    b.lm = &kp; b.st = st; b.num_rows = num_rows; b.num_cols = num_cols; b.padding_idx = padding_idx; b.lm_idx = lm_idx; b.res_idx = lm_idx + 3;
    lb_back_substitute_ex(&b, pose_inc, &l, inc_out, QJinc_head3);
    direction[0] = kp.direction[0]; direction[1] = kp.direction[1]; *inv_dist = kp.inv_dist;
    return l;
}

/* performQRHouseholder on an arbitrary row-major storage, in place */
void bs_la_t_householder_qr(float* st, int num_rows, int num_cols, int padding_idx, int lm_idx) {
    lblock b;
    memset(&b, 0, sizeof b);
    b.st = st; b.num_rows = num_rows; b.num_cols = num_cols; b.padding_idx = padding_idx; b.lm_idx = lm_idx; b.res_idx = lm_idx + 3;
    lb_perform_qr(&b);
}

void bs_la_t_gemm(int m, int n, int k, const float* a, int ars, int acs, const float* b, int brs, int bcs, float* c, int ldc) {
    la_skip_enable = 0;   /* the oracle feeds arbitrary C (including -0): no zero skipping */
    la_gemm(m, n, k, a, ars, acs, b, brs, bcs, c, ldc);
    la_skip_enable = 1;
}
void bs_la_t_gemv(int m, int n, const float* a, int ars, int acs, const float* x, int xinc, float* y) { la_gemv(m, n, a, ars, acs, x, xinc, y, 1.0f); }
