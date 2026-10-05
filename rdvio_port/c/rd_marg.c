/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_marg.h for provenance and licences. */
#include "rd_marg.h"
#include "rd_seig.h"
#include "rd_eigen.h"
#include "../../okvis_port/c/ok_dense.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define S3 3
#define EL(a, ld, r, c) (a)[(r) + (long)(ld) * (c)]

static double* dz(size_t n) { return (double*)calloc(n ? n : 1, sizeof(double)); }

/* ---- Eigen dynamic kernels ---- */

/* generic_product_impl<..., GemmProduct>::scaleAndAddTo (single thread, no OpenMP): R += alpha * A B with
 * general_matrix_matrix_product::run's loops over the blocking of computeProductBlockingSizes (ok_blocking_sizes), each block a gebp_kernel call.
 * A(i,k) = A[i ars + k acs], B(k,j) = B[k brs + j bcs], R(i,j) = R[i rrs + j rcs]. (Fixed-size GEMMs of static blocking: use ok_gebp directly.) */
static void gemm_acc(int rows, int cols, int depth, const double* A, long ars, long acs, const double* B, long brs, long bcs, double alpha, double* R,
                     long rrs, long rcs) {
    long kc = depth, mc = rows, nc = cols, i2, k2, j2;
    if (rows == 0 || cols == 0 || depth == 0) return;
    ok_blocking_sizes(&kc, &mc, &nc, 1);
    if (mc > rows) mc = rows;
    if (nc > cols) nc = cols;
    for (i2 = 0; i2 < rows; i2 += mc) {
        const long amc = (i2 + mc < rows ? i2 + mc : rows) - i2;
        for (k2 = 0; k2 < depth; k2 += kc) {
            const long akc = (k2 + kc < depth ? k2 + kc : depth) - k2;
            for (j2 = 0; j2 < cols; j2 += nc) {
                const long anc = (j2 + nc < cols ? j2 + nc : cols) - j2;
                ok_gebp((int)amc, (int)anc, (int)akc, A + i2 * ars + k2 * acs, ars, acs, B + k2 * brs + j2 * bcs, brs, bcs, alpha, R + i2 * rrs + j2 * rcs, rrs, rcs);
            }
        }
    }
}

/* general_matrix_vector_product<ColMajor>::run: per row one accumulator over the columns of a column block (block_cols = cols for cols < 128, else 16 when
 * the stride is < 4000 doubles, else 4), y_i = acc * alpha + y_i after every block. A(i,j) = A[i + lda j]. */
static void gemv_col(int rows, int cols, const double* A, long lda, const double* x, double* y, double alpha) {
    const long block_cols = cols < 128 ? cols : (lda * 8 < 32000 ? 16 : 4);
    long j2;
    int i;
    for (j2 = 0; j2 < cols; j2 += block_cols) {
        const long jend = j2 + block_cols < cols ? j2 + block_cols : cols;
        long j;
        for (i = 0; i < rows; ++i) {
            double c = 0.0;
            for (j = j2; j < jend; ++j) c = A[i + lda * j] * x[j] + c;
            y[i] = c * alpha + y[i];
        }
    }
}

/* Eigen redux (sum) over a contiguous dynamic expression without direct access: Packet2d, two two-lane accumulators (LinearVectorizedTraversal) */
static double redux2(const double* v, int n) {
    int end, end4, i;
    double a, b, c, d, sum;
    if (n == 0) return 0.0;
    end = n / 2 * 2;
    end4 = n / 4 * 4;
    if (!end) return v[0];
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

/* ---- factor state ---- */

void rd_marg_free(rd_marg* m) {
    free(m->ids); free(m->lin_pose); free(m->lin_motion); free(m->sqrt_inv_cov); free(m->infovec);
    memset(m, 0, sizeof *m);
}

static void marg_alloc(rd_marg* m, int nf) {
    const size_t N = (size_t)nf * RD_ES_SIZE;
    m->nf = nf;
    m->ids = (uint64_t*)calloc(nf ? (size_t)nf : 1, sizeof(uint64_t));
    m->lin_pose = (rd_pose*)calloc(nf ? (size_t)nf : 1, sizeof(rd_pose));
    m->lin_motion = (rd_motion*)calloc(nf ? (size_t)nf : 1, sizeof(rd_motion));
    m->sqrt_inv_cov = dz(N * N);
    m->infovec = dz(N);
}

void rd_marg_init(rd_marg* m, int nf_map, const rd_marg_frame* frames) {
    const int nf = nf_map - 1, N = nf * RD_ES_SIZE;
    int i, k;
    marg_alloc(m, nf);
    for (i = 0; i < nf; ++i) {
        m->ids[i] = frames[i].id;
        m->lin_pose[i] = frames[i].pose;
        m->lin_motion[i] = frames[i].motion;
    }
    /* sqrt_inv_cov.block<3,3>(ES_P, ES_P) = 1.0e15 * Identity; the same for ES_Q */
    for (k = 0; k < 3; ++k) {
        EL(m->sqrt_inv_cov, N, RD_ES_P + k, RD_ES_P + k) = 1.0e15 * 1.0;
        EL(m->sqrt_inv_cov, N, RD_ES_Q + k, RD_ES_Q + k) = 1.0e15 * 1.0;
    }
}

/* ---- Evaluate ---- */

int rd_marg_eval(const rd_marg* m, const double* const* params, double* residuals, double* const* jacobians) {
    const int nf = m->nf, N = nf * RD_ES_SIZE;
    int i, k;
    for (i = 0; i < nf; ++i) {
        ok_quat q, ql, d;
        double* r = residuals + RD_ES_SIZE * i;
        const double* p = params[5 * i + 1];
        const double* v = params[5 * i + 2];
        const double* bg = params[5 * i + 3];
        const double* ba = params[5 * i + 4];
        q.x = params[5 * i][0]; q.y = params[5 * i][1]; q.z = params[5 * i][2]; q.w = params[5 * i][3];
        ql = rd_quat_conj(m->lin_pose[i].q);
        ok_quat_mul(&ql, &q, &d);
        rd_logmap(&d, r + RD_ES_Q);
        for (k = 0; k < 3; ++k) {
            r[RD_ES_P + k] = p[k] - m->lin_pose[i].p[k];
            r[RD_ES_V + k] = v[k] - m->lin_motion[i].v[k];
            r[RD_ES_BG + k] = bg[k] - m->lin_motion[i].bg[k];
            r[RD_ES_BA + k] = ba[k] - m->lin_motion[i].ba[k];
        }
    }
    if (jacobians) {
        for (i = 0; i < nf; ++i) {
            for (k = 0; k < 5; ++k) {
                double* J = jacobians[5 * i + k];
                const int c = k == 0 ? 4 : 3;
                double* tmp;
                long r, cc;
                if (!J) continue;
                memset(J, 0, sizeof(double) * (size_t)N * c);
                if (k == 0) {
                    double jr[9], inv[9];
                    rd_right_jacobian(residuals + RD_ES_SIZE * i + RD_ES_Q, jr);
                    rd_inverse3(jr, inv);
                    for (r = 0; r < 3; ++r)
                        for (cc = 0; cc < 3; ++cc) J[(RD_ES_SIZE * i + RD_ES_Q + r) * 4 + cc] = inv[r + 3 * cc];
                } else {
                    for (r = 0; r < 3; ++r) J[(RD_ES_SIZE * i + k * 3 + r) * 3 + r] = 1.0;
                }
                /* dr_dk = sqrt_inv_cov * dr_dk: aliasing -> column-major temporary (GEMM, rows N, cols c, depth N), then copied into the row-major map */
                tmp = dz((size_t)N * c);
                gemm_acc(N, c, N, m->sqrt_inv_cov, 1, N, J, c, 1, 1.0, tmp, 1, N);
                for (r = 0; r < N; ++r)
                    for (cc = 0; cc < c; ++cc) J[r * c + cc] = tmp[r + (long)N * cc];
                free(tmp);
            }
        }
    }
    /* full_residual = sqrt_inv_cov * full_residual + infovec: the product evaluates into a temporary (GEMV), then the sum */
    {
        double* tmp = dz((size_t)N);
        gemv_col(N, N, m->sqrt_inv_cov, N, residuals, tmp, 1.0);
        for (i = 0; i < N; ++i) residuals[i] = tmp[i] + m->infovec[i];
        free(tmp);
    }
    return 1;
}

/* ---- marginalize ---- */

typedef struct lm_h { int frame; double h[6]; } lm_h;
typedef struct lm_info {
    uint64_t id;
    double mat, vec;
    int nh, caph;
    lm_h* h;
} lm_info;

static lm_h* lm_get(lm_info* L, int frame) {
    int i;
    for (i = 0; i < L->nh; ++i) if (L->h[i].frame == frame) return &L->h[i];
    if (L->nh == L->caph) {
        L->caph = L->caph ? 2 * L->caph : 4;
        L->h = (lm_h*)realloc(L->h, sizeof(lm_h) * (size_t)L->caph);
    }
    L->h[L->nh].frame = frame;
    memset(L->h[L->nh].h, 0, sizeof L->h[L->nh].h);
    return &L->h[L->nh++];
}

/* block<3,3>(r0, c0) += A^T * B with A, B row-major 2x3 (a small coefficient-based product: (a0 b0) + (a1 b1)) */
static void add_tprod(double* H, int N, int r0, int c0, const double* A, const double* B) {
    int r, c;
    for (c = 0; c < 3; ++c)
        for (r = 0; r < 3; ++r) EL(H, N, r0 + r, c0 + c) += (A[r] * B[c]) + (A[3 + r] * B[3 + c]);
}
/* segment<3>(r0) += A^T * v (3x2 * 2x1 coefficient-based) */
static void add_tvec(double* b, int r0, const double* A, const double* v) {
    int r;
    for (r = 0; r < 3; ++r) b[r0 + r] += (A[r] * v[0]) + (A[3 + r] * v[1]);
}

static int cmp_lm(const void* a, const void* b) {
    const lm_info* x = *(lm_info* const*)a;
    const lm_info* y = *(lm_info* const*)b;
    return x->id < y->id ? -1 : (x->id > y->id ? 1 : 0);
}

void rd_marg_dbg_free(rd_marg_dbg* d) {
    free(d->infomat); free(d->infovec); free(d->evals); free(d->evecs);
    memset(d, 0, sizeof *d);
}

int rd_marg_marginalize(rd_marg* m, int nf_map, const rd_marg_frame* fr, int index, int ntracks, const rd_marg_track* tracks, rd_marg_dbg* dbg) {
    const int NF = nf_map * RD_ES_SIZE, nff = m->nf, N1 = nff * RD_ES_SIZE;
    double* H = dz((size_t)NF * NF);       /* pose_motion_infomat */
    double* hb = dz((size_t)NF);           /* pose_motion_infovec */
    int* sidx = (int*)malloc(sizeof(int) * (size_t)nf_map);   /* frame_indices by map position */
    int* fpos = (int*)malloc(sizeof(int) * (size_t)(nff ? nff : 1));    /* map position of every frame of the old factor */
    lm_info* lms = NULL;
    lm_info** order = NULL;
    int nlm = 0, caplm = 0, i, j, k, ret = 0;
    int last_index = nf_map - 1;

    for (i = 0; i < nf_map; ++i) sidx[i] = i < index ? i : (i > index ? i - 1 : nf_map - 1);
    for (i = 0; i < nff; ++i) {
        fpos[i] = -1;
        for (j = 0; j < nf_map; ++j) if (fr[j].id == m->ids[i]) { fpos[i] = j; break; }
        if (fpos[i] < 0) { ret = 1; goto done; }
    }

    /* scope: marginalization factor */
    {
        const double** par = (const double**)calloc((size_t)(5 * nff + 1), sizeof(double*));
        double** jp = (double**)calloc((size_t)(5 * nff + 1), sizeof(double*));
        double** jm = (double**)calloc((size_t)(5 * nff + 1), sizeof(double*));
        double* mres = dz((size_t)N1);
        double** sj = (double**)malloc(sizeof(double*) * (size_t)(nff + 1));
        for (i = 0; i < nff; ++i) {
            const rd_marg_frame* f = &fr[fpos[i]];
            par[5 * i + 0] = &f->pose.q.x; par[5 * i + 1] = f->pose.p; par[5 * i + 2] = f->motion.v; par[5 * i + 3] = f->motion.bg; par[5 * i + 4] = f->motion.ba;
            for (k = 0; k < 5; ++k) { jm[5 * i + k] = dz((size_t)N1 * (k == 0 ? 4 : 3)); jp[5 * i + k] = jm[5 * i + k]; }
        }
        rd_marg_eval(m, (const double* const*)par, mres, jp);
        for (i = 0; i < nff; ++i) {
            double* d = dz((size_t)N1 * RD_ES_SIZE);   /* dr_ds: N1 x 15, column-major */
            long r, c;
            for (r = 0; r < N1; ++r) {
                for (c = 0; c < 3; ++c) d[r + (long)N1 * (RD_ES_Q + c)] = jm[5 * i][r * 4 + c];
                for (k = 1; k < 5; ++k)
                    for (c = 0; c < 3; ++c) d[r + (long)N1 * (3 * k + c)] = jm[5 * i + k][r * 3 + c];
            }
            sj[i] = d;
        }
        for (i = 0; i < nff; ++i) {
            const int fi = sidx[fpos[i]];
            for (j = 0; j < nff; ++j) {
                const int fj = sidx[fpos[j]];
                /* block<15,15> += J_i^T * J_j: GEMM (rows 15, cols 15, depth N1), accumulated in place */
                gemm_acc(RD_ES_SIZE, RD_ES_SIZE, N1, sj[i], N1, 1, sj[j], 1, N1, 1.0, &EL(H, NF, RD_ES_SIZE * fi, RD_ES_SIZE * fj), 1, NF);
            }
            ok_gemv_row(RD_ES_SIZE, N1, sj[i], N1, mres, &hb[RD_ES_SIZE * fi], 1.0);
        }
        for (i = 0; i < nff; ++i) free(sj[i]);
        for (i = 0; i < 5 * nff; ++i) free(jm[i]);
        free(par); free(jp); free(jm); free(mres); free(sj);
    }

    /* scope: preintegration factor */
    for (j = index; j <= index + 1; ++j) {
        double jq_i[15 * 4], jp_i[15 * 3], jv_i[15 * 3], jg_i[15 * 3], ja_i[15 * 3];
        double jq_j[15 * 4], jp_j[15 * 3], jv_j[15 * 3], jg_j[15 * 3], ja_j[15 * 3];
        double* jac[10] = {jq_i, jp_i, jv_i, jg_i, ja_i, jq_j, jp_j, jv_j, jg_j, ja_j};
        const double* par[10];
        double res[15], di[225], dj[225];
        const rd_marg_frame *fi, *fj;
        int ii, jj;
        if (j == 0) continue;
        if (j >= nf_map) continue;
        fi = &fr[j - 1];
        fj = &fr[j];
        par[0] = &fi->pose.q.x; par[1] = fi->pose.p; par[2] = fi->motion.v; par[3] = fi->motion.bg; par[4] = fi->motion.ba;
        par[5] = &fj->pose.q.x; par[6] = fj->pose.p; par[7] = fj->motion.v; par[8] = fj->motion.bg; par[9] = fj->motion.ba;
        if (!fj->kpre) { ret = 2; goto done; }
        rd_pie_eval(fj->kpre, &fi->imu.q_cs, fi->imu.p_cs, &fj->imu.q_cs, fj->imu.p_cs, fi->motion.bg, fi->motion.ba, par, res, jac);
        {
            double* out[2] = {di, dj};
            double** jb[2] = {&jac[0], &jac[5]};
            int s, r, c;
            for (s = 0; s < 2; ++s) {
                for (r = 0; r < 15; ++r) {
                    for (c = 0; c < 3; ++c) out[s][r + 15 * (RD_ES_Q + c)] = jb[s][0][r * 4 + c];
                    for (k = 1; k < 5; ++k)
                        for (c = 0; c < 3; ++c) out[s][r + 15 * (3 * k + c)] = jb[s][k][r * 3 + c];
                }
            }
        }
        ii = sidx[j - 1];
        jj = sidx[j];
        /* four fixed-size 15x15x15 GEMMs (static blocking), accumulated in place, then two fixed GEMVs */
        ok_gebp(15, 15, 15, di, 15, 1, di, 1, 15, 1.0, &EL(H, NF, 15 * ii, 15 * ii), 1, NF);
        ok_gebp(15, 15, 15, di, 15, 1, dj, 1, 15, 1.0, &EL(H, NF, 15 * ii, 15 * jj), 1, NF);
        ok_gebp(15, 15, 15, dj, 15, 1, di, 1, 15, 1.0, &EL(H, NF, 15 * jj, 15 * ii), 1, NF);
        ok_gebp(15, 15, 15, dj, 15, 1, dj, 1, 15, 1.0, &EL(H, NF, 15 * jj, 15 * jj), 1, NF);
        ok_gemv_row(15, 15, di, 15, res, &hb[15 * ii], 1.0);
        ok_gemv_row(15, 15, dj, 15, res, &hb[15 * jj], 1.0);
    }

    /* scope: reprojection error factor */
    for (i = 0; i < ntracks; ++i) {
        const rd_marg_track* t = &tracks[i];
        const int ref = sidx[t->ref];
        lm_info* L = NULL;
        for (j = 0; j < t->nobs; ++j) {
            const rd_marg_obs* o = &t->obs[j];
            const int tgt = o->tgt >= 0 ? sidx[o->tgt] : -1;
            const rd_marg_frame *ft, *frf;
            const double* par[5];
            double res[2];
            double jqt[8], jpt[6], jqr[8], jpr[6], jid[2];
            double* jac[5] = {jqt, jpt, jqr, jpr, jid};
            double lq_t[6], lq_r[6];
            int r, c;
            if (o->tgt < 0) continue;
            ft = &fr[o->tgt];
            frf = &fr[t->ref];
            par[0] = &ft->pose.q.x; par[1] = ft->pose.p; par[2] = &frf->pose.q.x; par[3] = frf->pose.p; par[4] = &t->inv_depth;
            rd_rpe_eval(o->z, o->z_ref, &o->cam_ref, &o->cam_tgt, o->sqrt_inv_cov, par, res, jac);
            for (r = 0; r < 2; ++r)
                for (c = 0; c < 3; ++c) { lq_t[r * 3 + c] = jqt[r * 4 + c]; lq_r[r * 3 + c] = jqr[r * 4 + c]; }
#define BLK(rf, cf, A, B) add_tprod(H, NF, RD_ES_SIZE * (rf), RD_ES_SIZE * (cf), A, B)
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_Q, RD_ES_SIZE * tgt + RD_ES_Q, lq_t, lq_t);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_P, RD_ES_SIZE * tgt + RD_ES_P, jpt, jpt);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_Q, RD_ES_SIZE * tgt + RD_ES_P, lq_t, jpt);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_P, RD_ES_SIZE * tgt + RD_ES_Q, jpt, lq_t);

            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_Q, RD_ES_SIZE * tgt + RD_ES_Q, lq_r, lq_t);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_P, RD_ES_SIZE * tgt + RD_ES_P, jpr, jpt);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_Q, RD_ES_SIZE * tgt + RD_ES_P, lq_r, jpt);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_P, RD_ES_SIZE * tgt + RD_ES_Q, jpr, lq_t);

            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_Q, RD_ES_SIZE * ref + RD_ES_Q, lq_t, lq_r);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_P, RD_ES_SIZE * ref + RD_ES_P, jpt, jpr);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_Q, RD_ES_SIZE * ref + RD_ES_P, lq_t, jpr);
            add_tprod(H, NF, RD_ES_SIZE * tgt + RD_ES_P, RD_ES_SIZE * ref + RD_ES_Q, jpt, lq_r);

            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_Q, RD_ES_SIZE * ref + RD_ES_Q, lq_r, lq_r);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_P, RD_ES_SIZE * ref + RD_ES_P, jpr, jpr);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_Q, RD_ES_SIZE * ref + RD_ES_P, lq_r, jpr);
            add_tprod(H, NF, RD_ES_SIZE * ref + RD_ES_P, RD_ES_SIZE * ref + RD_ES_Q, jpr, lq_r);
#undef BLK
            add_tvec(hb, RD_ES_SIZE * tgt + RD_ES_Q, lq_t, res);
            add_tvec(hb, RD_ES_SIZE * tgt + RD_ES_P, jpt, res);
            add_tvec(hb, RD_ES_SIZE * ref + RD_ES_Q, lq_r, res);
            add_tvec(hb, RD_ES_SIZE * ref + RD_ES_P, jpr, res);

            /* LandmarkInfo &linfo = landmark_info[track] */
            if (!L) {
                int q;
                for (q = 0; q < nlm; ++q) if (lms[q].id == t->id) { L = &lms[q]; break; }
                if (!L) {
                    if (nlm == caplm) {
                        caplm = caplm ? 2 * caplm : 16;
                        lms = (lm_info*)realloc(lms, sizeof(lm_info) * (size_t)caplm);
                    }
                    memset(&lms[nlm], 0, sizeof(lm_info));
                    lms[nlm].id = t->id;
                    L = &lms[nlm++];
                }
            }
            L->mat += (jid[0] * jid[0]) + (jid[1] * jid[1]);
            L->vec += (jid[0] * res[0]) + (jid[1] * res[1]);
            {
                lm_h *ht, *hr;
                (void)lm_get(L, tgt);
                (void)lm_get(L, ref);
                ht = lm_get(L, tgt);
                hr = lm_get(L, ref);
                for (c = 0; c < 3; ++c) {
                    ht->h[0 + c] += (jid[0] * lq_t[c]) + (jid[1] * lq_t[3 + c]);
                    ht->h[3 + c] += (jid[0] * jpt[c]) + (jid[1] * jpt[3 + c]);
                }
                for (c = 0; c < 3; ++c) {
                    hr->h[0 + c] += (jid[0] * lq_r[c]) + (jid[1] * lq_r[3 + c]);
                    hr->h[3 + c] += (jid[0] * jpr[c]) + (jid[1] * jpr[3 + c]);
                }
            }
        }
    }

    /* scope: marginalize landmarks (std::map<Track*, ..., compare<Track*>>: ascending track id) */
    order = (lm_info**)malloc(sizeof(lm_info*) * (size_t)(nlm ? nlm : 1));
    for (i = 0; i < nlm; ++i) order[i] = &lms[i];
    qsort(order, (size_t)nlm, sizeof(lm_info*), cmp_lm);
    for (i = 0; i < nlm; ++i) {
        const lm_info* L = order[i];
        const double inv = 1.0 / L->mat;
        int a, b, r, c;
        if (!isfinite(inv)) continue;
        for (a = 0; a < L->nh; ++a) {
            const double* hi = L->h[a].h;
            const int fi = L->h[a].frame;
            for (b = 0; b < L->nh; ++b) {
                const double* hj = L->h[b].h;
                const int fj = L->h[b].frame;
                /* block<6,6> -= (h_i^T * inv) * h_j: an outer product, columns of (h_j[c] * (h_i[r] * inv)) */
                for (c = 0; c < 6; ++c)
                    for (r = 0; r < 6; ++r) EL(H, NF, RD_ES_SIZE * fi + RD_ES_Q + r, RD_ES_SIZE * fj + RD_ES_Q + c) -= hj[c] * (hi[r] * inv);
            }
            for (r = 0; r < 6; ++r) hb[RD_ES_SIZE * fi + RD_ES_Q + r] -= (hi[r] * inv) * L->vec;
        }
    }

    /* scope: marginalize the corresponding frame */
    {
        const int M = RD_ES_SIZE * last_index;     /* ES_SIZE * (frame_num - 1) */
        const int c0 = RD_ES_SIZE * last_index;
        double blk[225], inv[225];
        double* X = dz((size_t)M * RD_ES_SIZE);
        double* C = dz((size_t)M * M);
        double* cv = dz((size_t)M);
        int a, b;
        for (b = 0; b < 15; ++b)
            for (a = 0; a < 15; ++a) blk[a + 15 * b] = EL(H, NF, c0 + a, c0 + b);
        rd_inverse_ppl(15, blk, inv);
        for (b = 0; b < M; ++b)
            for (a = 0; a < M; ++a) C[a + (long)M * b] = EL(H, NF, a, b);
        for (a = 0; a < M; ++a) cv[a] = hb[a];
        /* (H.block(0, c0, M, 15) * inv): GEMM into a zeroed column-major temporary (Dynamic x 15) */
        gemm_acc(M, 15, 15, &EL(H, NF, 0, c0), 1, NF, inv, 1, 15, 1.0, X, 1, M);
        /* complement_infomat_block -= X * H.block(c0, 0, 15, M): GEMM with alpha = -1 */
        gemm_acc(M, M, 15, X, 1, M, &EL(H, NF, c0, 0), 1, NF, -1.0, C, 1, M);
        /* complement_infovec_segment -= X * hb.segment(c0, 15): column-major GEMV, alpha = -1 */
        gemv_col(M, 15, X, M, &hb[c0], cv, -1.0);
        free(H); free(hb);
        H = C; hb = cv;      /* pose_motion_infomat = complement_infomat (M x M) */
        free(X);
    }

    /* scope: create marginalization factor */
    {
        const int n = RD_ES_SIZE * last_index;
        double* ev = dz((size_t)n);
        double* V = dz((size_t)n * n);
        double* prod = dz((size_t)n);
        rd_marg nm;
        int info;
        long a, b;
        info = rd_selfadjoint_eig(n, H, ev, V, 0);
        /* info == 1 (NoConvergence after 30 n QL iterations): SelfAdjointEigenSolver does not throw, the unsorted result is used as it is */
        if (info == 2) { free(ev); free(V); free(prod); ret = 3; goto done; }
        marg_alloc(&nm, last_index);
        for (a = 0; a < n; ++a) {
            const double lam = ev[a] > 1.0e-8 ? ev[a] : 0.0;
            const double sl = sqrt(lam);
            for (b = 0; b < n; ++b) nm.sqrt_inv_cov[a + (long)n * b] = sl * V[b + (long)n * a];
        }
        for (a = 0; a < n; ++a) {
            const double li = ev[a] > 1.0e-8 ? 1.0 / ev[a] : 0.0;
            const double sl = sqrt(li);
            /* (sqrt(lambda_inv) * V^T) * vec: the lazy diagonal product is a row-major lhs without direct access, so the GEMV is
             * dest(i) += alpha * (lhs.row(i).cwiseProduct(rhs^T)).sum(): a dynamic LinearVectorized redux (measured: two two-lane accumulators,
             * the same as the squaredNorm / dot redux) of the terms (d V(k,i)) * b_k */
            double acc;
            for (b = 0; b < n; ++b) prod[b] = (sl * V[b + (long)n * a]) * hb[b];
            acc = redux2(prod, (int)n);
            nm.infovec[a] = 0.0 + 1.0 * acc;
        }
        free(prod);
        if (dbg) {
            dbg->n = n;
            dbg->infomat = dz((size_t)n * n); memcpy(dbg->infomat, H, sizeof(double) * (size_t)n * n);
            dbg->infovec = dz((size_t)n); memcpy(dbg->infovec, hb, sizeof(double) * (size_t)n);
            dbg->evals = ev; dbg->evecs = V;
        } else {
            free(ev); free(V);
        }
        k = 0;
        for (i = 0; i < nf_map; ++i) {
            if (i == index) continue;
            nm.ids[k] = fr[i].id;
            nm.lin_pose[k] = fr[i].pose;
            nm.lin_motion[k] = fr[i].motion;
            ++k;
        }
        nm.eig_info = info;
        rd_marg_free(m);
        *m = nm;
    }

done:
    free(H); free(hb); free(sidx); free(fpos); free(order);
    if (lms) {
        for (i = 0; i < nlm; ++i) free(lms[i].h);
        free(lms);
    }
    return ret;
}
