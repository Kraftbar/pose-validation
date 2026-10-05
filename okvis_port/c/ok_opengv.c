/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 7c: OpenGV GP3P / rotation-only / Stewenius + the Ransac loop. See ok_opengv.h for the notices. */
#include "ok_opengv.h"
#include "ok_eigen.h"
#include "ok_eigen_eigsolver.h"
#include "ok_eigen_fullpivlu.h"
#include "ok_eigen_svd.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------------------------
 * std::mt19937(12345) and std::uniform_int_distribution<int>(0, INT_MAX) (libstdc++, GCC 13)
 * ---------------------------------------------------------------------------------------------------------------- */
void ok_og_rng_seed(ok_og_rng* e, uint32_t seed) {
    unsigned i;
    e->state[0] = seed;
    for (i = 1; i < 624; ++i) e->state[i] = (1812433253U * (e->state[i - 1] ^ (e->state[i - 1] >> 30)) + i);
    e->idx = 624;
}
static void rng_regen(ok_og_rng* e) {
    static const uint32_t mag01[2] = {0U, 0x9908b0dfU};
    unsigned kk;
    uint32_t* mt = e->state;
    for (kk = 0; kk < 624 - 397; ++kk) {
        const uint32_t y = (mt[kk] & 0x80000000U) | (mt[kk + 1] & 0x7fffffffU);
        mt[kk] = mt[kk + 397] ^ (y >> 1) ^ mag01[y & 1U];
    }
    for (; kk < 624 - 1; ++kk) {
        const uint32_t y = (mt[kk] & 0x80000000U) | (mt[kk + 1] & 0x7fffffffU);
        mt[kk] = mt[kk + (397 - 624)] ^ (y >> 1) ^ mag01[y & 1U];
    }
    {
        const uint32_t y = (mt[624 - 1] & 0x80000000U) | (mt[0] & 0x7fffffffU);
        mt[624 - 1] = mt[397 - 1] ^ (y >> 1) ^ mag01[y & 1U];
    }
    e->idx = 0;
}
uint32_t ok_og_rng_next(ok_og_rng* e) {
    uint32_t y;
    if (e->idx >= 624) rng_regen(e);
    y = e->state[e->idx++];
    y ^= (y >> 11);
    y ^= (y << 7) & 0x9d2c5680U;
    y ^= (y << 15) & 0xefc60000U;
    y ^= (y >> 18);
    return y;
}
/* urange = INT_MAX < urngrange = 2^32 - 1: the downscaling branch with Lemire's nearly divisionless method on 32 bit words
 * (_S_nd<uint32_t>), range = urange + 1 = 2^31 */
int ok_og_rng_rnd(ok_og_rng* e) {
    const uint32_t range = 0x80000000U;
    uint64_t product = (uint64_t)ok_og_rng_next(e) * (uint64_t)range;
    uint32_t low = (uint32_t)product;
    if (low < range) {
        const uint32_t threshold = (uint32_t)(0U - range) % range;
        while (low < threshold) {
            product = (uint64_t)ok_og_rng_next(e) * (uint64_t)range;
            low = (uint32_t)product;
        }
    }
    return (int)(uint32_t)(product >> 32);
}

/* ------------------------------------------------------------------------------------------------------------------
 * small Eigen expression models (see HANDOVER: which operand shapes vectorise and therefore how a sum associates)
 * ---------------------------------------------------------------------------------------------------------------- */
/* Matrix<double,3,4> * Vector4d into a plain Vector3d: rows 0-1 packet (left fold), row 2 scalar ((p0 + p1) + (p2 + p3)) */
static void m34_mulv4(const double a[12], const double b[4], double out[3]) {
    double r[3];
    int i;
    for (i = 0; i < 2; ++i) {
        double s = a[i] * b[0];
        s = s + a[i + 3] * b[1];
        s = s + a[i + 6] * b[2];
        s = s + a[i + 9] * b[3];
        r[i] = s;
    }
    r[2] = (a[2] * b[0] + a[5] * b[1]) + (a[8] * b[2] + a[11] * b[3]);
    memcpy(out, r, sizeof r);
}
static double dot3(const double a[3], const double b[3]) { return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]; }
static double stdmax(double a, double b) { return (a < b) ? b : a; }
static double stdmin(double a, double b) { return (b < a) ? b : a; }

/* ------------------------------------------------------------------------------------------------------------------
 * GP3P: opengv::absolute_pose::modules::gp3p_main
 * ---------------------------------------------------------------------------------------------------------------- */
static void cayley2rot(const double c[3], double R[9]) {   /* math::cayley2rot, rotation_t column-major */
    const double scale = 1 + c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    double r[9];
    int i;
    r[0] = 1 + c[0] * c[0] - c[1] * c[1] - c[2] * c[2];
    r[3] = 2 * (c[0] * c[1] - c[2]);
    r[6] = 2 * (c[0] * c[2] + c[1]);
    r[1] = 2 * (c[0] * c[1] + c[2]);
    r[4] = 1 - c[0] * c[0] + c[1] * c[1] - c[2] * c[2];
    r[7] = 2 * (c[1] * c[2] - c[0]);
    r[2] = 2 * (c[0] * c[2] - c[1]);
    r[5] = 2 * (c[1] * c[2] + c[0]);
    r[8] = 1 - c[0] * c[0] - c[1] * c[1] + c[2] * c[2];
    {
        const double s = 1 / scale;
        for (i = 0; i < 9; ++i) R[i] = s * r[i];
    }
}

long ok_og_eig_failures = 0;

int ok_og_gp3p_main(const double f[9], const double v[9], const double p[9], double sols[8][12]) {
    static double G[48 * 85];
    double M[64], er[8], ei[8], vr[64], vi[64];
    int nsol = 0, c, i, j;
    memset(G, 0, sizeof G);
    ok_og_gp3p_init(G, f, v, p);
    ok_og_gp3p_compute(G);
    memset(M, 0, sizeof M);
    for (j = 0; j < 8; ++j)
        for (i = 0; i < 6; ++i) M[i + 8 * j] = -G[(36 + i) * 85 + 77 + j];
    M[6 + 8 * 0] = 1.0;
    M[7 + 8 * 6] = 1.0;
    if (ok_eigensolver8(M, er, ei, vr, vi)) { ++ok_og_eig_failures; return 0; }
    for (c = 0; c < 8; ++c) {
        double cay[3], n[3], R[9], Rt[9], center_cam[3] = {0, 0, 0}, center_world[3] = {0, 0, 0}, T[12];
        if (!(ei[c] < 0.0001)) continue;
        for (i = 0; i < 3; ++i) {
            double re, im;
            ok_cdiv(vr[(i + 4) + 8 * c], vi[(i + 4) + 8 * c], vr[7 + 8 * c], vi[7 + 8 * c], &re, &im);
            cay[2 - i] = re;
            ok_cdiv(vr[(i + 1) + 8 * c], vi[(i + 1) + 8 * c], vr[7 + 8 * c], vi[7 + 8 * c], &re, &im);
            n[2 - i] = re;
        }
        cayley2rot(cay, R);
        for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) Rt[i + 3 * j] = R[j + 3 * i];       /* transposeInPlace */
        for (i = 0; i < 3; ++i) {
            double rhs[3], temp[3];
            int k;
            for (k = 0; k < 3; ++k) rhs[k] = n[i] * f[k + 3 * i] + v[k + 3 * i];
            ok_m3_mulv(Rt, rhs, temp);
            for (k = 0; k < 3; ++k) { center_cam[k] = center_cam[k] + temp[k]; center_world[k] = center_world[k] + p[k + 3 * i]; }
        }
        for (i = 0; i < 3; ++i) {
            center_cam[i] = center_cam[i] / 3.0;
            center_world[i] = center_world[i] / 3.0;
        }
        for (i = 0; i < 9; ++i) T[i] = Rt[i];
        for (i = 0; i < 3; ++i) T[9 + i] = center_world[i] - center_cam[i];
        memcpy(sols[nsol++], T, sizeof T);
    }
    return nsol;
}

/* ------------------------------------------------------------------------------------------------------------------
 * AbsolutePoseSacProblem (GP3P) as used by OKVIS2's FrameAbsolutePoseSacProblem
 * ---------------------------------------------------------------------------------------------------------------- */
/* inverseSolution = [R^T | -R^T t] */
static void invert_model(const double model[12], double inv[12]) {
    double t[3], r[3];
    int i, j;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) inv[i + 3 * j] = model[j + 3 * i];
    for (i = 0; i < 3; ++i) t[i] = model[9 + i];
    {
        double neg[9];
        for (i = 0; i < 9; ++i) neg[i] = -inv[i];
        ok_m3_mulv(neg, t, r);       /* the product is evaluated into a plain temporary: rows 0-1 packet, row 2 scalar (measured) */
    }
    for (i = 0; i < 3; ++i) inv[9 + i] = r[i];
}
/* reprojection of point idx into camera idx: R_c^T (inv * (p, 1) - offset) / norm */
static void abs_reproject(const ok_og_abs* a, const double inv[12], int i, double out[3]) {
    double ph[4], body[3], d[3], rep[3], nrm;
    int k;
    for (k = 0; k < 3; ++k) ph[k] = a->point[3 * i + k];
    ph[3] = 1.0;
    m34_mulv4(inv, ph, body);
    for (k = 0; k < 3; ++k) d[k] = body[k] - a->offset[3 * i + k];
    ok_m3_mulv_lhsT(a->rot + 9 * i, d, rep);
    nrm = ok_v3_norm(rep);
    for (k = 0; k < 3; ++k) out[k] = rep[k] / nrm;
}

int ok_og_abs_model(const ok_og_abs* a, const int idx[4], double model[12]) {
    double f[9], v[9], p[9], sols[8][12];
    int i, k, nsol, minIndex = -1;
    double minScore = 1000000.0;
    for (i = 0; i < 3; ++i) {
        double col[3], rc[3];
        for (k = 0; k < 3; ++k) col[k] = a->bearing[3 * idx[i] + k];
        ok_m3_mulv(a->rot + 9 * idx[i], col, rc);                 /* f.col(i) = R * f.col(i): aliasing -> plain temporary */
        for (k = 0; k < 3; ++k) {
            f[k + 3 * i] = rc[k];
            v[k + 3 * i] = a->offset[3 * idx[i] + k];
            p[k + 3 * i] = a->point[3 * idx[i] + k];
        }
    }
    nsol = ok_og_gp3p_main(f, v, p, sols);
    if (nsol == 1) { memcpy(model, sols[0], sizeof(double) * 12); return 1; }
    for (i = 0; i < nsol; ++i) {
        double inv[12], rep[3], bearing[3], score;
        invert_model(sols[i], inv);
        abs_reproject(a, inv, idx[3], rep);
        for (k = 0; k < 3; ++k) bearing[k] = a->bearing[3 * idx[3] + k];
        score = 1.0 - dot3(rep, bearing);
        if (score < minScore) { minScore = score; minIndex = i; }
    }
    if (minIndex == -1) return 0;
    memcpy(model, sols[minIndex], sizeof(double) * 12);
    return 1;
}

void ok_og_abs_scores(const ok_og_abs* a, const double model[12], double* scores) {
    double inv[12];
    int i, k;
    invert_model(model, inv);
    for (i = 0; i < a->n; ++i) {
        double rep[3], err[3], es;
        abs_reproject(a, inv, i, rep);
        for (k = 0; k < 3; ++k) err[k] = rep[k] - a->bearing[3 * i + k];
        es = dot3(err, err);
        scores[i] = es / a->sigma[i];
    }
}

/* ------------------------------------------------------------------------------------------------------------------
 * Ransac::computeModel with a SampleConsensusProblem (shuffled-index sampling with the bound mt19937(12345))
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct sac_problem {
    int n, sample_size, model_len;
    const void* adapter;
    int (*compute)(const void* adapter, const int* sample, double* model);
    void (*scores)(const void* adapter, const double* model, double* scores);
} sac_problem;

static int sac_run(const sac_problem* P, double threshold, int max_iterations, ok_og_result* out) {
    const double probability = 0.99;
    ok_og_rng rng;
    int* shuffled = (int*)malloc(sizeof(int) * (size_t)P->n);
    double* dist = (double*)malloc(sizeof(double) * (size_t)P->n);
    int sample[16];
    double model[16], best_model[16];
    int best_sample[16], have_model = 0, iterations = 0, n_best = -INT_MAX;
    double k = 1.0;
    unsigned skipped = 0;
    const long eig_failures0 = ok_og_eig_failures;
    const unsigned max_skip = (unsigned)max_iterations * 10;
    int i, s, ret;
    memset(model, 0, sizeof model); memset(best_model, 0, sizeof best_model);
    ok_og_rng_seed(&rng, 12345u);
    for (i = 0; i < P->n; ++i) shuffled[i] = i;
    out->inliers = NULL; out->ninliers = 0;
    while ((double)iterations < k && skipped < max_skip) {
        int n_in = 0;
        /* getSamples -> drawIndexSample (isSampleGood is always true) */
        for (s = 0; s < P->sample_size; ++s) {
            const size_t index_size = (size_t)P->n;
            const size_t j = (size_t)s + ((size_t)ok_og_rng_rnd(&rng) % (index_size - (size_t)s));
            const int tmp = shuffled[s]; shuffled[s] = shuffled[j]; shuffled[j] = tmp;
        }
        for (s = 0; s < P->sample_size; ++s) sample[s] = shuffled[s];
        if (!P->compute(P->adapter, sample, model)) { ++skipped; continue; }
        P->scores(P->adapter, model, dist);
        for (i = 0; i < P->n; ++i) if (dist[i] < threshold) ++n_in;
        if (n_in > n_best) {
            double w, p_no_outliers;
            n_best = n_in;
            memcpy(best_sample, sample, sizeof(int) * (size_t)P->sample_size);
            have_model = 1;
            memcpy(best_model, model, sizeof(double) * (size_t)P->model_len);
            w = (double)n_best / (double)P->n;
            p_no_outliers = 1.0 - pow(w, (double)P->sample_size);
            p_no_outliers = stdmax(DBL_EPSILON, p_no_outliers);
            p_no_outliers = stdmin(1.0 - DBL_EPSILON, p_no_outliers);
            k = log(1.0 - probability) / log(p_no_outliers);
        }
        ++iterations;
        if (iterations > max_iterations) break;
    }
    out->iterations = iterations;
    out->degenerate = (int)(ok_og_eig_failures - eig_failures0);
    ret = have_model;
    if (have_model) {
        int cnt = 0;
        P->scores(P->adapter, best_model, dist);
        out->inliers = (int*)malloc(sizeof(int) * (size_t)(P->n ? P->n : 1));
        for (i = 0; i < P->n; ++i) if (dist[i] < threshold) out->inliers[cnt++] = i;
        out->ninliers = cnt;
        memcpy(out->model, best_model, sizeof(double) * (size_t)P->model_len);
    }
    (void)best_sample;
    free(shuffled); free(dist);
    return ret;
}

static int abs_compute_cb(const void* ad, const int* sample, double* model) { return ok_og_abs_model((const ok_og_abs*)ad, sample, model); }
static void abs_scores_cb(const void* ad, const double* model, double* sc) { ok_og_abs_scores((const ok_og_abs*)ad, model, sc); }

int ok_og_ransac_abs(const ok_og_abs* a, double threshold, int max_iterations, ok_og_result* out) {
    sac_problem P;
    int r;
    memset(out, 0, sizeof *out);
    P.n = a->n; P.sample_size = 4; P.model_len = 12; P.adapter = a; P.compute = abs_compute_cb; P.scores = abs_scores_cb;
    r = sac_run(&P, threshold, max_iterations, out);
    out->rows = 3; out->cols = 4;
    return r;
}

/* ------------------------------------------------------------------------------------------------------------------
 * relative pose: triangulate2, rotation-only (twopt_rotationOnly + arun), Stewenius 5-point, CentralRelativePoseSacProblem
 * ---------------------------------------------------------------------------------------------------------------- */
/* A * B^T of 3x3 column-major matrices as Eigen evaluates a Matrix3d product (measured: rows 0-1 left fold, row 2 a0 b0 + (a1 b1 + a2 b2)) */
static void ok_m3_mul_nt(const double A[9], const double B[9], double out[9]) {
    double r[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) {
            const double a0 = A[i], a1 = A[i + 3], a2 = A[i + 6], b0 = B[j], b1 = B[j + 3], b2 = B[j + 6];
            r[i + 3 * j] = (i < 2) ? (a0 * b0 + a1 * b1) + a2 * b2 : a0 * b0 + (a1 * b1 + a2 * b2);
        }
    memcpy(out, r, sizeof r);
}
/* (U * W) * V^T for the SVD factors (MatrixXd) and the constant W of CentralRelativePoseSacProblem */
static void ok_m3_mul_nt_w(const double U[9], const double W[9], const double V[9], double out[9]) {
    double UW[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) UW[i + 3 * j] = U[i] * W[3 * j] + U[i + 3] * W[3 * j + 1] + U[i + 6] * W[3 * j + 2];
    ok_m3_mul_nt(UW, V, out);
}
/* the same through `rotation = U * W * V.transpose()` (an assignment: the product is evaluated into a dynamic temporary whose
 * coefficients are scalar dot products: every row a0 b0 + (a1 b1 + a2 b2)) */
static void ok_m3_mul_nt_w_assign(const double U[9], const double W[9], const double V[9], double out[9]) {
    double UW[9], r[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) UW[i + 3 * j] = U[i] * W[3 * j] + U[i + 3] * W[3 * j + 1] + U[i + 6] * W[3 * j + 2];
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) r[i + 3 * j] = UW[i] * V[j] + (UW[i + 3] * V[j + 3] + UW[i + 6] * V[j + 6]);
    memcpy(out, r, sizeof r);
}
static double det3(const double m[9]) {                       /* Eigen determinant_impl<Matrix3d>, column-major */
    return (m[0] * (m[4] * m[8] - m[7] * m[5])
            - m[1] * (m[3] * m[8] - m[6] * m[5])
            + m[2] * (m[3] * m[7] - m[6] * m[4]));
}

/* opengv::triangulation::triangulate2 for correspondence i with the relative pose (t12, R12) */
static void triangulate2(const ok_og_rel* a, int i, const double t12[3], const double R12[9], double pt[3]) {
    double f1[3], f2[3], f2u[3], b[2], A[4], det, invdet, Ai[4], lambda[2], xm[3], xn[3];
    int k;
    for (k = 0; k < 3; ++k) { f1[k] = a->f1[3 * i + k]; f2[k] = a->f2[3 * i + k]; }
    ok_m3_mulv(R12, f2, f2u);
    b[0] = dot3(t12, f1);
    b[1] = dot3(t12, f2u);
    A[0] = dot3(f1, f1);
    A[1] = dot3(f1, f2u);
    A[2] = -A[1];
    A[3] = -dot3(f2u, f2u);
    det = A[0] * A[3] - A[2] * A[1];                            /* A.inverse(): invdet = 1 / determinant() */
    invdet = 1.0 / det;
    Ai[0] = A[3] * invdet; Ai[1] = -A[1] * invdet; Ai[2] = -A[2] * invdet; Ai[3] = A[0] * invdet;
    lambda[0] = Ai[0] * b[0] + Ai[2] * b[1];
    lambda[1] = Ai[1] * b[0] + Ai[3] * b[1];
    for (k = 0; k < 3; ++k) {
        xm[k] = lambda[0] * f1[k];
        xn[k] = t12[k] + lambda[1] * f2u[k];
        pt[k] = (xm[k] + xn[k]) / 2;
    }
}

/* math::arun: H -> R = V U^T (V' with the third column negated when det < 0) */
static void arun(const double H[9], double R[9]) {
    double U[9], V[9], sv[3];
    ok_eigen_jacobisvd_3x3(H, U, V, sv);
    ok_m3_mul_nt(V, U, R);
    if (det3(R) < 0) {
        double Vp[9];
        memcpy(Vp, V, sizeof Vp);
        Vp[6] = -V[6]; Vp[7] = -V[7]; Vp[8] = -V[8];
        ok_m3_mul_nt(Vp, U, R);
    }
}

int ok_og_rotation_model(const ok_og_rel* a, const int idx[2], double model[9]) {
    double c1[3], c2[3], H[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, f[3], fp[3];
    int k, i, j, p;
    for (k = 0; k < 3; ++k) {
        c1[k] = a->f1[3 * idx[0] + k] + a->f1[3 * idx[1] + k];
        c2[k] = a->f2[3 * idx[0] + k] + a->f2[3 * idx[1] + k];
    }
    for (k = 0; k < 3; ++k) { c1[k] = c1[k] / 3.0; c2[k] = c2[k] / 3.0; }
    for (p = 0; p < 2; ++p) {
        for (k = 0; k < 3; ++k) { f[k] = a->f1[3 * idx[p] + k] - c1[k]; fp[k] = a->f2[3 * idx[p] + k] - c2[k]; }
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) H[i + 3 * j] = H[i + 3 * j] + f[j] * fp[i];   /* Hcross += fprime * f^T */
    }
    arun(H, model);
    return 1;
}

void ok_og_rotation_scores(const ok_og_rel* a, const double model[9], double* scores) {
    int i, k;
    for (i = 0; i < a->n; ++i) {
        double f1[3], f2[3], f2u[3], f1u[3], e1[3], e2[3], es1, es2;
        for (k = 0; k < 3; ++k) { f1[k] = a->f1[3 * i + k]; f2[k] = a->f2[3 * i + k]; }
        ok_m3_mulv(model, f2, f2u);                     /* model * f2 */
        ok_m3_mulv_lhsT(model, f1, f1u);                /* model.transpose() * f1 */
        for (k = 0; k < 3; ++k) { e1[k] = f2u[k] - f1[k]; e2[k] = f1u[k] - f2[k]; }
        es1 = dot3(e1, e1); es2 = dot3(e2, e2);
        scores[i] = es1 * 0.5 / a->s1[i] + es2 * 0.5 / a->s2[i];
    }
}

/* the Stewenius solver: 4 null-space vectors EE (9 x 4) -> the real parts of the 10 (complex) essential matrices, row-major 3x3 */
static int stewenius_main(const double EE[36], double Ereal[10][9]) {
    double A[200], A1[100], A2[100], I[100], inv[100], A3[100], M[100], er[10], ei[10], vr[100], vi[100];
    double sols[4][10][2], evec[9][10][2], nre[10], nim[10];
    ok_eigen_fullpivlu lu;
    int i, j, k, c, r;
    ok_og_stewenius_compose_a(EE, A);
    for (j = 0; j < 10; ++j) for (i = 0; i < 10; ++i) { A1[i + 10 * j] = A[i + 10 * j]; A2[i + 10 * j] = A[i + 10 * (10 + j)]; }
    ok_eigen_fullpivlu_compute(A1, 10, &lu);
    memset(I, 0, sizeof I);
    for (i = 0; i < 10; ++i) I[i + 10 * i] = 1.0;
    ok_eigen_fullpivlu_solve(&lu, I, 10, inv);                  /* luA1.inverse() */
    ok_gemm(10, 10, 10, inv, A2, A3);                           /* A3 = luA1.inverse() * A2 */
    memset(M, 0, sizeof M);
    for (j = 0; j < 10; ++j) {
        for (i = 0; i < 3; ++i) M[i + 10 * j] = -A3[i + 10 * j];
        for (i = 0; i < 2; ++i) M[(3 + i) + 10 * j] = -A3[(4 + i) + 10 * j];
        M[5 + 10 * j] = -A3[7 + 10 * j];
    }
    M[6 + 10 * 0] = 1.0; M[7 + 10 * 1] = 1.0; M[8 + 10 * 3] = 1.0; M[9 + 10 * 6] = 1.0;
    if (ok_eigensolver10(M, er, ei, vr, vi)) return 0;
    for (c = 0; c < 10; ++c) {
        for (r = 0; r < 3; ++r) ok_cdiv(vr[(6 + r) + 10 * c], vi[(6 + r) + 10 * c], vr[9 + 10 * c], vi[9 + 10 * c], &sols[r][c][0], &sols[r][c][1]);
        sols[3][c][0] = 1.0; sols[3][c][1] = 0.0;
    }
    for (c = 0; c < 10; ++c)
        for (r = 0; r < 9; ++r) {                               /* Evec = EE * SOLS (real * complex, left fold over k) */
            double re = EE[r + 9 * 0] * sols[0][c][0], im = EE[r + 9 * 0] * sols[0][c][1];
            for (k = 1; k < 4; ++k) { re = re + EE[r + 9 * k] * sols[k][c][0]; im = im + EE[r + 9 * k] * sols[k][c][1]; }
            evec[r][c][0] = re; evec[r][c][1] = im;
        }
    for (c = 0; c < 10; ++c) {
        double sr = 0.0, si = 0.0;
        for (r = 0; r < 9; ++r) {                               /* norms += pow(Evec, 2) */
            double pr, pi;
            ok_cmul(evec[r][c][0], evec[r][c][1], evec[r][c][0], evec[r][c][1], &pr, &pi);
            sr = sr + pr; si = si + pi;
        }
        ok_csqrt(sr, si, &nre[c], &nim[c]);
    }
    for (c = 0; c < 10; ++c)
        for (r = 0; r < 9; ++r) {
            double re, im;
            ok_cdiv(evec[r][c][0], evec[r][c][1], nre[c], nim[c], &re, &im);
            Ereal[c][r] = re;                                   /* E(r/3, r%3) = Evec(r, c) */
        }
    return 10;
}

/* 3x3 rotation block times a 3x4 helper: transformations are [R | t] column-major 3x4 */
/* opengv::relative_pose::fivept_stewenius(adapter, 5 indices): the real parts of the 10 essential matrices (row-major 3x3 each) */
int ok_og_stewenius_essentials(const ok_og_rel* a, const int idx[5], double Ereal[10][9]) {
    double Q[45], row[9], Vm[81], sv[9], EE[36];
    int i, j, k, rank;
    for (i = 0; i < 5; ++i) {
        const double* f = a->f2 + 3 * idx[i];                    /* Stewenius computes the inverse transformation: inputs swapped */
        const double* fp = a->f1 + 3 * idx[i];
        row[0] = f[0] * fp[0]; row[1] = f[1] * fp[0]; row[2] = f[2] * fp[0];
        row[3] = f[0] * fp[1]; row[4] = f[1] * fp[1]; row[5] = f[2] * fp[1];
        row[6] = f[0] * fp[2]; row[7] = f[1] * fp[2]; row[8] = f[2] * fp[2];
        for (k = 0; k < 9; ++k) Q[i + 5 * k] = row[k];
    }
    ok_eigen_jacobisvd_Nx9_v(Q, 5, Vm, sv, &rank);
    for (j = 0; j < 4; ++j) for (i = 0; i < 9; ++i) EE[i + 9 * j] = Vm[i + 9 * (5 + j)];
    return stewenius_main(EE, Ereal);
}

/* CentralRelativePoseSacProblem::computeModelCoefficients, STEWENIUS (indices: 8) */
int ok_og_stewenius_model(const ok_og_rel* a, const int idx[8], double model[12]) {
    double Ereal[10][9];
    double bestQuality = 1000000.0;
    int bestI = -1, bestJ = -1, i, j, k, r, c, nE;
    static const double W[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};      /* column-major of [[0,-1,0],[1,0,0],[0,0,1]] */
    double Wt[9];
    for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) Wt[r + 3 * c] = W[c + 3 * r];
    nE = ok_og_stewenius_essentials(a, idx, Ereal);
    if (nE == 0) return 0;
    for (i = 0; i < nE; ++i) {
        double E[9], U[9], V[9], s3[3], UW[9], UWt[9], Ra[9], Rb[9], scale, ta[3], tb[3];
        double T[4][12], inv[4][12];
        for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) E[r + 3 * c] = Ereal[i][3 * r + c];
        ok_eigen_jacobisvd_3x3(E, U, V, s3);
        scale = s3[0];
        ok_m3_mul_nt_w(U, W, V, Ra);                            /* U * W * V^T */
        ok_m3_mul_nt_w(U, Wt, V, Rb);                           /* U * W^T * V^T */
        for (k = 0; k < 3; ++k) { ta[k] = scale * U[k + 6]; tb[k] = -ta[k]; }
        if (det3(Ra) < 0) for (k = 0; k < 9; ++k) Ra[k] = -Ra[k];
        if (det3(Rb) < 0) for (k = 0; k < 9; ++k) Rb[k] = -Rb[k];
        (void)UW; (void)UWt;
        for (j = 0; j < 4; ++j) {
            const double* R = (j & 1) ? Rb : Ra;
            const double* t = (j & 2) ? tb : ta;
            memcpy(T[j], R, sizeof(double) * 9);
            memcpy(T[j] + 9, t, sizeof(double) * 3);
            invert_model(T[j], inv[j]);
        }
        for (j = 0; j < 4; ++j) {
            double quality = 0.0;
            for (k = 0; k < 8; ++k) {
                double p[3], ph[4], r1[3], r2[3], n1, n2, f1[3], f2[3], e1, e2;
                int q;
                triangulate2(a, idx[k], T[j] + 9, T[j], p);
                ph[0] = p[0]; ph[1] = p[1]; ph[2] = p[2]; ph[3] = 1.0;
                memcpy(r1, p, sizeof r1);
                m34_mulv4(inv[j], ph, r2);
                n1 = ok_v3_norm(r1); n2 = ok_v3_norm(r2);
                for (q = 0; q < 3; ++q) { r1[q] = r1[q] / n1; r2[q] = r2[q] / n2; f1[q] = a->f1[3 * idx[k] + q]; f2[q] = a->f2[3 * idx[k] + q]; }
                e1 = 1.0 - dot3(f1, r1);
                e2 = 1.0 - dot3(f2, r2);
                quality += e1 + e2;
            }
            if (quality < bestQuality) { bestQuality = quality; bestI = i; bestJ = j; }
        }
    }
    if (bestI == -1) return 0;
    {   /* rederive the best solution (the same SVD again) */
        double E[9], U[9], V[9], s3[3], R[9], t[3], scale;
        for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) E[r + 3 * c] = Ereal[bestI][3 * r + c];
        ok_eigen_jacobisvd_3x3(E, U, V, s3);
        scale = s3[0];
        ok_m3_mul_nt_w_assign(U, (bestJ & 1) ? Wt : W, V, R);
        for (k = 0; k < 3; ++k) t[k] = (bestJ & 2) ? (-scale) * U[k + 6] : scale * U[k + 6];
        if (det3(R) < 0) for (k = 0; k < 9; ++k) R[k] = -R[k];
        memcpy(model, R, sizeof(double) * 9);
        memcpy(model + 9, t, sizeof(double) * 3);
    }
    return 1;
}

void ok_og_stewenius_scores(const ok_og_rel* a, const double model[12], double* scores) {
    double inv[12];
    int i, k;
    invert_model(model, inv);
    for (i = 0; i < a->n; ++i) {
        double p[3], ph[4], r1[3], r2[3], n1, n2, f1[3], f2[3], e1[3], e2[3], es1, es2;
        triangulate2(a, i, model + 9, model, p);
        ph[0] = p[0]; ph[1] = p[1]; ph[2] = p[2]; ph[3] = 1.0;
        memcpy(r1, p, sizeof r1);
        m34_mulv4(inv, ph, r2);
        n1 = ok_v3_norm(r1); n2 = ok_v3_norm(r2);
        for (k = 0; k < 3; ++k) { r1[k] = r1[k] / n1; r2[k] = r2[k] / n2; f1[k] = a->f1[3 * i + k]; f2[k] = a->f2[3 * i + k]; e1[k] = r1[k] - f1[k]; e2[k] = r2[k] - f2[k]; }
        es1 = dot3(e1, e1); es2 = dot3(e2, e2);
        scores[i] = es1 * 0.5 / a->s1[i] + es2 * 0.5 / a->s2[i];
    }
}

static int rot_compute_cb(const void* ad, const int* sample, double* model) { return ok_og_rotation_model((const ok_og_rel*)ad, sample, model); }
static void rot_scores_cb(const void* ad, const double* model, double* sc) { ok_og_rotation_scores((const ok_og_rel*)ad, model, sc); }
static int stew_compute_cb(const void* ad, const int* sample, double* model) { return ok_og_stewenius_model((const ok_og_rel*)ad, sample, model); }
static void stew_scores_cb(const void* ad, const double* model, double* sc) { ok_og_stewenius_scores((const ok_og_rel*)ad, model, sc); }

int ok_og_ransac_rotation(const ok_og_rel* a, double threshold, int max_iterations, ok_og_result* out) {
    sac_problem P;
    int r;
    memset(out, 0, sizeof *out);
    P.n = a->n; P.sample_size = 2; P.model_len = 9; P.adapter = a; P.compute = rot_compute_cb; P.scores = rot_scores_cb;
    r = sac_run(&P, threshold, max_iterations, out);
    out->rows = 3; out->cols = 3;
    return r;
}
int ok_og_ransac_stewenius(const ok_og_rel* a, double threshold, int max_iterations, ok_og_result* out) {
    sac_problem P;
    int r;
    memset(out, 0, sizeof *out);
    P.n = a->n; P.sample_size = 8; P.model_len = 12; P.adapter = a; P.compute = stew_compute_cb; P.scores = stew_scores_cb;
    r = sac_run(&P, threshold, max_iterations, out);
    out->rows = 3; out->cols = 4;
    return r;
}
