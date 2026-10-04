/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_ransac.h for provenance and licences. */
#include "rd_ransac.h"
#include "rd_geom.h"
#include "rd_rand.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* p2.homogeneous().transpose() * Ep1 : redux of (x*e0, y*e1, 1*e2): measured against the real class (oracle "essgeo") to be the vectorised
 * left fold ((a + b) + c), not the scalar halving tree a + (b + c). */
#define RD_ESS_INNER(a, b, c) (((a) + (b)) + (c))

/* ------------------------------------------------------------ error functions */
double rd_essential_geometric_error(const double E[9], const double p1[2], const double p2[2]) {
    /* Ep1 = E * p1.homogeneous() == E.leftCols(2) * p1 + E.col(2) */
    double Ep1[3], r;
    int i;
    for (i = 0; i < 3; ++i) Ep1[i] = (E[i] * p1[0] + E[i + 3] * p1[1]) + E[i + 6];
    /* p2.homogeneous().transpose() * Ep1 : inner product, unrolled redux of the coefficient products */
    r = RD_ESS_INNER(p2[0] * Ep1[0], p2[1] * Ep1[1], 1.0 * Ep1[2]);
    return r * r / (Ep1[0] * Ep1[0] + Ep1[1] * Ep1[1]);
}

double rd_homography_geometric_error(const double H[9], const double p1[2], const double p2[2]) {
    double v[3], d0, d1;
    int i;
    for (i = 0; i < 3; ++i) v[i] = (H[i] * p1[0] + H[i + 3] * p1[1]) + H[i + 6];
    d0 = p2[0] - v[0] / v[2];
    d1 = p2[1] - v[1] / v[2];
    return d0 * d0 + d1 * d1;
}

double rd_rotation_error(const double R[9], const double p1[3], const double p2[3]) {
    double t[3];
    ok_m3_mulv(R, p1, t);
    return acos((t[0] * p2[0] + t[1] * p2[1]) + t[2] * p2[2]);
}

/* ------------------------------------------------------------------- RANSAC */
typedef struct { int dof; int maxmodels; int model_len; } rd_model_kind;
typedef int (*rd_solve_fn)(const size_t* idx, void* ctx, double* models);      /* returns the number of models, each model_len doubles */
typedef void (*rd_prepare_fn)(const double* model, void* ctx);
typedef double (*rd_eval_fn)(void* ctx, size_t i);

typedef struct {
    const double *p1, *p2;
    int d1, d2;                 /* doubles per point */
    double E[9], Et[9], H[9], Hinv[9], R[9];
} rd_geo_ctx;

static int solve_ess(const size_t* idx, void* c, double* models) {
    rd_geo_ctx* g = (rd_geo_ctx*)c;
    double a[5][2], b[5][2], E[10][9];
    int i, n;
    for (i = 0; i < 5; ++i) { a[i][0] = g->p1[2 * idx[i]]; a[i][1] = g->p1[2 * idx[i] + 1]; b[i][0] = g->p2[2 * idx[i]]; b[i][1] = g->p2[2 * idx[i] + 1]; }
    n = rd_solve_essential_5pt((const double(*)[2])a, (const double(*)[2])b, E);
    if (n < 0) n = 0;
    memcpy(models, E, sizeof(double) * 9 * (size_t)n);
    return n;
}
static void prep_ess(const double* m, void* c) {
    rd_geo_ctx* g = (rd_geo_ctx*)c; int i, j;
    memcpy(g->E, m, 72);
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) g->Et[i + 3 * j] = m[j + 3 * i];
}
static double eval_ess(void* c, size_t i) {
    rd_geo_ctx* g = (rd_geo_ctx*)c;
    return rd_essential_geometric_error(g->E, g->p1 + 2 * i, g->p2 + 2 * i) + rd_essential_geometric_error(g->Et, g->p2 + 2 * i, g->p1 + 2 * i);
}
static int solve_rot(const size_t* idx, void* c, double* models) {
    rd_geo_ctx* g = (rd_geo_ctx*)c;
    double a[2][3], b[2][3];
    int i, k;
    for (i = 0; i < 2; ++i) for (k = 0; k < 3; ++k) { a[i][k] = g->p1[3 * idx[i] + k]; b[i][k] = g->p2[3 * idx[i] + k]; }
    rd_solve_rotation_2pt((const double(*)[3])a, (const double(*)[3])b, models);
    return 1;
}
static void prep_rot(const double* m, void* c) { memcpy(((rd_geo_ctx*)c)->R, m, 72); }
static double eval_rot(void* c, size_t i) { rd_geo_ctx* g = (rd_geo_ctx*)c; return rd_rotation_error(g->R, g->p1 + 3 * i, g->p2 + 3 * i); }
static int solve_hom(const size_t* idx, void* c, double* models) {
    rd_geo_ctx* g = (rd_geo_ctx*)c;
    double a[4][2], b[4][2];
    int i;
    for (i = 0; i < 4; ++i) { a[i][0] = g->p1[2 * idx[i]]; a[i][1] = g->p1[2 * idx[i] + 1]; b[i][0] = g->p2[2 * idx[i]]; b[i][1] = g->p2[2 * idx[i] + 1]; }
    rd_solve_homography_4pt((const double(*)[2])a, (const double(*)[2])b, models);
    return 1;
}
static void prep_hom(const double* m, void* c) { rd_geo_ctx* g = (rd_geo_ctx*)c; memcpy(g->H, m, 72); rd_inverse3(m, g->Hinv); }
static double eval_hom(void* c, size_t i) {
    rd_geo_ctx* g = (rd_geo_ctx*)c;
    return rd_homography_geometric_error(g->H, g->p1 + 2 * i, g->p2 + 2 * i) + rd_homography_geometric_error(g->Hinv, g->p2 + 2 * i, g->p1 + 2 * i);
}

/* Ransac<DoF, ...>::solve. models_cap = most hypotheses one sample can give (10). */
static size_t ransac_run(int dof, rd_solve_fn solve, rd_prepare_fn prepare, rd_eval_fn eval, void* ctx, size_t n, char* mask, double threshold,
                         double confidence, size_t max_iteration, int seed, double* model_out) {
    rd_lotbox lb;
    double models[10 * 9], best[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    size_t idx[5], inlier_count = 0, iter, iter_max, i;
    char* cur;
    const double K = log(fmax(1 - confidence, 1.0e-5));
    memset(mask, 0, n);
    memcpy(model_out, best, 72);
    if (n < (size_t)dof) return 0;
    rd_lotbox_init(&lb, n);
    rd_lotbox_seed(&lb, (unsigned int)seed);
    cur = (char*)malloc(n ? n : 1);
    iter_max = max_iteration;
    for (iter = 0; iter < iter_max; ++iter) {
        int nm, m, s;
        rd_lotbox_refill_all(&lb);
        for (s = 0; s < dof; ++s) idx[s] = rd_lotbox_draw_without_replacement(&lb);
        nm = solve(idx, ctx, models);
        for (m = 0; m < nm; ++m) {
            size_t cnt = 0;
            memset(cur, 0, n);
            prepare(models + 9 * m, ctx);
            for (i = 0; i < n; ++i) {
                const double error = eval(ctx, i);
                if (error <= threshold) { cnt++; cur[i] = 1; }
            }
            if (cnt > inlier_count) {
                const double ratio = (double)cnt / (double)n;
                double N;
                memcpy(best, models + 9 * m, 72);
                inlier_count = cnt;
                memcpy(mask, cur, n);
                N = K / log(1 - pow(ratio, 5));
                if (N < (double)iter_max) iter_max = (size_t)ceil(N);
            }
        }
    }
    memcpy(model_out, best, 72);
    free(cur);
    rd_lotbox_free(&lb);
    return inlier_count;
}

size_t rd_find_essential_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                                size_t max_iteration, int seed, double E[9]) {
    rd_geo_ctx g; static const double t1 = 3.84;
    g.p1 = p1; g.p2 = p2;
    return ransac_run(5, solve_ess, prep_ess, eval_ess, &g, n, mask, 2.0 * t1 * threshold * threshold, confidence, max_iteration, seed, E);
}
size_t rd_find_rotation_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                               size_t max_iteration, int seed, double R[9]) {
    rd_geo_ctx g; static const double t2 = 5.99;
    g.p1 = p1; g.p2 = p2;
    return ransac_run(2, solve_rot, prep_rot, eval_rot, &g, n, mask, t2 * threshold * threshold, confidence, max_iteration, seed, R);
}
size_t rd_find_homography_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                                 size_t max_iteration, int seed, double H[9]) {
    rd_geo_ctx g; static const double t2 = 5.99;
    g.p1 = p1; g.p2 = p2;
    return ransac_run(4, solve_hom, prep_hom, eval_hom, &g, n, mask, 2.0 * t2 * threshold * threshold, confidence, max_iteration, seed, H);
}

/* ------------------------------------------------------------------- PARSAC */
#define NB 400
typedef struct {
    size_t nValid;
    float BinHeight, BinWidth;
    double loc[NB][2];
    size_t mapBinToValid[NB];
    size_t mapValidToBin[NB];
    size_t* mapDataToValid;    /* n */
    size_t validSize[NB];
    float validConf[NB];
    float prior[NB];
    float accPrior[NB + 1];
} rd_parsac_bins;

static float cmp_max_f(float a, float b) { return (a < b) ? b : a; }   /* std::max(a, b) */

static int parsac_setup(rd_parsac_bins* b, const rd_parsac_state* st, size_t n, const double* pts2) {
    const double norm_scale = 1.0;
    size_t i, j, ix, iy;
    float y, x, sum = 0.f, norm;
    b->BinHeight = (float)(2 * norm_scale / 20);
    b->BinWidth = (float)(2 * norm_scale / 20);
    /* CreateBucket */
    y = (float)(b->BinHeight * 0.5);
    for (i = 0, iy = 0; iy < 20; ++iy, y += b->BinHeight) {
        x = (float)(b->BinWidth * 0.5);
        for (j = 0; j < 20; ++j, x += b->BinWidth, ++i) { b->loc[i][0] = x - norm_scale; b->loc[i][1] = y - norm_scale; }
    }
    /* BucketData */
    b->mapDataToValid = (size_t*)malloc(sizeof(size_t) * (n ? n : 1));
    for (i = 0; i < NB; ++i) b->mapBinToValid[i] = (size_t)-1;
    b->nValid = 0;
    for (i = 0; i < n; ++i) {
        const double px = pts2[2 * i], py = pts2[2 * i + 1];
        size_t iBin, iv;
        if (!(px > -norm_scale && px < norm_scale && py > -norm_scale && py < norm_scale)) { free(b->mapDataToValid); return -1; }
        ix = (size_t)((px + norm_scale) / (double)b->BinWidth);
        iy = (size_t)((py + norm_scale) / (double)b->BinHeight);
        iBin = ix + 20 * iy;
        if (iBin >= NB) { free(b->mapDataToValid); return -1; }
        iv = b->mapBinToValid[iBin];
        if (iv == (size_t)-1) {
            b->mapBinToValid[iBin] = b->nValid;
            b->mapDataToValid[i] = b->nValid;
            b->mapValidToBin[b->nValid] = iBin;
            b->validSize[b->nValid] = 1;
            b->nValid++;
        } else {
            b->mapDataToValid[i] = iv;
            b->validSize[iv]++;
        }
    }
    /* ConvertConfidencesBinToValidBin, ThresholdAndNormalizeConfidences, AccumulateConfidences */
    for (i = 0; i < b->nValid; ++i) b->prior[i] = st->conf[b->mapValidToBin[i]];
    for (i = 0; i < b->nValid; ++i) { b->prior[i] = cmp_max_f(0.5f, b->prior[i]); sum += b->prior[i]; }
    norm = (float)(1.0 / sum);
    for (i = 0; i < b->nValid; ++i) b->prior[i] *= norm;
    b->accPrior[0] = 0;
    for (i = 0; i < b->nValid; ++i) b->accPrior[i + 1] = b->accPrior[i] + b->prior[i];
    norm = 1.f / b->accPrior[b->nValid];
    for (i = 0; i < b->nValid; ++i) b->accPrior[i] *= norm;     /* the last entry stays unnormalised, as in the C++ loop bound */
    return 0;
}

/* ComputeScore; inl_count[v] = number of inliers in valid bin v */
static float parsac_score(rd_parsac_bins* b, const size_t* inl_count) {
    float sumC = 0, sqC = 0, norm, Cxx = 0, Cxy = 0, Cyy = 0, imgRatio;
    double sum0 = 0.0, sum1 = 0.0, mean0, mean1;
    size_t v;
    for (v = 0; v < b->nValid; ++v) {
        const float c = (float)inl_count[v] / (float)b->validSize[v];
        const double x0 = b->loc[b->mapValidToBin[v]][0] * (double)c, x1 = b->loc[b->mapValidToBin[v]][1] * (double)c;
        b->validConf[v] = c;
        sum0 += x0; sum1 += x1;
        sumC += c;
        sqC += c * c;
    }
    norm = 1.f / sumC;
    mean0 = sum0 * (double)norm; mean1 = sum1 * (double)norm;
    for (v = 0; v < b->nValid; ++v) {
        const float c = b->validConf[v];
        const double dx0 = b->loc[b->mapValidToBin[v]][0] - mean0, dx1 = b->loc[b->mapValidToBin[v]][1] - mean1;
        Cxx = (float)((double)Cxx + (dx0 * dx0) * (double)c);
        Cxy = (float)((double)Cxy + (dx0 * dx1) * (double)c);
        Cyy = (float)((double)Cyy + (dx1 * dx1) * (double)c);
    }
    norm = sumC / (sumC * sumC - sqC);
    imgRatio = norm * sqrtf(Cxx * Cyy - Cxy * Cxy);
    return imgRatio * sumC;
}

static size_t parsac_run(int dof, rd_solve_fn solve, rd_prepare_fn prepare, rd_eval_fn eval, void* ctx, rd_parsac_state* st, size_t n,
                         const double* pts2, char* mask, double threshold, double confidence, size_t max_iteration, int seed, double* model_out) {
    rd_lotbox lb;
    rd_parsac_bins* b;
    double models[10 * 9], best[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    size_t idx[5], inlier_count = 0, iter, iter_max, i;
    size_t* cnt_v; size_t* best_cnt_v;
    char* cur;
    float scoreMax = 0, score;
    const double K = log(fmax(1 - confidence, 1.0e-5));
    memset(mask, 0, n);
    memcpy(model_out, best, 72);
    if (n < (size_t)dof) return 0;
    b = (rd_parsac_bins*)malloc(sizeof *b);
    if (parsac_setup(b, st, n, pts2)) { free(b); return (size_t)-1; }
    rd_lotbox_init(&lb, n);
    rd_lotbox_seed(&lb, (unsigned int)seed);
    rd_glibc_srand(0);       /* Sampler's constructor */
    cur = (char*)malloc(n ? n : 1);
    cnt_v = (size_t*)malloc(sizeof(size_t) * NB); best_cnt_v = (size_t*)calloc(NB, sizeof(size_t));
    iter_max = max_iteration;
    for (iter = 0; iter < iter_max; ++iter) {
        int nm, m, s;
        rd_lotbox_refill_all(&lb);
        for (s = 0; s < dof; ++s) {
            if (b->nValid > 20) {
                size_t index;
                for (;;) {   /* Sampler::draw_by_weight (+ is_sampled over the bins drawn so far in this iteration) */
                    const float r = (float)rd_glibc_rand() / 2147483648.0f;
                    size_t lo = 1, hi = b->nValid + 1, mid, k;
                    int seen = 0;
                    while (lo < hi) { mid = lo + (hi - lo) / 2; if (r < b->accPrior[mid]) hi = mid; else lo = mid + 1; }   /* upper_bound over [1, nValid+1) */
                    index = lo - 1;
                    for (k = 0; k < (size_t)s; ++k) if (idx[k] == index) seen = 1;
                    if (!seen) break;
                }
                idx[s] = index;
            } else {
                idx[s] = rd_lotbox_draw_without_replacement(&lb);
            }
        }
        nm = solve(idx, ctx, models);
        for (m = 0; m < nm; ++m) {
            size_t cnt = 0;
            memset(cur, 0, n);
            prepare(models + 9 * m, ctx);
            for (i = 0; i < n; ++i) {
                const double error = eval(ctx, i);
                if (error <= threshold) { cnt++; cur[i] = 1; }
            }
            memset(cnt_v, 0, sizeof(size_t) * b->nValid);
            for (i = 0; i < n; ++i) if (cur[i] == 1) cnt_v[b->mapDataToValid[i]]++;
            score = parsac_score(b, cnt_v);
            if (score > scoreMax || (score == scoreMax && (cnt > inlier_count))) {
                const double ratio = (double)cnt / (double)n;
                double N;
                scoreMax = score;
                memcpy(best, models + 9 * m, 72);
                inlier_count = cnt;
                memcpy(best_cnt_v, cnt_v, sizeof(size_t) * b->nValid);
                memcpy(mask, cur, n);
                N = K / log(1 - pow(ratio, 5));
                if (N < (double)iter_max) iter_max = (size_t)ceil(N);
            }
        }
    }
    parsac_score(b, best_cnt_v);   /* ComputeScore(validBinInliersBest, ...): leaves m_validBinConfidences = confidences of the best model */
    for (i = 0; i < NB; ++i) st->conf[i] = (b->mapBinToValid[i] == (size_t)-1) ? 0.f : b->validConf[b->mapBinToValid[i]];
    memcpy(model_out, best, 72);
    free(cur); free(cnt_v); free(best_cnt_v); free(b->mapDataToValid); free(b); rd_lotbox_free(&lb);
    return inlier_count;
}

void rd_parsac_state_init(rd_parsac_state* s) { int i; for (i = 0; i < NB; ++i) s->conf[i] = 0.5f; }

size_t rd_find_essential_matrix_parsac(rd_parsac_state* st, size_t n, const double* p1, const double* p2, char* mask, double threshold,
                                       double confidence, size_t max_iteration, int seed, double E[9]) {
    rd_geo_ctx g; static const double t1 = 3.84;
    g.p1 = p1; g.p2 = p2;
    return parsac_run(5, solve_ess, prep_ess, eval_ess, &g, st, n, p2, mask, 2.0 * t1 * threshold * threshold, confidence, max_iteration, seed, E);
}
size_t rd_find_homography_matrix_parsac(rd_parsac_state* st, size_t n, const double* p1, const double* p2, char* mask, double threshold,
                                        double confidence, size_t max_iteration, int seed, double H[9]) {
    rd_geo_ctx g; static const double t2 = 5.99;
    g.p1 = p1; g.p2 = p2;
    return parsac_run(4, solve_hom, prep_hom, eval_hom, &g, st, n, p2, mask, 2.0 * t2 * threshold * threshold, confidence, max_iteration, seed, H);
}
