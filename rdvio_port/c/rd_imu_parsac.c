/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port: find_pnp_matrix_parsac_imu / IMU_Parsac. See rd_imu_parsac.h. */
#include "rd_imu_parsac.h"
#include "rd_rand.h"
#include "rd_sys_eigen.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NB 400
#define DOF 6
typedef struct bins {
    size_t nValid;
    float BinHeight, BinWidth;
    double loc[NB][2];
    size_t mapBinToValid[NB], mapValidToBin[NB], validSize[NB];
    size_t* mapDataToValid;
    float validLens[NB], validConf[NB], prior[NB], accPrior[NB + 1];
    double dyn;
} bins;

static float max_f(float a, float b) { return (a < b) ? b : a; }   /* std::max(a, b) */

/* SetBins(20, 20), CreateBucket, BucketData(pts2) with the track lengths, the prior bin confidences */
static int setup(bins* b, const rd_parsac_state* st, size_t n, const double* pts2, const size_t* lens, double scale) {
    size_t i, j, iy;
    float y, x, sum = 0.f, norm;
    b->BinHeight = (float)(2 * scale / 20);
    b->BinWidth = (float)(2 * scale / 20);
    y = (float)(b->BinHeight * 0.5);
    for (i = 0, iy = 0; iy < 20; ++iy, y += b->BinHeight) {
        x = (float)(b->BinWidth * 0.5);
        for (j = 0; j < 20; ++j, x += b->BinWidth, ++i) { b->loc[i][0] = x - scale; b->loc[i][1] = y - scale; }
    }
    b->mapDataToValid = (size_t*)malloc(sizeof(size_t) * (n ? n : 1));
    for (i = 0; i < NB; ++i) b->mapBinToValid[i] = (size_t)-1;
    b->nValid = 0;
    for (i = 0; i < n; ++i) {
        const double px = pts2[2 * i], py = pts2[2 * i + 1];
        size_t iBin, iv;
        if (!(px > -scale && px < scale && py > -scale && py < scale)) { free(b->mapDataToValid); return -1; }
        iBin = (size_t)((px + scale) / (double)b->BinWidth) + 20 * (size_t)((py + scale) / (double)b->BinHeight);
        if (iBin >= NB) { free(b->mapDataToValid); return -1; }
        iv = b->mapBinToValid[iBin];
        if (iv == (size_t)-1) {
            b->mapBinToValid[iBin] = b->nValid;
            b->mapDataToValid[i] = b->nValid;
            b->mapValidToBin[b->nValid] = iBin;
            b->validSize[b->nValid] = 1;
            b->validLens[b->nValid] = (float)lens[i];
            b->nValid++;
        } else {
            b->mapDataToValid[i] = iv;
            b->validSize[iv]++;
            b->validLens[iv] = b->validLens[iv] + (float)lens[i];
        }
    }
    for (i = 0; i < b->nValid; ++i) b->validLens[i] = b->validLens[i] / (float)b->validSize[i];
    for (i = 0; i < b->nValid; ++i) b->prior[i] = st->conf[b->mapValidToBin[i]];
    for (i = 0; i < b->nValid; ++i) { b->prior[i] = max_f(0.5f, b->prior[i]); sum += b->prior[i]; }
    norm = (float)(1.0 / sum);
    for (i = 0; i < b->nValid; ++i) b->prior[i] *= norm;
    b->accPrior[0] = 0;
    for (i = 0; i < b->nValid; ++i) b->accPrior[i + 1] = b->accPrior[i] + b->prior[i];
    norm = 1.f / b->accPrior[b->nValid];
    for (i = 0; i < b->nValid; ++i) b->accPrior[i] *= norm;      /* the last entry stays unnormalised (loop bound N) */
    return 0;
}

/* ComputeScore; inl[v] = inliers in valid bin v */
static float score_of(bins* b, const size_t* inl) {
    float sumC = 0, sqC = 0, norm, Cxx = 0, Cxy = 0, Cyy = 0, imgRatio;
    double sum0 = 0.0, sum1 = 0.0, mean0, mean1;
    size_t v;
    for (v = 0; v < b->nValid; ++v) {
        const float t = (float)(1 - pow(b->dyn, 0.10 * (double)b->validLens[v]));
        const float c = t * (float)inl[v] / (float)b->validSize[v];
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

static void identity4(double T[16]) { memset(T, 0, 16 * sizeof(double)); T[0] = T[5] = T[10] = T[15] = 1.0; }

size_t rd_find_pnp_matrix_parsac_imu(rd_parsac_state* st, size_t n, const double* Xs, const double* xs, const size_t* lens,
                                     const double R[9], const double t[3], double dynamic_prob, double scale, char* mask,
                                     double threshold, double confidence, size_t max_iteration, int seed,
                                     rd_pnp6_fn solve, void* ctx, double T[16]) {
    static const double t2 = 5.99;
    const double th = 2.0 * t2 * threshold * threshold;
    const double K = log(fmax(1 - confidence, 1.0e-5));
    double prior[16], model[16], best[16];
    rd_lotbox lb;
    bins* b;
    char *pm, *cur;
    size_t *cnt_v, *best_cnt_v, idx[DOF], inlier_count = 0, iter, iter_max, i, nprior = 0;
    float scoreMax = -FLT_MAX, score;
    int r, c;
    memset(best, 0, sizeof best);
    if (n < DOF) { memset(mask, 0, n); memcpy(T, best, sizeof best); return 0; }   /* the C++ returns its unset model */
    b = (bins*)malloc(sizeof *b);
    b->dyn = dynamic_prob;
    rd_lotbox_init(&lb, n);
    rd_lotbox_seed(&lb, (unsigned int)seed);
    if (setup(b, st, n, xs, lens, scale)) { free(b); rd_lotbox_free(&lb); return (size_t)-1; }
    rd_glibc_srand(0);                                /* Sampler's constructor */
    /* ComputePriorDistribution with the prior model [R t; 0 1] */
    identity4(prior);
    for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) prior[r + 4 * c] = R[r + 3 * c];
    prior[12] = t[0]; prior[13] = t[1]; prior[14] = t[2];
    pm = (char*)calloc(n, 1);
    for (i = 0; i < n; ++i)
        if (rd_pnp_reproject_error(prior, Xs + 3 * i, xs + 2 * i) <= th * 2.0) { nprior++; pm[i] = 1; }
    if ((double)nprior / (double)n < 0.15 || nprior < 20) {
        memset(mask, 1, n); identity4(T);
        free(pm); free(b->mapDataToValid); free(b); rd_lotbox_free(&lb);
        return 0;
    }
    cur = (char*)malloc(n);
    cnt_v = (size_t*)malloc(sizeof(size_t) * NB); best_cnt_v = (size_t*)calloc(NB, sizeof(size_t));
    iter_max = max_iteration;
    for (iter = 0; iter < iter_max; ++iter) {
        double X[DOF][3], x[DOF][2];
        size_t cnt = 0, overlap = 0;
        int s;
        rd_lotbox_refill_all(&lb);
        for (s = 0; s < DOF; ++s) {
            if (b->nValid > 20) {
                size_t index;
                for (;;) {                            /* Sampler::draw_by_weight + is_sampled over this iteration's draws */
                    const float rr = (float)rd_glibc_rand() / 2147483648.0f;
                    size_t lo = 1, hi = b->nValid + 1, mid, k;
                    int seen = 0;
                    while (lo < hi) { mid = lo + (hi - lo) / 2; if (rr < b->accPrior[mid]) hi = mid; else lo = mid + 1; }
                    index = lo - 1;
                    for (k = 0; k < (size_t)s; ++k) if (idx[k] == index) seen = 1;
                    if (!seen) break;
                }
                idx[s] = index;                       /* a valid-bin index, used as a data index below */
            } else {
                idx[s] = rd_lotbox_draw_without_replacement(&lb);
            }
            memcpy(X[s], Xs + 3 * idx[s], sizeof X[s]);
            memcpy(x[s], xs + 2 * idx[s], sizeof x[s]);
        }
        solve(ctx, (const double(*)[3])X, (const double(*)[2])x, model);
        memset(cur, 0, n);
        for (i = 0; i < n; ++i)
            if (rd_pnp_reproject_error(model, Xs + 3 * i, xs + 2 * i) <= th) { cnt++; cur[i] = 1; }
        for (i = 0; i < n; ++i) if (pm[i] && cur[i]) overlap++;
        if (overlap < DOF) continue;
        memset(cnt_v, 0, sizeof(size_t) * b->nValid);
        for (i = 0; i < n; ++i) if (cur[i] == 1) cnt_v[b->mapDataToValid[i]]++;
        score = score_of(b, cnt_v);
        if (score > scoreMax || (score == scoreMax && overlap > inlier_count)) {
            const double ratio = (double)overlap / (double)n;
            double N;
            scoreMax = score;
            memcpy(best, model, sizeof best);
            inlier_count = overlap;
            memcpy(best_cnt_v, cnt_v, sizeof(size_t) * b->nValid);
            memcpy(mask, cur, n);
            N = K / log(1 - pow(ratio, 5));
            if (N < (double)iter_max) iter_max = (size_t)ceil(N);
        }
        (void)cnt;
    }
    if (inlier_count < DOF) {
        memset(mask, 1, n); identity4(T);
    } else {
        score_of(b, best_cnt_v);                      /* ComputeScore(validBinInliersBest): the confidences of the best model */
        for (i = 0; i < NB; ++i) st->conf[i] = (b->mapBinToValid[i] == (size_t)-1) ? 0.f : b->validConf[b->mapBinToValid[i]];
        memcpy(T, best, sizeof best);
    }
    free(pm); free(cur); free(cnt_v); free(best_cnt_v); free(b->mapDataToValid); free(b); rd_lotbox_free(&lb);
    return inlier_count;
}
