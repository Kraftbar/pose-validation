/* SPDX-License-Identifier: MIT */
/* Rotation-only two-view estimation, see sv_rot.h. */
#include "sv_rot.h"
#include "sv_frame.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ROT_MAX_HAMM 80u

/* ---- Horn's closed-form absolute orientation of unit vectors ---- */

/* eigen-decomposition of a symmetric 4x4 (cyclic Jacobi); v columns are the eigenvectors, v[4*r + c] */
static void jacobi4(double A[16], double v[16], double ev[4]) {
    int sweep, p, q, r;
    for (r = 0; r < 16; ++r) {
        v[r] = (r % 5 == 0) ? 1.0 : 0.0;
    }
    for (sweep = 0; sweep < 50; ++sweep) {
        double off = 0.0;
        for (p = 0; p < 4; ++p) {
            for (q = p + 1; q < 4; ++q) {
                off += A[4 * p + q] * A[4 * p + q];
            }
        }
        if (off < 1e-30) {
            break;
        }
        for (p = 0; p < 3; ++p) {
            for (q = p + 1; q < 4; ++q) {
                const double apq = A[4 * p + q];
                double theta, t, c, s;
                if (fabs(apq) < 1e-300) {
                    continue;
                }
                theta = (A[4 * q + q] - A[4 * p + p]) / (2.0 * apq);
                t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
                c = 1.0 / sqrt(t * t + 1.0);
                s = t * c;
                for (r = 0; r < 4; ++r) { /* A <- A J */
                    const double arp = A[4 * r + p], arq = A[4 * r + q];
                    A[4 * r + p] = c * arp - s * arq;
                    A[4 * r + q] = s * arp + c * arq;
                }
                for (r = 0; r < 4; ++r) { /* A <- J^T A */
                    const double apr = A[4 * p + r], aqr = A[4 * q + r];
                    A[4 * p + r] = c * apr - s * aqr;
                    A[4 * q + r] = s * apr + c * aqr;
                }
                for (r = 0; r < 4; ++r) {
                    const double vrp = v[4 * r + p], vrq = v[4 * r + q];
                    v[4 * r + p] = c * vrp - s * vrq;
                    v[4 * r + q] = s * vrp + c * vrq;
                }
            }
        }
    }
    for (r = 0; r < 4; ++r) {
        ev[r] = A[4 * r + r];
    }
}

int sv_rot_fit(const double* a, const double* b, unsigned int n, double R[9]) {
    double S[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, N[16], V[16], ev[4];
    double qw, qx, qy, qz, nrm;
    unsigned int i;
    int best, k;
    for (i = 0; i < n; ++i) {
        for (k = 0; k < 3; ++k) {
            int j;
            for (j = 0; j < 3; ++j) {
                S[3 * k + j] += a[3 * i + k] * b[3 * i + j]; /* S_kj = sum a_k b_j */
            }
        }
    }
    N[0] = S[0] + S[4] + S[8];
    N[1] = S[5] - S[7];
    N[2] = S[6] - S[2];
    N[3] = S[1] - S[3];
    N[5] = S[0] - S[4] - S[8];
    N[6] = S[1] + S[3];
    N[7] = S[6] + S[2];
    N[10] = -S[0] + S[4] - S[8];
    N[11] = S[5] + S[7];
    N[15] = -S[0] - S[4] + S[8];
    N[4] = N[1];
    N[8] = N[2];
    N[12] = N[3];
    N[9] = N[6];
    N[13] = N[7];
    N[14] = N[11];
    jacobi4(N, V, ev);
    best = 0;
    for (k = 1; k < 4; ++k) {
        if (ev[k] > ev[best]) {
            best = k;
        }
    }
    qw = V[4 * 0 + best];
    qx = V[4 * 1 + best];
    qy = V[4 * 2 + best];
    qz = V[4 * 3 + best];
    nrm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (!(nrm > 0.0)) {
        return -1;
    }
    qw /= nrm; qx /= nrm; qy /= nrm; qz /= nrm;
    R[0] = 1 - 2 * (qy * qy + qz * qz); R[1] = 2 * (qx * qy - qz * qw);     R[2] = 2 * (qx * qz + qy * qw);
    R[3] = 2 * (qx * qy + qz * qw);     R[4] = 1 - 2 * (qx * qx + qz * qz); R[5] = 2 * (qy * qz - qx * qw);
    R[6] = 2 * (qx * qz - qy * qw);     R[7] = 2 * (qy * qz + qx * qw);     R[8] = 1 - 2 * (qx * qx + qy * qy);
    return 0;
}

static unsigned int lcg_next(unsigned int* st) {
    *st = *st * 1664525u + 1013904223u;
    return *st >> 8;
}

static double ang_err(const double R[9], const double a[3], const double b[3]) {
    const double x = R[0] * a[0] + R[1] * a[1] + R[2] * a[2];
    const double y = R[3] * a[0] + R[4] * a[1] + R[5] * a[2];
    const double z = R[6] * a[0] + R[7] * a[1] + R[8] * a[2];
    double d = x * b[0] + y * b[1] + z * b[2];
    d = d > 1.0 ? 1.0 : (d < -1.0 ? -1.0 : d);
    return acos(d);
}

unsigned int sv_rot_ransac(const double* a, const double* b, unsigned int n, double thr_rad, unsigned int iters, double R_ab[9],
                           unsigned char* inl) {
    unsigned int st = 12345u, it, i, best_n = 0;
    double best_R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double *ia, *ib;
    unsigned int pass;
    if (n < 2) {
        memcpy(R_ab, best_R, sizeof(best_R));
        memset(inl, 0, n);
        return 0;
    }
    for (it = 0; it < iters; ++it) {
        unsigned int i0 = lcg_next(&st) % n, i1 = lcg_next(&st) % n, cnt = 0;
        double sa[6], sb[6], R[9], da, db;
        if (i0 == i1) {
            continue;
        }
        memcpy(sa, a + 3 * i0, 3 * sizeof(double));
        memcpy(sa + 3, a + 3 * i1, 3 * sizeof(double));
        memcpy(sb, b + 3 * i0, 3 * sizeof(double));
        memcpy(sb + 3, b + 3 * i1, 3 * sizeof(double));
        da = sa[0] * sa[3] + sa[1] * sa[4] + sa[2] * sa[5]; /* the angle between the two bearings is preserved by a rotation */
        db = sb[0] * sb[3] + sb[1] * sb[4] + sb[2] * sb[5];
        if (fabs(da - db) > 2.0 * thr_rad || fabs(da) > 0.9998) { /* inconsistent pair or (nearly) parallel bearings: degenerate */
            continue;
        }
        if (sv_rot_fit(sa, sb, 2, R) != 0) {
            continue;
        }
        for (i = 0; i < n; ++i) {
            cnt += ang_err(R, a + 3 * i, b + 3 * i) < thr_rad;
        }
        if (cnt > best_n) {
            best_n = cnt;
            memcpy(best_R, R, sizeof(R));
        }
    }
    if (best_n < 2) {
        memcpy(R_ab, best_R, sizeof(best_R));
        memset(inl, 0, n);
        return 0;
    }
    ia = (double*)malloc(3 * n * sizeof(double));
    ib = (double*)malloc(3 * n * sizeof(double));
    for (pass = 0; pass < 3; ++pass) { /* refit on all inliers, re-collect */
        unsigned int m = 0;
        double R[9];
        for (i = 0; i < n; ++i) {
            if (ang_err(best_R, a + 3 * i, b + 3 * i) < thr_rad) {
                memcpy(ia + 3 * m, a + 3 * i, 3 * sizeof(double));
                memcpy(ib + 3 * m, b + 3 * i, 3 * sizeof(double));
                ++m;
            }
        }
        if (m < 2 || sv_rot_fit(ia, ib, m, R) != 0) {
            break;
        }
        memcpy(best_R, R, sizeof(R));
    }
    best_n = 0;
    for (i = 0; i < n; ++i) {
        inl[i] = ang_err(best_R, a + 3 * i, b + 3 * i) < thr_rad;
        best_n += inl[i];
    }
    memcpy(R_ab, best_R, sizeof(best_R));
    free(ia);
    free(ib);
    return best_n;
}

/* ---- matching ---- */
static void bearing(const sv_tr_config* c, float x, float y, double b[3]) {
    const double X = ((double)x - c->cx) / c->fx, Y = ((double)y - c->cy) / c->fy, nn = sqrt(X * X + Y * Y + 1.0);
    b[0] = X / nn;
    b[1] = Y / nn;
    b[2] = 1.0 / nn;
}

/* A's keypoints into B under the rotation R (b = R a), window radius, unique on both sides; returns the number of matches */
static unsigned int match_pass(const sv_tr_config* cfg, const sv_tr_obs* A, const sv_tr_obs* B, const double R[9], double radius,
                               unsigned int hamm_thr, unsigned int* ma, unsigned int* mb, unsigned int* md) {
    const unsigned int nb = B->num_kp;
    unsigned int* cand = (unsigned int*)malloc((nb ? nb : 1) * sizeof(unsigned int));
    int* owner = (int*)malloc((nb ? nb : 1) * sizeof(int));
    unsigned int i, n = 0, k;
    for (k = 0; k < nb; ++k) {
        owner[k] = -1;
    }
    for (i = 0; i < A->num_kp; ++i) {
        double ba[3], p[3];
        float px, py;
        unsigned int nc, best = 256, second = 256;
        int best_k = -1, lv = A->kp[i].octave, lo, hi;
        bearing(cfg, A->kp[i].x, A->kp[i].y, ba);
        p[0] = R[0] * ba[0] + R[1] * ba[1] + R[2] * ba[2];
        p[1] = R[3] * ba[0] + R[4] * ba[1] + R[5] * ba[2];
        p[2] = R[6] * ba[0] + R[7] * ba[1] + R[8] * ba[2];
        if (p[2] < 0.2) {
            continue;
        }
        px = (float)(cfg->fx * p[0] / p[2] + cfg->cx);
        py = (float)(cfg->fy * p[1] / p[2] + cfg->cy);
        lo = lv - 1 < 0 ? 0 : lv - 1;
        hi = lv + 1 > (int)cfg->num_levels - 1 ? (int)cfg->num_levels - 1 : lv + 1;
        nc = sv_frame_get_keypoints_in_cell(&B->grid, B->kp, px, py, (float)radius, lo, hi, cand, nb);
        for (k = 0; k < nc; ++k) {
            const unsigned int d = sv_tr_hamming(A->desc + (size_t)i * SV_TR_DESC_BYTES, B->desc + (size_t)cand[k] * SV_TR_DESC_BYTES);
            if (d < best) {
                second = best;
                best = d;
                best_k = (int)cand[k];
            }
            else if (d < second) {
                second = d;
            }
        }
        if (best_k < 0 || best > hamm_thr || (double)best > 0.9 * (double)second) {
            continue;
        }
        if (owner[best_k] >= 0) { /* the keypoint of B is already taken: keep the closer descriptor */
            if (md[owner[best_k]] <= best) {
                continue;
            }
            ma[owner[best_k]] = i;
            mb[owner[best_k]] = (unsigned int)best_k;
            md[owner[best_k]] = best;
            continue;
        }
        owner[best_k] = (int)n;
        ma[n] = i;
        mb[n] = (unsigned int)best_k;
        md[n] = best;
        ++n;
    }
    free(cand);
    free(owner);
    return n;
}

static int cmp_double(const void* x, const void* y) {
    const double a = *(const double*)x, b = *(const double*)y;
    return (a > b) - (a < b);
}

int sv_rot_estimate(const sv_tr_config* cfg, const sv_tr_obs* A, const sv_tr_obs* B, const double R_pred[9], double radius_px,
                    unsigned int min_inl, sv_rot_result* out) {
    const unsigned int cap = A->num_kp ? A->num_kp : 1;
    unsigned int *ma = (unsigned int*)malloc(cap * sizeof(unsigned int)), *mb = (unsigned int*)malloc(cap * sizeof(unsigned int));
    unsigned int* md = (unsigned int*)malloc(cap * sizeof(unsigned int));
    double *ba, *bb, R1[9], R2[9], *par;
    unsigned char* inl;
    unsigned int n, i, n_in, pass;
    const double thr = 4.0 / cfg->fx; /* 4 px */
    int ok = 0;
    memset(out, 0, sizeof(*out));
    memcpy(out->R_ab, R_pred, 9 * sizeof(double));
    ba = (double*)malloc(3 * cap * sizeof(double));
    bb = (double*)malloc(3 * cap * sizeof(double));
    inl = (unsigned char*)malloc(cap);
    par = (double*)malloc(cap * sizeof(double));
    memcpy(R1, R_pred, sizeof(R1));
    for (pass = 0; pass < 2; ++pass) {
        n = match_pass(cfg, A, B, R1, pass == 0 ? radius_px : 12.0, pass == 0 ? 64u : ROT_MAX_HAMM, ma, mb, md);
        out->n_match = n;
        if (n < 8) {
            goto done;
        }
        for (i = 0; i < n; ++i) {
            bearing(cfg, A->kp[ma[i]].x, A->kp[ma[i]].y, ba + 3 * i);
            bearing(cfg, B->kp[mb[i]].x, B->kp[mb[i]].y, bb + 3 * i);
        }
        n_in = sv_rot_ransac(ba, bb, n, thr, 150, R2, inl);
        out->n_inlier = n_in;
        if (n_in < 8) {
            goto done;
        }
        memcpy(R1, R2, sizeof(R1));
        memcpy(out->R_ab, R1, sizeof(R1));
        {
            unsigned int m = 0;
            for (i = 0; i < n; ++i) {
                if (inl[i]) {
                    par[m++] = ang_err(R1, ba + 3 * i, bb + 3 * i);
                }
            }
            qsort(par, m, sizeof(double), cmp_double);
            out->parallax_med = m ? par[m / 2] : 0.0;
        }
    }
    ok = out->n_inlier >= min_inl;
done:
    free(ma); free(mb); free(md); free(ba); free(bb); free(inl); free(par);
    return ok;
}
