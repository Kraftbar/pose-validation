/* SPDX-License-Identifier: BSD-3-Clause
 * Own C99 code following the ideas of PoseLib (https://github.com/PoseLib/PoseLib):
 * Copyright (c) 2020, Viktor Larsson. All rights reserved. BSD 3-Clause (full text: LICENSES/poselib-BSD-3-Clause.txt).
 * Derived ideas: LO-RANSAC loop with MSAC scoring and local optimisation on every new best model; Sampson-error relative pose refinement and
 * reprojection pose refinement with a Cauchy loss and Levenberg-Marquardt damping; P3P minimal solver. No PoseLib source is copied; the
 * minimal solver is Grunert's distance formulation, the Jacobians are central differences. Project-authored parts: MIT (Pose Validation contributors).
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
#include "sv_poselib.h"
#include "sv_solve_essential_5pt.h"
#include "sv_solve_essential.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---------- 3-vectors and column-major 3x3 ---------- */
static double dot3(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static void cross3(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
static void unit3(double a[3]) {
    double n = sqrt(dot3(a, a));
    if (n > 0) { a[0] /= n; a[1] /= n; a[2] /= n; }
}
static void mulv(const double R[9], const double v[3], double o[3]) {
    int r;
    for (r = 0; r < 3; ++r) o[r] = R[r] * v[0] + R[3 + r] * v[1] + R[6 + r] * v[2];
}
static void mulm(const double A[9], const double B[9], double C[9]) {
    int r, c;
    for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) C[c * 3 + r] = A[r] * B[c * 3] + A[3 + r] * B[c * 3 + 1] + A[6 + r] * B[c * 3 + 2];
}
/* Rodrigues: R = exp([w]x) */
static void so3_exp(const double w[3], double R[9]) {
    double th2 = dot3(w, w), th = sqrt(th2), a, b;
    if (th < 1e-8) { a = 1.0 - th2 / 6.0; b = 0.5 - th2 / 24.0; }
    else { a = sin(th) / th; b = (1.0 - cos(th)) / th2; }
    /* R = I + a [w]x + b [w]x^2 */
    R[0] = 1 + b * (-w[2] * w[2] - w[1] * w[1]);  R[3] = -a * w[2] + b * w[0] * w[1];            R[6] = a * w[1] + b * w[0] * w[2];
    R[1] = a * w[2] + b * w[0] * w[1];            R[4] = 1 + b * (-w[2] * w[2] - w[0] * w[0]);   R[7] = -a * w[0] + b * w[1] * w[2];
    R[2] = -a * w[1] + b * w[0] * w[2];           R[5] = a * w[0] + b * w[1] * w[2];             R[8] = 1 + b * (-w[1] * w[1] - w[0] * w[0]);
}
/* nearest rotation by Gram-Schmidt on the columns (keeps long LM chains orthonormal) */
static void orthonormalize(double R[9]) {
    double c2[3];
    unit3(R);
    { double d = dot3(R, R + 3); R[3] -= d * R[0]; R[4] -= d * R[1]; R[5] -= d * R[2]; }
    unit3(R + 3);
    cross3(R, R + 3, c2);
    R[6] = c2[0]; R[7] = c2[1]; R[8] = c2[2];
}

/* ---------- Sampson distance ---------- */
void sv_essential_from_pose(const double R[9], const double t[3], double E[9]) {
    /* E = [t]x R, [t]x = (0 -t2 t1; t2 0 -t0; -t1 t0 0) */
    double T[9] = {0, t[2], -t[1], -t[2], 0, t[0], t[1], -t[0], 0};
    mulm(T, R, E);
}
static double sampson_xy(const double E[9], const double x1[2], const double x2[2]) {
    /* x1, x2 normalized-plane points (u, v, 1) */
    double e1[3] = {E[0] * x1[0] + E[3] * x1[1] + E[6], E[1] * x1[0] + E[4] * x1[1] + E[7], E[2] * x1[0] + E[5] * x1[1] + E[8]};
    double f2[2] = {E[0] * x2[0] + E[1] * x2[1] + E[2], E[3] * x2[0] + E[4] * x2[1] + E[5]};
    double num = x2[0] * e1[0] + x2[1] * e1[1] + e1[2];
    double den = e1[0] * e1[0] + e1[1] * e1[1] + f2[0] * f2[0] + f2[1] * f2[1];
    return den > 1e-300 ? num / sqrt(den) : 1e9;
}
double sv_sampson(const double E[9], const double b1[3], const double b2[3]) {
    double x1[2], x2[2];
    if (b1[2] < 1e-3 || b2[2] < 1e-3) return 1e9;
    x1[0] = b1[0] / b1[2]; x1[1] = b1[1] / b1[2]; x2[0] = b2[0] / b2[2]; x2[1] = b2[1] / b2[2];
    return sampson_xy(E, x1, x2);
}

/* ---------- generic damped Gauss-Newton with Cauchy weights and central-difference Jacobians ---------- */
typedef struct {
    unsigned npar, nres;
    void (*res)(void* ctx, const double* d, double* r); /* residuals (already divided by their scale) at base pose moved by tangent step d */
    void (*commit)(void* ctx, const double* d);
    void* ctx;
} lm_prob;

static int solve_small(double* A, double* b, unsigned n) { /* Gauss elimination, partial pivoting; solution in b */
    unsigned i, j, k;
    for (i = 0; i < n; ++i) {
        unsigned piv = i;
        double m, inv;
        for (j = i + 1; j < n; ++j) if (fabs(A[j * n + i]) > fabs(A[piv * n + i])) piv = j;
        if (fabs(A[piv * n + i]) < 1e-300) return -1;
        if (piv != i) {
            for (k = 0; k < n; ++k) { m = A[i * n + k]; A[i * n + k] = A[piv * n + k]; A[piv * n + k] = m; }
            m = b[i]; b[i] = b[piv]; b[piv] = m;
        }
        inv = 1.0 / A[i * n + i];
        for (j = i + 1; j < n; ++j) {
            m = A[j * n + i] * inv;
            for (k = i; k < n; ++k) A[j * n + k] -= m * A[i * n + k];
            b[j] -= m * b[i];
        }
    }
    for (i = n; i-- > 0;) {
        for (j = i + 1; j < n; ++j) b[i] -= A[i * n + j] * b[j];
        b[i] /= A[i * n + i];
    }
    return 0;
}
static double cauchy_cost(const double* r, unsigned n) {
    double c = 0;
    unsigned i;
    for (i = 0; i < n; ++i) c += log(1.0 + r[i] * r[i]);
    return c;
}
static void lm_run(const lm_prob* P, unsigned iters) {
    const unsigned np = P->npar, nr = P->nres;
    double *r0 = (double*)malloc(sizeof(double) * nr * 4), *rp, *rm, *r1, *J = (double*)malloc(sizeof(double) * nr * np);
    double d[8], H[64], g[8], A[64], x[8], lambda = 1e-3, cost;
    unsigned it, i, k, a, b;
    if (!r0 || !J) { free(r0); free(J); return; }
    rp = r0 + nr; rm = rp + nr; r1 = rm + nr;
    memset(d, 0, sizeof(d));
    P->res(P->ctx, d, r0);
    cost = cauchy_cost(r0, nr);
    for (it = 0; it < iters; ++it) {
        int moved = 0, tries;
        double h = 1e-6;
        for (k = 0; k < np; ++k) {
            memset(d, 0, sizeof(d)); d[k] = h; P->res(P->ctx, d, rp);
            d[k] = -h; P->res(P->ctx, d, rm);
            for (i = 0; i < nr; ++i) J[i * np + k] = (rp[i] - rm[i]) / (2 * h);
        }
        memset(H, 0, sizeof(H)); memset(g, 0, sizeof(g));
        for (i = 0; i < nr; ++i) {
            double w = 1.0 / (1.0 + r0[i] * r0[i]);
            for (a = 0; a < np; ++a) {
                g[a] += w * J[i * np + a] * r0[i];
                for (b = 0; b < np; ++b) H[a * np + b] += w * J[i * np + a] * J[i * np + b];
            }
        }
        for (tries = 0; tries < 8; ++tries) {
            double c1, dn = 0;
            for (a = 0; a < np; ++a) {
                for (b = 0; b < np; ++b) A[a * np + b] = H[a * np + b];
                A[a * np + a] += lambda * (H[a * np + a] + 1e-9);
                x[a] = -g[a];
            }
            if (solve_small(A, x, np)) { lambda *= 10; continue; }
            for (a = 0; a < np; ++a) dn += x[a] * x[a];
            memset(d, 0, sizeof(d));
            for (a = 0; a < np; ++a) d[a] = x[a];
            P->res(P->ctx, d, r1);
            c1 = cauchy_cost(r1, nr);
            if (c1 < cost) {
                P->commit(P->ctx, d);
                P->res(P->ctx, (const double[8]){0}, r0);
                cost = cauchy_cost(r0, nr);
                lambda = lambda * 0.3 > 1e-9 ? lambda * 0.3 : 1e-9;
                moved = dn > 1e-20;
                break;
            }
            lambda *= 10;
        }
        if (!moved) break;
    }
    free(r0); free(J);
}

/* ---------- relative pose refinement ---------- */
typedef struct { unsigned m; double* x1; double* x2; double inv_scale; double R[9], t[3]; } relp_ctx;
static void t_basis(const double t[3], double u[3], double v[3]) {
    double ax[3] = {0, 0, 0};
    unsigned k = fabs(t[0]) < fabs(t[1]) ? (fabs(t[0]) < fabs(t[2]) ? 0 : 2) : (fabs(t[1]) < fabs(t[2]) ? 1 : 2);
    ax[k] = 1;
    cross3(t, ax, u); unit3(u);
    cross3(t, u, v); unit3(v);
}
static void relp_pose(const relp_ctx* c, const double* d, double R[9], double t[3]) {
    double dR[9], u[3], v[3];
    so3_exp(d, dR);
    mulm(dR, c->R, R);
    t_basis(c->t, u, v);
    t[0] = c->t[0] + d[3] * u[0] + d[4] * v[0];
    t[1] = c->t[1] + d[3] * u[1] + d[4] * v[1];
    t[2] = c->t[2] + d[3] * u[2] + d[4] * v[2];
    unit3(t);
}
static void relp_res(void* ctx, const double* d, double* r) {
    relp_ctx* c = (relp_ctx*)ctx;
    double R[9], t[9], E[9];
    unsigned i;
    relp_pose(c, d, R, t);
    sv_essential_from_pose(R, t, E);
    for (i = 0; i < c->m; ++i) r[i] = sampson_xy(E, c->x1 + 2 * i, c->x2 + 2 * i) * c->inv_scale;
}
static void relp_commit(void* ctx, const double* d) {
    relp_ctx* c = (relp_ctx*)ctx;
    double R[9], t[3];
    relp_pose(c, d, R, t);
    memcpy(c->R, R, sizeof(R)); memcpy(c->t, t, sizeof(t));
    orthonormalize(c->R);
}
unsigned sv_relpose_refine(const double* b1, const double* b2, unsigned n, const unsigned char* mask, double scale, unsigned iters,
                           double R[9], double t[3]) {
    relp_ctx c;
    lm_prob P;
    unsigned i, m = 0;
    double tn = sqrt(dot3(t, t));
    c.x1 = (double*)malloc(sizeof(double) * 4 * (n ? n : 1)); c.x2 = c.x1 + 2 * (n ? n : 1);
    if (!c.x1) return 0;
    for (i = 0; i < n; ++i) {
        if ((mask && !mask[i]) || b1[3 * i + 2] < 1e-3 || b2[3 * i + 2] < 1e-3) continue;
        c.x1[2 * m] = b1[3 * i] / b1[3 * i + 2]; c.x1[2 * m + 1] = b1[3 * i + 1] / b1[3 * i + 2];
        c.x2[2 * m] = b2[3 * i] / b2[3 * i + 2]; c.x2[2 * m + 1] = b2[3 * i + 1] / b2[3 * i + 2];
        ++m;
    }
    if (m < 6 || tn < 1e-12) { free(c.x1); return 0; }
    /* x2 buffer was laid out for n entries: compact it behind the m used x1 entries */
    memmove(c.x1 + 2 * m, c.x2, sizeof(double) * 2 * m);
    c.x2 = c.x1 + 2 * m;
    c.m = m; c.inv_scale = 1.0 / scale;
    memcpy(c.R, R, sizeof(c.R));
    c.t[0] = t[0] / tn; c.t[1] = t[1] / tn; c.t[2] = t[2] / tn;
    P.npar = 5; P.nres = m; P.res = relp_res; P.commit = relp_commit; P.ctx = &c;
    lm_run(&P, iters);
    memcpy(R, c.R, sizeof(c.R));
    t[0] = c.t[0] * tn; t[1] = c.t[1] * tn; t[2] = c.t[2] * tn;
    free(c.x1);
    return m;
}

/* ---------- cheirality for an essential matrix ---------- */
unsigned sv_relpose_pick(const double E[9], const double* b1, const double* b2, unsigned n, const unsigned char* mask, double R[9], double t[3]) {
    double rots[4][9], transes[4][3];
    unsigned best = 0, bestn = 0, h, i;
    sv_solve_essential_decompose(E, rots, transes);
    for (h = 0; h < 4; ++h) {
        unsigned cnt = 0;
        for (i = 0; i < n; ++i) {
            double Rx[3], A00, A01, A11, y0, y1, det, l1, l2;
            if (mask && !mask[i]) continue;
            /* l2 x2 = l1 R x1 + t : least squares for (l1, l2) */
            mulv(rots[h], b1 + 3 * i, Rx);
            A00 = dot3(Rx, Rx); A01 = -dot3(Rx, b2 + 3 * i); A11 = dot3(b2 + 3 * i, b2 + 3 * i);
            y0 = -dot3(Rx, transes[h]); y1 = dot3(b2 + 3 * i, transes[h]);
            det = A00 * A11 - A01 * A01;
            if (fabs(det) < 1e-18) continue;
            l1 = (y0 * A11 - A01 * y1) / det; l2 = (A00 * y1 - A01 * y0) / det;
            if (l1 > 0 && l2 > 0) ++cnt;
        }
        if (cnt > bestn) { bestn = cnt; best = h; }
    }
    memcpy(R, rots[best], sizeof(rots[best]));
    memcpy(t, transes[best], sizeof(transes[best]));
    return best;
}

/* ---------- 5pt LO-RANSAC ---------- */
static double msac_rel(const double E[9], const double* x, unsigned n, double thr, unsigned char* mask, unsigned* ninl) {
    double sc = 0, t2 = thr * thr;
    unsigned i, c = 0;
    for (i = 0; i < n; ++i) {
        double e = sampson_xy(E, x + 4 * i, x + 4 * i + 2), e2 = e * e;
        if (e2 < t2) { sc += e2; ++c; if (mask) mask[i] = 1; }
        else { sc += t2; if (mask) mask[i] = 0; }
    }
    *ninl = c;
    return sc;
}
int sv_relpose_lo_ransac(const double* b1, const double* b2, unsigned n, double thr, unsigned max_iters, unsigned min_iters, sv_mt19937* rng,
                         double E[9], unsigned char* mask, unsigned* num_inliers) {
    double* x = (double*)malloc(sizeof(double) * 4 * (n ? n : 1));
    unsigned char* tmp = (unsigned char*)malloc(n ? n : 1);
    unsigned char* lo = (unsigned char*)malloc(n ? n : 1);
    double best = DBL_MAX;
    unsigned iter, need = max_iters, i, best_n = 0;
    sv_mt19937 local;
    *num_inliers = 0;
    if (!x || !tmp || !lo) { free(x); free(tmp); free(lo); return -1; }
    if (!rng) { sv_mt19937_init_default(&local); rng = &local; }
    if (n < 8) { free(x); free(tmp); free(lo); return 0; }
    for (i = 0; i < n; ++i) { /* normalized-plane points; behind-plane bearings get a far-away dummy so that they never count */
        const double* a = b1 + 3 * i;
        const double* b = b2 + 3 * i;
        x[4 * i] = a[2] > 1e-3 ? a[0] / a[2] : 1e6; x[4 * i + 1] = a[2] > 1e-3 ? a[1] / a[2] : 1e6;
        x[4 * i + 2] = b[2] > 1e-3 ? b[0] / b[2] : -1e6; x[4 * i + 3] = b[2] > 1e-3 ? b[1] / b[2] : 1e6;
    }
    memset(mask, 0, n);
    for (iter = 0; iter < need && iter < max_iters; ++iter) {
        uint32_t idx[5];
        double s1[15], s2[15], cand[90];
        int nc, k;
        sv_create_random_array(5, 0, n - 1, rng, idx);
        for (k = 0; k < 5; ++k) { memcpy(s1 + 3 * k, b1 + 3 * idx[k], 24); memcpy(s2 + 3 * k, b2 + 3 * idx[k], 24); }
        nc = sv_essential_5pt(s1, s2, cand, NULL);
        for (k = 0; k < nc; ++k) {
            unsigned ni, round;
            double sc = msac_rel(cand + 9 * k, x, n, thr, tmp, &ni), Ec[9];
            if (!(sc < best)) continue;
            memcpy(Ec, cand + 9 * k, sizeof(Ec));
            /* local optimisation: pose from the inliers, refine, re-score; up to 3 rounds while the score improves */
            for (round = 0; round < 3 && ni >= 8; ++round) {
                double R[9], t[3], E2[9], sc2;
                unsigned ni2;
                sv_relpose_pick(Ec, b1, b2, n, tmp, R, t);
                if (sv_relpose_refine(b1, b2, n, tmp, thr, 12, R, t) < 8) break;
                sv_essential_from_pose(R, t, E2);
                sc2 = msac_rel(E2, x, n, thr, lo, &ni2);
                if (!(sc2 < sc)) break;
                sc = sc2; ni = ni2; memcpy(Ec, E2, sizeof(Ec)); memcpy(tmp, lo, n);
            }
            if (sc < best) {
                best = sc; best_n = ni; memcpy(E, Ec, sizeof(Ec)); memcpy(mask, tmp, n);
                {
                    double w = (double)ni / (double)n, p5 = w * w * w * w * w;
                    need = p5 > 1e-12 ? (unsigned)(log(1.0 - 0.9999) / log(1.0 - (p5 < 0.999999 ? p5 : 0.999999))) + 1 : max_iters;
                    if (need < min_iters) need = min_iters;
                }
            }
            /* tmp was clobbered by later candidates only if accepted; re-score keeps mask consistent through memcpy above */
        }
    }
    *num_inliers = best < DBL_MAX ? best_n : 0;
    free(x); free(tmp); free(lo);
    return 0;
}

/* ---------- P3P (Grunert distances) ---------- */
/* real roots of c[0] + c[1] x + ... + c[deg] x^deg, ascending in out; returns count. Recursion on the derivative's roots, bisection between them. */
static double poly_eval(const double* c, int deg, double x) {
    double v = 0;
    int i;
    for (i = deg; i >= 0; --i) v = v * x + c[i];
    return v;
}
static int poly_roots(const double* c, int deg, double* out) {
    double scale = 0, bound, d[5], crit[6], pts[8], f0, f1;
    int i, n = 0, nc, np, k;
    for (i = 0; i <= deg; ++i) if (fabs(c[i]) > scale) scale = fabs(c[i]);
    while (deg > 0 && fabs(c[deg]) <= 1e-12 * scale) --deg;
    if (deg < 1) return 0;
    if (deg == 1) { out[0] = -c[0] / c[1]; return 1; }
    if (deg == 2) {
        double disc = c[1] * c[1] - 4 * c[2] * c[0], s, q;
        if (disc < 0) return 0;
        s = sqrt(disc);
        q = -0.5 * (c[1] + (c[1] >= 0 ? s : -s));
        out[0] = q / c[2]; out[1] = fabs(q) > 0 ? c[0] / q : out[0];
        if (out[0] > out[1]) { double tt = out[0]; out[0] = out[1]; out[1] = tt; }
        return 2;
    }
    for (i = 1; i <= deg; ++i) d[i - 1] = i * c[i];
    nc = poly_roots(d, deg - 1, crit);
    bound = 0;
    for (i = 0; i < deg; ++i) if (fabs(c[i] / c[deg]) > bound) bound = fabs(c[i] / c[deg]);
    bound += 1.0;
    np = 0; pts[np++] = -bound;
    for (k = 0; k < nc; ++k) pts[np++] = crit[k];
    pts[np++] = bound;
    for (k = 0; k + 1 < np; ++k) {
        double lo = pts[k], hi = pts[k + 1];
        int it;
        f0 = poly_eval(c, deg, lo); f1 = poly_eval(c, deg, hi);
        if (f0 == 0 && k > 0) continue; /* root at a critical point is reported by the interval to its left */
        if (f1 == 0) { out[n++] = hi; continue; }
        if ((f0 < 0) == (f1 < 0)) continue;
        for (it = 0; it < 100; ++it) {
            double mid = 0.5 * (lo + hi), fm = poly_eval(c, deg, mid);
            if ((fm < 0) == (f0 < 0)) { lo = mid; f0 = fm; } else hi = mid;
        }
        out[n++] = 0.5 * (lo + hi);
    }
    return n;
}
static void pmul(const double* a, int da, const double* b, int db, double* o) { /* o[0..da+db] */
    int i, j;
    for (i = 0; i <= da + db; ++i) o[i] = 0;
    for (i = 0; i <= da; ++i) for (j = 0; j <= db; ++j) o[i + j] += a[i] * b[j];
}
int sv_p3p(const double b[9], const double P[9], double R[4][9], double t[4][3]) {
    double a2, b2, c2, ca, cb, cc, Dl[2], Q2[3], N[3], A[3], tmp[5], tmp2[5], quart[5], q[5], roots[4];
    double e[3], f[3];
    int nr, i, k, np = 0;
    { double d[3] = {P[3] - P[6], P[4] - P[7], P[5] - P[8]}; a2 = dot3(d, d); }
    { double d[3] = {P[0] - P[6], P[1] - P[7], P[2] - P[8]}; b2 = dot3(d, d); }
    { double d[3] = {P[0] - P[3], P[1] - P[4], P[2] - P[5]}; c2 = dot3(d, d); }
    if (c2 < 1e-18) return 0;
    ca = dot3(b + 3, b + 6); cb = dot3(b, b + 6); cc = dot3(b, b + 3);
    Dl[0] = -2 * c2 * cb; Dl[1] = 2 * c2 * ca;                    /* D(u) = 2 c^2 (ca u - cb) */
    Q2[0] = 1; Q2[1] = -2 * cc; Q2[2] = 1;                         /* 1 + u^2 - 2 u cc */
    N[0] = (b2 - a2) - c2; N[1] = -2 * cc * (b2 - a2); N[2] = (b2 - a2) + c2;   /* (b^2-a^2) Q2 - c^2 (1-u^2) */
    /* quartic: c^2 N^2 - 2 cb c^2 N D + (c^2 - b^2 Q2) D^2 = 0 */
    pmul(N, 2, N, 2, tmp);
    for (i = 0; i < 5; ++i) quart[i] = c2 * tmp[i];
    pmul(N, 2, Dl, 1, tmp);
    for (i = 0; i < 4; ++i) quart[i] -= 2 * cb * c2 * tmp[i];
    A[0] = c2 - b2 * Q2[0]; A[1] = -b2 * Q2[1]; A[2] = -b2 * Q2[2];
    pmul(Dl, 1, Dl, 1, tmp2);
    pmul(A, 2, tmp2, 2, q);
    for (i = 0; i < 5; ++i) quart[i] += q[i];
    nr = poly_roots(quart, 4, roots);
    /* frame of the world points */
    for (i = 0; i < 3; ++i) { e[i] = P[3 + i] - P[i]; f[i] = P[6 + i] - P[i]; }
    for (k = 0; k < nr && np < 4; ++k) {
        double u = roots[k], q2 = Q2[0] + Q2[1] * u + Q2[2] * u * u, D = Dl[0] + Dl[1] * u, v, d1, d[3], X[9], ew[3], e3w[3], e2w[3], ec[3], e3c[3], e2c[3], fc[3], Fw[9], Fc[9], FwT[9];
        if (q2 <= 1e-12 || fabs(D) < 1e-14) continue;
        v = (N[0] + N[1] * u + N[2] * u * u) / D;
        d1 = sqrt(c2 / q2);
        d[0] = d1; d[1] = u * d1; d[2] = v * d1;
        if (!(d[0] > 0 && d[1] > 0 && d[2] > 0)) continue;
        for (i = 0; i < 3; ++i) { X[i] = d[0] * b[i]; X[3 + i] = d[1] * b[3 + i]; X[6 + i] = d[2] * b[6 + i]; }
        /* orthonormal frame (x = P1->P2, z = x cross (P1->P3), y = z cross x) in the world and in the camera */
        for (i = 0; i < 3; ++i) { ew[i] = e[i]; ec[i] = X[3 + i] - X[i]; fc[i] = X[6 + i] - X[i]; }
        unit3(ew); unit3(ec);
        cross3(ew, f, e3w); cross3(ec, fc, e3c);
        if (sqrt(dot3(e3w, e3w)) < 1e-9 || sqrt(dot3(e3c, e3c)) < 1e-9) continue;
        unit3(e3w); unit3(e3c);
        cross3(e3w, ew, e2w); cross3(e3c, ec, e2c);
        memcpy(Fw, ew, 24); memcpy(Fw + 3, e2w, 24); memcpy(Fw + 6, e3w, 24);
        memcpy(Fc, ec, 24); memcpy(Fc + 3, e2c, 24); memcpy(Fc + 6, e3c, 24);
        { int r, c; for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) FwT[c * 3 + r] = Fw[r * 3 + c]; }
        mulm(Fc, FwT, R[np]);
        { double RP[3]; mulv(R[np], P, RP); t[np][0] = X[0] - RP[0]; t[np][1] = X[1] - RP[1]; t[np][2] = X[2] - RP[2]; }
        ++np;
    }
    return np;
}

/* ---------- PnP refinement ---------- */
typedef struct { unsigned m; const double* x; const double* P; const double* inv_sigma; double R[9], t[3]; } pnp_ctx;
static void pnp_res(void* ctx, const double* d, double* r) {
    pnp_ctx* c = (pnp_ctx*)ctx;
    double dR[9];
    unsigned i;
    so3_exp(d, dR);
    for (i = 0; i < c->m; ++i) {
        double pc0[3], pc[3];
        mulv(c->R, c->P + 3 * i, pc0);
        pc0[0] += c->t[0]; pc0[1] += c->t[1]; pc0[2] += c->t[2];
        mulv(dR, pc0, pc);
        pc[0] += d[3]; pc[1] += d[4]; pc[2] += d[5];
        if (pc[2] < 1e-6) { r[2 * i] = 1e3; r[2 * i + 1] = 1e3; continue; }
        r[2 * i] = (pc[0] / pc[2] - c->x[2 * i]) * c->inv_sigma[i];
        r[2 * i + 1] = (pc[1] / pc[2] - c->x[2 * i + 1]) * c->inv_sigma[i];
    }
}
static void pnp_commit(void* ctx, const double* d) {
    pnp_ctx* c = (pnp_ctx*)ctx;
    double dR[9], R[9], t0[3], t[3];
    so3_exp(d, dR);
    mulm(dR, c->R, R);
    mulv(dR, c->t, t0);
    t[0] = t0[0] + d[3]; t[1] = t0[1] + d[4]; t[2] = t0[2] + d[5];
    memcpy(c->R, R, sizeof(R)); memcpy(c->t, t, sizeof(t));
    orthonormalize(c->R);
}
void sv_pnp_refine(const double* b, const double* p, const double* sigma, unsigned m, unsigned iters, double R[9], double t[3]) {
    pnp_ctx c;
    lm_prob P;
    double* buf;
    unsigned i, k = 0;
    if (m < 4) return;
    buf = (double*)malloc(sizeof(double) * 3 * m);
    if (!buf) return;
    for (i = 0; i < m; ++i) {
        buf[2 * i] = b[3 * i + 2] > 1e-6 ? b[3 * i] / b[3 * i + 2] : 0;
        buf[2 * i + 1] = b[3 * i + 2] > 1e-6 ? b[3 * i + 1] / b[3 * i + 2] : 0;
        buf[2 * m + i] = 1.0 / sigma[i];
    }
    (void)k;
    c.m = m; c.x = buf; c.P = p; c.inv_sigma = buf + 2 * m;
    memcpy(c.R, R, sizeof(c.R)); memcpy(c.t, t, sizeof(c.t));
    P.npar = 6; P.nres = 2 * m; P.res = pnp_res; P.commit = pnp_commit; P.ctx = &c;
    lm_run(&P, iters);
    memcpy(R, c.R, sizeof(c.R)); memcpy(t, c.t, sizeof(c.t));
    free(buf);
}

/* ---------- P3P LO-RANSAC ---------- */
int sv_pnp_lo_ransac(const double* b, const double* p, const int* octaves, unsigned n, const float* scales, unsigned levels, unsigned min_inliers,
                     unsigned max_iters, sv_mt19937* rng, sv_pnp_result* result, unsigned char* mask) {
    float* thr;
    double *sig, *bi, *pi, *si;
    unsigned char *tmp, *lo;
    unsigned iter, need = max_iters, i;
    sv_mt19937 local;
    int rc = 0;
    if (!result || !scales || !levels || (n && (!b || !p || !octaves || !mask)) || n > 2147483647u / 24u) return -1;
    for (i = 0; i < n; ++i) if (octaves[i] < 0 || (unsigned)octaves[i] >= levels) return -1;
    memset(result, 0, sizeof(*result));
    if (n) memset(mask, 0, n);
    if (n < 4 || n < min_inliers) return 0;
    thr = (float*)malloc(sizeof(float) * n); sig = (double*)malloc(sizeof(double) * n * 8);
    tmp = (unsigned char*)malloc(n); lo = (unsigned char*)malloc(n);
    if (!thr || !sig || !tmp || !lo) { free(thr); free(sig); free(tmp); free(lo); return -1; }
    bi = sig + n; pi = bi + 3 * n; si = pi + 3 * n;
    for (i = 0; i < n; ++i) {
        double s;
        thr[i] = sv_pnp_radial_threshold(scales[octaves[i]]);
        s = sqrt(2.0 * (1.0 - (double)thr[i])); /* angle of the cosine threshold [rad] ~ normalized-plane radius */
        sig[i] = s > 1e-6 ? s : 1e-6;
    }
    if (!rng) { sv_mt19937_init_default(&local); rng = &local; }
    result->cost = DBL_MAX;
    for (iter = 0; iter < need && iter < max_iters; ++iter) {
        uint32_t idx[3];
        double sb[9], sp[9], Rs[4][9], ts[4][3];
        int np, k, j;
        sv_create_random_array(3, 0, n - 1, rng, idx);
        for (j = 0; j < 3; ++j) { memcpy(sb + 3 * j, b + 3 * idx[j], 24); memcpy(sp + 3 * j, p + 3 * idx[j], 24); }
        np = sv_p3p(sb, sp, Rs, ts);
        for (k = 0; k < np; ++k) {
            double cost, R[9], t[3];
            unsigned cnt = sv_pnp_check_inliers(b, p, thr, n, Rs[k], ts[k], tmp, &cost), round;
            if (!(cost < result->cost)) continue;
            memcpy(R, Rs[k], sizeof(R)); memcpy(t, ts[k], sizeof(t));
            for (round = 0; round < 3 && cnt >= 6; ++round) { /* local optimisation on the inliers */
                double R2[9], t2[3], cost2;
                unsigned m = 0, cnt2;
                for (i = 0; i < n; ++i) if (tmp[i]) { memcpy(bi + 3 * m, b + 3 * i, 24); memcpy(pi + 3 * m, p + 3 * i, 24); si[m] = sig[i]; ++m; }
                memcpy(R2, R, sizeof(R2)); memcpy(t2, t, sizeof(t2));
                sv_pnp_refine(bi, pi, si, m, 10, R2, t2);
                cnt2 = sv_pnp_check_inliers(b, p, thr, n, R2, t2, lo, &cost2);
                if (!(cost2 < cost)) break;
                cost = cost2; cnt = cnt2; memcpy(R, R2, sizeof(R)); memcpy(t, t2, sizeof(t)); memcpy(tmp, lo, n);
            }
            if (cnt > min_inliers && cost < result->cost) {
                double w = (double)cnt / (double)n, p3 = w * w * w;
                result->cost = cost; result->inliers = cnt;
                memcpy(result->rotation, R, sizeof(R)); memcpy(result->translation, t, sizeof(t)); memcpy(mask, tmp, n);
                need = p3 > 1e-12 ? (unsigned)(log(1.0 - 0.9999) / log(1.0 - (p3 < 0.999999 ? p3 : 0.999999))) + 1 : max_iters;
                if (need < 20) need = 20;
            }
        }
    }
    result->valid = result->cost < DBL_MAX;
    free(thr); free(sig); free(tmp); free(lo);
    return rc;
}
