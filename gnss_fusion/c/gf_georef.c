/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources. See gf_georef.h. */
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "gf_georef.h"

#define RING 512

typedef struct { double t, p[3]; } spose;
typedef struct { double t, a[3], z[3], w; } pair_t;

struct gf_georef {
    gf_georef_config c;
    spose ring[RING]; int rn, rh;          /* ring of stream samples: count, head (next write) */
    pair_t *pr; int np, cap;               /* pairs, oldest first (compacted when max_pairs is reached) */
    int have;                              /* a fit exists */
    double psi, s, ma[2], mz[2], tz, sres;
};

void gf_georef_config_default(gf_georef_config *c)
{
    c->min_fixes = 8; c->min_extent_m = 30.0; c->scale_sigma = 0.15; c->corr_s = 40.0; c->forget_s = 0.0; c->huber_k = 2.5;
    c->sigma_floor = 0.5; c->use_sigma = 0; c->max_pairs = 8192; c->max_gap_s = 2.5;
}

gf_georef *gf_georef_create(const gf_georef_config *c)
{
    gf_georef *g = (gf_georef *)calloc(1, sizeof *g);
    if (!g) return NULL;
    if (c) g->c = *c; else gf_georef_config_default(&g->c);
    if (g->c.max_pairs < 16) g->c.max_pairs = 16;
    return g;
}

void gf_georef_destroy(gf_georef *g) { if (g) { free(g->pr); free(g); } }

int gf_georef_add_pose(gf_georef *g, double t, const double p[3])
{
    if (!g || !p || !isfinite(t) || !isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2])) return -5;
    spose *s = &g->ring[g->rh];
    s->t = t; s->p[0] = p[0]; s->p[1] = p[1]; s->p[2] = p[2];
    g->rh = (g->rh + 1) % RING; if (g->rn < RING) g->rn++;
    return 0;
}

/* stream position at time t from the ring (linear between the neighbours, clamped to the ends); returns 0 if the ring is empty */
static int ring_at(const gf_georef *g, double t, double max_gap, double p[3])
{
    if (g->rn == 0) return 0;
    int newest = (g->rh + RING - 1) % RING;
    const spose *lo = NULL, *hi = NULL;
    for (int k = 0; k < g->rn; ++k) {           /* newest to oldest */
        const spose *s = &g->ring[(newest + RING - k) % RING];
        if (s->t >= t) hi = s;
        else { lo = s; break; }
    }
    if (!lo && !hi) return 0;
    if (!lo) { if (hi->t - t > 0.5 * max_gap) return 0; memcpy(p, hi->p, 3 * sizeof(double)); return 1; }
    if (!hi) { if (t - lo->t > 0.5 * max_gap) return 0; memcpy(p, lo->p, 3 * sizeof(double)); return 1; }
    if (hi->t - lo->t > max_gap) return 0;   /* the stream has a hole here (tracking loss): no pair */
    double w = (hi->t > lo->t) ? (t - lo->t) / (hi->t - lo->t) : 0.0;
    for (int q = 0; q < 3; ++q) p[q] = (1.0 - w) * lo->p[q] + w * hi->p[q];
    return 1;
}

/* weighted 2-D Procrustes: psi, S_cr, S_aa about the weighted means; returns sum of weights */
static int cmp_d(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }

static double procrustes_w(const pair_t *a, int n, const double *w, double *psi, double *Scr, double *Saa, double ma[2], double mz[2])
{
    double ws = 0; ma[0] = ma[1] = mz[0] = mz[1] = 0;
    for (int i = 0; i < n; ++i) { ws += w[i]; for (int q = 0; q < 2; ++q) { ma[q] += w[i] * a[i].a[q]; mz[q] += w[i] * a[i].z[q]; } }
    if (!(ws > 0)) { *psi = 0; *Scr = 0; *Saa = 1; return 0; }
    for (int q = 0; q < 2; ++q) { ma[q] /= ws; mz[q] /= ws; }
    double cross = 0, dot = 0, saa = 0;
    for (int i = 0; i < n; ++i) {
        double ax = a[i].a[0] - ma[0], ay = a[i].a[1] - ma[1], cx = a[i].z[0] - mz[0], cy = a[i].z[1] - mz[1];
        cross += w[i] * (ax * cy - ay * cx); dot += w[i] * (ax * cx + ay * cy); saa += w[i] * (ax * ax + ay * ay);
    }
    *psi = atan2(cross, dot); *Scr = sqrt(cross * cross + dot * dot); *Saa = saa;
    return ws;
}

int gf_georef_solve(gf_georef *g)
{
    if (!g) return 0;
    const gf_georef_config *c = &g->c;
    int n = g->np;
    if (n < c->min_fixes) return g->have;
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300, zxmin = 1e300, zxmax = -1e300, zymin = 1e300, zymax = -1e300;
    for (int i = 0; i < n; ++i) {
        const double *a = g->pr[i].a, *z = g->pr[i].z;
        if (a[0] < xmin) xmin = a[0];
        if (a[0] > xmax) xmax = a[0];
        if (a[1] < ymin) ymin = a[1];
        if (a[1] > ymax) ymax = a[1];
        if (z[0] < zxmin) zxmin = z[0];
        if (z[0] > zxmax) zxmax = z[0];
        if (z[1] < zymin) zymin = z[1];
        if (z[1] > zymax) zymax = z[1];
    }
    /* the yaw is defined once the track spans min_extent_m: measured on the stream when its unit is metres (scale_sigma <= 10), on the fixes otherwise (monocular unit) */
    double ext = (c->scale_sigma > 0 && c->scale_sigma <= 10.0) ? (xmax - xmin) + (ymax - ymin) : (zxmax - zxmin) + (zymax - zymin);
    if (ext < c->min_extent_m || (xmax - xmin) + (ymax - ymin) < 1e-9) return g->have;
    double tn = g->pr[n - 1].t, t0 = g->pr[0].t;
    double *w = (double *)malloc(sizeof(double) * (size_t)n * 3), *w0 = w + n, *e2a = w + 2 * (size_t)n;
    if (!w) return g->have;
    double wmean = 0;
    for (int i = 0; i < n; ++i) wmean += g->pr[i].w;
    wmean = wmean > 0 ? wmean / (double)n : 1.0;   /* base weights are relative (mean 1) so that S_aa and the residual rms keep their metre units */
    for (int i = 0; i < n; ++i) {
        w0[i] = g->pr[i].w / wmean * ((c->forget_s > 0) ? exp(-(tn - g->pr[i].t) / c->forget_s) : 1.0);
        w[i] = w0[i];
    }
    double psi = 0, Scr = 0, Saa = 1, ma[2], mz[2], sres = 1.0;
    for (int pass = 0; pass < (c->huber_k > 0 ? 4 : 1); ++pass) {
        double ws = procrustes_w(g->pr, n, w, &psi, &Scr, &Saa, ma, mz);
        if (!(ws > 0)) { free(w); return g->have; }
        double cs = cos(psi), sn = sin(psi), ss = 0;   /* residual rms of the rigid fit */
        double wsum = 0;
        for (int i = 0; i < n; ++i) {
            double ax = g->pr[i].a[0] - ma[0], ay = g->pr[i].a[1] - ma[1];
            double ex = cs * ax - sn * ay - (g->pr[i].z[0] - mz[0]), ey = sn * ax + cs * ay - (g->pr[i].z[1] - mz[1]);
            double e2 = ex * ex + ey * ey;
            ss += w[i] * e2; wsum += w[i];   /* rms with the current (Huber) weights */
            e2a[i] = e2;
        }
        sres = sqrt(ss / (wsum > 0 ? wsum : 1.0));
        if (c->huber_k <= 0) break;
        /* robust scale of the 2-D residual norms (median / 1.177 for a Rayleigh distribution), not the rms, which a burst of outliers inflates */
        double *tmp = (double *)malloc(sizeof(double) * (size_t)n);
        if (!tmp) break;
        for (int i = 0; i < n; ++i) tmp[i] = sqrt(e2a[i]);
        qsort(tmp, (size_t)n, sizeof(double), cmp_d);
        double sig = tmp[n / 2] / 1.1774; free(tmp);
        double thr = c->huber_k * (sig > 0.5 ? sig : 0.5);
        for (int i = 0; i < n; ++i) { double e = sqrt(e2a[i]); w[i] = w0[i] * (e <= thr ? 1.0 : thr / e); }
    }
    double s = 1.0;
    if (c->scale_sigma > 0) {
        double k = 1.0;
        if (c->corr_s > 1.0 && tn > t0) { k = c->corr_s * (double)(n - 1) / (tn - t0); if (k < 1.0) k = 1.0; }
        double lam = sres * sres / (c->scale_sigma * c->scale_sigma);
        s = (Scr / k + lam) / (Saa / k + lam);
    }
    double wz = 0, sz = 0;
    for (int i = 0; i < n; ++i) { wz += w[i]; sz += w[i] * (g->pr[i].z[2] - s * g->pr[i].a[2]); }
    g->psi = psi; g->s = s; g->ma[0] = ma[0]; g->ma[1] = ma[1]; g->mz[0] = mz[0]; g->mz[1] = mz[1]; g->sres = sres;
    g->tz = wz > 0 ? sz / wz : 0.0;
    g->have = 1;
    free(w);
    return 1;
}

int gf_georef_add_fix(gf_georef *g, double t, const double z[3], double sigma_h, int refit)
{
    if (!g || !z || !isfinite(t) || !isfinite(z[0]) || !isfinite(z[1]) || !isfinite(z[2])) return 0;
    double a[3];
    if (!ring_at(g, t, g->c.max_gap_s, a)) return g->have;
    if (g->np == g->cap) {
        if (g->cap >= g->c.max_pairs) {   /* drop the oldest quarter */
            int drop = g->np / 4; memmove(g->pr, g->pr + drop, sizeof(pair_t) * (size_t)(g->np - drop)); g->np -= drop;
        } else {
            int nc = g->cap ? g->cap * 2 : 256; if (nc > g->c.max_pairs) nc = g->c.max_pairs;
            pair_t *p = (pair_t *)realloc(g->pr, sizeof(pair_t) * (size_t)nc);
            if (!p) return g->have;
            g->pr = p; g->cap = nc;
        }
    }
    pair_t *p = &g->pr[g->np++];
    p->t = t; memcpy(p->a, a, sizeof a); memcpy(p->z, z, 3 * sizeof(double));
    double sg = sigma_h > g->c.sigma_floor ? sigma_h : g->c.sigma_floor;
    p->w = g->c.use_sigma ? 1.0 / (sg * sg) : 1.0;
    return refit ? gf_georef_solve(g) : g->have;
}

int gf_georef_map(const gf_georef *g, const double p[3], double out[3])
{
    if (!g || !g->have) return 0;
    double cs = cos(g->psi), sn = sin(g->psi), x = p[0] - g->ma[0], y = p[1] - g->ma[1];
    out[0] = g->s * (cs * x - sn * y) + g->mz[0];
    out[1] = g->s * (sn * x + cs * y) + g->mz[1];
    out[2] = g->s * p[2] + g->tz;
    return 1;
}

int gf_georef_fit(const gf_georef *g, double *psi, double *scale, double *sigma_res, int *n_pairs)
{
    if (!g) return 0;
    if (psi) *psi = g->psi;
    if (scale) *scale = g->s;
    if (sigma_res) *sigma_res = g->sres;
    if (n_pairs) *n_pairs = g->np;
    return g->have;
}
