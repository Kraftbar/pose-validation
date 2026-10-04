/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources. See gf_auto.h. */
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "gf_auto.h"

typedef struct { double t, z[3], sg; } pfix;

struct gf_auto {
    gf_auto_config c;
    gf_t *A, *B;                   /* smoother with fixes; fix-free stream smoother (stream_mode 1) */
    gf_georef *G;
    pfix *q; int nq, qcap;         /* fixes waiting for the georef (paired at the next aligned stream sample at or after their time) */
    /* statistics of the fixes */
    double ef2, es2, ea2, elast; long nea; /* EW mean squares of the prequential georef error (fast / slow) and the plain mean */
    double sr_ew, sw2_ew;          /* reported sigma, white-noise variance */
    double lt, lz[3], lt2, lz2[3]; int nl;   /* last two fixes (second differences) */
    double lfix_t;
    double dis2, distr;
    int n_new, n_gap; long nl_odom;
    double w, last_t, t_first, st, hold;   /* blend weight, last odom time, first odom time, switch state (0 S / 1 G), G barred until */
    int have_out; gf_auto_out out;
    gf_auto_out gout; int have_gout;
    gf_auto_sig sig;
    gf_auto_clock_fn clk; gf_auto_timing tm;
};
#define TICK() (a->clk ? a->clk() : 0.0)

void gf_auto_config_default(gf_auto_config *c)
{
    memset(c, 0, sizeof *c);
    gf_config_default(&c->sm); gf_config_robust(&c->sm);
    c->st = c->sm; c->st.init_wait_s = 12.0;
    gf_georef_config_default(&c->geo);
    c->stream_mode = 0; c->policy = GF_AUTO_AUTO;
    c->tau_fast_s = 60.0; c->tau_slow_s = 300.0; c->blend_s = 20.0; c->down_s = 2.0;
    c->rho_on = 0.75; c->rho_off = 1.125; c->distr_on = 0.2; c->distr_off = 0.3; c->sig_min = 4.0; c->sig_floor = 0.1; c->fail_k = 3.0; c->dwell_s = 60.0; c->min_span_s = 0.0;
    c->min_pairs = 30; c->metric_only = 1;
}

gf_auto *gf_auto_create(const gf_auto_config *c)
{
    gf_auto *a = (gf_auto *)calloc(1, sizeof *a);
    if (!a) return NULL;
    a->c = *c;
    a->c.sm.keep_history = 1; a->c.st.keep_history = 1;
    a->A = gf_create(&a->c.sm, 1);
    a->G = gf_georef_create(&a->c.geo);
    if (a->c.stream_mode == 1) a->B = gf_create(&a->c.st, 1);
    if (!a->A || !a->G || (a->c.stream_mode == 1 && !a->B)) { gf_auto_destroy(a); return NULL; }
    return a;
}

void gf_auto_destroy(gf_auto *a)
{
    if (!a) return;
    if (a->A) gf_destroy(a->A);
    if (a->B) gf_destroy(a->B);
    if (a->G) gf_georef_destroy(a->G);
    free(a->q); free(a);
}

const gf_t *gf_auto_smoother(const gf_auto *a) { return a->A; }
void gf_auto_set_clock(gf_auto *a, gf_auto_clock_fn fn) { a->clk = fn; }
void gf_auto_get_timing(const gf_auto *a, gf_auto_timing *t) { *t = a->tm; }

int gf_auto_add_speed(gf_auto *a, double t, double speed, double sigma, double window_s, unsigned flags)
{
    double t0 = TICK();
    int r = gf_add_speed(a->A, t, speed, sigma, window_s, flags);
    if (a->B) gf_add_speed(a->B, t, speed, sigma, window_s, flags);
    a->tm.speed += TICK() - t0;
    return r;
}

int gf_auto_add_fix(gf_auto *a, const gf_fix *f)
{
    gf_stats s0, s1;
    int made = 0;
    double tf0 = TICK();
    gf_get_stats(a->A, &s0);
    gf_add_fix(a->A, f);
    gf_get_stats(a->A, &s1);
    if (s1.n_gnss_nodes != s0.n_gnss_nodes) {
        gf_pose p;
        if (gf_get_pose(a->A, &p) == 0 && (p.status & GF_ST_NO_ODOM)) {
            gf_auto_out *o = &a->gout;
            memset(o, 0, sizeof *o);
            o->t = p.t; memcpy(o->p, p.p, sizeof o->p); memcpy(o->q, p.q, sizeof o->q); o->status = p.status; o->have_sm = 1;
            memcpy(o->p_sm, p.p, sizeof o->p_sm); memcpy(o->q_sm, p.q, sizeof o->q_sm); o->status_sm = p.status;
            a->have_gout = 1; made = 1;
        }
    }
    if (a->nq == a->qcap) {
        int nc = a->qcap ? a->qcap * 2 : 64; pfix *np = (pfix *)realloc(a->q, sizeof(pfix) * (size_t)nc);
        if (!np) return made;
        a->q = np; a->qcap = nc;
    }
    a->q[a->nq].t = f->t; memcpy(a->q[a->nq].z, f->p, 3 * sizeof(double)); a->q[a->nq].sg = f->sigma_h; ++a->nq;
    a->tm.fix += TICK() - tf0; ++a->tm.n_fix;
    return made;
}

static void ew(double *m, double x, double dt, double tau) { double k = exp(-dt / tau); *m = k * *m + (1.0 - k) * x; }

/* statistics of one fix at the moment it is paired with the stream (the fit has not seen it yet) */
static void fix_stats(gf_auto *a, const pfix *f)
{
    double pr[3];
    double dt = a->lfix_t > 0 ? f->t - a->lfix_t : 1.0;
    if (dt <= 0) dt = 1e-3;
    if (a->nl == 0) { a->sr_ew = f->sg; a->sw2_ew = 0; }
    else ew(&a->sr_ew, f->sg, dt, 120.0);
    /* white noise from second differences of three consecutive fixes (E[(z2 - 2 z1 + z0)^2] = 6 sigma^2 for white noise) */
    if (a->nl >= 2 && dt < 5.0) {
        double d0 = f->z[0] - 2 * a->lz[0] + a->lz2[0], d1 = f->z[1] - 2 * a->lz[1] + a->lz2[1];
        double v = (d0 * d0 + d1 * d1) / 12.0;   /* per axis */
        if (a->nl == 2) a->sw2_ew = v; else ew(&a->sw2_ew, v, dt, 120.0);
    }
    memcpy(a->lz2, a->lz, sizeof a->lz); memcpy(a->lz, f->z, sizeof a->lz); a->lt2 = a->lt; a->lt = f->t; ++a->nl; a->lfix_t = f->t;
    if (gf_georef_predict(a->G, f->t, pr)) {
        double e2 = (pr[0] - f->z[0]) * (pr[0] - f->z[0]) + (pr[1] - f->z[1]) * (pr[1] - f->z[1]);
        if (a->nea == 0) { a->ef2 = a->es2 = e2; }
        else { ew(&a->ef2, e2, dt, a->c.tau_fast_s); ew(&a->es2, e2, dt, a->c.tau_slow_s); }
        a->ea2 += (e2 - a->ea2) / (double)(a->nea + 1); ++a->nea; a->elast = e2;
    }
}

static void nlerp(const double a[4], const double b[4], double w, double o[4])
{
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3], sg = d < 0 ? -1.0 : 1.0, n = 0;
    for (int i = 0; i < 4; ++i) { o[i] = (1.0 - w) * a[i] + w * sg * b[i]; n += o[i] * o[i]; }
    n = sqrt(n); if (n > 0) for (int i = 0; i < 4; ++i) o[i] /= n;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------------ the switch */
/* returns the target state (1 = georef) and whether the georef pose may be output; updates a->st / a->hold */
static void decide(gf_auto *a, int have_geo, double t)
{
    const gf_auto_config *c = &a->c;
    gf_auto_sig *s = &a->sig;
    double es = sqrt(s->e_slow), el = sqrt(s->e_last);
    int elig;
    if (c->policy == GF_AUTO_SMOOTHER || !have_geo) { a->st = 0.0; s->w_target = 0.0; return; }
    if (c->policy == GF_AUTO_GEOREF) { a->st = 1.0; s->w_target = 1.0; return; }
    elig = s->n_pairs >= c->min_pairs && es > 0 && t - a->t_first >= c->min_span_s && s->sig_rep >= c->sig_min;
    if (c->metric_only && c->geo.scale_sigma > 10.0) elig = 0;
    if (c->stream_mode == 0 && a->n_new > 0) elig = 0;   /* a raw odometry that restarted in a new frame: ONE similarity cannot describe it */
    if (elig) {
        double noise = s->sig_rep > c->sig_floor ? s->sig_rep : c->sig_floor;
        double rho = es / noise, lim = es > 2.0 * noise ? es : 2.0 * noise;
        s->rho = rho;
        if (a->st > 0.5 && c->fail_k > 0 && el > c->fail_k * lim) { a->st = 0.0; a->w = 0.0; a->hold = t + c->dwell_s; }
        else if (a->st < 0.5 && t >= a->hold && rho <= c->rho_on && s->distrust <= c->distr_on) a->st = 1.0;
        else if (a->st > 0.5 && (rho >= c->rho_off || s->distrust >= c->distr_off)) { a->st = 0.0; a->hold = t + c->dwell_s; }
    } else a->st = 0.0;
    s->w_target = a->st;
}

int gf_auto_add_odom(gf_auto *a, double t, const double p[3], const double q[4], unsigned flags)
{
    gf_pose pa, pb;
    double c0 = TICK(), c1, c2, c3;
    int rc = gf_add_odom(a->A, t, p, q, flags);
    if (rc) return rc;
    c1 = TICK();
    if (flags & GF_ODOM_NEW_FRAME) ++a->n_new;
    if ((flags & GF_ODOM_GAP) && !(flags & GF_ODOM_NEW_FRAME)) ++a->n_gap;
    int have_a = gf_get_pose(a->A, &pa) == 0;
    /* the georef stream sample */
    double sp[3] = { p[0], p[1], p[2] }, sq[4] = { q[0], q[1], q[2], q[3] };
    int have_s = 1;
    if (a->B) {
        have_s = 0;
        if (gf_add_odom(a->B, t, p, q, flags) == 0 && gf_get_pose(a->B, &pb) == 0 && (pb.status & GF_ST_INIT)) {
            memcpy(sp, pb.p, sizeof sp); memcpy(sq, pb.q, sizeof sq); have_s = 1;
        }
    }
    c2 = TICK();
    double dt = a->last_t > 0 ? t - a->last_t : 0.0;
    int have_geo = 0; double pg[3] = { 0, 0, 0 }, qg[4] = { 0, 0, 0, 1 };
    if (have_s) {
        gf_georef_add_pose(a->G, t, sp);
        int k = 0;
        while (k < a->nq && a->q[k].t <= t) {
            fix_stats(a, &a->q[k]);
            gf_georef_add_fix(a->G, a->q[k].t, a->q[k].z, a->q[k].sg, 1);
            ++k;
        }
        if (k) { memmove(a->q, a->q + k, sizeof(pfix) * (size_t)(a->nq - k)); a->nq -= k; }
        if (gf_georef_map(a->G, sp, pg)) {
            double psi; gf_georef_fit(a->G, &psi, NULL, NULL, NULL);
            double cs = cos(0.5 * psi), sn = sin(0.5 * psi);
            qg[0] = cs * sq[0] - sn * sq[1]; qg[1] = cs * sq[1] + sn * sq[0]; qg[2] = cs * sq[2] + sn * sq[3]; qg[3] = cs * sq[3] - sn * sq[2];
            have_geo = 1;
        }
    }
    /* signals */
    gf_auto_sig *s = &a->sig;
    s->t = t; s->n_new_frame = a->n_new; s->n_gap = a->n_gap;
    s->have_fit = gf_georef_fit(a->G, &s->psi, &s->scale, &s->sres, &s->n_pairs);
    s->e_fast = a->ef2; s->e_slow = a->es2; s->e_all = a->ea2; s->e_last = a->elast;
    s->sig_rep = a->sr_ew; s->sig_white = sqrt(a->sw2_ew);
    if (have_a && have_geo && (pa.status & GF_ST_INIT)) {
        double e2 = (pa.p[0] - pg[0]) * (pa.p[0] - pg[0]) + (pa.p[1] - pg[1]) * (pa.p[1] - pg[1]);
        if (a->dis2 == 0) a->dis2 = e2; else ew(&a->dis2, e2, dt > 0 ? dt : 0.1, 60.0);
    }
    s->dis_fast = sqrt(a->dis2);
    if (have_a) ew(&a->distr, (pa.status & GF_ST_ODOM_DISTRUSTED) ? 1.0 : 0.0, dt > 0 ? dt : 0.1, 60.0);
    s->distrust = a->distr;
    if (a->t_first == 0 && a->nl_odom == 0) a->t_first = t;
    ++a->nl_odom;
    decide(a, have_geo, t);
    /* cross-fade */
    {
        const int hs = have_a && (pa.status & GF_ST_INIT);
        double d = a->st - a->w, step = dt > 0 ? (d > 0 ? dt / (a->c.blend_s > 0 ? a->c.blend_s : 1e-9) : dt / (a->c.down_s > 0 ? a->c.down_s : 1e-9)) : 1.0;
        if (!have_geo) a->w = 0.0;
        else if (!hs) a->w = a->st > 0.5 ? 1.0 : 0.0;
        else a->w += d > step ? step : (d < -step ? -step : d);
        if (a->w < 0) a->w = 0;
        if (a->w > 1) a->w = 1;
    }
    a->last_t = t;
    gf_auto_out *o = &a->out;
    memset(o, 0, sizeof *o);
    o->t = t; o->have_sm = have_a; o->have_geo = have_geo;
    if (have_a) { memcpy(o->p_sm, pa.p, sizeof o->p_sm); memcpy(o->q_sm, pa.q, sizeof o->q_sm); o->status_sm = pa.status; }
    if (have_geo) { memcpy(o->p_geo, pg, sizeof pg); memcpy(o->q_geo, qg, sizeof qg); }
    if (a->c.policy == GF_AUTO_SMOOTHER) {
        if (!have_a) { a->have_out = 0; return 0; }
        memcpy(o->p, pa.p, sizeof o->p); memcpy(o->q, pa.q, sizeof o->q); o->status = pa.status; o->w_geo = 0.0;
    } else if (a->c.policy == GF_AUTO_GEOREF) {
        if (!have_geo) { a->have_out = 0; return 0; }
        memcpy(o->p, pg, sizeof pg); memcpy(o->q, qg, sizeof qg); o->w_geo = 1.0; o->status = GF_ST_INIT;
    } else if (have_a && have_geo && (pa.status & GF_ST_INIT)) {
        const double w = a->w;
        for (int i = 0; i < 3; ++i) o->p[i] = (1.0 - w) * pa.p[i] + w * pg[i];
        nlerp(pa.q, qg, w, o->q); o->w_geo = w; o->status = pa.status;
    } else if (have_geo && a->st > 0.5 && !(have_a && (pa.status & GF_ST_INIT))) {
        memcpy(o->p, pg, sizeof pg); memcpy(o->q, qg, sizeof qg); o->w_geo = 1.0; o->status = GF_ST_INIT;
    } else if (have_a) {
        memcpy(o->p, pa.p, sizeof o->p); memcpy(o->q, pa.q, sizeof o->q); o->w_geo = 0.0; o->status = pa.status;
    } else { a->have_out = 0; return 0; }
    a->have_out = 1;
    c3 = TICK();
    a->tm.sm += c1 - c0; a->tm.st += c2 - c1; a->tm.geo += c3 - c2; ++a->tm.n_odom;
    return 0;
}

int gf_auto_get(const gf_auto *a, gf_auto_out *out) { if (!a->have_out) return -1; *out = a->out; return 0; }
int gf_auto_get_gnss_only(const gf_auto *a, gf_auto_out *out) { if (!a->have_gout) return -1; *out = a->gout; return 0; }
void gf_auto_signals(const gf_auto *a, gf_auto_sig *s) { *s = a->sig; }
