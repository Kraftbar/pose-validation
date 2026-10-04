/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources. See gf_gait.h. */
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "gf_gait.h"

#define NSTEP 64
#define NHIST 400
#define NFIX  120

typedef struct { double t, amp; } step_t;
typedef struct { double t, odo, hd; } hist_t;
typedef struct { double t, e, n, s; } fix_t;

struct gf_gait {
    gf_gait_config c;
    int n; int have_prev; double t_prev;
    double slow, s1, s2, var_sm; long nstd;
    double hp2, gn;
    double p1, p2, tp1, last_step;
    step_t steps[NSTEP]; int nsteps;
    double grav[3], heading;
    double k, mc, rel;
    double odo, odo_base;
    hist_t hist[NHIST]; int nhist;
    fix_t fixes[NFIX]; int nfix;
    double on_c, on_d, on_last;
};

void gf_gait_config_default(gf_gait_config *c)
{
    memset(c, 0, sizeof *c);
    c->tau_slow = 0.8; c->tau_sm = 0.03; c->tau_std = 15.0; c->std_k = 0.35; c->thr_floor = 0.3; c->min_step_dt = 0.3;
    c->window_s = 6.0; c->min_steps = 4; c->max_last_age = 1.2; c->cad_min = 1.0; c->cad_max = 2.8;
    c->tau_stat = 1.5; c->stat_hp_rms = 0.12; c->stat_gyro = 0.12; c->tau_grav = 1.0;
    c->model_c = 0.389; c->model_p = 2.0; c->rel_sigma = 0.20; c->abs_sigma = 0.10; c->user_rel_sigma = 0.10;
    c->online = 0; c->on_win = 30.0; c->on_edge = 8.0; c->on_min_dist = 15.0; c->on_max_turn = 20.0 * 3.14159265358979323846 / 180.0;
    c->on_eval_dt = 5.0; c->on_sigma_max = 20.0; c->on_prior_m = 150.0; c->k_min = 0.6; c->k_max = 1.6;
}

static void reset(gf_gait *g)
{
    g->n = 0; g->have_prev = 0; g->t_prev = 0; g->slow = g->s1 = g->s2 = g->var_sm = 0; g->nstd = 0; g->hp2 = g->gn = 0;
    g->p1 = g->p2 = g->tp1 = 0; g->last_step = -1e30; g->nsteps = 0;
    g->grav[0] = g->grav[1] = g->grav[2] = 0; g->heading = 0;
    g->k = 1.0; g->mc = g->c.model_c; g->rel = g->c.rel_sigma;
    g->odo = g->odo_base = 0; g->nhist = 0; g->nfix = 0; g->on_c = g->on_d = 0; g->on_last = -1e30;
}

gf_gait *gf_gait_create(const gf_gait_config *c)
{
    gf_gait *g = (gf_gait *)calloc(1, sizeof *g);
    if (!g) return NULL;
    if (c) g->c = *c; else gf_gait_config_default(&g->c);
    reset(g);
    return g;
}

void gf_gait_destroy(gf_gait *g) { free(g); }

void gf_gait_set_model(gf_gait *g, double model_c)
{
    if (!g) return;
    g->mc = model_c; g->rel = g->c.user_rel_sigma;
}

static double ema(double y, double x, double tau, double dt) { double al = dt / (tau + dt); return y + al * (x - y); }

static void hist_add(gf_gait *g, double t)
{
    if (g->nhist == 0 || t - g->hist[g->nhist - 1].t >= 1.0 - 1e-9) {
        if (g->nhist == NHIST) { memmove(g->hist, g->hist + 1, sizeof(hist_t) * (NHIST - 1)); g->nhist--; }
        g->hist[g->nhist].t = t; g->hist[g->nhist].odo = g->odo_base; g->hist[g->nhist].hd = g->heading; g->nhist++;
    }
}

int gf_gait_estimate(const gf_gait *g, double t, double window_s, gf_gait_est *out)
{
    const gf_gait_config *c = &g->c;
    double W = window_s > 0 ? window_s : c->window_s;
    double first = 0, last = 0; int n = 0;
    for (int i = 0; i < g->nsteps; ++i) {
        double ts = g->steps[i].t;
        if (t - W < ts && ts <= t) { if (n == 0) first = ts; last = ts; ++n; }
    }
    memset(out, 0, sizeof *out);
    out->t = t; out->window_s = W; out->n_steps = n; out->cadence = 0.0; out->state = GF_GAIT_OTHER; out->speed = 0.0; out->sigma = 1.0;
    if (g->n < 2) return 0;
    int stationary = sqrt(g->hp2) < c->stat_hp_rms && g->gn < c->stat_gyro && (n < 2 || t - last > 2.0);
    if (stationary) { out->state = GF_GAIT_STATIONARY; out->speed = 0.0; out->sigma = 0.05; return 0; }
    if (n >= c->min_steps && t - last <= c->max_last_age && last > first) {
        double cad = (double)(n - 1) / (last - first);
        out->cadence = cad;
        if (c->cad_min <= cad && cad <= c->cad_max) {
            double v = g->k * g->mc * pow(cad, c->model_p);
            if (v > 0.0) {
                out->state = GF_GAIT_WALK; out->speed = v;
                double r = g->rel * v;
                out->sigma = sqrt(r * r + c->abs_sigma * c->abs_sigma);
            }
        }
    }
    return 0;
}

int gf_gait_push(gf_gait *g, double t, const double a[3], const double w[3])
{
    const gf_gait_config *c = &g->c;
    double an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    double wn = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    int stepped = 0;
    if (!g->have_prev) {
        g->slow = an; g->s1 = 0.0; g->s2 = 0.0;
        g->grav[0] = a[0]; g->grav[1] = a[1]; g->grav[2] = a[2]; g->gn = wn;
        g->t_prev = t; g->have_prev = 1; g->n = 1;
        g->p1 = g->p2 = 0.0; g->tp1 = t;
        hist_add(g, t);
        return 0;
    }
    double dt = t - g->t_prev;
    if (!(dt > 0)) return 0;
    g->t_prev = t;
    g->slow = ema(g->slow, an, c->tau_slow, dt);
    double hp = an - g->slow;
    g->s1 = ema(g->s1, hp, c->tau_sm, dt);
    g->s2 = ema(g->s2, g->s1, c->tau_sm, dt);
    double sm = g->s2;
    g->n++;
    g->nstd++;
    double al = dt / (c->tau_std + dt), al1 = 1.0 / (double)g->nstd;
    if (al1 > al) al = al1;
    g->var_sm += al * (sm * sm - g->var_sm);
    g->hp2 = ema(g->hp2, hp * hp, c->tau_stat, dt);
    g->gn = ema(g->gn, wn, c->tau_stat, dt);
    double thr = c->std_k * sqrt(g->var_sm);
    if (thr < c->thr_floor) thr = c->thr_floor;
    if (g->p1 > g->p2 && g->p1 >= sm && g->p1 > thr && (g->tp1 - g->last_step) >= c->min_step_dt) {
        g->last_step = g->tp1;
        if (g->nsteps == NSTEP) { memmove(g->steps, g->steps + 1, sizeof(step_t) * (NSTEP - 1)); g->nsteps--; }
        g->steps[g->nsteps].t = g->tp1; g->steps[g->nsteps].amp = g->p1; g->nsteps++;
        stepped = 1;
    }
    g->p2 = g->p1; g->p1 = sm; g->tp1 = t;
    double ag = dt / (c->tau_grav + dt);
    for (int i = 0; i < 3; ++i) g->grav[i] = g->grav[i] + ag * (a[i] - g->grav[i]);
    double gnorm = sqrt(g->grav[0] * g->grav[0] + g->grav[1] * g->grav[1] + g->grav[2] * g->grav[2]);
    if (gnorm > 1e-6) {
        double wz = (w[0] * g->grav[0] + w[1] * g->grav[1] + w[2] * g->grav[2]) / gnorm;
        g->heading += wz * dt;
    }
    gf_gait_est e; gf_gait_estimate(g, t, 0.0, &e);
    if (e.state == GF_GAIT_WALK) { g->odo += e.speed * dt; g->odo_base += (e.speed / g->k) * dt; }
    hist_add(g, t);
    return stepped;
}

static int hist_at(const gf_gait *g, double t, double *odo, double *hd)
{
    int n = g->nhist;
    if (n == 0 || t < g->hist[0].t || t > g->hist[n - 1].t) return 0;
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) { int m = (lo + hi) / 2; if (g->hist[m].t <= t) lo = m; else hi = m; }
    double d = g->hist[hi].t - g->hist[lo].t; if (d < 1e-9) d = 1e-9;
    double w = (t - g->hist[lo].t) / d;
    *odo = (1 - w) * g->hist[lo].odo + w * g->hist[hi].odo;
    *hd = (1 - w) * g->hist[lo].hd + w * g->hist[hi].hd;
    return 1;
}

void gf_gait_gnss_fix(gf_gait *g, double t, double e, double n, double sigma_h)
{
    const gf_gait_config *c = &g->c;
    if (!c->online) return;
    if (g->nfix == NFIX) { memmove(g->fixes, g->fixes + 1, sizeof(fix_t) * (NFIX - 1)); g->nfix--; }
    g->fixes[g->nfix].t = t; g->fixes[g->nfix].e = e; g->fixes[g->nfix].n = n; g->fixes[g->nfix].s = sigma_h; g->nfix++;
    if (t - g->on_last < c->on_eval_dt) return;
    int na = 0, nb = 0;
    double ta = 0, tb = 0, ea = 0, eb = 0, no_a = 0, no_b = 0, va = 0, vb = 0, smax = 0;
    for (int i = 0; i < g->nfix; ++i) {
        const fix_t *f = &g->fixes[i];
        if (f->t >= t - c->on_win && f->t <= t - c->on_win + c->on_edge) { ta += f->t; ea += f->e; no_a += f->n; va += f->s * f->s; ++na; if (f->s > smax) smax = f->s; }
        if (f->t >= t - c->on_edge && f->t <= t) { tb += f->t; eb += f->e; no_b += f->n; vb += f->s * f->s; ++nb; if (f->s > smax) smax = f->s; }
    }
    if (na < 3 || nb < 3) return;
    if (smax > c->on_sigma_max) return;
    ta /= na; tb /= nb; ea /= na; no_a /= na; eb /= nb; no_b /= nb;
    double sa = va / na / na, sb = vb / nb / nb;
    double oa, ha, ob, hb;
    if (!hist_at(g, ta, &oa, &ha) || !hist_at(g, tb, &ob, &hb)) return;
    g->on_last = t;
    double dist = ob - oa;
    if (dist < c->on_min_dist) return;
    if (fabs(hb - ha) > c->on_max_turn) return;
    double d2 = (eb - ea) * (eb - ea) + (no_b - no_a) * (no_b - no_a) - 2.0 * (sa + sb);
    double chord = d2 > 0 ? sqrt(d2) : 0.0;
    g->on_c += chord; g->on_d += dist;
    double m0 = c->on_prior_m * (c->on_win - c->on_edge) / c->on_eval_dt;
    double k = (g->on_c + m0 * 1.0) / (g->on_d + m0);
    if (k < c->k_min) k = c->k_min;
    if (k > c->k_max) k = c->k_max;
    g->k = k;
    double rel = c->rel_sigma / sqrt(1.0 + g->on_d / (m0 * 1.0));
    g->rel = rel > 0.10 ? rel : 0.10;
}

double gf_gait_heading(const gf_gait *g) { return g->heading; }
double gf_gait_scale(const gf_gait *g) { return g->k; }
double gf_gait_odometer(const gf_gait *g) { return g->odo; }
