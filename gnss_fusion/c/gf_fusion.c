/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 * See gf_fusion.h for the model. */
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "gf_fusion.h"
#include "gf_math.h"

#define PEND_MAX 32

typedef struct {
    double t;
    double pl[3];          /* odometry position: gravity-aligned, scaled by unit (internal units) */
    double a[3];           /* antenna lever arm in the aligned odometry axes (metres) */
    double ra[9], unit;    /* alignment of this node's odometry frame */
    double x[5], xo[5];    /* state; output estimate (value when the node was last solved at the head) */
    double z[3], sg[3];    /* assigned fix */
    double fdt;            /* |t_fix - t_node| */
    double v[3], sv;       /* fix velocity */
    double cum;            /* path length up to this node [m] */
    long   id;             /* absolute node id */
    int    lk;             /* link from the previous node: 0 odometry, 1 gap (same frame), 2 new frame */
    double rho, sig;       /* consistency test diagnostics: window similarity-fit residual, GNSS noise estimate [m] */
    double Rr[9];          /* GNSS-only node: orientation of the last odometry (aligned), held */
    double ch;             /* cumulative horizontal odometry path (internal units) along odometry links */
    double spd_v, spd_sg, spd_w, spd_fdt;   /* speed measurement assigned to this node: mean speed over [t - spd_w, t], sigma, |t_meas - t_node| */
    unsigned char has_spd, spd_stat, zupt;   /* zupt: covered by a stationary measurement (zero displacement to the previous node) */
    unsigned char loose;     /* GF_ODOM_LOOSE seen since the previous node */
    unsigned char has_fix, gated, has_vel, bad, prov, gnss;   /* gnss: node made from a fix alone (no odometry) */
} node_t;
#define OFIX(n) ((n)->has_fix && !(n)->gnss)
#define UFIX(n) (OFIX(n) && !(n)->gated)

struct gf_t {
    gf_config c;
    int causal;
    node_t *nd; int n, cap; long next_id;
    int ready;                    /* causal: initialised; batch: solved at least once */
    int have_grid; double t0; long m_next;
    int have_last; double lt; double lraw[3]; double lR[9];   /* last odometry sample: raw position, aligned rotation */
    double cur_ra[9], cur_unit; int have_up, up_pending; double up_next[3];
    int loose_pend;               /* GF_ODOM_LOOSE seen since the last node was made */
    long pend_frame;              /* absolute id of the first node of a not yet aligned frame, -1 if none */
    gf_fix pend[PEND_MAX]; int npend;
    struct { double t, v, sg, w; unsigned flags; } pspd[PEND_MAX]; int nspd;   /* speed measurements waiting for their node */
    int grow_done;                /* causal: the growing initial window has ended (time cap or fix geometry strong enough) */
    double zmin[2], zmax[2], sg_sum; long sg_n;   /* extent of the fixes seen and their mean reported sigma (geometry test of the growing window) */
    double *ws; size_t ws_cap;    /* solver workspace */
    int *iws; int iws_cap;
    gf_stats st;
};

void gf_config_robust(gf_config *c)
{
    c->trust = 1; c->trust_long_s = 120.0; c->gate_chi2 = 16.27; c->robust_init = 1;
    c->scale_rw_mono = 0.05; c->scale_prior_mono = 2.0;   /* monocular scale drifts; the weak prior is only a regulariser */
    c->trust_start_m = 3.0;      /* consistency fits robust to bursts of common-offset fixes */
    c->trust_state_k = 1.6;      /* monocular scale collapse / re-scale: fit scale vs scale state */
    c->grow_s = 120.0; c->grow_ratio = 20.0;           /* metric odometry: no frozen nodes during the first 2 minutes (yaw / scale information is kept while it is still scarce) */
    c->scale_min = 0.25; c->scale_max = 4.0;   /* the scale state cannot flip sign / collapse into an attractor */
    c->gnss_only_nodes = 1;      /* the output continues through odometry loss */
}

void gf_config_default(gf_config *c)
{
    memset(c, 0, sizeof *c);
    c->node_dt = 1.0; c->window_s = 30.0;
    c->batch_iters = 25; c->causal_iters = 4; c->init_iters = 10; c->settle_nodes = 10; c->init_wait_s = 30.0;
    c->keep_history = 0;
    c->odom_sp = 0.05; c->odom_kp = 0.02; c->yaw_rw = 0.5 * 3.14159265358979323846 / 180.0; c->scale_rw = 0.003; c->scale_prior = 0.2; c->scale_rw_mono = 0.003; c->scale_prior_mono = 0.2;
    c->metric_scale = 1; c->gravity_aligned = 1;
    c->loss = GF_LOSS_HUBER; c->loss_k = 2.5; c->min_sigma = 0.02; c->gate_chi2 = 0.0; c->gate_floor = 1.0; c->gate_min_fixes = 5; c->robust_init = 0;
    c->gap_s = 2.0; c->max_speed = 0.0; c->link_speed = 3.0; c->seg_min_extent = 4.0;
    c->trust = 0; c->trust_window_s = 30.0; c->trust_min_fixes = 6; c->trust_scale_k = 1.5; c->trust_rho_k = 4.0;
    c->trust_rho_min = 5.0; c->trust_scale_k_mono = 0.0; c->trust_long_s = 0.0; c->trust_long_and = 0; c->trust_rho_long = 8.0; c->trust_spread_k = 3.0; c->trust_q_scale = 10.0;
    c->drift_rate = 0.02; c->blackout_s = 10.0;
    c->speed_on = 0; c->speed_k = 2.0; c->speed_sigma_scale = 1.0; c->speed_link_sigma = 0.3; c->zupt_sigma = 0.05; c->speed_align = 0; c->speed_scale_rw_rel = 0.0; c->speed_scale_lim = 1e5; c->speed_scale_rw = 0.0; c->loose_k = 1.0; c->speed_align_metric = 0;
    c->trust_start_m = 0.0; c->trust_state_k = 0.0; c->gnss_only_nodes = 0; c->grow_s = 0.0; c->grow_s_mono = 0.0; c->grow_ratio = 0.0; c->scale_min = 0.0; c->scale_max = 0.0;
}

gf_t *gf_create(const gf_config *c, int causal)
{
    gf_t *g = (gf_t *)calloc(1, sizeof *g);
    if (!g) return NULL;
    if (c) g->c = *c; else gf_config_default(&g->c);
    g->causal = causal ? 1 : 0;
    gf_reset(g);
    return g;
}

void gf_destroy(gf_t *g)
{
    if (!g) return;
    free(g->nd); free(g->ws); free(g->iws); free(g);
}

void gf_reset(gf_t *g)
{
    g->n = 0; g->next_id = 0; g->ready = 0; g->have_grid = 0; g->have_last = 0; g->have_up = 0; g->up_pending = 0;
    g->pend_frame = -1; g->npend = 0; g->nspd = 0; g->cur_unit = 1.0; g->loose_pend = 0;
    g->grow_done = 0; g->zmin[0] = g->zmin[1] = 1e300; g->zmax[0] = g->zmax[1] = -1e300; g->sg_sum = 0; g->sg_n = 0;
    for (int i = 0; i < 9; ++i) g->cur_ra[i] = (i % 4 == 0) ? 1.0 : 0.0;
    memset(&g->st, 0, sizeof g->st);
}

/* ------------------------------------------------------------------ storage */
static int ensure_nodes(gf_t *g)
{
    if (g->n < g->cap) return 0;
    int nc = g->cap ? g->cap * 2 : 256;
    node_t *p = (node_t *)realloc(g->nd, (size_t)nc * sizeof *p);
    if (!p) return GF_ERR_ALLOC;
    g->nd = p; g->cap = nc;
    return 0;
}
static int ensure_ws(gf_t *g, int n)
{
    size_t need = (size_t)n * 85 + 64;   /* D, O, G: 25 each; b, dx: 5 each */
    if (g->ws_cap < need) {
        double *p = (double *)realloc(g->ws, need * sizeof(double));
        if (!p) return GF_ERR_ALLOC;
        g->ws = p; g->ws_cap = need;
    }
    return 0;
}
static int ensure_iws(gf_t *g, int n)
{
    if (g->iws_cap < n) {
        int *p = (int *)realloc(g->iws, (size_t)n * sizeof(int));
        if (!p) return GF_ERR_ALLOC;
        g->iws = p; g->iws_cap = n;
    }
    return 0;
}
static int idx_of_id(const gf_t *g, long id)
{
    if (g->n == 0) return -1;
    long k = id - g->nd[0].id;
    return (k >= 0 && k < g->n) ? (int)k : -1;
}
/* growing initial window (no frozen nodes): metric odometry, until grow_s seconds or, if grow_ratio > 0, until the fixes span grow_ratio times their
 * reported sigma (then yaw / scale are geometrically well determined; with a stationary start or poor fixes this takes long) */
static double grow_limit(const gf_t *g) { return g->c.metric_scale ? g->c.grow_s : g->c.grow_s_mono; }
static int growing(const gf_t *g) { return grow_limit(g) > 0 && !g->grow_done; }
static void update_grow(gf_t *g)
{
    double gs = grow_limit(g);
    if (gs <= 0 || g->grow_done || g->n == 0) return;
    if (g->nd[g->n - 1].t - g->t0 >= gs) { g->grow_done = 1; return; }
    if (g->c.grow_ratio > 0 && g->sg_n >= 5) {
        double ext = hypot(g->zmax[0] - g->zmin[0], g->zmax[1] - g->zmin[1]);
        if (ext >= g->c.grow_ratio * g->sg_sum / (double)g->sg_n) g->grow_done = 1;
    }
}
/* causal, bounded memory: drop nodes older than the window (keep a pending frame start) */
static void trim_nodes(gf_t *g)
{
    if (!g->causal || g->c.keep_history) return;
    if (!g->ready && g->n < 100000) return;   /* the initial alignment still needs the first nodes */
    update_grow(g);
    int W = (int)(g->c.window_s / g->c.node_dt + 0.5); if (W < 3) W = 3;
    if (growing(g)) return;
    int keep = W + 8;
    if (g->n < 4 * keep) return;
    int from = g->n - keep;
    if (g->pend_frame >= 0) { int pi = idx_of_id(g, g->pend_frame); if (pi >= 0 && pi - 1 < from) from = pi > 0 ? pi - 1 : 0; }
    if (from <= 0) return;
    memmove(g->nd, g->nd + from, (size_t)(g->n - from) * sizeof(node_t));
    g->n -= from;
}

/* ------------------------------------------------------------------ robust weights */
static double loss_weight(const gf_config *c, double e)
{
    double ae = fabs(e);
    if (c->loss == GF_LOSS_HUBER) return ae <= c->loss_k ? 1.0 : c->loss_k / ae;
    if (c->loss == GF_LOSS_CAUCHY) { double u = e / c->loss_k; return 1.0 / (1.0 + u * u); }
    return 1.0;
}

/* ------------------------------------------------------------------ factor accumulation */
/* one residual row: r, Ji (5), Jj (5, or NULL); node ki (window index, may be <0 = frozen), kj */
static void acc_row(double *D, double *O, double *b, int ki, int kj, double r, const double *Ji, const double *Jj, double w)
{
    if (Jj) {
        double *Dj = D + 25 * kj;
        for (int a = 0; a < GF_NS; ++a) {
            if (Jj[a] == 0.0) continue;
            for (int c = 0; c < GF_NS; ++c) Dj[GF_NS * a + c] += w * Jj[a] * Jj[c];
            b[GF_NS * kj + a] -= w * Jj[a] * r;
        }
    }
    if (Ji && ki >= 0) {
        double *Di = D + 25 * ki;
        for (int a = 0; a < GF_NS; ++a) {
            if (Ji[a] == 0.0) continue;
            for (int c = 0; c < GF_NS; ++c) Di[GF_NS * a + c] += w * Ji[a] * Ji[c];
            b[GF_NS * ki + a] -= w * Ji[a] * r;
        }
        if (Jj && ki >= 0 && kj >= 0) {
            double *Oi = O + 25 * ki;
            for (int a = 0; a < GF_NS; ++a)
                for (int c = 0; c < GF_NS; ++c) Oi[GF_NS * a + c] += w * Ji[a] * Jj[c];
        }
    }
}

static int link_is_bad(const gf_t *g, int i, int j)
{
    return g->nd[i].bad || g->nd[j].bad;
}

/* fix residual (antenna) for node i */
static void fix_residual(const node_t *n, double *r)
{
    double ra[3];
    gf_rz_vec(n->x[0], n->a, ra);
    for (int k = 0; k < 3; ++k) r[k] = n->x[1 + k] + ra[k] - n->z[k];
}

static void update_gates(gf_t *g, int lo, int hi)
{
    double thr = g->c.gate_chi2;
    int nfix = 0, ngate = 0;
    /* a fix can only be judged against a prediction once its odometry segment (since the last gap / new frame) has some fixes of its
     * own: right after a break the state is unconstrained and every fix would look like an outlier */
    int s0 = lo, steps = 0;
    while (s0 > 0 && g->nd[s0].lk == 0 && steps++ < 4000) --s0;
    if (ensure_iws(g, hi - s0 + 2)) return;
    int *tot = g->iws;
    for (int i = s0; i <= hi; ) {
        int j = i + 1;
        while (j <= hi && g->nd[j].lk == 0) ++j;
        int cnt = 0;
        for (int k = i; k < j; ++k) cnt += OFIX(&g->nd[k]);
        for (int k = i; k < j; ++k) tot[k - s0] = cnt;
        i = j;
    }
    for (int i = lo; i <= hi; ++i) {
        node_t *n = &g->nd[i];
        if (!OFIX(n)) continue;
        if (tot[i - s0] < g->c.gate_min_fixes) { n->gated = 0; continue; }
        double r[3]; fix_residual(n, r);
        double e2 = 0, fl2 = g->c.gate_floor * g->c.gate_floor;   /* floor: the prediction from odometry is not exact either */
        for (int k = 0; k < 3; ++k) e2 += r[k] * r[k] / (n->sg[k] * n->sg[k] + fl2);
        n->gated = e2 > thr;
        ++nfix; ngate += n->gated;
    }
    if (nfix > 0 && 2 * ngate > nfix)   /* the model, not the fixes, is probably wrong: keep them */
        for (int i = lo; i <= hi; ++i) g->nd[i].gated = 0;
}

/* ------------------------------------------------------------------ speed (gait) prior */
static double speed_weight(const gf_config *c, double e)
{
    double ae = fabs(e);
    return (c->speed_k > 0 && ae > c->speed_k) ? c->speed_k / ae : 1.0;
}

/* horizontal odometry path (internal units) and duration of the connected odometry links ending at node k and reaching back at most W seconds;
 * returns 1 if at least one link is used */
static int speed_span(const gf_t *g, int k, double W, double *dh, double *T)
{
    const node_t *nk = &g->nd[k];
    int i = k;
    while (i > 0) {
        const node_t *ni = &g->nd[i];
        if (ni->lk != 0 || ni->gnss || g->nd[i - 1].gnss) break;   /* distrusted links still measure the odometry's own path length, which is what the scale state multiplies */
        if (nk->t - g->nd[i - 1].t > W + 0.5 * g->c.node_dt + 1e-6) break;
        --i;
    }
    *T = nk->t - g->nd[i].t; *dh = nk->ch - g->nd[i].ch;
    return i < k;
}

/* Gauss-Newton on nodes lo..hi (inclusive); node lo-1 (if any) is a frozen boundary. */
static int solve_window(gf_t *g, int lo, int hi, int iters)
{
    const gf_config *c = &g->c;
    int n = hi - lo + 1;
    const double sprior = c->metric_scale ? c->scale_prior : c->scale_prior_mono;
    const double srw = (c->speed_on && c->speed_scale_rw > 0) ? c->speed_scale_rw : (c->metric_scale ? c->scale_rw : c->scale_rw_mono);
    if (n <= 0) return 0;
    if (ensure_ws(g, n)) return GF_ERR_ALLOC;
    double *D = g->ws, *O = D + 25 * (size_t)n, *b = O + 25 * (size_t)n, *dx = b + 5 * (size_t)n, *G = dx + 5 * (size_t)n;
    g->st.n_solves++;
    for (int it = 0; it < iters; ++it) {
        if (c->gate_chi2 > 0) update_gates(g, lo, hi);
        memset(D, 0, sizeof(double) * 25 * (size_t)n);
        memset(O, 0, sizeof(double) * 25 * (size_t)(n > 1 ? n - 1 : 1));
        memset(b, 0, sizeof(double) * 5 * (size_t)n);
        for (int k = 0; k < n; ++k) {
            int i = lo + k;
            node_t *nd = &g->nd[i];
            double *Dk = D + 25 * k;
            if (nd->has_fix && !nd->gated) {
                double r[3], dra[3], J[3][5];
                fix_residual(nd, r);
                gf_drz_vec(nd->x[0], nd->a, dra);
                for (int ax = 0; ax < 3; ++ax) {
                    for (int q = 0; q < 5; ++q) J[ax][q] = 0.0;
                    J[ax][0] = dra[ax]; J[ax][1 + ax] = 1.0;
                    double sga = nd->sg[ax];
                    double e = r[ax] / sga;
                    double w = loss_weight(c, e);
                    double Ja[5];
                    for (int q = 0; q < 5; ++q) Ja[q] = J[ax][q] / sga;
                    acc_row(D, NULL, b, -1, k, e, NULL, Ja, w);
                }
            }
            if (nd->has_vel && !nd->gated && !nd->gnss && i + 1 < g->n && g->nd[i + 1].lk == 0 && !g->nd[i + 1].gnss && !link_is_bad(g, i, i + 1)) {
                const node_t *nx = &g->nd[i + 1];
                double dt = nx->t - nd->t, dL[3], rv[3], dr[3];
                for (int q = 0; q < 3; ++q) dL[q] = (nx->pl[q] - nd->pl[q]) / dt;
                gf_rz_vec(nd->x[0], dL, rv); gf_drz_vec(nd->x[0], dL, dr);
                for (int ax = 0; ax < 3; ++ax) {
                    double s = nd->x[4] * 1.0;
                    double r = (s * rv[ax] - nd->v[ax]) / nd->sv;
                    double Ja[5] = { s * dr[ax] / nd->sv, 0, 0, 0, rv[ax] / nd->sv };
                    acc_row(D, NULL, b, -1, k, r, NULL, Ja, loss_weight(c, r));
                }
            }
            if (c->speed_on && nd->has_spd && !nd->spd_stat && !nd->gnss) {   /* gait speed: s * (horizontal odometry path / duration) = v */
                double dh, T, sg = nd->spd_sg * c->speed_sigma_scale;
                if (sg < 0.02) sg = 0.02;
                if (speed_span(g, i, nd->spd_w, &dh, &T) && T >= 0.5 * nd->spd_w && dh > 1e-9) {
                    double e = (nd->x[4] * dh / T - nd->spd_v) / sg;
                    double Ja[5] = { 0, 0, 0, 0, dh / T / sg };
                    acc_row(D, NULL, b, -1, k, e, NULL, Ja, speed_weight(c, e));
                }
            }
            /* weak scale prior s ~ 1 */
            double spr = sprior;
            if (c->speed_on && c->speed_scale_rw_rel && nd->x[4] > 1.0) spr *= nd->x[4];
            Dk[GF_NS * 4 + 4] += 1.0 / (spr * spr);
            b[GF_NS * k + 4] -= (nd->x[4] - 1.0) / (spr * spr);
        }
        for (int i = (lo > 0 ? lo - 1 : 0); i < hi; ++i) {
            int j = i + 1;
            const node_t *ni = &g->nd[i], *nj = &g->nd[j];
            int ki = i - lo, kj = j - lo;
            double dt = nj->t - ni->t;
            double dL[3], dp[3], dLn;
            for (int q = 0; q < 3; ++q) { dL[q] = nj->pl[q] - ni->pl[q]; dp[q] = nj->x[1 + q] - ni->x[1 + q]; }
            dLn = sqrt(dL[0] * dL[0] + dL[1] * dL[1] + dL[2] * dL[2]);
            int weak = (nj->lk != 0) || ni->gnss || nj->gnss || link_is_bad(g, i, j);
            double qy = c->yaw_rw * sqrt(dt), qs = srw * sqrt(dt);
            if (c->speed_on && c->speed_scale_rw_rel && ni->x[4] > 1.0) qs *= ni->x[4];
            int rw = 1;
            if (nj->lk == 2) rw = 0;
            else if (link_is_bad(g, i, j)) { qy *= c->trust_q_scale; qs = c->metric_scale ? qs * c->trust_q_scale : 1.0; }
            if (!weak) {
                double sp = c->odom_sp + c->odom_kp * dLn;
                if (nj->loose) sp *= c->loose_k;
                double rr[3], dpr[3], Jp[3];
                gf_rzt_vec(ni->x[0], dp, rr);
                gf_drzt_vec(ni->x[0], dp, dpr);
                for (int ax = 0; ax < 3; ++ax) {
                    double r = (rr[ax] - ni->x[4] * dL[ax]) / sp;
                    double Ji[5] = { dpr[ax] / sp, 0, 0, 0, -dL[ax] / sp }, Jj[5] = { 0, 0, 0, 0, 0 };
                    /* Rz^T row ax : d rr[ax]/d p_i = -Rz^T[ax][.] , d/d p_j = +Rz^T[ax][.] */
                    double cs = cos(ni->x[0]), sn = sin(ni->x[0]);
                    double row[3] = { 0, 0, 0 };
                    if (ax == 0) { row[0] = cs; row[1] = sn; } else if (ax == 1) { row[0] = -sn; row[1] = cs; } else row[2] = 1.0;
                    for (int q = 0; q < 3; ++q) { Ji[1 + q] = -row[q] / sp; Jj[1 + q] = row[q] / sp; }
                    (void)Jp;
                    acc_row(D, O, b, ki, kj, r, Ji, Jj, 1.0);
                }
            } else {
                double sl = 1.0 + c->link_speed * dt;
                for (int ax = 0; ax < 3; ++ax) {
                    double r = dp[ax] / sl;
                    double Ji[5] = { 0, 0, 0, 0, 0 }, Jj[5] = { 0, 0, 0, 0, 0 };
                    Ji[1 + ax] = -1.0 / sl; Jj[1 + ax] = 1.0 / sl;
                    acc_row(D, O, b, ki, kj, r, Ji, Jj, 1.0);
                }
            }
            if (c->speed_on && (nj->has_spd || nj->zupt) && nj->lk != 2) {
                if (nj->zupt) {   /* zero velocity: no displacement over this link */
                    double sz = c->zupt_sigma * dt; if (sz < 0.01) sz = 0.01;
                    for (int ax = 0; ax < 3; ++ax) {
                        double Ji[5] = { 0, 0, 0, 0, 0 }, Jj[5] = { 0, 0, 0, 0, 0 };
                        Ji[1 + ax] = -1.0 / sz; Jj[1 + ax] = 1.0 / sz;
                        acc_row(D, O, b, ki, kj, dp[ax] / sz, Ji, Jj, 1.0);
                    }
                } else if (weak && dt <= 3.0 && nj->has_spd && !nj->spd_stat) {   /* the odometry says nothing about this displacement: constrain its horizontal length */
                    double dhp = sqrt(dp[0] * dp[0] + dp[1] * dp[1]);
                    double sg = nj->spd_sg * c->speed_sigma_scale; sg = sqrt(sg * sg + c->speed_link_sigma * c->speed_link_sigma);
                    if (dhp > 0.05) {
                        double e = (dhp / dt - nj->spd_v) / sg;
                        double Ji[5] = { 0, -dp[0] / dhp / dt / sg, -dp[1] / dhp / dt / sg, 0, 0 }, Jj[5] = { 0, dp[0] / dhp / dt / sg, dp[1] / dhp / dt / sg, 0, 0 };
                        acc_row(D, O, b, ki, kj, e, Ji, Jj, speed_weight(c, e));
                    }
                }
            }
            if (rw) {
                double r = (nj->x[0] - ni->x[0]) / qy;
                double Ji[5] = { -1 / qy, 0, 0, 0, 0 }, Jj[5] = { 1 / qy, 0, 0, 0, 0 };
                acc_row(D, O, b, ki, kj, r, Ji, Jj, 1.0);
                r = (nj->x[4] - ni->x[4]) / qs;
                double Js_i[5] = { 0, 0, 0, 0, -1 / qs }, Js_j[5] = { 0, 0, 0, 0, 1 / qs };
                acc_row(D, O, b, ki, kj, r, Js_i, Js_j, 1.0);
            }
        }
        for (int k = 0; k < n; ++k) for (int q = 0; q < GF_NS; ++q) D[25 * k + GF_NS * q + q] += 1e-6;
        gf_block_thomas(n, D, O, b, G, dx);
        double mx = 0.0;
        for (int k = 0; k < n; ++k)
            for (int q = 0; q < GF_NS; ++q) {
                g->nd[lo + k].x[q] += dx[GF_NS * k + q];
                if (q == 4 && c->speed_on && c->speed_scale_lim > 1.0) { double *sv = &g->nd[lo + k].x[4]; if (*sv < 1.0 / c->speed_scale_lim) *sv = 1.0 / c->speed_scale_lim; if (*sv > c->speed_scale_lim) *sv = c->speed_scale_lim; }
                else if (q == 4 && c->scale_max > 0) { double *sv = &g->nd[lo + k].x[4]; if (*sv < c->scale_min) *sv = c->scale_min; if (*sv > c->scale_max) *sv = c->scale_max; }
                if (fabs(dx[GF_NS * k + q]) > mx) mx = fabs(dx[GF_NS * k + q]);
            }
        if (mx < 1e-6) break;
    }
    return 0;
}

/* ------------------------------------------------------------------ alignment (2D Procrustes with scale) */
typedef struct { double psi, s, ml[3], mz[3], rho; int ok; } fit_t;

/* least-squares similarity (yaw, scale, translation) over the masked points; returns the number of points used */
static int fit_masked(const gf_t *g, const int *ids, int m, const unsigned char *mask, fit_t *f)
{
    double ml[3] = { 0, 0, 0 }, mz[3] = { 0, 0, 0 };
    int cnt = 0; double sa2 = 0, sc2 = 0, cross = 0, dot = 0;
    for (int k = 0; k < m; ++k) if (mask[k]) {
        const node_t *nd = &g->nd[ids[k]];
        for (int q = 0; q < 3; ++q) { ml[q] += nd->pl[q] + nd->a[q]; mz[q] += nd->z[q]; }
        ++cnt;
    }
    if (cnt < 3) return cnt;
    for (int q = 0; q < 3; ++q) { ml[q] /= cnt; mz[q] /= cnt; }
    for (int k = 0; k < m; ++k) if (mask[k]) {
        const node_t *nd = &g->nd[ids[k]];
        double ax = nd->pl[0] + nd->a[0] - ml[0], ay = nd->pl[1] + nd->a[1] - ml[1];
        double cx = nd->z[0] - mz[0], cy = nd->z[1] - mz[1];
        cross += ax * cy - ay * cx; dot += ax * cx + ay * cy; sa2 += ax * ax + ay * ay; sc2 += cx * cx + cy * cy;
    }
    f->psi = atan2(cross, dot);
    f->s = sqrt(sc2 / (sa2 > 1e-9 ? sa2 : 1e-9));
    f->ok = 1;
    for (int q = 0; q < 3; ++q) { f->ml[q] = ml[q]; f->mz[q] = mz[q]; }
    return cnt;
}

/* horizontal residual of every listed point under fit f */
static void fit_residuals(const gf_t *g, const int *ids, int m, const fit_t *f, double *res)
{
    double cs = cos(f->psi), sn = sin(f->psi);
    for (int k = 0; k < m; ++k) {
        const node_t *nd = &g->nd[ids[k]];
        double ax = nd->pl[0] + nd->a[0] - f->ml[0], ay = nd->pl[1] + nd->a[1] - f->ml[1];
        double px = f->s * (cs * ax - sn * ay), py = f->s * (sn * ax + cs * ay);
        double ex = px - (nd->z[0] - f->mz[0]), ey = py - (nd->z[1] - f->mz[1]);
        res[k] = sqrt(ex * ex + ey * ey);
    }
}

/* fit over the listed nodes: positions pl + a vs fixes z. do_trim: up to 3 rounds dropping residuals above 3x the median-based sigma (min 1 m).
 * start_m > 0 (needs do_trim): the first round starts from the best of several contiguous sub-sets (most points within start_m metres of their
 * own fit) instead of the all-points fit, so that a minority-or-majority block of common-offset fixes (multipath burst) cannot drag the fit. */
static void procrustes(const gf_t *g, const int *ids, int m, int do_trim, double start_m, fit_t *f)
{
    f->ok = 0; f->psi = 0; f->s = 1; f->rho = 0;
    if (m < 3) return;
    unsigned char *mask = (unsigned char *)malloc((size_t)m);
    unsigned char *cand = (unsigned char *)malloc((size_t)m);
    double *res = (double *)malloc(sizeof(double) * (size_t)m * 2);
    if (!mask || !cand || !res) { free(mask); free(cand); free(res); return; }
    memset(mask, 1, (size_t)m);
    if (do_trim && start_m > 0 && m >= 8) {
        int blk = m / 2 > 6 ? m / 2 : 6, best = -1, bestn = -1;
        for (int st = 0; st + blk <= m; st += (blk / 3 > 1 ? blk / 3 : 1)) {
            memset(cand, 0, (size_t)m); memset(cand + st, 1, (size_t)blk);
            fit_t fc; fc.ok = 0;
            if (fit_masked(g, ids, m, cand, &fc) < 3 || !fc.ok) continue;
            fit_residuals(g, ids, m, &fc, res);
            int n_in = 0; for (int k = 0; k < m; ++k) n_in += res[k] <= start_m;
            if (n_in > bestn) { bestn = n_in; best = st; }
        }
        if (best >= 0 && bestn >= 3) {
            memset(cand, 0, (size_t)m); memset(cand + best, 1, (size_t)blk);
            fit_t fc; fc.ok = 0; fit_masked(g, ids, m, cand, &fc); fit_residuals(g, ids, m, &fc, res);
            for (int k = 0; k < m; ++k) mask[k] = res[k] <= start_m;
        }
    }
    for (int round = 0; round < (do_trim ? 3 : 1); ++round) {
        int cnt = fit_masked(g, ids, m, mask, f);
        if (cnt < 3) break;
        fit_residuals(g, ids, m, f, res);
        double ss = 0; int nn = 0;
        for (int k = 0; k < m; ++k) if (mask[k]) { ss += res[k] * res[k]; ++nn; }
        f->rho = sqrt(ss / (nn > 0 ? nn : 1));
        if (!do_trim || round == 2) break;
        double *tmp = res + m; int cn = 0;
        for (int k = 0; k < m; ++k) if (mask[k]) tmp[cn++] = res[k];
        for (int i = 1; i < cn; ++i) { double v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; --j; } tmp[j + 1] = v; }
        double med = tmp[cn / 2], thr = 3.0 * 1.4826 * med; if (thr < 1.0) thr = 1.0;
        int nm = 0; for (int k = 0; k < m; ++k) { mask[k] = res[k] <= thr; nm += mask[k]; }
        if (nm < 3 || nm == cn) break;
    }
    free(mask); free(cand); free(res);
}

static void apply_fit(gf_t *g, int lo, int hi, const fit_t *f, double s_used)
{
    for (int i = lo; i <= hi; ++i) {
        node_t *n = &g->nd[i];
        double v[3], ra[3], rv[3];
        if (n->gnss) { n->x[0] = f->psi; n->x[4] = s_used; for (int q = 0; q < 3; ++q) n->x[1 + q] = n->has_fix ? n->z[q] : f->mz[q]; continue; }
        for (int q = 0; q < 3; ++q) v[q] = s_used * (n->pl[q] + n->a[q] - f->ml[q]);
        gf_rz_vec(f->psi, v, rv);
        gf_rz_vec(f->psi, n->a, ra);
        n->x[0] = f->psi; n->x[4] = s_used;
        for (int q = 0; q < 3; ++q) n->x[1 + q] = rv[q] + f->mz[q] - ra[q];
    }
}

/* frame [lo,hi]: collect fix nodes (optionally only t <= tmax), return count */
static int collect_fixes(gf_t *g, int lo, int hi, double tmax, int **ids_out)
{
    if (ensure_iws(g, hi - lo + 2)) return -1;
    int m = 0;
    for (int i = lo; i <= hi; ++i)
        if (OFIX(&g->nd[i]) && g->nd[i].t <= tmax) g->iws[m++] = i;
    *ids_out = g->iws;
    return m;
}

/* frame without enough fixes (indoors): scale from the speed measurements (sum v T / sum of the horizontal odometry paths), yaw 0, the first node at the
 * end of the previous frame (or the origin). Needs cfg.speed_on and cfg.speed_align. Returns 1 on success. */
static int align_speed(gf_t *g, int lo, int hi, double tmax)
{
    const gf_config *c = &g->c;
    if (!c->speed_on || !c->speed_align) return 0;
    if (g->sg_n >= 3) return 0;   /* the run has fixes: frames wait for their own fixes (aligning from the speed alone would lock a possibly wrong yaw / position) */
    double sv = 0, sd = 0, path = 0;
    for (int i = lo + 1; i <= hi; ++i) {
        const node_t *n = &g->nd[i];
        if (n->t > tmax || n->gnss) continue;
        if (n->has_spd && !n->spd_stat) {
            double dh, T;
            int i0 = i;
            while (i0 > lo) {
                if (g->nd[i0].lk != 0 || g->nd[i0 - 1].gnss) break;
                if (n->t - g->nd[i0 - 1].t > n->spd_w + 0.5 * c->node_dt + 1e-6) break;
                --i0;
            }
            T = n->t - g->nd[i0].t; dh = n->ch - g->nd[i0].ch;
            if (i0 < i && T >= 0.5 * n->spd_w) { sv += n->spd_v * T; sd += dh; }
        }
    }
    for (int i = lo + 1; i <= hi; ++i) if (g->nd[i].t <= tmax && g->nd[i].lk == 0 && !g->nd[i].gnss) path += g->nd[i].ch - g->nd[i - 1].ch;
    if (!(sd > 1e-9)) return 0;
    if (!(c->speed_align_metric && !c->metric_scale) && path < c->seg_min_extent) return 0;
    double u = sv / sd;
    if (c->speed_align_metric && !c->metric_scale && path * u < c->seg_min_extent) return 0;
    fit_t f; memset(&f, 0, sizeof f); f.ok = 1; f.psi = 0.0; f.s = 1.0;
    for (int q = 0; q < 3; ++q) { f.ml[q] = g->nd[lo].pl[q] + g->nd[lo].a[q]; f.mz[q] = 0.0; }
    if (lo > 0) { double ra[3]; gf_rz_vec(g->nd[lo - 1].x[0], g->nd[lo].a, ra); f.psi = g->nd[lo - 1].x[0]; for (int q = 0; q < 3; ++q) f.mz[q] = g->nd[lo - 1].x[1 + q] + ra[q]; }
    if (!c->metric_scale) {
        if (u < 1e-6) u = 1e-6;
        if (u > 1e6) u = 1e6;
        for (int i = lo; i <= hi; ++i) { for (int q = 0; q < 3; ++q) g->nd[i].pl[q] *= u; g->nd[i].unit *= u; g->nd[i].ch *= u; }
        f.ml[0] *= u; f.ml[1] *= u; f.ml[2] *= u;
        g->cur_unit = g->nd[hi].unit;
        apply_fit(g, lo, hi, &f, 1.0);
    } else {
        double lim = c->speed_scale_lim > 1.0 ? c->speed_scale_lim : 4.0;
        if (u < 1.0 / lim) u = 1.0 / lim;
        if (u > lim) u = lim;
        apply_fit(g, lo, hi, &f, u);
    }
    return 1;
}

/* align the frame nodes lo..hi from their own fixes; returns 1 on success */
static int align_frame(gf_t *g, int lo, int hi, double tmax, int require_extent)
{
    int *ids; int m = collect_fixes(g, lo, hi, tmax, &ids);
    if (m < 3) return align_speed(g, lo, hi, tmax);
    if (require_extent > 0) {   /* the track must span some distance in both the odometry and the fixes */
        double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;   /* extent of the fixes */
        for (int a = 0; a < m; ++a) {
            const node_t *p = &g->nd[ids[a]];
            if (p->z[0] < xmin) xmin = p->z[0];
            if (p->z[0] > xmax) xmax = p->z[0];
            if (p->z[1] < ymin) ymin = p->z[1];
            if (p->z[1] > ymax) ymax = p->z[1];
        }
        double mg = hypot(xmax - xmin, ymax - ymin);
        if (mg < g->c.seg_min_extent) return 0;
    }
    fit_t f;
    if (!g->c.metric_scale) {   /* monocular: first fit the unit of this frame, rescale, then the normal alignment */
        procrustes(g, ids, m, g->c.robust_init, 0.0, &f);
        if (!f.ok) return 0;
        double u = f.s; if (u < 1e-6) u = 1e-6; if (u > 1e6) u = 1e6;
        for (int i = lo; i <= hi; ++i) { for (int q = 0; q < 3; ++q) g->nd[i].pl[q] *= u; g->nd[i].unit *= u; g->nd[i].ch *= u; }
        g->cur_unit = g->nd[hi].unit;
    }
    procrustes(g, ids, m, g->c.robust_init, 0.0, &f);
    if (!f.ok) return 0;
    double s = f.s; if (s < 0.5) s = 0.5; if (s > 1.5) s = 1.5;
    apply_fit(g, lo, hi, &f, s);
    return 1;
}

/* ------------------------------------------------------------------ trust test */
static void trust_eval(gf_t *g, int lo, int hi)
{
    const gf_config *c = &g->c;
    if (!c->trust) return;
    double H2 = 0.5 * c->trust_window_s;
    if (ensure_iws(g, 2 * g->n + 4)) return;
    for (int i = lo; i <= hi; ++i) {
        if (g->nd[i].gnss) { g->nd[i].bad = 0; continue; }
        /* fix nodes within +-H2 of node i, same frame */
        int a = i, b = i;
        while (a > 0 && g->nd[a].lk != 2 && g->nd[i].t - g->nd[a - 1].t <= H2) --a;
        while (b + 1 < g->n && g->nd[b + 1].lk != 2 && g->nd[b + 1].t - g->nd[i].t <= H2) ++b;
        int m = 0;
        for (int k = a; k <= b; ++k) if (UFIX(&g->nd[k])) g->iws[m++] = k;
        if (m < c->trust_min_fixes) { g->nd[i].bad = 0; continue; }
        fit_t f;
        procrustes(g, g->iws, m, 1, c->trust_start_m, &f);
        if (!f.ok) { g->nd[i].bad = 0; continue; }
        /* GNSS noise from second differences of the horizontal fixes (robust, ignores sparse outliers) */
        double d2[128]; int nd2 = 0; double sh = 0;
        for (int k = 1; k + 1 < m && nd2 < 128; ++k) {
            const node_t *p = &g->nd[g->iws[k - 1]], *q = &g->nd[g->iws[k]], *r = &g->nd[g->iws[k + 1]];
            for (int ax = 0; ax < 2 && nd2 < 128; ++ax) d2[nd2++] = fabs(p->z[ax] - 2 * q->z[ax] + r->z[ax]);
            sh += q->sg[0];
        }
        double sig = 1.0;
        if (nd2 >= 4) {
            for (int p = 1; p < nd2; ++p) { double v = d2[p]; int j = p - 1; while (j >= 0 && d2[j] > v) { d2[j + 1] = d2[j]; --j; } d2[j + 1] = v; }
            sig = 1.4826 * d2[nd2 / 2] / sqrt(6.0);
            if (nd2 > 0 && sh / (nd2 / 2 > 0 ? nd2 / 2 : 1) < sig) sig = sh / (nd2 / 2 > 0 ? nd2 / 2 : 1);
        }
        if (sig < 0.5) sig = 0.5;
        /* spread of the fixes in the window */
        double mzx = 0, mzy = 0; for (int k = 0; k < m; ++k) { mzx += g->nd[g->iws[k]].z[0]; mzy += g->nd[g->iws[k]].z[1]; }
        mzx /= m; mzy /= m;
        double sp2 = 0; for (int k = 0; k < m; ++k) { double dx = g->nd[g->iws[k]].z[0] - mzx, dy = g->nd[g->iws[k]].z[1] - mzy; sp2 += dx * dx + dy * dy; }
        double spread = sqrt(sp2 / m);
        int bad = 0;
        if (!c->metric_scale && c->trust_scale_k_mono > 0) {   /* monocular: the scale must be stable; compare the similarity scale of the windows left and right of the node */
            int al = i, br = i;
            while (al > 0 && g->nd[al].lk != 2 && g->nd[i].t - g->nd[al - 1].t <= H2) --al;
            while (br + 1 < g->n && g->nd[br + 1].lk != 2 && g->nd[br + 1].t - g->nd[i].t <= H2) ++br;
            int ml = 0, mr = 0; int *buf = g->iws + m;   /* iws has room: m <= n */
            for (int k = al; k <= i; ++k) if (UFIX(&g->nd[k])) g->iws[m + ml++] = k;
            (void)buf;
            fit_t fl, fr;
            procrustes(g, g->iws + m, ml, 0, 0.0, &fl);
            mr = 0;
            for (int k = i; k <= br; ++k) if (UFIX(&g->nd[k])) g->iws[m + mr++] = k;
            procrustes(g, g->iws + m, mr, 0, 0.0, &fr);
            if (ml >= 4 && mr >= 4 && fl.ok && fr.ok && spread > c->trust_spread_k * sig) {
                double ratio = fl.s / fr.s;
                if (ratio > c->trust_scale_k_mono || ratio < 1.0 / c->trust_scale_k_mono) bad = 1;
            } else if (mr < 4 && i > 0 && g->nd[i].lk != 2) bad = g->nd[i - 1].bad;   /* newest nodes: no right window yet, keep the verdict of the previous node */
        }
        if (c->metric_scale && spread > c->trust_spread_k * sig && (f.s < 1.0 / c->trust_scale_k || f.s > c->trust_scale_k)) bad = 1;
        if (c->trust_state_k > 1.0 && spread > c->trust_spread_k * sig && g->nd[i].x[4] > 1e-3) {
            /* the scale the fixes ask for (similarity fit of the window) is far from the scale state: the odometry collapsed / re-scaled (monocular) */
            double ratio = f.s / g->nd[i].x[4];
            if (ratio > c->trust_state_k || ratio < 1.0 / c->trust_state_k) bad = 1;
        }
        if (f.rho > c->trust_rho_k * sig && f.rho > c->trust_rho_min) bad = 1;
        if (c->trust_long_s > 0 && (c->trust_long_and ? bad : !bad)) {
            /* long window (robust to bursts of bad fixes thanks to trimming): AND-mode confirms a short-window verdict,
             * OR-mode additionally catches slow drift of the similarity that the short window cannot see */
            int a2 = i, b2 = i; double HL = 0.5 * c->trust_long_s;
            while (a2 > 0 && g->nd[a2].lk != 2 && g->nd[i].t - g->nd[a2 - 1].t <= HL) --a2;
            while (b2 + 1 < g->n && g->nd[b2 + 1].lk != 2 && g->nd[b2 + 1].t - g->nd[i].t <= HL) ++b2;
            int ml = 0;
            for (int k = a2; k <= b2; ++k) if (UFIX(&g->nd[k])) g->iws[ml++] = k;
            int long_bad = 0;
            if (ml >= 3 * c->trust_min_fixes) {
                fit_t fl;
                procrustes(g, g->iws, ml, 1, 0.0, &fl);
                if (fl.ok && fl.rho > c->trust_rho_long) long_bad = 1;
                if (fl.ok && c->trust_long_and && c->metric_scale && (fl.s < 1.0 / c->trust_scale_k || fl.s > c->trust_scale_k)) long_bad = 1;
            } else if (c->trust_long_and) long_bad = 1;   /* not enough fixes for a second opinion: keep the short verdict */
            bad = c->trust_long_and ? (bad && long_bad) : (bad || long_bad);
        }
        g->nd[i].bad = (unsigned char)bad;
        g->nd[i].rho = f.rho; g->nd[i].sig = sig;
    }
}

/* ------------------------------------------------------------------ causal machinery */
static int window_nodes(const gf_t *g)
{
    int W = (int)(g->c.window_s / g->c.node_dt + 0.5);
    if (W < 3) W = 3;
    if (growing(g)) W = g->n + 1;   /* young trajectory: the window grows with it */
    return W;
}

static int any_fix(const gf_t *g, int lo, int hi)
{
    for (int i = lo; i <= hi; ++i) if (g->nd[i].has_fix || (g->c.speed_on && g->nd[i].has_spd)) return 1;
    return 0;
}

static void propagate_node(gf_t *g, int k)
{
    node_t *n = &g->nd[k]; const node_t *p = &g->nd[k - 1];
    n->x[0] = p->x[0]; n->x[4] = p->x[4];
    if (n->lk == 2 || p->gnss) { for (int q = 0; q < 3; ++q) n->x[1 + q] = p->x[1 + q]; return; }
    double d[3], r[3];
    for (int q = 0; q < 3; ++q) d[q] = p->x[4] * (n->pl[q] - p->pl[q]);
    gf_rz_vec(p->x[0], d, r);
    for (int q = 0; q < 3; ++q) n->x[1 + q] = p->x[1 + q] + r[q];
}

static void try_align_pending(gf_t *g)
{
    if (g->pend_frame < 0) return;
    int fs = idx_of_id(g, g->pend_frame);
    if (fs < 0) { g->pend_frame = -1; return; }
    if (align_frame(g, fs, g->n - 1, 1e300, 1)) {
        for (int i = fs; i <= g->n - 1; ++i) g->nd[i].prov = 0;
        g->pend_frame = -1;
        int W = window_nodes(g);
        int lo = g->n - 1 - W; if (lo > fs - 1) lo = fs - 1; if (lo < 0) lo = 0;
        solve_window(g, lo, g->n - 1, g->c.init_iters);
        for (int i = fs; i < g->n; ++i) memcpy(g->nd[i].xo, g->nd[i].x, sizeof g->nd[i].x);
    }
}

/* advance node k (causal): propagate, evaluate trust, solve the window */
static void advance_node(gf_t *g, int k)
{
    update_grow(g);
    propagate_node(g, k);
    int W = window_nodes(g);
    if (g->nd[k].lk == 2) {
        g->nd[k].prov = 1;
        if (g->pend_frame < 0) g->pend_frame = g->nd[k].id;
    } else if (g->nd[k].lk != 2 && k > 0) g->nd[k].prov = g->nd[k - 1].prov;
    int lo = k - W; if (lo < 0) lo = 0;
    if (g->c.trust) trust_eval(g, lo, k);
    if (any_fix(g, lo, k)) solve_window(g, lo, k, g->c.causal_iters);
    memcpy(g->nd[k].xo, g->nd[k].x, sizeof g->nd[k].x);
    if (g->pend_frame >= 0) try_align_pending(g);
}

static int causal_try_init(gf_t *g)
{
    if (g->ready || g->n < 2) return 0;
    const gf_config *c = &g->c;
    double t0 = g->nd[0].t;
    if (g->nd[g->n - 1].t - t0 <= c->init_wait_s) return 0;
    int fe = g->n - 1;                          /* first frame only */
    for (int i = 1; i < g->n; ++i) if (g->nd[i].lk == 2) { fe = i - 1; break; }
    double tmax = t0 + c->init_wait_s;
    if (!align_frame(g, 0, fe, tmax, 0)) {
        if (!align_frame(g, 0, fe, 1e300, 0)) return 0;   /* not enough fixes yet: retry with everything seen so far */
    }
    for (int i = 0; i < g->n; ++i) { g->nd[i].prov = 0; }
    g->ready = 1;
    int S = c->settle_nodes; if (S > fe) S = fe;
    if (c->trust) trust_eval(g, 0, S);
    solve_window(g, 0, S, c->init_iters);
    for (int i = 0; i <= S; ++i) memcpy(g->nd[i].xo, g->nd[i].x, sizeof g->nd[i].x);
    for (int k = S + 1; k < g->n; ++k) advance_node(g, k);
    return 1;
}

/* ------------------------------------------------------------------ fixes */
static int node_nearest(const gf_t *g, double t)
{
    int lo = 0, hi = g->n - 1;
    if (g->n == 0) return -1;
    while (lo < hi) { int mid = (lo + hi) / 2; if (g->nd[mid].t < t) lo = mid + 1; else hi = mid; }
    if (lo > 0 && fabs(g->nd[lo - 1].t - t) <= fabs(g->nd[lo].t - t)) --lo;
    return lo;
}

/* returns 1 if the fix was attached to a node, 0 if it must wait, -1 if dropped */
static int assign_fix(gf_t *g, const gf_fix *f)
{
    if (g->n == 0) return 0;
    double tl = g->nd[g->n - 1].t, half = 0.5 * g->c.node_dt;
    if (f->t > tl + half) return 0;
    int k = node_nearest(g, f->t);
    node_t *nd = &g->nd[k];
    double d = fabs(nd->t - f->t);
    if (d > half + 1e-6) return -1;
    if (nd->has_fix && nd->fdt <= d) return -1;
    double ms = g->c.min_sigma;
    double sh = f->sigma_h > ms ? f->sigma_h : ms, sv = f->sigma_v > ms ? f->sigma_v : ms;
    nd->has_fix = 1; nd->gated = 0; nd->fdt = d;
    for (int q = 0; q < 3; ++q) nd->z[q] = f->p[q];
    nd->sg[0] = sh; nd->sg[1] = sh; nd->sg[2] = sv;
    nd->has_vel = (unsigned char)(f->has_vel && f->sigma_vel > 0);
    if (nd->has_vel) { for (int q = 0; q < 3; ++q) nd->v[q] = f->v[q]; nd->sv = f->sigma_vel; }
    g->st.n_fix_assigned++;
    for (int q = 0; q < 2; ++q) { if (f->p[q] < g->zmin[q]) g->zmin[q] = f->p[q]; if (f->p[q] > g->zmax[q]) g->zmax[q] = f->p[q]; }
    g->sg_sum += sh; g->sg_n++;
    return k + 1;   /* >0 : assigned (node index + 1) */
}

/* returns node index + 1 if the measurement was attached, 0 if it must wait, -1 if dropped */
static int assign_speed(gf_t *g, double t, double v, double sg, double w, unsigned flags)
{
    if (g->n == 0) return 0;
    double tl = g->nd[g->n - 1].t, half = 0.5 * g->c.node_dt;
    if (t > tl + half) return 0;
    int k = node_nearest(g, t);
    node_t *nd = &g->nd[k];
    double d = fabs(nd->t - t);
    if (d > half + 1e-6) return -1;
    if (nd->has_spd && nd->spd_fdt <= d) return -1;
    nd->has_spd = 1; nd->spd_fdt = d; nd->spd_v = v; nd->spd_sg = sg; nd->spd_w = w; nd->spd_stat = (flags & GF_SPEED_STATIONARY) ? 1 : 0;
    g->st.n_speed_assigned++;
    if (nd->spd_stat) for (int i = k; i >= 0 && g->nd[i].t > t - w; --i) { if (g->nd[i].lk == 2) break; g->nd[i].zupt = 1; }
    return k + 1;
}

/* odometry has been silent for more than gap_s: a fix makes a node of its own (GNSS-only, no odometry link) so the output continues through tracking loss */
static int add_gnss_node(gf_t *g, const gf_fix *f)
{
    if (ensure_nodes(g)) return -1;
    const node_t *pv = &g->nd[g->n - 1];
    node_t *nd = &g->nd[g->n];
    memset(nd, 0, sizeof *nd);
    nd->t = f->t; nd->id = g->next_id++; nd->lk = 1; nd->gnss = 1;
    memcpy(nd->ra, pv->ra, sizeof nd->ra); nd->unit = pv->unit; nd->cum = pv->cum; nd->ch = pv->ch; nd->prov = pv->prov;
    memcpy(nd->x, pv->x, sizeof nd->x); memcpy(nd->xo, pv->x, sizeof nd->xo);
    memcpy(nd->Rr, pv->gnss ? pv->Rr : g->lR, sizeof nd->Rr);
    for (int q = 0; q < 3; ++q) nd->x[1 + q] = f->p[q];
    memcpy(nd->xo, nd->x, sizeof nd->xo);
    g->n++; g->st.n_nodes++; g->st.n_gnss_nodes++;
    return assign_fix(g, f);
}

static void resolve_after_fix(gf_t *g, int k)
{
    update_grow(g);
    if (!g->causal || !g->ready) return;
    int W = window_nodes(g);
    int head = g->n - 1;
    if (k < head - W - 1) return;           /* frozen: too old to matter */
    int lo = head - W; if (lo < 0) lo = 0;
    if (g->c.trust) trust_eval(g, lo, head);
    solve_window(g, lo, head, g->c.causal_iters);
    memcpy(g->nd[head].xo, g->nd[head].x, sizeof g->nd[head].x);
    if (g->pend_frame >= 0) try_align_pending(g);
}

static void process_pending(gf_t *g)
{
    int w = 0, last = -1;
    for (int i = 0; i < g->npend; ++i) {
        const gf_fix *pf = &g->pend[i];
        int r;
        if (g->c.gnss_only_nodes && g->have_last && g->n > 0 && pf->t > g->lt + g->c.gap_s && pf->t > g->nd[g->n - 1].t + 0.5 * g->c.node_dt) r = add_gnss_node(g, pf);
        else r = assign_fix(g, pf);
        if (r == 0) g->pend[w++] = g->pend[i];
        else if (r > 0) { if (r - 1 > last) last = r - 1; }
    }
    g->npend = w;
    w = 0;
    for (int i = 0; i < g->nspd; ++i) {
        int r = assign_speed(g, g->pspd[i].t, g->pspd[i].v, g->pspd[i].sg, g->pspd[i].w, g->pspd[i].flags);
        if (r == 0) g->pspd[w++] = g->pspd[i];
        else if (r > 0 && g->c.speed_on) { if (r - 1 > last) last = r - 1; }
    }
    g->nspd = w;
    if (last >= 0) {
        if (g->causal && !g->ready) causal_try_init(g);
        else resolve_after_fix(g, last);
    }
}

int gf_add_fix(gf_t *g, const gf_fix *f)
{
    if (!g || !f) return GF_ERR_ARG;
    if (!isfinite(f->t) || !isfinite(f->p[0]) || !isfinite(f->p[1]) || !isfinite(f->p[2])) return GF_ERR_ARG;
    g->st.n_fix++;
    if (g->npend >= PEND_MAX) { memmove(g->pend, g->pend + 1, sizeof(gf_fix) * (PEND_MAX - 1)); g->npend--; }
    g->pend[g->npend++] = *f;
    process_pending(g);
    return 0;
}

int gf_add_speed(gf_t *g, double t, double speed, double sigma, double window_s, unsigned flags)
{
    if (!g) return GF_ERR_ARG;
    if (!isfinite(t) || !isfinite(speed) || !isfinite(sigma) || !isfinite(window_s) || !(window_s > 0) || !(sigma > 0)) return GF_ERR_ARG;
    if (!g->c.speed_on) return 0;
    if (g->nspd >= PEND_MAX) { memmove(g->pspd, g->pspd + 1, sizeof g->pspd[0] * (PEND_MAX - 1)); g->nspd--; }
    g->pspd[g->nspd].t = t; g->pspd[g->nspd].v = speed; g->pspd[g->nspd].sg = sigma; g->pspd[g->nspd].w = window_s; g->pspd[g->nspd].flags = flags; g->nspd++;
    process_pending(g);
    return 0;
}

/* ------------------------------------------------------------------ odometry */
int gf_set_gravity(gf_t *g, const double up[3])
{
    if (!g || !up) return GF_ERR_ARG;
    double n = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    if (!(n > 1e-9) || !isfinite(n)) return GF_ERR_ARG;
    for (int q = 0; q < 3; ++q) g->up_next[q] = up[q];
    g->up_pending = 1; g->have_up = 1;
    return 0;
}

int gf_add_odom(gf_t *g, double t, const double p[3], const double q[4], unsigned flags)
{
    if (!g || !p || !q) return GF_ERR_ARG;
    if (!isfinite(t) || !isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]) || !isfinite(q[0]) || !isfinite(q[1]) || !isfinite(q[2]) || !isfinite(q[3])) return GF_ERR_ARG;
    if (g->have_last && t <= g->lt) return GF_ERR_ORDER;
    g->st.n_odom++;
    int first = !g->have_last;
    int newframe = (flags & GF_ODOM_NEW_FRAME) != 0;
    int gap = (flags & GF_ODOM_GAP) != 0;
    if (flags & GF_ODOM_LOOSE) g->loose_pend = 1;
    if (!first) {
        if (t - g->lt > g->c.gap_s) gap = 1;
        if (g->c.max_speed > 0 && !newframe) {
            double d = hypot(hypot(p[0] - g->lraw[0], p[1] - g->lraw[1]), p[2] - g->lraw[2]) * g->cur_unit;
            if (d / (t - g->lt) > g->c.max_speed) newframe = 1;
        }
    }
    if (first || newframe) {
        if (!g->c.gravity_aligned) {
            if (!g->up_pending) { if (!g->have_up || first) return GF_ERR_GRAVITY; if (newframe) return GF_ERR_GRAVITY; }
            gf_align_up(g->up_next, g->cur_ra); g->up_pending = 0;
        } else {
            for (int i = 0; i < 9; ++i) g->cur_ra[i] = (i % 4 == 0) ? 1.0 : 0.0;
        }
        g->cur_unit = 1.0;
    }
    /* aligned rotation and antenna lever arm */
    double R[9], Rr[9], a[3], pr[3];
    gf_quat_to_mat(q, R); gf_mat3_mul(g->cur_ra, R, Rr);
    gf_mat3_vec(Rr, g->c.rsa, a);
    gf_mat3_vec(g->cur_ra, p, pr);
    g->have_last = 1; g->lt = t; memcpy(g->lraw, p, sizeof g->lraw); memcpy(g->lR, Rr, sizeof Rr);

    int make = 0;
    if (!g->have_grid) { g->have_grid = 1; g->t0 = t; g->m_next = 1; make = 1; }
    else if (t >= g->t0 + (double)g->m_next * g->c.node_dt) make = 1;
    int lk = first ? 0 : (newframe ? 2 : (gap ? 1 : 0));
    if (lk != 0) make = 1;
    if (!make) {
        process_pending(g);
        return 0;
    }
    int rc = ensure_nodes(g);
    if (rc) return rc;
    while (g->t0 + (double)g->m_next * g->c.node_dt <= t) g->m_next++;
    node_t *nd = &g->nd[g->n];
    memset(nd, 0, sizeof *nd);
    nd->t = t; nd->id = g->next_id++; nd->lk = lk; nd->loose = (unsigned char)g->loose_pend; g->loose_pend = 0;
    for (int i = 0; i < 3; ++i) { nd->pl[i] = g->cur_unit * pr[i]; nd->a[i] = a[i]; }
    memcpy(nd->ra, g->cur_ra, sizeof nd->ra); nd->unit = g->cur_unit;
    nd->x[4] = 1.0;
    if (g->n > 0 && lk == 0) {
        const node_t *pv = &g->nd[g->n - 1];
        double d = hypot(hypot(nd->pl[0] - pv->pl[0], nd->pl[1] - pv->pl[1]), nd->pl[2] - pv->pl[2]);
        nd->cum = pv->cum + d * (g->ready ? pv->x[4] : 1.0);
        nd->ch = pv->ch + hypot(nd->pl[0] - pv->pl[0], nd->pl[1] - pv->pl[1]);
    } else if (g->n > 0) { nd->cum = g->nd[g->n - 1].cum; nd->ch = g->nd[g->n - 1].ch; }
    g->n++; g->st.n_nodes++;
    if (lk == 2) g->st.n_segments++; else if (first) g->st.n_segments = 1;
    int k = g->n - 1;
    if (g->causal) {
        if (g->ready) advance_node(g, k);
        else if (!g->nd[k].has_fix) { /* wait for the initial alignment */ }
    }
    process_pending(g);
    if (g->causal && !g->ready) causal_try_init(g);
    trim_nodes(g);
    return 0;
}

/* ------------------------------------------------------------------ batch */
int gf_solve_batch(gf_t *g)
{
    if (!g || g->n < 2) return GF_ERR_ARG;
    process_pending(g);
    /* frames */
    int start = 0;
    int first_done = 0;
    for (int i = 1; i <= g->n; ++i) {
        if (i == g->n || g->nd[i].lk == 2) {
            int ok = align_frame(g, start, i - 1, 1e300, first_done ? 1 : 0);
            if (!ok) {
                if (!first_done) return -2;
                /* inherit from the previous frame's end */
                for (int k = start; k < i; ++k) { propagate_node(g, k); g->nd[k].prov = 1; }
            } else first_done = 1;
            start = i;
        }
    }
    if (g->c.trust) trust_eval(g, 0, g->n - 1);
    solve_window(g, 0, g->n - 1, g->c.batch_iters);
    for (int i = 0; i < g->n; ++i) memcpy(g->nd[i].xo, g->nd[i].x, sizeof g->nd[i].x);
    g->ready = 1;
    return 0;
}

/* ------------------------------------------------------------------ output */
static int find_node(const gf_t *g, double t)
{
    int lo = 0, hi = g->n - 1;
    if (g->n == 0 || t < g->nd[0].t - 1e-9) return -1;
    while (lo < hi) { int mid = (lo + hi + 1) / 2; if (g->nd[mid].t <= t) lo = mid; else hi = mid - 1; }
    return lo;
}

static int pose_from(const gf_t *g, int k, double t, const double p_raw[3], const double Rr[9], gf_pose *o)
{
    const node_t *nd = &g->nd[k];
    double pl[3], d[3], r[3];
    double pa[3];
    gf_mat3_vec(nd->ra, p_raw, pa);
    for (int q = 0; q < 3; ++q) pl[q] = nd->unit * pa[q];
    for (int q = 0; q < 3; ++q) d[q] = nd->xo[4] * (pl[q] - nd->pl[q]);
    gf_rz_vec(nd->xo[0], d, r);
    int bad = nd->bad || (k + 1 < g->n && g->nd[k + 1].lk == 0 && g->nd[k + 1].bad);
    for (int q = 0; q < 3; ++q) o->p[q] = nd->xo[1 + q] + r[q];
    if (bad) {   /* odometry distrusted here: bridge between the node estimates, which follow the fixes */
        if (k + 1 < g->n && g->nd[k + 1].lk != 2) {
            const node_t *nx = &g->nd[k + 1];
            double w = (t - nd->t) / (nx->t - nd->t); if (w < 0) w = 0; if (w > 1) w = 1;
            for (int q = 0; q < 3; ++q) o->p[q] = (1 - w) * nd->xo[1 + q] + w * nx->xo[1 + q];
        } else if (k > 0 && g->nd[k].lk != 2) {
            const node_t *pv = &g->nd[k - 1];
            double dt = t - nd->t, h = nd->t - pv->t; if (dt > 2.0) dt = 2.0;
            for (int q = 0; q < 3; ++q) o->p[q] = nd->xo[1 + q] + (nd->xo[1 + q] - pv->xo[1 + q]) / h * dt;
        } else for (int q = 0; q < 3; ++q) o->p[q] = nd->xo[1 + q];
    }
    double Rg[9], Rz[9], c = cos(nd->xo[0]), s = sin(nd->xo[0]);
    Rz[0] = c; Rz[1] = -s; Rz[2] = 0; Rz[3] = s; Rz[4] = c; Rz[5] = 0; Rz[6] = 0; Rz[7] = 0; Rz[8] = 1;
    gf_mat3_mul(Rz, Rr, Rg);
    gf_mat_to_quat(Rg, o->q);
    o->t = t; o->yaw = nd->xo[0];
    o->scale = nd->xo[4] * nd->unit;   /* odometry unit -> metres */
    o->status = GF_ST_INIT | (nd->prov ? GF_ST_SEG_PROVISIONAL : 0u) | (bad ? GF_ST_ODOM_DISTRUSTED : 0u);
    return 0;
}

int gf_query(const gf_t *g, double t, const double p[3], const double q[4], gf_pose *out)
{
    if (!g || !g->ready || g->n == 0) return -1;
    int k = find_node(g, t);
    if (k < 0) return -1;
    double R[9], Rr[9];
    gf_quat_to_mat(q, R); gf_mat3_mul(g->nd[k].ra, R, Rr);
    pose_from(g, k, t, p, Rr, out);
    out->sigma_h = 0; out->dist_since_fix = 0;
    return 0;
}

/* pose of a GNSS-only node (no odometry): position = fused estimate, orientation = the last odometry orientation, held, in the current yaw */
static void gnss_node_pose(const gf_t *g, int i, gf_pose *o)
{
    const node_t *nd = &g->nd[i];
    double Rz[9], Rg[9], c = cos(nd->xo[0]), s = sin(nd->xo[0]);
    Rz[0] = c; Rz[1] = -s; Rz[2] = 0; Rz[3] = s; Rz[4] = c; Rz[5] = 0; Rz[6] = 0; Rz[7] = 0; Rz[8] = 1;
    gf_mat3_mul(Rz, nd->Rr, Rg);
    gf_mat_to_quat(Rg, o->q);
    for (int q = 0; q < 3; ++q) o->p[q] = nd->xo[1 + q];
    o->t = nd->t; o->yaw = nd->xo[0]; o->scale = nd->xo[4] * nd->unit;
    o->status = GF_ST_INIT | GF_ST_NO_ODOM | (nd->prov ? GF_ST_SEG_PROVISIONAL : 0u);
}

int gf_node_pose(const gf_t *g, int i, gf_pose *out)
{
    if (!g || !out || i < 0 || i >= g->n) return -1;
    if (!g->nd[i].gnss) return 1;
    memset(out, 0, sizeof *out);
    gnss_node_pose(g, i, out);
    out->sigma_h = g->nd[i].sg[0];
    return 0;
}

int gf_get_pose(const gf_t *g, gf_pose *o)
{
    if (!g || !g->have_last) return -1;
    memset(o, 0, sizeof *o);
    if (g->ready && g->n > 0 && g->nd[g->n - 1].gnss) {   /* odometry is lost: the newest estimate is a GNSS-only node */
        gnss_node_pose(g, g->n - 1, o);
        o->sigma_h = g->nd[g->n - 1].sg[0];
        o->t = g->nd[g->n - 1].t;
        return 0;
    }
    if (!g->ready || g->n == 0) {   /* not aligned yet: odometry passthrough (gravity-aligned frame) */
        double pa[3]; gf_mat3_vec(g->cur_ra, g->lraw, pa);
        for (int q = 0; q < 3; ++q) o->p[q] = pa[q] * g->cur_unit;
        gf_mat_to_quat(g->lR, o->q);
        o->t = g->lt; o->scale = g->cur_unit; o->sigma_h = 1e3;
        return 0;
    }
    int k = g->n - 1;
    pose_from(g, k, g->lt, g->lraw, g->lR, o);
    /* heuristic uncertainty and blackout state */
    int back = 0; const node_t *lf_ = NULL;
    for (int i = k; i >= 0 && back < 4000; --i, ++back) if (g->nd[i].has_fix && !g->nd[i].gated) { lf_ = &g->nd[i]; break; }
    double dist = g->nd[k].cum - (lf_ ? lf_->cum : 0.0);
    double sf = lf_ ? lf_->sg[0] : 1e3;
    double nfx = 0; int w = window_nodes(g); for (int i = k; i >= 0 && i > k - w; --i) if (g->nd[i].has_fix && !g->nd[i].gated) nfx += 1.0;
    if (nfx < 1.0) nfx = 1.0;
    double drift = g->c.drift_rate * dist;
    o->sigma_h = sqrt(sf * sf / nfx + drift * drift);
    o->dist_since_fix = dist;
    if (!lf_ || g->lt - lf_->t > g->c.blackout_s) o->status |= GF_ST_NO_RECENT_FIX;
    return 0;
}

void gf_get_stats(const gf_t *g, gf_stats *s)
{
    *s = g->st;
    long nb = 0, ng = 0;
    for (int i = 0; i < g->n; ++i) { nb += g->nd[i].bad; ng += g->nd[i].has_fix && g->nd[i].gated; }
    s->n_distrusted_nodes = nb; s->n_fix_gated = ng;
}

int gf_node(const gf_t *g, int i, double *t, double x[5], int *distrusted, int *fix_used)
{
    if (!g || i < 0 || i >= g->n) return -1;
    const node_t *n = &g->nd[i];
    if (t) *t = n->t;
    if (x) for (int q = 0; q < 5; ++q) x[q] = n->xo[q];
    if (distrusted) *distrusted = n->bad;
    if (fix_used) *fix_used = n->has_fix && !n->gated;
    return 0;
}

int gf_node_diag(const gf_t *g, int i, double *rho, double *sig)
{
    if (!g || i < 0 || i >= g->n) return -1;
    if (rho) *rho = g->nd[i].rho;
    if (sig) *sig = g->nd[i].sig;
    return 0;
}
