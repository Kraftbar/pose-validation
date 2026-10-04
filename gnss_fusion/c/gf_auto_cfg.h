/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code.
 * gf_auto_cfg.h : key=value parsing of gf_auto_config for the drivers (gf_auto_run.c, phone_pipeline/c/pp_live.c). Static functions, header only.
 *   plain gf_run keys apply to BOTH smoothers; a.KEY only the smoother with fixes, b.KEY only the fix-free stream smoother, g.KEY the georef, switch keys:
 *   stream policy tau_fast tau_slow blend down rho_on rho_off distr_on distr_off sig_min sig_floor fail_k dwell min_span min_pairs metric_only (see gf_auto.h) */
#ifndef GF_AUTO_CFG_H
#define GF_AUTO_CFG_H
#include <stdlib.h>
#include <string.h>
#include "gf_auto.h"

static int cfg_kv(gf_config *c, const char *kv)
{
    char key[64]; const char *eq = strchr(kv, '='); if (!eq || eq - kv > 60) return -1;
    memcpy(key, kv, (size_t)(eq - kv)); key[eq - kv] = 0;
    const char *v = eq + 1; double d = atof(v);
#define D(name, field) if (!strcmp(key, name)) { c->field = d; return 0; }
    D("node_dt", node_dt) D("window_s", window_s) D("batch_iters", batch_iters) D("causal_iters", causal_iters) D("init_iters", init_iters)
    D("settle_nodes", settle_nodes) D("init_wait_s", init_wait_s) D("odom_sp", odom_sp) D("odom_kp", odom_kp) D("scale_rw", scale_rw)
    D("scale_prior", scale_prior) D("scale_rw_mono", scale_rw_mono) D("scale_prior_mono", scale_prior_mono) D("metric", metric_scale) D("gravity_aligned", gravity_aligned) D("loss", loss) D("loss_k", loss_k)
    D("min_sigma", min_sigma) D("gate_chi2", gate_chi2) D("gate_floor", gate_floor) D("gate_min_fixes", gate_min_fixes) D("robust_init", robust_init) D("gap_s", gap_s) D("max_speed", max_speed)
    D("link_speed", link_speed) D("seg_min_extent", seg_min_extent) D("trust", trust) D("trust_window_s", trust_window_s)
    D("trust_min_fixes", trust_min_fixes) D("trust_long_s", trust_long_s) D("trust_long_and", trust_long_and) D("trust_rho_long", trust_rho_long) D("trust_scale_k", trust_scale_k) D("trust_scale_k_mono", trust_scale_k_mono) D("trust_rho_k", trust_rho_k) D("trust_rho_min", trust_rho_min)
    D("trust_spread_k", trust_spread_k) D("trust_q_scale", trust_q_scale) D("drift_rate", drift_rate) D("blackout_s", blackout_s)
    D("speed", speed_on) D("speed_k", speed_k) D("speed_sigma_scale", speed_sigma_scale) D("speed_link_sigma", speed_link_sigma) D("zupt_sigma", zupt_sigma) D("speed_align", speed_align) D("speed_scale_rw_rel", speed_scale_rw_rel) D("speed_scale_lim", speed_scale_lim) D("speed_scale_rw", speed_scale_rw) D("loose_k", loose_k) D("speed_align_metric", speed_align_metric)
    D("keep_history", keep_history) D("trust_start_m", trust_start_m) D("gnss_only_nodes", gnss_only_nodes) D("grow_s", grow_s) D("grow_s_mono", grow_s_mono) D("grow_ratio", grow_ratio) D("trust_state_k", trust_state_k) D("scale_min", scale_min) D("scale_max", scale_max)
#undef D
    if (!strcmp(key, "preset")) {
        if (!strcmp(v, "robust")) { gf_config_robust(c); return 0; }
        if (!strcmp(v, "robust1")) {   /* the first version of the robust preset (section 10 of the study), without the section-11 features */
            gf_config_robust(c);
            c->trust_start_m = 0.0; c->trust_state_k = 0.0; c->grow_s = 0.0; c->scale_min = 0.0; c->scale_max = 0.0; c->gnss_only_nodes = 0;
            return 0;
        }
        return -1;
    }
    if (!strcmp(key, "yaw_rw_deg")) { c->yaw_rw = d * 3.14159265358979323846 / 180.0; return 0; }
    if (!strcmp(key, "rsa")) { sscanf(v, "%lf,%lf,%lf", &c->rsa[0], &c->rsa[1], &c->rsa[2]); return 0; }
    return -1;
}


static int geo_kv(gf_georef_config *c, const char *key, double d)
{
    if (!strcmp(key, "min_fixes")) c->min_fixes = (int)d; else if (!strcmp(key, "min_extent")) c->min_extent_m = d;
    else if (!strcmp(key, "scale_sigma")) c->scale_sigma = d; else if (!strcmp(key, "corr_s")) c->corr_s = d;
    else if (!strcmp(key, "forget_s")) c->forget_s = d; else if (!strcmp(key, "huber_k")) c->huber_k = d;
    else if (!strcmp(key, "sigma_floor")) c->sigma_floor = d; else if (!strcmp(key, "use_sigma")) c->use_sigma = (int)d;
    else if (!strcmp(key, "max_pairs")) c->max_pairs = (int)d; else if (!strcmp(key, "max_gap_s")) c->max_gap_s = d;
    else return -1;
    return 0;
}

static int sw_kv(gf_auto_config *c, const char *key, double d)
{
    if (!strcmp(key, "stream")) c->stream_mode = (int)d; else if (!strcmp(key, "policy")) c->policy = (int)d;
    else if (!strcmp(key, "tau_fast")) c->tau_fast_s = d; else if (!strcmp(key, "tau_slow")) c->tau_slow_s = d; else if (!strcmp(key, "blend")) c->blend_s = d;
    else if (!strcmp(key, "down")) c->down_s = d; else if (!strcmp(key, "rho_on")) c->rho_on = d; else if (!strcmp(key, "rho_off")) c->rho_off = d;
    else if (!strcmp(key, "distr_on")) c->distr_on = d; else if (!strcmp(key, "distr_off")) c->distr_off = d; else if (!strcmp(key, "sig_min")) c->sig_min = d;
    else if (!strcmp(key, "sig_floor")) c->sig_floor = d; else if (!strcmp(key, "fail_k")) c->fail_k = d; else if (!strcmp(key, "dwell")) c->dwell_s = d;
    else if (!strcmp(key, "min_span")) c->min_span_s = d; else if (!strcmp(key, "min_pairs")) c->min_pairs = (int)d; else if (!strcmp(key, "metric_only")) c->metric_only = (int)d;
    else return -1;
    return 0;
}

/* one "key=value"; returns 0 on success */
static int gf_auto_set(gf_auto_config *cfg, const char *kv)
{
    const char *eq = strchr(kv, '='); char key[64];
    if (!eq || eq - kv > 60) return -1;
    memcpy(key, kv, (size_t)(eq - kv)); key[eq - kv] = 0;
    double d = atof(eq + 1);
    if (!strncmp(key, "g.", 2)) return geo_kv(&cfg->geo, key + 2, d);
    if (!strncmp(key, "a.", 2)) return cfg_kv(&cfg->sm, kv + 2);
    if (!strncmp(key, "b.", 2)) return cfg_kv(&cfg->st, kv + 2);
    if (!sw_kv(cfg, key, d)) return 0;
    int bad = cfg_kv(&cfg->sm, kv);
    if (!bad) bad = cfg_kv(&cfg->st, kv);
    if (!bad && !strcmp(key, "preset") && !strcmp(eq + 1, "robust")) cfg->st.init_wait_s = 12.0;
    return bad;
}
#endif
