/* SV_PORT_SOURCES: check_sv_g2o_ba.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_ba.c sv_bundle_adjuster.c sv_umap_order.c sv_eigen_amd.c sv_eigen_llt.c sv_linalg.c
 * SPDX-License-Identifier: MIT
 *
 * Harness (module 4b part 2): replays every real bundle-adjustment call
 * captured by stella_port/reference_tools/dump_stella_g2o_ba.cc (patch
 * 0007-ba-trace.patch inside the real library; fr1_xyz / fr1_desk run
 * through the synchronous single-threaded reference driver):
 *   - every local_bundle_adjuster_g2o::optimize() call,
 *   - the initial-map global_bundle_adjuster::optimize_for_initialization(),
 *   - two real global_bundle_adjuster::optimize() (loop-BA entry point)
 *     calls on the final map (huber on / off).
 * For each call the port (sv_bundle_adjuster.c on top of sv_g2o_ba.c) is fed
 * the recorded map view (keyframe/landmark data the upstream function reads)
 * and must reproduce, bit for bit (LOCAL calls, tolerance 0):
 *   - the g2o graph it builds: vertex ids / kinds / fixed flags / owners /
 *     initial estimates, and edge insertion order / measurements / information
 *     / Huber delta (this is where the libstdc++ unordered_map iteration
 *     order of the local keyframes/landmarks matters),
 *   - for every optimize() stage: iterations run, flag state, terminate_action
 *     stop, edge levels at stage start, and per iteration robust chi2, lambda,
 *     Levenberg trial count and stop flag,
 *   - the final vertex estimates, per-edge chi2 / error / depth-positive /
 *     level, the outlier observation list, the Mat44 poses handed to
 *     set_pose_cw() and (global) the optimized landmark set.
 * GLOBAL calls use the same SimplicialLLT path instead of upstream's
 * LinearSolverCSparse; structure (graph, iteration counts, stop decisions,
 * trial counts, edge levels, outliers, optimized-landmark set) must always
 * match exactly, numeric values (chi2, lambda, vertex estimates, edge
 * errors, poses) may deviate by at most SV_BA_GLOBAL_TOL (default 1e-9)
 * measured as |ref-port| / max(1, |ref|); SV_BA_GLOBAL_TOL=0 demands bit
 * equality. The maximum deviations seen are printed on stderr.
 *
 * Reads runs/stella_port/reference_g2o_ba/<seq>/ba_calls.bin (derived from
 * <dump_dir> like check_sv_g2o_pose.c derives reference_g2o); skips cleanly
 * (prints "<seq>: 0/0") when the dump does not exist. SV_BA_VERBOSE=1 prints
 * the first differing field per call. Usage: <seq_label> <fixtures_dir>
 * <dump_dir> [max_calls] (shared-runner contract; fixtures_dir unused).
 */
#include "sv_bundle_adjuster.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ reader */

typedef struct rd {
    const unsigned char* p;
    const unsigned char* end;
    int bad;
} rd;

static uint32_t r_u32(rd* r) {
    uint32_t v = 0;
    if (r->p + 4 > r->end) {
        r->bad = 1;
        return 0;
    }
    memcpy(&v, r->p, 4);
    r->p += 4;
    return v;
}
static int32_t r_i32(rd* r) { return (int32_t)r_u32(r); }
static float r_f32(rd* r) {
    float v = 0;
    if (r->p + 4 > r->end) {
        r->bad = 1;
        return 0;
    }
    memcpy(&v, r->p, 4);
    r->p += 4;
    return v;
}
static double r_f64(rd* r) {
    double v = 0;
    if (r->p + 8 > r->end) {
        r->bad = 1;
        return 0;
    }
    memcpy(&v, r->p, 8);
    r->p += 8;
    return v;
}

typedef struct call {
    uint32_t kind, call_index, curr_keyfrm_id, fixed_thr, num_first, num_second;
    double gain;
    uint32_t use_huber, fix_markers, flag_given, flag_in, use_additional, has_markers, ran_optimize, returned_ok;
    double fx, fy, cx, cy;
    int n_isq;
    float* isq;
    int n_order_kf;
    uint32_t* order_kf;
    int n_order_lm;
    uint32_t* order_lm;
    /* map view storage */
    sv_bav_kf* kfs;
    int n_kfs;
    sv_bav_lm* lms;
    int n_lms;
    /* graph as dumped */
    int n_vert;
    struct dv {
        uint32_t id, is_lm, fixed, owner;
        double init[7], fin[7];
    }* vert;
    int n_edge;
    struct de {
        uint32_t lm_vtx, kf_vtx, kf_id, lm_id, idx;
        double obs[2], isq, delta;
        uint32_t has_kernel;
        double chi2, err[2];
        uint32_t depth, level;
    }* edge;
    int n_stage;
    struct ds {
        uint32_t requested;
        int32_t returned;
        uint32_t flag_after, stopped;
        int n_levels;
        unsigned char* levels;
        int n_iter;
        struct di {
            double chi2, lambda;
            int32_t lev;
            uint32_t flag;
        }* iters;
    }* stage;
    int n_out;
    uint32_t (*outl)[2];
    int n_applied;
    uint32_t* applied_kf;
    double (*applied)[16];
    int n_opt;
    uint32_t* opt;
} call;

static void call_free(call* c) {
    int i;
    free(c->isq);
    free(c->order_kf);
    free(c->order_lm);
    for (i = 0; i < c->n_kfs; ++i) {
        free((void*)c->kfs[i].slots);
        free((void*)c->kfs[i].kps);
    }
    free(c->kfs);
    for (i = 0; i < c->n_lms; ++i) {
        free((void*)c->lms[i].obs_kf);
        free((void*)c->lms[i].obs_idx);
    }
    free(c->lms);
    free(c->vert);
    free(c->edge);
    for (i = 0; i < c->n_stage; ++i) {
        free(c->stage[i].levels);
        free(c->stage[i].iters);
    }
    free(c->stage);
    free(c->outl);
    free(c->applied_kf);
    free(c->applied);
    free(c->opt);
    memset(c, 0, sizeof(*c));
}

static int read_call(rd* r, call* c) {
    int i, j;
    memset(c, 0, sizeof(*c));
    c->kind = r_u32(r);
    c->call_index = r_u32(r);
    c->curr_keyfrm_id = r_u32(r);
    c->fixed_thr = r_u32(r);
    c->num_first = r_u32(r);
    c->num_second = r_u32(r);
    c->gain = r_f64(r);
    c->use_huber = r_u32(r);
    c->fix_markers = r_u32(r);
    c->flag_given = r_u32(r);
    c->flag_in = r_u32(r);
    c->use_additional = r_u32(r);
    c->has_markers = r_u32(r);
    c->ran_optimize = r_u32(r);
    c->returned_ok = r_u32(r);
    c->fx = r_f64(r);
    c->fy = r_f64(r);
    c->cx = r_f64(r);
    c->cy = r_f64(r);
    c->n_isq = (int)r_u32(r);
    c->isq = (float*)malloc(sizeof(float) * (size_t)(c->n_isq + 1));
    for (i = 0; i < c->n_isq; ++i) {
        c->isq[i] = r_f32(r);
    }
    c->n_order_kf = (int)r_u32(r);
    c->order_kf = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(c->n_order_kf + 1));
    for (i = 0; i < c->n_order_kf; ++i) {
        c->order_kf[i] = r_u32(r);
    }
    c->n_order_lm = (int)r_u32(r);
    c->order_lm = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(c->n_order_lm + 1));
    for (i = 0; i < c->n_order_lm; ++i) {
        c->order_lm[i] = r_u32(r);
    }
    c->n_kfs = (int)r_u32(r);
    c->kfs = (sv_bav_kf*)calloc((size_t)(c->n_kfs + 1), sizeof(sv_bav_kf));
    for (i = 0; i < c->n_kfs; ++i) {
        sv_bav_kf* k = &c->kfs[i];
        k->id = r_u32(r);
        k->erased = (int)r_u32(r);
        k->spanning_root = (int)r_u32(r);
        for (j = 0; j < 16; ++j) {
            k->pose_cw[j] = r_f64(r);
        }
        k->has_slots = (int)r_u32(r);
        k->n_slots = (int)r_u32(r);
        uint32_t* sl = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(k->n_slots + 1));
        for (j = 0; j < k->n_slots; ++j) {
            sl[j] = r_u32(r);
        }
        k->slots = sl;
        k->n_kp = (int)r_u32(r);
        sv_bav_kp* kp = (sv_bav_kp*)malloc(sizeof(sv_bav_kp) * (size_t)(k->n_kp + 1));
        for (j = 0; j < k->n_kp; ++j) {
            kp[j].idx = r_u32(r);
            kp[j].x = r_f32(r);
            kp[j].y = r_f32(r);
            kp[j].octave = r_i32(r);
        }
        k->kps = kp;
    }
    c->n_lms = (int)r_u32(r);
    c->lms = (sv_bav_lm*)calloc((size_t)(c->n_lms + 1), sizeof(sv_bav_lm));
    for (i = 0; i < c->n_lms; ++i) {
        sv_bav_lm* l = &c->lms[i];
        l->id = r_u32(r);
        l->erased = (int)r_u32(r);
        for (j = 0; j < 3; ++j) {
            l->pos[j] = r_f64(r);
        }
        l->n_obs = (int)r_u32(r);
        uint32_t* ok = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(l->n_obs + 1));
        uint32_t* oi = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(l->n_obs + 1));
        for (j = 0; j < l->n_obs; ++j) {
            ok[j] = r_u32(r);
            oi[j] = r_u32(r);
        }
        l->obs_kf = ok;
        l->obs_idx = oi;
    }
    c->n_vert = (int)r_u32(r);
    c->vert = (struct dv*)calloc((size_t)(c->n_vert + 1), sizeof(struct dv));
    for (i = 0; i < c->n_vert; ++i) {
        c->vert[i].id = r_u32(r);
        c->vert[i].is_lm = r_u32(r);
        c->vert[i].fixed = r_u32(r);
        c->vert[i].owner = r_u32(r);
        for (j = 0; j < 7; ++j) {
            c->vert[i].init[j] = r_f64(r);
        }
        for (j = 0; j < 7; ++j) {
            c->vert[i].fin[j] = r_f64(r);
        }
    }
    c->n_edge = (int)r_u32(r);
    c->edge = (struct de*)calloc((size_t)(c->n_edge + 1), sizeof(struct de));
    for (i = 0; i < c->n_edge; ++i) {
        struct de* e = &c->edge[i];
        e->lm_vtx = r_u32(r);
        e->kf_vtx = r_u32(r);
        e->kf_id = r_u32(r);
        e->lm_id = r_u32(r);
        e->idx = r_u32(r);
        e->obs[0] = r_f64(r);
        e->obs[1] = r_f64(r);
        e->isq = r_f64(r);
        e->delta = r_f64(r);
        e->has_kernel = r_u32(r);
        e->chi2 = r_f64(r);
        e->err[0] = r_f64(r);
        e->err[1] = r_f64(r);
        e->depth = r_u32(r);
        e->level = r_u32(r);
    }
    c->n_stage = (int)r_u32(r);
    c->stage = (struct ds*)calloc((size_t)(c->n_stage + 1), sizeof(struct ds));
    for (i = 0; i < c->n_stage; ++i) {
        struct ds* s = &c->stage[i];
        s->requested = r_u32(r);
        s->returned = r_i32(r);
        s->flag_after = r_u32(r);
        s->stopped = r_u32(r);
        s->n_levels = (int)r_u32(r);
        s->levels = (unsigned char*)malloc((size_t)(s->n_levels + 1));
        if (r->p + s->n_levels > r->end) {
            r->bad = 1;
            return 0;
        }
        memcpy(s->levels, r->p, (size_t)s->n_levels);
        r->p += s->n_levels;
        r->p += (4 - (s->n_levels % 4)) % 4;
        s->n_iter = (int)r_u32(r);
        s->iters = (struct di*)calloc((size_t)(s->n_iter + 1), sizeof(struct di));
        for (j = 0; j < s->n_iter; ++j) {
            s->iters[j].chi2 = r_f64(r);
            s->iters[j].lambda = r_f64(r);
            s->iters[j].lev = r_i32(r);
            s->iters[j].flag = r_u32(r);
        }
    }
    c->n_out = (int)r_u32(r);
    c->outl = (uint32_t(*)[2])malloc(sizeof(uint32_t[2]) * (size_t)(c->n_out + 1));
    for (i = 0; i < c->n_out; ++i) {
        c->outl[i][0] = r_u32(r);
        c->outl[i][1] = r_u32(r);
    }
    c->n_applied = (int)r_u32(r);
    c->applied_kf = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(c->n_applied + 1));
    c->applied = (double(*)[16])malloc(sizeof(double[16]) * (size_t)(c->n_applied + 1));
    for (i = 0; i < c->n_applied; ++i) {
        c->applied_kf[i] = r_u32(r);
        for (j = 0; j < 16; ++j) {
            c->applied[i][j] = r_f64(r);
        }
    }
    c->n_opt = (int)r_u32(r);
    c->opt = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(c->n_opt + 1));
    for (i = 0; i < c->n_opt; ++i) {
        c->opt[i] = r_u32(r);
    }
    return !r->bad;
}

/* ---------------------------------------------------------------- compare */

typedef struct cmpst {
    double tol;         /* tolerance on |ref-port| / max(1, |ref|) (0 = bit-exact) */
    int verbose;
    int diffs;          /* structural or out-of-tolerance differences in this call */
    double max_abs[8];  /* max absolute deviation per category */
    double max_mix[8];  /* max |ref-port| / max(1,|ref|) per category */
    int first_printed;
    long n_num_diff[8];
} cmpst;

enum { CAT_CHI2, CAT_LAMBDA, CAT_VERT, CAT_EDGE, CAT_POSE, CAT_N };
static const char* cat_name[CAT_N] = {"chi2", "lambda", "vertex", "edge", "applied_pose"};

static void note(cmpst* s, const char* what, long a, long b) {
    s->diffs++;
    if (s->verbose && !s->first_printed) {
        fprintf(stderr, "    first diff: %s (ref %ld, port %ld)\n", what, a, b);
        s->first_printed = 1;
    }
}

static void note_num(cmpst* s, const char* what, int cat, double ref, double port, long index) {
    /* exact-equal bits => fine */
    if (memcmp(&ref, &port, sizeof(double)) == 0) {
        return;
    }
    const double dev = fabs(ref - port);
    const double mix = dev / (fabs(ref) > 1.0 ? fabs(ref) : 1.0);
    if (dev > s->max_abs[cat]) {
        s->max_abs[cat] = dev;
    }
    if (mix > s->max_mix[cat]) {
        s->max_mix[cat] = mix;
    }
    s->n_num_diff[cat]++;
    if (s->tol > 0 && mix <= s->tol) {
        return;
    }
    s->diffs++;
    if (s->verbose && !s->first_printed) {
        fprintf(stderr, "    first diff: %s[%ld] ref %.17g port %.17g (dev %.3g)\n", what, index, ref, port, dev);
        s->first_printed = 1;
    }
}

static void compare_result(const call* c, const sv_bav_result* r, cmpst* s) {
    int i, j;
    if (r->n_stages != c->n_stage) {
        note(s, "n_stages", c->n_stage, r->n_stages);
        return;
    }
    /* graph as built */
    if (r->nv0 != c->n_vert) {
        note(s, "n_vertices", c->n_vert, r->nv0);
        return;
    }
    if (r->ne0 != c->n_edge) {
        note(s, "n_edges", c->n_edge, r->ne0);
        return;
    }
    for (i = 0; i < c->n_vert; ++i) {
        const sv_ba_vertex* v = &r->v0[i];
        const struct dv* d = &c->vert[i];
        if (v->id != d->id || (uint32_t)v->is_landmark != d->is_lm || (uint32_t)v->fixed != d->fixed ||
            r->vtx_owner[i] != d->owner) {
            if (s->verbose) {
                fprintf(stderr, "\n      vertex %d: port id %u lm %d fixed %d owner %u | ref id %u lm %u fixed %u owner %u\n", i, v->id,
                        v->is_landmark, v->fixed, r->vtx_owner[i], d->id, d->is_lm, d->fixed, d->owner);
            }
            note(s, "vertex id/kind/fixed/owner", i, i);
            return;
        }
        if (!v->is_landmark) {
            const double est[7] = {v->pose.q.x, v->pose.q.y, v->pose.q.z, v->pose.q.w, v->pose.t[0], v->pose.t[1], v->pose.t[2]};
            for (j = 0; j < 7; ++j) {
                note_num(s, "vertex_init", CAT_VERT, d->init[j], est[j], i * 7 + j);
            }
        } else {
            for (j = 0; j < 3; ++j) {
                note_num(s, "vertex_init", CAT_VERT, d->init[j], v->pos[j], i * 7 + j);
            }
        }
    }
    for (i = 0; i < c->n_edge; ++i) {
        const sv_ba_edge* e = &r->e0[i];
        const struct de* d = &c->edge[i];
        if (r->g.v[e->lm].id != d->lm_vtx || r->g.v[e->kf].id != d->kf_vtx || r->edge_kf_id[i] != d->kf_id ||
            r->edge_lm_id[i] != d->lm_id || r->edge_idx[i] != d->idx || (uint32_t)e->has_kernel != d->has_kernel) {
            note(s, "edge ids/kernel", i, i);
            return;
        }
        if (memcmp(e->obs, d->obs, 16) != 0 || memcmp(&e->info, &d->isq, 8) != 0 ||
            (e->has_kernel && memcmp(&e->huber_delta, &d->delta, 8) != 0)) {
            note(s, "edge measurement/information/delta", i, i);
            return;
        }
    }
    /* stages */
    for (i = 0; i < c->n_stage; ++i) {
        const struct ds* d = &c->stage[i];
        const sv_bav_stage* p = &r->stages[i];
        if (d->requested != p->requested_iters) {
            note(s, "stage requested", d->requested, p->requested_iters);
        }
        if (d->returned != p->returned_iters) {
            note(s, "stage returned iterations", d->returned, p->returned_iters);
            return;
        }
        if (d->flag_after != p->flag_after) {
            note(s, "stage flag_after", d->flag_after, p->flag_after);
        }
        if (d->stopped != (uint32_t)p->stopped_by_terminate) {
            note(s, "stage stopped_by_terminate", d->stopped, p->stopped_by_terminate);
        }
        if (d->n_levels != p->n_levels || memcmp(d->levels, p->levels, (size_t)d->n_levels) != 0) {
            note(s, "stage edge levels", i, i);
            return;
        }
        if (d->n_iter != p->n_iters) {
            note(s, "stage n_iters", d->n_iter, p->n_iters);
            return;
        }
        for (j = 0; j < d->n_iter; ++j) {
            note_num(s, "iter chi2", CAT_CHI2, d->iters[j].chi2, p->iters[j].chi2, i * 1000 + j);
            note_num(s, "iter lambda", CAT_LAMBDA, d->iters[j].lambda, p->iters[j].lambda, i * 1000 + j);
            if (d->iters[j].lev != p->iters[j].lev_iter) {
                note(s, "iter lev_iter", d->iters[j].lev, p->iters[j].lev_iter);
                return;
            }
            if (d->iters[j].flag != p->iters[j].flag) {
                note(s, "iter flag", d->iters[j].flag, p->iters[j].flag);
            }
        }
    }
    /* finals */
    for (i = 0; i < c->n_vert; ++i) {
        const sv_ba_vertex* v = &r->g.v[i];
        const struct dv* d = &c->vert[i];
        if (!v->is_landmark) {
            const double est[7] = {v->pose.q.x, v->pose.q.y, v->pose.q.z, v->pose.q.w, v->pose.t[0], v->pose.t[1], v->pose.t[2]};
            for (j = 0; j < 7; ++j) {
                note_num(s, "vertex_final", CAT_VERT, d->fin[j], est[j], i * 7 + j);
            }
        } else {
            for (j = 0; j < 3; ++j) {
                note_num(s, "vertex_final", CAT_VERT, d->fin[j], v->pos[j], i * 7 + j);
            }
        }
    }
    for (i = 0; i < c->n_edge; ++i) {
        const sv_ba_edge* e = &r->g.e[i];
        const struct de* d = &c->edge[i];
        note_num(s, "edge_chi2", CAT_EDGE, d->chi2, sv_ba_edge_chi2(e), i);
        note_num(s, "edge_err0", CAT_EDGE, d->err[0], e->err[0], i);
        note_num(s, "edge_err1", CAT_EDGE, d->err[1], e->err[1], i);
        if (d->depth != (uint32_t)sv_ba_edge_depth_positive(&r->g, e)) {
            note(s, "edge depth_positive", i, i);
        }
        if (d->level != (uint32_t)e->level) {
            note(s, "edge level_final", d->level, e->level);
        }
    }
    if (r->n_outliers != c->n_out) {
        note(s, "n_outliers", c->n_out, r->n_outliers);
    } else {
        for (i = 0; i < c->n_out; ++i) {
            if (c->outl[i][0] != r->outliers[i][0] || c->outl[i][1] != r->outliers[i][1]) {
                note(s, "outlier pair", i, i);
                break;
            }
        }
    }
    if (r->n_applied != c->n_applied) {
        note(s, "n_applied", c->n_applied, r->n_applied);
    } else {
        for (i = 0; i < c->n_applied; ++i) {
            if (c->applied_kf[i] != r->applied_kf[i]) {
                note(s, "applied kf id", i, i);
                break;
            }
            for (j = 0; j < 16; ++j) {
                note_num(s, "applied_pose", CAT_POSE, c->applied[i][j], r->applied_pose[i][j], i * 16 + j);
            }
        }
    }
    if (c->kind != 0) {
        if (r->n_opt_lm != c->n_opt) {
            note(s, "n_optimized_lm", c->n_opt, r->n_opt_lm);
        } else {
            for (i = 0; i < c->n_opt; ++i) {
                if (c->opt[i] != r->opt_lm[i]) {
                    note(s, "optimized lm id", i, i);
                    break;
                }
            }
        }
    }
}

/* ---- optional: real g2o LinearSolverEigen replay (replay_ba_eigen.cc) ---- */
typedef struct replay_call {
    uint32_t call_index;
    int32_t ret;
    int n_iter;
    struct {
        double chi2, lambda;
        int32_t lev;
    }* it;
    int n_vert;
    double (*est)[7];
} replay_call;

static replay_call* load_replay(const char* path, int* n_out) {
    FILE* f = fopen(path, "rb");
    *n_out = 0;
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* b = (unsigned char*)malloc((size_t)size);
    if (fread(b, 1, (size_t)size, f) != (size_t)size) {
        fclose(f);
        free(b);
        return NULL;
    }
    fclose(f);
    rd r = {b, b + size, 0};
    if (r_u32(&r) != 0x31525245u) {
        free(b);
        return NULL;
    }
    const int n = (int)r_u32(&r);
    replay_call* rc = (replay_call*)calloc((size_t)(n + 1), sizeof(replay_call));
    int i, j, k;
    for (i = 0; i < n; ++i) {
        rc[i].call_index = r_u32(&r);
        rc[i].ret = r_i32(&r);
        rc[i].n_iter = (int)r_u32(&r);
        rc[i].it = calloc((size_t)(rc[i].n_iter + 1), 24);
        for (j = 0; j < rc[i].n_iter; ++j) {
            double* d = (double*)((char*)rc[i].it + (size_t)j * 24);
            d[0] = r_f64(&r);
            d[1] = r_f64(&r);
            *(int32_t*)(d + 2) = r_i32(&r);
        }
        rc[i].n_vert = (int)r_u32(&r);
        rc[i].est = (double(*)[7])malloc(sizeof(double[7]) * (size_t)(rc[i].n_vert + 1));
        for (j = 0; j < rc[i].n_vert; ++j) {
            for (k = 0; k < 7; ++k) {
                rc[i].est[j][k] = r_f64(&r);
            }
        }
    }
    free(b);
    *n_out = n;
    return rc;
}

/* port vs real-LinearSolverEigen replay: bit-exact; also tallies the
 * CSparse (dump) vs Eigen (replay) deviation. Returns number of diffs. */
static int compare_replay(const call* c, const sv_bav_result* r, const replay_call* rc, double* max_csparse_dev,
                          double* max_csparse_chi_rel, int* n_perm_diff_iters) {
    int i, j, diffs = 0;
    const sv_bav_stage* p = &r->stages[0];
    if (rc->ret != p->returned_iters || rc->n_iter != p->n_iters) {
        return 1;
    }
    for (i = 0; i < rc->n_iter; ++i) {
        const double* d = (const double*)((const char*)rc->it + (size_t)i * 24);
        if (memcmp(&d[0], &p->iters[i].chi2, 8) != 0 || memcmp(&d[1], &p->iters[i].lambda, 8) != 0 ||
            *(const int32_t*)(d + 2) != p->iters[i].lev_iter) {
            ++diffs;
        }
        if (i < c->stage[0].n_iter) {
            const double ref = c->stage[0].iters[i].chi2;
            const double rel = fabs(ref - d[0]) / (fabs(ref) > 1.0 ? fabs(ref) : 1.0);
            if (rel > *max_csparse_chi_rel) {
                *max_csparse_chi_rel = rel;
            }
            if (memcmp(&ref, &d[0], 8) != 0) {
                ++*n_perm_diff_iters;
            }
        }
    }
    if (rc->n_vert != r->g.nv) {
        return diffs + 1;
    }
    for (i = 0; i < rc->n_vert; ++i) {
        const sv_ba_vertex* v = &r->g.v[i];
        const double est[7] = {v->pose.q.x, v->pose.q.y, v->pose.q.z, v->pose.q.w, v->pose.t[0], v->pose.t[1], v->pose.t[2]};
        const int nd = v->is_landmark ? 3 : 7;
        for (j = 0; j < nd; ++j) {
            const double mine = v->is_landmark ? v->pos[j] : est[j];
            if (memcmp(&rc->est[i][j], &mine, 8) != 0) {
                ++diffs;
            }
            {
                const double ref = c->vert[i].fin[j];
                const double dev = fabs(ref - rc->est[i][j]);
                if (dev > *max_csparse_dev) {
                    *max_csparse_dev = dev;
                }
            }
        }
    }
    return diffs;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_g2o_ba <seq_label> <fixtures_dir> <dump_dir> [max_calls]\n");
        return 1;
    }
    const char* seq_label = argv[1];
    const char* dump_dir = argv[3];
    long max_calls = (argc >= 5) ? atol(argv[4]) : -1;

    char path[4096];
    {
        const char* pos = strstr(dump_dir, "reference_dumps");
        if (pos) {
            snprintf(path, sizeof(path), "%.*sreference_g2o_ba/%s/ba_calls.bin", (int)(pos - dump_dir), dump_dir, seq_label);
        } else {
            snprintf(path, sizeof(path), "%s/../reference_g2o_ba/%s/ba_calls.bin", dump_dir, seq_label);
        }
    }
    FILE* f = fopen(path, "rb");
    if (!f) {
        printf("%s: 0/0\n", seq_label); /* no BA dump for this sequence: skip cleanly */
        fprintf(stderr, "check_sv_g2o_ba: no dump at %s (skipped)\n", path);
        return 0;
    }
    char rpath[4096];
    snprintf(rpath, sizeof(rpath), "%s", path);
    {
        char* slash = strrchr(rpath, '/');
        if (slash) {
            strcpy(slash + 1, "ba_eigen_replay.bin");
        }
    }
    int n_replay = 0;
    replay_call* replay = load_replay(rpath, &n_replay);
    long replay_total = 0, replay_bad = 0;
    double cs_dev = 0, cs_chi_rel = 0;
    int cs_diff_iters = 0;
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* buf = (unsigned char*)malloc((size_t)size);
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) {
        fprintf(stderr, "short read\n");
        return 2;
    }
    fclose(f);
    rd r = {buf, buf + size, 0};
    if (r_u32(&r) != 0x32434142u) {
        fprintf(stderr, "bad magic\n");
        return 2;
    }
    const uint32_t n_calls = r_u32(&r);

    const char* tol_env = getenv("SV_BA_GLOBAL_TOL");
    const double global_tol = tol_env ? atof(tol_env) : 1e-9;
    const int verbose = getenv("SV_BA_VERBOSE") != NULL;

    long total = 0, mismatches = 0;
    long stat_two_stage = 0, stat_edges = 0, stat_iters = 0, stat_outliers = 0, stat_local = 0;
    long by_kind_total[3] = {0, 0, 0}, by_kind_bad[3] = {0, 0, 0};
    double max_abs[3][CAT_N], max_mix[3][CAT_N];
    long n_num_diff[3][CAT_N];
    memset(max_abs, 0, sizeof(max_abs));
    memset(max_mix, 0, sizeof(max_mix));
    memset(n_num_diff, 0, sizeof(n_num_diff));
    uint32_t ci;
    for (ci = 0; ci < n_calls; ++ci) {
        call c;
        if (!read_call(&r, &c)) {
            fprintf(stderr, "truncated dump at call %u\n", ci);
            return 2;
        }
        if (max_calls >= 0 && total >= max_calls) {
            call_free(&c);
            break;
        }
        if (c.has_markers || (!c.ran_optimize && c.kind != 0)) {
            call_free(&c);
            continue;
        }
        cmpst s;
        memset(&s, 0, sizeof(s));
        s.tol = (c.kind == 0) ? 0.0 : global_tol;
        s.verbose = verbose;

        sv_bav_view view;
        memset(&view, 0, sizeof(view));
        view.kfs = c.kfs;
        view.n_kfs = c.n_kfs;
        view.lms = c.lms;
        view.n_lms = c.n_lms;
        view.fx = c.fx;
        view.fy = c.fy;
        view.cx = c.cx;
        view.cy = c.cy;
        view.n_isq = c.n_isq;
        view.isq = c.isq;
        view.fixed_keyframe_id_threshold = c.fixed_thr;

        sv_bav_result res;
        int flag = (int)c.flag_in;
        int* flag_ptr = c.flag_given ? &flag : NULL;
        if (c.kind == 0) {
            sv_bav_local(&view, c.curr_keyfrm_id, c.order_kf, c.n_order_kf, c.num_first, c.num_second,
                         (int)c.use_additional, flag_ptr, &res);
        } else if (c.kind == 1) {
            sv_bav_global_init(&view, c.order_kf, c.n_order_kf, c.order_lm, c.n_order_lm, c.num_first, (int)c.use_huber,
                               c.gain, flag_ptr, &res);
        } else {
            sv_bav_global_loop(&view, c.order_kf, c.n_order_kf, c.num_first, (int)c.use_huber, flag_ptr, &res);
        }
        if (verbose) {
            fprintf(stderr, "call %u kind %u: ", c.call_index, c.kind);
        }
        if (!c.ran_optimize) {
            if (res.ran_optimize) {
                note(&s, "ran_optimize", 0, 1);
            }
        } else {
            compare_result(&c, &res, &s);
        }
        if (replay && c.kind != 0 && c.ran_optimize) {
            int q;
            for (q = 0; q < n_replay; ++q) {
                if (replay[q].call_index == c.call_index) {
                    const int d = compare_replay(&c, &res, &replay[q], &cs_dev, &cs_chi_rel, &cs_diff_iters);
                    ++replay_total;
                    if (d) {
                        ++replay_bad;
                        s.diffs++;
                    }
                    if (verbose) {
                        fprintf(stderr, "[real LinearSolverEigen replay: %s] ", d ? "MISMATCH" : "bit-exact");
                    }
                    break;
                }
            }
        }
        if (verbose) {
            fprintf(stderr, "%s\n", s.diffs ? "MISMATCH" : "ok");
        }
        ++total;
        stat_edges += c.n_edge;
        stat_outliers += c.n_out;
        {
            int q;
            for (q = 0; q < c.n_stage; ++q) {
                stat_iters += c.stage[q].n_iter;
            }
            if (c.kind == 0) {
                ++stat_local;
                if (c.n_stage == 2) {
                    ++stat_two_stage;
                }
            }
        }
        ++by_kind_total[c.kind];
        if (s.diffs) {
            ++mismatches;
            ++by_kind_bad[c.kind];
        }
        {
            int k;
            for (k = 0; k < CAT_N; ++k) {
                if (s.max_abs[k] > max_abs[c.kind][k]) {
                    max_abs[c.kind][k] = s.max_abs[k];
                }
                if (s.max_mix[k] > max_mix[c.kind][k]) {
                    max_mix[c.kind][k] = s.max_mix[k];
                }
                n_num_diff[c.kind][k] += s.n_num_diff[k];
            }
        }
        sv_bav_result_free(&res);
        call_free(&c);
    }
    free(buf);
    fprintf(stderr,
            "check_sv_g2o_ba %s: local %ld/%ld, global-init %ld/%ld, global-loop %ld/%ld mismatching calls "
            "(global tol %g)\n",
            seq_label, by_kind_bad[0], by_kind_total[0], by_kind_bad[1], by_kind_total[1], by_kind_bad[2],
            by_kind_total[2], global_tol);
    {
        int kd, k;
        for (kd = 0; kd < 3; ++kd) {
            for (k = 0; k < CAT_N; ++k) {
                if (n_num_diff[kd][k]) {
                    fprintf(stderr, "  kind %d %-13s: %ld differing values, max abs dev %.3g, max |d|/max(1,|ref|) %.3g\n", kd,
                            cat_name[k], n_num_diff[kd][k], max_abs[kd][k], max_mix[kd][k]);
                }
            }
        }
    }
    fprintf(stderr,
            "  coverage: %ld calls, %ld edges, %ld LM iterations, %ld outlier observations; local calls that ran the "
            "second (robust-kernel-free) stage: %ld/%ld\n",
            total, stat_edges, stat_iters, stat_outliers, stat_two_stage, stat_local);
    if (replay_total) {
        fprintf(stderr,
                "  real g2o LinearSolverEigen replay of the global graphs: %ld/%ld calls differ from the port "
                "(tolerance 0); CSparse reference vs real Eigen solver: max abs vertex dev %.3g, max chi2 rel dev %.3g, "
                "%d iterations with non-identical chi2\n",
                replay_bad, replay_total, cs_dev, cs_chi_rel, cs_diff_iters);
    }
    printf("%s: %ld/%ld\n", seq_label, mismatches, total);
    return mismatches == 0 ? 0 : 1;
}
