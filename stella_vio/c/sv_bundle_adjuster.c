/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_bundle_adjuster.h (BSD, stella_vslam/g2o-derived). */
#include "sv_bundle_adjuster.h"
#include "sv_umap_order.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NONE_ID 0xFFFFFFFFu

/* pose_optimizer / BA files declare these as FLOAT: `constexpr float
 * chi_sq_2D = 5.99146; const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);` */
#define CHI_SQ_2D ((double)5.99146f)

/* ---------------------------------------------------------------- helpers */

static int find_kf(const sv_bav_view* v, unsigned int id) {
    int lo = 0, hi = v->n_kfs - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (v->kfs[mid].id == id) {
            return mid;
        }
        if (v->kfs[mid].id < id) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return -1;
}

static int find_lm(const sv_bav_view* v, unsigned int id) {
    int lo = 0, hi = v->n_lms - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (v->lms[mid].id == id) {
            return mid;
        }
        if (v->lms[mid].id < id) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return -1;
}

static const sv_bav_kp* find_kp(const sv_bav_kf* kf, unsigned int idx) {
    int lo = 0, hi = kf->n_kp - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (kf->kps[mid].idx == idx) {
            return &kf->kps[mid];
        }
        if (kf->kps[mid].idx < idx) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return NULL;
}

void sv_bav_pose_from_mat44(const double m[16], sv_se3* out) {
    double colmajor_r[9];
    int r, c;
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            colmajor_r[c * 3 + r] = m[r * 4 + c];
        }
    }
    sv_quat_from_mat3(colmajor_r, &out->q);
    out->t[0] = m[0 * 4 + 3];
    out->t[1] = m[1 * 4 + 3];
    out->t[2] = m[2 * 4 + 3];
    /* `g2o::SE3Quat{rot,trans}` constructor calls normalizeRotation() */
    sv_se3_normalize_rotation(out);
}

void sv_bav_mat44_from_pose(const sv_se3* pose, double m[16]) {
    double R[9];
    int r, c;
    sv_quat_to_mat3(&pose->q, R);
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            m[r * 4 + c] = R[c * 3 + r];
        }
        m[r * 4 + 3] = pose->t[r];
    }
    m[12] = 0;
    m[13] = 0;
    m[14] = 0;
    m[15] = 1;
}

void sv_bav_result_free(sv_bav_result* r) {
    int i;
    sv_ba_graph_free(&r->g);
    free(r->v0);
    free(r->e0);
    free(r->vtx_owner);
    free(r->edge_kf_id);
    free(r->edge_lm_id);
    free(r->edge_idx);
    for (i = 0; i < r->n_stages; ++i) {
        free(r->stages[i].levels);
        free(r->stages[i].iters);
    }
    free(r->outliers);
    free(r->applied_kf);
    free(r->applied_pose);
    free(r->opt_lm);
    memset(r, 0, sizeof(*r));
}

/* growable per-vertex / per-edge metadata kept beside the graph */
typedef struct meta {
    unsigned int* vo;
    int vo_n, vo_cap;
    unsigned int* ek;
    unsigned int* el;
    unsigned int* ei;
    int e_n, e_cap;
} meta;

static void meta_add_vertex(meta* m, unsigned int owner) {
    if (m->vo_n == m->vo_cap) {
        m->vo_cap = m->vo_cap ? 2 * m->vo_cap : 64;
        m->vo = (unsigned int*)realloc(m->vo, sizeof(unsigned int) * (size_t)m->vo_cap);
    }
    m->vo[m->vo_n++] = owner;
}

static void meta_add_edge(meta* m, unsigned int kf, unsigned int lm, unsigned int idx) {
    if (m->e_n == m->e_cap) {
        m->e_cap = m->e_cap ? 2 * m->e_cap : 256;
        m->ek = (unsigned int*)realloc(m->ek, sizeof(unsigned int) * (size_t)m->e_cap);
        m->el = (unsigned int*)realloc(m->el, sizeof(unsigned int) * (size_t)m->e_cap);
        m->ei = (unsigned int*)realloc(m->ei, sizeof(unsigned int) * (size_t)m->e_cap);
    }
    m->ek[m->e_n] = kf;
    m->el[m->e_n] = lm;
    m->ei[m->e_n] = idx;
    m->e_n++;
}

/* Adds the reprojection edges of one landmark (both BA files share this
 * loop shape); returns the number of edges created. `kf_vtx[i]` is the
 * vertex index of view keyframe i or -1. */
static int add_landmark_edges(const sv_bav_view* view, sv_ba_graph* g, meta* mt, const sv_bav_lm* lm, int lm_vtx,
                              const int* kf_vtx, int use_huber) {
    const double sqrt_chi_sq_2d = (double)sqrtf((float)CHI_SQ_2D);
    int n_edges = 0, o;
    for (o = 0; o < lm->n_obs; ++o) {
        const int ki = find_kf(view, lm->obs_kf[o]);
        const unsigned int idx = lm->obs_idx[o];
        if (ki < 0) {
            continue;
        }
        const sv_bav_kf* kf = &view->kfs[ki];
        if (kf->erased) {
            continue;
        }
        if (kf_vtx[ki] < 0) {
            continue;
        }
        const sv_bav_kp* kp = find_kp(kf, idx);
        if (!kp) {
            continue; /* not captured: cannot happen for a valid view */
        }
        const double obs[2] = {(double)kp->x, (double)kp->y};
        const double isq = (double)view->isq[kp->octave];
        sv_ba_add_edge(g, lm_vtx, kf_vtx[ki], obs, isq, use_huber, use_huber ? sqrt_chi_sq_2d : 0.0);
        meta_add_edge(mt, kf->id, lm->id, idx);
        ++n_edges;
    }
    return n_edges;
}

static void graph_common_init(sv_ba_graph* g, const sv_bav_view* view) {
    sv_ba_graph_init(g);
    g->fx = view->fx;
    g->fy = view->fy;
    g->cx = view->cx;
    g->cy = view->cy;
}

static void stage_begin(sv_bav_result* r, unsigned int requested) {
    sv_bav_stage* st = &r->stages[r->n_stages++];
    int i;
    if (r->n_stages == 1) { /* snapshot of the graph as built */
        r->nv0 = r->g.nv;
        r->ne0 = r->g.ne;
        r->v0 = (sv_ba_vertex*)malloc(sizeof(sv_ba_vertex) * (size_t)(r->g.nv > 0 ? r->g.nv : 1));
        r->e0 = (sv_ba_edge*)malloc(sizeof(sv_ba_edge) * (size_t)(r->g.ne > 0 ? r->g.ne : 1));
        memcpy(r->v0, r->g.v, sizeof(sv_ba_vertex) * (size_t)r->g.nv);
        memcpy(r->e0, r->g.e, sizeof(sv_ba_edge) * (size_t)r->g.ne);
    }
    memset(st, 0, sizeof(*st));
    st->requested_iters = requested;
    st->n_levels = r->g.ne;
    st->levels = (unsigned char*)malloc((size_t)(r->g.ne > 0 ? r->g.ne : 1));
    for (i = 0; i < r->g.ne; ++i) {
        st->levels[i] = (unsigned char)r->g.e[i].level;
    }
}

static void stage_end(sv_bav_result* r, int returned) {
    sv_bav_stage* st = &r->stages[r->n_stages - 1];
    sv_ba_graph* g = &r->g;
    st->returned_iters = returned;
    st->flag_after = (g->stop_flag && *g->stop_flag) ? 1u : 0u;
    st->stopped_by_terminate = g->stopped_by_terminate;
    st->n_iters = g->n_iters;
    st->iters = (sv_ba_iter*)malloc(sizeof(sv_ba_iter) * (size_t)(g->n_iters > 0 ? g->n_iters : 1));
    memcpy(st->iters, g->iters, sizeof(sv_ba_iter) * (size_t)g->n_iters);
}

static void export_meta(sv_bav_result* r, meta* mt) {
    r->vtx_owner = mt->vo;
    r->edge_kf_id = mt->ek;
    r->edge_lm_id = mt->el;
    r->edge_idx = mt->ei;
}

/* ============================================================ local BA == */

int sv_bav_local(const sv_bav_view* view, unsigned int curr_id, const unsigned int* covis, int n_covis,
                 unsigned int num_first_iter, unsigned int num_second_iter, int use_additional_keyframes,
                 int* force_stop_flag, sv_bav_result* out) {
    memset(out, 0, sizeof(*out));
    out->returned_ok = 1;
    int i, j, o;

    const int curr_ki = find_kf(view, curr_id);
    if (curr_ki < 0) {
        return -1;
    }

    /* 1. Aggregate the local and fixed keyframes, and local landmarks */
    sv_umap_order local_keyfrms, fixed_keyfrms, local_lms;
    sv_umap_init(&local_keyfrms);
    sv_umap_init(&fixed_keyfrms);
    sv_umap_init(&local_lms);
    const int has_scale = 0; /* monocular */

    sv_umap_insert(&local_keyfrms, curr_id);
    for (i = 0; i < n_covis; ++i) {
        const int ki = find_kf(view, covis[i]);
        if (ki < 0) {
            continue;
        }
        const sv_bav_kf* lk = &view->kfs[ki];
        if (lk->erased) {
            continue;
        }
        if (lk->spanning_root) {
            continue;
        }
        if (lk->id < view->fixed_keyframe_id_threshold) {
            continue;
        }
        sv_umap_insert(&local_keyfrms, lk->id);
    }

    /* Correct landmarks seen in local keyframes */
    for (i = local_keyfrms.first; i != -1; i = local_keyfrms.next[i]) {
        const sv_bav_kf* lk = &view->kfs[find_kf(view, local_keyfrms.keys[i])];
        for (j = 0; j < lk->n_slots; ++j) {
            if (lk->slots[j] == NONE_ID) {
                continue;
            }
            const int li = find_lm(view, lk->slots[j]);
            if (li < 0 || view->lms[li].erased) {
                continue;
            }
            /* Avoid duplication */
            sv_umap_insert(&local_lms, view->lms[li].id);
        }
    }

    /* Fixed keyframes: keyframes which observe local landmarks but which are NOT in local keyframes */
    for (i = local_lms.first; i != -1; i = local_lms.next[i]) {
        const sv_bav_lm* lm = &view->lms[find_lm(view, local_lms.keys[i])];
        for (o = 0; o < lm->n_obs; ++o) {
            const int ki = find_kf(view, lm->obs_kf[o]);
            if (ki < 0 || view->kfs[ki].erased) {
                continue;
            }
            if (sv_umap_contains(&local_keyfrms, view->kfs[ki].id)) {
                continue;
            }
            sv_umap_insert(&fixed_keyfrms, view->kfs[ki].id); /* duplicates ignored */
        }
    }

    if (use_additional_keyframes) {
        /* Ensure that there are always at least two fixed keyframes */
        const size_t additional = (size_t)2 - (size_t)fixed_keyfrms.count;
        if (!has_scale && fixed_keyfrms.count < 2 && (size_t)local_keyfrms.count > additional) {
            size_t k;
            for (k = 0; k < additional; ++k) {
                const unsigned int id = local_keyfrms.keys[local_keyfrms.first];
                sv_umap_erase(&local_keyfrms, id);
                sv_umap_insert(&fixed_keyfrms, id);
            }
        }
    }

    /* 2./3. optimizer, keyframe vertices */
    sv_ba_graph* g = &out->g;
    graph_common_init(g, view);
    g->gain_threshold = 1e-3;
    meta mt;
    memset(&mt, 0, sizeof(mt));
    sv_ba_set_force_stop_flag(g, force_stop_flag);

    int* kf_vtx = (int*)malloc(sizeof(int) * (size_t)(view->n_kfs > 0 ? view->n_kfs : 1));
    for (i = 0; i < view->n_kfs; ++i) {
        kf_vtx[i] = -1;
    }
    unsigned int vtx_id_offset = 0;
    for (i = local_keyfrms.first; i != -1; i = local_keyfrms.next[i]) {
        const int ki = find_kf(view, local_keyfrms.keys[i]);
        sv_se3 pose;
        sv_bav_pose_from_mat44(view->kfs[ki].pose_cw, &pose);
        kf_vtx[ki] = sv_ba_add_shot_vertex(g, vtx_id_offset++, &pose, 0);
        meta_add_vertex(&mt, view->kfs[ki].id);
    }
    for (i = fixed_keyfrms.first; i != -1; i = fixed_keyfrms.next[i]) {
        const int ki = find_kf(view, fixed_keyfrms.keys[i]);
        sv_se3 pose;
        sv_bav_pose_from_mat44(view->kfs[ki].pose_cw, &pose);
        kf_vtx[ki] = sv_ba_add_shot_vertex(g, vtx_id_offset++, &pose, 1);
        meta_add_vertex(&mt, view->kfs[ki].id);
    }

    /* 4. landmark vertices + reprojection edges */
    for (i = local_lms.first; i != -1; i = local_lms.next[i]) {
        const sv_bav_lm* lm = &view->lms[find_lm(view, local_lms.keys[i])];
        if (lm->n_obs == 0) {
            continue; /* "empty observation" */
        }
        const int lv = sv_ba_add_landmark_vertex(g, vtx_id_offset++, lm->pos, 0);
        meta_add_vertex(&mt, lm->id);
        add_landmark_edges(view, g, &mt, lm, lv, kf_vtx, 1);
    }

    /* 5. Perform the first optimization */
    if (force_stop_flag && *force_stop_flag) {
        free(kf_vtx);
        sv_umap_free(&local_keyfrms);
        sv_umap_free(&fixed_keyfrms);
        sv_umap_free(&local_lms);
        export_meta(out, &mt);
        out->returned_ok = 1;
        return 0;
    }

    out->ran_optimize = 1;
    stage_begin(out, num_first_iter);
    sv_ba_initialize_optimization(g);
    stage_end(out, sv_ba_optimize(g, (int)num_first_iter));

    /* 6. Discard outliers, then perform the second optimization */
    int run_robust_ba = 1;
    if (force_stop_flag && *force_stop_flag) {
        run_robust_ba = 0;
    }

    if (run_robust_ba) {
        for (i = 0; i < g->ne; ++i) {
            sv_ba_edge* e = &g->e[i];
            const int li = find_lm(view, mt.el[i]);
            if (view->lms[li].erased) {
                continue;
            }
            if (CHI_SQ_2D < sv_ba_edge_chi2(e) || !sv_ba_edge_depth_positive(g, e)) {
                e->level = 1; /* set_as_outlier */
            }
            e->has_kernel = 0; /* setRobustKernel(nullptr) */
        }
        stage_begin(out, num_second_iter);
        sv_ba_initialize_optimization(g);
        stage_end(out, sv_ba_optimize(g, (int)num_second_iter));
    }

    /* 7. Count the outliers */
    out->outliers = (unsigned int(*)[2])malloc(sizeof(unsigned int[2]) * (size_t)(g->ne > 0 ? g->ne : 1));
    for (i = 0; i < g->ne; ++i) {
        const sv_ba_edge* e = &g->e[i];
        const int li = find_lm(view, mt.el[i]);
        if (view->lms[li].erased) {
            continue;
        }
        if (CHI_SQ_2D < sv_ba_edge_chi2(e) || !sv_ba_edge_depth_positive(g, e)) {
            out->outliers[out->n_outliers][0] = mt.ek[i];
            out->outliers[out->n_outliers][1] = mt.el[i];
            out->n_outliers++;
        }
    }

    /* 8. Update the information: local keyframe poses */
    out->applied_kf = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(local_keyfrms.count + 1));
    out->applied_pose = (double(*)[16])malloc(sizeof(double[16]) * (size_t)(local_keyfrms.count + 1));
    for (i = local_keyfrms.first; i != -1; i = local_keyfrms.next[i]) {
        const int ki = find_kf(view, local_keyfrms.keys[i]);
        sv_bav_mat44_from_pose(&g->v[kf_vtx[ki]].pose, out->applied_pose[out->n_applied]);
        out->applied_kf[out->n_applied] = view->kfs[ki].id;
        out->n_applied++;
    }

    free(kf_vtx);
    sv_umap_free(&local_keyfrms);
    sv_umap_free(&fixed_keyfrms);
    sv_umap_free(&local_lms);
    export_meta(out, &mt);
    return 0;
}

/* ===================================================== global BA (impl) == */

/* optimize_impl(): keyframe vertices, landmark vertices + edges, initialize,
 * optimize. `lm_ids` may contain NONE_ID. `is_optimized_lm[i]` out. */
static int global_impl(const sv_bav_view* view, const unsigned int* keyfrm_ids, int n_keyfrms,
                       const unsigned int* lm_ids, int n_lms, unsigned int num_iter, int use_huber,
                       double gain_threshold, int* force_stop_flag, sv_bav_result* out, unsigned char* is_optimized_lm,
                       int** kf_vtx_out) {
    int i;
    sv_ba_graph* g = &out->g;
    graph_common_init(g, view);
    g->gain_threshold = gain_threshold;
    meta mt;
    memset(&mt, 0, sizeof(mt));
    sv_ba_set_force_stop_flag(g, force_stop_flag);

    int* kf_vtx = (int*)malloc(sizeof(int) * (size_t)(view->n_kfs > 0 ? view->n_kfs : 1));
    for (i = 0; i < view->n_kfs; ++i) {
        kf_vtx[i] = -1;
    }
    unsigned int vtx_id_offset = 0;
    for (i = 0; i < n_keyfrms; ++i) {
        if (keyfrm_ids[i] == NONE_ID) {
            continue;
        }
        const int ki = find_kf(view, keyfrm_ids[i]);
        if (ki < 0 || view->kfs[ki].erased) {
            continue;
        }
        sv_se3 pose;
        sv_bav_pose_from_mat44(view->kfs[ki].pose_cw, &pose);
        kf_vtx[ki] = sv_ba_add_shot_vertex(g, vtx_id_offset++, &pose, view->kfs[ki].spanning_root);
        meta_add_vertex(&mt, view->kfs[ki].id);
    }

    for (i = 0; i < n_lms; ++i) {
        is_optimized_lm[i] = 1;
        if (lm_ids[i] == NONE_ID) {
            continue;
        }
        const int li = find_lm(view, lm_ids[i]);
        if (li < 0 || view->lms[li].erased) {
            continue;
        }
        const sv_bav_lm* lm = &view->lms[li];
        const int lv = sv_ba_add_landmark_vertex(g, vtx_id_offset++, lm->pos, 0);
        meta_add_vertex(&mt, lm->id);
        const int ne = add_landmark_edges(view, g, &mt, lm, lv, kf_vtx, use_huber);
        if (ne == 0) {
            /* optimizer.removeVertex(lm_vtx): the id stays consumed */
            g->nv--;
            mt.vo_n--;
            is_optimized_lm[i] = 0;
        }
    }

    out->ran_optimize = 1;
    stage_begin(out, num_iter);
    sv_ba_initialize_optimization(g);
    stage_end(out, sv_ba_optimize(g, (int)num_iter));
    export_meta(out, &mt);
    *kf_vtx_out = kf_vtx;
    return 0;
}

int sv_bav_global_init(const sv_bav_view* view, const unsigned int* keyfrm_ids, int n_keyfrms,
                       const unsigned int* lm_ids, int n_lms, unsigned int num_iter, int use_huber,
                       double gain_threshold, int* force_stop_flag, sv_bav_result* out) {
    int i;
    memset(out, 0, sizeof(*out));
    out->returned_ok = 1;
    unsigned char* is_opt = (unsigned char*)malloc((size_t)(n_lms > 0 ? n_lms : 1));
    int* kf_vtx = NULL;
    global_impl(view, keyfrm_ids, n_keyfrms, lm_ids, n_lms, num_iter, use_huber, gain_threshold, force_stop_flag, out,
                is_opt, &kf_vtx);
    if (force_stop_flag && *force_stop_flag) {
        free(is_opt);
        free(kf_vtx);
        return 0;
    }
    /* Extract the result */
    out->applied_kf = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(n_keyfrms + 1));
    out->applied_pose = (double(*)[16])malloc(sizeof(double[16]) * (size_t)(n_keyfrms + 1));
    for (i = 0; i < n_keyfrms; ++i) {
        const int ki = find_kf(view, keyfrm_ids[i]);
        if (ki < 0 || view->kfs[ki].erased) {
            continue;
        }
        sv_bav_mat44_from_pose(&out->g.v[kf_vtx[ki]].pose, out->applied_pose[out->n_applied]);
        out->applied_kf[out->n_applied] = view->kfs[ki].id;
        out->n_applied++;
    }
    out->opt_lm = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(n_lms + 1));
    for (i = 0; i < n_lms; ++i) {
        if (!is_opt[i] || lm_ids[i] == NONE_ID) {
            continue;
        }
        const int li = find_lm(view, lm_ids[i]);
        if (li < 0 || view->lms[li].erased) {
            continue;
        }
        out->opt_lm[out->n_opt_lm++] = lm_ids[i];
    }
    free(is_opt);
    free(kf_vtx);
    return 0;
}

int sv_bav_global_loop(const sv_bav_view* view, const unsigned int* keyfrm_ids, int n_keyfrms, unsigned int num_iter,
                       int use_huber, int* force_stop_flag, sv_bav_result* out) {
    int i, j;
    memset(out, 0, sizeof(*out));
    out->returned_ok = 1;

    /* landmark list from the keyframes' slots, first occurrence order */
    sv_umap_order found;
    sv_umap_init(&found);
    unsigned int* lms = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(view->n_lms + 1));
    int n_lms = 0;
    for (i = 0; i < n_keyfrms; ++i) {
        const int ki = find_kf(view, keyfrm_ids[i]);
        if (ki < 0) {
            continue;
        }
        const sv_bav_kf* kf = &view->kfs[ki];
        for (j = 0; j < kf->n_slots; ++j) {
            if (kf->slots[j] == NONE_ID) {
                continue;
            }
            const int li = find_lm(view, kf->slots[j]);
            if (li < 0 || view->lms[li].erased) {
                continue;
            }
            if (sv_umap_contains(&found, view->lms[li].id)) {
                continue;
            }
            sv_umap_insert(&found, view->lms[li].id);
            lms[n_lms++] = view->lms[li].id;
        }
    }
    sv_umap_free(&found);

    unsigned char* is_opt = (unsigned char*)malloc((size_t)(n_lms > 0 ? n_lms : 1));
    int* kf_vtx = NULL;
    global_impl(view, keyfrm_ids, n_keyfrms, lms, n_lms, num_iter, use_huber, 1e-3, force_stop_flag, out, is_opt,
                &kf_vtx);
    if (force_stop_flag && *force_stop_flag && !out->g.stopped_by_terminate) {
        out->returned_ok = 0;
        free(is_opt);
        free(kf_vtx);
        free(lms);
        return 0;
    }
    out->applied_kf = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(n_keyfrms + 1));
    out->applied_pose = (double(*)[16])malloc(sizeof(double[16]) * (size_t)(n_keyfrms + 1));
    for (i = 0; i < n_keyfrms; ++i) {
        const int ki = find_kf(view, keyfrm_ids[i]);
        if (ki < 0 || view->kfs[ki].erased) {
            continue;
        }
        sv_bav_mat44_from_pose(&out->g.v[kf_vtx[ki]].pose, out->applied_pose[out->n_applied]);
        out->applied_kf[out->n_applied] = view->kfs[ki].id;
        out->n_applied++;
    }
    out->opt_lm = (unsigned int*)malloc(sizeof(unsigned int) * (size_t)(n_lms + 1));
    for (i = 0; i < n_lms; ++i) {
        if (!is_opt[i]) {
            continue;
        }
        out->opt_lm[out->n_opt_lm++] = lms[i];
    }
    free(is_opt);
    free(kf_vtx);
    free(lms);
    return 0;
}
