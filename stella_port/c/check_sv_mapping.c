/* SV_PORT_SOURCES: check_sv_mapping.c sv_mapping.c sv_map_match.c sv_eigen_svd.c sv_eigen_qr.c sv_rbtree.c sv_bundle_adjuster.c sv_g2o_ba.c sv_umap_order.c sv_eigen_amd.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c
 * SPDX-License-Identifier: MIT
 *
 * Harness for module 6 (mapping module): replays mapping_module::
 * mapping_with_new_keyframe() for EVERY keyframe of the deterministic
 * single-threaded reference run (fr1_xyz: 37 keyframes, fr1_desk: 59),
 * teacher forced step by step on top of check_sv_track.c's per-frame state:
 *   - pre-state of the step for keyframe k (inserted at frame t) = snapshot
 *     after frame t-1 (keyframes.tsv / landmarks.tsv) + the tracking of frame t
 *     replayed by module 5 (landmark visibility counters) + the new keyframe
 *     (sv_tr_create_new_keyframe, verified against kf_insert*.tsv);
 *   - the first step (keyframe 1, the second initial keyframe, frame 12/55)
 *     starts from snapshot(t) with the covisibility lists cleared (module 4a
 *     never fills them);
 *   - state the snapshots do not expose is carried by the C mapping context
 *     across steps: local_map_cleaner's fresh-landmark list, every keyframe's
 *     full connected_keyfrms_and_num_shared_lms_ map, landmark first-keyframe
 *     ids and the next landmark id (all rebuilt from scratch at the first step).
 * After the step the C map is compared with snapshot(t) -- every alive
 * keyframe (pose bits, ordered covisibilities + weights, spanning parent /
 * children, per-keypoint landmark ids), every alive landmark (position,
 * descriptor, mean normal, min/max valid distance, observed/observable
 * counters, reference keyframe, observation list) and the map's id sets -- and
 * the step trace with the mapping-pass dumps: culled_landmarks.tsv,
 * triangulation.tsv (matches per neighbor, accepted landmark ids + position
 * bits), fused_landmarks.tsv, culled_keyframes.tsv, local_ba.tsv (invoked).
 * Prints "<seq>: mismatches/total" (plus a per-category breakdown on stderr).
 * usage: check_sv_mapping <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (run from the repo root: reads external/candidates/orb_vocab.fbow)
 */
#define SV_MAPPING_HARNESS 1
#include "sv_mapping.h"

#define main sv_track_main
#include "check_sv_track.c"
#undef main

/* ------------------------------------------------------------------ */
/* tsv helpers                                                        */
/* ------------------------------------------------------------------ */
static char* xstrdup(const char* s) {
    size_t n = strlen(s) + 1;
    char* d = (char*)malloc(n);
    memcpy(d, s, n);
    return d;
}

typedef struct tsv {
    char** row;
    unsigned int n, cap;
} tsv;

static void tsv_load(tsv* t, const char* dir, const char* name) {
    char path[4096];
    char* line = (char*)malloc(LINE_MAX_LEN);
    FILE* f;
    memset(t, 0, sizeof(*t));
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "check_sv_mapping: cannot open %s\n", path);
        exit(2);
    }
    if (!fgets(line, LINE_MAX_LEN, f)) {
        exit(2);
    }
    while (fgets(line, LINE_MAX_LEN, f)) {
        if (t->n == t->cap) {
            t->cap = t->cap ? t->cap * 2 : 256;
            t->row = (char**)realloc(t->row, t->cap * sizeof(char*));
        }
        t->row[t->n++] = xstrdup(line);
    }
    fclose(f);
    free(line);
}

static int row_match(const char* row, long frame, long kf) {
    long a, b;
    const char* p = row;
    char* end;
    a = strtol(p, &end, 10);
    if (*end != '\t') return 0;
    b = strtol(end + 1, &end, 10);
    return a == frame && b == kf;
}

/* ------------------------------------------------------------------ */
/* hook state                                                         */
/* ------------------------------------------------------------------ */
enum { CAT_STATE_KF, CAT_STATE_LM, CAT_STATE_CONN, CAT_TRACE_CULL_LM, CAT_TRACE_TRI, CAT_TRACE_FUSE, CAT_TRACE_CULL_KF, CAT_TRACE_BA, CAT_MISC, CAT_N };
static const char* const cat_name[CAT_N] = {"state:keyframes", "state:landmarks", "state:connected_maps", "trace:culled_landmarks", "trace:triangulation",
                                            "trace:fused", "trace:culled_keyframes", "trace:local_ba_invoked", "misc"};
static unsigned long cat_total[CAT_N], cat_bad[CAT_N];

static void citem(int cat, int ok, const char* what) {
    ++cat_total[cat];
    if (!ok) ++cat_bad[cat];
    item(ok, what);
}

static sv_mapping g_map;
static int g_map_ready;
static world g_wA, g_wB; /* private worlds: first-step pre-state, post-step reference snapshot */
static block_reader g_bkA, g_blA, g_bkB, g_blB;
static long g_loadedB = -100;
static tsv g_tri, g_fused, g_cull_lm, g_cull_kf, g_lba, g_conn;
/* keyframe object lifetime schedule (kf_destroyed.tsv): frame / phase of the destruction, -1 = never */
static int* g_kf_dead_frame;
static int* g_kf_dead_phase;
static unsigned int g_kf_dead_cap;
static unsigned long g_steps, g_ba_steps, g_ties, g_dupconn, g_stale;
static unsigned long g_stat_tri_rows, g_stat_acc, g_stat_fused, g_stat_cull_lm, g_stat_cull_kf;

static void load_destroyed(const char* dir) {
    tsv t;
    unsigned int i;
    tsv_load(&t, dir, "kf_destroyed.tsv");
    g_kf_dead_cap = 4096;
    g_kf_dead_frame = (int*)malloc(g_kf_dead_cap * sizeof(int));
    g_kf_dead_phase = (int*)malloc(g_kf_dead_cap * sizeof(int));
    for (i = 0; i < g_kf_dead_cap; ++i) g_kf_dead_frame[i] = g_kf_dead_phase[i] = -1;
    for (i = 0; i < t.n; ++i) {
        int f, ph;
        unsigned int id;
        if (sscanf(t.row[i], "%d\t%d\t%u", &f, &ph, &id) == 3 && id < g_kf_dead_cap) {
            g_kf_dead_frame[id] = f;
            g_kf_dead_phase[id] = ph;
        }
    }
}

/* Lifetime of the erased keyframe objects (reference patch 0011). At the start of the mapping pass
 * of frame t the objects destroyed during frame t's tracking / pass start (phase 0, 1) and earlier
 * frames have expired; at the dump of frame t everything destroyed up to frame t has. */
static void set_expiry(long t, int at_dump) {
    unsigned int id;
    for (id = 0; id < g_kf_dead_cap; ++id) {
        const int df = g_kf_dead_frame[id], dp = g_kf_dead_phase[id];
        int exp = 0;
        if (df >= 0) exp = at_dump ? (df <= t) : (df < t || (df == t && dp <= 1));
        sv_mapping_set_expired(&g_map, id, exp);
    }
}

static sv_tr_lm* alloc_hook(void* user, unsigned int id) {
    return lm_record((world*)user, id);
}

static void init_private_world(world* dst, const world* src) {
    memset(dst, 0, sizeof(*dst));
    dst->cfg = src->cfg;
    dst->frames = src->frames;
    dst->nframes = src->nframes;
    dst->kfmeta = src->kfmeta;
    dst->kfmeta_cap = src->kfmeta_cap;
    ensure_map_capacity(dst, 64, 4096);
}

/* ------------------------------------------------------------------ */
/* comparison                                                         */
/* ------------------------------------------------------------------ */
static void compare_state(const sv_tr_map* A, const sv_tr_map* B) {
    unsigned int id, nk = A->kf_cap > B->kf_cap ? A->kf_cap : B->kf_cap;
    unsigned int nl = A->lm_cap > B->lm_cap ? A->lm_cap : B->lm_cap;
    for (id = 0; id < nk; ++id) {
        const sv_tr_kf* a = (id < A->kf_cap && A->kfs[id] && A->kfs[id]->alive) ? A->kfs[id] : NULL;
        const sv_tr_kf* b = (id < B->kf_cap && B->kfs[id] && B->kfs[id]->alive) ? B->kfs[id] : NULL;
        unsigned int i;
        if (!a && !b) continue;
        citem(CAT_STATE_KF, (a != NULL) == (b != NULL), "keyframe alive set");
        if (!a || !b) continue;
        citem(CAT_STATE_KF, same_pose(a->pose_cw, b->pose_cw), "keyframe pose_cw");
        citem(CAT_STATE_KF, memcmp(a->trans_wc, b->trans_wc, sizeof(a->trans_wc)) == 0, "keyframe trans_wc");
        {
            /* graph_node::get_covisibilities() + get_num_shared_landmarks(), as the dump reads them */
            unsigned int* cid = (unsigned int*)malloc((a->n_covis + 1) * sizeof(unsigned int));
            unsigned int* cw = (unsigned int*)malloc((a->n_covis + 1) * sizeof(unsigned int));
            const unsigned int nc = sv_mapping_covisibilities(&g_map, a, cid, cw);
            int same = nc == b->n_covis;
            for (i = 0; same && i < nc; ++i) same = cid[i] == b->covis[i] && cw[i] == b->covis_w[i];
            citem(CAT_STATE_KF, same, "keyframe ordered covisibilities");
            if (!same && getenv("SV_MAP_VERBOSE")) {
                fprintf(stderr, "  kf %u covis C:", id);
                for (i = 0; i < nc; ++i) fprintf(stderr, " %u:%u", cid[i], cw[i]);
                fprintf(stderr, "\n  kf %u covis R:", id);
                for (i = 0; i < b->n_covis; ++i) fprintf(stderr, " %u:%u", b->covis[i], b->covis_w[i]);
                fprintf(stderr, "\n");
            }
            free(cid);
            free(cw);
        }
        citem(CAT_STATE_KF, a->parent == b->parent, "keyframe spanning parent");
        {
            int same = a->n_children == b->n_children;
            for (i = 0; same && i < a->n_children; ++i) same = a->children[i] == b->children[i];
            citem(CAT_STATE_KF, same, "keyframe spanning children");
        }
        citem(CAT_STATE_KF, a->obs == b->obs && memcmp(a->lm, b->lm, a->obs->num_kp * sizeof(int)) == 0, "keyframe landmark slots");
    }
    for (id = 0; id < nl; ++id) {
        const sv_tr_lm* a = (id < A->lm_cap && A->lms[id] && A->lms[id]->alive) ? A->lms[id] : NULL;
        const sv_tr_lm* b = (id < B->lm_cap && B->lms[id] && B->lms[id]->alive) ? B->lms[id] : NULL;
        unsigned int i;
        if (!a && !b) continue;
        citem(CAT_STATE_LM, (a != NULL) == (b != NULL), "landmark alive set");
        if (!a || !b) continue;
        citem(CAT_STATE_LM, memcmp(a->pos_w, b->pos_w, sizeof(a->pos_w)) == 0, "landmark position");
        citem(CAT_STATE_LM, memcmp(a->desc, b->desc, 32) == 0, "landmark descriptor");
        citem(CAT_STATE_LM, memcmp(a->mean_normal, b->mean_normal, sizeof(a->mean_normal)) == 0, "landmark mean normal");
        citem(CAT_STATE_LM, memcmp(&a->min_valid_dist, &b->min_valid_dist, sizeof(float)) == 0 &&
                                memcmp(&a->max_valid_dist, &b->max_valid_dist, sizeof(float)) == 0, "landmark valid distances");
        citem(CAT_STATE_LM, a->num_observed == b->num_observed && a->num_observable == b->num_observable, "landmark observed/observable");
        citem(CAT_STATE_LM, a->ref_kf == b->ref_kf, "landmark reference keyframe");
        {
            int same = a->num_obs == b->num_obs;
            for (i = 0; same && i < a->num_obs; ++i) same = a->obs_kf[i] == b->obs_kf[i] && a->obs_idx[i] == b->obs_idx[i];
            citem(CAT_STATE_LM, same, "landmark observations");
        }
    }
}

/* every alive keyframe's connected_keyfrms_and_num_shared_lms_ in map order (conn.tsv, patch 0011);
 * an expired key is dumped as id -1 */
static void compare_conn(const sv_tr_map* B, long frame) {
    unsigned int id, i;
    for (id = 0; id < B->kf_cap; ++id) {
        const sv_tr_kf* b = (B->kfs[id] && B->kfs[id]->alive) ? B->kfs[id] : NULL;
        unsigned int n_ref = 0, *ids, *w, n_c;
        char* cp = NULL;
        int have = 0;
        if (!b) continue;
        for (i = 0; i < g_conn.n; ++i) {
            if (!row_match(g_conn.row[i], frame, id)) continue;
            cp = xstrdup(g_conn.row[i]);
            have = 1;
            break;
        }
        citem(CAT_STATE_CONN, have, "conn.tsv row present");
        if (!have) continue;
        {
            char* p = cp;
            const char* c;
            unsigned int cap = 64, nc;
            unsigned int* rid = (unsigned int*)malloc(cap * sizeof(unsigned int));
            unsigned int* rw = (unsigned int*)malloc(cap * sizeof(unsigned int));
            next_field(&p);
            next_field(&p);
            c = next_field(&p);
            while (*c) {
                char* end;
                long a = strtol(c, &end, 10);
                unsigned long wgt;
                if (end == c) break;
                c = end;
                if (*c == ':') ++c;
                wgt = strtoul(c, &end, 10);
                c = end;
                if (*c == ',') ++c;
                if (n_ref == cap) {
                    cap *= 2;
                    rid = (unsigned int*)realloc(rid, cap * sizeof(unsigned int));
                    rw = (unsigned int*)realloc(rw, cap * sizeof(unsigned int));
                }
                rid[n_ref] = a < 0 ? 0xFFFFFFFFu : (unsigned int)a;
                rw[n_ref] = (unsigned int)wgt;
                ++n_ref;
            }
            nc = n_ref + 64;
            ids = (unsigned int*)malloc(nc * sizeof(unsigned int));
            w = (unsigned int*)malloc(nc * sizeof(unsigned int));
            n_c = sv_mapping_dump_conn(&g_map, id, ids, w, nc);
            {
                int same = n_c == n_ref;
                for (i = 0; same && i < n_c; ++i) same = ids[i] == rid[i] && w[i] == rw[i];
                citem(CAT_STATE_CONN, same, "connected map (in order, incl. expired keys)");
                if (!same && getenv("SV_MAP_VERBOSE")) {
                    fprintf(stderr, "  kf %u conn C:", id);
                    for (i = 0; i < n_c; ++i) fprintf(stderr, " %d:%u", (int)ids[i], w[i]);
                    fprintf(stderr, "\n  kf %u conn R:", id);
                    for (i = 0; i < n_ref; ++i) fprintf(stderr, " %d:%u", (int)rid[i], rw[i]);
                    fprintf(stderr, "\n");
                }
            }
            free(ids);
            free(w);
            free(rid);
            free(rw);
        }
        free(cp);
    }
}

static unsigned int parse_uints(const char* s, unsigned int** out) {
    unsigned int n = 0, cap = 0;
    *out = NULL;
    while (*s) {
        char* end;
        unsigned long v = strtoul(s, &end, 10);
        if (end == s) break;
        s = end;
        if (*s == ',') ++s;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            *out = (unsigned int*)realloc(*out, cap * sizeof(unsigned int));
        }
        (*out)[n++] = (unsigned int)v;
    }
    return n;
}

static unsigned int parse_pairs(const char* s, unsigned int (**out)[2]) {
    unsigned int n = 0, cap = 0;
    *out = NULL;
    while (*s) {
        char* end;
        unsigned long a = strtoul(s, &end, 10), b;
        if (end == s) break;
        s = end;
        if (*s == ':') ++s;
        b = strtoul(s, &end, 10);
        s = end;
        if (*s == ',') ++s;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            *out = (unsigned int(*)[2])realloc(*out, cap * sizeof(unsigned int[2]));
        }
        (*out)[n][0] = (unsigned int)a;
        (*out)[n][1] = (unsigned int)b;
        ++n;
    }
    return n;
}

static void compare_trace(long frame, unsigned int kf_id, const sv_mapping_trace* tr) {
    unsigned int i, j, nrow;

    if (getenv("SV_MAP_VERBOSE")) {
        fprintf(stderr, "frame %ld kf %u: C culled lms:", frame, kf_id);
        for (i = 0; i < tr->n_culled_lms; ++i) fprintf(stderr, " %u", tr->culled_lms[i]);
        fprintf(stderr, "\n");
    }
    if (getenv("SV_MAP_VERBOSE")) {
        fprintf(stderr, "  C fused:");
        for (i = 0; i < tr->n_replaced; ++i) fprintf(stderr, " %u>%u", tr->replaced[i][0], tr->replaced[i][1]);
        fprintf(stderr, "\n  C tri:");
        for (i = 0; i < tr->n_tri; ++i) fprintf(stderr, " [n%u m%u a%u]", tr->tri[i].ngh_id, tr->tri[i].n_matches, tr->tri[i].n_acc);
        fprintf(stderr, "\n  C culled kfs:");
        for (i = 0; i < tr->n_culled_kfs; ++i) fprintf(stderr, " %u", tr->culled_kfs[i]);
        fprintf(stderr, "\n");
    }
    /* culled landmarks: ids in the order remove_invalid_landmarks() culled them */
    nrow = 0;
    for (i = 0; i < g_cull_lm.n; ++i) {
        if (!row_match(g_cull_lm.row[i], frame, kf_id)) continue;
        {
            char* cp = xstrdup(g_cull_lm.row[i]);
            char* p = cp;
            unsigned int id;
            next_field(&p);
            next_field(&p);
            id = (unsigned int)atol(next_field(&p));
            citem(CAT_TRACE_CULL_LM, nrow < tr->n_culled_lms && tr->culled_lms[nrow] == id, "culled landmark id/order");
            free(cp);
        }
        ++nrow;
    }
    citem(CAT_TRACE_CULL_LM, nrow == tr->n_culled_lms, "culled landmark count");
    g_stat_cull_lm += nrow;

    /* triangulation, per neighbor */
    nrow = 0;
    for (i = 0; i < g_tri.n; ++i) {
        char* cp;
        char* p;
        char* fld[9];
        unsigned int (*pairs)[2];
        unsigned int np, *ids, nid;
        const sv_mapping_tri* t;
        if (!row_match(g_tri.row[i], frame, kf_id)) continue;
        cp = xstrdup(g_tri.row[i]);
        p = cp;
        for (j = 0; j < 9; ++j) fld[j] = next_field(&p);
        t = nrow < tr->n_tri ? &tr->tri[nrow] : NULL;
        citem(CAT_TRACE_TRI, t != NULL, "triangulation neighbor row present");
        if (t) {
            citem(CAT_TRACE_TRI, t->ngh_id == (unsigned int)atol(fld[3]), "triangulation neighbor keyframe id");
            citem(CAT_TRACE_TRI, t->n_matches == (unsigned int)atol(fld[4]), "triangulation num_matches");
            np = parse_pairs(fld[5], &pairs);
            if (getenv("SV_MAP_VERBOSE") && np != t->n_matches) {
                fprintf(stderr, "  neighbor %u C matches:", t->ngh_id);
                for (j = 0; j < t->n_matches; ++j) fprintf(stderr, " %u:%u", t->matches[j][0], t->matches[j][1]);
                fprintf(stderr, "\n");
            }
            citem(CAT_TRACE_TRI, np == t->n_matches, "triangulation match count vs list");
            for (j = 0; j < np && j < t->n_matches; ++j) {
                citem(CAT_TRACE_TRI, pairs[j][0] == t->matches[j][0] && pairs[j][1] == t->matches[j][1], "triangulation match pair");
            }
            free(pairs);
            nid = parse_uints(fld[6], &ids);
            citem(CAT_TRACE_TRI, nid == t->n_acc, "triangulation accepted landmark count");
            {
                double* pos = (double*)malloc((3 * (nid ? nid : 1)) * sizeof(double));
                unsigned int got;
                {
                    char* sc;
                    for (sc = fld[8]; *sc; ++sc) if (*sc == ';') *sc = ',';
                }
                got = (unsigned int)parse_hex_list(fld[8], pos, (int)(3 * nid));
                citem(CAT_TRACE_TRI, got == 3 * nid, "triangulation accepted position count");
                for (j = 0; j < nid && j < t->n_acc; ++j) {
                    citem(CAT_TRACE_TRI, ids[j] == t->acc_ids[j], "triangulation accepted landmark id");
                    citem(CAT_TRACE_TRI, memcmp(pos + 3 * j, t->acc_pos[j], 3 * sizeof(double)) == 0, "triangulation accepted position bits");
                }
                free(pos);
            }
            free(ids);
            g_stat_acc += nid;
        }
        free(cp);
        ++nrow;
        ++g_stat_tri_rows;
    }
    citem(CAT_TRACE_TRI, nrow == tr->n_tri, "triangulation neighbor count");

    /* fused pairs (replaced away, replaced by), id ordered */
    nrow = 0;
    for (i = 0; i < g_fused.n; ++i) {
        char* cp;
        char* p;
        unsigned int a, b;
        if (!row_match(g_fused.row[i], frame, kf_id)) continue;
        cp = xstrdup(g_fused.row[i]);
        p = cp;
        next_field(&p);
        next_field(&p);
        a = (unsigned int)atol(next_field(&p));
        b = (unsigned int)atol(next_field(&p));
        citem(CAT_TRACE_FUSE, nrow < tr->n_replaced && tr->replaced[nrow][0] == a && tr->replaced[nrow][1] == b, "fused pair");
        free(cp);
        ++nrow;
    }
    citem(CAT_TRACE_FUSE, nrow == tr->n_replaced, "fused pair count");
    g_stat_fused += nrow;

    /* culled keyframes */
    nrow = 0;
    for (i = 0; i < g_cull_kf.n; ++i) {
        char* cp;
        char* p;
        unsigned int id;
        if (!row_match(g_cull_kf.row[i], frame, kf_id)) continue;
        cp = xstrdup(g_cull_kf.row[i]);
        p = cp;
        next_field(&p);
        next_field(&p);
        id = (unsigned int)atol(next_field(&p));
        citem(CAT_TRACE_CULL_KF, nrow < tr->n_culled_kfs && tr->culled_kfs[nrow] == id, "culled keyframe id/order");
        free(cp);
        ++nrow;
    }
    citem(CAT_TRACE_CULL_KF, nrow == tr->n_culled_kfs, "culled keyframe count");
    g_stat_cull_kf += nrow;

    /* local BA invoked */
    nrow = 0;
    for (i = 0; i < g_lba.n; ++i) {
        char* cp;
        char* p;
        int inv;
        if (!row_match(g_lba.row[i], frame, kf_id)) continue;
        cp = xstrdup(g_lba.row[i]);
        p = cp;
        next_field(&p);
        next_field(&p);
        inv = atoi(next_field(&p));
        citem(CAT_TRACE_BA, inv == tr->local_ba_invoked, "local BA invoked");
        free(cp);
        ++nrow;
    }
    citem(CAT_TRACE_BA, nrow == 1, "local_ba.tsv row present");
    g_ba_steps += tr->local_ba_invoked;
    g_ties += tr->n_span_ties;
    g_dupconn += tr->n_dup_connect;
    g_stale += tr->n_stale_neighbor;
}

/* runs the mapping step for `kf_id` (inserted at frame t) on `pre` and compares */
static void run_step(world* pre, unsigned int t, unsigned int kf_id) {
    sv_mapping_trace tr;
    int rc;
    g_cur_frame = t;
    g_map.map = &pre->map;
    g_map.alloc_user = pre;
    set_expiry((long)t, 0);
    rc = sv_mapping_step(&g_map, kf_id, &tr);
    set_expiry((long)t, 1);
    citem(CAT_MISC, rc == 0, "sv_mapping_step succeeded");
    if (rc != 0) return;
    if (g_loadedB != (long)t) {
        if (load_snapshot(&g_wB, &g_bkB, &g_blB, (long)t) != 0) {
            fprintf(stderr, "check_sv_mapping: no snapshot for frame %u\n", t);
            exit(2);
        }
        g_loadedB = (long)t;
    }
    compare_state(&pre->map, &g_wB.map);
    compare_conn(&g_wB.map, (long)t);
    compare_trace((long)t, kf_id, &tr);
    ++g_steps;
    sv_mapping_trace_free(&tr);
}

static void mapping_hook_begin(world* w, const pre_row* pre, unsigned int nframes, const char* dump_dir) {
    char path[4096];
    FILE *fk, *fl, *fk2, *fl2;
    long f0;
    unsigned int id, max_lm = 0;
    (void)pre;
    (void)nframes;

    tsv_load(&g_tri, dump_dir, "triangulation.tsv");
    tsv_load(&g_fused, dump_dir, "fused_landmarks.tsv");
    tsv_load(&g_cull_lm, dump_dir, "culled_landmarks.tsv");
    tsv_load(&g_cull_kf, dump_dir, "culled_keyframes.tsv");
    tsv_load(&g_lba, dump_dir, "local_ba.tsv");
    tsv_load(&g_conn, dump_dir, "conn.tsv");
    load_destroyed(dump_dir);

    init_private_world(&g_wA, w);
    init_private_world(&g_wB, w);
    snprintf(path, sizeof(path), "%s/keyframes.tsv", dump_dir);
    fk = fopen(path, "r");
    fk2 = fopen(path, "r");
    snprintf(path, sizeof(path), "%s/landmarks.tsv", dump_dir);
    fl = fopen(path, "r");
    fl2 = fopen(path, "r");
    if (!fk || !fk2 || !fl || !fl2) {
        fprintf(stderr, "check_sv_mapping: cannot open snapshots in %s\n", dump_dir);
        exit(2);
    }
    br_open(&g_bkA, fk);
    br_open(&g_blA, fl);
    br_open(&g_bkB, fk2);
    br_open(&g_blB, fl2);

    br_fill(&g_bkA);
    f0 = g_bkA.frame;

    /* first mapping step: keyframe 1 (second initial keyframe) at the first snapshot frame */
    if (load_snapshot(&g_wA, &g_bkA, &g_blA, f0) != 0) {
        fprintf(stderr, "check_sv_mapping: no first snapshot\n");
        exit(2);
    }
    for (id = 0; id < g_wA.map.kf_cap; ++id) {
        sv_tr_kf* k = g_wA.map.kfs[id];
        if (k) k->n_covis = 0; /* create_map_for_monocular never fills the covisibility graph */
    }
    /* snapshot(f0) is the state AFTER both initial passes. The pass over keyframe 0 triangulates
     * landmarks between keyframe 0 and keyframe 1 (reference keyframe = keyframe 0) and the pass over
     * keyframe 1 may add more; ids continue after the initializer's landmarks. Drop everything from
     * the first id created by a mapping pass on (smallest id with reference keyframe 0, or the first
     * id accepted by the triangulation row of the keyframe-1 pass) so that both passes are replayed
     * for real. */
    {
        unsigned int n0 = 0xFFFFFFFFu, i2;
        for (id = 0; id < g_wA.map.lm_cap; ++id) {
            const sv_tr_lm* lm = g_wA.map.lms[id];
            if (lm && lm->alive && lm->ref_kf == 0 && id < n0) n0 = id;
            if (lm && lm->alive && id + 1 > max_lm) max_lm = id + 1;
        }
        for (i2 = 0; i2 < g_tri.n; ++i2) {
            char* cp;
            char* p2;
            char* fld[9];
            unsigned int j2, *ids2, nid2;
            if (!row_match(g_tri.row[i2], f0, 1)) continue;
            cp = xstrdup(g_tri.row[i2]);
            p2 = cp;
            for (j2 = 0; j2 < 9; ++j2) fld[j2] = next_field(&p2);
            nid2 = parse_uints(fld[6], &ids2);
            for (j2 = 0; j2 < nid2; ++j2) if (ids2[j2] < n0) n0 = ids2[j2];
            free(ids2);
            free(cp);
        }
        max_lm = 0;
        for (id = 0; id < g_wA.map.lm_cap; ++id) {
            sv_tr_lm* lm = g_wA.map.lms[id];
            unsigned int o;
            if (!lm || !lm->alive) continue;
            if (id >= n0) {
                for (o = 0; o < lm->num_obs; ++o) {
                    sv_tr_kf* k = g_wA.map.kfs[lm->obs_kf[o]];
                    if (k) k->lm[lm->obs_idx[o]] = SV_TR_NONE;
                }
                lm->alive = 0;
                g_wA.map.lms[id] = NULL;
            }
            else if (id + 1 > max_lm) {
                max_lm = id + 1;
            }
        }
    }
    sv_mapping_init(&g_map, &w->cfg, &g_wA.map);
    g_map.alloc_lm = alloc_hook;
    g_map.alloc_user = &g_wA;
    for (id = 0; id < g_wA.map.lm_cap; ++id) {
        if (g_wA.map.lms[id] && g_wA.map.lms[id]->alive) {
            sv_mapping_set_lm_first_kf(&g_map, id, 1); /* initial landmarks: landmark(id, pos, curr_keyfrm) */
        }
    }
    g_map.next_landmark_id = max_lm;
    g_map_ready = 1;
    /* tracking_module::initialize() passes ALL initial keyframes to the mapping module in
     * spanning-tree BFS order (get_keyframes_from_root(): keyframe 0, then keyframe 1); the
     * reference dumps only the last pass (keyframe 1), the first one is replayed unchecked
     * (its effects -- fresh landmark list, covisibility graph -- are checked through the
     * next steps and snapshot(f0)) */
    {
        sv_mapping_trace t0;
        g_cur_frame = f0;
        set_expiry(f0, 0);
        g_map.map = &g_wA.map;
        g_map.alloc_user = &g_wA;
        citem(CAT_MISC, sv_mapping_step(&g_map, 0, &t0) == 0, "sv_mapping_step (initial keyframe 0)");
        sv_mapping_trace_free(&t0);
    }
    run_step(&g_wA, (unsigned int)f0, 1);
    /* the private pre-state world holds the post-state of this step; the main world
     * continues from snapshots, the mapping context carries the hidden state */
}

static void mapping_hook_after_insert(world* w, unsigned int t, int inserted_id) {
    if (!g_map_ready) return;
    /* register records the library allocated through the hook with the main world's pools */
    run_step(w, t, (unsigned int)inserted_id);
}

static void mapping_hook_end(void) {
    unsigned int c;
    fprintf(stderr, "check_sv_mapping: mapping steps=%lu (local BA invoked in %lu); triangulation neighbor rows=%lu accepted landmarks=%lu; "
                    "fused pairs=%lu culled landmarks=%lu culled keyframes=%lu; recover_spanning ties=%lu, duplicate connects=%lu, stale neighbours=%lu\n",
            g_steps, g_ba_steps, g_stat_tri_rows, g_stat_acc, g_stat_fused, g_stat_cull_lm, g_stat_cull_kf, g_ties, g_dupconn, g_stale);
    for (c = 0; c < CAT_N; ++c) {
        fprintf(stderr, "  %-26s %lu/%lu\n", cat_name[c], cat_bad[c], cat_total[c]);
    }
    sv_mapping_free(&g_map);
}

int main(int argc, char** argv) {
    return sv_track_main(argc, argv);
}
