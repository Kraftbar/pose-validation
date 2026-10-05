/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 6: ViSlamBackend. See ok_vslam.h for the notices and the backend entry-record layouts.
 * Every function mirrors the C++ method of the same name; every call into one of the two graphs is reported to
 * hooks.trace in the layout of the patch-0010 record (arguments and results, pointer fields zeroed). */
#include "ok_vslam.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------------------------
 * containers
 * ---------------------------------------------------------------------------------------------------------------- */
static void idset_clear(ok_idset* s) { s->n = 0; }
static void idset_copy(ok_idset* d, const ok_idset* s) {
    if (d->cap < s->n) { d->cap = s->n + 8; d->a = (uint64_t*)realloc(d->a, sizeof(uint64_t) * (size_t)d->cap); }
    if (s->n) memcpy(d->a, s->a, sizeof(uint64_t) * (size_t)s->n);
    d->n = s->n;
}
static void idset_insert_all(ok_idset* d, const ok_idset* s) { int i; for (i = 0; i < s->n; ++i) ok_idset_add(d, s->a[i]); }

/* byte buffer */
typedef struct bb { unsigned char* p; size_t n, cap; } bb;
static void bb_raw(bb* b, const void* d, size_t n) {
    if (b->n + n > b->cap) { b->cap = (b->n + n) * 2 + 256; b->p = (unsigned char*)realloc(b->p, b->cap); }
    if (n) memcpy(b->p + b->n, d, n);
    b->n += n;
}
static void bb_u32(bb* b, uint32_t v) { bb_raw(b, &v, 4); }
static void bb_u64(bb* b, uint64_t v) { bb_raw(b, &v, 8); }
static void bb_f64(bb* b, double v) { bb_raw(b, &v, 8); }
static void bb_f64n(bb* b, const double* v, size_t n) { bb_raw(b, v, 8 * n); }
static void bb_time(bb* b, ok_time t) { bb_u32(b, t.sec); bb_u32(b, t.nsec); }
static void bb_kid(bb* b, ok_vg_kid k) { bb_u64(b, k.frame); bb_u32(b, k.cam); bb_u32(b, k.kp); }
static void bb_meas(bb* b, const ok_imu_meas* m, size_t n) {
    size_t i;
    bb_u64(b, n);
    for (i = 0; i < n; ++i) { bb_time(b, m[i].t); bb_f64n(b, m[i].gyr, 3); bb_f64n(b, m[i].acc, 3); }
}
static void bb_free(bb* b) { free(b->p); b->p = NULL; b->n = b->cap = 0; }

/* ------------------------------------------------------------------------------------------------------------------
 * state
 * ---------------------------------------------------------------------------------------------------------------- */
typedef ok_vsb_cam_view vcam;
typedef ok_vsb_frame_view vframe;
typedef struct aux {           /* AuxiliaryState */
    int alive, is_kf, is_imu, is_pg, closed_loop, is_prf;
    uint64_t loop_id;
    ok_idset recent;
} aux;
typedef struct backlog { ok_time t; uint64_t id; ok_imu_meas* meas; size_t n; } backlog;
typedef struct relinfo { double T[7]; double info[36]; uint64_t pi, pj; } relinfo;
typedef struct elimpair { uint64_t id, ref; } elimpair;

struct ok_vsb {
    ok_vsb_hooks h;
    ok_vg* g[2];                               /* 0 realtime, 1 full */
    int ncam_graph;
    unsigned char* cam_header[OK_VSB_MAXCAM]; size_t cam_hlen[OK_VSB_MAXCAM]; ok_cam cam_model[OK_VSB_MAXCAM];
    double kptradius;
    vframe* frames; int nframes_alloc; int nframes_alive;      /* indexed by state id */
    aux* aux; int naux_alloc;                                   /* indexed by state id */
    ok_idset aux_ids;                                           /* ascending ids of the live auxiliary states */
    ok_idset imu_frames, key_frames, lc_frames, cur_lc_frames, touched_states, touched_landmarks, updated_lc_attempt;
    elimpair* elim; int nelim, capelim;                         /* eliminateStates_ (map, ascending id) */
    backlog* bl; int nbl, capbl;
    relinfo* rel; int nrel, caprel;
    uint64_t last_freeze;
    int needs_full, is_loop_closing, is_loop_closure_available;
};

#define NOW_LOOP(b) ((b)->is_loop_closing || (b)->is_loop_closure_available)

static void tr(ok_vsb* b, int graph, int op, const bb* a, const bb* r) {
    if (b->h.trace) b->h.trace(b->h.ctx, graph, op, a ? a->p : NULL, a ? a->n : 0, r ? r->p : NULL, r ? r->n : 0);
}

/* ---- frame / aux tables ---- */
static vframe* fr_get(ok_vsb* b, uint64_t id) { return (id < (uint64_t)b->nframes_alloc && b->frames[id].alive) ? &b->frames[id] : NULL; }
static vframe* fr_make(ok_vsb* b, uint64_t id) {
    if (id >= (uint64_t)b->nframes_alloc) {
        const int n = (int)id * 2 + 16;
        b->frames = (vframe*)realloc(b->frames, sizeof(vframe) * (size_t)n);
        memset(b->frames + b->nframes_alloc, 0, sizeof(vframe) * (size_t)(n - b->nframes_alloc));
        b->nframes_alloc = n;
    }
    return &b->frames[id];
}
static void fr_erase(ok_vsb* b, uint64_t id) {
    vframe* f = fr_get(b, id);
    int c;
    if (!f) return;
    for (c = 0; c < f->ncam; ++c) { free(f->cam[c].kp); free(f->cam[c].lm); free(f->cam[c].lmhp); free(f->cam[c].lminit); }
    memset(f, 0, sizeof *f);
    b->nframes_alive--;
}
static aux* ax_get(ok_vsb* b, uint64_t id) { return (id < (uint64_t)b->naux_alloc && b->aux[id].alive) ? &b->aux[id] : NULL; }
static aux* ax_make(ok_vsb* b, uint64_t id) {
    if (id >= (uint64_t)b->naux_alloc) {
        const int n = (int)id * 2 + 16;
        b->aux = (aux*)realloc(b->aux, sizeof(aux) * (size_t)n);
        memset(b->aux + b->naux_alloc, 0, sizeof(aux) * (size_t)(n - b->naux_alloc));
        b->naux_alloc = n;
    }
    ok_idset_free(&b->aux[id].recent);
    memset(&b->aux[id], 0, sizeof(aux));
    b->aux[id].alive = 1;
    b->aux[id].is_prf = 1;                      /* isPlaceRecognitionFrame defaults to true */
    ok_idset_add(&b->aux_ids, id);
    return &b->aux[id];
}
static void ax_erase(ok_vsb* b, uint64_t id) {
    aux* a = ax_get(b, id);
    if (!a) return;
    ok_idset_free(&a->recent);
    memset(a, 0, sizeof *a);
    ok_idset_del(&b->aux_ids, id);
}

/* ---- the graph-call wrappers: do the call on the C graph, then report it ---- */
static ok_vg* G(ok_vsb* b, int which) { return b->g[which]; }
static void a_id(bb* a, uint64_t id) { bb_u64(a, id); }
static uint64_t cur_state(ok_vsb* b) {           /* currentStateId() = realtimeGraph_.currentStateId() */
    const int n = ok_vg_state_count(b->g[0]);
    ok_vg_state_view v;
    if (n == 0) return 0;
    ok_vg_state_at(b->g[0], n - 1, &v);
    return v.id;
}

static void w_set_keyframe(ok_vsb* b, int w, uint64_t id, int flag) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_set_keyframe(G(b, w), id, flag);
    a_id(&a, id); bb_u32(&a, flag ? 1u : 0u);
    tr(b, w, OK_M_SETKF, &a, NULL); bb_free(&a);
}
static void w_add_landmark_id(ok_vsb* b, int w, uint64_t id, const double hp[4], int init) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_add_landmark_id(G(b, w), id, hp, init);
    a_id(&a, id); bb_f64n(&a, hp, 4); bb_u32(&a, init ? 1u : 0u);
    tr(b, w, OK_M_ADDLM_ID, &a, NULL); bb_free(&a);
}
static void w_set_landmark_full(ok_vsb* b, int w, uint64_t id, const double hp[4], int init) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_set_landmark(G(b, w), id, hp, 1, init);
    a_id(&a, id); bb_f64n(&a, hp, 4); bb_u32(&a, init ? 1u : 0u);
    tr(b, w, OK_M_SETLM_FULL, &a, NULL); bb_free(&a);
}
static void w_set_landmark_quality(ok_vsb* b, int w, uint64_t id, double q) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_set_landmark_quality(G(b, w), id, q);
    a_id(&a, id); bb_f64(&a, q);
    tr(b, w, OK_M_SETLMQ, &a, NULL); bb_free(&a);
}
static void w_remove_landmark(ok_vsb* b, int w, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_remove_landmark(G(b, w), id);
    a_id(&a, id);
    tr(b, w, OK_M_RMLM, &a, NULL); bb_free(&a);
}
static void w_set_pose(ok_vsb* b, int w, uint64_t id, const double T7[7]) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_set_pose(G(b, w), id, T7);
    a_id(&a, id); bb_f64n(&a, T7, 7);
    tr(b, w, OK_M_SETPOSE, &a, NULL); bb_free(&a);
}
static void w_set_sb(ok_vsb* b, int w, uint64_t id, const double sb[9]) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_set_speed_and_bias(G(b, w), id, sb);
    a_id(&a, id); bb_f64n(&a, sb, 9);
    tr(b, w, OK_M_SETSB, &a, NULL); bb_free(&a);
}
static void w_remove_observation(ok_vsb* b, int w, ok_vg_kid kid) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_remove_observation(G(b, w), kid);
    bb_kid(&a, kid);
    tr(b, w, OK_M_RMOBS, &a, NULL); bb_free(&a);
}
static void w_remove_all_observations(ok_vsb* b, int w, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_remove_all_observations(G(b, w), id);
    a_id(&a, id);
    tr(b, w, OK_M_RMALLOBS, &a, NULL); bb_free(&a);
}
static void w_add_external_observation(ok_vsb* b, int w, uint64_t lm, ok_vg_kid kid, int uc, const ok_reproj_err* src) {
    bb a; unsigned char* pl; size_t n;
    memset(&a, 0, sizeof a);
    ok_vg_add_external_observation(G(b, w), lm, kid, uc, src);
    a_id(&a, lm); bb_kid(&a, kid); bb_u32(&a, uc ? 1u : 0u); bb_u64(&a, 0);
    n = ok_vg_reproj_payload(src, &pl); bb_raw(&a, pl, n); free(pl);
    tr(b, w, OK_M_ADDEXTOBS, &a, NULL); bb_free(&a);
}
static void w_covis(ok_vsb* b, int w) {              /* computeCovisibilities(): a record every call (empty result when cached) */
    ok_vg* g = G(b, w);
    const int was_dirty = ok_vg_covisibilities_dirty(g);
    bb r; memset(&r, 0, sizeof r);
    ok_vg_compute_covisibilities(g);
    if (was_dirty) {
        const uint64_t (*ab)[2]; const int* cn; const uint64_t* vis;
        const int np = ok_vg_covis_pairs(g, &ab, &cn), nv = ok_vg_visible_frames(g, &vis);
        int i, j;
        bb_u32(&r, 1u); bb_u32(&r, (uint32_t)ok_vg_covis_size(g));
        for (i = 0; i < np;) {
            j = i;
            while (j < np && ab[j][0] == ab[i][0]) ++j;
            bb_u64(&r, ab[i][0]); bb_u32(&r, (uint32_t)(j - i));
            { int k; for (k = i; k < j; ++k) { bb_u64(&r, ab[k][1]); bb_u32(&r, (uint32_t)cn[k]); } }
            i = j;
        }
        bb_u32(&r, (uint32_t)nv);
        for (i = 0; i < nv; ++i) bb_u64(&r, vis[i]);
    }
    tr(b, w, OK_M_COVIS, NULL, &r); bb_free(&r);
}
static void w_freeze_poses(ok_vsb* b, int w, uint64_t id, int rm) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_freeze_poses_until(G(b, w), id, rm);
    a_id(&a, id); bb_u32(&a, rm ? 1u : 0u);
    tr(b, w, OK_M_FREEZE_POSES, &a, NULL); bb_free(&a);
}
static void w_freeze_sb(ok_vsb* b, int w, uint64_t id, int rm) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_freeze_sb_until(G(b, w), id, rm);
    a_id(&a, id); bb_u32(&a, rm ? 1u : 0u);
    tr(b, w, OK_M_FREEZE_SB, &a, NULL); bb_free(&a);
}
static void w_unfreeze_poses(ok_vsb* b, int w, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_unfreeze_poses_from(G(b, w), id);
    a_id(&a, id);
    tr(b, w, OK_M_UNFREEZE_POSES, &a, NULL); bb_free(&a);
}
static void w_unfreeze_sb(ok_vsb* b, int w, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_unfreeze_sb_from(G(b, w), id);
    a_id(&a, id);
    tr(b, w, OK_M_UNFREEZE_SB, &a, NULL); bb_free(&a);
}
static void w_poke(ok_vsb* b, int w, int op, const bb* a) { tr(b, w, op, a, NULL); }

/* a multiframe's landmark id write that the backend itself does (the frontend's go through ok_vsb_set_landmark_id) */
static void fr_set_lm(ok_vsb* b, uint64_t frame, uint32_t cam, uint32_t kp, uint64_t id) {
    vframe* f = fr_get(b, frame);
    if (f && (int)cam < f->ncam && (int)kp < f->cam[cam].nkp) f->cam[cam].lm[kp] = id;
}
static void ml_trace(ok_vsb* b, uint64_t frame, uint32_t cam, uint32_t kp, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    bb_u64(&a, frame); bb_u32(&a, cam); bb_u32(&a, kp); bb_u64(&a, id);
    tr(b, -1, OK_B_ML, &a, NULL); bb_free(&a);
    fr_set_lm(b, frame, cam, kp, id);
}
void ok_vsb_set_landmark_id(ok_vsb* b, uint64_t frame, uint32_t cam, uint32_t kp, uint64_t id) { fr_set_lm(b, frame, cam, kp, id); }

/* ViGraphEstimator::mergeLandmark(from, into, multiFrames): the multiframe writes, then the graph record */
static void w_merge_landmark(ok_vsb* b, int w, uint64_t from, uint64_t into) {
    ok_vg* g = G(b, w);
    ok_vg_kid* kids; int n, i;
    bb a; memset(&a, 0, sizeof a);
    if (!ok_vg_landmark_find(g, from, NULL) || !ok_vg_landmark_find(g, into, NULL)) return;
    n = ok_vg_landmark_obs(g, from, &kids);
    for (i = 0; i < n; ++i) ml_trace(b, kids[i].frame, kids[i].cam, kids[i].kp, into);
    free(kids);
    ok_vg_merge_landmark(g, from, into);
    a_id(&a, from); a_id(&a, into);
    tr(b, w, OK_M_MERGELM, &a, NULL); bb_free(&a);
}

/* ------------------------------------------------------------------------------------------------------------------
 * construction
 * ---------------------------------------------------------------------------------------------------------------- */
ok_vsb* ok_vsb_new(const ok_vsb_hooks* h) {
    ok_vsb* b = (ok_vsb*)calloc(1, sizeof(ok_vsb));
    b->h = *h;
    b->g[0] = ok_vg_new(); b->g[1] = ok_vg_new();
    b->kptradius = 0.09;
    return b;
}
static void reset_state(ok_vsb* b) {
    int i;
    for (i = 0; i < b->nframes_alloc; ++i) if (b->frames[i].alive) fr_erase(b, (uint64_t)i);
    for (i = 0; i < b->naux_alloc; ++i) if (b->aux[i].alive) ax_erase(b, (uint64_t)i);
    b->nframes_alive = 0;
    idset_clear(&b->imu_frames); idset_clear(&b->key_frames); idset_clear(&b->lc_frames); idset_clear(&b->cur_lc_frames);
    idset_clear(&b->touched_states); idset_clear(&b->touched_landmarks); idset_clear(&b->updated_lc_attempt);
    for (i = 0; i < b->nbl; ++i) free(b->bl[i].meas);
    b->nbl = 0; b->nelim = 0; b->nrel = 0; b->last_freeze = 0;
    b->needs_full = b->is_loop_closing = b->is_loop_closure_available = 0;
}
void ok_vsb_free(ok_vsb* b) {
    int i;
    if (!b) return;
    reset_state(b);
    ok_vg_free(b->g[0]); ok_vg_free(b->g[1]);
    for (i = 0; i < OK_VSB_MAXCAM; ++i) free(b->cam_header[i]);
    free(b->frames); free(b->aux); free(b->elim); free(b->bl); free(b->rel);
    ok_idset_free(&b->aux_ids); ok_idset_free(&b->imu_frames); ok_idset_free(&b->key_frames); ok_idset_free(&b->lc_frames);
    ok_idset_free(&b->cur_lc_frames); ok_idset_free(&b->touched_states); ok_idset_free(&b->touched_landmarks); ok_idset_free(&b->updated_lc_attempt);
    free(b);
}
ok_vg* ok_vsb_graph(ok_vsb* b, int which) { return b->g[which]; }

int ok_vsb_add_camera(ok_vsb* b, int do_ext, double sr, double sa) {
    bb a; int w;
    for (w = 1; w >= 0; --w) {                     /* fullGraph_ first, then realtimeGraph_ */
        memset(&a, 0, sizeof a);
        ok_vg_add_camera(b->g[w], do_ext, sr, sa);
        bb_u32(&a, do_ext ? 1u : 0u); bb_f64(&a, sr); bb_f64(&a, sa);
        tr(b, w, OK_M_ADDCAM, &a, NULL); bb_free(&a);
    }
    return ok_vg_num_cameras(b->g[0]) - 1;
}
int ok_vsb_add_imu(ok_vsb* b, const ok_vg_imu_cfg* c) {
    bb a; int w;
    for (w = 1; w >= 0; --w) {
        memset(&a, 0, sizeof a);
        ok_vg_add_imu(b->g[w], c);
        bb_u32(&a, (uint32_t)c->use); bb_f64n(&a, c->T_BS, 7);
        bb_f64(&a, c->a_max); bb_f64(&a, c->g_max); bb_f64(&a, c->sigma_g_c); bb_f64(&a, c->sigma_bg);
        bb_f64(&a, c->sigma_a_c); bb_f64(&a, c->sigma_ba); bb_f64(&a, c->sigma_gw_c); bb_f64(&a, c->sigma_aw_c);
        bb_f64n(&a, c->g0, 3); bb_f64n(&a, c->a0, 3); bb_f64(&a, c->g);
        tr(b, w, OK_M_ADDIMU, &a, NULL); bb_free(&a);
    }
    return 0;
}

/* parse the camera model blob (ok_cam.h header as logged) */
static int parse_cam_header(const unsigned char* p, size_t n, ok_cam* cam) {
    uint32_t tag, w, h, nd, i; double f[4], d[OK_CAM_MAX_DIST];
    size_t off = 0;
    memset(d, 0, sizeof d);
    if (n < 4 * 3 + 32 + 4) return 0;
    memcpy(&tag, p + off, 4); off += 4; memcpy(&w, p + off, 4); off += 4; memcpy(&h, p + off, 4); off += 4;
    memcpy(f, p + off, 32); off += 32;
    memcpy(&nd, p + off, 4); off += 4;
    if (nd > OK_CAM_MAX_DIST || off + 8 * (size_t)nd > n) return 0;
    for (i = 0; i < nd; ++i) { memcpy(&d[i], p + off, 8); off += 8; }
    if (tag != OK_CAM_RADTAN && tag != OK_CAM_EQUIDISTANT && tag != OK_CAM_NODIST) return 0;
    ok_cam_init(cam, (int)tag, (int)w, (int)h, f[0], f[1], f[2], f[3], d);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * addStates
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_add_states(ok_vsb* b, ok_time t, const ok_imu_meas* meas, size_t n, int as_kf, double kptradius, int ncam, const ok_vsb_cam_in* cams) {
    uint64_t id;
    vframe* f;
    aux* ax;
    int c, k;
    double T_SC[OK_VSB_MAXCAM][7];
    bb a, r;
    b->kptradius = kptradius;
    for (c = 0; c < ncam && c < OK_VSB_MAXCAM; ++c) {
        if (!b->cam_header[c]) {
            b->cam_header[c] = (unsigned char*)malloc(cams[c].hlen ? cams[c].hlen : 1);
            memcpy(b->cam_header[c], cams[c].header, cams[c].hlen); b->cam_hlen[c] = cams[c].hlen;
            parse_cam_header(cams[c].header, cams[c].hlen, &b->cam_model[c]);
        }
        memcpy(T_SC[c], cams[c].T_SC, sizeof(double) * 7);
    }
    if (b->nframes_alive == 0) {
        double pose[7], sb[9];
        int w;
        for (w = 0; w < 2; ++w) {                   /* realtime first, then full */
            memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
            id = ok_vg_add_states_initialise(b->g[w], t, meas, n, ncam, (const double(*)[7])T_SC);
            bb_time(&a, t); bb_meas(&a, meas, n); bb_u32(&a, (uint32_t)ncam);
            for (c = 0; c < ncam; ++c) bb_f64n(&a, T_SC[c], 7);
            ok_vg_pose_values(b->g[w], id, pose); ok_vg_sb_values(b->g[w], id, sb);
            bb_u64(&r, id); bb_f64n(&r, pose, 7); bb_f64n(&r, sb, 9);
            tr(b, w, OK_M_INIT, &a, &r); bb_free(&a); bb_free(&r);
        }
    } else {
        double pose[7], sb[9];
        memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
        id = ok_vg_add_states_propagate(b->g[0], t, meas, n, as_kf);
        bb_time(&a, t); bb_meas(&a, meas, n); bb_u32(&a, as_kf ? 1u : 0u);
        ok_vg_pose_values(b->g[0], id, pose); ok_vg_sb_values(b->g[0], id, sb);
        bb_u64(&r, id); bb_f64n(&r, pose, 7); bb_f64n(&r, sb, 9);
        tr(b, 0, OK_M_PROP, &a, &r); bb_free(&a); bb_free(&r);
        if (NOW_LOOP(b)) {
            backlog bk;
            bk.t = t; bk.id = id; bk.n = n; bk.meas = (ok_imu_meas*)malloc(sizeof(ok_imu_meas) * (n ? n : 1));
            if (n) memcpy(bk.meas, meas, sizeof(ok_imu_meas) * n);
            if (b->nbl == b->capbl) { b->capbl = b->capbl ? 2 * b->capbl : 8; b->bl = (backlog*)realloc(b->bl, sizeof(backlog) * (size_t)b->capbl); }
            b->bl[b->nbl++] = bk;
            ok_idset_add(&b->touched_states, id);
        } else {
            memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
            { const uint64_t id2 = ok_vg_add_states_propagate(b->g[1], t, meas, n, as_kf);
              bb_time(&a, t); bb_meas(&a, meas, n); bb_u32(&a, as_kf ? 1u : 0u);
              ok_vg_pose_values(b->g[1], id2, pose); ok_vg_sb_values(b->g[1], id2, sb);
              bb_u64(&r, id2); bb_f64n(&r, pose, 7); bb_f64n(&r, sb, 9); }
            tr(b, 1, OK_M_PROP, &a, &r); bb_free(&a); bb_free(&r);
        }
    }
    f = fr_make(b, id);
    memset(f, 0, sizeof *f);
    f->alive = 1; f->ncam = ncam; b->nframes_alive++;
    for (c = 0; c < ncam; ++c) {
        f->cam[c].rows = cams[c].rows; f->cam[c].cols = cams[c].cols; f->cam[c].nkp = cams[c].nkp;
        memcpy(f->cam[c].T_SC, cams[c].T_SC, sizeof(double) * 7);
        f->cam[c].kp = (float*)malloc(sizeof(float) * 3 * (size_t)(cams[c].nkp ? cams[c].nkp : 1));
        if (cams[c].nkp) memcpy(f->cam[c].kp, cams[c].kp, sizeof(float) * 3 * (size_t)cams[c].nkp);
        f->cam[c].lm = (uint64_t*)calloc((size_t)(cams[c].nkp ? cams[c].nkp : 1), sizeof(uint64_t));
        f->cam[c].lmhp = (double*)calloc(4 * (size_t)(cams[c].nkp ? cams[c].nkp : 1), sizeof(double));
        f->cam[c].lminit = (unsigned char*)calloc((size_t)(cams[c].nkp ? cams[c].nkp : 1), 1);
        for (k = 0; k < cams[c].nz; ++k) if ((int)cams[c].nz_kp[k] < cams[c].nkp) f->cam[c].lm[cams[c].nz_kp[k]] = cams[c].nz_id[k];
    }
    ax = ax_make(b, id);
    ax->is_imu = 1; ax->is_kf = as_kf; ax->loop_id = id;
    ok_idset_add(&b->imu_frames, id);
    return 1;
}

int ok_vsb_set_keyframe(ok_vsb* b, uint64_t id, int flag) {
    aux* ax = ax_get(b, id);
    if (ax) ax->is_kf = flag;
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_states, id);
    else w_set_keyframe(b, 1, id, flag);
    w_set_keyframe(b, 0, id, flag);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * landmarks and observations
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_add_landmark_id(ok_vsb* b, uint64_t id, const double hp[4], int init) {
    w_add_landmark_id(b, 0, id, hp, init);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_landmarks, id);
    else w_add_landmark_id(b, 1, id, hp, init);
    return 1;
}
uint64_t ok_vsb_add_landmark(ok_vsb* b, const double hp[4], int init) {
    bb a, r;
    uint64_t id;
    memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
    id = ok_vg_add_landmark(b->g[0], hp, init);
    bb_f64n(&a, hp, 4); bb_u32(&a, init ? 1u : 0u); bb_u64(&r, id);
    tr(b, 0, OK_M_ADDLM_NEW, &a, &r); bb_free(&a); bb_free(&r);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_landmarks, id);
    else w_add_landmark_id(b, 1, id, hp, init);
    return id;
}
int ok_vsb_set_landmark(ok_vsb* b, uint64_t id, const double hp[4], int init) {
    w_set_landmark_full(b, 0, id, hp, init);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_landmarks, id);
    else w_set_landmark_full(b, 1, id, hp, init);
    return 1;
}
int ok_vsb_set_landmark_classification(ok_vsb* b, uint64_t id, int cls) {
    bb a;
    if (!ok_vg_landmark_find(b->g[0], id, NULL)) return 0;
    memset(&a, 0, sizeof a);
    ok_vg_set_landmark_classification(b->g[0], id, cls);
    a_id(&a, id); bb_u32(&a, (uint32_t)cls);
    tr(b, 0, OK_M_SETCLASS, &a, NULL); bb_free(&a);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_landmarks, id);
    else {
        memset(&a, 0, sizeof a);
        ok_vg_set_landmark_classification(b->g[1], id, cls);
        a_id(&a, id); bb_u32(&a, (uint32_t)cls);
        tr(b, 1, OK_M_SETCLASS, &a, NULL); bb_free(&a);
    }
    return 1;
}

static int w_add_observation(ok_vsb* b, int w, uint64_t lm, ok_vg_kid kid, int uc, const vframe* f) {
    bb a; double meas[2], size;
    const float* k = &f->cam[kid.cam].kp[3 * kid.kp];
    int ok;
    meas[0] = (double)k[0]; meas[1] = (double)k[1]; size = (double)k[2];
    memset(&a, 0, sizeof a);
    ok = ok_vg_add_observation(b->g[w], lm, kid, uc, &b->cam_model[kid.cam], meas, size);
    a_id(&a, lm); bb_kid(&a, kid); bb_u32(&a, uc ? 1u : 0u); bb_f64n(&a, meas, 2); bb_f64(&a, size);
    bb_raw(&a, b->cam_header[kid.cam], b->cam_hlen[kid.cam]);
    tr(b, w, OK_M_ADDOBS, &a, NULL); bb_free(&a);
    return ok;
}
int ok_vsb_add_observation(ok_vsb* b, uint64_t lm, uint64_t state, uint32_t cam, uint32_t kp, int uc) {
    ok_vg_kid kid; const vframe* f = fr_get(b, state);
    int ok;
    kid.frame = state; kid.cam = cam; kid.kp = kp;
    if (!f) return 0;
    ok = w_add_observation(b, 0, lm, kid, uc, f);
    if (NOW_LOOP(b)) { ok_idset_add(&b->touched_states, state); ok_idset_add(&b->touched_landmarks, lm); }
    else w_add_observation(b, 1, lm, kid, uc, f);
    return ok;
}
int ok_vsb_remove_observation(ok_vsb* b, uint64_t state, uint32_t cam, uint32_t kp) {
    ok_vg_kid kid; uint64_t lm = 0;
    int ok;
    kid.frame = state; kid.cam = cam; kid.kp = kp;
    if (NOW_LOOP(b)) {
        ok_vg_obs_find(b->g[0], kid, &lm, NULL, NULL);
        ok_idset_add(&b->touched_landmarks, lm); ok_idset_add(&b->touched_states, state);
    } else w_remove_observation(b, 1, kid);
    ok = ok_vg_obs_find(b->g[0], kid, NULL, NULL, NULL);
    w_remove_observation(b, 0, kid);
    ml_trace(b, state, cam, kp, 0);
    return ok;
}
int ok_vsb_set_observation_information(ok_vsb* b, uint64_t state, uint32_t cam, uint32_t kp, const double info[4]) {
    ok_vg_kid kid; bb a;
    kid.frame = state; kid.cam = cam; kid.kp = kp;
    if (NOW_LOOP(b)) {
        uint64_t lm = 0;
        ok_vg_obs_find(b->g[0], kid, &lm, NULL, NULL);
        ok_idset_add(&b->touched_landmarks, lm); ok_idset_add(&b->touched_states, state);
    } else {
        memset(&a, 0, sizeof a);
        ok_vg_poke_set_observation_information(b->g[1], kid, info);
        bb_kid(&a, kid); bb_f64n(&a, info, 4);
        w_poke(b, 1, OK_M_POKE_SETINFO, &a); bb_free(&a);
    }
    memset(&a, 0, sizeof a);
    ok_vg_poke_set_observation_information(b->g[0], kid, info);
    bb_kid(&a, kid); bb_f64n(&a, info, 4);
    w_poke(b, 0, OK_M_POKE_SETINFO, &a); bb_free(&a);
    return 1;
}

int ok_vsb_merge_landmark(ok_vsb* b, uint64_t from, uint64_t into) {
    ok_vg_kid* kids; int n, i, ok = 1;
    w_merge_landmark(b, 0, from, into);
    n = ok_vg_landmark_obs(b->g[0], into, &kids);
    for (i = 0; i < n; ++i) ml_trace(b, kids[i].frame, kids[i].cam, kids[i].kp, into);
    if (NOW_LOOP(b)) {
        for (i = 0; i < n; ++i) ok_idset_add(&b->touched_states, kids[i].frame);
        ok_idset_add(&b->touched_landmarks, from); ok_idset_add(&b->touched_landmarks, into);
    } else w_merge_landmark(b, 1, from, into);
    free(kids);
    return ok;
}

int ok_vsb_merge_landmarks(ok_vsb* b, const uint64_t* from_in, const uint64_t* into_in, int n) {
    uint64_t* from = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(n ? n : 1));
    uint64_t* into = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(n ? n : 1));
    uint64_t* ck = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(n ? n : 1));     /* changes: from -> into (std::map) */
    uint64_t* cv = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(n ? n : 1));
    int nc = 0, ctr = 0, i, j;
    memcpy(from, from_in, sizeof(uint64_t) * (size_t)n); memcpy(into, into_in, sizeof(uint64_t) * (size_t)n);
    for (i = 0; i < n; ++i) {
        int again = 1;
        while (again) { again = 0; for (j = 0; j < nc; ++j) if (ck[j] == from[i]) { from[i] = cv[j]; again = 1; break; } }
        again = 1;
        while (again) { again = 0; for (j = 0; j < nc; ++j) if (ck[j] == into[i]) { into[i] = cv[j]; again = 1; break; } }
        if (from[i] == into[i]) continue;
        {   /* realtimeGraph_.mergeLandmark(...) succeeds iff both landmarks exist */
            const int ok = ok_vg_landmark_find(b->g[0], from[i], NULL) && ok_vg_landmark_find(b->g[0], into[i], NULL);
            w_merge_landmark(b, 0, from[i], into[i]);
            if (ok) ctr++;
        }
        w_merge_landmark(b, 1, from[i], into[i]);
        { int found = 0; for (j = 0; j < nc; ++j) if (ck[j] == from[i]) { cv[j] = into[i]; found = 1; } if (!found) { ck[nc] = from[i]; cv[nc] = into[i]; nc++; } }
        {   ok_vg_kid* kids; const int m = ok_vg_landmark_obs(b->g[0], into[i], &kids); int k;
            for (k = 0; k < m; ++k) ml_trace(b, kids[k].frame, kids[k].cam, kids[k].kp, into[i]);
            free(kids); }
    }
    free(from); free(into); free(ck); free(cv);
    return ctr;
}

int ok_vsb_set_pose(ok_vsb* b, uint64_t id, const double T7[7]) {
    w_set_pose(b, 0, id, T7);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_states, id); else w_set_pose(b, 1, id, T7);
    return 1;
}
int ok_vsb_set_speed_and_bias(ok_vsb* b, uint64_t id, const double sb[9]) {
    w_set_sb(b, 0, id, sb);
    if (NOW_LOOP(b)) ok_idset_add(&b->touched_states, id); else w_set_sb(b, 1, id, sb);
    return 1;
}
int ok_vsb_set_extrinsics(ok_vsb* b, uint64_t id, int cam, const double T7[7]) {
    bb a; int w;
    for (w = 0; w < 2; ++w) {
        if (w == 1 && NOW_LOOP(b)) { ok_idset_add(&b->touched_states, id); break; }
        memset(&a, 0, sizeof a);
        ok_vg_set_extrinsics(b->g[w], id, cam, T7);
        a_id(&a, id); bb_u32(&a, (uint32_t)cam); bb_f64n(&a, T7, 7);
        tr(b, w, OK_M_SETEXTR, &a, NULL); bb_free(&a);
    }
    return 1;
}

int ok_vsb_clean_unobserved_landmarks(ok_vsb* b) {
    uint64_t* lms; ok_vg_kid* kids; int* hk; int nrem, removed1, i;
    bb r; memset(&r, 0, sizeof r);
    removed1 = ok_vg_clean_unobserved_landmarks_ex(b->g[0], &lms, &kids, &hk, &nrem);
    bb_u32(&r, (uint32_t)removed1);
    tr(b, 0, OK_M_CLEANLM, NULL, &r); bb_free(&r);
    for (i = 0; i < nrem; ++i) if (hk[i]) ml_trace(b, kids[i].frame, kids[i].cam, kids[i].kp, 0);
    if (NOW_LOOP(b)) {
        for (i = 0; i < nrem; ++i) {
            ok_idset_add(&b->touched_landmarks, lms[i]);
            if (hk[i]) ok_idset_add(&b->touched_states, kids[i].frame);
        }
    } else {
        const int removed0 = ok_vg_clean_unobserved_landmarks(b->g[1]);
        memset(&r, 0, sizeof r);
        bb_u32(&r, (uint32_t)removed0);
        tr(b, 1, OK_M_CLEANLM, NULL, &r); bb_free(&r);
    }
    free(lms); free(kids); free(hk);
    return removed1;
}

double ok_vsb_overlap_fraction(const ok_vsb* b, uint64_t ida, uint64_t idb) {
    const vframe* fa = (ida < (uint64_t)b->nframes_alloc && b->frames[ida].alive) ? &b->frames[ida] : NULL;
    const vframe* fb = (idb < (uint64_t)b->nframes_alloc && b->frames[idb].alive) ? &b->frames[idb] : NULL;
    if (!fa || !fb) return 0.0;
    return ok_vsb_overlap(fa, fb, b->kptradius);
}

const ok_vsb_frame_view* ok_vsb_frame(const ok_vsb* b, uint64_t id) { return (id < (uint64_t)b->nframes_alloc && b->frames[id].alive) ? &b->frames[id] : NULL; }
int ok_vsb_num_frames(const ok_vsb* b) { return b->nframes_alive; }
const ok_cam* ok_vsb_camera(const ok_vsb* b, int cam) { return (cam >= 0 && cam < OK_VSB_MAXCAM && b->cam_header[cam]) ? &b->cam_model[cam] : NULL; }
int ok_vsb_is_in_imu_window(const ok_vsb* b, uint64_t id) { return (id < (uint64_t)b->naux_alloc && b->aux[id].alive) ? b->aux[id].is_imu : 0; }

uint64_t ok_vsb_current_state_id(const ok_vsb* b) { return cur_state((ok_vsb*)b); }
const ok_idset* ok_vsb_key_frames(const ok_vsb* b) { return &b->key_frames; }
const ok_idset* ok_vsb_imu_frames(const ok_vsb* b) { return &b->imu_frames; }
const ok_idset* ok_vsb_loop_closure_frames(const ok_vsb* b) { return &b->lc_frames; }
int ok_vsb_needs_full_graph_optimisation(const ok_vsb* b) { return b->needs_full; }
int ok_vsb_is_loop_closing(const ok_vsb* b) { return b->is_loop_closing; }
int ok_vsb_is_pose_graph_frame(const ok_vsb* b, uint64_t id) { const aux* a = (id < (uint64_t)b->naux_alloc && b->aux[id].alive) ? &b->aux[id] : NULL; return a ? a->is_pg : 0; }
int ok_vsb_is_place_recognition_frame(const ok_vsb* b, uint64_t id) { const aux* a = (id < (uint64_t)b->naux_alloc && b->aux[id].alive) ? &b->aux[id] : NULL; return a ? a->is_prf : 0; }
int ok_vsb_is_loop_closure_frame(const ok_vsb* b, uint64_t id) { return ok_idset_has(&b->lc_frames, id); }
int ok_vsb_is_recent_loop_closure_frame(const ok_vsb* b, uint64_t id) {
    int i;
    for (i = 0; i < b->key_frames.n; ++i) {
        const aux* a = (b->key_frames.a[i] < (uint64_t)b->naux_alloc && b->aux[b->key_frames.a[i]].alive) ? &b->aux[b->key_frames.a[i]] : NULL;
        if (a && ok_idset_has(&a->recent, id)) return 1;
    }
    for (i = 0; i < b->lc_frames.n; ++i) {
        const aux* a = (b->lc_frames.a[i] < (uint64_t)b->naux_alloc && b->aux[b->lc_frames.a[i]].alive) ? &b->aux[b->lc_frames.a[i]] : NULL;
        if (a && ok_idset_has(&a->recent, id)) return 1;
    }
    return 0;
}
int ok_vsb_is_loop_closure_available(const ok_vsb* b) { return b->is_loop_closure_available && !b->is_loop_closing; }

uint64_t ok_vsb_most_overlapped_state_id(const ok_vsb* bc, uint64_t frame, int consider_lc) {
    ok_vsb* b = (ok_vsb*)bc;
    ok_idset all; uint64_t ret = 0; double overlap = 0.0; int i;
    memset(&all, 0, sizeof all);
    idset_insert_all(&all, &b->key_frames); idset_insert_all(&all, &b->imu_frames); idset_insert_all(&all, &b->lc_frames);
    if (!consider_lc) for (i = 0; i < b->cur_lc_frames.n; ++i) ok_idset_del(&all, b->cur_lc_frames.a[i]);
    for (i = 0; i < all.n; ++i) {
        const uint64_t id = all.a[i];
        ok_vg_state_view v;
        double this_overlap;
        if (id == frame) continue;
        if (!ok_vg_state_find(b->g[0], id, &v)) continue;           /* states_.at(id) would throw */
        if (!v.is_kf) continue;
        this_overlap = ok_vsb_overlap_fraction(b, id, frame);
        if (this_overlap >= overlap) { ret = id; overlap = this_overlap; }
    }
    ok_idset_free(&all);
    return ret;
}
static uint64_t current_keyframe_state_id(ok_vsb* b, int consider_lc) { return ok_vsb_most_overlapped_state_id(b, cur_state(b), consider_lc); }
static uint64_t current_loopclosure_state_id(ok_vsb* b) {
    const uint64_t cur = cur_state(b);
    uint64_t ret = 0; double overlap = 0.0; int i;
    for (i = 0; i < b->lc_frames.n; ++i) {
        const uint64_t id = b->lc_frames.a[i];
        ok_vg_state_view v;
        double this_overlap;
        if (id == cur) continue;
        if (!ok_vg_state_find(b->g[0], id, &v) || !v.is_kf) continue;
        this_overlap = ok_vsb_overlap_fraction(b, id, cur);
        if (this_overlap >= overlap) { ret = id; overlap = this_overlap; }
    }
    if (overlap > 0.5) return ret;
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------
 * optimise: ViGraph::optimise through hooks.solve, then the solver's changes are applied to the graph
 * ---------------------------------------------------------------------------------------------------------------- */
static int apply_opt_result(ok_vg* g, const unsigned char* r, size_t n) {
    size_t off = 0;
    uint32_t nch, i, k, nil;
    ok_vg_blkref* bl; ok_vg_imuref* il;
    int nbl, nilk;
#define RD(dst, sz) do { if (off + (sz) > n) { rc = 0; goto done; } memcpy(&(dst), r + off, (sz)); off += (sz); } while (0)
    int rc = 1;
    nbl = ok_vg_blocks(g, &bl);
    nilk = ok_vg_imu_links(g, &il);
    RD(nch, 4);
    for (i = 0; i < nch; ++i) {
        uint32_t idx; double x[9];
        RD(idx, 4);
        if ((int)idx >= nbl) { rc = 0; goto done; }
        if (off + 8 * (size_t)bl[idx].b->size > n) { rc = 0; goto done; }
        memcpy(x, r + off, 8 * (size_t)bl[idx].b->size); off += 8 * (size_t)bl[idx].b->size;
        ok_vg_blk_set(bl[idx].b, x);
    }
    RD(nil, 4);
    for (i = 0; i < nil; ++i) {
        uint64_t id; uint32_t redo, counter; double sb[9];
        RD(id, 8); RD(redo, 4); RD(counter, 4);
        if (off + 72 > n) { rc = 0; goto done; }
        memcpy(sb, r + off, 72); off += 72;
        for (k = 0; k < (uint32_t)nilk; ++k) if (il[k].state_id == id) { ok_vg_imu_apply_redo(il[k].e, (int)redo, (int)counter, sb); break; }
    }
done:
    free(bl); free(il);
    return rc;
#undef RD
}
static int do_optimise(ok_vsb* b, int w, int max_iter) {
    unsigned char* res = NULL; size_t rlen = 0;
    int ok;
    if (!b->h.solve) return 0;
    ok = b->h.solve(b->h.ctx, w, b->g[w], max_iter, &res, &rlen);
    if (ok) ok = apply_opt_result(b->g[w], res, rlen);
    free(res);
    return ok;
}

/* ------------------------------------------------------------------------------------------------------------------
 * optimiseRealtimeGraph
 * ---------------------------------------------------------------------------------------------------------------- */
static void tf_to_coeffs(const ok_tf* t, double c[7]) { memcpy(c, t->r, sizeof(double) * 3); c[3] = t->q.x; c[4] = t->q.y; c[5] = t->q.z; c[6] = t->q.w; }

static void copy_state_to_full(ok_vsb* b, uint64_t id) {
    double T[7], sb[9]; bb a;
    memset(&a, 0, sizeof a);
    ok_vg_pose_values(b->g[0], id, T); ok_vg_sb_values(b->g[0], id, sb);
    ok_vg_poke_copy_state(b->g[1], id, T, sb);
    a_id(&a, id); bb_f64n(&a, T, 7); bb_f64n(&a, sb, 9);
    w_poke(b, 1, OK_M_POKE_COPYSTATE, &a); bb_free(&a);
}

static void push_id(uint64_t** v, int* n, int* cap, uint64_t id) {
    if (*n == *cap) { *cap = *cap ? 2 * *cap : 16; *v = (uint64_t*)realloc(*v, sizeof(uint64_t) * (size_t)*cap); }
    (*v)[(*n)++] = id;
}

int ok_vsb_optimise_realtime(ok_vsb* b, int num_iter, int num_threads, int verbose, int only_newest, int is_initialised, uint64_t** updated, int* nupdated) {
    ok_vg* rt = b->g[0];
    int have_fix = 0, frozen = 0, i, cap = 0, ns;
    uint64_t unfreeze_id = 0;
    bb a;
    (void)num_threads; (void)verbose;
    *updated = NULL; *nupdated = 0;
    ns = ok_vg_state_count(rt);
    if (!is_initialised) {
        double informationDiag_unused = 0.0; ok_vg_state_view last; double p1[7], pl[7], T7[7]; ok_tf T; ok_quat q;
        (void)informationDiag_unused;
        ok_vg_poke_fixation_add(rt);
        memset(&a, 0, sizeof a); w_poke(b, 0, OK_M_POKE_FIX_ADD, &a); bb_free(&a);
        have_fix = 1;
        ok_vg_state_at(rt, ns - 1, &last);
        ok_vg_pose_values(rt, 1, p1); ok_vg_pose_values(rt, last.id, pl);
        q.x = pl[3]; q.y = pl[4]; q.z = pl[5]; q.w = pl[6];
        ok_tf_from_rq(&T, p1, &q, 1);                          /* Transformation T_WS(r of state 1, q of the newest) */
        tf_to_coeffs(&T, T7);
        w_set_pose(b, 0, last.id, T7);
    }
    if (only_newest) {
        ok_vg_state_view v;
        for (i = ns - 1; i >= 0; --i) {                          /* paranoid: find last frozen */
            ok_vg_state_at(rt, i, &v);
            if (v.pose_fixed) break; else unfreeze_id = v.id;
        }
        if (ns - 2 >= 0) {
            ok_vg_state_at(rt, ns - 2, &v);
            w_freeze_poses(b, 0, v.id, 0);
            w_freeze_sb(b, 0, v.id, 0);
            frozen = 1;
        }
        ok_vg_poke_landmarks_constant(rt, 1);
        memset(&a, 0, sizeof a); bb_u32(&a, 1u); w_poke(b, 0, OK_M_POKE_LM_CONST, &a); bb_free(&a);
    }
    ok_vg_set_solver_options(rt, 3, ok_vg_function_tolerance(rt));          /* DENSE_SCHUR */
    do_optimise(b, 0, num_iter);
    if (only_newest) {
        if (frozen) { w_unfreeze_poses(b, 0, unfreeze_id); w_unfreeze_sb(b, 0, unfreeze_id); }
        ok_vg_poke_landmarks_constant(rt, 0);
        memset(&a, 0, sizeof a); bb_u32(&a, 0u); w_poke(b, 0, OK_M_POKE_LM_CONST, &a); bb_free(&a);
        if (have_fix) { ok_vg_poke_fixation_remove(rt); memset(&a, 0, sizeof a); w_poke(b, 0, OK_M_POKE_FIX_REMOVE, &a); bb_free(&a); }
        if (!NOW_LOOP(b)) {
            ok_vg_state_view last;
            ok_vg_state_at(rt, ok_vg_state_count(rt) - 1, &last);
            copy_state_to_full(b, last.id);
        }
        return 1;
    }
    /* import landmarks */
    ok_vg_update_landmarks(rt);
    memset(&a, 0, sizeof a); tr(b, 0, OK_M_UPDLM, &a, NULL); bb_free(&a);
    for (i = 0; i < b->updated_lc_attempt.n; ++i) push_id(updated, nupdated, &cap, b->updated_lc_attempt.a[i]);
    ns = ok_vg_state_count(rt);
    for (i = ns - 1; i >= 0; --i) {
        ok_vg_state_view v;
        ok_vg_state_at(rt, i, &v);
        if (v.pose_fixed && v.sb_fixed) break;
        if (!ok_idset_has(&b->updated_lc_attempt, v.id)) push_id(updated, nupdated, &cap, v.id);
        if (!NOW_LOOP(b)) copy_state_to_full(b, v.id);
    }
    idset_clear(&b->updated_lc_attempt);
    if (!NOW_LOOP(b)) {
        const int nl = ok_vg_landmark_count(rt);
        for (i = 0; i < nl; ++i) {
            ok_vg_lm_view lv;
            ok_vg_landmark_find(rt, ok_vg_landmark_id_at(rt, i), &lv);
            w_set_landmark_full(b, 1, lv.id, lv.hp, lv.initialised);
            w_set_landmark_quality(b, 1, lv.id, lv.quality);
        }
    }
    for (i = ns - 1; i >= 0; --i) {
        ok_vg_state_view v;
        ok_vg_state_at(rt, i, &v);
        if (!v.has_prev_imu) continue;
        if (!NOW_LOOP(b)) {
            memset(&a, 0, sizeof a);
            ok_vg_poke_sync_imu(b->g[1], rt, v.id);
            bb_u64(&a, 0); bb_u64(&a, v.id);
            w_poke(b, 1, OK_M_POKE_SYNCIMU, &a); bb_free(&a);
        }
        if (v.pose_fixed) break;
    }
    if (have_fix) { ok_vg_poke_fixation_remove(rt); memset(&a, 0, sizeof a); w_poke(b, 0, OK_M_POKE_FIX_REMOVE, &a); bb_free(&a); }
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * optimiseFullGraph
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_optimise_full(ok_vsb* b, int num_iter, int num_threads, int verbose) {
    ok_vg* full = b->g[1];
    int i;
    (void)num_threads; (void)verbose;
    b->needs_full = 0;
    b->is_loop_closing = 1;
    if (b->nrel > 0) {
        for (i = 0; i < b->nrel; ++i) {
            double info[36]; int k; bb a;
            for (k = 0; k < 36; ++k) info[k] = 100.0 * b->rel[i].info[k];            /* 100 * information */
            memset(&a, 0, sizeof a);
            ok_vg_add_relative_pose_constraint(full, b->rel[i].pi, b->rel[i].pj, b->rel[i].T, info);
            a_id(&a, b->rel[i].pi); a_id(&a, b->rel[i].pj); bb_f64n(&a, b->rel[i].T, 7); bb_f64n(&a, info, 36);
            tr(b, 1, OK_M_ADDRELPOSE, &a, NULL); bb_free(&a);
        }
        ok_vg_set_solver_options(full, ok_vg_solver_type(full), 0.001);
        do_optimise(b, 1, num_iter / 3);
        for (i = 0; i < b->nrel; ++i) {
            bb a; memset(&a, 0, sizeof a);
            ok_vg_remove_relative_pose_constraint(full, b->rel[i].pi, b->rel[i].pj);
            a_id(&a, b->rel[i].pi); a_id(&a, b->rel[i].pj);
            tr(b, 1, OK_M_RMRELPOSE, &a, NULL); bb_free(&a);
        }
        b->nrel = 0;
    }
    ok_vg_set_solver_options(full, ok_vg_solver_type(full), 1e-6);
    do_optimise(b, 1, num_iter);
    ok_vg_update_landmarks(full);
    { bb a; memset(&a, 0, sizeof a); tr(b, 1, OK_M_UPDLM, &a, NULL); bb_free(&a); }
    b->is_loop_closure_available = 1;
    b->is_loop_closing = 0;
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * pose-graph conversion of frames (ViSlamBackend::convertToPoseGraphMst) and frontier expansion
 * ---------------------------------------------------------------------------------------------------------------- */
static void w_mst(ok_vsb* b, const ok_idset* convert, const ok_idset* consider, ok_idset* affected, ok_vg_mst_result* res) {
    bb a, r; int i;
    memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
    ok_vg_convert_to_pose_graph_mst(b->g[0], convert->a, convert->n, consider->a, consider->n, res);
    bb_u32(&a, (uint32_t)convert->n); for (i = 0; i < convert->n; ++i) bb_u64(&a, convert->a[i]);
    bb_u32(&a, (uint32_t)consider->n); for (i = 0; i < consider->n; ++i) bb_u64(&a, consider->a[i]);
    if (!res->ret) bb_u32(&r, 0u);
    else {
        bb_u32(&r, 1u); bb_u32(&r, (uint32_t)res->nmst);
        for (i = 0; i < res->nmst; ++i) { bb_u64(&r, res->mst[i][0]); bb_u64(&r, res->mst[i][1]); }
        bb_u32(&r, (uint32_t)res->ncreated);
        for (i = 0; i < res->ncreated; ++i) { bb_u64(&r, res->created[i][0]); bb_u64(&r, res->created[i][1]); bb_u64(&r, 0); }
        bb_u32(&r, (uint32_t)res->nremoved_tp);
        for (i = 0; i < res->nremoved_tp; ++i) { bb_u64(&r, res->removed_tp[i][0]); bb_u64(&r, res->removed_tp[i][1]); }
        bb_u32(&r, (uint32_t)res->nremoved_obs);
        for (i = 0; i < res->nremoved_obs; ++i) bb_kid(&r, res->removed_obs[i]);
    }
    tr(b, 0, OK_M_MST, &a, &r); bb_free(&a); bb_free(&r);
    (void)affected;
}

static int convert_to_pose_graph_mst(ok_vsb* b, const ok_idset* convert, const ok_idset* consider, ok_idset* affected) {
    ok_vg_mst_result res;
    int i;
    /* remember landmarks in frames (transformed to sensor frame): MultiFrame::setLandmark(i, k, T_SW * landmark, initialised) */
    for (i = 0; i < convert->n; ++i) {
        vframe* mf = fr_get(b, convert->a[i]);
        double c7[7];
        ok_tf T_WS, T_SW;
        int im, k;
        if (!mf || !ok_vg_state_find(b->g[0], convert->a[i], NULL)) continue;
        ok_vg_pose_values(b->g[0], convert->a[i], c7);
        ok_tf_convert(&T_WS, c7);
        ok_tf_inverse(&T_WS, &T_SW, 1);
        for (im = 0; im < mf->ncam; ++im)
            for (k = 0; k < mf->cam[im].nkp; ++k) {
                const uint64_t lm = mf->cam[im].lm[k];
                ok_vg_lm_view lv;
                if (lm && ok_vg_landmark_find(b->g[0], lm, &lv)) {
                    double out[4];
                    ok_tf_mul_v4(&T_SW, lv.hp, out, 1);
                    memcpy(mf->cam[im].lmhp + 4 * k, out, sizeof out);
                    mf->cam[im].lminit[k] = (unsigned char)(lv.initialised ? 1 : 0);
                }
            }
    }
    w_mst(b, convert, consider, affected, &res);
    for (i = 0; i < res.ncreated; ++i) { ok_idset_add(affected, res.created[i][1]); ok_idset_add(affected, res.created[i][0]); }
    for (i = 0; i < res.nremoved_tp; ++i) { ok_idset_add(affected, res.removed_tp[i][0]); ok_idset_add(affected, res.removed_tp[i][1]); }
    if (NOW_LOOP(b)) {
        for (i = 0; i < res.ncreated; ++i) { ok_idset_add(&b->touched_states, res.created[i][0]); ok_idset_add(&b->touched_states, res.created[i][1]); }
        for (i = 0; i < res.nremoved_tp; ++i) { ok_idset_add(&b->touched_states, res.removed_tp[i][0]); ok_idset_add(&b->touched_states, res.removed_tp[i][1]); }
        for (i = 0; i < res.nremoved_obs; ++i) {
            uint64_t lm = 0;
            ok_idset_add(&b->touched_states, res.removed_obs[i].frame);
            if (ok_vg_obs_find(b->g[1], res.removed_obs[i], &lm, NULL, NULL)) ok_idset_add(&b->touched_landmarks, lm);   /* lmId.isInitialised() is always true */
        }
    } else {
        for (i = 0; i < res.nremoved_obs; ++i) w_remove_observation(b, 1, res.removed_obs[i]);
        for (i = 0; i < res.nremoved_tp; ++i) {
            bb a; memset(&a, 0, sizeof a);
            ok_vg_remove_two_pose_const_link(b->g[1], res.removed_tp[i][0], res.removed_tp[i][1]);
            a_id(&a, res.removed_tp[i][0]); a_id(&a, res.removed_tp[i][1]);
            tr(b, 1, OK_M_RMTPC, &a, NULL); bb_free(&a);
        }
        for (i = 0; i < res.ncreated; ++i) {
            ok_tp_std clone; bb a; unsigned char* pl; size_t n;
            const int have = ok_vg_clone_two_pose_const(b->g[0], res.created[i][0], res.created[i][1], &clone);
            memset(&a, 0, sizeof a);
            if (have) ok_vg_add_external_two_pose_link(b->g[1], res.created[i][0], res.created[i][1], &clone);
            a_id(&a, res.created[i][0]); a_id(&a, res.created[i][1]); bb_u32(&a, have ? 1u : 0u);
            if (have) { n = ok_vg_tp_payload(&clone, &pl); bb_raw(&a, pl, n); free(pl); }
            tr(b, 1, OK_M_ADDEXTTP, &a, NULL); bb_free(&a);
        }
    }
    ok_vg_mst_result_free(&res);
    return 1;
}

static void w_conv_obs(ok_vsb* b, uint64_t id, ok_vg_conv_result* res) {
    bb a, r; int i;
    memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
    ok_vg_convert_to_observations(b->g[0], id, res);
    a_id(&a, id);
    bb_u32(&r, (uint32_t)res->ctr); bb_u32(&r, (uint32_t)res->nobs);
    for (i = 0; i < res->nobs; ++i) { bb_kid(&r, res->kid[i]); bb_u64(&r, 0); bb_u64(&r, res->lm[i]); }
    bb_u32(&r, (uint32_t)res->nlm); for (i = 0; i < res->nlm; ++i) bb_u64(&r, res->lms[i]);
    bb_u32(&r, (uint32_t)res->nconnected); for (i = 0; i < res->nconnected; ++i) bb_u64(&r, res->connected[i]);
    tr(b, 0, OK_M_CONVOBS, &a, &r); bb_free(&a); bb_free(&r);
}

static void rm_tpc_all_full(ok_vsb* b, uint64_t id) {
    bb a; memset(&a, 0, sizeof a);
    ok_vg_remove_two_pose_const_links(b->g[1], id);
    a_id(&a, id); tr(b, 1, OK_M_RMTPC_ALL, &a, NULL); bb_free(&a);
}
static void add_lm_copy_full(ok_vsb* b, const ok_vg_conv_result* res) {
    int i;
    for (i = 0; i < res->nlm; ++i) {
        ok_vg_lm_view lv;
        if (!ok_vg_landmark_find(b->g[1], res->lms[i], NULL)) {
            ok_vg_landmark_find(b->g[0], res->lms[i], &lv);
            w_add_landmark_id(b, 1, res->lms[i], lv.hp, lv.initialised);
            w_set_landmark_quality(b, 1, res->lms[i], lv.quality);
        }
    }
}
static int expand_keyframe(ok_vsb* b, uint64_t keyframe) {
    ok_vg_conv_result res;
    int ctr = 0, i;
    /* (asserts: keyframe is not an IMU frame, is a keyframe / loop-closure frame, has two-pose links) */
    w_conv_obs(b, keyframe, &res);
    if (NOW_LOOP(b)) {
        for (i = 0; i < res.nconnected; ++i) { uint64_t c = res.connected[i]; ok_idset_add(&b->touched_states, c); }
        for (i = 0; i < res.nlm; ++i) ok_idset_add(&b->touched_landmarks, res.lms[i]);
    } else {
        rm_tpc_all_full(b, keyframe);
        add_lm_copy_full(b, &res);
        for (i = 0; i < res.nobs; ++i) w_add_external_observation(b, 1, res.lm[i], res.kid[i], res.cauchy[i], res.err[i]);
    }
    /* connected is not sorted/unique in the C result: the C++ set is */
    {   ok_idset cn; memset(&cn, 0, sizeof cn);
        for (i = 0; i < res.nconnected; ++i) ok_idset_add(&cn, res.connected[i]);
        for (i = 0; i < cn.n; ++i) {
            const uint64_t c = cn.a[i];
            aux* ax = ax_get(b, c);
            if (!ax) continue;
            if (!ok_idset_has(&b->lc_frames, c) && ax->is_pg) { ok_idset_add(&b->key_frames, c); ax->is_pg = 0; ctr++; }
            if (!ok_idset_has(&b->key_frames, c) && ax->is_pg) { ok_idset_add(&b->lc_frames, c); ax->is_pg = 0; ctr++; }
        }
        ok_idset_free(&cn); }
    ok_vg_conv_result_free(&res);
    return ctr;
}

/* ------------------------------------------------------------------------------------------------------------------
 * eliminateImuFrames, applyStrategy
 * ---------------------------------------------------------------------------------------------------------------- */
static void eliminate_imu_frames(ok_vsb* b, size_t num_imu) {
    ok_idset copy; int i; memset(&copy, 0, sizeof copy);
    idset_copy(&copy, &b->imu_frames);
    for (i = 0; i < copy.n; ++i) {
        const uint64_t id = copy.a[i];
        ok_vg_state_view v;
        if ((size_t)b->imu_frames.n <= num_imu) break;
        ok_vg_state_find(b->g[0], id, &v);
        if (v.is_kf) {
            aux* ax = ax_get(b, id);
            ok_idset_del(&b->imu_frames, id);
            ok_idset_add(&b->key_frames, id);
            ok_idset_del(&b->lc_frames, id);          /* make sure the two sets are not intersecting */
            if (ax) ax->is_imu = 0;
        } else {
            const uint64_t ref = ok_vsb_most_overlapped_state_id(b, id, 0);
            ok_vg_kid* kids; uint64_t* lms; int nobs, k;
            double T[7], v3[3]; uint64_t kf = 0, h = 0; bb a, r;
            nobs = ok_vg_state_obs(b->g[0], id, &kids, &lms);            /* auto observations = states_.at(id).observations */
            w_remove_all_observations(b, 0, id);
            memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
            ok_vg_eliminate_state_by_imu_merge_h(b->g[0], id, ref, &kf, T, v3, &h);
            a_id(&a, id); a_id(&a, ref); bb_u64(&r, kf); bb_f64n(&r, T, 7); bb_f64n(&r, v3, 3); bb_u64(&r, h);
            tr(b, 0, OK_M_ELIM, &a, &r); bb_free(&a); bb_free(&r);
            if (NOW_LOOP(b)) {
                for (k = 0; k < nobs; ++k) ok_idset_add(&b->touched_landmarks, lms[k]);
                { int f = 0, j; for (j = 0; j < b->nelim; ++j) if (b->elim[j].id == id) { b->elim[j].ref = ref; f = 1; }
                  if (!f) {
                      if (b->nelim == b->capelim) { b->capelim = b->capelim ? 2 * b->capelim : 16; b->elim = (elimpair*)realloc(b->elim, sizeof(elimpair) * (size_t)b->capelim); }
                      j = b->nelim++;
                      while (j > 0 && b->elim[j - 1].id > id) { b->elim[j] = b->elim[j - 1]; --j; }
                      b->elim[j].id = id; b->elim[j].ref = ref;
                  } }
            } else {
                memset(&r, 0, sizeof r);
                w_remove_all_observations(b, 1, id);
                ok_vg_eliminate_state_by_imu_merge_h(b->g[1], id, ref, &kf, T, v3, &h);
                memset(&a, 0, sizeof a);
                a_id(&a, id); a_id(&a, ref); bb_u64(&r, kf); bb_f64n(&r, T, 7); bb_f64n(&r, v3, 3); bb_u64(&r, h);
                tr(b, 1, OK_M_ELIM, &a, &r); bb_free(&a); bb_free(&r);
            }
            free(kids); free(lms);
            ok_idset_del(&b->imu_frames, id);
            fr_erase(b, id);
            ax_erase(b, id);
        }
    }
    ok_idset_free(&copy);
}

/* index in the live auxiliary states of the largest id <= v ... helpers for the reverse iteration over auxiliaryStates_ */
int ok_vsb_apply_strategy(ok_vsb* b, size_t num_kf, size_t num_lc, size_t num_imu, int expand, uint64_t** affected_out, int* naffected) {
    ok_idset affected; memset(&affected, 0, sizeof affected);
    uint64_t current_kf_id, current_frame_id;
    int key_frame_eliminated = 0, ctr_pg = 0, ctr_lc = 0;
    *affected_out = NULL; *naffected = 0;
    /* (check / handle lost: trackingQuality only logs) */
    eliminate_imu_frames(b, num_imu);
    current_kf_id = current_keyframe_state_id(b, 1);
    current_frame_id = cur_state(b);
    if (current_kf_id == 0) goto out;

    if ((size_t)b->key_frames.n > num_kf) {
        while ((size_t)b->key_frames.n > num_kf) {
            uint64_t min_id = 0, max_id = 0;
            int min_obs = 100000, max_co = 0, i;
            ok_idset consider, observed, convert;
            memset(&consider, 0, sizeof consider); memset(&observed, 0, sizeof observed); memset(&convert, 0, sizeof convert);
            w_covis(b, 0);
            current_kf_id = current_keyframe_state_id(b, 1);
            current_frame_id = cur_state(b);
            for (i = 0; i < b->key_frames.n; ++i) {
                const uint64_t kfid = b->key_frames.a[i];
                int c1 = ok_vg_covisibilities(b->g[0], current_frame_id, kfid), c2 = ok_vg_covisibilities(b->g[0], current_kf_id, kfid);
                const int co = c1 < c2 ? c2 : c1;                       /* std::max(a, b) = (a < b) ? b : a */
                if (kfid == b->key_frames.a[0]) { if (co >= 2) continue; }     /* spare the oldest */
                if (co < min_obs) { min_obs = co; min_id = kfid; }
            }
            idset_copy(&observed, &b->key_frames); idset_insert_all(&observed, &b->lc_frames);
            for (i = 0; i < observed.n; ++i) {
                const uint64_t fid = observed.a[i];
                ok_vg_state_view v;
                const int co = ok_vg_covisibilities(b->g[0], min_id, fid);
                if (co >= max_co) { max_id = fid; max_co = co; }
                if (ok_vg_state_find(b->g[0], fid, &v) && v.ntp > 0) ok_idset_add(&consider, fid);     /* frontier node */
            }
            ok_idset_add(&convert, min_id);
            { aux* ax = ax_get(b, min_id); if (ax) ax->is_pg = 1; }
            ok_idset_del(&b->key_frames, min_id);
            { aux* ax = ax_get(b, min_id);
              if (ax) for (i = 0; i < b->key_frames.n; ++i) ok_idset_add(&ax->recent, b->key_frames.a[i]); }
            ok_idset_add(&consider, min_id);
            ok_idset_add(&consider, max_id);
            key_frame_eliminated = 1;
            if (max_co == 0) {
                w_remove_all_observations(b, 0, min_id);
                if (NOW_LOOP(b)) { ok_idset_add(&b->touched_states, min_id); ok_idset_add(&affected, min_id); }
                else w_remove_all_observations(b, 1, min_id);
                ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert);
                continue;
            }
            convert_to_pose_graph_mst(b, &convert, &consider, &affected);
            for (i = 0; i < convert.n; ++i) {                    /* free image memory now */
                vframe* f = fr_get(b, convert.a[i]);
                if (f) { int c; for (c = 0; c < f->ncam; ++c) f->cam[c].images_cleared = 1; }
            }
            ctr_pg++;
            ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert);
            if (ctr_pg >= 3) break;                              /* max 3 at the time... */
        }
    }

    /* freeze old states */
    if (key_frame_eliminated) {
        int ridx = b->aux_ids.n - 1;                             /* auxiliaryStates_.rbegin() */
        uint64_t oldest = 0;
        size_t p;
        int ctr = 0, it;
        for (p = 0; p < (num_kf + num_imu) && ridx >= 0; ++p) ridx--;
        if (ridx >= 0) oldest = b->aux_ids.a[ridx];
        it = ok_vg_state_index(b->g[0], oldest);
        for (; it >= 0; --it) {                                  /* states_.find(oldest) ... --iter until begin */
            if (ctr == 12) {
                ok_vg_state_view last, cv;
                const int nst = ok_vg_state_count(b->g[0]);
                ok_vg_state_at(b->g[0], nst - 1, &last);
                ok_vg_state_at(b->g[0], it, &cv);
                while (ok_duration_to_sec(ok_time_sub(last.ts, cv.ts)) < 2.0) {
                    if (it == 0) break;
                    --it;
                    ok_vg_state_at(b->g[0], it, &cv);
                }
                if (it != 0) {
                    uint64_t freeze_id = cv.id > b->last_freeze ? cv.id : b->last_freeze;      /* std::max(lastFreeze_, iter->first) */
                    b->last_freeze = freeze_id;
                    if (ok_vg_state_index(b->g[0], freeze_id) != 0) w_freeze_poses(b, 0, freeze_id, 0);
                    w_freeze_sb(b, 0, freeze_id, 0);
                }
                break;
            }
            if (it == 0) break;
            ctr++;
        }
    }

    /* loop-closure frames */
    if (b->lc_frames.n > 0) {
        do {
            uint64_t min_id = 0, max_id = 0;
            int min_obs = 100000, max_co = 0, i;
            ok_idset consider, observed, convert;
            memset(&consider, 0, sizeof consider); memset(&observed, 0, sizeof observed); memset(&convert, 0, sizeof convert);
            w_covis(b, 0);
            current_kf_id = current_keyframe_state_id(b, 1);
            current_frame_id = cur_state(b);
            for (i = 0; i < b->lc_frames.n; ++i) {
                const uint64_t lcf = b->lc_frames.a[i];
                int c1 = ok_vg_covisibilities(b->g[0], current_frame_id, lcf), c2 = ok_vg_covisibilities(b->g[0], current_kf_id, lcf);
                const int co = c1 < c2 ? c2 : c1;
                if (co < min_obs) { min_obs = co; min_id = lcf; }
            }
            if (min_obs != 0 && (size_t)b->lc_frames.n <= num_lc) { ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert); break; }
            idset_copy(&observed, &b->key_frames); idset_insert_all(&observed, &b->lc_frames);
            for (i = 0; i < observed.n; ++i) {
                const uint64_t fid = observed.a[i];
                ok_vg_state_view v;
                const int co = ok_vg_covisibilities(b->g[0], min_id, fid);
                if (co >= max_co) { max_id = fid; max_co = co; }
                if (ok_vg_state_find(b->g[0], fid, &v) && v.ntp > 0) ok_idset_add(&consider, fid);
            }
            ok_idset_add(&convert, min_id);
            { aux* ax = ax_get(b, min_id); if (ax) ax->is_pg = 1; }
            ok_idset_del(&b->lc_frames, min_id);
            ok_idset_add(&consider, min_id);
            ok_idset_add(&consider, max_id);
            if (max_co == 0) {
                w_remove_all_observations(b, 0, min_id);
                if (NOW_LOOP(b)) { ok_idset_add(&b->touched_states, min_id); ok_idset_add(&affected, min_id); }
                else w_remove_all_observations(b, 1, min_id);
                ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert);
                continue;
            }
            if (convert.n > 0 && max_co > 0) {
                convert_to_pose_graph_mst(b, &convert, &consider, &affected);
                ++ctr_lc;
                if (ctr_lc >= 3) { ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert); break; }
            }
            ok_idset_free(&consider); ok_idset_free(&observed); ok_idset_free(&convert);
        } while ((size_t)b->lc_frames.n > num_lc);
    }

    /* expand frontier */
    if (expand && ctr_lc < 3 && ctr_pg < 3) {
        uint64_t cur_kf = current_keyframe_state_id(b, 1), cur_lc;
        ok_vg_state_view v;
        if (cur_kf != 0) {
            if (ok_vg_state_find(b->g[0], cur_kf, &v) && v.ntp > 0) expand_keyframe(b, cur_kf);
        }
        cur_lc = current_loopclosure_state_id(b);
        if (cur_lc != 0) {
            if (ok_vg_state_find(b->g[0], cur_lc, &v) && v.ntp > 0) expand_keyframe(b, cur_lc);
        }
    }
out:
    *affected_out = affected.a; *naffected = affected.n;
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * attemptLoopClosure
 * ---------------------------------------------------------------------------------------------------------------- */
static double v3_norm_d(const double v[3]) { return ok_v3_norm(v); }

int ok_vsb_attempt_loop_closure(ok_vsb* b, uint64_t pose_i, uint64_t pose_j, const double T_Si_Sj7[7], const double information[36], double drift, int* skip_full) {
    ok_vg* rt = b->g[0];
    int num_steps = 0, idx_i, i, ns, ctr;
    uint64_t last_loop, first_id;
    double distance_travelled = 0.0, distance_travelled2 = 0.0;
    double* distances = NULL; int ndist = 0;
    double dvec[3] = {0.0, 0.0, 0.0};
    ok_tf T_WS_i, T_WS_j;
    ok_vg_state_view v;
    *skip_full = 0;
    if (!ok_vg_state_find(rt, pose_i, NULL)) return 0;
    if (!ok_vg_state_find(rt, pose_j, NULL)) return 0;
    ns = ok_vg_state_count(rt);
    idx_i = ok_vg_state_index(rt, pose_i);
    last_loop = ax_get(b, pose_i)->loop_id;
    for (i = idx_i; i < ns; ++i) {
        uint64_t loop_id;
        ok_vg_state_at(rt, i, &v);
        loop_id = ax_get(b, v.id)->loop_id;
        if (last_loop != loop_id) { last_loop = loop_id; num_steps++; }
    }
    first_id = ax_get(b, pose_i)->loop_id;
    last_loop = first_id;
    {   double pi7[7]; ok_vg_pose_values(rt, pose_i, pi7); ok_tf_convert(&T_WS_i, pi7); }
    distances = (double*)malloc(sizeof(double) * (size_t)(ns + 1));
    if (pose_i != last_loop) {
        double lp[7], dd[3];
        ok_vg_pose_values(rt, last_loop, lp);
        dd[0] = lp[0] - T_WS_i.r[0]; dd[1] = lp[1] - T_WS_i.r[1]; dd[2] = lp[2] - T_WS_i.r[2];
        dvec[0] += dd[0]; dvec[1] += dd[1]; dvec[2] += dd[2];
        distance_travelled2 += v3_norm_d(dvec);
    }
    for (i = idx_i + 1; i < ns; ++i) {
        uint64_t loop_id;
        double pj[7];
        ok_vg_state_at(rt, i, &v);
        loop_id = ax_get(b, v.id)->loop_id;
        ok_vg_pose_values(rt, v.id, pj); ok_tf_convert(&T_WS_j, pj);
        if (last_loop != loop_id) {
            double ds_vec[3], ds;
            ds_vec[0] = T_WS_j.r[0] - T_WS_i.r[0]; ds_vec[1] = T_WS_j.r[1] - T_WS_i.r[1]; ds_vec[2] = T_WS_j.r[2] - T_WS_i.r[2];
            ds = v3_norm_d(ds_vec);
            last_loop = loop_id;
            distances[ndist++] = ds;
            dvec[0] += ds_vec[0]; dvec[1] += ds_vec[1]; dvec[2] += ds_vec[2];
            distance_travelled += ds;
            distance_travelled2 += ds;
            T_WS_i = T_WS_j;
        }
    }
    *skip_full = 0;
    {
        ok_tf T_WSi, T_WSj_old, T_WSj_new, T_Wnew_Wold_final, T_Si_Sj, T_WSj_old_inv, T_WW, T_WS_prev, T_WS;
        double tmp7[7], aa_angle, axis[3], dr_W[3];
        double rel_pos_err, rel_ori_err, pos_budget, ori_budget, P[36], Pm3[9], ev[3], evec[9], sigma;
        ok_quat q;
        int rank;
        ok_vg_pose_values(rt, pose_i, tmp7); ok_tf_convert(&T_WSi, tmp7);
        ok_vg_pose_values(rt, pose_j, tmp7); ok_tf_convert(&T_WSj_old, tmp7);
        ok_tf_set_coeffs(&T_Si_Sj, T_Si_Sj7, 1);
        ok_tf_mul(&T_WSi, &T_Si_Sj, &T_WSj_new, 1);
        ok_tf_inverse(&T_WSj_old, &T_WSj_old_inv, 1);
        ok_tf_mul(&T_WSj_new, &T_WSj_old_inv, &T_Wnew_Wold_final, 1);
        /* rotation increment for the later averaging: AngleAxisd(q), angle * (1/numSteps), Quaterniond(aa) */
        ok_quat_to_angle_axis(&T_Wnew_Wold_final.q, &aa_angle, axis);
        aa_angle = aa_angle * (1.0 / (double)num_steps);
        q = ok_quat_from_angle_axis(aa_angle, axis);
        { const double zero[3] = {0.0, 0.0, 0.0}; ok_tf_from_rq(&T_WW, zero, &q, 1); }

        /* compute rotation adjustments */
        ok_vg_pose_values(rt, pose_i, tmp7); ok_tf_convert(&T_WS_prev, tmp7);
        ok_vg_pose_values(rt, pose_i, tmp7); ok_tf_convert(&T_WS, tmp7);
        last_loop = first_id;
        for (i = idx_i + 1; i < ns; ++i) {
            ok_tf T_WSk_old, prev_inv, T_SS, t1;
            uint64_t loop_id;
            ok_vg_state_at(rt, i, &v);
            ok_vg_pose_values(rt, v.id, tmp7); ok_tf_convert(&T_WSk_old, tmp7);
            ok_tf_inverse(&T_WS_prev, &prev_inv, 1);
            ok_tf_mul(&prev_inv, &T_WSk_old, &T_SS, 1);
            T_WS_prev = T_WSk_old;
            loop_id = ax_get(b, v.id)->loop_id;
            if (last_loop != loop_id) {
                ok_tf_mul(&T_WW, &T_WS, &t1, 1);
                ok_tf_mul(&t1, &T_SS, &T_WS, 1);
                last_loop = loop_id;
            } else {
                ok_tf_mul(&T_WS, &T_SS, &t1, 1);
                T_WS = t1;
            }
        }
        for (i = 0; i < 3; ++i) dr_W[i] = T_WSj_new.r[i] - T_WS.r[i];
        rel_pos_err = v3_norm_d(dr_W) / distance_travelled2;
        rel_ori_err = ok_quat_angular_distance(&T_WSj_new.q, &T_WSj_old.q) / (double)num_steps;       /* T_WSj_new.q().angularDistance(T_WSj_old.q()) */
        pos_budget = drift / 100.0 + 0.02 * v3_norm_d(dvec) / distance_travelled2 + 0.08 / sqrt((double)num_steps);
        ori_budget = 0.0004 + 0.004 / sqrt((double)num_steps);
        if (rel_pos_err > pos_budget || rel_ori_err > ori_budget || num_steps < 1) { free(distances); return 0; }

        /* do not accept uncertain loop closures */
        { int r, c; ok_pinv_symm(6, information, P, 2.220446049250313e-16, &rank);
          for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) Pm3[r + 3 * c] = P[r + 6 * c]; }
        ok_selfadjoint_eig(3, Pm3, ev, evec, 0);
        sigma = sqrt(ev[0] + ev[1] + ev[2]);
        if (sigma > 0.1 && 3.0 * sigma > pos_budget * distance_travelled2) { free(distances); return 0; }

        /* full adjustments */
        ok_vg_pose_values(rt, pose_i, tmp7); ok_tf_convert(&T_WS_prev, tmp7);
        ok_vg_pose_values(rt, pose_i, tmp7); ok_tf_convert(&T_WS, tmp7);
        last_loop = first_id;
        ctr = 0;
        {
            double r = 0.0;
            for (i = idx_i + 1; i < ns; ++i) {
                ok_tf T_WSk_old, prev_inv, T_SS, t1, T_WS_set, T_Wnew_Wold, old_inv;
                uint64_t loop_id;
                double sb[9], v_Wold[3], v_new[3], c7[7], set7[7], rr[3];
                ok_quat qq;
                ok_vg_state_at(rt, i, &v);
                ok_vg_pose_values(rt, v.id, tmp7); ok_tf_convert(&T_WSk_old, tmp7);
                ok_tf_inverse(&T_WS_prev, &prev_inv, 1);
                ok_tf_mul(&prev_inv, &T_WSk_old, &T_SS, 1);
                T_WS_prev = T_WSk_old;
                loop_id = ax_get(b, v.id)->loop_id;
                if (last_loop != loop_id) {
                    r += distances[ctr] / distance_travelled;
                    ok_tf_mul(&T_WW, &T_WS, &t1, 1);
                    ok_tf_mul(&t1, &T_SS, &T_WS, 1);
                    last_loop = loop_id;
                    ++ctr;
                } else {
                    ok_tf_mul(&T_WS, &T_SS, &t1, 1);
                    T_WS = t1;
                }
                rr[0] = r * dr_W[0]; rr[1] = r * dr_W[1]; rr[2] = r * dr_W[2];
                c7[0] = T_WS.r[0] + rr[0]; c7[1] = T_WS.r[1] + rr[1]; c7[2] = T_WS.r[2] + rr[2];
                qq = T_WS.q;
                ok_tf_from_rq(&T_WS_set, c7, &qq, 1);                      /* Transformation(r, q) normalises q again */
                ok_tf_inverse(&T_WSk_old, &old_inv, 1);
                ok_tf_mul(&T_WS_set, &old_inv, &T_Wnew_Wold, 1);
                ok_vg_sb_values(rt, v.id, sb);
                v_Wold[0] = sb[0]; v_Wold[1] = sb[1]; v_Wold[2] = sb[2];
                ok_m3_mulv(T_Wnew_Wold.C, v_Wold, v_new);
                sb[0] = v_new[0]; sb[1] = v_new[1]; sb[2] = v_new[2];
                tf_to_coeffs(&T_WS_set, set7);
                w_set_pose(b, 0, v.id, set7);
                w_set_pose(b, 1, v.id, set7);
                w_set_sb(b, 0, v.id, sb);
                w_set_sb(b, 1, v.id, sb);
                ok_idset_add(&b->updated_lc_attempt, v.id);
            }
        }
        /* update landmarks */
        {
            const int nl = ok_vg_landmark_count(rt);
            for (i = 0; i < nl; ++i) {
                ok_vg_lm_view lv; double hn[4];
                ok_vg_landmark_find(rt, ok_vg_landmark_id_at(rt, i), &lv);
                ok_tf_mul_v4(&T_Wnew_Wold_final, lv.hp, hn, 1);
                w_set_landmark_full(b, 0, lv.id, hn, lv.initialised);
                w_set_landmark_full(b, 1, lv.id, hn, lv.initialised);
            }
        }
    }
    ax_get(b, pose_j)->closed_loop = 1;
    if (b->nrel == b->caprel) { b->caprel = b->caprel ? 2 * b->caprel : 8; b->rel = (relinfo*)realloc(b->rel, sizeof(relinfo) * (size_t)b->caprel); }
    memcpy(b->rel[b->nrel].T, T_Si_Sj7, sizeof(double) * 7); memcpy(b->rel[b->nrel].info, information, sizeof(double) * 36);
    b->rel[b->nrel].pi = pose_i; b->rel[b->nrel].pj = pose_j; b->nrel++;
    free(distances);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * addLoopClosureFrame
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_add_loop_closure_frame(ok_vsb* b, uint64_t id, int skip_full, uint64_t** landmarks_out, int* nlandmarks) {
    ok_vg_conv_result res;
    ok_idset cn, lcl;
    int i;
    *landmarks_out = NULL; *nlandmarks = 0;
    if (ok_idset_has(&b->lc_frames, id)) return 0;               /* previously added */
    if (ok_idset_has(&b->key_frames, id)) return 0;              /* current keyframe */
    memset(&cn, 0, sizeof cn); memset(&lcl, 0, sizeof lcl);
    ok_idset_add(&b->lc_frames, id);
    ok_idset_add(&b->cur_lc_frames, id);
    w_conv_obs(b, id, &res);
    for (i = 0; i < res.nlm; ++i) ok_idset_add(&lcl, res.lms[i]);
    for (i = 0; i < res.nconnected; ++i) ok_idset_add(&cn, res.connected[i]);
    add_lm_copy_full(b, &res);
    rm_tpc_all_full(b, id);
    for (i = 0; i < res.nobs; ++i) w_add_external_observation(b, 1, res.lm[i], res.kid[i], res.cauchy[i], res.err[i]);
    for (i = 0; i < cn.n; ++i) {
        const uint64_t c = cn.a[i];
        aux* ax = ax_get(b, c);
        if (ax && ax->is_pg && !ok_idset_has(&b->key_frames, c)) {
            ok_idset_add(&b->lc_frames, c); ok_idset_add(&b->cur_lc_frames, c);
            ax->is_pg = 0;
        }
    }
    ax_get(b, id)->is_pg = 0;
    /* remember oldest Id / freeze / unfreeze */
    if (b->lc_frames.n > 0) {
        if (ok_vg_state_find(b->g[1], id, NULL)) {
            const uint64_t oldest_var = ax_get(b, id)->loop_id;
            int ai = -1, lo = 0, hi = b->aux_ids.n, ctr = 0, k;
            ok_vg_state_view fv;
            ok_time oldest_t;
            while (lo < hi) { const int mid = (lo + hi) / 2; if (b->aux_ids.a[mid] < oldest_var) lo = mid + 1; else hi = mid; }
            ai = lo;
            for (k = ai; k < b->aux_ids.n; ++k) ax_get(b, b->aux_ids.a[k])->loop_id = oldest_var;
            ok_vg_state_find(b->g[1], id, &fv); oldest_t = fv.ts;
            for (;; --ai) {
                if (ctr == 12 || ai == 0) {
                    ok_vg_state_view sv;
                    ok_vg_state_find(b->g[1], b->aux_ids.a[ai], &sv);
                    while (ok_duration_to_sec(ok_time_sub(oldest_t, sv.ts)) < 2.0) {
                        if (ai == 0) break;
                        --ai;
                        ok_vg_state_find(b->g[1], b->aux_ids.a[ai], &sv);
                    }
                    if (b->last_freeze != 0) {
                        if (b->aux_ids.a[ai] > b->last_freeze) {
                            while (b->aux_ids.a[ai] > b->last_freeze) { if (ai == 0) break; --ai; }
                        }
                    }
                    w_unfreeze_poses(b, 1, b->aux_ids.a[ai]);
                    if (ai != 0) w_freeze_poses(b, 1, b->aux_ids.a[ai], 0);
                    w_unfreeze_sb(b, 1, b->aux_ids.a[ai]);
                    if (ai != 0) w_freeze_sb(b, 1, b->aux_ids.a[ai], 0);
                    break;
                }
                if (ai == 0) { w_unfreeze_poses(b, 1, b->aux_ids.a[ai]); w_unfreeze_sb(b, 1, b->aux_ids.a[ai]); break; }
                ctr++;
            }
        }
    }
    /* remember as recent loop closures */
    for (i = 0; i < b->key_frames.n; ++i) { aux* ax = ax_get(b, b->key_frames.a[i]); if (ax) idset_insert_all(&ax->recent, &b->lc_frames); }
    for (i = 0; i < b->lc_frames.n; ++i) { aux* ax = ax_get(b, b->lc_frames.a[i]); if (ax) idset_insert_all(&ax->recent, &b->lc_frames); }
    if (!skip_full) b->needs_full = 1;
    *landmarks_out = lcl.a; *nlandmarks = lcl.n;
    ok_idset_free(&cn);
    ok_vg_conv_result_free(&res);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * synchroniseRealtimeAndFullGraph
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_synchronise(ok_vsb* b, uint64_t** updated_out, int* nupdated) {
    ok_vg* rt = b->g[0]; ok_vg* full = b->g[1];
    int i, cap = 0, nl;
    uint64_t old_id = 0;
    ok_tf T_WS_old, T_WS_new, T_Wnew_Wold, inv;
    double tmp7[7];
    *updated_out = NULL; *nupdated = 0;
    idset_insert_all(&b->key_frames, &b->lc_frames);
    idset_clear(&b->lc_frames);

    /* remove landmarks in the full graph that have disappeared */
    nl = ok_vg_landmark_count(full);
    {   uint64_t* ids = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(nl + 1));
        for (i = 0; i < nl; ++i) ids[i] = ok_vg_landmark_id_at(full, i);
        for (i = 0; i < nl; ++i) if (!ok_vg_landmark_find(rt, ids[i], NULL)) w_remove_landmark(b, 1, ids[i]);
        free(ids); }

    /* compute pose change */
    for (i = ok_vg_state_count(full) - 1; i >= 0; --i) {
        ok_vg_state_view v;
        ok_vg_state_at(full, i, &v);
        if (ok_vg_state_find(rt, v.id, NULL)) { old_id = v.id; break; }
    }
    ok_vg_pose_values(rt, old_id, tmp7); ok_tf_convert(&T_WS_old, tmp7);
    ok_vg_pose_values(full, old_id, tmp7); ok_tf_convert(&T_WS_new, tmp7);
    ok_tf_inverse(&T_WS_old, &inv, 1);
    ok_tf_mul(&T_WS_new, &inv, &T_Wnew_Wold, 1);

    /* process added new states */
    for (i = 0; i < b->nbl; ++i) {
        const backlog* bk = &b->bl[i];
        ok_vg_state_view rv;
        const int exists = ok_vg_state_find(rt, bk->id, &rv);
        const int as_kf = exists && rv.is_kf;
        ok_tf T_WS; double sb[9], pose7[7], sb_new[9];
        uint64_t nid; bb a, r;
        memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
        nid = ok_vg_add_states_propagate(full, bk->t, bk->meas, bk->n, as_kf);
        bb_time(&a, bk->t); bb_meas(&a, bk->meas, bk->n); bb_u32(&a, as_kf ? 1u : 0u);
        ok_vg_pose_values(full, nid, pose7); ok_vg_sb_values(full, nid, sb_new);
        bb_u64(&r, nid); bb_f64n(&r, pose7, 7); bb_f64n(&r, sb_new, 9);
        tr(b, 1, OK_M_PROP, &a, &r); bb_free(&a); bb_free(&r);
        if (!exists) {
            uint64_t kf; double Tsk7[7], vsk[3], kfpose[7], vtmp[3];
            ok_tf T_Sk_S, T_WSk, kfc;
            ok_vg_anystate_get(rt, bk->id, &kf, Tsk7, vsk);
            ok_tf_set_coeffs(&T_Sk_S, Tsk7, 1);
            ok_vg_pose_values(rt, kf, kfpose); ok_tf_convert(&kfc, kfpose);
            ok_tf_mul(&T_Wnew_Wold, &kfc, &T_WSk, 1);
            ok_tf_mul(&T_WSk, &T_Sk_S, &T_WS, 1);
            ok_vg_sb_values(full, bk->id, sb);
            ok_m3_mulv(T_WSk.C, vsk, vtmp);
            sb[0] = vtmp[0]; sb[1] = vtmp[1]; sb[2] = vtmp[2];
        } else {
            /* T_S0S1 = rt.pose(old).inverse() * rt.pose(id) in TransformationCacheless arithmetic; T_WS = full.pose(old) * T_S0S1
             * (cacheless again), assigned to a cached Transformation */
            ok_tf rtc_old, rtc_new, inv_c, s01, full_old, prod, inv_full;
            double sb_old[9], v_S[3], vtmp[3], p_old[7], p_new[7], p_full_old[7], c7[7], Cm[9], v3[3];
            ok_vg_pose_values(rt, old_id, p_old); ok_vg_pose_values(rt, bk->id, p_new); ok_vg_pose_values(full, old_id, p_full_old);
            ok_tf_set_coeffs(&rtc_old, p_old, 0); ok_tf_set_coeffs(&rtc_new, p_new, 0);
            ok_tf_inverse(&rtc_old, &inv_c, 0);
            ok_tf_mul(&inv_c, &rtc_new, &s01, 0);
            ok_tf_set_coeffs(&full_old, p_full_old, 0);
            ok_tf_mul(&full_old, &s01, &prod, 0);
            tf_to_coeffs(&prod, c7); ok_tf_convert(&T_WS, c7);
            ok_vg_sb_values(rt, bk->id, sb_old);
            ok_vg_sb_values(full, bk->id, sb);
            ok_tf_inverse(&rtc_old, &inv_full, 0);                        /* pose(old).inverse().C(): toRotationMatrix of the inverse */
            ok_quat_to_mat3(&inv_full.q, Cm);
            v3[0] = sb_old[0]; v3[1] = sb_old[1]; v3[2] = sb_old[2];
            ok_m3_mulv(Cm, v3, v_S);
            ok_m3_mulv(T_WS.C, v_S, vtmp);
            sb[0] = vtmp[0]; sb[1] = vtmp[1]; sb[2] = vtmp[2];
        }
        tf_to_coeffs(&T_WS, pose7);
        w_set_pose(b, 1, bk->id, pose7);
        w_set_sb(b, 1, bk->id, sb);
    }
    for (i = 0; i < b->nbl; ++i) free(b->bl[i].meas);
    b->nbl = 0;

    for (i = 0; i < b->nelim; ++i) {
        bb a, r; double T[7], v3[3]; uint64_t kf = 0, h = 0;
        w_remove_all_observations(b, 1, b->elim[i].id);
        memset(&a, 0, sizeof a); memset(&r, 0, sizeof r);
        ok_vg_eliminate_state_by_imu_merge_h(full, b->elim[i].id, b->elim[i].ref, &kf, T, v3, &h);
        a_id(&a, b->elim[i].id); a_id(&a, b->elim[i].ref); bb_u64(&r, kf); bb_f64n(&r, T, 7); bb_f64n(&r, v3, 3); bb_u64(&r, h);
        tr(b, 1, OK_M_ELIM, &a, &r); bb_free(&a); bb_free(&r);
    }
    b->nelim = 0;
    /* (the second eliminateStates_ loop of the C++ runs over the just-cleared map) */
    for (i = ok_vg_state_count(full) - 1; i >= 0; --i) {
        ok_vg_state_view fv, rv;
        ok_vg_state_at(full, i, &fv);
        ok_vg_state_find(rt, fv.id, &rv);
        if (fv.pose_fixed && fv.sb_fixed && rv.pose_fixed && rv.sb_fixed) break;
        if (rv.has_prev_imu) {
            bb a; memset(&a, 0, sizeof a);
            ok_vg_poke_sync_imu(rt, full, fv.id);
            bb_u64(&a, 0); bb_u64(&a, fv.id);
            w_poke(b, 0, OK_M_POKE_SYNCIMU, &a); bb_free(&a);
        }
    }

    /* update new landmarks with pose change and insert into full graph */
    nl = ok_vg_landmark_count(rt);
    for (i = 0; i < nl; ++i) {
        ok_vg_lm_view lv;
        ok_vg_landmark_find(rt, ok_vg_landmark_id_at(rt, i), &lv);
        if (!ok_vg_landmark_find(full, lv.id, NULL)) {
            double hn[4];
            ok_tf_mul_v4(&T_Wnew_Wold, lv.hp, hn, 1);
            {   bb a; memset(&a, 0, sizeof a);                    /* realtimeGraph_.setLandmark(id, hp): the 4-vector overload */
                ok_vg_set_landmark(rt, lv.id, hn, 0, 0);
                a_id(&a, lv.id); bb_f64n(&a, hn, 4);
                tr(b, 0, OK_M_SETLM, &a, NULL); bb_free(&a); }
            { ok_vg_lm_view l2; ok_vg_landmark_find(rt, lv.id, &l2);
              w_add_landmark_id(b, 1, lv.id, l2.hp, l2.initialised);
              w_set_landmark_quality(b, 1, lv.id, l2.quality); }
        }
    }

    /* copy the result over now */
    for (i = ok_vg_state_count(full) - 1; i >= 0; --i) {
        ok_vg_state_view fv, rv;
        double T[7], sb[9];
        ok_vg_state_at(full, i, &fv);
        if (!ok_vg_state_find(rt, fv.id, &rv)) return 0;
        if (fv.pose_fixed && fv.sb_fixed && rv.pose_fixed && rv.sb_fixed) break;
        push_id(updated_out, nupdated, &cap, fv.id);
        ok_vg_pose_values(full, fv.id, T); ok_vg_sb_values(full, fv.id, sb);
        w_set_pose(b, 0, fv.id, T);
        w_set_sb(b, 0, fv.id, sb);
        /* extrinsics: only when not fixed (do_extrinsics is off in every shipped config) */
    }

    /* update landmarks */
    nl = ok_vg_landmark_count(full);
    for (i = 0; i < nl; ++i) {
        ok_vg_lm_view lv;
        ok_vg_landmark_find(full, ok_vg_landmark_id_at(full, i), &lv);
        w_set_landmark_full(b, 0, lv.id, lv.hp, lv.initialised);
        w_set_landmark_quality(b, 0, lv.id, lv.quality);
    }

    /* process touched landmarks / observations */
    for (i = 0; i < b->touched_landmarks.n; ++i) {
        const uint64_t lm = b->touched_landmarks.a[i];
        ok_vg_kid* kids; int n, k;
        if (!ok_vg_landmark_find(full, lm, NULL)) continue;
        n = ok_vg_landmark_obs(full, lm, &kids);
        for (k = 0; k < n; ++k) w_remove_observation(b, 1, kids[k]);
        free(kids);
    }
    for (i = 0; i < b->touched_landmarks.n; ++i) {
        const uint64_t lm = b->touched_landmarks.a[i];
        ok_vg_kid* kids; int n, k;
        if (!ok_vg_landmark_find(full, lm, NULL)) continue;
        n = ok_vg_landmark_obs(rt, lm, &kids);
        for (k = 0; k < n; ++k) {
            const ok_reproj_err* err = NULL;
            ok_vg_obs_find(rt, kids[k], NULL, &err, NULL);
            w_add_external_observation(b, 1, lm, kids[k], 1, err);
        }
        free(kids);
    }

    /* remove all other edges */
    for (i = 0; i < b->touched_states.n; ++i) {
        const uint64_t sid = b->touched_states.a[i];
        uint64_t (*pr)[2]; int n, k;
        if (!ok_vg_state_find(full, sid, NULL)) continue;               /* later deleted */
        n = ok_vg_state_links(full, sid, 0, &pr);
        for (k = 0; k < n; ++k) {
            bb a; memset(&a, 0, sizeof a);
            ok_vg_remove_relative_pose_constraint(full, pr[k][0], pr[k][1]);
            a_id(&a, pr[k][0]); a_id(&a, pr[k][1]);
            tr(b, 1, OK_M_RMRELPOSE, &a, NULL); bb_free(&a);
        }
        free(pr);
        n = ok_vg_state_links(full, sid, 2, &pr);
        for (k = 0; k < n; ++k) {
            bb a; memset(&a, 0, sizeof a);
            ok_vg_remove_two_pose_const_link(full, pr[k][0], pr[k][1]);
            a_id(&a, pr[k][0]); a_id(&a, pr[k][1]);
            tr(b, 1, OK_M_RMTPC, &a, NULL); bb_free(&a);
        }
        free(pr);
    }
    /* re-add (a link is added once: it is identified by its residual block, here by its state pair) */
    {
        uint64_t (*added_rel)[2] = NULL, (*added_tp)[2] = NULL; int nar = 0, nat = 0, car = 0, cat = 0;
        for (i = 0; i < b->touched_states.n; ++i) {
            const uint64_t sid = b->touched_states.a[i];
            uint64_t (*pr)[2]; int n, k, j;
            if (!ok_vg_state_find(full, sid, NULL)) continue;
            n = ok_vg_state_links(rt, sid, 0, &pr);
            for (k = 0; k < n; ++k) {
                int dup = 0; double T7[7], info[36]; bb a;
                for (j = 0; j < nar; ++j) if (added_rel[j][0] == pr[k][0] && added_rel[j][1] == pr[k][1]) dup = 1;
                if (dup) continue;
                ok_vg_rel_link_get(rt, pr[k][0], pr[k][1], T7, info);
                memset(&a, 0, sizeof a);
                ok_vg_add_relative_pose_constraint(full, pr[k][0], pr[k][1], T7, info);
                a_id(&a, pr[k][0]); a_id(&a, pr[k][1]); bb_f64n(&a, T7, 7); bb_f64n(&a, info, 36);
                tr(b, 1, OK_M_ADDRELPOSE, &a, NULL); bb_free(&a);
                if (nar == car) { car = car ? 2 * car : 8; added_rel = (uint64_t(*)[2])realloc(added_rel, sizeof(uint64_t) * 2 * (size_t)car); }
                added_rel[nar][0] = pr[k][0]; added_rel[nar][1] = pr[k][1]; nar++;
            }
            free(pr);
            n = ok_vg_state_links(rt, sid, 1, &pr);
            for (k = 0; k < n; ++k) {
                int dup = 0; ok_tp_std clone; bb a; unsigned char* pl; size_t m; int have;
                for (j = 0; j < nat; ++j) if (added_tp[j][0] == pr[k][0] && added_tp[j][1] == pr[k][1]) dup = 1;
                if (dup) continue;
                have = ok_vg_clone_two_pose_const(rt, pr[k][0], pr[k][1], &clone);
                memset(&a, 0, sizeof a);
                if (have) ok_vg_add_external_two_pose_link(full, pr[k][0], pr[k][1], &clone);
                a_id(&a, pr[k][0]); a_id(&a, pr[k][1]); bb_u32(&a, have ? 1u : 0u);
                if (have) { m = ok_vg_tp_payload(&clone, &pl); bb_raw(&a, pl, m); free(pl); }
                tr(b, 1, OK_M_ADDEXTTP, &a, NULL); bb_free(&a);
                if (nat == cat) { cat = cat ? 2 * cat : 8; added_tp = (uint64_t(*)[2])realloc(added_tp, sizeof(uint64_t) * 2 * (size_t)cat); }
                added_tp[nat][0] = pr[k][0]; added_tp[nat][1] = pr[k][1]; nat++;
            }
            free(pr);
        }
        free(added_rel); free(added_tp);
    }
    idset_clear(&b->touched_states);
    idset_clear(&b->touched_landmarks);

    idset_clear(&b->cur_lc_frames);
    b->is_loop_closure_available = 0;
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * clear / doFinalBa
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vsb_clear(ok_vsb* b) { (void)b; return 0; }          /* ViGraphEstimator::clear is not ported (never exercised) */
int ok_vsb_do_final_ba(ok_vsb* b, int num_iter, double ext_pos_unc, double ext_ori_unc) {
    ok_vg* full = b->g[1];
    int i;
    (void)ext_pos_unc; (void)ext_ori_unc;
    for (i = 0; i < ok_vg_state_count(full); ++i) {
        ok_vg_state_view v;
        ok_vg_state_at(full, i, &v);
        if (v.ntpc > 0) {
            if (!ok_idset_has(&b->key_frames, v.id) && !ok_idset_has(&b->lc_frames, v.id)) ok_idset_add(&b->key_frames, v.id);
            expand_keyframe(b, v.id);
        }
    }
    ok_vg_clean_unobserved_landmarks(full);
    { bb a; memset(&a, 0, sizeof a); tr(b, 1, OK_M_CLEANLM, &a, NULL); bb_free(&a); }
    w_unfreeze_poses(b, 1, 1);
    w_unfreeze_sb(b, 1, 1);
    /* ImuError::redoPropagationAlways = true is not modelled; the optimisation, removeSpeedAndBiasPrior(StateId(1)),
     * the second optimisation and synchronise follow the C++ but are NOT exercised by EuRoC (do_final_ba is off) */
    ok_vsb_optimise_full(b, num_iter, 1, 0);
    return 1;
}
