/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 5d: ViGraph / ViGraphEstimator state and mutations. See ok_vigraph.h for notices and the
 * mutation-log layout. Every function mirrors the C++ method of the same name; the Problem calls are pushed to an event
 * queue in the order the C++ code makes them. */
#include "ok_vigraph.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------------------------
 * small containers: ordered map (sorted array, two u64 keys), hash map (two u64 keys), growable pointer vector
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct ent { uint64_t k0, k1; void* p; } ent;
typedef struct emap { ent* a; int n, cap; } emap;

static int ent_cmp(uint64_t a0, uint64_t a1, uint64_t b0, uint64_t b1) {
    if (a0 != b0) return a0 < b0 ? -1 : 1;
    if (a1 != b1) return a1 < b1 ? -1 : 1;
    return 0;
}
/* index of the key if present (found = 1) or its insertion position (found = 0) */
static int emap_search(const emap* m, uint64_t k0, uint64_t k1, int* found) {
    int lo = 0, hi = m->n;
    while (lo < hi) {
        const int mid = (lo + hi) / 2;
        const int c = ent_cmp(m->a[mid].k0, m->a[mid].k1, k0, k1);
        if (c == 0) { *found = 1; return mid; }
        if (c < 0) lo = mid + 1; else hi = mid;
    }
    *found = 0;
    return lo;
}
static void* emap_get(const emap* m, uint64_t k0, uint64_t k1) {
    int f, i = emap_search(m, k0, k1, &f);
    return f ? m->a[i].p : NULL;
}
static int emap_has(const emap* m, uint64_t k0, uint64_t k1) { int f; emap_search(m, k0, k1, &f); return f; }
static void emap_put(emap* m, uint64_t k0, uint64_t k1, void* p) {
    int f, i = emap_search(m, k0, k1, &f);
    if (f) { m->a[i].p = p; return; }
    if (m->n == m->cap) { m->cap = m->cap ? 2 * m->cap : 8; m->a = (ent*)realloc(m->a, sizeof(ent) * (size_t)m->cap); }
    memmove(&m->a[i + 1], &m->a[i], sizeof(ent) * (size_t)(m->n - i));
    m->a[i].k0 = k0; m->a[i].k1 = k1; m->a[i].p = p;
    m->n++;
}
static int emap_del(emap* m, uint64_t k0, uint64_t k1) {
    int f, i = emap_search(m, k0, k1, &f);
    if (!f) return 0;
    memmove(&m->a[i], &m->a[i + 1], sizeof(ent) * (size_t)(m->n - i - 1));
    m->n--;
    return 1;
}
static void emap_free(emap* m) { free(m->a); m->a = NULL; m->n = m->cap = 0; }

/* open-addressing hash map with tombstones */
typedef struct hent { uint64_t k0, k1; void* p; int state; } hent;   /* 0 empty, 1 used, 2 deleted */
typedef struct hmap { hent* a; int cap, n, used; } hmap;
static uint64_t hash2(uint64_t a, uint64_t b) {
    uint64_t h = a * 0x9E3779B97F4A7C15ULL;
    h ^= (h >> 29); h += b * 0xC2B2AE3D27D4EB4FULL; h ^= (h >> 32); h *= 0x165667B19E3779F9ULL; h ^= (h >> 28);
    return h;
}
static void hmap_grow(hmap* m) {
    hent* old = m->a; const int oc = m->cap, ncap = oc ? (m->n * 4 > oc ? oc * 2 : oc) : 1024;
    int i;
    m->a = (hent*)calloc((size_t)ncap, sizeof(hent)); m->cap = ncap; m->n = 0; m->used = 0;
    for (i = 0; i < oc; ++i)
        if (old[i].state == 1) {
            uint64_t h = hash2(old[i].k0, old[i].k1) & (uint64_t)(ncap - 1);
            while (m->a[h].state) h = (h + 1) & (uint64_t)(ncap - 1);
            m->a[h] = old[i]; m->n++; m->used++;
        }
    free(old);
}
static void* hmap_get(const hmap* m, uint64_t k0, uint64_t k1) {
    uint64_t h;
    if (!m->cap) return NULL;
    h = hash2(k0, k1) & (uint64_t)(m->cap - 1);
    while (m->a[h].state) {
        if (m->a[h].state == 1 && m->a[h].k0 == k0 && m->a[h].k1 == k1) return m->a[h].p;
        h = (h + 1) & (uint64_t)(m->cap - 1);
    }
    return NULL;
}
static void hmap_put(hmap* m, uint64_t k0, uint64_t k1, void* p) {
    uint64_t h;
    if (!m->cap || (m->used + 1) * 2 > m->cap) hmap_grow(m);
    h = hash2(k0, k1) & (uint64_t)(m->cap - 1);
    while (m->a[h].state) {
        if (m->a[h].state == 1 && m->a[h].k0 == k0 && m->a[h].k1 == k1) { m->a[h].p = p; return; }
        h = (h + 1) & (uint64_t)(m->cap - 1);
    }
    m->a[h].k0 = k0; m->a[h].k1 = k1; m->a[h].p = p; m->a[h].state = 1; m->n++; m->used++;
}
static int hmap_del(hmap* m, uint64_t k0, uint64_t k1) {
    uint64_t h;
    if (!m->cap) return 0;
    h = hash2(k0, k1) & (uint64_t)(m->cap - 1);
    while (m->a[h].state) {
        if (m->a[h].state == 1 && m->a[h].k0 == k0 && m->a[h].k1 == k1) { m->a[h].state = 2; m->n--; return 1; }
        h = (h + 1) & (uint64_t)(m->cap - 1);
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------
 * graph objects
 * ---------------------------------------------------------------------------------------------------------------- */
#define MAXCAM OK_TP_MAXEXTR

typedef struct obs {            /* Observation: error term, residual block (= this object), landmark */
    ok_vg_kid kid;
    uint64_t lm;
    ok_reproj_err* err;
    int loss;                   /* 1 = Cauchy loss, 0 = none */
} obs;
typedef struct imulink {        /* ImuLink: shared by the previous state's nextImuLink and the next state's previousImuLink */
    ok_imu_error* e;
    int refs;
} imulink;
typedef struct tplink {         /* TwoPoseLink */
    uint64_t state0, state1;
    ok_twopose* term;
    int refs;
} tplink;
typedef struct tpclink {        /* TwoPoseConstLink */
    uint64_t state0, state1;
    ok_tp_std term;
    int refs;
} tpclink;
typedef struct rlink {          /* RelativePoseLink */
    uint64_t state0, state1;
    ok_relpose_err term;
    int refs;
} rlink;
typedef struct prior_pose { int has; ok_pose_err e; } prior_pose;
typedef struct prior_sb { int has; ok_sab_err e; } prior_sb;

typedef struct state {
    uint64_t id;
    ok_time ts;
    int is_kf;
    ok_vg_blk *pose, *sb, *extr[MAXCAM];
    emap obs;                   /* KeypointIdentifier -> obs */
    imulink *next_imu, *prev_imu;
    prior_pose* pose_prior;     /* allocated when present: its address is the residual block */
    prior_sb* sb_prior;
    emap tp, tpc, rel;          /* other state id -> link */
} state;
typedef struct landmark {
    uint64_t id;
    ok_vg_blk* hp;
    emap obs;
    double quality;
    int classification;
} landmark;
typedef struct anystate { uint64_t kf; ok_time ts; ok_tf T_Sk_S; double v_Sk[3]; } anystate;

typedef enum { RK_OBS = 1, RK_IMU, RK_POSEPRIOR, RK_SBPRIOR, RK_TP, RK_TPC, RK_REL, RK_FIX } rkind;
typedef struct rdesc { int kind; void* term; int loss, nb; uint64_t blk[OK_PB_MAXB]; int type; } rdesc;

#define EVCAP_INIT 256
struct ok_vg {
    int ncam; int do_ext[MAXCAM]; double sigma_r[MAXCAM], sigma_alpha[MAXCAM];
    int has_imu; ok_vg_imu_cfg imu; ok_imu_params imu_p;
    emap states;                /* StateId -> state* */
    emap landmarks;             /* LandmarkId -> landmark* */
    hmap observations;          /* kid -> obs* */
    emap anystates;             /* StateId -> anystate* */
    ok_problem pb;
    hmap resid;                 /* residual block handle -> rdesc* */
    ok_vg_event* ev; int nev, capev;
    /* covisibilities */
    int covis_computed;
    uint64_t (*co_ab)[2]; int* co_cnt; int n_co, cap_co;
    uint64_t* visible; int n_vis, cap_vis;
    /* MST scratch */
    uint64_t* mst_ids; int n_mst_ids; emap mst_idx; int (*mst_edges)[2]; int n_mst_edges;
    /* the initial-fixation PoseError of ViSlamBackend::optimiseRealtimeGraph */
    ok_pose_err* fix_term; uint64_t fix_rb;
    emap blkq;
    int solver_type; double ftol;   /* ceres Solver::Options (linear_solver_type, function_tolerance) as ViGraph::optimise logs them */
};

static void* xmalloc(size_t n) { void* p = malloc(n ? n : 1); return p; }
static void* xcalloc(size_t n) { void* p = calloc(1, n ? n : 1); return p; }

/* ---- events ---- */
static ok_vg_event* ev_new(ok_vg* g, int kind) {
    ok_vg_event* e;
    if (g->nev == g->capev) { g->capev = g->capev ? 2 * g->capev : EVCAP_INIT; g->ev = (ok_vg_event*)realloc(g->ev, sizeof(ok_vg_event) * (size_t)g->capev); }
    e = &g->ev[g->nev++];
    memset(e, 0, sizeof *e);
    e->kind = kind;
    return e;
}
uint64_t ok_vg_problem_events(ok_vg* g, const ok_vg_event** ev, int* n) { *ev = g->ev; *n = g->nev; return 0; }
void ok_vg_events_clear(ok_vg* g) { g->nev = 0; }
const ok_problem* ok_vg_problem(const ok_vg* g) { return &g->pb; }

/* ---- Problem wrappers ---- */
static uint64_t H(const void* p) { return (uint64_t)(uintptr_t)p; }

static void p_add_param(ok_vg* g, ok_vg_blk* b, int manifold) {   /* Problem::AddParameterBlock(values, size[, manifold]) */
    ok_vg_event* e;
    ok_problem_add_parameter_block(&g->pb, H(b), b->size);
    e = ev_new(g, OK_P_ADDPARAM); e->a = H(b); e->b = (uint64_t)b->size;
    if (manifold) {
        ok_problem_set_manifold(&g->pb, H(b), (uint64_t)manifold);
        e = ev_new(g, OK_P_SETMANIFOLD); e->a = H(b); e->b = (uint64_t)manifold;
    }
}
static void p_add_resid(ok_vg* g, void* rb, int kind, void* term, int type, int loss, int nb, ok_vg_blk* const* blks) {
    ok_vg_event* e = ev_new(g, OK_P_ADDRESID);
    rdesc* d = (rdesc*)xcalloc(sizeof(rdesc));
    uint64_t v[OK_PB_MAXB];
    int i;
    for (i = 0; i < nb; ++i) { v[i] = H(blks[i]); e->v[i] = v[i]; d->blk[i] = v[i]; }
    e->a = H(rb); e->loss = loss; e->nb = nb;
    d->kind = kind; d->term = term; d->loss = loss; d->nb = nb; d->type = type;
    ok_problem_add_residual_block(&g->pb, H(rb), 0, (uint64_t)loss, nb, v);
    hmap_put(&g->resid, H(rb), 0, d);
}
static void p_rm_resid(ok_vg* g, void* rb) {
    ok_vg_event* e = ev_new(g, OK_P_RMRESID);
    rdesc* d = (rdesc*)hmap_get(&g->resid, H(rb), 0);
    e->a = H(rb);
    ok_problem_remove_residual_block(&g->pb, H(rb));
    if (d) { free(d); hmap_del(&g->resid, H(rb), 0); }
}
static void p_rm_param(ok_vg* g, ok_vg_blk* b) {
    ok_vg_event* e = ev_new(g, OK_P_RMPARAM);
    e->a = H(b);
    ok_problem_remove_parameter_block(&g->pb, H(b));
}
static void p_set_const(ok_vg* g, ok_vg_blk* b, int c) {
    ok_vg_event* e = ev_new(g, c ? OK_P_SETCONST : OK_P_SETVAR);
    e->a = H(b);
    ok_problem_set_constant(&g->pb, H(b), c);
}
static int p_has(const ok_vg* g, const ok_vg_blk* b) { return ok_problem_find_param(&g->pb, H(b)) >= 0; }

/* resid types as ok_solve.h */
enum { T_REPROJ = 1, T_IMU = 2, T_POSE = 3, T_SAB = 4, T_REL = 5, T_TP = 7, T_TPC = 8 };

/* ---- construction ---- */
ok_vg* ok_vg_new(void) {
    ok_vg* g = (ok_vg*)xcalloc(sizeof(ok_vg));
    ok_problem_init(&g->pb);
    g->covis_computed = 1;      /* "init with true since no observations in the beginning" */
    g->solver_type = 2;         /* ViGraph ctor: SPARSE_NORMAL_CHOLESKY */
    g->ftol = 1e-6;             /* ceres default */
    return g;
}

static ok_vg_blk* blk_new(int size, uint64_t id, ok_time ts) {
    ok_vg_blk* b = (ok_vg_blk*)xcalloc(sizeof(ok_vg_blk));
    b->size = size; b->id = id; b->ts = ts;
    return b;
}

void ok_vg_free(ok_vg* g) {
    int i;
    if (!g) return;
    for (i = 0; i < g->resid.cap; ++i) if (g->resid.a[i].state == 1) free(g->resid.a[i].p);
    ok_problem_free(&g->pb);
    free(g->ev); free(g->resid.a);
    free(g);
}

int ok_vg_add_camera(ok_vg* g, int do_extrinsics, double sigma_r, double sigma_alpha) {
    if (g->ncam >= MAXCAM) return -1;
    g->do_ext[g->ncam] = do_extrinsics; g->sigma_r[g->ncam] = sigma_r; g->sigma_alpha[g->ncam] = sigma_alpha;
    return g->ncam++;
}
int ok_vg_add_imu(ok_vg* g, const ok_vg_imu_cfg* c) {
    if (g->has_imu) return -1;
    g->imu = *c; g->has_imu = 1;
    g->imu_p.sigma_g_c = c->sigma_g_c; g->imu_p.sigma_a_c = c->sigma_a_c;
    g->imu_p.sigma_gw_c = c->sigma_gw_c; g->imu_p.sigma_aw_c = c->sigma_aw_c;
    g->imu_p.g = c->g; g->imu_p.g_max = c->g_max; g->imu_p.a_max = c->a_max;
    return 0;
}

/* ---- lookups ---- */
static state* st_get(const ok_vg* g, uint64_t id) { return (state*)emap_get(&g->states, id, 0); }
static landmark* lm_get(const ok_vg* g, uint64_t id) { return (landmark*)emap_get(&g->landmarks, id, 0); }
static obs* ob_get(const ok_vg* g, ok_vg_kid k) { return (obs*)hmap_get(&g->observations, k.frame, ((uint64_t)k.cam << 32) | k.kp); }
static uint64_t kid_k1(ok_vg_kid k) { return ((uint64_t)k.cam << 32) | k.kp; }
static ok_vg_kid kid_of(uint64_t k0, uint64_t k1) { ok_vg_kid k; k.frame = k0; k.cam = (uint32_t)(k1 >> 32); k.kp = (uint32_t)(k1 & 0xffffffffu); return k; }

/* ------------------------------------------------------------------------------------------------------------------
 * addStatesInitialise / addStatesPropagate
 * ---------------------------------------------------------------------------------------------------------------- */
static void anystate_add(ok_vg* g, uint64_t id, ok_time ts) {
    anystate* a = (anystate*)xcalloc(sizeof(anystate));
    a->ts = ts;
    ok_tf_identity(&a->T_Sk_S);          /* Transformation::Identity(), v_Sk = 0 */
    emap_put(&g->anystates, id, 0, a);
}

uint64_t ok_vg_add_states_initialise(ok_vg* g, ok_time t, const ok_imu_meas* meas, size_t n, int ncam, const double (*T_SC)[7]) {
    (void)ncam;
    state* s = (state*)xcalloc(sizeof(state));
    double T7[7], sbv[9], diag[6], pe[7];
    ok_tf Tws;
    int i;
    const uint64_t id = 1;
    s->id = id; s->ts = t; s->is_kf = 1;
    /* gravity alignment from the mean accelerometer reading, T_WS.oplus(-poseIncrement) */
    ok_imu_init_pose(meas, n, T7);
    s->pose = blk_new(7, id, t);
    memcpy(s->pose->x, T7, sizeof T7);
    for (i = 0; i < 9; ++i) sbv[i] = 0.0;
    for (i = 0; i < 3; ++i) { sbv[6 + i] = g->imu.a0[i]; sbv[3 + i] = g->imu.g0[i]; }
    s->sb = blk_new(9, id, t);
    memcpy(s->sb->x, sbv, sizeof sbv);
    p_add_param(g, s->pose, 1);
    p_add_param(g, s->sb, 0);
    for (i = 0; i < g->ncam; ++i) {
        s->extr[i] = blk_new(7, id, t);
        memcpy(s->extr[i]->x, T_SC[i], sizeof(double) * 7);
        p_add_param(g, s->extr[i], 1);
    }
    /* priors: yaw and pitch free, position pinned (information 1e8), roll... as upstream */
    for (i = 0; i < 6; ++i) diag[i] = 1.0;
    diag[0] = 1.0e8; diag[1] = 1.0e8; diag[2] = 1.0e8;
    diag[3] = 0.0; diag[4] = 0.0;
    diag[5] = 1.0e2;
    memcpy(pe, s->pose->x, sizeof pe);
    ok_tf_set_coeffs(&Tws, pe, 1);
    s->pose_prior = (prior_pose*)xcalloc(sizeof(prior_pose));
    s->pose_prior->has = 1;
    ok_pose_err_init_diag(&s->pose_prior->e, &Tws, diag);
    s->sb_prior = (prior_sb*)xcalloc(sizeof(prior_sb));
    s->sb_prior->has = 1;
    ok_sab_err_init_var(&s->sb_prior->e, sbv, 0.1, g->imu.sigma_bg * g->imu.sigma_bg, g->imu.sigma_ba * g->imu.sigma_ba);
    { ok_vg_blk* b1[1];
      b1[0] = s->pose; p_add_resid(g, s->pose_prior, RK_POSEPRIOR, &s->pose_prior->e, T_POSE, 0, 1, b1);
      b1[0] = s->sb; p_add_resid(g, s->sb_prior, RK_SBPRIOR, &s->sb_prior->e, T_SAB, 0, 1, b1); }
    for (i = 0; i < g->ncam; ++i) {
        if (g->do_ext[i]) return 0;          /* online extrinsics calibration: not used by any shipped config */
        p_set_const(g, s->extr[i], 1);
        s->extr[i]->fixed = 1;
    }
    emap_put(&g->states, id, 0, s);
    anystate_add(g, id, t);
    return id;
}

uint64_t ok_vg_add_states_propagate(ok_vg* g, ok_time t, const ok_imu_meas* meas, size_t n, int is_keyframe) {
    state* last = (state*)g->states.a[g->states.n - 1].p;
    state* s = (state*)xcalloc(sizeof(state));
    const uint64_t id = last->id + 1;
    double T7[7], sbv[9];
    ok_tf T;
    imulink* L;
    ok_vg_blk* b4[4];
    int i;
    s->id = id; s->ts = t; s->is_kf = is_keyframe;
    ok_tf_convert(&T, last->pose->x);                       /* T_WS = lastState.pose->estimate() (cacheless -> cached) */
    memcpy(T7, T.r, sizeof(double) * 3);
    T7[3] = T.q.x; T7[4] = T.q.y; T7[5] = T.q.z; T7[6] = T.q.w;
    memcpy(sbv, last->sb->x, sizeof sbv);
    ok_imu_propagation(meas, n, &g->imu_p, T7, sbv, last->ts, t, NULL, NULL);
    s->pose = blk_new(7, id, t); memcpy(s->pose->x, T7, sizeof T7);
    p_add_param(g, s->pose, 1);
    s->sb = blk_new(9, id, t); memcpy(s->sb->x, sbv, sizeof sbv);
    p_add_param(g, s->sb, 0);
    L = (imulink*)xcalloc(sizeof(imulink));
    L->e = (ok_imu_error*)xcalloc(sizeof(ok_imu_error));
    ok_imu_error_init(L->e, meas, n, &g->imu_p, last->ts, t);
    L->refs = 2;
    b4[0] = last->pose; b4[1] = last->sb; b4[2] = s->pose; b4[3] = s->sb;
    p_add_resid(g, L, RK_IMU, L->e, T_IMU, 0, 4, b4);
    last->next_imu = L; s->prev_imu = L;
    for (i = 0; i < g->ncam; ++i) s->extr[i] = last->extr[i];      /* re-use the same extrinsics */
    emap_put(&g->states, id, 0, s);
    anystate_add(g, id, t);
    return id;
}

/* ------------------------------------------------------------------------------------------------------------------
 * landmarks and observations
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vg_add_landmark_id(ok_vg* g, uint64_t id, const double hp[4], int initialised) {
    landmark* l = (landmark*)xcalloc(sizeof(landmark));
    l->id = id; l->classification = -1;
    l->hp = blk_new(4, id, ok_time_make(0, 0)); memcpy(l->hp->x, hp, sizeof(double) * 4); l->hp->initialised = initialised;
    p_add_param(g, l->hp, 2);
    emap_put(&g->landmarks, id, 0, l);
    return 1;
}
uint64_t ok_vg_add_landmark(ok_vg* g, const double hp[4], int initialised) {
    const uint64_t id = g->landmarks.n ? g->landmarks.a[g->landmarks.n - 1].k0 + 1 : 1;   /* always increase the highest ID by 1 */
    ok_vg_add_landmark_id(g, id, hp, initialised);
    return id;
}

int ok_vg_remove_observation(ok_vg* g, ok_vg_kid kid) {
    obs* o = ob_get(g, kid);
    landmark* l;
    state* s;
    if (!o) return 0;
    p_rm_resid(g, o);
    l = lm_get(g, o->lm);
    s = st_get(g, kid.frame);
    if (l) emap_del(&l->obs, kid.frame, kid_k1(kid));
    if (s) emap_del(&s->obs, kid.frame, kid_k1(kid));
    hmap_del(&g->observations, kid.frame, kid_k1(kid));
    free(o->err); free(o);
    g->covis_computed = 0;
    return 1;
}

int ok_vg_remove_landmark(ok_vg* g, uint64_t id) {
    landmark* l = lm_get(g, id);
    int i;
    if (!l) return 0;
    for (i = 0; i < l->obs.n; ++i) {            /* remove all observations (the landmark's own map stays until it is erased) */
        obs* o = (obs*)l->obs.a[i].p;
        state* s = st_get(g, o->kid.frame);
        p_rm_resid(g, o);
        hmap_del(&g->observations, o->kid.frame, kid_k1(o->kid));
        if (s) emap_del(&s->obs, o->kid.frame, kid_k1(o->kid));
        free(o->err); free(o);
    }
    p_rm_param(g, l->hp);
    emap_free(&l->obs);
    emap_del(&g->landmarks, id, 0);
    free(l);
    return 1;
}

int ok_vg_set_landmark_initialised(ok_vg* g, uint64_t id, int initialised) { landmark* l = lm_get(g, id); if (!l) return 0; l->hp->initialised = initialised; return 1; }
int ok_vg_set_landmark_quality(ok_vg* g, uint64_t id, double q) { landmark* l = lm_get(g, id); if (!l) return 0; l->quality = q; return 1; }
int ok_vg_set_landmark_classification(ok_vg* g, uint64_t id, int c) { landmark* l = lm_get(g, id); if (!l) return 0; l->classification = c; return 1; }
int ok_vg_set_landmark(ok_vg* g, uint64_t id, const double hp[4], int has_init, int initialised) {
    landmark* l = lm_get(g, id);
    if (!l) return 0;
    memcpy(l->hp->x, hp, sizeof(double) * 4);
    if (has_init) l->hp->initialised = initialised;
    return 1;
}

static void obs_register(ok_vg* g, obs* o, landmark* l, state* s) {
    hmap_put(&g->observations, o->kid.frame, kid_k1(o->kid), o);
    emap_put(&l->obs, o->kid.frame, kid_k1(o->kid), o);
    emap_put(&s->obs, o->kid.frame, kid_k1(o->kid), o);
    g->covis_computed = 0;
}
static void obs_add_resid(ok_vg* g, obs* o, state* s, landmark* l) {
    ok_vg_blk* b3[3];
    b3[0] = s->pose; b3[1] = l->hp; b3[2] = s->extr[o->kid.cam];
    p_add_resid(g, o, RK_OBS, o->err, T_REPROJ, o->loss, 3, b3);
}

int ok_vg_add_observation(ok_vg* g, uint64_t lm, ok_vg_kid kid, int use_cauchy, const ok_cam* cam, const double meas[2], double size) {
    landmark* l = lm_get(g, lm);
    state* s = st_get(g, kid.frame);
    obs* o;
    double info[4];
    if (!l || !s) return 0;
    /* information = I * (64 / size^2) */
    { const double f = 64.0 / (size * size);
      info[0] = 1.0 * f; info[1] = 0.0 * f; info[2] = 0.0 * f; info[3] = 1.0 * f; }
    o = (obs*)xcalloc(sizeof(obs));
    o->err = (ok_reproj_err*)xcalloc(sizeof(ok_reproj_err));
    ok_reproj_err_init(o->err, cam, meas, info);
    o->kid = kid; o->lm = lm; o->loss = use_cauchy ? 1 : 0;
    obs_add_resid(g, o, s, l);
    obs_register(g, o, l, s);
    return 1;
}

int ok_vg_add_external_observation(ok_vg* g, uint64_t lm, ok_vg_kid kid, int use_cauchy, const ok_reproj_err* src) {
    landmark* l = lm_get(g, lm);
    state* s = st_get(g, kid.frame);
    obs* o;
    if (!l || !s) return 0;
    o = (obs*)xcalloc(sizeof(obs));
    o->err = (ok_reproj_err*)xmalloc(sizeof(ok_reproj_err));
    *o->err = *src;                                  /* reprojectionError->clone() */
    o->kid = kid; o->lm = lm; o->loss = use_cauchy ? 1 : 0;
    obs_add_resid(g, o, s, l);
    obs_register(g, o, l, s);
    return 1;
}

int ok_vg_remove_all_observations(ok_vg* g, uint64_t state_id) {
    state* s = st_get(g, state_id);
    ok_vg_kid* ks;
    int i, n;
    if (!s) return 0;
    n = s->obs.n;
    ks = (ok_vg_kid*)xmalloc(sizeof(ok_vg_kid) * (size_t)n);
    for (i = 0; i < n; ++i) ks[i] = kid_of(s->obs.a[i].k0, s->obs.a[i].k1);   /* copy: the loop erases from the map */
    for (i = 0; i < n; ++i) ok_vg_remove_observation(g, ks[i]);
    free(ks);
    return 1;
}

int ok_vg_clean_unobserved_landmarks(ok_vg* g) {
    int ctr = 0, i = 0;
    while (i < g->landmarks.n) {
        landmark* l = (landmark*)g->landmarks.a[i].p;
        if (l->obs.n <= 1) {
            if (l->obs.n == 1) {
                const ent e = l->obs.a[0];
                ok_vg_remove_observation(g, kid_of(e.k0, e.k1));
            }
            p_rm_param(g, l->hp);
            emap_free(&l->obs);
            memmove(&g->landmarks.a[i], &g->landmarks.a[i + 1], sizeof(ent) * (size_t)(g->landmarks.n - i - 1));
            g->landmarks.n--;
            free(l);
            ctr++;
        } else ++i;
    }
    return ctr;
}

/* cleanUnobservedLandmarks(&removed): removed[lm] = {the single observation, if any}, ascending landmark id */
int ok_vg_clean_unobserved_landmarks_ex(ok_vg* g, uint64_t** lms, ok_vg_kid** kids, int** has_kid, int* nrem) {
    int ctr = 0, i = 0, cap = 0;
    *lms = NULL; *kids = NULL; *has_kid = NULL; *nrem = 0;
    while (i < g->landmarks.n) {
        landmark* l = (landmark*)g->landmarks.a[i].p;
        if (l->obs.n <= 1) {
            if (*nrem == cap) {
                cap = cap ? 2 * cap : 16;
                *lms = (uint64_t*)realloc(*lms, sizeof(uint64_t) * (size_t)cap);
                *kids = (ok_vg_kid*)realloc(*kids, sizeof(ok_vg_kid) * (size_t)cap);
                *has_kid = (int*)realloc(*has_kid, sizeof(int) * (size_t)cap);
            }
            (*lms)[*nrem] = l->id; (*has_kid)[*nrem] = l->obs.n == 1;
            memset(&(*kids)[*nrem], 0, sizeof(ok_vg_kid));
            if (l->obs.n == 1) {
                const ent e = l->obs.a[0];
                (*kids)[*nrem] = kid_of(e.k0, e.k1);
                ok_vg_remove_observation(g, kid_of(e.k0, e.k1));
            }
            (*nrem)++;
            p_rm_param(g, l->hp);
            emap_free(&l->obs);
            memmove(&g->landmarks.a[i], &g->landmarks.a[i + 1], sizeof(ent) * (size_t)(g->landmarks.n - i - 1));
            g->landmarks.n--;
            free(l);
            ctr++;
        } else ++i;
    }
    return ctr;
}

int ok_vg_merge_landmark(ok_vg* g, uint64_t from, uint64_t into) {
    landmark* lf = lm_get(g, from);
    landmark* li = lm_get(g, into);
    ent* snap;
    int i, n;
    if (!lf || !li) return 0;
    n = lf->obs.n;
    snap = (ent*)xmalloc(sizeof(ent) * (size_t)n);
    if (n) memcpy(snap, lf->obs.a, sizeof(ent) * (size_t)n);
    for (i = 0; i < n; ++i) {
        obs* old = (obs*)snap[i].p;
        const int use_loss = old->loss;          /* GetLossFunctionForResidualBlock */
        ok_vg_kid kid = old->kid;
        ok_reproj_err* err = old->err;
        state* s;
        obs* o;
        old->err = NULL;                          /* the error term survives: it is re-added */
        ok_vg_remove_observation(g, kid);
        s = st_get(g, kid.frame);
        o = (obs*)xcalloc(sizeof(obs));
        o->err = err; o->kid = kid; o->lm = into; o->loss = use_loss;
        obs_add_resid(g, o, s, li);
        hmap_put(&g->observations, kid.frame, kid_k1(kid), o);
        emap_put(&li->obs, kid.frame, kid_k1(kid), o);
        emap_put(&s->obs, kid.frame, kid_k1(kid), o);
    }
    free(snap);
    ok_vg_remove_landmark(g, from);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * covisibilities
 * ---------------------------------------------------------------------------------------------------------------- */
static int cmp_u64(const void* a, const void* b) { const uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b; return x < y ? -1 : (x > y); }
static int cmp_pair(const void* a, const void* b) {
    const uint64_t* x = (const uint64_t*)a; const uint64_t* y = (const uint64_t*)b;
    if (x[0] != y[0]) return x[0] < y[0] ? -1 : 1;
    return x[1] < y[1] ? -1 : (x[1] > y[1]);
}

int ok_vg_compute_covisibilities(ok_vg* g) {
    uint64_t (*pairs)[2] = NULL;
    int npairs = 0, cappairs = 0, li, i, j, k;
    uint64_t* frames = NULL; int capf = 0;
    if (g->covis_computed) return 1;      /* already done previously */
    g->n_co = 0; g->n_vis = 0;
    for (li = 0; li < g->landmarks.n; ++li) {
        landmark* l = (landmark*)g->landmarks.a[li].p;
        int nf = 0;
        if (l->classification == 10 || l->classification == 11) continue;
        if (l->obs.n > capf) { capf = l->obs.n * 2; frames = (uint64_t*)realloc(frames, sizeof(uint64_t) * (size_t)capf); }
        for (i = 0; i < l->obs.n; ++i) {         /* std::set<uint64>: the frame ids, ascending and unique (obs are in kid order) */
            const uint64_t f = l->obs.a[i].k0;
            if (nf == 0 || frames[nf - 1] != f) frames[nf++] = f;
            if (g->n_vis == g->cap_vis) { g->cap_vis = g->cap_vis ? 2 * g->cap_vis : 1024; g->visible = (uint64_t*)realloc(g->visible, sizeof(uint64_t) * (size_t)g->cap_vis); }
            g->visible[g->n_vis++] = f;
        }
        for (i = 0; i < nf; ++i)
            for (j = 0; j < i; ++j) {                  /* *i1 < *i0 */
                if (npairs == cappairs) { cappairs = cappairs ? 2 * cappairs : 4096; pairs = (uint64_t(*)[2])realloc(pairs, sizeof(uint64_t) * 2 * (size_t)cappairs); }
                pairs[npairs][0] = frames[i]; pairs[npairs][1] = frames[j]; npairs++;
            }
    }
    /* visibleFrames_ is a std::set<StateId> */
    qsort(g->visible, (size_t)g->n_vis, sizeof(uint64_t), cmp_u64);
    for (i = 0, k = 0; i < g->n_vis; ++i) if (k == 0 || g->visible[k - 1] != g->visible[i]) g->visible[k++] = g->visible[i];
    g->n_vis = k;
    /* coObservationCounts_[a][b] counts the landmarks that see both frames */
    qsort(pairs, (size_t)npairs, sizeof(uint64_t) * 2, cmp_pair);
    for (i = 0; i < npairs;) {
        j = i;
        while (j < npairs && pairs[j][0] == pairs[i][0] && pairs[j][1] == pairs[i][1]) ++j;
        if (g->n_co == g->cap_co) { g->cap_co = g->cap_co ? 2 * g->cap_co : 1024; g->co_ab = (uint64_t(*)[2])realloc(g->co_ab, sizeof(uint64_t) * 2 * (size_t)g->cap_co); g->co_cnt = (int*)realloc(g->co_cnt, sizeof(int) * (size_t)g->cap_co); }
        g->co_ab[g->n_co][0] = pairs[i][0]; g->co_ab[g->n_co][1] = pairs[i][1]; g->co_cnt[g->n_co] = j - i; g->n_co++;
        i = j;
    }
    free(pairs); free(frames);
    g->covis_computed = 1;
    return g->n_co > 0;
}
int ok_vg_covisibilities_dirty(const ok_vg* g) { return !g->covis_computed; }
int ok_vg_covisibilities(const ok_vg* g, uint64_t i, uint64_t j) {
    uint64_t a, b;
    int lo = 0, hi = g->n_co;
    if (i == j) return 0;
    if (i < j) { a = j; b = i; } else { a = i; b = j; }
    while (lo < hi) {
        const int mid = (lo + hi) / 2;
        const int c = ent_cmp(g->co_ab[mid][0], g->co_ab[mid][1], a, b);
        if (c == 0) return g->co_cnt[mid];
        if (c < 0) lo = mid + 1; else hi = mid;
    }
    return 0;
}
int ok_vg_covis_size(const ok_vg* g) {                      /* number of distinct first frames */
    int i, n = 0;
    for (i = 0; i < g->n_co; ++i) if (i == 0 || g->co_ab[i][0] != g->co_ab[i - 1][0]) n++;
    return n;
}
int ok_vg_covis_pairs(const ok_vg* g, const uint64_t (**ab)[2], const int** count) { *ab = g->co_ab; *count = g->co_cnt; return g->n_co; }
int ok_vg_visible_frames(const ok_vg* g, const uint64_t** ids) { *ids = g->visible; return g->n_vis; }

/* ------------------------------------------------------------------------------------------------------------------
 * relative pose constraints, simple setters
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vg_add_relative_pose_constraint(ok_vg* g, uint64_t id0, uint64_t id1, const double T7[7], const double info_cm[36]) {
    state* s0 = st_get(g, id0);
    state* s1 = st_get(g, id1);
    rlink* r;
    ok_tf T;
    ok_vg_blk* b2[2];
    if (!s0 || !s1) return 0;
    r = (rlink*)xcalloc(sizeof(rlink));
    ok_tf_set_coeffs(&T, T7, 1);
    ok_relpose_err_init_info(&r->term, info_cm, &T);
    r->state0 = id0; r->state1 = id1; r->refs = 2;
    b2[0] = s0->pose; b2[1] = s1->pose;
    p_add_resid(g, r, RK_REL, &r->term, T_REL, 0, 2, b2);
    emap_put(&s0->rel, id1, 0, r);
    emap_put(&s1->rel, id0, 0, r);
    return 1;
}
int ok_vg_remove_relative_pose_constraint(ok_vg* g, uint64_t id0, uint64_t id1) {
    state* s0 = st_get(g, id0);
    state* s1 = st_get(g, id1);
    rlink* r;
    if (!s0 || !s1) return 0;
    r = (rlink*)emap_get(&s0->rel, id1, 0);
    if (!r) return 0;
    p_rm_resid(g, r);
    emap_del(&s0->rel, id1, 0);
    emap_del(&s1->rel, id0, 0);
    free(r);
    return 1;
}

int ok_vg_set_keyframe(ok_vg* g, uint64_t id, int flag) { state* s = st_get(g, id); if (!s) return 0; s->is_kf = flag; return 1; }
int ok_vg_set_pose(ok_vg* g, uint64_t id, const double T7[7]) { state* s = st_get(g, id); if (!s) return 0; memcpy(s->pose->x, T7, sizeof(double) * 7); return 1; }
int ok_vg_set_speed_and_bias(ok_vg* g, uint64_t id, const double sb[9]) { state* s = st_get(g, id); if (!s) return 0; memcpy(s->sb->x, sb, sizeof(double) * 9); return 1; }
int ok_vg_set_extrinsics(ok_vg* g, uint64_t id, int cam, const double T7[7]) { state* s = st_get(g, id); if (!s) return 0; memcpy(s->extr[cam]->x, T7, sizeof(double) * 7); return 1; }
int ok_vg_pose_values(const ok_vg* g, uint64_t id, double out7[7]) { state* s = st_get(g, id); if (!s) return 0; memcpy(out7, s->pose->x, sizeof(double) * 7); return 1; }
int ok_vg_extrinsics_values(const ok_vg* g, uint64_t id, int cam, double out7[7]) { state* s = st_get(g, id); if (!s || cam < 0 || cam >= MAXCAM || !s->extr[cam]) return 0; memcpy(out7, s->extr[cam]->x, sizeof(double) * 7); return 1; }
int ok_vg_sb_values(const ok_vg* g, uint64_t id, double out9[9]) { state* s = st_get(g, id); if (!s) return 0; memcpy(out9, s->sb->x, sizeof(double) * 9); return 1; }

/* ------------------------------------------------------------------------------------------------------------------
 * updateLandmarks
 * ---------------------------------------------------------------------------------------------------------------- */
void ok_vg_update_landmarks(ok_vg* g) {
    int li, i;
    ok_lm_obs* ob = NULL; int capob = 0;
    for (li = 0; li < g->landmarks.n; ++li) {
        landmark* l = (landmark*)g->landmarks.a[li].p;
        double quality = 0.0;
        int init = 0;
        if (l->obs.n > capob) { capob = l->obs.n * 2; ob = (ok_lm_obs*)realloc(ob, sizeof(ok_lm_obs) * (size_t)capob); }
        for (i = 0; i < l->obs.n; ++i) {
            obs* o = (obs*)l->obs.a[i].p;
            state* s = st_get(g, o->kid.frame);
            ob[i].frame_id = o->kid.frame; ob[i].cam = (int)o->kid.cam; ob[i].kp = (int)o->kid.kp;
            memcpy(ob[i].pose, s->pose->x, sizeof(double) * 7);
            memcpy(ob[i].extr, s->extr[o->kid.cam]->x, sizeof(double) * 7);
            ob[i].err = *o->err;
        }
        if (l->obs.n > 0) ok_graph_update_landmark(l->hp->x, ob, l->obs.n, &quality, &init);
        l->hp->initialised = init;
        l->quality = quality > 0.0 ? quality : 0.0;      /* std::max(0.0, quality) */
    }
    free(ob);
}

/* ------------------------------------------------------------------------------------------------------------------
 * eliminateStateByImuMerge
 * ---------------------------------------------------------------------------------------------------------------- */
void ok_vg_imu_copy(ok_imu_error* dst, const ok_imu_error* src) {   /* ImuError::syncFrom */
    ok_imu_meas* m = dst->meas;
    size_t cap = src->n_meas ? src->n_meas : 1;
    ok_imu_error tmp = *src;
    free(m);
    tmp.meas = (ok_imu_meas*)xmalloc(sizeof(ok_imu_meas) * cap);
    if (src->n_meas) memcpy(tmp.meas, src->meas, sizeof(ok_imu_meas) * src->n_meas);
    tmp.cap_meas = cap;
    *dst = tmp;
}

static uint64_t g_elim_hash;
int ok_vg_eliminate_state_by_imu_merge_h(ok_vg* g, uint64_t id, uint64_t ref, uint64_t* kf, double T_Sk_S7[7], double v_Sk_out[3], uint64_t* imu_hash) {
    const int r = ok_vg_eliminate_state_by_imu_merge(g, id, ref, kf, T_Sk_S7, v_Sk_out);
    if (imu_hash) *imu_hash = g_elim_hash;
    return r;
}
int ok_vg_eliminate_state_by_imu_merge(ok_vg* g, uint64_t id, uint64_t ref, uint64_t* kf, double T_Sk_S7[7], double v_Sk_out[3]) {
    state* s = st_get(g, id);
    state *prev, *next, *other;
    int idx, f;
    imulink* L1;
    imulink* L2;
    ok_vg_blk* b4[4];
    ok_tf A, B, Tsk_W, Tsk_S, Tsw;
    double v3[3], c[7];
    anystate* as;
    if (!s) return 0;
    /* also remove relative pose links (in the order of the other state id) */
    while (s->rel.n > 0) {              /* iterate over a copy ordered by the other state id */
        rlink* r = (rlink*)s->rel.a[0].p;
        const uint64_t a = r->state0, b = r->state1;
        ok_vg_remove_relative_pose_constraint(g, a, b);
    }
    idx = emap_search(&g->states, id, 0, &f);
    if (idx == 0 || idx >= g->states.n - 1) return 0;   /* cannot eliminate the first / last state */
    prev = (state*)g->states.a[idx - 1].p;
    next = (state*)g->states.a[idx + 1].p;
    L1 = prev->next_imu;
    L2 = s->next_imu;
    /* merge the IMU measurements (ImuError::append with the current speed and biases) */
    ok_imu_append(L1->e, s->sb->x, L2->e->meas, L2->e->n_meas, L2->e->t1);
    /* remove links in ceres */
    p_rm_resid(g, L2);
    p_rm_resid(g, L1);
    p_rm_param(g, s->pose);
    p_rm_param(g, s->sb);
    { unsigned char* sn; const size_t n = ok_vg_imu_snapshot(L1->e, 0, 1, &sn); g_elim_hash = ok_vg_fnv(sn, n); free(sn); }
    /* re-add the appended IMU error term */
    b4[0] = prev->pose; b4[1] = prev->sb; b4[2] = next->pose; b4[3] = next->sb;
    p_add_resid(g, L1, RK_IMU, L1->e, T_IMU, 0, 4, b4);
    next->prev_imu = L1;
    L1->refs = 2;
    /* anyState: pose and velocity of the removed state in its reference keyframe's sensor frame */
    as = (anystate*)emap_get(&g->anystates, id, 0);
    as->kf = ref;
    other = st_get(g, ref);
    ok_tf_set_coeffs(&A, other->pose->x, 0);
    ok_tf_inverse(&A, &B, 0);                              /* otherstate.pose->estimate().inverse() (cacheless) */
    memcpy(c, B.r, sizeof(double) * 3); c[3] = B.q.x; c[4] = B.q.y; c[5] = B.q.z; c[6] = B.q.w;
    ok_tf_set_coeffs(&Tsk_W, c, 1);                        /* -> Transformation */
    ok_tf_convert(&Tsw, s->pose->x);                       /* states_[stateId].pose->estimate() */
    ok_tf_mul(&Tsk_W, &Tsw, &Tsk_S, 1);                    /* T_Sk_S = T_Sk_W * T_WS */
    as->T_Sk_S = Tsk_S;
    ok_m3_mulv(Tsk_W.C, other->sb->x, v3);                 /* v_Sk = T_Sk_W.C() * v_W */
    memcpy(as->v_Sk, v3, sizeof v3);
    if (kf) *kf = ref;
    if (T_Sk_S7) { memcpy(T_Sk_S7, Tsk_S.r, sizeof(double) * 3); T_Sk_S7[3] = Tsk_S.q.x; T_Sk_S7[4] = Tsk_S.q.y; T_Sk_S7[5] = Tsk_S.q.z; T_Sk_S7[6] = Tsk_S.q.w; }
    if (v_Sk_out) memcpy(v_Sk_out, v3, sizeof v3);
    /* book-keeping: erase the state (its second IMU link dies with it) */
    ok_imu_error_free(L2->e); free(L2->e); free(L2);
    emap_del(&g->states, id, 0);
    emap_free(&s->obs); emap_free(&s->tp); emap_free(&s->tpc); emap_free(&s->rel);
    free(s->pose_prior); free(s->sb_prior);
    free(s);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * freeze / unfreeze
 * ---------------------------------------------------------------------------------------------------------------- */
static int freeze_until(ok_vg* g, uint64_t id, int remove_in_ceres, int which) {   /* which: 0 pose, 1 speed-and-bias */
    int f, idx = emap_search(&g->states, id, 0, &f), first = 1;
    if (!f) return 0;
    for (;; --idx) {
        state* s = (state*)g->states.a[idx].p;
        ok_vg_blk* b = which ? s->sb : s->pose;
        if (b->fixed) {
            if (remove_in_ceres && p_has(g, b)) p_rm_param(g, b);
            break;
        }
        b->fixed = 1;
        if (first || !remove_in_ceres) { p_set_const(g, b, 1); first = 0; }
        else if (p_has(g, b)) p_rm_param(g, b);
        if (idx == 0) break;
    }
    return 1;
}
static int unfreeze_from(ok_vg* g, uint64_t id, int which) {
    int f, i;
    emap_search(&g->states, id, 0, &f);
    if (!f) return 0;
    for (i = g->states.n - 1; i >= 0; --i) {
        state* s = (state*)g->states.a[i].p;
        ok_vg_blk* b = which ? s->sb : s->pose;
        b->fixed = 0;
        p_set_const(g, b, 0);
        if (s->id == id) break;
    }
    return 1;
}
int ok_vg_freeze_poses_until(ok_vg* g, uint64_t id, int r) { return freeze_until(g, id, r, 0); }
int ok_vg_unfreeze_poses_from(ok_vg* g, uint64_t id) { return unfreeze_from(g, id, 0); }
int ok_vg_freeze_sb_until(ok_vg* g, uint64_t id, int r) { return freeze_until(g, id, r, 1); }
int ok_vg_unfreeze_sb_from(ok_vg* g, uint64_t id) { return unfreeze_from(g, id, 1); }

/* ------------------------------------------------------------------------------------------------------------------
 * two-pose links: convertToPoseGraphMst (buildMst = Kruskal), convertToObservations, const links
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct medge { int w, u, v; } medge;
static int cmp_medge(const void* a, const void* b) {       /* std::sort of pair<int, pair<int,int>> */
    const medge* x = (const medge*)a; const medge* y = (const medge*)b;
    if (x->w != y->w) return x->w < y->w ? -1 : 1;
    if (x->u != y->u) return x->u < y->u ? -1 : 1;
    return x->v < y->v ? -1 : (x->v > y->v);
}
static int ds_find(int* parent, int u) { if (u != parent[u]) parent[u] = ds_find(parent, parent[u]); return parent[u]; }

static int build_mst(ok_vg* g, const uint64_t* states, int n, int keyframes_only) {
    int i, k, ne = 0;
    medge* edges;
    int *parent, *rnk, V, cap;
    if (!ok_vg_compute_covisibilities(g)) return -1;       /* success0 must hold */
    free(g->mst_ids); g->mst_ids = NULL; g->n_mst_ids = 0;
    emap_free(&g->mst_idx);
    free(g->mst_edges); g->mst_edges = NULL; g->n_mst_edges = 0;
    g->mst_ids = (uint64_t*)xmalloc(sizeof(uint64_t) * (size_t)n);
    for (i = 0; i < n; ++i) { g->mst_ids[i] = states[i]; emap_put(&g->mst_idx, states[i], 0, (void*)(intptr_t)i); }
    g->n_mst_ids = n;
    edges = (medge*)xmalloc(sizeof(medge) * (size_t)(g->n_co + 1));
    for (k = 0; k < g->n_co; ++k) {
        const uint64_t a = g->co_ab[k][0], b = g->co_ab[k][1];
        int consider = 1;
        if (keyframes_only) { state* sa = st_get(g, a); state* sb = st_get(g, b); consider = sa && sb && sa->is_kf && sb->is_kf; }
        if (consider && emap_has(&g->mst_idx, a, 0) && emap_has(&g->mst_idx, b, 0)) {
            edges[ne].u = (int)(intptr_t)emap_get(&g->mst_idx, a, 0);
            edges[ne].v = (int)(intptr_t)emap_get(&g->mst_idx, b, 0);
            edges[ne].w = -g->co_cnt[k];
            ne++;
        }
    }
    qsort(edges, (size_t)ne, sizeof(medge), cmp_medge);
    V = n; cap = V + 1;
    parent = (int*)xmalloc(sizeof(int) * (size_t)cap); rnk = (int*)xmalloc(sizeof(int) * (size_t)cap);
    for (i = 0; i <= V; ++i) { rnk[i] = 0; parent[i] = i; }
    g->mst_edges = (int(*)[2])xmalloc(sizeof(int) * 2 * (size_t)(ne + 1));
    for (i = 0; i < ne; ++i) {
        const int su = ds_find(parent, edges[i].u), sv = ds_find(parent, edges[i].v);
        if (su != sv) {
            int x, y;
            g->mst_edges[g->n_mst_edges][0] = edges[i].u; g->mst_edges[g->n_mst_edges][1] = edges[i].v; g->n_mst_edges++;
            x = ds_find(parent, su); y = ds_find(parent, sv);       /* DisjointSets::merge(set_u, set_v) */
            if (rnk[x] > rnk[y]) parent[y] = x; else parent[x] = y;
            if (rnk[x] == rnk[y]) rnk[y]++;
        }
    }
    free(edges); free(parent); free(rnk);
    return g->n_mst_edges > 0;
}

/* counters keyed by state id (numEdges) */
static int ne_get(emap* m, uint64_t id) { return (int)(intptr_t)emap_get(m, id, 0); }
static void ne_add(emap* m, uint64_t id, int d) { emap_put(m, id, 0, (void*)(intptr_t)(ne_get(m, id) + d)); }

void ok_vg_mst_result_free(ok_vg_mst_result* r) { free(r->mst); free(r->created); free(r->created_term); free(r->removed_tp); free(r->removed_obs); memset(r, 0, sizeof *r); }

int ok_vg_convert_to_pose_graph_mst(ok_vg* g, const uint64_t* states, int n, const uint64_t* consider, int m, ok_vg_mst_result* out) {
    emap num_edges; memset(&num_edges, 0, sizeof num_edges);
    uint64_t (*to_create)[2] = NULL; int n_create = 0, ei, k;
    int built;
    (void)n;
    memset(out, 0, sizeof *out);
    built = build_mst(g, consider, m, 1);
    if (built < 0) return 0;
    if (g->n_mst_edges == 0) { out->ret = 0; return 0; }
    out->ret = 1;
    out->nmst = g->n_mst_edges;
    out->mst = (uint64_t(*)[2])xmalloc(sizeof(uint64_t) * 2 * (size_t)g->n_mst_edges);
    for (ei = 0; ei < g->n_mst_edges; ++ei) { out->mst[ei][0] = g->mst_ids[g->mst_edges[ei][0]]; out->mst[ei][1] = g->mst_ids[g->mst_edges[ei][1]]; }
    /* number of MST edges per node */
    for (ei = 0; ei < g->n_mst_edges; ++ei) { ne_add(&num_edges, out->mst[ei][0], 1); ne_add(&num_edges, out->mst[ei][1], 1); }
    to_create = (uint64_t(*)[2])xmalloc(sizeof(uint64_t) * 2 * (size_t)(g->n_mst_edges + 1));
    for (ei = 0; ei < g->n_mst_edges; ++ei) {          /* create the links touching the frames to convert; edges are MST graph indices */
        int in_states = 0;
        for (k = 0; k < n; ++k) if (states[k] == out->mst[ei][0] || states[k] == out->mst[ei][1]) in_states = 1;
        if (in_states) { to_create[n_create][0] = (uint64_t)g->mst_edges[ei][0]; to_create[n_create][1] = (uint64_t)g->mst_edges[ei][1]; n_create++; }
    }
    /* always add the longest-term edge */
    {
        const uint64_t id_newest = g->mst_ids[g->n_mst_ids - 1], id_oldest = g->mst_ids[0];
        int old_in = 0;
        for (k = 0; k < n; ++k) if (states[k] == id_oldest) old_in = 1;
        if (old_in && ok_vg_covisibilities(g, id_newest, id_oldest) >= 2 && id_newest != id_oldest) {
            const uint64_t io = (uint64_t)(intptr_t)emap_get(&g->mst_idx, id_oldest, 0), in = (uint64_t)(intptr_t)emap_get(&g->mst_idx, id_newest, 0);
            int already = 0;
            for (k = 0; k < n_create; ++k)
                if ((to_create[k][0] == io && to_create[k][1] == in) || (to_create[k][0] == in && to_create[k][1] == io)) { already = 1; break; }
            if (!already) {
                to_create[n_create][0] = io; to_create[n_create][1] = in; n_create++;
                ne_add(&num_edges, id_oldest, 1); ne_add(&num_edges, id_newest, 1);
            }
        }
    }
    /* make edges */
    for (ei = 0; ei < n_create; ++ei) {
        const uint64_t f0 = g->mst_ids[to_create[ei][0]], f1 = g->mst_ids[to_create[ei][1]];
        const uint64_t ref_id = f0 < f1 ? f0 : f1, other_id = f0 < f1 ? f1 : f0;
        state* rs = st_get(g, ref_id);
        state* os = st_get(g, other_id);
        tplink* link = (tplink*)xcalloc(sizeof(tplink));
        ok_twopose* t = (ok_twopose*)xcalloc(sizeof(ok_twopose));
        int keep_ref, keep_other, in_ref = 0, in_other = 0, c;
        uint64_t *refl = NULL, *othl = NULL; int nref = 0, noth = 0;
        uint64_t* both = NULL; int nboth = 0;
        ent* ref_obs; ent* oth_obs; int nro, noo;
        ok_twopose_init(t, ref_id, other_id, g->ncam, 1);
        link->term = t; link->state0 = ref_id; link->state1 = other_id; link->refs = 2;
        /* an existing link between the two is converted back into observations first (rare) */
        if (emap_has(&rs->tp, other_id, 0)) {
            tplink* old = (tplink*)emap_get(&rs->tp, other_id, 0);
            ok_twopose* ot = old->term;
            int g1, o1, nmarg = 0, nout;
            struct pend { ok_tp_obs ob; } *pend;
            double (*hpw)[4];
            for (g1 = 0; g1 < ot->ngroups; ++g1) for (o1 = 0; o1 < ot->groups[g1].nobs; ++o1) if (ot->groups[g1].obs[o1].is_marginalised) nmarg++;
            pend = (struct pend*)xmalloc(sizeof(struct pend) * (size_t)(nmarg + 1));
            hpw = (double(*)[4])xmalloc(sizeof(double) * 4 * (size_t)(nmarg + 1));
            nmarg = 0;
            for (g1 = 0; g1 < ot->ngroups; ++g1) for (o1 = 0; o1 < ot->groups[g1].nobs; ++o1) if (ot->groups[g1].obs[o1].is_marginalised) pend[nmarg++].ob = ot->groups[g1].obs[o1];
            memcpy(ot->pose_live[0], rs->pose->x, sizeof(double) * 7);
            nout = ok_twopose_convert(ot, rs->pose->x, hpw, nmarg, NULL);
            for (g1 = 0; g1 < nout; ++g1) {
                ok_tp_obs* ob = &pend[g1].ob;
                ok_vg_blk* pose = st_get(g, ob->frame_id)->pose;
                ok_vg_blk* extr = st_get(g, ob->frame_id)->extr[ob->cam];
                if (ob->is_duplication) ok_reproj_err_set_information(&ob->err, ob->err.info);
                ok_twopose_add_observation(t, ob->frame_id, ob->cam, ob->kp, &ob->err, ob->loss, pose->id, pose->x, ob->hpoint_id, hpw[g1],
                                           ob->hp_live_init ? *ob->hp_live_init : ob->hpoint_initialised, extr->id, extr->x, ob->is_duplication);
            }
            free(pend); free(hpw);
            p_rm_resid(g, old);
            emap_del(&rs->tp, other_id, 0);
            emap_del(&os->tp, ref_id, 0);
            ok_twopose_free(ot); free(ot); free(old);
            out->removed_tp = (uint64_t(*)[2])realloc(out->removed_tp, sizeof(uint64_t) * 2 * (size_t)(out->nremoved_tp + 1));
            out->removed_tp[out->nremoved_tp][0] = ref_id; out->removed_tp[out->nremoved_tp][1] = other_id; out->nremoved_tp++;
        }
        keep_ref = ne_get(&num_edges, ref_id) > 1;
        keep_other = ne_get(&num_edges, other_id) > 1;
        for (c = 0; c < n; ++c) { if (states[c] == ref_id) in_ref = 1; if (states[c] == other_id) in_other = 1; }
        if (!in_ref) keep_ref = 1;
        if (!in_other) keep_other = 1;
        /* landmarks observed in both frames (std::set_intersection of the two landmark-id sets) */
        nro = rs->obs.n; noo = os->obs.n;
        ref_obs = (ent*)xmalloc(sizeof(ent) * (size_t)(nro + 1)); if (nro) memcpy(ref_obs, rs->obs.a, sizeof(ent) * (size_t)nro);
        oth_obs = (ent*)xmalloc(sizeof(ent) * (size_t)(noo + 1)); if (noo) memcpy(oth_obs, os->obs.a, sizeof(ent) * (size_t)noo);
        refl = (uint64_t*)xmalloc(sizeof(uint64_t) * (size_t)(nro + 1)); othl = (uint64_t*)xmalloc(sizeof(uint64_t) * (size_t)(noo + 1));
        for (c = 0; c < nro; ++c) refl[nref++] = ((obs*)ref_obs[c].p)->lm;
        for (c = 0; c < noo; ++c) othl[noth++] = ((obs*)oth_obs[c].p)->lm;
        qsort(refl, (size_t)nref, sizeof(uint64_t), cmp_u64); qsort(othl, (size_t)noth, sizeof(uint64_t), cmp_u64);
        both = (uint64_t*)xmalloc(sizeof(uint64_t) * (size_t)(nref + 1));
        { int a = 0, b = 0;
          while (a < nref && b < noth) {
              if (refl[a] < othl[b]) a++;
              else if (othl[b] < refl[a]) b++;
              else { if (nboth == 0 || both[nboth - 1] != refl[a]) both[nboth++] = refl[a]; a++; b++; }
          } }
        /* observations of both frames go into the pose-graph term (and are removed from the graph unless the frame is kept) */
        for (c = 0; c < nro + noo; ++c) {
            const int is_ref = c < nro;
            obs* o = (obs*)(is_ref ? ref_obs[c].p : oth_obs[c - nro].p);
            state* sx = is_ref ? rs : os;
            const int keep = is_ref ? keep_ref : keep_other;
            landmark* l = lm_get(g, o->lm);
            int considered = 0, b;
            ok_vg_kid kid = o->kid;
            for (b = 0; b < nboth; ++b) if (both[b] == o->lm) { considered = 1; break; }
            if (!considered) {
                if (!keep) {
                    ok_vg_remove_observation(g, kid);
                    out->removed_obs = (ok_vg_kid*)realloc(out->removed_obs, sizeof(ok_vg_kid) * (size_t)(out->nremoved_obs + 1));
                    out->removed_obs[out->nremoved_obs++] = kid;
                }
                continue;
            }
            if (keep) ok_reproj_err_set_information(o->err, o->err->info);   /* "half the information": re-set (LLT) */
            ok_twopose_add_observation(t, kid.frame, (int)kid.cam, (int)kid.kp, o->err, 1, sx->pose->id, sx->pose->x, l->hp->id, l->hp->x,
                                       l->hp->initialised, sx->extr[kid.cam]->id, sx->extr[kid.cam]->x, keep);
            { ok_tp_group* grp = &t->groups[0]; int gi;
              for (gi = 0; gi < t->ngroups; ++gi) if (t->groups[gi].lm_id == l->hp->id) { grp = &t->groups[gi]; break; }
              grp->obs[grp->nobs - 1].hp_live_init = &l->hp->initialised; }
            if (!keep) {
                ok_vg_remove_observation(g, kid);
                out->removed_obs = (ok_vg_kid*)realloc(out->removed_obs, sizeof(ok_vg_kid) * (size_t)(out->nremoved_obs + 1));
                out->removed_obs[out->nremoved_obs++] = kid;
            }
        }
        free(ref_obs); free(oth_obs); free(refl); free(othl);
        /* compute and add to the graph */
        if (nboth > 0) {
            ok_vg_blk* b2[2];
            memcpy(t->pose_live[0], rs->pose->x, sizeof(double) * 7);
            ok_twopose_compute(t);
            b2[0] = rs->pose; b2[1] = os->pose;
            p_add_resid(g, link, RK_TP, t, T_TP, 0, 2, b2);
            emap_put(&rs->tp, other_id, 0, link);
            emap_put(&os->tp, ref_id, 0, link);
            ne_add(&num_edges, ref_id, -1); ne_add(&num_edges, other_id, -1);
            out->created = (uint64_t(*)[2])realloc(out->created, sizeof(uint64_t) * 2 * (size_t)(out->ncreated + 1));
            out->created_term = (void**)realloc(out->created_term, sizeof(void*) * (size_t)(out->ncreated + 1));
            out->created[out->ncreated][0] = ref_id; out->created[out->ncreated][1] = other_id; out->created_term[out->ncreated] = t; out->ncreated++;
        } else {
            ne_add(&num_edges, ref_id, -1); ne_add(&num_edges, other_id, -1);
            ok_twopose_free(t); free(t); free(link);
        }
        free(both);
    }
    free(to_create);
    emap_free(&num_edges);
    return 1;
}

int ok_vg_add_external_two_pose_link(ok_vg* g, uint64_t ref, uint64_t other, const ok_tp_std* term) {
    state* rs = st_get(g, ref);
    state* os = st_get(g, other);
    tpclink* c;
    ok_vg_blk* b2[2];
    if (!rs || !os) return 0;
    c = (tpclink*)xcalloc(sizeof(tpclink));
    c->term = *term; c->term.is_computed = 1; c->state0 = ref; c->state1 = other; c->refs = 2;
    b2[0] = rs->pose; b2[1] = os->pose;
    p_add_resid(g, c, RK_TPC, &c->term, T_TPC, 0, 2, b2);
    emap_put(&rs->tpc, other, 0, c);
    emap_put(&os->tpc, ref, 0, c);
    return 1;
}
int ok_vg_remove_two_pose_const_links(ok_vg* g, uint64_t id) {
    state* s = st_get(g, id);
    int i;
    if (!s) return 0;
    for (i = 0; i < s->tpc.n; ++i) {
        tpclink* c = (tpclink*)s->tpc.a[i].p;
        state* o = st_get(g, s->tpc.a[i].k0);
        emap_del(&o->tpc, id, 0);
        p_rm_resid(g, c);
        free(c);
    }
    s->tpc.n = 0;
    return 1;
}
int ok_vg_remove_two_pose_const_link(ok_vg* g, uint64_t i, uint64_t j) {
    state* si = st_get(g, i);
    state* sj = st_get(g, j);
    tpclink* c;
    if (!si || !sj) return 0;
    c = (tpclink*)emap_get(&sj->tpc, i, 0);
    if (!c) return 0;
    emap_del(&si->tpc, j, 0);
    p_rm_resid(g, c);
    emap_del(&sj->tpc, i, 0);
    free(c);
    return 1;
}
int ok_vg_clone_two_pose_const(const ok_vg* g, uint64_t ref, uint64_t other, ok_tp_std* out) {
    state* rs = st_get(g, ref);
    tplink* l;
    if (!rs) return 0;
    l = (tplink*)emap_get(&rs->tp, other, 0);
    if (!l) return 0;
    *out = l->term->term;                       /* cloneTwoPoseGraphErrorConst: DeltaX_, J_, linearisation point */
    out->is_computed = 1;
    return 1;
}

void ok_vg_conv_result_free(ok_vg_conv_result* r) {
    int i;
    for (i = 0; i < r->nobs; ++i) free(r->err[i]);
    free(r->kid); free(r->err); free(r->lm); free(r->lms); free(r->connected); free(r->cauchy);
    memset(r, 0, sizeof *r);
}

int ok_vg_convert_to_observations(ok_vg* g, uint64_t id, ok_vg_conv_result* out) {
    state* cs = st_get(g, id);
    int i;
    memset(out, 0, sizeof *out);
    if (!cs) return 0;
    for (i = 0; i < cs->tp.n; ++i) {
        tplink* link = (tplink*)cs->tp.a[i].p;
        ok_twopose* t = link->term;
        const uint64_t ref = t->ref_id, oth = t->other_id;
        state* rs = st_get(g, ref);
        int g1, o1, nmarg = 0, nout, k;
        struct pend { ok_tp_obs ob; } *pend;
        double (*hpw)[4];
        if (ref != id) {
            out->connected = (uint64_t*)realloc(out->connected, sizeof(uint64_t) * (size_t)(out->nconnected + 1));
            out->connected[out->nconnected++] = ref;
            emap_del(&st_get(g, ref)->tp, id, 0);
        }
        if (oth != id) {
            out->connected = (uint64_t*)realloc(out->connected, sizeof(uint64_t) * (size_t)(out->nconnected + 1));
            out->connected[out->nconnected++] = oth;
            emap_del(&st_get(g, oth)->tp, id, 0);
        }
        for (g1 = 0; g1 < t->ngroups; ++g1) for (o1 = 0; o1 < t->groups[g1].nobs; ++o1) if (t->groups[g1].obs[o1].is_marginalised) nmarg++;
        pend = (struct pend*)xmalloc(sizeof(struct pend) * (size_t)(nmarg + 1));
        hpw = (double(*)[4])xmalloc(sizeof(double) * 4 * (size_t)(nmarg + 1));
        nmarg = 0;
        for (g1 = 0; g1 < t->ngroups; ++g1) for (o1 = 0; o1 < t->groups[g1].nobs; ++o1) if (t->groups[g1].obs[o1].is_marginalised) pend[nmarg++].ob = t->groups[g1].obs[o1];
        nout = ok_twopose_convert(t, rs->pose->x, hpw, nmarg, NULL);
        for (k = 0; k < nout; ++k) {
            const ok_tp_obs* ob = &pend[k].ob;
            ok_vg_kid kid; kid.frame = ob->frame_id; kid.cam = (uint32_t)ob->cam; kid.kp = (uint32_t)ob->kp;
            if (ob_get(g, kid)) continue;                   /* duplication: just keep the existing one */
            if (!lm_get(g, ob->hpoint_id)) {
                ok_vg_add_landmark_id(g, ob->hpoint_id, hpw[k], ob->hp_live_init ? *ob->hp_live_init : ob->hpoint_initialised);
            }
            out->lms = (uint64_t*)realloc(out->lms, sizeof(uint64_t) * (size_t)(out->nlm + 1));
            { int f; for (f = 0; f < out->nlm; ++f) if (out->lms[f] == ob->hpoint_id) break;
              if (f == out->nlm) out->lms[out->nlm++] = ob->hpoint_id; }
            ok_vg_add_external_observation(g, ob->hpoint_id, kid, ob->loss != 0, &ob->err);
            out->kid = (ok_vg_kid*)realloc(out->kid, sizeof(ok_vg_kid) * (size_t)(out->nobs + 1));
            out->err = (ok_reproj_err**)realloc(out->err, sizeof(ok_reproj_err*) * (size_t)(out->nobs + 1));
            out->lm = (uint64_t*)realloc(out->lm, sizeof(uint64_t) * (size_t)(out->nobs + 1));
            out->cauchy = (int*)realloc(out->cauchy, sizeof(int) * (size_t)(out->nobs + 1));
            out->cauchy[out->nobs] = ob->loss != 0;
            out->kid[out->nobs] = kid; out->lm[out->nobs] = ob->hpoint_id;
            out->err[out->nobs] = (ok_reproj_err*)xmalloc(sizeof(ok_reproj_err)); *out->err[out->nobs] = ob->err;
            out->nobs++;
            out->ctr++;
        }
        free(pend); free(hpw);
        p_rm_resid(g, link);
    }
    qsort(out->lms, (size_t)out->nlm, sizeof(uint64_t), cmp_u64);       /* std::set<LandmarkId> */
    /* erase all pose graph errors for this frame */
    for (i = 0; i < cs->tp.n; ++i) {
        tplink* link = (tplink*)cs->tp.a[i].p;
        ok_twopose_free(link->term); free(link->term); free(link);
    }
    cs->tp.n = 0;
    return 1;
}

int ok_vg_remove_speed_and_bias_prior(ok_vg* g, uint64_t id) {
    state* s = st_get(g, id);
    if (s && s->sb_prior) {
        p_rm_resid(g, s->sb_prior);
        free(s->sb_prior); s->sb_prior = NULL;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------
 * direct accesses of ViSlamBackend
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vg_poke_fixation_add(ok_vg* g) {
    state* s1 = st_get(g, 1);
    state* nw = (state*)g->states.a[g->states.n - 1].p;
    double diag[6] = {1.0e8, 1.0e8, 1.0e8, 0.0, 0.0, 0.0};
    ok_tf T;
    ok_vg_blk* b1[1];
    ok_tf_convert(&T, s1->pose->x);
    g->fix_term = (ok_pose_err*)xcalloc(sizeof(ok_pose_err));
    ok_pose_err_init_diag(g->fix_term, &T, diag);
    g->fix_rb = H(g->fix_term);
    b1[0] = nw->pose;
    p_add_resid(g, g->fix_term, RK_FIX, g->fix_term, T_POSE, 0, 1, b1);
    return 1;
}
int ok_vg_poke_fixation_remove(ok_vg* g) {
    if (!g->fix_term) return 0;
    p_rm_resid(g, g->fix_term);
    free(g->fix_term); g->fix_term = NULL;
    return 1;
}
int ok_vg_poke_landmarks_constant(ok_vg* g, int constant) {
    int i;
    for (i = 0; i < g->landmarks.n; ++i) p_set_const(g, ((landmark*)g->landmarks.a[i].p)->hp, constant);
    return 1;
}
int ok_vg_poke_set_observation_information(ok_vg* g, ok_vg_kid kid, const double info[4]) {
    obs* o = ob_get(g, kid);
    if (!o) return 0;
    ok_reproj_err_set_information(o->err, info);
    return 1;
}
int ok_vg_poke_copy_state(ok_vg* g, uint64_t id, const double T7[7], const double sb[9]) {
    return ok_vg_set_pose(g, id, T7) && ok_vg_set_speed_and_bias(g, id, sb);
}
int ok_vg_poke_sync_imu(ok_vg* g, const ok_vg* src, uint64_t id) {
    state* d = st_get(g, id);
    state* s = st_get(src, id);
    if (!d || !s || !d->prev_imu || !s->prev_imu) return 0;
    ok_vg_imu_copy(d->prev_imu->e, s->prev_imu->e);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * observation of the state (replay harness)
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_vg_num_states(const ok_vg* g) { return g->states.n; }
int ok_vg_num_landmarks(const ok_vg* g) { return g->landmarks.n; }
int ok_vg_num_observations(const ok_vg* g) { return g->observations.n; }

int ok_vg_blocks(ok_vg* g, ok_vg_blkref** out) {
    int cap = g->states.n * (2 + g->ncam) + g->landmarks.n + 1, n = 0, i, c;
    ok_vg_blkref* r = (ok_vg_blkref*)xcalloc(sizeof(ok_vg_blkref) * (size_t)cap);
    emap seen; memset(&seen, 0, sizeof seen);
    for (i = 0; i < g->states.n; ++i) {
        state* s = (state*)g->states.a[i].p;
        r[n++].b = s->pose;
        r[n++].b = s->sb;
        for (c = 0; c < g->ncam; ++c) {
            if (emap_has(&seen, H(s->extr[c]), 0)) continue;
            emap_put(&seen, H(s->extr[c]), 0, s->extr[c]);
            r[n++].b = s->extr[c];
        }
    }
    for (i = 0; i < g->landmarks.n; ++i) {
        landmark* l = (landmark*)g->landmarks.a[i].p;
        r[n].b = l->hp; r[n].is_landmark = 1; r[n].lm_id = l->id; r[n].quality = l->quality; r[n].classification = l->classification; n++;
    }
    emap_free(&seen);
    *out = r;
    return n;
}
int ok_vg_state_infos(const ok_vg* g, ok_vg_state_info** out) {
    ok_vg_state_info* r = (ok_vg_state_info*)xcalloc(sizeof(ok_vg_state_info) * (size_t)(g->states.n + 1));
    int i;
    for (i = 0; i < g->states.n; ++i) {
        state* s = (state*)g->states.a[i].p;
        r[i].id = s->id; r[i].is_kf = s->is_kf; r[i].ts = s->ts; r[i].nobs = s->obs.n; r[i].ntp = s->tp.n; r[i].ntpc = s->tpc.n; r[i].nrel = s->rel.n;
    }
    *out = r;
    return g->states.n;
}
int ok_vg_imu_links(ok_vg* g, ok_vg_imuref** out) {
    ok_vg_imuref* r = (ok_vg_imuref*)xcalloc(sizeof(ok_vg_imuref) * (size_t)(g->states.n + 1));
    int i, n = 0;
    for (i = 0; i < g->states.n; ++i) {
        state* s = (state*)g->states.a[i].p;
        if (s->prev_imu) { r[n].state_id = s->id; r[n].e = s->prev_imu->e; n++; }
    }
    *out = r;
    return n;
}
void ok_vg_blk_set(ok_vg_blk* b, const double* x) { memcpy(b->x, x, sizeof(double) * (size_t)b->size); }
void ok_vg_imu_apply_redo(ok_imu_error* e, int redo, int counter, const double sb_ref[9]) {
    if (counter != e->redo_counter) ok_imu_redo_preintegration(e, sb_ref);   /* a flag change alone (redo_ stays set for long terms) re-integrates nothing */
    e->redo = redo; e->redo_counter = counter;
}

int ok_vg_find_resid(const ok_vg* g, uint64_t rb, ok_vg_resid* out) {
    const rdesc* d = (const rdesc*)hmap_get(&g->resid, rb, 0);
    if (!d) return 0;
    out->type = d->type; out->loss = d->loss; out->nb = d->nb; out->term = d->term;
    memcpy(out->blk, d->blk, sizeof out->blk);
    return 1;
}
int ok_vg_is_constant(const ok_vg* g, uint64_t ptr) {
    int s = ok_problem_find_param(&g->pb, ptr);
    return s >= 0 ? g->pb.params[s].constant : -1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * payload serialisations (the layouts of patch 0008 / ok_solve.h)
 * ---------------------------------------------------------------------------------------------------------------- */
uint64_t ok_vg_fnv(const void* p, size_t n) {
    const unsigned char* c = (const unsigned char*)p;
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    for (i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
    return h;
}

typedef struct wbuf { unsigned char* p; size_t n, cap; } wbuf;
static void w_raw(wbuf* w, const void* d, size_t n) {
    if (w->n + n > w->cap) { w->cap = (w->n + n) * 2 + 256; w->p = (unsigned char*)realloc(w->p, w->cap); }
    memcpy(w->p + w->n, d, n); w->n += n;
}
static void w_u32(wbuf* w, uint32_t v) { w_raw(w, &v, 4); }
static void w_u64(wbuf* w, uint64_t v) { w_raw(w, &v, 8); }
static void w_f64n(wbuf* w, const double* v, size_t n) { w_raw(w, v, 8 * n); }

size_t ok_vg_imu_snapshot(const ok_imu_error* e, int with_meas, int zero_uninit, unsigned char** out) {
    wbuf w; size_t i; double z[225]; double dq[4];
    memset(&w, 0, sizeof w); memset(z, 0, sizeof z);
    w_u64(&w, e->n_meas);
    if (with_meas)
        for (i = 0; i < e->n_meas; ++i) {
            w_u32(&w, e->meas[i].t.sec); w_u32(&w, e->meas[i].t.nsec);
            w_f64n(&w, e->meas[i].gyr, 3); w_f64n(&w, e->meas[i].acc, 3);
        }
    { const double p[7] = {e->params.sigma_g_c, e->params.sigma_a_c, e->params.sigma_gw_c, e->params.sigma_aw_c, e->params.g, e->params.g_max, e->params.a_max};
      w_f64n(&w, p, 7); }
    w_u32(&w, e->t0.sec); w_u32(&w, e->t0.nsec); w_u32(&w, e->t1.sec); w_u32(&w, e->t1.nsec);
    dq[0] = e->delta_q.x; dq[1] = e->delta_q.y; dq[2] = e->delta_q.z; dq[3] = e->delta_q.w;
    w_f64n(&w, dq, 4);
    w_f64n(&w, e->C_integral, 9); w_f64n(&w, e->C_doubleintegral, 9); w_f64n(&w, e->acc_integral, 3); w_f64n(&w, e->acc_doubleintegral, 3);
    w_f64n(&w, e->cross, 9); w_f64n(&w, e->dalpha_db_g, 9); w_f64n(&w, e->dv_db_g, 9); w_f64n(&w, e->dp_db_g, 9);
    w_f64n(&w, e->P_delta, 225); w_f64n(&w, e->sb_ref, 9);
    w_u32(&w, e->redo ? 1u : 0u); w_u32(&w, (uint32_t)e->redo_counter);
    w_f64n(&w, (zero_uninit && e->redo_counter == 0) ? z : e->information, 225);
    w_f64n(&w, (zero_uninit && e->redo_counter == 0) ? z : e->sqrt_information, 225);
    { const uint32_t nd = 4u; /* the ctor resizes dPdsigma_ to 4 zero matrices */ uint32_t k; w_u32(&w, nd); for (k = 0; k < nd; ++k) w_f64n(&w, e->dPdsigma[k], 225); }
    *out = w.p;
    return w.n;
}

size_t ok_vg_reproj_payload(const ok_reproj_err* e, unsigned char** out) {
    wbuf w; const ok_cam* c = &e->cam; int r, cc;
    memset(&w, 0, sizeof w);
    w_u32(&w, (uint32_t)c->dist); w_u32(&w, (uint32_t)c->w); w_u32(&w, (uint32_t)c->h);
    { const double f[4] = {c->fu, c->fv, c->cu, c->cv}; w_f64n(&w, f, 4); }
    w_u32(&w, (uint32_t)c->nd); w_f64n(&w, c->d, (size_t)c->nd);
    w_f64n(&w, e->meas, 2);
    for (r = 0; r < 2; ++r) for (cc = 0; cc < 2; ++cc) w_f64n(&w, &e->info[r + 2 * cc], 1);
    *out = w.p;
    return w.n;
}
static void w_mat_rm(wbuf* w, const double* cm, int n) { int r, c; for (r = 0; r < n; ++r) for (c = 0; c < n; ++c) w_f64n(w, &cm[r + n * c], 1); }
static void tf_coeffs(const ok_tf* t, double c[7]) { memcpy(c, t->r, sizeof(double) * 3); c[3] = t->q.x; c[4] = t->q.y; c[5] = t->q.z; c[6] = t->q.w; }
size_t ok_vg_pose_payload(const ok_pose_err* e, unsigned char** out) {
    wbuf w; double c[7];
    memset(&w, 0, sizeof w);
    tf_coeffs(&e->meas, c); w_f64n(&w, c, 7); w_u32(&w, 6);
    w_mat_rm(&w, e->info, 6); w_mat_rm(&w, e->sqrt_info, 6);
    *out = w.p; return w.n;
}
size_t ok_vg_sab_payload(const ok_sab_err* e, unsigned char** out) {
    wbuf w;
    memset(&w, 0, sizeof w);
    w_f64n(&w, e->meas, 9); w_u32(&w, 9);
    w_mat_rm(&w, e->info, 9); w_mat_rm(&w, e->sqrt_info, 9);
    *out = w.p; return w.n;
}
size_t ok_vg_relpose_payload(const ok_relpose_err* e, unsigned char** out) {
    wbuf w; double c[7];
    memset(&w, 0, sizeof w);
    tf_coeffs(&e->T_AB, c); w_f64n(&w, c, 7); w_u32(&w, 6);
    w_mat_rm(&w, e->info, 6); w_mat_rm(&w, e->sqrt_info, 6);
    *out = w.p; return w.n;
}
size_t ok_vg_tp_payload(const ok_tp_std* e, unsigned char** out) {
    wbuf w; double c[7];
    memset(&w, 0, sizeof w);
    w_u32(&w, e->is_computed ? 1u : 0u); w_f64n(&w, e->DeltaX, 6); w_f64n(&w, e->J, 36);
    tf_coeffs(&e->lin_T_S0S1, c); w_f64n(&w, c, 7);
    *out = w.p; return w.n;
}

/* ------------------------------------------------------------------------------------------------------------------
 * read access for ViSlamBackend (module M6)
 * ---------------------------------------------------------------------------------------------------------------- */
static void state_view(const state* s, ok_vg_state_view* v) {
    v->id = s->id; v->is_kf = s->is_kf; v->pose_fixed = s->pose->fixed; v->sb_fixed = s->sb->fixed;
    v->nobs = s->obs.n; v->ntp = s->tp.n; v->ntpc = s->tpc.n; v->nrel = s->rel.n; v->has_prev_imu = s->prev_imu != NULL; v->ts = s->ts;
}
int ok_vg_state_count(const ok_vg* g) { return g->states.n; }
int ok_vg_state_at(const ok_vg* g, int idx, ok_vg_state_view* v) { if (idx < 0 || idx >= g->states.n) return 0; state_view((const state*)g->states.a[idx].p, v); return 1; }
int ok_vg_state_find(const ok_vg* g, uint64_t id, ok_vg_state_view* v) { const state* s = st_get(g, id); if (!s) return 0; if (v) state_view(s, v); return 1; }
int ok_vg_state_index(const ok_vg* g, uint64_t id) { int f, i = emap_search(&g->states, id, 0, &f); return f ? i : -1; }
int ok_vg_state_obs(const ok_vg* g, uint64_t id, ok_vg_kid** kids, uint64_t** lms) {
    const state* s = st_get(g, id);
    int i, n;
    *kids = NULL; *lms = NULL;
    if (!s) return 0;
    n = s->obs.n;
    *kids = (ok_vg_kid*)xmalloc(sizeof(ok_vg_kid) * (size_t)n);
    *lms = (uint64_t*)xmalloc(sizeof(uint64_t) * (size_t)n);
    for (i = 0; i < n; ++i) { (*kids)[i] = kid_of(s->obs.a[i].k0, s->obs.a[i].k1); (*lms)[i] = ((const obs*)s->obs.a[i].p)->lm; }
    return n;
}
int ok_vg_landmark_count(const ok_vg* g) { return g->landmarks.n; }
uint64_t ok_vg_landmark_id_at(const ok_vg* g, int i) { return g->landmarks.a[i].k0; }
int ok_vg_landmark_find(const ok_vg* g, uint64_t id, ok_vg_lm_view* v) {
    const landmark* l = lm_get(g, id);
    if (!l) return 0;
    if (v) { v->id = id; memcpy(v->hp, l->hp->x, sizeof v->hp); v->initialised = l->hp->initialised; v->quality = l->quality; v->nobs = l->obs.n; }
    return 1;
}
int ok_vg_landmark_obs(const ok_vg* g, uint64_t id, ok_vg_kid** kids) {
    const landmark* l = lm_get(g, id);
    int i, n;
    *kids = NULL;
    if (!l) return 0;
    n = l->obs.n;
    *kids = (ok_vg_kid*)xmalloc(sizeof(ok_vg_kid) * (size_t)(n ? n : 1));
    for (i = 0; i < n; ++i) (*kids)[i] = kid_of(l->obs.a[i].k0, l->obs.a[i].k1);
    return n;
}
int ok_vg_obs_find(const ok_vg* g, ok_vg_kid kid, uint64_t* lm, const ok_reproj_err** err, int* cauchy) {
    const obs* o = ob_get(g, kid);
    if (!o) return 0;
    if (lm) *lm = o->lm;
    if (err) *err = o->err;
    if (cauchy) *cauchy = o->loss != 0;
    return 1;
}
int ok_vg_anystate_get(const ok_vg* g, uint64_t id, uint64_t* kf, double T7[7], double v3[3]) {
    const anystate* a = (const anystate*)emap_get(&g->anystates, id, 0);
    if (!a) return 0;
    if (kf) *kf = a->kf;
    if (T7) { memcpy(T7, a->T_Sk_S.r, sizeof(double) * 3); T7[3] = a->T_Sk_S.q.x; T7[4] = a->T_Sk_S.q.y; T7[5] = a->T_Sk_S.q.z; T7[6] = a->T_Sk_S.q.w; }
    if (v3) memcpy(v3, a->v_Sk, sizeof a->v_Sk);
    return 1;
}
int ok_vg_anystate_count(const ok_vg* g) { return g->anystates.n; }
int ok_vg_anystate_at(const ok_vg* g, int i, uint64_t* id, uint64_t* kf, ok_time* ts, double T7[7], double v3[3]) {
    const anystate* a;
    if (i < 0 || i >= g->anystates.n) return 0;
    a = (const anystate*)g->anystates.a[i].p;
    if (id) *id = g->anystates.a[i].k0;
    if (ts) *ts = a->ts;
    return ok_vg_anystate_get(g, g->anystates.a[i].k0, kf, T7, v3);
}
int ok_vg_imu_use(const ok_vg* g) { return g->imu.use; }
int ok_vg_num_cameras(const ok_vg* g) { return g->ncam; }
void ok_vg_set_solver_options(ok_vg* g, int linear_solver_type, double function_tolerance) { g->solver_type = linear_solver_type; g->ftol = function_tolerance; }
int ok_vg_solver_type(const ok_vg* g) { return g->solver_type; }
double ok_vg_function_tolerance(const ok_vg* g) { return g->ftol; }
int ok_vg_pose_fixed(const ok_vg* g, uint64_t id) { const state* s = st_get(g, id); return s ? s->pose->fixed : -1; }
int ok_vg_state_links(const ok_vg* g, uint64_t id, int kind, uint64_t (**pairs)[2]) {
    const state* s = st_get(g, id);
    const emap* m;
    int i, n;
    *pairs = NULL;
    if (!s) return 0;
    m = kind == 0 ? &s->rel : (kind == 1 ? &s->tp : &s->tpc);
    n = m->n;
    *pairs = (uint64_t(*)[2])xmalloc(sizeof(uint64_t) * 2 * (size_t)(n ? n : 1));
    for (i = 0; i < n; ++i) {
        if (kind == 0) { const rlink* l = (const rlink*)m->a[i].p; (*pairs)[i][0] = l->state0; (*pairs)[i][1] = l->state1; }
        else if (kind == 1) { const tplink* l = (const tplink*)m->a[i].p; (*pairs)[i][0] = l->state0; (*pairs)[i][1] = l->state1; }
        else { const tpclink* l = (const tpclink*)m->a[i].p; (*pairs)[i][0] = l->state0; (*pairs)[i][1] = l->state1; }
    }
    return n;
}
int ok_vg_rel_link_get(const ok_vg* g, uint64_t s0, uint64_t s1, double T7[7], double info[36]) {
    const state* a = st_get(g, s0);
    const rlink* l;
    if (!a) return 0;
    l = (const rlink*)emap_get(&a->rel, s1, 0);
    if (!l) return 0;
    memcpy(T7, l->term.T_AB.r, sizeof(double) * 3);
    T7[3] = l->term.T_AB.q.x; T7[4] = l->term.T_AB.q.y; T7[5] = l->term.T_AB.q.z; T7[6] = l->term.T_AB.q.w;
    memcpy(info, l->term.info, sizeof(double) * 36);
    return 1;
}
