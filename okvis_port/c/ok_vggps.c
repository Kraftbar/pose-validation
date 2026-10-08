/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2-X GNSS fusion, ViGraph side (see ok_vggps.h for scope and notices). Line-by-line port of the GNSS parts of
 * okvis_ceres/src/ViGraph.cpp (upstream line numbers in the comments). */
#include "ok_vggps.h"
#include "ok_align4.h"
#include "ok_gps.h"
#include "ok_gps_init.h"
#include "ok_vslam.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- observe-only event log ---- */
static FILE* g_log; static int g_log_init;
int ok_gnss_log_enabled(void) {
    if (!g_log_init) { const char* p = getenv("OKVIS_PORT_GNSS_LOG"); g_log_init = 1; if (p && *p) g_log = fopen(p, "w"); }
    return g_log != NULL;
}
void ok_gnss_logf(const char* fmt, ...) {
    va_list ap;
    if (!ok_gnss_log_enabled()) return;
    va_start(ap, fmt); vfprintf(g_log, fmt, ap); va_end(ap);
}

typedef struct u64set { uint64_t* a; int n, cap; } u64set;      /* std::set<StateId> */
static int set_lb(const u64set* s, uint64_t v) { int lo = 0, hi = s->n; while (lo < hi) { const int m = (lo + hi) / 2; if (s->a[m] < v) lo = m + 1; else hi = m; } return lo; }
static int set_add(u64set* s, uint64_t v) {
    const int i = set_lb(s, v);
    if (i < s->n && s->a[i] == v) return 0;
    if (s->n == s->cap) { s->cap = s->cap ? 2 * s->cap : 64; s->a = (uint64_t*)realloc(s->a, sizeof(uint64_t) * (size_t)s->cap); }
    memmove(s->a + i + 1, s->a + i, sizeof(uint64_t) * (size_t)(s->n - i));
    s->a[i] = v; s->n++;
    return 1;
}
static void set_del(u64set* s, uint64_t v) {
    const int i = set_lb(s, v);
    if (i < s->n && s->a[i] == v) { memmove(s->a + i, s->a + i + 1, sizeof(uint64_t) * (size_t)(s->n - i - 1)); s->n--; }
}

typedef struct entry { uint64_t sid; ok_gps_fix f; } entry;     /* gpsInitMap_ element */

typedef struct hpt { uint64_t key; double gps[3], world[3], cov[9]; } hpt;     /* OUR modification (FIX_HISTORY) */
typedef struct ok_vgps {
    int status;                           /* gpsStatus_ */
    int observable, fixed;                /* gpsObservability_, gpsFixed_ */
    ok_tf T_GW_init_;
    u64set gps_states, reinit_states;
    entry* imap; int nimap, capimap;      /* std::multimap<StateId, GpsMeasurement> gpsInitMap_ (key order, equal keys in insertion order) */
    ok_imu_meas* imuq; size_t nimuq, capimuq;
    int needs_initial, needs_pos, needs_full;
    uint64_t dropout_id, pos_aligned_id;
    int reinitialised;
    struct hpt* hist; int nhist, caphist;  /* OUR modification FIX_HISTORY: points of states already eliminated from the window (not upstream) */
    int name;                             /* log tag: 0 realtime, 1 full */
} ok_vgps;

static ok_vgps* P_(const ok_vg* g) { return (ok_vgps*)ok_vg_gps_policy(g); }
static void policy_free(void* p) {
    ok_vgps* P = (ok_vgps*)p;
    if (!P) return;
    free(P->hist); free(P->gps_states.a); free(P->reinit_states.a); free(P->imap); free(P->imuq); free(P);
}
static void policy_removed(void* p, uint64_t id) {
    ok_vgps* P = (ok_vgps*)p;
    int i, j = 0;
    set_del(&P->gps_states, id);                       /* ViGraphEstimator::eliminateStateByImuMerge: gpsStates_.erase(stateId) */
    set_del(&P->reinit_states, id);                    /* gpsReInitStates_.erase */
    for (i = 0; i < P->nimap; ++i) if (P->imap[i].sid != id) P->imap[j++] = P->imap[i];   /* gpsInitMap_.erase(stateId) */
    P->nimap = j;
}
static void imap_insert(ok_vgps* P, uint64_t sid, const ok_gps_fix* f) {      /* multimap::insert: after the equal range */
    int i = 0;
    while (i < P->nimap && P->imap[i].sid <= sid) ++i;
    if (P->nimap == P->capimap) { P->capimap = P->capimap ? 2 * P->capimap : 64; P->imap = (entry*)realloc(P->imap, sizeof(entry) * (size_t)P->capimap); }
    memmove(P->imap + i + 1, P->imap + i, sizeof(entry) * (size_t)(P->nimap - i));
    P->imap[i].sid = sid; P->imap[i].f = *f; P->nimap++;
}

int ok_vgps_add_gps(ok_vg* g, const double r_SA[3], double yaw_error_threshold, int robust) {
    ok_vgps* P;
    if (ok_vg_gps_enable(g, r_SA, yaw_error_threshold, robust)) return -1;
    P = (ok_vgps*)calloc(1, sizeof *P);
    ok_tf_identity(&P->T_GW_init_);
    ok_vg_gps_set_policy(g, P, policy_free, policy_removed);
    return 0;
}
int ok_vgps_status(const ok_vg* g) { return P_(g)->status; }
void ok_vgps_set_name(ok_vg* g, int name) { P_(g)->name = name; }
void ok_vgps_set_status(ok_vg* g, int status) {
    ok_vgps* P = P_(g);
    if (P->status != status) ok_gnss_logf("ST %d %d->%d set\n", P->name, P->status, status);
    P->status = status;
}
int ok_vgps_is_fixed(const ok_vg* g) { return P_(g)->fixed; }
int ok_vgps_is_observable(const ok_vg* g) { return P_(g)->observable; }
int ok_vgps_needs_initial_alignment(const ok_vg* g) { return P_(g)->needs_initial; }   /* :952 */

/* ---- ViGraph.cpp:835 freeze / unfreeze / set ---- */
void ok_vgps_freeze(ok_vg* g) { ok_vg_gps_set_const(g, 1); P_(g)->fixed = 1; ok_gnss_logf("TG %d freeze\n", P_(g)->name); }
void ok_vgps_unfreeze(ok_vg* g) { ok_vg_gps_set_const(g, 0); P_(g)->fixed = 0; ok_gnss_logf("TG %d unfreeze\n", P_(g)->name); }

/* ---- the state iteration helpers (std::map<StateId, State> states_) ---- */
static int st_view(const ok_vg* g, int i, ok_vg_state_view* v) { return ok_vg_state_at(g, i, v); }

/* ---- :973 addGpsMeasurement ---- */
int ok_vgps_add_measurement(ok_vg* g, uint64_t sid, const ok_gps_fix* m, const ok_imu_meas* imu, size_t nimu) {
    ok_vgps* P = P_(g);
    ok_vg_state_view sv;
    if (!ok_vg_state_find(g, sid, &sv)) { ok_gnss_logf("AM %d sid=%llu MISSING\n", P->name, (unsigned long long)sid); return 0; }   /* the C++ throws */
    if (!ok_time_le(imu[0].t, sv.ts)) {                                                                                          /* front <= state */
        ok_gnss_logf("AM %d sid=%llu t=%u.%09u IMUOLD\n", P->name, (unsigned long long)sid, m->t.sec, m->t.nsec);
        return 0;
    }
    set_add(&P->gps_states, sid);
    ok_vg_gps_push_factor(g, sid, m, imu, nimu,
                          P->status == OK_GPS_INITIALISING || P->status == OK_GPS_INITIALISED || P->status == OK_GPS_REINITIALISING);
    ok_gnss_logf("AM %d sid=%llu t=%u.%09u status=%d res=%d pos=%.17g,%.17g,%.17g\n", P->name, (unsigned long long)sid, m->t.sec, m->t.nsec,
                 P->status, P->status >= OK_GPS_INITIALISING, m->pos[0], m->pos[1], m->pos[2]);
    return 1;
}

/* ceres Solver Report of Align4DoF_Ceres (the C++ prints summary.BriefReport() to stdout): into the GNSS log */
static void align4_on_end(void* ctx, const ok_sv_end* e) {
    (void)ctx;
    ok_gnss_logf("CS Iterations: %d, Initial cost: %e, Final cost: %e, Termination: %d\n", e->num_iterations, e->initial_cost, e->final_cost, e->termination_type);
}

/* ---- :1014 checkForGpsInit (robust: last 100 states + RANSAC + Align4DoF_Ceres) ---- */
static uint64_t* hist_keys_; static int nkeys_cap_;       /* OUR modification: fix time of each gathered point (FIX_HISTORY key) */

/* OUR observe-only diagnostic (OKVIS_PORT_FIX_DIAG=1 + OKVIS_PORT_GNSS_LOG): what the yaw gate would say for the points handed to the check, WITHOUT the
 * RANSAC size requirement: spread (rms distance from the centroid, xy) of the VIO-world points and of the GNSS points, the travelled path length of the
 * world points, and the yaw sigma (degrees) of the Hessian at the Umeyama fit of all points. Pure functions, no state touched. */
static void diag_points(const ok_vgps* P, int nstates, int npts, const double* gps, const double* world, const double* cov) {
    double cw[3] = {0, 0, 0}, cg[3] = {0, 0, 0}, sw = 0, sg = 0, path = 0, Hess[16], Pm[16], ysig = -1.0, bounds[4] = {1e300, -1e300, 1e300, -1e300};
    ok_tf T;
    int i, k;
    if (!ok_gnss_log_enabled() || npts < 3) return;
    for (i = 0; i < npts; ++i) for (k = 0; k < 3; ++k) { cw[k] += world[3 * i + k] / npts; cg[k] += gps[3 * i + k] / npts; }
    for (i = 0; i < npts; ++i) {
        for (k = 0; k < 2; ++k) { const double a = world[3 * i + k] - cw[k], b = gps[3 * i + k] - cg[k]; sw += a * a; sg += b * b; }
        if (i) { const double dx = world[3 * i] - world[3 * i - 3], dy = world[3 * i + 1] - world[3 * i - 2], dz = world[3 * i + 2] - world[3 * i - 1]; path += sqrt(dx * dx + dy * dy + dz * dz); }
    }
    (void)bounds;
    ok_gps_umeyama(npts, gps, world, &T);
    ok_gps_yaw_hessian(npts, world, cov, T.C, Hess);
    ok_gps_inverse4(Hess, Pm);
    ysig = sqrt(Pm[3 + 4 * 3]) / 3.14159265358979323846 * 180.0;
    ok_gnss_logf("CD %d n=%d npts=%d spreadW=%.4f spreadG=%.4f pathW=%.4f yawAll=%.6f cov00=%.4f\n", P->name, nstates, npts, sqrt(sw / npts), sqrt(sg / npts), path, ysig, cov[0]);
}

static int check_for_gps_init(ok_vg* g, ok_tf* T_GW, const u64set* considered, double* yaw_error) {
    ok_vgps* P = P_(g);
    const int robust = ok_vg_gps_robust(g);
    const double* r_SA = ok_vg_gps_r_SA(g);
    int start = 0, i, k, npts = 0, cap = 0;
    double *gps = NULL, *world = NULL, *cov = NULL, yaw = 0.0, ratio = 0.0;
    int r;
    if (considered->n < 2) { ok_gnss_logf("CI %d n=%d EARLY\n", P->name, considered->n); return 0; }
    if (considered->n > 100 && robust) start = considered->n - 100;
    for (i = start; i < considered->n; ++i) {
        const uint64_t sid = considered->a[i];
        double pose[7], sb[9];
        ok_tf T_WS_state;
        int nf = ok_vg_gps_nfactors(g, sid);
        ok_vg_pose_values(g, sid, pose); ok_vg_sb_values(g, sid, sb);
        ok_tf_convert(&T_WS_state, pose);
        for (k = 0; k < nf; ++k) {
            ok_gps_async* e = ok_vg_gps_factor(g, sid, k);
            ok_tf T_prop;
            double lv[3];
            if (npts == cap) {
                cap = cap ? 2 * cap : 128;
                gps = (double*)realloc(gps, sizeof(double) * 3 * (size_t)cap); world = (double*)realloc(world, sizeof(double) * 3 * (size_t)cap);
                cov = (double*)realloc(cov, sizeof(double) * 9 * (size_t)cap);
            }
            if (npts >= nkeys_cap_) { nkeys_cap_ = nkeys_cap_ ? 2 * nkeys_cap_ : 128; hist_keys_ = (uint64_t*)realloc(hist_keys_, sizeof(uint64_t) * (size_t)nkeys_cap_); }
            hist_keys_[npts] = (uint64_t)e->imu.t1.sec * 1000000000ull + (uint64_t)e->imu.t1.nsec;
            memcpy(gps + 3 * npts, e->meas, sizeof(double) * 3);
            ok_gps_async_apply_preint(e, &T_WS_state, sb, &T_prop);
            ok_m3_mulv(T_prop.C, r_SA, lv);                                /* T_WS_prop.r() + T_WS_prop.C() * r_SA */
            world[3 * npts] = T_prop.r[0] + lv[0]; world[3 * npts + 1] = T_prop.r[1] + lv[1]; world[3 * npts + 2] = T_prop.r[2] + lv[2];
            memcpy(cov + 9 * npts, e->covariance, sizeof(double) * 9);
            npts++;
        }
    }
    /* OUR modification FIX_HISTORY (not upstream): upstream only sees the fixes of the states still in the sliding window (at 5 Hz ~28 fixes,
     * too few for RANSAC and for a 1 degree yaw sigma). With the switch (robust, initial init only) every point seen in a check is stored
     * (keyed by the fix time, newest estimate wins) and the check uses the last OKVIS_PORT_FIX_HISTORY_N (default 200) stored points: the
     * window points plus the world positions the eliminated states had at their last check. */
    if (robust && considered == &P->gps_states && ok_port_fix("HISTORY")) {
        const char* en = getenv("OKVIS_PORT_FIX_HISTORY_N");
        const int nmax = en && atoi(en) > 0 ? atoi(en) : 200;
        int j, nn, from;
        for (i = 0, j = 0; i < npts; ++i) {
            hpt h; int lo = 0, hi = P->nhist;
            uint64_t key = hist_keys_[i];
            h.key = key; memcpy(h.gps, gps + 3 * i, sizeof h.gps); memcpy(h.world, world + 3 * i, sizeof h.world); memcpy(h.cov, cov + 9 * i, sizeof h.cov);
            while (lo < hi) { const int m = (lo + hi) / 2; if (P->hist[m].key < key) lo = m + 1; else hi = m; }
            if (lo < P->nhist && P->hist[lo].key == key) { P->hist[lo] = h; continue; }
            if (P->nhist == P->caphist) { P->caphist = P->caphist ? 2 * P->caphist : 256; P->hist = (hpt*)realloc(P->hist, sizeof(hpt) * (size_t)P->caphist); }
            memmove(P->hist + lo + 1, P->hist + lo, sizeof(hpt) * (size_t)(P->nhist - lo));
            P->hist[lo] = h; P->nhist++; (void)j;
        }
        nn = P->nhist < nmax ? P->nhist : nmax; from = P->nhist - nn;
        if (nn > cap) { cap = nn; gps = (double*)realloc(gps, sizeof(double) * 3 * (size_t)cap); world = (double*)realloc(world, sizeof(double) * 3 * (size_t)cap); cov = (double*)realloc(cov, sizeof(double) * 9 * (size_t)cap); }
        for (i = 0; i < nn; ++i) { memcpy(gps + 3 * i, P->hist[from + i].gps, 24); memcpy(world + 3 * i, P->hist[from + i].world, 24); memcpy(cov + 9 * i, P->hist[from + i].cov, 72); }
        npts = nn;
    }
    /* OUR modification FIX_RANSAC_SMALL, see ok_gps_init.c: allowed only for the first (Idle -> Initialising, yaw_error != NULL) check of the initial
     * initialisation; Initialising -> Initialised (1 degree gate, Align4DoF_Ceres, freeze) and the re-initialisation keep the upstream 40-point rule. */
    ok_port_set_small_allowed(robust && yaw_error != NULL && considered == &P->gps_states);
    if (ok_port_fix("DIAG")) diag_points(P, considered->n, npts, gps, world, cov);
    r = ok_gps_init_core(npts, gps, world, cov, robust, T_GW, &yaw, &ratio);
    if (r) { ok_gnss_logf("CI %d n=%d npts=%d RANSACREJECT\n", P->name, considered->n, npts); free(gps); free(world); free(cov); return 0; }
    if (yaw_error) *yaw_error = yaw;
    ok_gnss_logf("CI %d n=%d npts=%d yaw=%.17g T=%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n", P->name, considered->n, npts, yaw,
                 T_GW->r[0], T_GW->r[1], T_GW->r[2], T_GW->q.x, T_GW->q.y, T_GW->q.z, T_GW->q.w);
    if (yaw < ok_vg_gps_yaw_error_threshold(g)) {
        if (robust) {             /* Align4DoF_Ceres(gpsPoints, worldPoints, T_GW, T_GW_refined); T_GW = T_GW_refined */
            ok_tf T_ref;
            ok_sv_hooks hk;
            memset(&hk, 0, sizeof hk);
            hk.on_end = align4_on_end;
            ok_align4dof_ceres(npts, gps, world, T_GW, &T_ref, &hk);
            ok_gnss_logf("CR %d npts=%d T=%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n", P->name, npts, T_ref.r[0], T_ref.r[1], T_ref.r[2],
                         T_ref.q.x, T_ref.q.y, T_ref.q.z, T_ref.q.w);
            *T_GW = T_ref;
        }
        free(gps); free(world); free(cov);
        return 1;
    }
    free(gps); free(world); free(cov);
    return 0;
}

static void tf_coeffs(const ok_tf* t, double c[7]) { memcpy(c, t->r, sizeof(double) * 3); c[3] = t->q.x; c[4] = t->q.y; c[5] = t->q.z; c[6] = t->q.w; }

/* ---- :1216 addGpsMeasurements ---- */
int ok_vgps_add_measurements(ok_vg* g, const ok_gps_fix* m, int n, const ok_imu_meas* imu, size_t nimu, uint64_t* sids, int* nsids) {
    ok_vgps* P = P_(g);
    int i, ri, k;
    if (nsids) *nsids = 0;
    if (n == 0 || nimu == 0) return 0;
    if (!ok_time_le(imu[0].t, m[0].t)) {
        ok_gnss_logf("AMS %d IMUTOONEW\n", P->name);
        return 0;
    }
    if (P->status != OK_GPS_INITIALISED) {                       /* save the IMU measurements for the observability consideration */
        for (i = 0; i < (int)nimu; ++i) {
            if (P->nimuq && ok_time_lt(imu[i].t, P->imuq[P->nimuq - 1].t)) continue;
            if (P->nimuq == P->capimuq) { P->capimuq = P->capimuq ? 2 * P->capimuq : 256; P->imuq = (ok_imu_meas*)realloc(P->imuq, sizeof(ok_imu_meas) * P->capimuq); }
            P->imuq[P->nimuq++] = imu[i];
        }
    }
    ri = ok_vg_state_count(g) - 1;                                /* states_.rbegin() */
    for (k = n - 1; k >= 0; --k) {
        ok_vg_state_view sv;
        uint64_t sid;
        while (ri >= 0) {
            st_view(g, ri, &sv);
            if (ok_time_lt(m[k].t, sv.ts)) --ri; else break;      /* state.timestamp > fix.timestamp */
        }
        if (ri < 0) break;                                        /* rIterStates == states_.rend() */
        st_view(g, ri, &sv);
        sid = sv.id;
        if (sids) { memmove(sids + 1, sids, sizeof(uint64_t) * (size_t)*nsids); sids[0] = sid; (*nsids)++; }   /* sids->push_front */
        ok_vg_gps_set_mode(g, sid, P->status);
        switch (P->status) {
            case OK_GPS_OFF:
                imap_insert(P, sid, &m[k]);
                ok_gnss_logf("AMS %d first measurement sid=%llu\n", P->name, (unsigned long long)sid);
                ok_vgps_set_status(g, OK_GPS_IDLE);
                break;
            case OK_GPS_IDLE:
            case OK_GPS_INITIALISING:
                imap_insert(P, sid, &m[k]);
                ok_vgps_add_measurement(g, sid, &m[k], imu, nimu);
                break;
            case OK_GPS_INITIALISED:
                ok_vgps_add_measurement(g, sid, &m[k], imu, nimu);
                break;
            case OK_GPS_REINITIALISING:
                imap_insert(P, sid, &m[k]);
                set_add(&P->reinit_states, sid);
                ok_vgps_add_measurement(g, sid, &m[k], imu, nimu);
                break;
            default: return 0;
        }
    }
    return 1;
}


/* ---- ViGraph.cpp:1128 checkValidGpsMeasurements (called by ViSlamBackend::addGpsMeasurementsOnAllGraphs for robust_gps_init only) ----
 * `in` is the fixes of the batch in deque order; `out` (capacity n) receives the accepted ones in the order the C++ pushes them (push_back
 * while walking the input in REVERSE, so out is the reversed subsequence); returns their number. Quirks kept: the state pointer only moves
 * backwards (fixes older than every state end the walk), the Initialised branch tests each fix against states_.at(sid) (pose and the shared
 * T_GW block): |error| > 3 sigma per axis rejects it; a fixed last GPS state (dropout) or ReInitialising accepts everything. Cartesian only
 * (the geodetic Forward() is not ported). */
int ok_vgps_check_valid_measurements(ok_vg* g, const ok_gps_fix* in, int n, ok_gps_fix* out) {
    ok_vgps* P = P_(g);
    const double* r_SA = ok_vg_gps_r_SA(g);
    int ri = ok_vg_state_count(g) - 1, k, nvalid = 0, needs_reinit = 0;
    if (n == 0) return 0;
    if (P->status == OK_GPS_INITIALISED && P->gps_states.n > 0) needs_reinit = ok_vg_pose_fixed(g, P->gps_states.a[P->gps_states.n - 1]);
    for (k = n - 1; k >= 0; --k) {
        ok_vg_state_view sv;
        uint64_t sid;
        while (ri >= 0) {
            st_view(g, ri, &sv);
            if (ok_time_lt(in[k].t, sv.ts)) --ri; else break;      /* state.timestamp > fix.timestamp */
        }
        if (ri < 0) break;
        st_view(g, ri, &sv);
        sid = sv.id;
        if (P->status == OK_GPS_OFF || P->status == OK_GPS_IDLE || P->status == OK_GPS_INITIALISING) {
            if (in[k].cov[0] > 36.0 || in[k].cov[8] > 100.0) { ok_gnss_logf("CV %d reject-inaccurate sid=%llu\n", P->name, (unsigned long long)sid); continue; }   /* pow(6.0, 2), pow(10.0, 2) */
            out[nvalid++] = in[k];
        }
        if (P->status == OK_GPS_INITIALISED && !needs_reinit) {
            double pose[7], gw[7], lv[3], pm[3], err[3], sx, sy, sz;
            ok_tf T_WS, T_GW, T_inv;
            ok_vg_pose_values(g, sid, pose); ok_tf_convert(&T_WS, pose);
            ok_vg_gps_get_T_GW(g, gw); ok_tf_convert(&T_GW, gw);
            ok_tf_inverse(&T_GW, &T_inv, 1);
            ok_m3_mulv(T_inv.C, in[k].pos, pm);                                    /* T3x4 * homogeneous: block * v, then += translation */
            pm[0] += T_inv.r[0]; pm[1] += T_inv.r[1]; pm[2] += T_inv.r[2];
            ok_m3_mulv(T_WS.C, r_SA, lv);
            err[0] = (T_WS.r[0] + lv[0]) - pm[0]; err[1] = (T_WS.r[1] + lv[1]) - pm[1]; err[2] = (T_WS.r[2] + lv[2]) - pm[2];
            sx = sqrt(in[k].cov[0]); sy = sqrt(in[k].cov[4]); sz = sqrt(in[k].cov[8]);
            if (fabs(err[0]) > 3.0 * sx || fabs(err[1]) > 3.0 * sy || fabs(err[2]) > 3.0 * sz) {
                ok_gnss_logf("CV %d reject-3sigma sid=%llu err=%.9g,%.9g,%.9g\n", P->name, (unsigned long long)sid, err[0], err[1], err[2]);
                continue;
            }
            out[nvalid++] = in[k];
        } else if (P->status == OK_GPS_REINITIALISING || needs_reinit) {
            out[nvalid++] = in[k];
        }
    }
    return nvalid;
}

/* ---- :1318 initializationStrategy ---- */
int ok_vgps_initialization_strategy(ok_vg* g, double T_GW_est[7]) {
    ok_vgps* P = P_(g);
    ok_tf T_GW_init;
    int init_successful = 0;
    double yaw_error = 100.0;
    ok_tf_identity(&T_GW_init);
    switch (P->status) {
        case OK_GPS_OFF: break;
        case OK_GPS_IDLE:
            check_for_gps_init(g, &T_GW_init, &P->gps_states, &yaw_error);
            if (yaw_error < 5.0) {
                tf_coeffs(&T_GW_init, T_GW_est);
                ok_vgps_set_status(g, OK_GPS_INITIALISING);
                init_successful = 1;
            }
            break;
        case OK_GPS_INITIALISING:
            P->observable = check_for_gps_init(g, &T_GW_init, &P->gps_states, NULL);
            if (P->observable) {
                double c[7];
                tf_coeffs(&T_GW_init, c);
                ok_vg_gps_set_T_GW(g, c);                          /* setGpsExtrinsics */
                ok_vgps_set_status(g, OK_GPS_INITIALISED);
                P->T_GW_init_ = T_GW_init;
                tf_coeffs(&T_GW_init, T_GW_est);
                P->needs_initial = 1;
                ok_gnss_logf("TG %d set(init) %.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n", P->name, c[0], c[1], c[2], c[3], c[4], c[5], c[6]);
            }
            break;
        case OK_GPS_INITIALISED: break;
        case OK_GPS_REINITIALISING:
            P->reinitialised = check_for_gps_init(g, &T_GW_init, &P->reinit_states, NULL);
            if (P->reinitialised) {
                double distance = 0.0;
                ok_vg_state_view sv;
                double pose[7], Tgw[7], rel_err, budget;
                ok_tf T_WS_i, T_WS_j;
                ok_quat qgw;
                int num_steps = 0, idx;
                (void)distance;
                ok_vg_pose_values(g, P->dropout_id, pose); ok_tf_set_coeffs(&T_WS_i, pose, 0);
                idx = ok_vg_state_index(g, P->dropout_id);
                for (++idx; idx < ok_vg_state_count(g); ++idx) {
                    double dv[3], ds;
                    st_view(g, idx, &sv);
                    ok_vg_pose_values(g, sv.id, pose); ok_tf_set_coeffs(&T_WS_j, pose, 0);
                    dv[0] = T_WS_j.r[0] - T_WS_i.r[0]; dv[1] = T_WS_j.r[1] - T_WS_i.r[1]; dv[2] = T_WS_j.r[2] - T_WS_i.r[2];
                    ds = ok_v3_norm(dv);
                    distance += ds;
                    T_WS_i = T_WS_j;
                    num_steps++;
                }
                ok_vg_gps_get_T_GW(g, Tgw);
                qgw.x = Tgw[3]; qgw.y = Tgw[4]; qgw.z = Tgw[5]; qgw.w = Tgw[6];
                rel_err = ok_quat_angular_distance(&qgw, &T_GW_init.q) / (double)num_steps;
                budget = 0.03 + 0.004 / sqrt((double)num_steps);
                ok_gnss_logf("RI %d steps=%d rel=%.17g budget=%.17g\n", P->name, num_steps, rel_err, budget);
                if (rel_err > budget) P->reinitialised = 0;
            }
            if (P->reinitialised) {
                P->needs_full = 1;
                P->T_GW_init_ = T_GW_init;
                ok_gnss_logf("RI %d fully re-initialised\n", P->name);
            }
            break;
        default: break;
    }
    ok_gnss_logf("IS %d status=%d ret=%d\n", P->name, P->status, init_successful);
    return init_successful;
}

/* ---- :1401 addGpsInitFactors: the entries of gpsInitMap_ in key order, every factor of the entry's state each time ---- */
void ok_vgps_add_init_factors(ok_vg* g) {
    ok_vgps* P = P_(g);
    int i;
    for (i = 0; i < P->nimap; ++i) ok_vg_gps_add_residuals(g, P->imap[i].sid);
    ok_gnss_logf("IF %d entries=%d\n", P->name, P->nimap);
}

/* ---- :852 needsGpsReInit ---- */
int ok_vgps_needs_reinit(ok_vg* g) {
    ok_vgps* P = P_(g);
    uint64_t last;
    if (P->gps_states.n == 0) return 0;
    last = P->gps_states.a[P->gps_states.n - 1];
    if (P->status == OK_GPS_INITIALISED && ok_vg_pose_fixed(g, last)) {
        P->needs_pos = 1;
        P->dropout_id = last;
        ok_gnss_logf("RE %d dropout=%llu\n", P->name, (unsigned long long)last);
        return 1;
    }
    if (P->status == OK_GPS_REINITIALISING) {
        int i, n = ok_vg_state_count(g);
        ok_vg_state_view sv;
        for (i = 0; i < n; ++i) { st_view(g, i, &sv); if (sv.id >= P->pos_aligned_id) break; }          /* states_.lower_bound(positionAlignedId_) */
        if (i < n && sv.pose_fixed) { P->needs_pos = 1; ok_gnss_logf("RE %d reinit-fixed\n", P->name); return 1; }
    }
    return 0;
}
/* :872 */
void ok_vgps_reinit(ok_vg* g) {
    ok_vgps* P = P_(g);
    ok_vgps_set_status(g, OK_GPS_REINITIALISING);
    P->needs_pos = 1;
}
/* :878 */
int ok_vgps_needs_full_alignment(ok_vg* g, uint64_t* loss, uint64_t* align, double T_GW_new[7]) {
    ok_vgps* P = P_(g);
    if (!P->needs_full) return 0;
    *loss = P->dropout_id;
    *align = P->reinit_states.a[P->reinit_states.n - 1];
    tf_coeffs(&P->T_GW_init_, T_GW_new);
    return 1;
}
/* :891 */
int ok_vgps_needs_pos_alignment(ok_vg* g, uint64_t* loss, uint64_t* align, double pos_error[3]) {
    ok_vgps* P = P_(g);
    entry last;
    int ri;
    ok_vg_state_view sv;
    ok_gps_async test;
    double info[9], pose[7], sb[9], tgw[7], res[3];
    const double* params[3];
    ok_tf T_GW_original, inv;
    if (ok_vg_gps_robust(g)) return 0;
    if (!P->needs_pos) return 0;
    *loss = P->dropout_id;
    last = P->imap[P->nimap - 1];                                /* gpsInitMap_.rbegin()->second */
    ri = ok_vg_state_count(g) - 1;
    while (ri >= 0) { st_view(g, ri, &sv); if (ok_time_lt(last.f.t, sv.ts)) --ri; else break; }
    if (ri < 0) return 0;
    st_view(g, ri, &sv);
    ok_gps_inverse3(last.f.cov, info);
    ok_gps_async_init(&test, last.f.pos, info, ok_vg_gps_r_SA(g), P->imuq, P->nimuq, ok_vg_imu_params(g), sv.ts, last.f.t);
    ok_vg_pose_values(g, sv.id, pose); ok_vg_sb_values(g, sv.id, sb); ok_vg_gps_get_T_GW(g, tgw);
    params[0] = pose; params[1] = sb; params[2] = tgw;
    ok_tf_convert(&T_GW_original, tgw);
    ok_gps_async_evaluate(&test, params, res, NULL, NULL);
    ok_tf_inverse(&T_GW_original, &inv, 1);
    ok_m3_mulv(inv.C, test.error, pos_error);                    /* T_GW_original.inverse().C() * posError_G */
    *align = sv.id;
    P->pos_aligned_id = sv.id;
    ok_gnss_logf("PA %d loss=%llu align=%llu err=%.17g,%.17g,%.17g\n", P->name, (unsigned long long)*loss, (unsigned long long)*align,
                 pos_error[0], pos_error[1], pos_error[2]);
    ok_gps_async_free(&test);
    return 1;
}
/* :957 resetFullGpsAlignment, :969 resetPosGpsAlignment, ViGraph.hpp:508 resetInitialGpsAlignment */
void ok_vgps_reset_full_alignment(ok_vg* g) {
    ok_vgps* P = P_(g);
    P->reinitialised = 0;
    P->needs_full = 0;
    P->needs_pos = 0;
    ok_vgps_set_status(g, OK_GPS_INITIALISED);
    P->nimap = 0;
    P->nimuq = 0;
    P->reinit_states.n = 0;
}
void ok_vgps_reset_pos_alignment(ok_vg* g) { P_(g)->needs_pos = 0; }
void ok_vgps_reset_initial_alignment(ok_vg* g) { ok_vgps* P = P_(g); P->nimap = 0; P->nimuq = 0; P->needs_initial = 0; }
