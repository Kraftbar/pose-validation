/* BS_PORT_SOURCES: check_bs_dump.c */
/*
 * basalt_port module M0: reader for the observe-only dump of basalt_port/reference/patches/0003 (include/basalt/utils/bs_port_dump.h).
 * Parses every record type of <dir>/{flow,imu,iter,marg,summary}.bin with bounds-checked cursors, checks that each payload is
 * consumed exactly, checks the cheap internal invariants listed below and prints counts. It is the skeleton the replay harnesses
 * (check_bs_*.c) of later modules start from: copy the cursor + record-walk code and replace the "skip" with a replay.
 *
 * Framed records: u32 tag, u64 payload bytes, payload (little endian). Every file starts with a HDR record (tag 100):
 *   u32 version(1), u32 sizeof(Scalar)(4 = float), u32 every_flow, every_imu, every_iter, every_marg, u32 full.
 * S = float32. Matrices column major. Layouts (authoritative copy is the comment next to each emitting site in the patch):
 *   1 FLOW      i64 t_ns, u64 frame_counter, u32 ncam, ncam x {u32 w, u32 h, u64 fnv1a(pixels)},
 *               ncam x {u32 n, n x {u64 id, f32 m00 m01 m02 m10 m11 m12}}                                  (ids ascending)
 *   2 IMU_PREINT i64 start_t, i64 end_t, S bg[3], ba[3], accel_cov[3], gyro_cov[3], u32 ns, ns x {i64 t, S a[3], S g[3]},
 *               i64 dt_ns, S dq[4], dp[3], dv[3], cov[81], d_state_d_ba[27], d_state_d_bg[27]
 *   5 IMU_PREDICT i64 t0, i64 t1, S g[3], S state0[16], S state1[16]            (state = quat xyzw, p, v, bg, ba)
 *   7 OPT_BEGIN i64 t_ns, u32 n_poses, n_states, n_lms, n_obs, n_hosts, aom_total, n_imu, S lambda_init,
 *               u64 state_hash, lm_order_hash, host_order_hash, lm_value_hash, u32 prior_rows, prior_cols,
 *               u64 prior_H_hash, prior_b_hash, u32 full, [u32 nl, u64 ids[nl], u32 nh, nh x {i64 frame, u64 cam}]
 *   3 ITER_STEP i64 t_ns, u32 it, j, flags(1 accepted, 2 step_valid), S lambda0, lambda1, error_total, after_vi_err,
 *               after_marg_err, l_diff, f_diff, relative_decrease, step_norminf, u32 Hdim, u64 H_hash, b_hash, inc_hash,
 *               state_pre, state_post, lm_value_post, u32 full, [S H[n*n], S b[n], S inc[n]]
 *   6 OPT_END   i64 t_ns, u32 it, it_rejected, converged, terminated, S lambda, u64 state_hash
 *   4 MARG      i64 t_ns, i64 last_state_to_marg, u32 path(0 SqrtToSqrt, 1 SqToSqrt, 2 SqToSq), is_lin_sqrt, is_sqrt,
 *               aom_total, aom_items, n_poses_to_marg, n_states_marg_all, n_states_marg_vel_bias, n_kfs_to_marg, n_kf_all,
 *               u32 n_aom, n_aom x {i64 frame, u32 start, u32 size}, u32 nk, i32 keep[nk], u32 nm, i32 marg[nm],
 *               u32 nkm, i64 kfs_to_marg[nkm], u32 nka, i64 kf_ids_all[nka],
 *               u32 prior_rows, prior_cols, u64 prior_H_hash, prior_b_hash, u32 q_rows, q_cols, u64 q_J_hash, q_r_hash,
 *               u32 out_rows, out_cols, u64 out_H_hash, u32 out_nb, u64 out_b_hash, u64 final_b_hash, u32 order_total, u32 full,
 *               [S prior_H[pr*pc], prior_b[pr], Q2Jp[qr*qc], Q2r[qr], H_new[or*oc], b_new[nb], final_b[order_total]]
 *   9 SUMMARY   u32 n, n x {u32 len, char name[len], u64 value}
 * Module M6 (patch 0004, <dir>/m6.bin, BASALT_PORT_M6=1; HDR = u32 version 1, u32 sizeof(Scalar) 4 only):
 *   20 LM_ADD   u64 id, f32 dir[2], f32 inv_dist, i64 host_frame, u64 host_cam           21 LM_OBS  i64 frame, u64 cam, u64 kpt_id, f32 pos[2]
 *   22 LM_RMFRAME i64 frame        23 LM_RMKF u32 n, i64[n] (kfs), u32 n, i64[n] (poses), u32 n, i64[n] (states)
 *   24 LM_RMLM  u64 id             25 LM_RMOBS u64 id, u32 n, n x {i64 frame, u64 cam}
 *   26 ORDER    u32 kind, u32 n_lm, u32 n_hosts, u64 lm_order_hash, u64 host_order_hash
 *   27 UNCONN   u32 n, i32 emplace_order[n], u32 m, i32 iteration_order[m]              (unordered_set<int> unconnected_obs0)
 *   28 BIAS_LIN i64 start_t, f32 bg_lin[3], f32 ba_lin[3]
 *   30 PROBLEM  u32 kind(0 optimize, 1 marginalize), i64 t_ns, u32 np, np x {i64 t, u32 lin, f32 lin[7], f32 cur[7], f32 delta[6]},
 *               u32 ns, ns x {i64 t, u32 lin, f32 lin[16], f32 cur[16], f32 delta[15]}, u32 na, na x {i64 t, u32 start, u32 size}, u32 total,
 *               u32 nimu, nimu x {i64 key, i64 start_t, i64 dt, f32 dq[4], dp[3], dv[3], cov[81], d_ba[27], d_bg[27]}, f32 g[3], gw[3], aw[3],
 *               u32 has_marg, [u32 is_sqrt, u32 n, n x {i64 t, u32 start, u32 size}, u32 total, u32 rows, u32 cols, f32 H[rows*cols], f32 b[rows]],
 *               u32 n_used (0xFFFFFFFF = none), i64[n_used], u32 n_lost (0xFFFFFFFF = none), u64[n_lost],
 *               u32 nl, nl x {u64 id, f32 dir[2], f32 inv_dist, i64 host_frame, u64 host_cam, u32 nobs, nobs x {i64 frame, u64 cam, f32 pos[2]}}
 *
 * Modules M7 / M8 (patch 0005, <dir>/m78.bin, BASALT_PORT_M78=1; HDR as m6.bin; records follow the sampling of iter.bin / marg.bin):
 *   40 OPT_PRE  i64 t_ns, f32 lambda_vee, f32 min_lambda, f32 max_lambda, f64 lm_lambda_initial, i32 max_iterations, u32 opt_started
 *   41 OPT_POST i64 t_ns, f32 lambda_vee, f32 lambda, u64 lm_value_hash
 *   42 MARG_IN  i64 t_ns, u32 max_states, u32 max_kfs, f64 ratio, u32 n, i64 kf_ids[n], u32 n, n x {i64, i32} num_points_connected, u32 n, n x {i64, i32} num_points_kf,
 *               u32 n, i64 imu keys[n], u32 n, i64 prev_opt_flow_res keys[n], u32 n, i64 frame_poses keys[n], u32 n, i64 frame_states keys[n]
 *   43 MARG_OUT i64 t_ns, u32 n, i64 kf_ids[n], u32 n, n x {i64, u32 lin} poses, u32 n, n x {i64, u32 lin} states, u32 n, i64 imu keys[n], u32 n, i64 prev keys[n],
 *               u32 n, n x {i64 t, u32 start, u32 size} marg order, u32 order_total, u32 H rows, u32 H cols, u64 hash(H), u64 hash(b), u64 state_digest,
 *               u64 lm_order_hash, host_order_hash, lm_value_hash, u32 n_landmarks, u32 n_observations
 *
 * Usage: check_bs_dump <label> <dump_dir>      prints counts; the last stdout line is "<label>: <bad>/<total>", exit 0 iff bad == 0.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const uint8_t *p; uint64_t n, i; int err; } cur_t;

static uint32_t rd32(cur_t *c) { uint32_t v = 0; if (c->i + 4 > c->n) { c->err = 1; return 0; } memcpy(&v, c->p + c->i, 4); c->i += 4; return v; }
static uint64_t rd64(cur_t *c) { uint64_t v = 0; if (c->i + 8 > c->n) { c->err = 1; return 0; } memcpy(&v, c->p + c->i, 8); c->i += 8; return v; }
static int64_t ri64(cur_t *c) { return (int64_t)rd64(c); }
static float rdf(cur_t *c) { float v = 0; if (c->i + 4 > c->n) { c->err = 1; return 0; } memcpy(&v, c->p + c->i, 4); c->i += 4; return v; }
/* read n floats, return 1 if all finite; n may be huge (checked against the remaining bytes first) */
static int rdfn(cur_t *c, uint64_t n) {
    int fin = 1;
    if (n > (c->n - c->i) / 4) { c->err = 1; return 0; }
    for (uint64_t k = 0; k < n; k++) if (!isfinite(rdf(c))) fin = 0;
    return fin;
}
static void skip(cur_t *c, uint64_t bytes) { if (bytes > c->n - c->i) { c->err = 1; return; } c->i += bytes; }

enum { T_FLOW = 1, T_IMU_PREINT = 2, T_ITER_STEP = 3, T_MARG = 4, T_IMU_PREDICT = 5, T_OPT_END = 6, T_OPT_BEGIN = 7, T_SUMMARY = 9, T_HDR = 100,
       T_LM_ADD = 20, T_LM_OBS = 21, T_LM_RMFRAME = 22, T_LM_RMKF = 23, T_LM_RMLM = 24, T_LM_RMOBS = 25, T_ORDER = 26, T_UNCONN = 27, T_BIAS_LIN = 28, T_PROBLEM = 30,
       T_OPT_PRE = 40, T_OPT_POST = 41, T_MARG_IN = 42, T_MARG_OUT = 43 };

typedef struct {
    uint64_t total, bad;
    uint64_t rec[128];
    /* flow */ uint64_t flow_kp[2], flow_nonasc, flow_nonfinite;
    /* imu */ uint64_t imu_samples, imu_dt_bad, imu_t_bad, imu_nonfinite;
    /* iter */ uint64_t it_acc, it_rej, it_invalid, it_full, it_nonfinite, opt_begin_full, hmax;
    /* marg */ uint64_t marg_path[3], marg_full, marg_rowsmax, marg_colsmax, marg_idx_bad;
    /* m6 */ uint64_t m6_nonfinite, m6_unconn_bad, m6_unconn_n, m6_problems[2], m6_problem_lms, m6_problem_obs, m6_problem_imu, m6_bad_struct;
    /* m78 */ uint64_t m78_bad, m78_kf_max, m78_order_items_max, m78_vee_max_x100;
    int hdr_ver, hdr_scalar, hdr_full, hdr_every[4];
    int nctr; char cname[64][40]; uint64_t cval[64];
} stats_t;


static void bad(stats_t *s, const char *file, uint32_t tag, uint64_t idx, const char *why) {
    s->bad++;
    if (s->bad <= 20) fprintf(stderr, "  BAD %s tag %u record %llu: %s\n", file, tag, (unsigned long long)idx, why);
}

static void parse_flow(cur_t *c, stats_t *s, const char *f, uint64_t idx) {
    ri64(c); rd64(c);
    uint32_t ncam = rd32(c);
    if (ncam == 0 || ncam > 4) { bad(s, f, T_FLOW, idx, "ncam"); c->err = 1; return; }
    for (uint32_t i = 0; i < ncam; i++) { rd32(c); rd32(c); rd64(c); }
    for (uint32_t i = 0; i < ncam; i++) {
        uint32_t n = rd32(c);
        uint64_t prev = 0;
        for (uint32_t k = 0; k < n && !c->err; k++) {
            uint64_t id = rd64(c);
            if (k && id <= prev) s->flow_nonasc++;
            prev = id;
            if (!rdfn(c, 6)) s->flow_nonfinite++;
        }
        if (i < 2) s->flow_kp[i] += n;
    }
}
static void parse_preint(cur_t *c, stats_t *s) {
    int64_t t0 = ri64(c), t1 = ri64(c);
    if (!rdfn(c, 12)) s->imu_nonfinite++;
    uint32_t ns = rd32(c);
    int64_t last = t0;
    for (uint32_t k = 0; k < ns && !c->err; k++) {
        int64_t t = ri64(c);
        if (t <= last || t > t1) s->imu_t_bad++;
        last = t;
        if (!rdfn(c, 6)) s->imu_nonfinite++;
    }
    s->imu_samples += ns;
    int64_t dt = ri64(c);
    if (dt != t1 - t0) s->imu_dt_bad++;
    if (!rdfn(c, 4 + 3 + 3 + 81 + 27 + 27)) s->imu_nonfinite++;
}
static void parse_predict(cur_t *c, stats_t *s) {
    int64_t t0 = ri64(c), t1 = ri64(c);
    if (t1 <= t0) s->imu_dt_bad++;
    if (!rdfn(c, 3 + 16 + 16)) s->imu_nonfinite++;
}
static void parse_begin(cur_t *c, stats_t *s) {
    ri64(c);
    for (int i = 0; i < 7; i++) rd32(c);
    rdf(c);
    for (int i = 0; i < 4; i++) rd64(c);
    rd32(c); rd32(c); rd64(c); rd64(c);
    uint32_t full = rd32(c);
    if (full) {
        s->opt_begin_full++;
        uint32_t nl = rd32(c);
        skip(c, 8ull * nl);
        uint32_t nh = rd32(c);
        skip(c, 16ull * nh);
    }
}
static void parse_step(cur_t *c, stats_t *s) {
    ri64(c);
    rd32(c); rd32(c);
    uint32_t flags = rd32(c);
    if (flags & 1) s->it_acc++; else s->it_rej++;
    if (!(flags & 2)) s->it_invalid++;
    if (!rdfn(c, 9)) s->it_nonfinite++;
    uint32_t n = rd32(c);
    if (n > s->hmax) s->hmax = n;
    for (int i = 0; i < 6; i++) rd64(c);
    uint32_t full = rd32(c);
    if (full) { s->it_full++; rdfn(c, (uint64_t)n * n + 2ull * n); }
}
static void parse_end(cur_t *c, stats_t *s) {
    (void)s;
    ri64(c);
    for (int i = 0; i < 4; i++) rd32(c);
    rdf(c);
    rd64(c);
}
static void parse_marg(cur_t *c, stats_t *s) {
    ri64(c); ri64(c);
    uint32_t path = rd32(c);
    if (path < 3) s->marg_path[path]++;
    for (int i = 0; i < 9; i++) rd32(c);   /* is_lin_sqrt, is_sqrt, aom_total, aom_items, n_poses, n_states_all, n_states_vb, n_kfs, n_kf_all */
    uint32_t naom = rd32(c);
    for (uint32_t i = 0; i < naom && !c->err; i++) { ri64(c); rd32(c); rd32(c); }
    uint32_t nk = rd32(c);
    skip(c, 4ull * nk);
    uint32_t nm = rd32(c);
    skip(c, 4ull * nm);
    uint32_t nkm = rd32(c);
    skip(c, 8ull * nkm);
    uint32_t nka = rd32(c);
    skip(c, 8ull * nka);
    uint32_t pr = rd32(c), pc = rd32(c);
    rd64(c); rd64(c);
    uint32_t qr = rd32(c), qc = rd32(c);
    rd64(c); rd64(c);
    uint32_t orr = rd32(c), oc = rd32(c);
    rd64(c);
    uint32_t nb = rd32(c);
    rd64(c); rd64(c);
    uint32_t ot = rd32(c);
    uint32_t full = rd32(c);
    if (nk + nm != qc) s->marg_idx_bad++;
    if (qr > s->marg_rowsmax) s->marg_rowsmax = qr;
    if (qc > s->marg_colsmax) s->marg_colsmax = qc;
    if (full) {
        s->marg_full++;
        rdfn(c, (uint64_t)pr * pc + pr + (uint64_t)qr * qc + qr + (uint64_t)orr * oc + nb + ot);
    }
}
static void parse_summary(cur_t *c, stats_t *s) {
    uint32_t n = rd32(c);
    for (uint32_t i = 0; i < n && !c->err; i++) {
        uint32_t len = rd32(c);
        if (len > 39 || c->i + len > c->n || s->nctr >= 64) { c->err = 1; return; }
        memcpy(s->cname[s->nctr], c->p + c->i, len);
        s->cname[s->nctr][len] = 0;
        c->i += len;
        s->cval[s->nctr++] = rd64(c);
    }
}

static int read_file(const char *path, uint8_t **buf, uint64_t *len) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    *buf = (uint8_t *)malloc(n ? (size_t)n : 1);
    if (!*buf || fread(*buf, 1, (size_t)n, f) != (size_t)n) { fclose(f); return 0; }
    fclose(f);
    *len = (uint64_t)n;
    return 1;
}


/* ---- module M6 records */
static void parse_unconn(cur_t *c, stats_t *s) {
    uint32_t n = rd32(c);
    if (n > (c->n - c->i) / 4) { c->err = 1; return; }
    int32_t *seq = (int32_t *)malloc(sizeof(int32_t) * (n ? n : 1));
    for (uint32_t k = 0; k < n; k++) { uint32_t v = rd32(c); seq[k] = (int32_t)v; }
    uint32_t m = rd32(c);
    if (m > (c->n - c->i) / 4) { c->err = 1; free(seq); return; }
    s->m6_unconn_n += n;
    /* the iteration order must be a permutation of the emplaced ids (all distinct, as measure() emplaces each keypoint of cam 0 once) */
    if (m != n) s->m6_unconn_bad++;
    for (uint32_t k = 0; k < m; k++) {
        int32_t v = (int32_t)rd32(c);
        uint32_t q;
        for (q = 0; q < n; q++) if (seq[q] == v) break;
        if (q == n) s->m6_unconn_bad++;
    }
    free(seq);
}
static void parse_problem(cur_t *c, stats_t *s) {
    uint32_t kind = rd32(c);
    ri64(c);
    if (kind > 1) { s->m6_bad_struct++; c->err = 1; return; }
    s->m6_problems[kind]++;
    uint32_t np = rd32(c);
    for (uint32_t i = 0; i < np && !c->err; i++) { ri64(c); rd32(c); if (!rdfn(c, 7 + 7 + 6)) s->m6_nonfinite++; }
    uint32_t ns = rd32(c);
    for (uint32_t i = 0; i < ns && !c->err; i++) { ri64(c); rd32(c); if (!rdfn(c, 16 + 16 + 15)) s->m6_nonfinite++; }
    uint32_t na = rd32(c);
    for (uint32_t i = 0; i < na && !c->err; i++) { ri64(c); rd32(c); rd32(c); }
    rd32(c);
    uint32_t nimu = rd32(c);
    s->m6_problem_imu += nimu;
    for (uint32_t i = 0; i < nimu && !c->err; i++) { ri64(c); ri64(c); ri64(c); if (!rdfn(c, 4 + 3 + 3 + 81 + 27 + 27)) s->m6_nonfinite++; }
    if (!rdfn(c, 9)) s->m6_nonfinite++;
    if (rd32(c)) {
        rd32(c);
        uint32_t n = rd32(c);
        for (uint32_t i = 0; i < n && !c->err; i++) { ri64(c); rd32(c); rd32(c); }
        rd32(c);
        uint64_t rows = rd32(c), cols = rd32(c);
        if (!rdfn(c, rows * cols + rows)) s->m6_nonfinite++;
    }
    uint32_t nu = rd32(c);
    if (nu != 0xFFFFFFFFu) skip(c, 8ull * nu);
    uint32_t nlost = rd32(c);
    if (nlost != 0xFFFFFFFFu) skip(c, 8ull * nlost);
    uint32_t nl = rd32(c);
    s->m6_problem_lms += nl;
    for (uint32_t i = 0; i < nl && !c->err; i++) {
        rd64(c);
        if (!rdfn(c, 3)) s->m6_nonfinite++;
        ri64(c); rd64(c);
        uint32_t nobs = rd32(c);
        if (nobs < 2) s->m6_bad_struct++;
        s->m6_problem_obs += nobs;
        for (uint32_t o = 0; o < nobs && !c->err; o++) { ri64(c); rd64(c); if (!rdfn(c, 2)) s->m6_nonfinite++; }
    }
}


/* M7 / M8 records (patch 0005) */
static void parse_i64_set_asc(cur_t *c, stats_t *s) {       /* u32 n, i64[n] strictly ascending (std::set / std::map keys) */
    uint32_t n = rd32(c);
    int64_t prev = 0;
    for (uint32_t i = 0; i < n && !c->err; i++) { int64_t v = ri64(c); if (i && v <= prev) s->m78_bad++; prev = v; }
}
static void parse_pairs_asc(cur_t *c, stats_t *s) {         /* u32 n, n x {i64, i32} keys strictly ascending */
    uint32_t n = rd32(c);
    int64_t prev = 0;
    for (uint32_t i = 0; i < n && !c->err; i++) { int64_t v = ri64(c); rd32(c); if (i && v <= prev) s->m78_bad++; prev = v; }
}
static void parse_opt_pre(cur_t *c, stats_t *s) {
    ri64(c);
    float vee = rdf(c), minl = rdf(c), maxl = rdf(c);
    rd64(c); rd32(c); rd32(c);
    if (!(vee >= 2.0f && minl > 0.0f && maxl > minl)) s->m78_bad++;
    if ((uint64_t)(vee * 100) > s->m78_vee_max_x100) s->m78_vee_max_x100 = (uint64_t)(vee * 100);
}
static void parse_opt_post(cur_t *c, stats_t *s) {
    ri64(c);
    float vee = rdf(c), lam = rdf(c);
    rd64(c);
    if (!(vee >= 2.0f && lam > 0.0f && isfinite(lam))) s->m78_bad++;
}
static void parse_marg_in(cur_t *c, stats_t *s) {
    ri64(c); rd32(c); rd32(c); rd64(c);
    parse_i64_set_asc(c, s);
    parse_pairs_asc(c, s); parse_pairs_asc(c, s);
    parse_i64_set_asc(c, s); parse_i64_set_asc(c, s); parse_i64_set_asc(c, s); parse_i64_set_asc(c, s);
}
static void parse_lin_asc(cur_t *c, stats_t *s) {
    uint32_t n = rd32(c);
    int64_t prev = 0;
    for (uint32_t i = 0; i < n && !c->err; i++) { int64_t v = ri64(c); uint32_t l = rd32(c); if ((i && v <= prev) || l > 1) s->m78_bad++; prev = v; }
}
static void parse_marg_out(cur_t *c, stats_t *s) {
    ri64(c);
    parse_i64_set_asc(c, s);
    parse_lin_asc(c, s); parse_lin_asc(c, s);
    parse_i64_set_asc(c, s); parse_i64_set_asc(c, s);
    uint32_t n = rd32(c);
    uint32_t expect = 0;
    int64_t prev = 0;
    for (uint32_t i = 0; i < n && !c->err; i++) {
        int64_t t = ri64(c); uint32_t st = rd32(c), sz = rd32(c);
        if ((i && t <= prev) || st != expect || (sz != 6 && sz != 15)) s->m78_bad++;
        expect += sz; prev = t;
    }
    if (n > s->m78_order_items_max) s->m78_order_items_max = n;
    uint32_t total = rd32(c), hr = rd32(c), hc = rd32(c);
    if (total != expect || hc != total || hr < 1) s->m78_bad++;
    rd64(c); rd64(c); rd64(c); rd64(c); rd64(c); rd64(c);
    rd32(c); rd32(c);
}

static void walk(const char *dir, const char *name, stats_t *s, int *present) {
    char path[1024];
    snprintf(path, sizeof path, "%s/%s.bin", dir, name);
    uint8_t *buf;
    uint64_t len;
    if (!read_file(path, &buf, &len)) { *present = 0; return; }
    *present = 1;
    uint64_t pos = 0, idx = 0;
    int first = 1;
    while (pos < len) {
        if (pos + 12 > len) { bad(s, name, 0, idx, "truncated frame header"); break; }
        uint32_t tag;
        uint64_t n;
        memcpy(&tag, buf + pos, 4);
        memcpy(&n, buf + pos + 4, 8);
        pos += 12;
        if (n > len - pos) { bad(s, name, tag, idx, "payload beyond end of file"); break; }
        cur_t c = {buf + pos, n, 0, 0};
        s->total++;
        if (tag < 128) s->rec[tag]++;
        if (first) {
            if (tag != T_HDR) bad(s, name, tag, idx, "first record is not HDR");
            first = 0;
        }
        switch (tag) {
        case T_HDR: {
            s->hdr_ver = (int)rd32(&c); s->hdr_scalar = (int)rd32(&c);
            if (c.n > 8) {                                    /* m6.bin carries only version + scalar size */
                for (int i = 0; i < 4; i++) s->hdr_every[i] = (int)rd32(&c);
                s->hdr_full = (int)rd32(&c);
            }
            if (s->hdr_ver != 1 || s->hdr_scalar != 4) bad(s, name, tag, idx, "unsupported HDR (need version 1, float)");
            break;
        }
        case T_FLOW: parse_flow(&c, s, name, idx); break;
        case T_IMU_PREINT: parse_preint(&c, s); break;
        case T_IMU_PREDICT: parse_predict(&c, s); break;
        case T_OPT_BEGIN: parse_begin(&c, s); break;
        case T_ITER_STEP: parse_step(&c, s); break;
        case T_OPT_END: parse_end(&c, s); break;
        case T_MARG: parse_marg(&c, s); break;
        case T_SUMMARY: parse_summary(&c, s); break;
        case T_LM_ADD: rd64(&c); if (!rdfn(&c, 3)) s->m6_nonfinite++; ri64(&c); rd64(&c); break;
        case T_LM_OBS: ri64(&c); rd64(&c); rd64(&c); if (!rdfn(&c, 2)) s->m6_nonfinite++; break;
        case T_LM_RMFRAME: ri64(&c); break;
        case T_LM_RMKF: for (int q = 0; q < 3; q++) { uint32_t n = rd32(&c); skip(&c, 8ull * n); } break;
        case T_LM_RMLM: rd64(&c); break;
        case T_LM_RMOBS: rd64(&c); { uint32_t n = rd32(&c); skip(&c, 16ull * n); } break;
        case T_ORDER: rd32(&c); rd32(&c); rd32(&c); rd64(&c); rd64(&c); break;
        case T_UNCONN: parse_unconn(&c, s); break;
        case T_BIAS_LIN: ri64(&c); if (!rdfn(&c, 6)) s->m6_nonfinite++; break;
        case T_PROBLEM: parse_problem(&c, s); break;
        case T_OPT_PRE: parse_opt_pre(&c, s); break;
        case T_OPT_POST: parse_opt_post(&c, s); break;
        case T_MARG_IN: parse_marg_in(&c, s); break;
        case T_MARG_OUT: parse_marg_out(&c, s); break;
        default: bad(s, name, tag, idx, "unknown tag"); c.i = c.n; break;
        }
        if (c.err) bad(s, name, tag, idx, "payload shorter than its layout");
        else if (c.i != c.n) bad(s, name, tag, idx, "payload longer than its layout");
        pos += n;
        idx++;
    }
    free(buf);
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <label> <dump_dir>\n", argv[0]); return 2; }
    const char *label = argv[1], *dir = argv[2];
    stats_t s;
    memset(&s, 0, sizeof s);
    static const char *files[] = {"flow", "imu", "iter", "marg", "summary", "m6", "m78"};
    int present[7], npresent = 0;
    for (int i = 0; i < 7; i++) { walk(dir, files[i], &s, &present[i]); npresent += present[i]; }
    if (!npresent) { fprintf(stderr, "no dump files in %s\n", dir); printf("%s: 1/1\n", label); return 1; }
    printf("dump %s (files:", dir);
    for (int i = 0; i < 7; i++) if (present[i]) printf(" %s", files[i]);
    printf(")\n  records: HDR %llu  FLOW %llu  IMU_PREINT %llu  IMU_PREDICT %llu  OPT_BEGIN %llu  ITER_STEP %llu  OPT_END %llu  MARG %llu  SUMMARY %llu\n",
           (unsigned long long)s.rec[T_HDR], (unsigned long long)s.rec[T_FLOW], (unsigned long long)s.rec[T_IMU_PREINT],
           (unsigned long long)s.rec[T_IMU_PREDICT], (unsigned long long)s.rec[T_OPT_BEGIN], (unsigned long long)s.rec[T_ITER_STEP],
           (unsigned long long)s.rec[T_OPT_END], (unsigned long long)s.rec[T_MARG], (unsigned long long)s.rec[T_SUMMARY]);
    printf("  flow: keypoints cam0 %llu cam1 %llu, id order violations %llu, non-finite %llu\n", (unsigned long long)s.flow_kp[0],
           (unsigned long long)s.flow_kp[1], (unsigned long long)s.flow_nonasc, (unsigned long long)s.flow_nonfinite);
    printf("  imu: %llu integrated samples, dt mismatches %llu, sample-time violations %llu, non-finite %llu\n",
           (unsigned long long)s.imu_samples, (unsigned long long)s.imu_dt_bad, (unsigned long long)s.imu_t_bad,
           (unsigned long long)s.imu_nonfinite);
    printf("  iter: accepted %llu rejected %llu (invalid %llu), max H dim %llu, records with full arrays %llu, non-finite %llu\n",
           (unsigned long long)s.it_acc, (unsigned long long)s.it_rej, (unsigned long long)s.it_invalid, (unsigned long long)s.hmax,
           (unsigned long long)s.it_full, (unsigned long long)s.it_nonfinite);
    printf("  marg: path SqrtToSqrt %llu SqToSqrt %llu SqToSq %llu, max Q2Jp %llu x %llu, keep+marg != cols %llu, records with full arrays %llu\n",
           (unsigned long long)s.marg_path[0], (unsigned long long)s.marg_path[1], (unsigned long long)s.marg_path[2],
           (unsigned long long)s.marg_rowsmax, (unsigned long long)s.marg_colsmax, (unsigned long long)s.marg_idx_bad,
           (unsigned long long)s.marg_full);
    if (present[5]) {
        printf("  m6: LM_ADD %llu LM_OBS %llu LM_RMFRAME %llu LM_RMKF %llu LM_RMLM %llu LM_RMOBS %llu ORDER %llu UNCONN %llu (%llu ids) BIAS_LIN %llu PROBLEM %llu (optimize %llu, marginalize %llu: %llu landmarks, %llu observations, %llu imu blocks)\n",
               (unsigned long long)s.rec[T_LM_ADD], (unsigned long long)s.rec[T_LM_OBS], (unsigned long long)s.rec[T_LM_RMFRAME], (unsigned long long)s.rec[T_LM_RMKF],
               (unsigned long long)s.rec[T_LM_RMLM], (unsigned long long)s.rec[T_LM_RMOBS], (unsigned long long)s.rec[T_ORDER], (unsigned long long)s.rec[T_UNCONN],
               (unsigned long long)s.m6_unconn_n, (unsigned long long)s.rec[T_BIAS_LIN], (unsigned long long)s.rec[T_PROBLEM], (unsigned long long)s.m6_problems[0],
               (unsigned long long)s.m6_problems[1], (unsigned long long)s.m6_problem_lms, (unsigned long long)s.m6_problem_obs, (unsigned long long)s.m6_problem_imu);
        printf("  m6: unconnected_obs0 order violations %llu, non-finite %llu, structure violations %llu\n", (unsigned long long)s.m6_unconn_bad,
               (unsigned long long)s.m6_nonfinite, (unsigned long long)s.m6_bad_struct);
    }
    if (present[6]) {
        printf("  m78: OPT_PRE %llu OPT_POST %llu MARG_IN %llu MARG_OUT %llu, max lambda_vee %.2f, max prior items %llu, set-order / layout violations %llu\n",
               (unsigned long long)s.rec[T_OPT_PRE], (unsigned long long)s.rec[T_OPT_POST], (unsigned long long)s.rec[T_MARG_IN], (unsigned long long)s.rec[T_MARG_OUT],
               s.m78_vee_max_x100 / 100.0, (unsigned long long)s.m78_order_items_max, (unsigned long long)s.m78_bad);
    }
    if (s.nctr) printf("  executed-path counters:\n");
    for (int i = 0; i < s.nctr; i++) printf("    %-26s %llu\n", s.cname[i], (unsigned long long)s.cval[i]);
    uint64_t extra = s.flow_nonasc + s.flow_nonfinite + s.imu_dt_bad + s.imu_t_bad + s.imu_nonfinite + s.it_nonfinite + s.marg_idx_bad + s.m6_nonfinite + s.m6_unconn_bad + s.m6_bad_struct + s.m78_bad;
    uint64_t bad_total = s.bad + extra;
    printf("%s: %llu/%llu\n", label, (unsigned long long)bad_total, (unsigned long long)s.total);
    return bad_total == 0 ? 0 : 1;
}
