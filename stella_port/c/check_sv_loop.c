/* SV_PORT_SOURCES: check_sv_loop.c sv_bow_db.c sv_g2o_sim3.c sv_sim3.c sv_eigen_lu3.c sv_map_match.c sv_eigen_svd.c sv_eigen_qr.c sv_rbtree.c sv_bundle_adjuster.c sv_g2o_ba.c sv_umap_order.c sv_eigen_amd.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c
 * SPDX-License-Identifier: MIT
 *
 * Harness for module 7 (loop closing): replays global_optimization_module::run_step() for EVERY keyframe step of
 * the deterministic single-threaded reference run, teacher forced from the loop dump of
 * tools/dump_stella_loop.py (patch 0013): the map at the start of the step (loop_snap.tsv, phase 0) is loaded,
 * the loop detector (BoW candidate acquisition, continuity sets, candidate validation incl. Sim3 estimation and
 * the final projection re-search) runs on it, and -- for an accepted loop -- correct_loop (Sim3 propagation over
 * the covisibility neighbours, landmark correction / fusion, new connections, essential graph optimization, loop
 * BA and the map update after it) follows.
 *   * every event the C port emits is formatted exactly like the reference's trace line (util/loop_trace.h,
 *     %a doubles) and compared as text with loop_events.tsv (candidate lists, continuity sets, matches,
 *     pose-optimizer results, Sim3 solver / transform optimizer results, accepted loop, corrected Sim3s and
 *     landmark positions, pose graph vertices / edges / result, ...);
 *   * the map after the pose graph (phase 1) and after the loop BA (phase 2) is compared with the reference
 *     snapshots (pose bits, spanning tree, loop edges, connected maps, covisibility orders, landmark position /
 *     descriptor / normal / observation lists ...);
 *   * the BoW database content the reference used (DBIDS) is compared with the model "alive keyframes 1..cur-1".
 * Injected inputs (see sv_loop.h): the RANSAC result of solve::pnp_solver (PNP event) and the previous step's
 * continuity sets (CONT events of step-1, carried like the reference does).
 * Bit-exact when the dump was made with STELLA_PORT_EIGEN_SOLVER=1 (dump_stella_loop.py --eigen-solver, marker
 * loop_params.txt); otherwise pose graph / loop BA results are compared with a relative tolerance
 * (SV_LOOP_TOL, default 1e-9) because the reference then used the CSparse solver.
 * Prints "<seq>: mismatches/total" (+ per-category breakdown on stderr; SV_MAX_REPORT limits the diagnostics).
 * usage: check_sv_loop <seq_label> <fixtures_dir(unused)> <dump_dir> [max_steps]
 * (run from the repo root: reads external/candidates/orb_vocab.fbow) */
#include "sv_loop.c"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS 512

static void copy_to(char* dst, const char* src) {
    memcpy(dst, src, strlen(src) + 1);
}

/* ------------------------------------------------------------------ */
/* file / line helpers                                                */
/* ------------------------------------------------------------------ */
static char* slurp(const char* path, size_t* len_out) {
    FILE* f = fopen(path, "rb");
    char* buf;
    long n;
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (char*)malloc((size_t)n + 2);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    buf[n] = '\n';
    buf[n + 1] = 0;
    *len_out = (size_t)n + 1;
    return buf;
}

typedef struct lines_t {
    char** p;
    size_t n, cap;
} lines_t;

static void split_lines(char* buf, size_t len, lines_t* out) {
    size_t i = 0, start = 0;
    memset(out, 0, sizeof(*out));
    for (i = 0; i < len; ++i) {
        if (buf[i] == '\n') {
            buf[i] = 0;
            if (i > start) {
                if (out->n == out->cap) {
                    out->cap = out->cap ? out->cap * 2 : 1024;
                    out->p = (char**)realloc(out->p, out->cap * sizeof(char*));
                }
                out->p[out->n++] = buf + start;
            }
            start = i + 1;
        }
    }
}

/* splits `line` in place on tabs (empty fields kept); returns the number of fields */
static int split_tabs(char* line, char** f, int max) {
    int n = 0;
    char* p = line;
    f[n++] = p;
    while (*p) {
        if (*p == '\t') {
            *p = 0;
            if (n < max) {
                f[n++] = p + 1;
            }
        }
        ++p;
    }
    return n;
}

static double hexd(const char* s) { return strtod(s, NULL); }

static int parse_ints(const char* s, int* out, int max) {
    int n = 0;
    const char* p = s;
    while (*p && n < max) {
        char* end;
        out[n++] = (int)strtol(p, &end, 10);
        p = end;
        if (*p == ',') {
            ++p;
        }
        else {
            break;
        }
    }
    return n;
}

static int parse_doubles(const char* s, double* out, int max) {
    int n = 0;
    const char* p = s;
    while (*p && n < max) {
        char* end;
        out[n++] = strtod(p, &end);
        p = end;
        if (*p == ',') {
            ++p;
        }
        else {
            break;
        }
    }
    return n;
}

/* "a:b,c:d" -> pairs */
static int parse_pairs(const char* s, int* a, int* b, int max) {
    int n = 0;
    const char* p = s;
    while (*p && n < max) {
        char* end;
        a[n] = (int)strtol(p, &end, 10);
        if (*end != ':') {
            break;
        }
        b[n] = (int)strtol(end + 1, &end, 10);
        ++n;
        p = end;
        if (*p == ',') {
            ++p;
        }
        else {
            break;
        }
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* counters / reporting                                               */
/* ------------------------------------------------------------------ */
static unsigned long g_total = 0, g_mism = 0, g_reported = 0, g_reported_trace = 0;
static unsigned long g_cat_total[64], g_cat_mism[64];
static const char* g_cat_name[64];
static int g_ncat = 0;
static double g_tol = 0.0;

static int cat_id(const char* name) {
    int i;
    for (i = 0; i < g_ncat; ++i) {
        if (strcmp(g_cat_name[i], name) == 0) {
            return i;
        }
    }
    g_cat_name[g_ncat] = strcpy((char*)malloc(strlen(name) + 1), name);
    return g_ncat++;
}

static void count(const char* cat, int ok) {
    const int c = cat_id(cat);
    g_total++;
    g_cat_total[c]++;
    if (!ok) {
        g_mism++;
        g_cat_mism[c]++;
    }
}

static unsigned long max_report(void) {
    const char* e = getenv("SV_MAX_REPORT");
    return e ? (unsigned long)atol(e) : 20ul;
}

static void report(const char* fmt, const char* a, const char* b, long step) {
    if (g_reported < max_report()) {
        fprintf(stderr, fmt, step, a, b);
        ++g_reported;
    }
}

static int close_enough(double a, double b) {
    if (a == b) {
        return 1;
    }
    if (g_tol <= 0.0) {
        return 0;
    }
    {
        const double d = fabs(a - b);
        const double m = fabs(b) > 1.0 ? fabs(b) : 1.0;
        return d <= g_tol * m;
    }
}

/* ------------------------------------------------------------------ */
/* keyframe observations                                              */
/* ------------------------------------------------------------------ */
typedef struct kf_obs_rec {
    int present;
    unsigned int n;
    sv_keypoint* kp;
    uint8_t* desc;
    sv_tr_obs obs;
} kf_obs_rec;

static kf_obs_rec* g_obs;
static unsigned int g_obs_cap;

static int hexval(char c) { return c >= 'a' ? c - 'a' + 10 : c - '0'; }

static void load_kfobs(const char* dir, const sv_tr_config* cfg) {
    char path[4096];
    size_t len;
    char* buf;
    lines_t ls;
    size_t i;
    char* f[16];
    snprintf(path, sizeof(path), "%s/loop_kfobs.tsv", dir);
    buf = slurp(path, &len);
    if (!buf) {
        fprintf(stderr, "check_sv_loop: cannot open %s\n", path);
        exit(2);
    }
    split_lines(buf, len, &ls);
    /* pass 1: sizes */
    for (i = 0; i < ls.n; ++i) {
        const unsigned int id = (unsigned int)atol(ls.p[i]);
        if (id >= g_obs_cap) {
            const unsigned int nc = id + 64;
            g_obs = (kf_obs_rec*)realloc(g_obs, nc * sizeof(kf_obs_rec));
            memset(g_obs + g_obs_cap, 0, (nc - g_obs_cap) * sizeof(kf_obs_rec));
            g_obs_cap = nc;
        }
        g_obs[id].n++;
        g_obs[id].present = 1;
    }
    for (i = 0; i < g_obs_cap; ++i) {
        if (g_obs[i].present) {
            g_obs[i].kp = (sv_keypoint*)calloc(g_obs[i].n ? g_obs[i].n : 1, sizeof(sv_keypoint));
            g_obs[i].desc = (uint8_t*)calloc(g_obs[i].n ? g_obs[i].n : 1, 32);
            g_obs[i].n = 0;
        }
    }
    for (i = 0; i < ls.n; ++i) {
        int nf = split_tabs(ls.p[i], f, 16);
        unsigned int id, k;
        kf_obs_rec* r;
        int j;
        if (nf < 7) {
            continue;
        }
        id = (unsigned int)atol(f[0]);
        r = &g_obs[id];
        k = r->n++;
        r->kp[k].x = (float)hexd(f[2]);
        r->kp[k].y = (float)hexd(f[3]);
        r->kp[k].octave = atoi(f[4]);
        r->kp[k].angle = (float)hexd(f[5]);
        for (j = 0; j < 32; ++j) {
            r->desc[k * 32 + j] = (uint8_t)((hexval(f[6][2 * j]) << 4) | hexval(f[6][2 * j + 1]));
        }
    }
    for (i = 0; i < g_obs_cap; ++i) {
        if (g_obs[i].present) {
            sv_tr_obs_init(&g_obs[i].obs, cfg, g_obs[i].kp, g_obs[i].desc, g_obs[i].n);
        }
    }
    free(ls.p); /* the text buffer is intentionally kept (unused) */
}

/* ------------------------------------------------------------------ */
/* snapshot index                                                     */
/* ------------------------------------------------------------------ */
typedef struct range_t {
    size_t first, n;
} range_t;

static lines_t g_snap;
static range_t* g_snap_idx; /* [step * 3 + phase] */
static unsigned int g_max_step;

static void index_snapshots(void) {
    size_t i;
    unsigned int max_step = 0;
    for (i = 0; i < g_snap.n; ++i) {
        const unsigned int s = (unsigned int)atol(g_snap.p[i]);
        if (s > max_step) {
            max_step = s;
        }
    }
    g_max_step = max_step;
    g_snap_idx = (range_t*)calloc((max_step + 2) * 3, sizeof(range_t));
    for (i = 0; i < g_snap.n; ++i) {
        const char* p = g_snap.p[i];
        char* end;
        const unsigned int s = (unsigned int)strtol(p, &end, 10);
        const unsigned int ph = (unsigned int)strtol(end + 1, &end, 10);
        range_t* r = &g_snap_idx[s * 3 + ph];
        if (r->n == 0) {
            r->first = i;
        }
        r->n++;
    }
}

/* ------------------------------------------------------------------ */
/* world (map + mapping context) from a snapshot phase                 */
/* ------------------------------------------------------------------ */
typedef struct world {
    sv_tr_map map;
    sv_mapping mp;
    unsigned int expired_next;
} world;

static void world_free(world* w) {
    unsigned int i;
    for (i = 0; i < w->map.kf_cap; ++i) {
        sv_tr_kf* k = w->map.kfs[i];
        if (k) {
            free(k->lm);
            free(k->covis);
            free(k->covis_w);
            free(k->children);
            free(k);
        }
    }
    for (i = 0; i < w->map.lm_cap; ++i) {
        sv_tr_lm* l = w->map.lms[i];
        if (l) {
            free(l->obs_kf);
            free(l->obs_idx);
            free(l);
        }
    }
    free(w->map.kfs);
    free(w->map.lms);
    sv_mapping_free(&w->mp);
}

/* map slots of records erased during the step are NULL but the records live on: keep them for freeing */
static sv_tr_lm** g_all_lms;
static unsigned int g_all_lms_n;

static void world_build(world* w, const sv_tr_config* cfg, unsigned int step, unsigned int phase, sv_loop* L) {
    const range_t r = g_snap_idx[step * 3 + phase];
    size_t i;
    unsigned int max_kf = 0, max_lm = 0;
    char* f[32];
    char* tmp = (char*)malloc(1 << 22);
    memset(w, 0, sizeof(*w));
    for (i = r.first; i < r.first + r.n; ++i) {
        const char* p = g_snap.p[i];
        char* q;
        unsigned int id;
        strtol(p, &q, 10);
        strtol(q + 1, &q, 10);
        if (q[1] == 'K') {
            id = (unsigned int)strtol(q + 3, NULL, 10);
            if (id > max_kf) {
                max_kf = id;
            }
        }
        else {
            id = (unsigned int)strtol(q + 3, NULL, 10);
            if (id > max_lm) {
                max_lm = id;
            }
        }
    }
    w->map.kf_cap = max_kf + 1;
    w->map.lm_cap = max_lm + 1;
    w->map.kfs = (sv_tr_kf**)calloc(w->map.kf_cap, sizeof(sv_tr_kf*));
    w->map.lms = (sv_tr_lm**)calloc(w->map.lm_cap, sizeof(sv_tr_lm*));
    w->map.last_inserted_kf = SV_TR_NONE;
    sv_mapping_init(&w->mp, cfg, &w->map);
    w->expired_next = w->map.kf_cap + 16;
    L->mp = &w->mp;
    L->map = &w->map;
    /* landmarks first (slots of the keyframes come from their observation lists) */
    for (i = r.first; i < r.first + r.n; ++i) {
        int nf;
        sv_tr_lm* lm;
        int j, no;
        copy_to(tmp, g_snap.p[i]);
        nf = split_tabs(tmp, f, 32);
        if (nf < 13 || f[2][0] != 'L') {
            continue;
        }
        lm = (sv_tr_lm*)calloc(1, sizeof(sv_tr_lm));
        lm->id = (unsigned int)atol(f[3]);
        lm->alive = 1;
        parse_doubles(f[4], lm->pos_w, 3);
        if (strlen(f[5]) >= 64) {
            for (j = 0; j < 32; ++j) {
                lm->desc[j] = (uint8_t)((hexval(f[5][2 * j]) << 4) | hexval(f[5][2 * j + 1]));
            }
        }
        parse_doubles(f[6], lm->mean_normal, 3);
        lm->min_valid_dist = (float)hexd(f[7]);
        lm->max_valid_dist = (float)hexd(f[8]);
        lm->num_observed = (unsigned int)atol(f[9]);
        lm->num_observable = (unsigned int)atol(f[10]);
        lm->ref_kf = atoi(f[11]);
        {
            int* a = (int*)malloc(4096 * sizeof(int));
            int* b = (int*)malloc(4096 * sizeof(int));
            no = parse_pairs(f[12], a, b, 4096);
            lm->num_obs = (unsigned int)no;
            lm->obs_kf = (unsigned int*)malloc((no ? no : 1) * sizeof(unsigned int));
            lm->obs_idx = (unsigned int*)malloc((no ? no : 1) * sizeof(unsigned int));
            for (j = 0; j < no; ++j) {
                lm->obs_kf[j] = (unsigned int)a[j];
                lm->obs_idx[j] = (unsigned int)b[j];
            }
            free(a);
            free(b);
        }
        w->map.lms[lm->id] = lm;
    }
    /* keyframes */
    for (i = r.first; i < r.first + r.n; ++i) {
        int nf;
        sv_tr_kf* kf;
        double pose_rm[16], pose_cm[16];
        int j, n;
        int ids[4096], ws[4096];
        copy_to(tmp, g_snap.p[i]);
        nf = split_tabs(tmp, f, 32);
        if (nf < 12 || f[2][0] != 'K') {
            continue;
        }
        kf = (sv_tr_kf*)calloc(1, sizeof(sv_tr_kf));
        kf->id = (unsigned int)atol(f[3]);
        kf->alive = 1;
        kf->obs = &g_obs[kf->id].obs;
        kf->lm = (int*)malloc((kf->obs->num_kp ? kf->obs->num_kp : 1) * sizeof(int));
        for (j = 0; j < (int)kf->obs->num_kp; ++j) {
            kf->lm[j] = SV_TR_NONE;
        }
        parse_doubles(f[4], pose_rm, 16);
        for (j = 0; j < 16; ++j) {
            pose_cm[(j % 4) * 4 + (j / 4)] = pose_rm[j];
        }
        sv_tr_kf_set_pose_cw(kf, pose_cm);
        kf->is_root = atoi(f[6]);
        kf->parent = atoi(f[7]);
        {
            int ch[4096];
            n = parse_ints(f[8], ch, 4096);
            kf->n_children = (unsigned int)n;
            kf->children = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
            for (j = 0; j < n; ++j) {
                kf->children[j] = (unsigned int)ch[j];
            }
        }
        {
            int le[4096];
            n = parse_ints(f[9], le, 4096);
            if (n > 0) {
                unsigned int tmp_le[4096];
                for (j = 0; j < n; ++j) {
                    tmp_le[j] = (unsigned int)le[j];
                }
                sv_loop_set_loop_edges(L, kf->id, tmp_le, (unsigned int)n);
            }
            else {
                sv_loop_set_loop_edges(L, kf->id, NULL, 0);
            }
        }
        /* ordered covisibilities (-1 = expired) */
        n = parse_pairs(f[11], ids, ws, 4096);
        kf->n_covis = (unsigned int)n;
        kf->covis = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
        kf->covis_w = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
        for (j = 0; j < n; ++j) {
            kf->covis[j] = ids[j] < 0 ? SV_LOOP_NONE : (unsigned int)ids[j];
            kf->covis_w[j] = (unsigned int)ws[j];
        }
        /* connected map, in map order; an expired key gets a placeholder id (its tree position is what matters) */
        {
            sv_rbtree* t = conn_tree(&w->mp, kf->id);
            n = parse_pairs(f[10], ids, ws, 4096);
            for (j = 0; j < n; ++j) {
                unsigned int key;
                if (ids[j] < 0) {
                    key = w->expired_next++;
                    sv_mapping_set_expired(&w->mp, key, 1);
                }
                else {
                    key = (unsigned int)ids[j];
                }
                sv_rb_insert_at_end(t, key, (unsigned int)ws[j], conn_less, &w->mp);
            }
        }
        w->map.kfs[kf->id] = kf;
        w->map.num_keyframes++;
    }
    /* keyframe landmark slots from the observation lists */
    for (i = 0; i < w->map.lm_cap; ++i) {
        const sv_tr_lm* lm = w->map.lms[i];
        unsigned int o;
        if (!lm) {
            continue;
        }
        for (o = 0; o < lm->num_obs; ++o) {
            sv_tr_kf* k = lm->obs_kf[o] < w->map.kf_cap ? w->map.kfs[lm->obs_kf[o]] : NULL;
            if (k && lm->obs_idx[o] < k->obs->num_kp) {
                k->lm[lm->obs_idx[o]] = (int)lm->id;
            }
        }
    }
    free(tmp);
    (void)g_all_lms;
    (void)g_all_lms_n;
}

/* ------------------------------------------------------------------ */
/* trace capture (text identical to util/loop_trace.h)                */
/* ------------------------------------------------------------------ */
typedef struct sbuf {
    char* s;
    size_t n, cap;
} sbuf;

static void sb_put(sbuf* b, const char* fmt, ...) {
    va_list ap;
    char tmp[512];
    int k;
    va_start(ap, fmt);
    k = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (k < 0) {
        return;
    }
    if (b->n + (size_t)k + 1 > b->cap) {
        b->cap = (b->n + (size_t)k + 1) * 2;
        b->s = (char*)realloc(b->s, b->cap);
    }
    memcpy(b->s + b->n, tmp, (size_t)k);
    b->n += (size_t)k;
    b->s[b->n] = 0;
}

static char** g_mine;
static size_t g_mine_n, g_mine_cap;
static unsigned int g_cur_step;

static void trace_cb(void* user, const char* tag, const sv_tf* f, int nf) {
    sbuf b;
    int i;
    unsigned int k;
    (void)user;
    memset(&b, 0, sizeof(b));
    sb_put(&b, "%u\t%s", g_cur_step, tag);
    for (i = 0; i < nf; ++i) {
        switch (f[i].k) {
            case 'u':
                sb_put(&b, "\t%lu", (unsigned long)f[i].i);
                break;
            case 'i':
                sb_put(&b, "\t%ld", f[i].i);
                break;
            case 'd':
                sb_put(&b, "\t%a", f[i].d);
                break;
            case 'L':
                sb_put(&b, "\t");
                for (k = 0; k < f[i].n; ++k) {
                    sb_put(&b, "%s%d", k ? "," : "", f[i].l[k]);
                }
                break;
            case 'A': {
                int first = 1;
                sb_put(&b, "\t");
                for (k = 0; k < f[i].n; ++k) {
                    if (f[i].l[k] < 0) {
                        continue;
                    }
                    sb_put(&b, "%s%u:%d", first ? "" : ",", k, f[i].l[k]);
                    first = 0;
                }
                break;
            }
            case 'T':
                sb_put(&b, "\t");
                for (k = 0; k < f[i].n; ++k) {
                    sb_put(&b, "%s%d:%d:%d", k ? "," : "", f[i].l[3 * k], f[i].l[3 * k + 1], f[i].l[3 * k + 2]);
                }
                break;
            case 'P':
                sb_put(&b, "\t");
                for (k = 0; k < f[i].n; ++k) {
                    sb_put(&b, "%s%d:%d", k ? "," : "", f[i].l[2 * k], f[i].l[2 * k + 1]);
                }
                break;
            default:
                break;
        }
    }
    if (g_mine_n == g_mine_cap) {
        g_mine_cap = g_mine_cap ? g_mine_cap * 2 : 256;
        g_mine = (char**)realloc(g_mine, g_mine_cap * sizeof(char*));
    }
    g_mine[g_mine_n++] = b.s;
}

/* tolerant comparison of two trace lines: identical text, or (with g_tol > 0) equal tags and every hex-float
 * field numerically close */
static int lines_equal(const char* a, const char* b) {
    if (strcmp(a, b) == 0) {
        return 1;
    }
    if (g_tol <= 0.0) {
        return 0;
    }
    {
        const char *pa = a, *pb = b;
        int tab_a = 0, tab_b = 0;
        while (*pa && *pb) {
            if (*pa == '\t') {
                ++tab_a;
            }
            if (*pb == '\t') {
                ++tab_b;
            }
            if ((*pa == '0' && pa[1] == 'x' && *pb == '0' && pb[1] == 'x') ||
                (*pa == '-' && pa[1] == '0' && pa[2] == 'x' && *pb == '-' && pb[1] == '0' && pb[2] == 'x')) {
                char *ea, *eb;
                const double da = strtod(pa, &ea), db = strtod(pb, &eb);
                if (!close_enough(da, db)) {
                    return 0;
                }
                pa = ea;
                pb = eb;
                continue;
            }
            if (*pa != *pb) {
                return 0;
            }
            ++pa;
            ++pb;
        }
        return *pa == 0 && *pb == 0;
    }
}

/* ------------------------------------------------------------------ */
/* events of a step                                                   */
/* ------------------------------------------------------------------ */
static lines_t g_ev;
static range_t* g_ev_idx; /* [step] */
static unsigned int g_ev_max_step;

static void index_events(void) {
    size_t i;
    unsigned int max_step = 0;
    for (i = 0; i < g_ev.n; ++i) {
        const unsigned int s = (unsigned int)atol(g_ev.p[i]);
        if (s > max_step) {
            max_step = s;
        }
    }
    g_ev_max_step = max_step;
    g_ev_idx = (range_t*)calloc(max_step + 2, sizeof(range_t));
    for (i = 0; i < g_ev.n; ++i) {
        const unsigned int s = (unsigned int)atol(g_ev.p[i]);
        if (g_ev_idx[s].n == 0) {
            g_ev_idx[s].first = i;
        }
        g_ev_idx[s].n++;
    }
}

/* returns the line index of the `nth` (0-based) event with the given tag in `step`, or -1 */
static long find_event(unsigned int step, const char* tag, int nth) {
    size_t i;
    if (step > g_ev_max_step) {
        return -1;
    }
    for (i = g_ev_idx[step].first; i < g_ev_idx[step].first + g_ev_idx[step].n; ++i) {
        const char* p = strchr(g_ev.p[i], '\t');
        if (p && strncmp(p + 1, tag, strlen(tag)) == 0 && (p[1 + strlen(tag)] == '\t' || p[1 + strlen(tag)] == 0)) {
            if (nth-- == 0) {
                return (long)i;
            }
        }
    }
    return -1;
}

static char* copy_line(const char* s) {
    const size_t n = strlen(s) + 1;
    char* d = (char*)malloc(n);
    memcpy(d, s, n);
    return d;
}

/* PnP injection: the k-th call of a step returns the k-th PNP event */
static unsigned int g_pnp_call;
static unsigned int* g_pnp_inl;
static int g_pnp_mismatch_n_valid;

static int pnp_cb(void* user, unsigned int cand_id, unsigned int n_valid, sv_loop_pnp* out) {
    const long li = find_event(g_cur_step, "PNP", (int)g_pnp_call++);
    char* f[MAX_FIELDS];
    char* line;
    int nf;
    (void)user;
    (void)cand_id;
    if (li < 0) {
        out->valid = 0;
        return 0;
    }
    line = copy_line(g_ev.p[li]);
    nf = split_tabs(line, f, MAX_FIELDS);
    out->valid = atoi(f[2]);
    if ((unsigned int)atol(f[3]) != n_valid) {
        g_pnp_mismatch_n_valid = 1;
    }
    if (out->valid && nf >= 5 + 16) {
        int i;
        double pose[16];
        int ids[8192];
        int n;
        for (i = 0; i < 16; ++i) {
            pose[i] = hexd(f[4 + i]);
        }
        memcpy(out->pose_rm, pose, sizeof(pose));
        n = parse_ints(f[4 + 16], ids, 8192);
        free(g_pnp_inl);
        g_pnp_inl = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
        for (i = 0; i < n; ++i) {
            g_pnp_inl[i] = (unsigned int)ids[i];
        }
        out->n_inliers = (unsigned int)n;
        out->inliers = g_pnp_inl;
    }
    free(line);
    return 0;
}

/* ------------------------------------------------------------------ */
/* state comparison against a snapshot phase                          */
/* ------------------------------------------------------------------ */
static int same_d(double a, double b) { return close_enough(a, b); }

static void compare_state(world* w, sv_loop* L, unsigned int step, unsigned int phase, const char* prefix) {
    const range_t r = g_snap_idx[step * 3 + phase];
    size_t i;
    char* f[32];
    char* tmp = (char*)malloc(1 << 22);
    unsigned int nk_ref = 0, nl_ref = 0, k;
    unsigned char* kf_seen = (unsigned char*)calloc(w->map.kf_cap + 1, 1);
    unsigned char* lm_seen = (unsigned char*)calloc(w->map.lm_cap + 1, 1);
    char cname[64];
    for (i = r.first; i < r.first + r.n; ++i) {
        int nf;
        copy_to(tmp, g_snap.p[i]);
        nf = split_tabs(tmp, f, 32);
        if (nf >= 12 && f[2][0] == 'K') {
            const unsigned int id = (unsigned int)atol(f[3]);
            const sv_tr_kf* kf = id < w->map.kf_cap ? w->map.kfs[id] : NULL;
            double pose_rm[16];
            int j, n, ok;
            int ids[4096], ws[4096];
            ++nk_ref;
            snprintf(cname, sizeof(cname), "%s.kf_present", prefix);
            count(cname, kf != NULL);
            if (!kf) {
                report("step %ld: keyframe missing (%s %s)\n", f[3], "", (long)step);
                continue;
            }
            kf_seen[id] = 1;
            parse_doubles(f[4], pose_rm, 16);
            ok = 1;
            for (j = 0; j < 16; ++j) {
                ok = ok && same_d(kf->pose_cw[(j % 4) * 4 + (j / 4)], pose_rm[j]);
            }
            snprintf(cname, sizeof(cname), "%s.kf_pose", prefix);
            count(cname, ok);
            if (!ok) {
                char msg[64];
                snprintf(msg, sizeof(msg), "kf %u pose", id);
                report("step %ld: mismatch %s %s\n", msg, "", (long)step);
            }
            snprintf(cname, sizeof(cname), "%s.kf_tree", prefix);
            {
                int ch[4096];
                n = parse_ints(f[8], ch, 4096);
                ok = (kf->is_root == atoi(f[6])) && (kf->parent == atoi(f[7])) && ((unsigned int)n == kf->n_children);
                for (j = 0; ok && j < n; ++j) {
                    ok = ok && ((unsigned int)ch[j] == kf->children[j]);
                }
                count(cname, ok);
            }
            snprintf(cname, sizeof(cname), "%s.kf_loop_edges", prefix);
            {
                int le[4096];
                const unsigned int* mine;
                unsigned int nm;
                n = parse_ints(f[9], le, 4096);
                nm = sv_loop_get_loop_edges(L, id, &mine);
                ok = ((unsigned int)n == nm);
                for (j = 0; ok && j < n; ++j) {
                    ok = ok && ((unsigned int)le[j] == mine[j]);
                }
                count(cname, ok);
            }
            snprintf(cname, sizeof(cname), "%s.kf_connected", prefix);
            {
                unsigned int cid[4096], cw[4096];
                const unsigned int nc = sv_mapping_dump_conn(&w->mp, id, cid, cw, 4096);
                n = parse_pairs(f[10], ids, ws, 4096);
                ok = ((unsigned int)n == nc);
                for (j = 0; ok && j < n; ++j) {
                    ok = ok && ((cid[j] == SV_LOOP_NONE ? -1 : (int)cid[j]) == ids[j]) && (cw[j] == (unsigned int)ws[j]);
                }
                count(cname, ok);
            }
            snprintf(cname, sizeof(cname), "%s.kf_covis_order", prefix);
            {
                n = parse_pairs(f[11], ids, ws, 4096);
                ok = ((unsigned int)n == kf->n_covis);
                for (j = 0; ok && j < n; ++j) {
                    ok = ok && ((kf->covis[j] == SV_LOOP_NONE ? -1 : (int)kf->covis[j]) == ids[j]) && (kf->covis_w[j] == (unsigned int)ws[j]);
                }
                count(cname, ok);
            }
        }
        else if (nf >= 13 && f[2][0] == 'L') {
            const unsigned int id = (unsigned int)atol(f[3]);
            const sv_tr_lm* lm = id < w->map.lm_cap ? w->map.lms[id] : NULL;
            double v3[3];
            int j, ok;
            ++nl_ref;
            snprintf(cname, sizeof(cname), "%s.lm_present", prefix);
            count(cname, lm != NULL);
            if (!lm) {
                continue;
            }
            lm_seen[id] = 1;
            parse_doubles(f[4], v3, 3);
            ok = same_d(lm->pos_w[0], v3[0]) && same_d(lm->pos_w[1], v3[1]) && same_d(lm->pos_w[2], v3[2]);
            snprintf(cname, sizeof(cname), "%s.lm_pos", prefix);
            count(cname, ok);
            if (!ok) {
                char msg[64];
                snprintf(msg, sizeof(msg), "lm %u pos", id);
                report("step %ld: mismatch %s %s\n", msg, "", (long)step);
            }
            ok = 1;
            if (strlen(f[5]) >= 64) {
                for (j = 0; j < 32; ++j) {
                    ok = ok && lm->desc[j] == (uint8_t)((hexval(f[5][2 * j]) << 4) | hexval(f[5][2 * j + 1]));
                }
            }
            snprintf(cname, sizeof(cname), "%s.lm_desc", prefix);
            count(cname, ok);
            parse_doubles(f[6], v3, 3);
            ok = same_d(lm->mean_normal[0], v3[0]) && same_d(lm->mean_normal[1], v3[1]) && same_d(lm->mean_normal[2], v3[2]);
            snprintf(cname, sizeof(cname), "%s.lm_normal", prefix);
            count(cname, ok);
            if (!ok) {
                char msg[64];
                snprintf(msg, sizeof(msg), "lm %u normal", id);
                report("step %ld: mismatch %s %s\n", msg, "", (long)step);
            }
            ok = same_d((double)lm->min_valid_dist, hexd(f[7])) && same_d((double)lm->max_valid_dist, hexd(f[8]));
            snprintf(cname, sizeof(cname), "%s.lm_dist", prefix);
            count(cname, ok);
            ok = (lm->num_observed == (unsigned int)atol(f[9])) && (lm->num_observable == (unsigned int)atol(f[10])) &&
                 (lm->ref_kf == atoi(f[11]));
            snprintf(cname, sizeof(cname), "%s.lm_counters", prefix);
            count(cname, ok);
            {
                int a[4096], b[4096];
                const int no = parse_pairs(f[12], a, b, 4096);
                ok = ((unsigned int)no == lm->num_obs);
                for (j = 0; ok && j < no; ++j) {
                    ok = ok && ((unsigned int)a[j] == lm->obs_kf[j]) && ((unsigned int)b[j] == lm->obs_idx[j]);
                }
                snprintf(cname, sizeof(cname), "%s.lm_obs", prefix);
                count(cname, ok);
            }
        }
    }
    /* sets: alive records that the reference does not have */
    {
        unsigned int nk_mine = 0, nl_mine = 0;
        for (k = 0; k < w->map.kf_cap; ++k) {
            nk_mine += (w->map.kfs[k] && w->map.kfs[k]->alive) ? 1u : 0u;
        }
        for (k = 0; k < w->map.lm_cap; ++k) {
            nl_mine += (w->map.lms[k] && w->map.lms[k]->alive) ? 1u : 0u;
        }
        snprintf(cname, sizeof(cname), "%s.kf_set", prefix);
        count(cname, nk_mine == nk_ref);
        snprintf(cname, sizeof(cname), "%s.lm_set", prefix);
        count(cname, nl_mine == nl_ref);
        if (nl_mine != nl_ref) {
            char msg[80];
            snprintf(msg, sizeof(msg), "alive landmarks mine %u ref %u", nl_mine, nl_ref);
            report("step %ld: %s%s\n", msg, "", (long)step);
        }
    }
    free(kf_seen);
    free(lm_seen);
    free(tmp);
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */
typedef struct step_ctx {
    world* w;
    sv_loop* L;
    unsigned int step;
} step_ctx;

static step_ctx g_ctx;

static void hook_cb(void* user, int phase) {
    (void)user;
    if (phase == 1 && g_snap_idx[g_ctx.step * 3 + 1].n) {
        compare_state(g_ctx.w, g_ctx.L, g_ctx.step, 1, "post_graph");
    }
}

static int read_int_file(const char* dir, const char* name, const char* key, int def) {
    char path[4096];
    size_t len;
    char* buf;
    char* p;
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    buf = slurp(path, &len);
    if (!buf) {
        return def;
    }
    p = strstr(buf, key);
    if (p) {
        def = atoi(p + strlen(key));
    }
    free(buf);
    return def;
}

int main(int argc, char** argv) {
    const char* seq_label;
    const char* dump_dir;
    long max_steps = -1;
    char path[4096];
    size_t len;
    char* buf;
    uint8_t* vocab_buf;
    size_t vocab_len;
    sv_bow_vocab vocab;
    sv_image_bounds bounds;
    sv_camera_params cam = {517.306408, 516.469215, 318.643040, 255.313989, 0.262383, -0.953104, -0.005358, 0.002628, 1.163314};
    sv_tr_config cfg;
    sv_loop L;
    sv_mapping dummy_mp;
    sv_tr_map dummy_map;
    unsigned int step;
    int eigen_mode;
    unsigned int thr_neighbor_keyframes, min_num_shared_lms_graph, loop_ba_num_iter;
    sv_loop_set* carried = NULL;
    unsigned int n_carried = 0;
    unsigned long n_steps_run = 0, n_accept = 0, n_cand_ev = 0;

    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_loop <seq_label> <fixtures_dir> <dump_dir> [max_steps]\n");
        return 1;
    }
    seq_label = argv[1];
    dump_dir = argv[3];
    if (argc >= 5) {
        max_steps = atol(argv[4]);
    }
    eigen_mode = read_int_file(dump_dir, "loop_params.txt", "eigen_solver=", 0);
    thr_neighbor_keyframes = (unsigned int)read_int_file(dump_dir, "loop_params.txt", "thr_neighbor_keyframes=", 15);
    min_num_shared_lms_graph = (unsigned int)read_int_file(dump_dir, "loop_params.txt", "min_num_shared_lms_graph=", 100);
    loop_ba_num_iter = (unsigned int)read_int_file(dump_dir, "loop_params.txt", "loop_ba_num_iter=", 10);
    {
        const char* e = getenv("SV_LOOP_TOL");
        g_tol = eigen_mode ? 0.0 : (e ? atof(e) : 1e-6);
        if (e && eigen_mode) {
            g_tol = atof(e);
        }
    }

    {
        FILE* f = fopen("external/candidates/orb_vocab.fbow", "rb");
        long n;
        if (!f) {
            fprintf(stderr, "check_sv_loop: cannot open external/candidates/orb_vocab.fbow (run from the repo root)\n");
            return 2;
        }
        fseek(f, 0, SEEK_END);
        n = ftell(f);
        fseek(f, 0, SEEK_SET);
        vocab_buf = (uint8_t*)malloc((size_t)n);
        if (fread(vocab_buf, 1, (size_t)n, f) != (size_t)n) {
            return 2;
        }
        fclose(f);
        vocab_len = (size_t)n;
    }
    if (sv_bow_load_memory(vocab_buf, vocab_len, &vocab) != 0) {
        fprintf(stderr, "check_sv_loop: bad vocabulary\n");
        return 2;
    }
    sv_compute_image_bounds(&cam, 640, 480, &bounds);
    sv_tr_config_init(&cfg, cam.fx, cam.fy, cam.cx, cam.cy, &bounds, &vocab);

    load_kfobs(dump_dir, &cfg);
    snprintf(path, sizeof(path), "%s/loop_events.tsv", dump_dir);
    buf = slurp(path, &len);
    if (!buf) {
        fprintf(stderr, "check_sv_loop: cannot open %s\n", path);
        return 2;
    }
    split_lines(buf, len, &g_ev);
    index_events();
    snprintf(path, sizeof(path), "%s/loop_snap.tsv", dump_dir);
    buf = slurp(path, &len);
    if (!buf) {
        fprintf(stderr, "check_sv_loop: cannot open %s\n", path);
        return 2;
    }
    split_lines(buf, len, &g_snap);
    index_snapshots();

    memset(&dummy_map, 0, sizeof(dummy_map));
    sv_mapping_init(&dummy_mp, &cfg, &dummy_map);
    sv_loop_init(&L, &cfg, &dummy_mp);
    L.thr_neighbor_keyframes = thr_neighbor_keyframes;
    L.min_num_shared_lms_graph = min_num_shared_lms_graph;
    L.loop_ba_num_iter = loop_ba_num_iter;
    L.trace = trace_cb;
    L.pnp = pnp_cb;
    L.hook = hook_cb;

    for (step = 1; step <= g_ev_max_step; ++step) {
        long li_s, li_db, li_det;
        char* fs[MAX_FIELDS];
        char* line;
        unsigned int cur_id, prev_loop;
        int db_ids[8192], n_db;
        int detected = 0, validated = 0;
        size_t i, j, mine_first;
        world w;
        char* fdet[MAX_FIELDS];
        int have_snap = g_snap_idx[step * 3 + 0].n > 0;
        long li_init;

        if (max_steps >= 0 && (long)step > max_steps) {
            break;
        }
        g_cur_step = step;
        li_s = find_event(step, "S", 0);
        li_db = find_event(step, "DBIDS", 0);
        li_det = find_event(step, "DET", 0);
        if (li_s < 0 || li_db < 0 || li_det < 0) {
            continue;
        }
        line = copy_line(g_ev.p[li_s]);
        split_tabs(line, fs, MAX_FIELDS);
        cur_id = (unsigned int)atol(fs[2]);
        free(line);
        line = copy_line(g_ev.p[li_det]);
        split_tabs(line, fdet, MAX_FIELDS);
        prev_loop = (unsigned int)atol(fdet[3]);
        L.num_final_matches_thr = (unsigned int)atol(fdet[5]);
        L.min_continuity = (unsigned int)atol(fdet[6]);
        L.reject_by_graph_distance = atoi(fdet[7]);
        L.min_distance_on_graph = atoi(fdet[8]);
        L.num_matches_thr = (unsigned int)atol(fdet[9]);
        L.num_matches_thr_brute_force = (unsigned int)atol(fdet[10]);
        L.num_optimized_inliers_thr = (unsigned int)atol(fdet[11]);
        L.top_n_covisibilities_to_search = (unsigned int)atol(fdet[12]);
        L.num_common_words_thr_ratio = (float)hexd(fdet[13]);
        free(line);
        {
            long ci = find_event(step, "CAND", 0);
            if (ci >= 0) {
                char* fc[MAX_FIELDS];
                char* cl = copy_line(g_ev.p[ci]);
                split_tabs(cl, fc, MAX_FIELDS);
                L.thr_opt1 = (unsigned int)atol(fc[4]);
                L.thr_a = (unsigned int)atol(fc[5]);
                L.thr_b = (unsigned int)atol(fc[6]);
                free(cl);
            }
        }
        line = copy_line(g_ev.p[li_db]);
        split_tabs(line, fs, MAX_FIELDS);
        n_db = parse_ints(fs[2], db_ids, 8192);
        free(line);
        li_init = find_event(step, "INIT", 0);

        if (have_snap) {
            unsigned int db_u[8192];
            char* f_[4];
            (void)f_;
            world_build(&w, &cfg, step, 0, &L);
            g_ctx.w = &w;
            g_ctx.L = &L;
            g_ctx.step = step;
            L.hook_user = &g_ctx;
            L.prev_loop_correct_keyfrm_id = prev_loop;
            for (i = 0; i < (size_t)n_db; ++i) {
                db_u[i] = (unsigned int)db_ids[i];
            }
            /* teacher-forced previous continuity sets */
            sv_loop_clear_prev(&L);
            for (i = 0; i < n_carried; ++i) {
                sv_loop_add_prev(&L, carried[i].lead, carried[i].continuity, carried[i].ids, carried[i].n);
            }
            /* DBIDS model: alive keyframes 1 .. cur-1 (keyframe 0 is never queued to the loop detector) */
            {
                unsigned int expect[8192], ne = 0, q;
                int ok;
                for (q = 1; q < cur_id && q < w.map.kf_cap; ++q) {
                    if (w.map.kfs[q] && w.map.kfs[q]->alive) {
                        expect[ne++] = q;
                    }
                }
                ok = ((int)ne == n_db);
                for (q = 0; ok && q < ne; ++q) {
                    ok = ok && expect[q] == (unsigned int)db_ids[q];
                }
                count("bow_db_content", ok);
                if (!ok) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "db model %u vs ref %d entries", ne, n_db);
                    report("step %ld: DBIDS mismatch %s%s\n", msg, "", (long)step);
                }
            }
            /* run */
            for (j = 0; j < g_mine_n; ++j) {
                free(g_mine[j]);
            }
            g_mine_n = 0;
            g_pnp_call = 0;
            g_pnp_mismatch_n_valid = 0;
            detected = sv_loop_detect(&L, cur_id, db_u, (unsigned int)n_db);
            if (detected) {
                validated = sv_loop_validate(&L, cur_id);
                if (validated) {
                    ++n_accept;
                    sv_loop_correct(&L, cur_id);
                }
            }
            /* compare the trace (S and DBIDS are not emitted by the library) */
            {
                size_t ri = g_ev_idx[step].first, rend = g_ev_idx[step].first + g_ev_idx[step].n;
                size_t mi = 0, nref = 0, nmatch = 0;
                mine_first = 0;
                for (; ri < rend; ++ri) {
                    const char* p = strchr(g_ev.p[ri], '\t');
                    int is_skip = p && (strncmp(p + 1, "S\t", 2) == 0 || strncmp(p + 1, "DBIDS", 5) == 0);
                    const char* tagp = p ? p + 1 : "";
                    char tag[16];
                    int tl = 0;
                    if (is_skip) {
                        continue;
                    }
                    while (tagp[tl] && tagp[tl] != '\t' && tl < 15) {
                        tag[tl] = tagp[tl];
                        ++tl;
                    }
                    tag[tl] = 0;
                    ++nref;
                    if (mi < g_mine_n && lines_equal(g_mine[mi], g_ev.p[ri])) {
                        ++nmatch;
                        count("trace_line", 1);
                    }
                    else {
                        count("trace_line", 0);
                        if (g_reported_trace < max_report()) {
                            fprintf(stderr, "step %u: trace mismatch at ref line %zu (%s)\n  ref : %.300s\n  mine: %.300s\n", step, ri - g_ev_idx[step].first, tag,
                                    g_ev.p[ri], mi < g_mine_n ? g_mine[mi] : "(none)");
                            ++g_reported_trace;
                        }
                    }
                    ++mi;
                    if (strcmp(tag, "CAND") == 0) {
                        ++n_cand_ev;
                    }
                }
                if (g_mine_n > mi) {
                    /* surplus lines of the port */
                    unsigned long extra = (unsigned long)(g_mine_n - mi);
                    while (extra--) {
                        count("trace_line", 0);
                    }
                    if (g_reported < max_report()) {
                        fprintf(stderr, "step %u: port emitted %zu more trace lines than the reference\n", step, g_mine_n - mi);
                        ++g_reported;
                    }
                }
                (void)nref;
                (void)nmatch;
                (void)mine_first;
            }
            count("pnp_input", !g_pnp_mismatch_n_valid);
            if (validated && g_snap_idx[step * 3 + 2].n) {
                compare_state(&w, &L, step, 2, "post_loop");
            }
            /* the reference dump contains the phase-2 snapshot only for accepted loops */
            if (validated != (g_snap_idx[step * 3 + 2].n > 0)) {
                count("loop_accepted_flag", 0);
            }
            else {
                count("loop_accepted_flag", 1);
            }
            ++n_steps_run;
            world_free(&w);
        }

        /* carried continuity state after this step (from the reference events) */
        if (li_init >= 0) {
            long ci;
            int nth = 0;
            char* fi[MAX_FIELDS];
            char* il = copy_line(g_ev.p[li_init]);
            int init_empty;
            split_tabs(il, fi, MAX_FIELDS);
            init_empty = (fi[2][0] == 0);
            free(il);
            for (i = 0; i < n_carried; ++i) {
                free(carried[i].ids);
            }
            free(carried);
            carried = NULL;
            n_carried = 0;
            if (!init_empty) {
                while ((ci = find_event(step, "CONT", nth++)) >= 0) {
                    char* fc[MAX_FIELDS];
                    char* cl = copy_line(g_ev.p[ci]);
                    int ids[8192], n;
                    split_tabs(cl, fc, MAX_FIELDS);
                    n = parse_ints(fc[4], ids, 8192);
                    carried = (sv_loop_set*)realloc(carried, (n_carried + 1) * sizeof(sv_loop_set));
                    carried[n_carried].lead = (unsigned int)atol(fc[2]);
                    carried[n_carried].continuity = (unsigned int)atol(fc[3]);
                    carried[n_carried].n = (unsigned int)n;
                    carried[n_carried].ids = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
                    for (j = 0; j < (size_t)n; ++j) {
                        carried[n_carried].ids[j] = ids[j] < 0 ? SV_LOOP_NONE : (unsigned int)ids[j];
                    }
                    /* keep the set sorted like the port (NONE last) */
                    qsort(carried[n_carried].ids, (size_t)n, sizeof(unsigned int), cmp_uint);
                    ++n_carried;
                    free(cl);
                }
            }
        }
        (void)fdet;
    }

    fprintf(stderr, "check_sv_loop %s: %lu steps replayed, %lu accepted loops, %lu candidate validations; diag: expired-in-loop %u, stale-covis %u, dup-connect %u, tol %g\n",
            seq_label, n_steps_run, n_accept, n_cand_ev, L.n_expired_in_loop, L.n_stale_covis, L.n_dup_connect, g_tol);
    {
        int c;
        for (c = 0; c < g_ncat; ++c) {
            fprintf(stderr, "  %-24s %lu/%lu\n", g_cat_name[c], g_cat_mism[c], g_cat_total[c]);
        }
    }
    printf("%s: %lu/%lu\n", seq_label, g_mism, g_total);
    return g_mism ? 1 : 0;
}
