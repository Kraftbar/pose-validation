/* SV_PORT_SOURCES: check_sv_track.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c
 * SPDX-License-Identifier: MIT
 *
 * Harness for module 5 (tracking side): teacher-forced, frame by frame replay
 * of stella_vslam's tracking_module::feed_frame() (state == Tracking) against
 * the deterministic single-threaded reference dumps under
 * runs/stella_port/reference_dumps/<seq>/.
 *
 * Inputs per frame t (all from the dump dir):
 *   - map state at the START of frame t == snapshot after frame t-1
 *     (keyframes.tsv / landmarks.tsv blocks of frame t-1; keyframe keypoints
 *     and descriptors come from the keyframe's source frame, found through
 *     keyframe_meta.tsv timestamp -> track_pre.tsv timestamp);
 *   - tracking_module state at the start of frame t (track_pre.tsv: twist,
 *     last_cam_pose_from_ref_keyfrm, last frame pose/ref keyframe, last
 *     reloc, last inserted keyframe, keyframe count) and the last frame's
 *     landmark associations (matches.tsv of frame t-1, hash-checked against
 *     track_pre.tsv);
 *   - the current frame's undistorted keypoints + descriptors
 *     (keypoints.tsv / descriptors.tsv, module 1 outputs).
 * It runs sv_tracker_track() and compares, bit for bit / id for id:
 *   track path, pose after track_current_frame() (frame_trace initial pose),
 *   final pose, every keypoint's landmark association (matches.tsv),
 *   local keyframe list and local landmark list (order too), num_tracked /
 *   num_reliable, the keyframe decision and each sub-flag (kf_decision.tsv),
 *   the reference keyframe id after insertion, and (via
 *   sv_tracker_finish_frame on the post-frame snapshot) the propagated
 *   velocity twist and last_cam_pose_from_ref_keyfrm against the NEXT
 *   frame's track_pre.tsv.
 * Each compared scalar/list/keypoint counts as one item ("total"); any
 * difference counts as a mismatch. Prints "<seq>: mismatches/total".
 * usage: check_sv_track <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (run from the repo root: reads external/candidates/orb_vocab.fbow)
 */
#include "sv_track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_MAX_LEN (1 << 20)

/* ------------------------------------------------------------------ */
/* small parsing helpers                                              */
/* ------------------------------------------------------------------ */
static char* next_field(char** p) {
    char* start = *p;
    char* tab = strchr(start, '\t');
    if (tab) {
        *tab = '\0';
        *p = tab + 1;
    } else {
        char* nl = strpbrk(start, "\r\n");
        if (nl) *nl = '\0';
        *p = start + strlen(start);
    }
    return start;
}

static int parse_hex_list(const char* s, double* out, int n) {
    int i = 0;
    const char* p = s;
    while (i < n && *p) {
        char* end;
        out[i++] = strtod(p, &end);
        if (end == p) break;
        p = end;
        if (*p == ',') ++p;
    }
    return i;
}

/* row-major 16 -> column-major 16 */
static void rowmajor_to_col(const double r[16], double c[16]) {
    int i, j;
    for (i = 0; i < 4; ++i)
        for (j = 0; j < 4; ++j) c[j * 4 + i] = r[i * 4 + j];
}

static int hexbyte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static void parse_desc_hex(const char* s, uint8_t out[32]) {
    int i;
    for (i = 0; i < 32; ++i) out[i] = (uint8_t)((hexbyte(s[2 * i]) << 4) | hexbyte(s[2 * i + 1]));
}

static FILE* open_or_die(const char* dir, const char* name) {
    char path[4096];
    FILE* f;
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "check_sv_track: cannot open %s\n", path);
        exit(2);
    }
    return f;
}

static uint8_t* read_whole_file(const char* path, size_t* len) {
    FILE* f = fopen(path, "rb");
    uint8_t* buf;
    long n;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (uint8_t*)malloc((size_t)n);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

/* ------------------------------------------------------------------ */
/* per-frame data                                                     */
/* ------------------------------------------------------------------ */
typedef struct frame_data {
    unsigned int n;
    unsigned int cap;
    sv_keypoint* kp;
    uint8_t* desc;
    int* match; /* matches.tsv landmark id per keypoint */
    sv_tr_obs obs;
    int obs_built;
    double timestamp;
} frame_data;

typedef struct trace_row {
    int have;
    char path[32];
    long ref_kf;
    int initial_valid;
    double initial_pose[16]; /* col-major */
    int final_valid;
    double final_pose[16];
    unsigned int num_tracked, num_reliable;
} trace_row;

typedef struct pre_row {
    int have;
    double timestamp;
    int tracking_state, twist_valid;
    double twist[16], last_cam_pose_from_ref[16];
    unsigned int last_reloc_frm_id;
    double last_reloc_ts;
    int last_frm_pose_valid;
    long long last_frm_id;
    double last_frm_pose[16];
    long long last_frm_ref_kf;
    unsigned int last_frm_num_lms;
    unsigned long long last_frm_hash;
    long long last_ins_id;
    double last_ins_ts, last_ins_trans_wc[3];
    unsigned int num_keyframes, fixed_thr;
} pre_row;

typedef struct list_row {
    unsigned int n;
    unsigned int* v;
} list_row;

typedef struct dec_row {
    int have;
    int verdict, paused;
    unsigned int ref, rel, trk;
    float dist;
    int f[9]; /* max_interval, min_interval, max_distance, min_distance, view_changed, not_enough, enough_kf, unstable, almost_all */
    int skipping;
} dec_row;

typedef struct ins_lm {
    unsigned int kp, id;
    double pos[3];
    uint8_t desc[32];
    double normal[3];
    float minv, maxv;
    unsigned int nobs, nobserved, nobservable;
    int ref;
} ins_lm;

typedef struct ins_rec {
    int have;
    unsigned int kf_id;
    double ts;
    double pose_cw[16], pose_wc[16], trans_wc[3];
    unsigned int lm_start, lm_count;
} ins_rec;

static void grow_frames(frame_data** fr, unsigned int* cap, unsigned int need) {
    if (need <= *cap) return;
    {
        unsigned int nc = *cap ? *cap : 1024;
        while (nc < need) nc *= 2;
        *fr = (frame_data*)realloc(*fr, nc * sizeof(frame_data));
        memset(*fr + *cap, 0, (nc - *cap) * sizeof(frame_data));
        *cap = nc;
    }
}

/* ------------------------------------------------------------------ */
/* snapshot reader (keyframes.tsv / landmarks.tsv block per frame)     */
/* ------------------------------------------------------------------ */
typedef struct block_reader {
    FILE* f;
    char* line;
    int have_line;
    long frame; /* frame index of the pending line */
} block_reader;

static void br_open(block_reader* b, FILE* f) {
    b->f = f;
    b->line = (char*)malloc(LINE_MAX_LEN);
    b->have_line = 0;
    b->frame = -1;
    if (!fgets(b->line, LINE_MAX_LEN, f)) return; /* header */
}

static void br_fill(block_reader* b) {
    if (b->have_line) return;
    if (fgets(b->line, LINE_MAX_LEN, b->f)) {
        b->have_line = 1;
        b->frame = atol(b->line);
    } else {
        b->frame = -2;
    }
}

/* drop every pending line whose frame index is below `frame` */
static void br_skip_until(block_reader* b, long frame) {
    for (;;) {
        br_fill(b);
        if (!b->have_line || b->frame >= frame) return;
        b->have_line = 0;
    }
}

/* ------------------------------------------------------------------ */
/* map construction from a snapshot                                    */
/* ------------------------------------------------------------------ */
typedef struct kf_static {
    int known;
    unsigned int src_frame;
    double timestamp;
} kf_static;

typedef struct world {
    sv_tr_config cfg;
    frame_data* frames;
    unsigned int nframes;
    kf_static* kfmeta;
    unsigned int kfmeta_cap;
    sv_tr_map map;
    sv_tr_kf** kf_rec; /* pointer-stable records, indexed by keyframe id */
    sv_tr_lm** lm_rec; /* pointer-stable records, indexed by landmark id */
} world;

static sv_tr_obs* frame_obs(world* w, unsigned int fi) {
    frame_data* fd = &w->frames[fi];
    if (!fd->obs_built) {
        sv_tr_obs_init(&fd->obs, &w->cfg, fd->kp, fd->desc, fd->n);
        fd->obs_built = 1;
    }
    return &fd->obs;
}

static void ensure_map_capacity(world* w, unsigned int kf_cap, unsigned int lm_cap) {
    sv_tr_map* m = &w->map;
    if (kf_cap > m->kf_cap) {
        unsigned int nc = kf_cap + 64, i;
        m->kfs = (sv_tr_kf**)realloc(m->kfs, nc * sizeof(sv_tr_kf*));
        w->kf_rec = (sv_tr_kf**)realloc(w->kf_rec, nc * sizeof(sv_tr_kf*));
        for (i = m->kf_cap; i < nc; ++i) {
            m->kfs[i] = NULL;
            w->kf_rec[i] = NULL;
        }
        m->kf_cap = nc;
    }
    if (lm_cap > m->lm_cap) {
        unsigned int nc = lm_cap + 4096, i;
        m->lms = (sv_tr_lm**)realloc(m->lms, nc * sizeof(sv_tr_lm*));
        w->lm_rec = (sv_tr_lm**)realloc(w->lm_rec, nc * sizeof(sv_tr_lm*));
        for (i = m->lm_cap; i < nc; ++i) {
            m->lms[i] = NULL;
            w->lm_rec[i] = NULL;
        }
        m->lm_cap = nc;
    }
}

static sv_tr_kf* kf_record(world* w, unsigned int id) {
    ensure_map_capacity(w, id + 1, w->map.lm_cap);
    if (!w->kf_rec[id]) w->kf_rec[id] = (sv_tr_kf*)calloc(1, sizeof(sv_tr_kf));
    return w->kf_rec[id];
}

static sv_tr_lm* lm_record(world* w, unsigned int id) {
    ensure_map_capacity(w, w->map.kf_cap, id + 1);
    if (!w->lm_rec[id]) w->lm_rec[id] = (sv_tr_lm*)calloc(1, sizeof(sv_tr_lm));
    return w->lm_rec[id];
}

/* Loads the snapshot block of `frame` into w->map (kfs + lms). Returns 0 on
 * success, -1 if there is no block for that frame. Pool records are reused
 * but the map's pointer tables are rebuilt each time (a NULL slot == the
 * object is not in the map). */
static int load_snapshot(world* w, block_reader* bk, block_reader* bl, long frame) {
    sv_tr_map* m = &w->map;
    unsigned int i;
    int any = 0;
    char* p;

    /* reset */
    for (i = 0; i < m->kf_cap; ++i) {
        if (m->kfs[i]) {
            sv_tr_kf* k = m->kfs[i];
            free(k->lm); k->lm = NULL;
            free(k->covis); k->covis = NULL;
            free(k->covis_w); k->covis_w = NULL;
            free(k->children); k->children = NULL;
            k->n_covis = 0; k->n_children = 0;
            m->kfs[i] = NULL;
        }
    }
    for (i = 0; i < m->lm_cap; ++i) {
        if (m->lms[i]) {
            m->lms[i]->alive = 0;
            m->lms[i] = NULL;
        }
    }
    m->num_keyframes = 0;

    br_skip_until(bk, frame);
    br_skip_until(bl, frame);

    /* keyframes */
    for (;;) {
        char* line;
        char* fld[8];
        unsigned int id, j;
        sv_tr_kf* k;
        double pose_rm[16], pose_cm[16];
        kf_static* ks;
        br_fill(bk);
        if (!bk->have_line || bk->frame != frame) break;
        any = 1;
        line = bk->line;
        p = line;
        for (j = 0; j < 8; ++j) fld[j] = next_field(&p);
        bk->have_line = 0;
        id = (unsigned int)atol(fld[1]);
        if (id >= w->kfmeta_cap || !w->kfmeta[id].known) {
            fprintf(stderr, "check_sv_track: keyframe %u has no keyframe_meta row\n", id);
            exit(2);
        }
        ks = &w->kfmeta[id];
        k = kf_record(w, id);
        free(k->lm); free(k->covis); free(k->covis_w); free(k->children);
        memset(k, 0, sizeof(*k));
        k->id = id;
        k->alive = (atoi(fld[4]) == 0);
        k->timestamp = ks->timestamp;
        k->obs = frame_obs(w, ks->src_frame);
        parse_hex_list(fld[3], pose_rm, 16);
        rowmajor_to_col(pose_rm, pose_cm);
        sv_tr_kf_set_pose_cw(k, pose_cm);
        /* covisibilities: "id:w,id:w" */
        {
            const char* c = fld[5];
            unsigned int cap = 0;
            while (*c) {
                char* end;
                unsigned long a = strtoul(c, &end, 10);
                unsigned long wgt;
                if (end == c) break;
                c = end;
                if (*c == ':') ++c;
                wgt = strtoul(c, &end, 10);
                c = end;
                if (*c == ',') ++c;
                if (k->n_covis == cap) {
                    cap = cap ? cap * 2 : 16;
                    k->covis = (unsigned int*)realloc(k->covis, cap * sizeof(unsigned int));
                    k->covis_w = (unsigned int*)realloc(k->covis_w, cap * sizeof(unsigned int));
                }
                k->covis[k->n_covis] = (unsigned int)a;
                k->covis_w[k->n_covis] = (unsigned int)wgt;
                ++k->n_covis;
            }
        }
        k->parent = (int)atol(fld[6]);
        k->is_root = (k->parent < 0); /* the snapshot dumps the root's parent as -1 */
        {
            const char* c = fld[7];
            unsigned int cap = 0;
            while (*c) {
                char* end;
                unsigned long a = strtoul(c, &end, 10);
                if (end == c) break;
                c = end;
                if (*c == ',') ++c;
                if (k->n_children == cap) {
                    cap = cap ? cap * 2 : 8;
                    k->children = (unsigned int*)realloc(k->children, cap * sizeof(unsigned int));
                }
                k->children[k->n_children++] = (unsigned int)a;
            }
        }
        k->lm = (int*)malloc((k->obs->num_kp ? k->obs->num_kp : 1) * sizeof(int));
        for (j = 0; j < k->obs->num_kp; ++j) k->lm[j] = SV_TR_NONE;
        if (k->alive) {
            m->kfs[id] = k;
            m->num_keyframes++;
        }
    }

    /* landmarks */
    for (;;) {
        char* fld[13];
        unsigned int id, j;
        sv_tr_lm* lm;
        double v3[3];
        br_fill(bl);
        if (!bl->have_line || bl->frame != frame) break;
        p = bl->line;
        for (j = 0; j < 13; ++j) fld[j] = next_field(&p);
        bl->have_line = 0;
        id = (unsigned int)atol(fld[1]);
        lm = lm_record(w, id);
        free(lm->obs_kf);
        free(lm->obs_idx);
        memset(lm, 0, sizeof(*lm));
        lm->id = id;
        lm->alive = 1;
        parse_hex_list(fld[3], v3, 3);
        lm->pos_w[0] = v3[0]; lm->pos_w[1] = v3[1]; lm->pos_w[2] = v3[2];
        parse_desc_hex(fld[4], lm->desc);
        parse_hex_list(fld[6], v3, 3);
        lm->mean_normal[0] = v3[0]; lm->mean_normal[1] = v3[1]; lm->mean_normal[2] = v3[2];
        lm->min_valid_dist = strtof(fld[7], NULL);
        lm->max_valid_dist = strtof(fld[8], NULL);
        lm->num_observed = (unsigned int)atol(fld[9]);
        lm->num_observable = (unsigned int)atol(fld[10]);
        lm->ref_kf = (int)atol(fld[11]);
        {
            const char* c = fld[12];
            unsigned int cap = 0;
            while (*c) {
                char* end;
                unsigned long a = strtoul(c, &end, 10), b;
                if (end == c) break;
                c = end;
                if (*c == ':') ++c;
                b = strtoul(c, &end, 10);
                c = end;
                if (*c == ',') ++c;
                if (lm->num_obs == cap) {
                    cap = cap ? cap * 2 : 16;
                    lm->obs_kf = (unsigned int*)realloc(lm->obs_kf, cap * sizeof(unsigned int));
                    lm->obs_idx = (unsigned int*)realloc(lm->obs_idx, cap * sizeof(unsigned int));
                }
                lm->obs_kf[lm->num_obs] = (unsigned int)a;
                lm->obs_idx[lm->num_obs] = (unsigned int)b;
                ++lm->num_obs;
                if (a < m->kf_cap && m->kfs[a] && b < m->kfs[a]->obs->num_kp) m->kfs[a]->lm[b] = (int)id;
            }
        }
        m->lms[id] = lm;
    }
    return any ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* comparison bookkeeping                                              */
/* ------------------------------------------------------------------ */
static unsigned long g_total = 0, g_mismatch = 0;
static unsigned long g_report = 0;
static long g_cur_frame = 0;

static void item(int ok, const char* what) {
    ++g_total;
    if (!ok) {
        ++g_mismatch;
        if (g_report < (getenv("SV_MAX_REPORT") ? (unsigned long)atol(getenv("SV_MAX_REPORT")) : 40ul)) {
            fprintf(stderr, "  frame %ld: mismatch in %s\n", g_cur_frame, what);
            ++g_report;
        }
    }
}

static int same_pose(const double a[16], const double b[16]) {
    return memcmp(a, b, 16 * sizeof(double)) == 0;
}

static unsigned long long lm_hash(const int* lm, unsigned int n, unsigned int* count) {
    unsigned long long h = 1469598103934665603ULL;
    unsigned int i;
    *count = 0;
    for (i = 0; i < n; ++i) {
        unsigned long long vals[2];
        int k;
        if (lm[i] < 0) continue;
        ++*count;
        vals[0] = i;
        vals[1] = (unsigned long long)lm[i];
        for (k = 0; k < 2; ++k) {
            h ^= vals[k];
            h *= 1099511628211ULL;
        }
    }
    return h;
}

static int parse_uint_list(const char* s, list_row* out) {
    unsigned int cap = 0;
    out->n = 0;
    out->v = NULL;
    while (*s) {
        char* end;
        unsigned long a = strtoul(s, &end, 10);
        if (end == s) break;
        s = end;
        if (*s == ',') ++s;
        if (out->n == cap) {
            cap = cap ? cap * 2 : 64;
            out->v = (unsigned int*)realloc(out->v, cap * sizeof(unsigned int));
        }
        out->v[out->n++] = (unsigned int)a;
    }
    return 0;
}

#ifdef SV_MAPPING_HARNESS
/* module 6 hooks (check_sv_mapping.c defines them): the mapping step of every
 * inserted keyframe is replayed on top of this harness's per-frame state. */
static void mapping_hook_begin(world* w, const pre_row* pre, unsigned int nframes, const char* dump_dir);
static void mapping_hook_after_insert(world* w, unsigned int t, int inserted_id);
static void mapping_hook_end(void);
#endif

int main(int argc, char** argv) {
    const char* seq_label;
    const char* dump_dir;
    long max_frames = -1;
    world w;
    FILE *fk, *fd, *fm, *ft, *fl, *fdec, *fpre, *fmeta, *fsk, *fsl;
    char* line;
    unsigned int nframes = 0, fcap = 0;
    trace_row* trace = NULL;
    pre_row* pre = NULL;
    list_row *lm_kf = NULL, *lm_lm = NULL;
    dec_row* dec = NULL;
    ins_rec* ins = NULL;
    ins_lm* inslms = NULL;
    unsigned int n_inslms = 0, cap_inslms = 0;
    block_reader bk, bl;
    uint8_t* vocab_buf;
    size_t vocab_len;
    sv_bow_vocab vocab;
    sv_image_bounds bounds;
    sv_camera_params cam = {517.306408, 516.469215, 318.643040, 255.313989,
                            0.262383, -0.953104, -0.005358, 0.002628, 1.163314};
    unsigned int t;
    sv_tracker trk;
    long processed = 0, robust_frames = 0;

    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_track <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 1;
    }
    seq_label = argv[1];
    dump_dir = argv[3];
    if (argc >= 5) max_frames = atol(argv[4]);
#ifdef SV_TRACK_DUMP_SUBST
    /* variant harness: replay the opt-in fault-injection dumps
     * (runs/stella_port/reference_dumps_force_*, patch 0010) instead */
    {
        static char variant_dir[4096];
        const char* pos = strstr(dump_dir, "reference_dumps");
        char probe[4200];
        FILE* pf;
        if (!pos) return 2;
        snprintf(variant_dir, sizeof(variant_dir), "%.*s%s%s", (int)(pos - dump_dir), dump_dir,
                 SV_TRACK_DUMP_SUBST, pos + strlen("reference_dumps"));
        snprintf(probe, sizeof(probe), "%s/track_pre.tsv", variant_dir);
        pf = fopen(probe, "r");
        if (!pf) {
            fprintf(stderr, "%s: no %s dump (%s) -- skipping (tools/dump_stella_reference.py --force-path ...)\n",
                    seq_label, SV_TRACK_DUMP_SUBST, variant_dir);
            printf("%s: 0/0\n", seq_label);
            return 0;
        }
        fclose(pf);
        dump_dir = variant_dir;
    }
#endif

    memset(&w, 0, sizeof(w));
    line = (char*)malloc(LINE_MAX_LEN);

    vocab_buf = read_whole_file("external/candidates/orb_vocab.fbow", &vocab_len);
    if (!vocab_buf || sv_bow_load_memory(vocab_buf, vocab_len, &vocab) != 0) {
        fprintf(stderr, "check_sv_track: cannot load external/candidates/orb_vocab.fbow (run from the repo root)\n");
        return 2;
    }
    sv_compute_image_bounds(&cam, 640, 480, &bounds);
    sv_tr_config_init(&w.cfg, cam.fx, cam.fy, cam.cx, cam.cy, &bounds, &vocab);

    /* ---- track_pre.tsv (also gives the frame count and timestamps) ---- */
    fpre = open_or_die(dump_dir, "track_pre.tsv");
    if (!fgets(line, LINE_MAX_LEN, fpre)) return 2;
    while (fgets(line, LINE_MAX_LEN, fpre)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        pre_row* r;
        double tmp[16];
        if (fi + 1 > fcap) {
            unsigned int nc = fcap ? fcap * 2 : 1024;
            while (nc < fi + 1) nc *= 2;
            pre = (pre_row*)realloc(pre, nc * sizeof(pre_row));
            memset(pre + fcap, 0, (nc - fcap) * sizeof(pre_row));
            trace = (trace_row*)realloc(trace, nc * sizeof(trace_row));
            memset(trace + fcap, 0, (nc - fcap) * sizeof(trace_row));
            dec = (dec_row*)realloc(dec, nc * sizeof(dec_row));
            memset(dec + fcap, 0, (nc - fcap) * sizeof(dec_row));
            lm_kf = (list_row*)realloc(lm_kf, nc * sizeof(list_row));
            memset(lm_kf + fcap, 0, (nc - fcap) * sizeof(list_row));
            lm_lm = (list_row*)realloc(lm_lm, nc * sizeof(list_row));
            memset(lm_lm + fcap, 0, (nc - fcap) * sizeof(list_row));
            ins = (ins_rec*)realloc(ins, nc * sizeof(ins_rec));
            memset(ins + fcap, 0, (nc - fcap) * sizeof(ins_rec));
            fcap = nc;
        }
        grow_frames(&w.frames, &w.nframes, (unsigned int)fi + 1);
        r = &pre[fi];
        r->have = 1;
        r->timestamp = strtod(next_field(&p), NULL);
        w.frames[fi].timestamp = r->timestamp;
        r->tracking_state = atoi(next_field(&p));
        r->twist_valid = atoi(next_field(&p));
        parse_hex_list(next_field(&p), tmp, 16);
        rowmajor_to_col(tmp, r->twist);
        parse_hex_list(next_field(&p), tmp, 16);
        rowmajor_to_col(tmp, r->last_cam_pose_from_ref);
        r->last_reloc_frm_id = (unsigned int)atol(next_field(&p));
        r->last_reloc_ts = strtod(next_field(&p), NULL);
        r->last_frm_pose_valid = atoi(next_field(&p));
        r->last_frm_id = atoll(next_field(&p));
        parse_hex_list(next_field(&p), tmp, 16);
        rowmajor_to_col(tmp, r->last_frm_pose);
        r->last_frm_ref_kf = atoll(next_field(&p));
        r->last_frm_num_lms = (unsigned int)atol(next_field(&p));
        r->last_frm_hash = strtoull(next_field(&p), NULL, 10);
        r->last_ins_id = atoll(next_field(&p));
        r->last_ins_ts = strtod(next_field(&p), NULL);
        parse_hex_list(next_field(&p), r->last_ins_trans_wc, 3);
        r->num_keyframes = (unsigned int)atol(next_field(&p));
        r->fixed_thr = (unsigned int)atol(next_field(&p));
        if (fi + 1 > nframes) nframes = (unsigned int)fi + 1;
    }
    fclose(fpre);
    w.nframes = fcap ? (w.nframes > nframes ? w.nframes : nframes) : 0;

    /* ---- keypoints.tsv ---- */
    fk = open_or_die(dump_dir, "keypoints.tsv");
    if (!fgets(line, LINE_MAX_LEN, fk)) return 2;
    while (fgets(line, LINE_MAX_LEN, fk)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        frame_data* f = &w.frames[fi];
        sv_keypoint* kp;
        next_field(&p); /* kp_idx */
        next_field(&p); /* x_9g */
        if (f->n == f->cap) {
            f->cap = f->cap ? f->cap * 2 : 1024;
            f->kp = (sv_keypoint*)realloc(f->kp, f->cap * sizeof(sv_keypoint));
        }
        kp = &f->kp[f->n++];
        memset(kp, 0, sizeof(*kp));
        kp->x = strtof(next_field(&p), NULL);
        next_field(&p); /* y_9g */
        kp->y = strtof(next_field(&p), NULL);
        kp->octave = atoi(next_field(&p));
        next_field(&p); /* angle_9g */
        kp->angle = strtof(next_field(&p), NULL);
    }
    fclose(fk);

    /* ---- descriptors.tsv ---- */
    fd = open_or_die(dump_dir, "descriptors.tsv");
    if (!fgets(line, LINE_MAX_LEN, fd)) return 2;
    {
        unsigned int fi_idx;
        for (fi_idx = 0; fi_idx < w.nframes; ++fi_idx) {
            w.frames[fi_idx].desc = (uint8_t*)calloc(w.frames[fi_idx].n ? w.frames[fi_idx].n : 1, 32);
            w.frames[fi_idx].match = (int*)malloc((w.frames[fi_idx].n ? w.frames[fi_idx].n : 1) * sizeof(int));
        }
    }
    while (fgets(line, LINE_MAX_LEN, fd)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        unsigned long ki = (unsigned long)atol(next_field(&p));
        char* hex = next_field(&p);
        if (fi < w.nframes && ki < w.frames[fi].n) parse_desc_hex(hex, w.frames[fi].desc + ki * 32);
    }
    fclose(fd);

    /* ---- matches.tsv ---- */
    fm = open_or_die(dump_dir, "matches.tsv");
    if (!fgets(line, LINE_MAX_LEN, fm)) return 2;
    while (fgets(line, LINE_MAX_LEN, fm)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        unsigned long ki = (unsigned long)atol(next_field(&p));
        int id = atoi(next_field(&p));
        if (fi < w.nframes && ki < w.frames[fi].n) w.frames[fi].match[ki] = id;
    }
    fclose(fm);

    /* ---- frame_trace.tsv ---- */
    ft = open_or_die(dump_dir, "frame_trace.tsv");
    if (!fgets(line, LINE_MAX_LEN, ft)) return 2;
    while (fgets(line, LINE_MAX_LEN, ft)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        trace_row* r = &trace[fi];
        double tmp[16];
        r->have = 1;
        snprintf(r->path, sizeof(r->path), "%s", next_field(&p));
        r->ref_kf = atol(next_field(&p));
        r->initial_valid = atoi(next_field(&p));
        next_field(&p);
        parse_hex_list(next_field(&p), tmp, 16);
        rowmajor_to_col(tmp, r->initial_pose);
        r->final_valid = atoi(next_field(&p));
        next_field(&p);
        {
            char* hx = next_field(&p);
            if (r->final_valid) {
                parse_hex_list(hx, tmp, 16);
                rowmajor_to_col(tmp, r->final_pose);
            }
        }
        r->num_tracked = (unsigned int)atol(next_field(&p));
        r->num_reliable = (unsigned int)atol(next_field(&p));
    }
    fclose(ft);

    /* ---- local_map.tsv ---- */
    fl = open_or_die(dump_dir, "local_map.tsv");
    if (!fgets(line, LINE_MAX_LEN, fl)) return 2;
    while (fgets(line, LINE_MAX_LEN, fl)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        char* a = next_field(&p);
        char* b = next_field(&p);
        parse_uint_list(a, &lm_kf[fi]);
        parse_uint_list(b, &lm_lm[fi]);
    }
    fclose(fl);

    /* ---- kf_decision.tsv ---- */
    fdec = open_or_die(dump_dir, "kf_decision.tsv");
    if (!fgets(line, LINE_MAX_LEN, fdec)) return 2;
    while (fgets(line, LINE_MAX_LEN, fdec)) {
        char* p = line;
        unsigned long fi = (unsigned long)atol(next_field(&p));
        dec_row* r = &dec[fi];
        int k;
        r->have = 1;
        r->verdict = atoi(next_field(&p));
        r->paused = atoi(next_field(&p));
        r->ref = (unsigned int)atol(next_field(&p));
        r->rel = (unsigned int)atol(next_field(&p));
        r->trk = (unsigned int)atol(next_field(&p));
        r->dist = strtof(next_field(&p), NULL);
        for (k = 0; k < 9; ++k) r->f[k] = atoi(next_field(&p));
        r->skipping = atoi(next_field(&p));
    }
    fclose(fdec);

    /* ---- kf_insert.tsv / kf_insert_lms.tsv (patch 0009) ---- */
    {
        char path[4096];
        FILE *fi_ = NULL, *fil_ = NULL;
        snprintf(path, sizeof(path), "%s/kf_insert.tsv", dump_dir);
        fi_ = fopen(path, "r");
        snprintf(path, sizeof(path), "%s/kf_insert_lms.tsv", dump_dir);
        fil_ = fopen(path, "r");
        if (!fi_ || !fil_) {
            fprintf(stderr, "check_sv_track: kf_insert*.tsv missing in %s (re-run tools/dump_stella_reference.py)\n", dump_dir);
            return 2;
        }
        if (!fgets(line, LINE_MAX_LEN, fi_)) return 2;
        while (fgets(line, LINE_MAX_LEN, fi_)) {
            char* p = line;
            unsigned long fi = (unsigned long)atol(next_field(&p));
            ins_rec* r = &ins[fi];
            double tmp[16];
            r->have = 1;
            r->kf_id = (unsigned int)atol(next_field(&p));
            r->ts = strtod(next_field(&p), NULL);
            parse_hex_list(next_field(&p), tmp, 16);
            rowmajor_to_col(tmp, r->pose_cw);
            parse_hex_list(next_field(&p), tmp, 16);
            rowmajor_to_col(tmp, r->pose_wc);
            parse_hex_list(next_field(&p), r->trans_wc, 3);
        }
        fclose(fi_);
        if (!fgets(line, LINE_MAX_LEN, fil_)) return 2;
        while (fgets(line, LINE_MAX_LEN, fil_)) {
            char* p = line;
            unsigned long fi = (unsigned long)atol(next_field(&p));
            ins_lm* l;
            next_field(&p); /* kf_id */
            if (n_inslms == cap_inslms) {
                cap_inslms = cap_inslms ? cap_inslms * 2 : 4096;
                inslms = (ins_lm*)realloc(inslms, cap_inslms * sizeof(ins_lm));
            }
            l = &inslms[n_inslms];
            l->kp = (unsigned int)atol(next_field(&p));
            l->id = (unsigned int)atol(next_field(&p));
            parse_hex_list(next_field(&p), l->pos, 3);
            parse_desc_hex(next_field(&p), l->desc);
            parse_hex_list(next_field(&p), l->normal, 3);
            l->minv = strtof(next_field(&p), NULL);
            l->maxv = strtof(next_field(&p), NULL);
            l->nobs = (unsigned int)atol(next_field(&p));
            l->nobserved = (unsigned int)atol(next_field(&p));
            l->nobservable = (unsigned int)atol(next_field(&p));
            l->ref = atoi(next_field(&p));
            if (ins[fi].lm_count == 0) ins[fi].lm_start = n_inslms;
            ins[fi].lm_count++;
            ++n_inslms;
        }
        fclose(fil_);
    }

    /* ---- keyframe_meta.tsv ---- */
    fmeta = open_or_die(dump_dir, "keyframe_meta.tsv");
    if (!fgets(line, LINE_MAX_LEN, fmeta)) return 2;
    while (fgets(line, LINE_MAX_LEN, fmeta)) {
        char* p = line;
        unsigned int id;
        double ts;
        unsigned int fi;
        next_field(&p);
        id = (unsigned int)atol(next_field(&p));
        ts = strtod(next_field(&p), NULL);
        if (id >= w.kfmeta_cap) {
            unsigned int nc = id + 64;
            w.kfmeta = (kf_static*)realloc(w.kfmeta, nc * sizeof(kf_static));
            memset(w.kfmeta + w.kfmeta_cap, 0, (nc - w.kfmeta_cap) * sizeof(kf_static));
            w.kfmeta_cap = nc;
        }
        w.kfmeta[id].known = 0;
        for (fi = 0; fi < nframes; ++fi) {
            if (pre[fi].have && memcmp(&pre[fi].timestamp, &ts, sizeof(double)) == 0) {
                w.kfmeta[id].known = 1;
                w.kfmeta[id].src_frame = fi;
                w.kfmeta[id].timestamp = ts;
                break;
            }
        }
    }
    fclose(fmeta);

    /* ---- snapshots ---- */
    fsk = open_or_die(dump_dir, "keyframes.tsv");
    fsl = open_or_die(dump_dir, "landmarks.tsv");
    br_open(&bk, fsk);
    br_open(&bl, fsl);
    ensure_map_capacity(&w, 64, 4096);

    sv_tracker_init(&trk, &w.cfg);
#ifdef SV_MAPPING_HARNESS
    mapping_hook_begin(&w, pre, nframes, dump_dir);
#endif

    {
        /* Two maps' worth of state is not needed: snapshot(t-1) is loaded,
         * tracking runs, then snapshot(t) replaces it for finish_frame. */
        long loaded = -100;
        for (t = 1; t < nframes; ++t) {
            const pre_row* pr = &pre[t];
            frame_data* cur = &w.frames[t];
            frame_data* lst = &w.frames[t - 1];
            sv_tr_frame input;
            unsigned int cnt;
            unsigned int i;
            unsigned long long hsh;
            int track_ok, verdict = 0, inserted_id = -1;
            const trace_row* tr = &trace[t];
            const dec_row* dr = &dec[t];
            sv_tr_frame last;

            if (max_frames >= 0 && processed >= max_frames) break;
            if (!pr->have || pr->tracking_state != 1) continue;
#ifdef SV_MAPPING_HARNESS
            if (!ins[t].have) continue; /* mapping runs only for inserted keyframes */
#endif
            g_cur_frame = t;
            ++processed;
            if (strcmp(tr->path, "robust_match") == 0) ++robust_frames;

            /* map at the start of frame t == snapshot after frame t-1 */
            if (loaded != (long)t - 1) {
                if (load_snapshot(&w, &bk, &bl, (long)t - 1) != 0) {
                    fprintf(stderr, "check_sv_track: no snapshot for frame %u\n", t - 1);
                    return 2;
                }
                loaded = (long)t - 1;
            }
            w.map.num_keyframes = w.map.num_keyframes; /* from the snapshot */
            item(w.map.num_keyframes == pr->num_keyframes, "map num_keyframes vs track_pre");
            w.map.num_keyframes = pr->num_keyframes;
            w.map.last_inserted_kf = (int)pr->last_ins_id;
            w.map.last_inserted_timestamp = pr->last_ins_ts;
            w.map.last_inserted_trans_wc[0] = pr->last_ins_trans_wc[0];
            w.map.last_inserted_trans_wc[1] = pr->last_ins_trans_wc[1];
            w.map.last_inserted_trans_wc[2] = pr->last_ins_trans_wc[2];
            w.map.fixed_keyframe_id_threshold = pr->fixed_thr;

            /* tracker state (teacher forced from the reference's own state) */
            sv_tr_frame_init(&last, (unsigned int)pr->last_frm_id, w.frames[t - 1].timestamp, frame_obs(&w, t - 1));
            for (i = 0; i < lst->n; ++i) last.lm[i] = lst->match[i];
            hsh = lm_hash(last.lm, lst->n, &cnt);
            item(hsh == pr->last_frm_hash && cnt == pr->last_frm_num_lms && pr->last_frm_id == (long long)t - 1,
                 "last frame landmark hash / id vs track_pre");
            if (pr->last_frm_pose_valid) sv_tr_frame_set_pose_cw(&last, pr->last_frm_pose);
            last.ref_kf = (int)pr->last_frm_ref_kf;
            if (trk.last_frm_valid) sv_tr_frame_free(&trk.last_frm);
            trk.last_frm = last;
            trk.last_frm_valid = 1;
            trk.tracking_state = pr->tracking_state;
            trk.twist_valid = pr->twist_valid;
            memcpy(trk.twist, pr->twist, sizeof(trk.twist));
            memcpy(trk.last_cam_pose_from_ref_keyfrm, pr->last_cam_pose_from_ref, sizeof(trk.last_cam_pose_from_ref_keyfrm));
            trk.last_reloc_frm_id = pr->last_reloc_frm_id;
            trk.last_reloc_frm_timestamp = pr->last_reloc_ts;

#ifdef SV_TRACK_FORCE_PERIOD
            trk.force_skip_motion = (t % SV_TRACK_FORCE_PERIOD) == 0;
            trk.force_skip_bow = SV_TRACK_FORCE_SKIP_BOW && (t % SV_TRACK_FORCE_PERIOD) == 0;
#endif
            sv_tr_frame_init(&input, t, cur->timestamp, frame_obs(&w, t));
            track_ok = sv_tracker_track(&trk, &w.map, &input);
            sv_tr_frame_free(&input);

            /* --- compare the tracking outputs --- */
            {
                const char* pname = trk.path == SV_TR_PATH_MOTION ? "motion_model"
                                    : trk.path == SV_TR_PATH_BOW ? "bow_match"
                                    : trk.path == SV_TR_PATH_ROBUST ? "robust_match" : "none";
                item(strcmp(pname, tr->path) == 0, "track path");
                item(trk.initial_pose_valid == tr->initial_valid, "initial pose valid flag");
                if (trk.initial_pose_valid && tr->initial_valid)
                    item(same_pose(trk.initial_pose, tr->initial_pose), "pose after track_current_frame");
                item((track_ok != 0) == (tr->final_valid != 0), "tracking success vs final pose valid");
                if (track_ok && tr->final_valid) {
                    /* feed_frame() returns curr_frm_.get_pose_wc() = [rot_wc | trans_wc] */
                    double wc[16];
                    int r_, c_;
                    for (c_ = 0; c_ < 16; ++c_) wc[c_] = (c_ % 5 == 0) ? 1.0 : 0.0;
                    for (c_ = 0; c_ < 3; ++c_)
                        for (r_ = 0; r_ < 3; ++r_) wc[c_ * 4 + r_] = trk.curr_frm.rot_wc[c_ * 3 + r_];
                    for (r_ = 0; r_ < 3; ++r_) wc[3 * 4 + r_] = trk.curr_frm.trans_wc[r_];
                    item(same_pose(wc, tr->final_pose), "final pose (returned pose_wc)");
                }
                item(trk.num_tracked_lms == tr->num_tracked, "num_tracked_lms");
                item(trk.num_reliable_lms == tr->num_reliable, "num_reliable_lms");
            }
            for (i = 0; i < cur->n; ++i) {
                item(trk.curr_frm.lm[i] == cur->match[i], "keypoint landmark association");
            }
            {
                int same = (trk.local.n_kfs == lm_kf[t].n);
                for (i = 0; same && i < trk.local.n_kfs; ++i) same = (trk.local.kfs[i] == lm_kf[t].v[i]);
                item(same, "local keyframe list");
                same = (trk.local.n_lms == lm_lm[t].n);
                for (i = 0; same && i < trk.local.n_lms; ++i) same = (trk.local.lms[i] == lm_lm[t].v[i]);
                item(same, "local landmark list");
            }
            if (track_ok && trk.decision_evaluated) {
                const sv_tr_kf_decision* d = &trk.decision;
                item(d->verdict == dr->verdict, "kf decision verdict");
                item(d->mapper_paused_or_pausing == dr->paused, "kf decision mapper_paused");
                item(d->num_reliable_lms_ref == dr->ref, "kf decision num_reliable_lms_ref");
                item(d->num_reliable_lms == dr->rel, "kf decision num_reliable_lms");
                item(d->num_tracked_lms == dr->trk, "kf decision num_tracked_lms");
                item(memcmp(&d->distance_traveled, &dr->dist, sizeof(float)) == 0, "kf decision distance_traveled");
                item(d->max_interval_elapsed == dr->f[0], "kf max_interval_elapsed");
                item(d->min_interval_elapsed == dr->f[1], "kf min_interval_elapsed");
                item(d->max_distance_traveled == dr->f[2], "kf max_distance_traveled");
                item(d->min_distance_traveled == dr->f[3], "kf min_distance_traveled");
                item(d->view_changed == dr->f[4], "kf view_changed");
                item(d->not_enough_lms == dr->f[5], "kf not_enough_lms");
                item(d->enough_keyfrms == dr->f[6], "kf enough_keyfrms");
                item(d->tracking_is_unstable == dr->f[7], "kf tracking_is_unstable");
                item(d->almost_all_lms_are_tracked == dr->f[8], "kf almost_all_lms_are_tracked");
                item(d->mapper_is_skipping_localBA == dr->skipping, "kf mapper_is_skipping_localBA");
                verdict = d->verdict;
            }

            /* --- keyframe_inserter::create_new_keyframe() against kf_insert*.tsv --- */
            item((verdict != 0) == (ins[t].have != 0), "keyframe insertion happened iff verdict");
            if (verdict) inserted_id = (int)pr->last_ins_id + 1;
            if (verdict && ins[t].have) {
                const ins_rec* ir = &ins[t];
                sv_tr_kf* nk = kf_record(&w, (unsigned int)inserted_id);
                unsigned int j = 0, kp;
                int rc;
                free(nk->lm); free(nk->covis); free(nk->covis_w); free(nk->children);
                memset(nk, 0, sizeof(*nk));
                rc = sv_tr_create_new_keyframe(&w.cfg, &w.map, &trk.curr_frm, (unsigned int)inserted_id, cur->timestamp, nk);
                item(rc == 0, "create_new_keyframe succeeded");
                item((unsigned int)inserted_id == ir->kf_id, "new keyframe id");
                item(same_pose(nk->pose_cw, ir->pose_cw), "new keyframe pose_cw");
                item(same_pose(nk->pose_wc, ir->pose_wc), "new keyframe pose_wc");
                item(memcmp(nk->trans_wc, ir->trans_wc, 3 * sizeof(double)) == 0, "new keyframe trans_wc");
                for (kp = 0; kp < nk->obs->num_kp; ++kp) {
                    const sv_tr_lm* lm;
                    const ins_lm* il;
                    int ok;
                    if (nk->lm[kp] < 0 || !(lm = sv_tr_map_lm(&w.map, nk->lm[kp]))) continue;
                    if (j >= ir->lm_count) {
                        item(0, "create_new_keyframe: more landmarks than the reference");
                        continue;
                    }
                    il = &inslms[ir->lm_start + j];
                    ++j;
                    ok = il->kp == kp && il->id == lm->id &&
                         memcmp(il->pos, lm->pos_w, sizeof(il->pos)) == 0 &&
                         memcmp(il->desc, lm->desc, 32) == 0 &&
                         memcmp(il->normal, lm->mean_normal, sizeof(il->normal)) == 0 &&
                         memcmp(&il->minv, &lm->min_valid_dist, sizeof(float)) == 0 &&
                         memcmp(&il->maxv, &lm->max_valid_dist, sizeof(float)) == 0 &&
                         il->nobs == lm->num_obs && il->nobserved == lm->num_observed &&
                         il->nobservable == lm->num_observable && il->ref == lm->ref_kf;
                    item(ok, "landmark state after update_landmarks()");
                }
                item(j == ir->lm_count, "create_new_keyframe landmark count");
            }

#ifdef SV_MAPPING_HARNESS
            if (verdict && ins[t].have) mapping_hook_after_insert(&w, t, inserted_id);
#endif
            /* --- second half of feed_frame() against snapshot(t) --- */
            if (load_snapshot(&w, &bk, &bl, (long)t) != 0) {
                fprintf(stderr, "check_sv_track: no snapshot for frame %u\n", t);
                return 2;
            }
            loaded = (long)t;
            sv_tracker_finish_frame(&trk, &w.map, inserted_id);
            item((long)trk.curr_frm.ref_kf == tr->ref_kf, "reference keyframe id after insertion");
            if (t + 1 < nframes && pre[t + 1].have) {
                const pre_row* nx = &pre[t + 1];
                item(trk.tracking_state == nx->tracking_state, "next tracking_state");
                item(trk.twist_valid == nx->twist_valid, "next twist_valid");
                if (trk.twist_valid && nx->twist_valid) item(same_pose(trk.twist, nx->twist), "velocity (twist)");
                if (trk.curr_frm.pose_valid) {
                    item(same_pose(trk.last_cam_pose_from_ref_keyfrm, nx->last_cam_pose_from_ref), "last_cam_pose_from_ref_keyfrm");
                    item(same_pose(trk.curr_frm.pose_cw, nx->last_frm_pose), "last frame pose");
                }
                item((long long)trk.last_frm.ref_kf == nx->last_frm_ref_kf, "last frame ref keyframe");
            }
        }
    }

#ifdef SV_MAPPING_HARNESS
    mapping_hook_end();
#endif
    printf("%s: %lu/%lu\n", seq_label, g_mismatch, g_total);
    fprintf(stderr, "%s: frames processed=%ld, robust fallback frames checked=%ld\n",
            seq_label, processed, robust_frames);
    sv_tracker_free(&trk);
    (void)fsk; (void)fsl;
    return g_mismatch == 0 ? 0 : 1;
}
