/* SV_PORT_SOURCES: check_sv_map_init.c sv_map.c sv_landmark_descriptor.c sv_linalg.c
 * SPDX-License-Identifier: MIT
 *
 * Module-4a harness: builds the initial map (data::keyframe/landmark/
 * graph_node subset + module::initializer::create_map_for_monocular's
 * pre-BA block, then everything AFTER a given BA result -- see sv_map.h)
 * from module 1's already-validated keypoints/descriptors
 * (runs/stella_port/reference_dumps/<seq>/{keypoints,descriptors}.tsv)
 * and module 3's already-exact initializer output for the sequence's one
 * successful init attempt, both taken as exact input here (NOT
 * recomputed): runs/stella_port/reference_map_init/<seq>/{init_state,
 * matches}.tsv (stella_port/reference_tools/dump_stella_map_init.cc).
 *
 * Two comparisons per sequence:
 *   PRE-BA:  sv_map_build_pre_ba() output vs keyframes_pre.tsv/
 *            landmarks_pre.tsv, bit-exact.
 *   POST-BA: curr_keyfrm pose + every landmark position INJECTED from
 *            keyframes_postba.tsv/landmarks_postba_pos.tsv (the real
 *            global_bundle_adjuster's genuine output, dumped by the
 *            reference tool -- the real g2o BA is a separate,
 *            concurrently-developed module, not ported here), then
 *            sv_map_apply_post_ba() output vs keyframes_post.tsv/
 *            landmarks_post.tsv/scale.tsv, bit-exact.
 *
 * Usage: check_sv_map_init <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (dump_dir == runs/stella_port/reference_dumps/<seq>, reused here for
 * keypoints.tsv/descriptors.tsv; the module-4a reference_map_init dir is
 * derived from it by substring substitution, same technique
 * check_sv_frame.c/check_sv_init.c use.)
 * Prints "<seq_label>: <mismatches>/<total>\n"; exits 0 iff mismatches==0.
 */
#define _POSIX_C_SOURCE 200809L
#include "sv_map.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static long g_mismatches = 0, g_total = 0;

static void check_eq_i(long a, long b, const char* what) {
    g_total++;
    if (a != b) {
        g_mismatches++;
        if (g_mismatches < 200) fprintf(stderr, "MISMATCH %s: got %ld want %ld\n", what, a, b);
    }
}
static void check_eq_hexd(double a, double b, const char* what) {
    g_total++;
    if (a != b) {
        g_mismatches++;
        if (g_mismatches < 200) fprintf(stderr, "MISMATCH %s: got %a want %a\n", what, a, b);
    }
}
static void check_eq_hexf(float a, float b, const char* what) {
    g_total++;
    if (a != b) {
        g_mismatches++;
        if (g_mismatches < 200) fprintf(stderr, "MISMATCH %s: got %a want %a (float)\n", what, (double)a, (double)b);
    }
}
static void check_eq_bytes(const uint8_t* a, const uint8_t* b, size_t n, const char* what) {
    g_total++;
    if (memcmp(a, b, n) != 0) {
        g_mismatches++;
        if (g_mismatches < 200) fprintf(stderr, "MISMATCH %s (descriptor bytes)\n", what);
    }
}

static char* derive_dir(const char* dump_dir, const char* repl) {
    const char* needle = "reference_dumps";
    const char* p = strstr(dump_dir, needle);
    size_t prefix_len, needle_len, out_len;
    char* out;
    if (!p) return NULL;
    prefix_len = (size_t)(p - dump_dir);
    needle_len = strlen(needle);
    out_len = prefix_len + strlen(repl) + strlen(dump_dir + prefix_len + needle_len);
    out = (char*)malloc(out_len + 1);
    memcpy(out, dump_dir, prefix_len);
    memcpy(out + prefix_len, repl, strlen(repl));
    strcpy(out + prefix_len + strlen(repl), dump_dir + prefix_len + needle_len);
    return out;
}

/* ---- module-1 keypoints/descriptors, grouped per frame (same loader as
 * check_sv_init.c) ---- */
typedef struct {
    sv_keypoint* kp;
    uint8_t* desc;
    unsigned int n;
} frame_data;

static frame_data* g_frames = NULL;
static int g_num_frames = 0;

static void load_frames(const char* dump_dir) {
    char path[4096];
    FILE* f;
    char line[1024];
    int max_fi = -1;

    snprintf(path, sizeof(path), "%s/keypoints.tsv", dump_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f)) {
        int fi;
        if (sscanf(line, "%d", &fi) == 1 && fi > max_fi) max_fi = fi;
    }
    fclose(f);

    g_num_frames = max_fi + 1;
    g_frames = (frame_data*)calloc((size_t)g_num_frames, sizeof(frame_data));

    f = fopen(path, "r");
    fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f)) {
        int fi, ki;
        if (sscanf(line, "%d\t%d", &fi, &ki) == 2 && fi < g_num_frames) {
            if ((unsigned int)(ki + 1) > g_frames[fi].n) g_frames[fi].n = (unsigned int)(ki + 1);
        }
    }
    fclose(f);
    {
        int i;
        for (i = 0; i < g_num_frames; ++i) {
            g_frames[i].kp = (sv_keypoint*)calloc(g_frames[i].n ? g_frames[i].n : 1, sizeof(sv_keypoint));
            g_frames[i].desc = (uint8_t*)calloc((g_frames[i].n ? g_frames[i].n : 1) * 32, 1);
        }
    }

    f = fopen(path, "r");
    fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f)) {
        int fi, ki, octave;
        double dummy;
        char x_hex[64], y_hex[64], angle_hex[64], response_hex[64];
        int n = sscanf(line, "%d\t%d\t%lf\t%63s\t%lf\t%63s\t%d\t%lf\t%63s\t%lf\t%63s",
                       &fi, &ki, &dummy, x_hex, &dummy, y_hex, &octave, &dummy, angle_hex, &dummy, response_hex);
        if (n != 11 || fi >= g_num_frames) continue;
        g_frames[fi].kp[ki].x = strtof(x_hex, NULL);
        g_frames[fi].kp[ki].y = strtof(y_hex, NULL);
        g_frames[fi].kp[ki].octave = octave;
        g_frames[fi].kp[ki].angle = strtof(angle_hex, NULL);
    }
    fclose(f);

    snprintf(path, sizeof(path), "%s/descriptors.tsv", dump_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    fgets(line, sizeof(line), f);
    while (fgets(line, sizeof(line), f)) {
        int fi, ki;
        char hex[128];
        int n = sscanf(line, "%d\t%d\t%127s", &fi, &ki, hex);
        if (n != 3 || fi >= g_num_frames) continue;
        if ((unsigned int)ki >= g_frames[fi].n) continue;
        {
            int b;
            for (b = 0; b < 32; ++b) {
                unsigned int byte;
                sscanf(hex + b * 2, "%2x", &byte);
                g_frames[fi].desc[(size_t)ki * 32 + b] = (uint8_t)byte;
            }
        }
    }
    fclose(f);
}

/* ---- reference_map_init tsvs -------------------------------------------- */
static void parse_hexn(const char* s, double* out, int n) {
    const char* p = s;
    int i;
    for (i = 0; i < n; ++i) {
        out[i] = strtod(p, (char**)&p);
        if (*p == ',') p++;
    }
}

typedef struct {
    unsigned int ref_frame_id, cur_frame_id, num_kp_ref, num_kp_cur;
    double rot9[9], trans3[3];
} init_state_row;

static int load_init_state(const char* dir, init_state_row* out) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/init_state.tsv", dir);
    f = fopen(path, "r");
    if (!f) return -1;
    getline(&line, &cap, f); /* header */
    if (getline(&line, &cap, f) == -1) { fclose(f); free(line); return -1; }
    {
        char rot_s[1024], trans_s[256];
        int nf = sscanf(line, "%u\t%u\t%u\t%u\t%1023[^\t]\t%255s",
                        &out->ref_frame_id, &out->cur_frame_id, &out->num_kp_ref, &out->num_kp_cur,
                        rot_s, trans_s);
        if (nf != 6) { fclose(f); free(line); return -1; }
        parse_hexn(rot_s, out->rot9, 9);
        parse_hexn(trans_s, out->trans3, 3);
    }
    free(line);
    fclose(f);
    return 0;
}

/* matches.tsv -> init_matches[num_kp_ref] (-1 default), is_triangulated,
 * triangulated_pts (num_kp_ref*3, 0 for non-triangulated rows). */
static void load_matches(const char* dir, unsigned int num_kp_ref,
                         int* init_matches, unsigned char* is_tri, double* tri_pts) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    unsigned int i;
    for (i = 0; i < num_kp_ref; ++i) { init_matches[i] = -1; is_tri[i] = 0; }
    memset(tri_pts, 0, sizeof(double) * 3 * num_kp_ref);

    snprintf(path, sizeof(path), "%s/matches.tsv", dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    while (getline(&line, &cap, f) != -1) {
        unsigned int ref_idx; int cur_idx, tri;
        char xh[64], yh[64], zh[64];
        int nf = sscanf(line, "%u\t%d\t%d\t%63s\t%63s\t%63s", &ref_idx, &cur_idx, &tri, xh, yh, zh);
        if (nf != 6 || ref_idx >= num_kp_ref) continue;
        init_matches[ref_idx] = cur_idx;
        is_tri[ref_idx] = (unsigned char)tri;
        if (tri) {
            tri_pts[(size_t)ref_idx * 3 + 0] = strtod(xh, NULL);
            tri_pts[(size_t)ref_idx * 3 + 1] = strtod(yh, NULL);
            tri_pts[(size_t)ref_idx * 3 + 2] = strtod(zh, NULL);
        }
    }
    free(line);
    fclose(f);
}

/* keyframes_{pre,post}.tsv row (kf_id 0 or 1). */
typedef struct {
    double pose_hex[16];
    int spanning_parent, spanning_root;
    unsigned int children[8], num_children;
} kf_row;

static void load_keyframes(const char* dir, const char* fname, kf_row rows[2]) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/%s", dir, fname);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    while (getline(&line, &cap, f) != -1) {
        unsigned int id;
        char pose_s[2048], childs_s[256];
        int parent, root;
        int nf = sscanf(line, "%u\t%2047[^\t]\t%d\t%d\t%255s", &id, pose_s, &parent, &root, childs_s);
        if (nf < 4 || id > 1) continue;
        parse_hexn(pose_s, rows[id].pose_hex, 16);
        rows[id].spanning_parent = parent;
        rows[id].spanning_root = root;
        rows[id].num_children = 0;
        if (nf == 5) {
            char* p = childs_s;
            while (*p) {
                rows[id].children[rows[id].num_children++] = (unsigned int)strtoul(p, &p, 10);
                if (*p == ',') p++;
                else break;
            }
        }
    }
    free(line);
    fclose(f);
}

typedef struct {
    unsigned int lm_id;
    double pos_hex[3];
    uint8_t descriptor[32];
    double mean_normal_hex[3];
    float min_valid_dist_hex, max_valid_dist_hex;
    unsigned int num_observed, num_observable, ref_keyfrm_id;
    sv_map_observation obs[8];
    unsigned int num_obs;
} lm_row;

static lm_row* g_lm_rows = NULL;
static unsigned int g_num_lm_rows = 0;

static void load_landmarks(const char* dir, const char* fname) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    unsigned int cnt = 0;
    snprintf(path, sizeof(path), "%s/%s", dir, fname);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    {
        long pos = ftell(f);
        while (getline(&line, &cap, f) != -1) cnt++;
        fseek(f, pos, SEEK_SET);
    }
    g_lm_rows = (lm_row*)calloc(cnt, sizeof(lm_row));
    g_num_lm_rows = 0;
    while (getline(&line, &cap, f) != -1) {
        lm_row* r = &g_lm_rows[g_num_lm_rows];
        char pos_s[256], desc_s[128], mn_s[256], minv_s[64], maxv_s[64], obs_s[256];
        int nf = sscanf(line, "%u\t%255[^\t]\t%127[^\t]\t%255[^\t]\t%63[^\t]\t%63[^\t]\t%u\t%u\t%u\t%255s",
                        &r->lm_id, pos_s, desc_s, mn_s, minv_s, maxv_s,
                        &r->num_observed, &r->num_observable, &r->ref_keyfrm_id, obs_s);
        if (nf != 10) continue;
        parse_hexn(pos_s, r->pos_hex, 3);
        parse_hexn(mn_s, r->mean_normal_hex, 3);
        r->min_valid_dist_hex = strtof(minv_s, NULL);
        r->max_valid_dist_hex = strtof(maxv_s, NULL);
        {
            int b;
            for (b = 0; b < 32; ++b) {
                unsigned int byte;
                sscanf(desc_s + b * 2, "%2x", &byte);
                r->descriptor[b] = (uint8_t)byte;
            }
        }
        r->num_obs = 0;
        {
            char* p = obs_s;
            while (*p) {
                unsigned int kf = (unsigned int)strtoul(p, &p, 10);
                unsigned int idx;
                if (*p == ':') p++;
                idx = (unsigned int)strtoul(p, &p, 10);
                r->obs[r->num_obs].keyframe_id = kf;
                r->obs[r->num_obs].idx = idx;
                r->num_obs++;
                if (*p == ',') p++;
                else break;
            }
        }
        g_num_lm_rows++;
    }
    free(line);
    fclose(f);
}

/* keyframes_postba.tsv / landmarks_postba_pos.tsv (injected BA fixture). */
static void load_postba_poses(const char* dir, double pose1[16]) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/keyframes_postba.tsv", dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    while (getline(&line, &cap, f) != -1) {
        unsigned int id;
        char pose_s[2048];
        if (sscanf(line, "%u\t%2047s", &id, pose_s) == 2 && id == 1) {
            parse_hexn(pose_s, pose1, 16);
        }
    }
    free(line);
    fclose(f);
}

typedef struct { unsigned int lm_id; double pos[3]; } postba_pos_row;
static postba_pos_row* g_postba_pos = NULL;
static unsigned int g_num_postba_pos = 0;

static void load_postba_landmark_pos(const char* dir) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    unsigned int cnt = 0;
    snprintf(path, sizeof(path), "%s/landmarks_postba_pos.tsv", dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    {
        long pos = ftell(f);
        while (getline(&line, &cap, f) != -1) cnt++;
        fseek(f, pos, SEEK_SET);
    }
    g_postba_pos = (postba_pos_row*)calloc(cnt, sizeof(postba_pos_row));
    g_num_postba_pos = 0;
    while (getline(&line, &cap, f) != -1) {
        postba_pos_row* r = &g_postba_pos[g_num_postba_pos];
        char pos_s[256];
        if (sscanf(line, "%u\t%255s", &r->lm_id, pos_s) == 2) {
            parse_hexn(pos_s, r->pos, 3);
            g_num_postba_pos++;
        }
    }
    free(line);
    fclose(f);
}

static void load_scale(const char* dir, double* median_scale, double* inv_median_scale,
                       double* applied_scale, char* verdict, size_t verdict_cap) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/scale.tsv", dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    if (getline(&line, &cap, f) != -1) {
        char ms[64], ims[64], as[64], v[64];
        if (sscanf(line, "%63s\t%63s\t%63s\t%63s", ms, ims, as, v) == 4) {
            *median_scale = strtod(ms, NULL);
            *inv_median_scale = strtod(ims, NULL);
            *applied_scale = strtod(as, NULL);
            strncpy(verdict, v, verdict_cap - 1);
            verdict[verdict_cap - 1] = 0;
        }
    }
    free(line);
    fclose(f);
}

/* ---- comparisons ---------------------------------------------------- */

static void check_keyframe(const sv_map_keyframe* kf, const kf_row* want, const char* tag) {
    int i;
    char what[128];
    for (i = 0; i < 16; ++i) {
        snprintf(what, sizeof(what), "%s.pose_cw[%d]", tag, i);
        check_eq_hexd(kf->pose_cw[i], want->pose_hex[i], what);
    }
    snprintf(what, sizeof(what), "%s.spanning_parent", tag);
    check_eq_i(kf->spanning_parent_id, want->spanning_parent, what);
    snprintf(what, sizeof(what), "%s.spanning_root", tag);
    check_eq_i(kf->spanning_root_id, want->spanning_root, what);
    snprintf(what, sizeof(what), "%s.num_spanning_children", tag);
    check_eq_i((long)kf->num_spanning_children, (long)want->num_children, what);
    for (i = 0; i < (int)kf->num_spanning_children && i < (int)want->num_children; ++i) {
        snprintf(what, sizeof(what), "%s.spanning_children[%d]", tag, i);
        check_eq_i((long)kf->spanning_children[i], (long)want->children[i], what);
    }
}

static const lm_row* find_lm_row(unsigned int id) {
    unsigned int i;
    for (i = 0; i < g_num_lm_rows; ++i) {
        if (g_lm_rows[i].lm_id == id) return &g_lm_rows[i];
    }
    return NULL;
}

static void check_landmarks(const sv_map_landmark* lms, unsigned int n, const char* tag) {
    unsigned int i, j;
    char what[160];
    check_eq_i((long)n, (long)g_num_lm_rows, "num_landmarks");
    for (i = 0; i < n; ++i) {
        const sv_map_landmark* lm = &lms[i];
        const lm_row* want = find_lm_row(lm->id);
        if (!want) {
            g_mismatches++;
            g_total++;
            if (g_mismatches < 200) fprintf(stderr, "MISMATCH %s: lm id %u not found in reference\n", tag, lm->id);
            continue;
        }
        for (j = 0; j < 3; ++j) {
            snprintf(what, sizeof(what), "%s.lm[%u].pos_w[%u]", tag, lm->id, j);
            check_eq_hexd(lm->pos_w[j], want->pos_hex[j], what);
        }
        snprintf(what, sizeof(what), "%s.lm[%u].descriptor", tag, lm->id);
        check_eq_bytes(lm->descriptor, want->descriptor, 32, what);
        for (j = 0; j < 3; ++j) {
            snprintf(what, sizeof(what), "%s.lm[%u].mean_normal[%u]", tag, lm->id, j);
            check_eq_hexd(lm->mean_normal[j], want->mean_normal_hex[j], what);
        }
        snprintf(what, sizeof(what), "%s.lm[%u].min_valid_dist", tag, lm->id);
        check_eq_hexf(lm->min_valid_dist, want->min_valid_dist_hex, what);
        snprintf(what, sizeof(what), "%s.lm[%u].max_valid_dist", tag, lm->id);
        check_eq_hexf(lm->max_valid_dist, want->max_valid_dist_hex, what);
        snprintf(what, sizeof(what), "%s.lm[%u].num_observed", tag, lm->id);
        check_eq_i((long)lm->num_observed, (long)want->num_observed, what);
        snprintf(what, sizeof(what), "%s.lm[%u].num_observable", tag, lm->id);
        check_eq_i((long)lm->num_observable, (long)want->num_observable, what);
        snprintf(what, sizeof(what), "%s.lm[%u].ref_keyfrm_id", tag, lm->id);
        check_eq_i((long)lm->ref_keyfrm_id, (long)want->ref_keyfrm_id, what);
        snprintf(what, sizeof(what), "%s.lm[%u].num_observations", tag, lm->id);
        check_eq_i((long)lm->num_observations, (long)want->num_obs, what);
        for (j = 0; j < lm->num_observations && j < want->num_obs; ++j) {
            snprintf(what, sizeof(what), "%s.lm[%u].obs[%u].kf", tag, lm->id, j);
            check_eq_i((long)lm->observations[j].keyframe_id, (long)want->obs[j].keyframe_id, what);
            snprintf(what, sizeof(what), "%s.lm[%u].obs[%u].idx", tag, lm->id, j);
            check_eq_i((long)lm->observations[j].idx, (long)want->obs[j].idx, what);
        }
    }
}

static double find_postba_pos(unsigned int lm_id, int component) {
    unsigned int i;
    for (i = 0; i < g_num_postba_pos; ++i) {
        if (g_postba_pos[i].lm_id == lm_id) return g_postba_pos[i].pos[component];
    }
    return 0.0;
}

static void run_seq(const char* seq_label, const char* dump_dir) {
    char* init_dir = derive_dir(dump_dir, "reference_map_init");
    init_state_row st;
    sv_map_orb_params orb_params;
    sv_map_init_map map;
    sv_map_landmark* landmarks;
    int* init_matches;
    unsigned char* is_tri;
    double* tri_pts;
    kf_row pre_rows[2], post_rows[2];
    double injected_pose1[16];
    double median_scale, inv_median_scale, applied_scale;
    char verdict[64];
    unsigned int i;

    if (!init_dir) { fprintf(stderr, "%s: could not derive reference_map_init dir\n", seq_label); exit(2); }
    if (load_init_state(init_dir, &st) != 0) {
        fprintf(stderr, "%s: no init_state.tsv in %s\n", seq_label, init_dir);
        exit(2);
    }

    load_frames(dump_dir);
    if ((int)st.ref_frame_id >= g_num_frames || (int)st.cur_frame_id >= g_num_frames) {
        fprintf(stderr, "%s: frame ids out of range\n", seq_label);
        exit(2);
    }

    sv_map_orb_params_init(&orb_params, 1.2f, 8);

    init_matches = (int*)malloc(sizeof(int) * st.num_kp_ref);
    is_tri = (unsigned char*)malloc(st.num_kp_ref);
    tri_pts = (double*)malloc(sizeof(double) * 3 * st.num_kp_ref);
    load_matches(init_dir, st.num_kp_ref, init_matches, is_tri, tri_pts);

    landmarks = (sv_map_landmark*)malloc(sizeof(sv_map_landmark) * st.num_kp_ref);

    sv_map_build_pre_ba(st.ref_frame_id, st.cur_frame_id,
                        g_frames[st.ref_frame_id].kp, g_frames[st.ref_frame_id].desc, st.num_kp_ref,
                        g_frames[st.cur_frame_id].kp, g_frames[st.cur_frame_id].desc, st.num_kp_cur,
                        &orb_params, st.rot9, st.trans3, init_matches, is_tri, tri_pts,
                        landmarks, &map);

    load_keyframes(init_dir, "keyframes_pre.tsv", pre_rows);
    load_landmarks(init_dir, "landmarks_pre.tsv");
    check_keyframe(&map.init_keyfrm, &pre_rows[0], "pre.init_keyfrm");
    check_keyframe(&map.curr_keyfrm, &pre_rows[1], "pre.curr_keyfrm");
    check_landmarks(map.landmarks, map.num_landmarks, "pre");
    free(g_lm_rows); g_lm_rows = NULL; g_num_lm_rows = 0;

    /* ---- inject real BA output, then finish the map like create_map_for_monocular does after its BA call ---- */
    load_postba_poses(init_dir, injected_pose1);
    load_postba_landmark_pos(init_dir);
    sv_map_keyframe_set_pose_cw(&map.curr_keyfrm, injected_pose1);
    for (i = 0; i < map.num_landmarks; ++i) {
        map.landmarks[i].pos_w[0] = find_postba_pos(map.landmarks[i].id, 0);
        map.landmarks[i].pos_w[1] = find_postba_pos(map.landmarks[i].id, 1);
        map.landmarks[i].pos_w[2] = find_postba_pos(map.landmarks[i].id, 2);
    }

    sv_map_apply_post_ba(&map, 50, 1.0);

    load_keyframes(init_dir, "keyframes_post.tsv", post_rows);
    load_landmarks(init_dir, "landmarks_post.tsv");
    check_keyframe(&map.init_keyfrm, &post_rows[0], "post.init_keyfrm");
    check_keyframe(&map.curr_keyfrm, &post_rows[1], "post.curr_keyfrm");
    check_landmarks(map.landmarks, map.num_landmarks, "post");

    load_scale(init_dir, &median_scale, &inv_median_scale, &applied_scale, verdict, sizeof(verdict));
    check_eq_hexf(map.median_scale, (float)median_scale, "scale.median_scale");
    check_eq_hexd(map.inv_median_scale, inv_median_scale, "scale.inv_median_scale");
    check_eq_hexd(map.applied_scale, applied_scale, "scale.applied_scale");
    check_eq_i(map.reset_wrong_init, strcmp(verdict, "wrong_init") == 0, "scale.verdict");

    free(init_matches);
    free(is_tri);
    free(tri_pts);
    free(landmarks);
    free(g_lm_rows); g_lm_rows = NULL; g_num_lm_rows = 0;
    free(g_postba_pos); g_postba_pos = NULL; g_num_postba_pos = 0;
    {
        int i2;
        for (i2 = 0; i2 < g_num_frames; ++i2) { free(g_frames[i2].kp); free(g_frames[i2].desc); }
        free(g_frames); g_frames = NULL; g_num_frames = 0;
    }
    free(init_dir);
}

int main(int argc, char** argv) {
    const char* seq_label;
    const char* dump_dir;
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_map_init <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 2;
    }
    seq_label = argv[1];
    dump_dir = argv[3];
    run_seq(seq_label, dump_dir);
    printf("%s: %ld/%ld\n", seq_label, g_mismatches, g_total);
    return g_mismatches == 0 ? 0 : 1;
}
