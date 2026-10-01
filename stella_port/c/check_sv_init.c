/* SV_PORT_SOURCES: check_sv_init.c sv_init.c sv_match_area.c sv_solve_homography.c sv_solve_fundamental.c sv_solve_essential.c sv_solve_common.c sv_triangulate.c sv_linalg.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c sv_frame.c sv_undistort.c
 * SPDX-License-Identifier: MIT
 *
 * Module-3 replay harness: replays module::initializer's monocular state
 * machine (module 3's own reimplementation of it -- see sv_init.h) over
 * already-validated module-1 keypoints/descriptors
 * (runs/stella_port/reference_dumps/<seq>/{keypoints,descriptors}.tsv,
 * undistorted, module 1's own harness already checks these bit-exact
 * against the reference), and compares every attempt's num_matches,
 * H/F cost + solution_valid + H21/F21, inlier masks, per-hypothesis
 * (num_valid_pts, num_triangulated_pts, parallax_cos, rot, trans), and
 * final (selected_hyp, rot, trans) against
 * runs/stella_port/reference_init/<seq>/{attempts,matches,inliers,hyps,final}.tsv
 * (stella_port/reference_tools/dump_stella_init.cc). rng.tsv is not
 * re-checked here (sv_rng.c already validated standalone against real
 * libstdc++, see check_sv_rng.c) -- RANSAC index draws are internal to
 * sv_solve_{homography,fundamental}_find_via_ransac and not separately
 * exposed; H21/F21/cost/inliers matching bit-exact is the real end-to-end
 * proof the draws (and all downstream math) are correct.
 *
 * Usage: check_sv_init <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (dump_dir == runs/stella_port/reference_dumps/<seq>, module 1/2's own
 * dump dir -- reused here for keypoints.tsv/descriptors.tsv; the module-3
 * reference_init dir is derived from it by substring substitution, same
 * technique check_sv_frame.c uses for reference_frame_bow.)
 * Prints "<seq_label>: <mismatches>/<total>\n"; exits 0 iff mismatches==0.
 */
#define _POSIX_C_SOURCE 200809L
#include "sv_init.h"
#include "sv_match_area.h"
#include "sv_frame.h"
#include "sv_undistort.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const sv_camera_params CAM_U = {
    517.306408, 516.469215, 318.643040, 255.313989,
    0.262383, -0.953104, -0.005358, 0.002628, 1.163314
};
static const int CAM_COLS = 640, CAM_ROWS = 480;

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

/* ---- module-1 keypoints/descriptors, grouped per frame ---- */
typedef struct {
    sv_keypoint* kp;
    uint8_t* desc; /* 32 bytes/row */
    unsigned int n;
} frame_data;

static frame_data* g_frames = NULL;
static int g_num_frames = 0;

static void load_frames(const char* dump_dir, int max_frames) {
    char path[4096];
    FILE* f;
    char line[1024];
    int max_fi = -1;

    snprintf(path, sizeof(path), "%s/keypoints.tsv", dump_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    fgets(line, sizeof(line), f); /* header */
    while (fgets(line, sizeof(line), f)) {
        int fi;
        if (sscanf(line, "%d", &fi) == 1 && fi > max_fi) max_fi = fi;
    }
    fclose(f);

    g_num_frames = max_fi + 1;
    if (max_frames > 0 && max_frames < g_num_frames) g_num_frames = max_frames;
    g_frames = (frame_data*)calloc((size_t)g_num_frames, sizeof(frame_data));

    /* pass 1: counts */
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

    /* pass 2: fill keypoints */
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

    /* descriptors */
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

/* ---- reference_init tsvs ---- */
typedef struct {
    int ref_frame_id, cur_frame_id;
    unsigned int num_matches;
    char verdict[32], model[8];
    float cost_h, cost_f, rel_cost_h;
    int h_valid, f_valid;
    double H21[9], F21[9];
} attempt_row;

static attempt_row* g_att = NULL;
static int g_num_att = 0;

static void parse_hex9(const char* s, double out[9]) {
    int i = 0;
    const char* p = s;
    for (i = 0; i < 9; ++i) {
        out[i] = strtod(p, (char**)&p);
        if (*p == ',') p++;
    }
}

static void load_attempts(const char* init_dir) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/attempts.tsv", init_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    {
        int cnt = 0;
        long pos = ftell(f);
        while (getline(&line, &cap, f) != -1) cnt++;
        fseek(f, pos, SEEK_SET);
        g_att = (attempt_row*)calloc((size_t)cnt, sizeof(attempt_row));
        g_num_att = cnt;
    }
    {
        int i = 0;
        while (getline(&line, &cap, f) != -1) {
            int aid;
            char h21s[1024], f21s[1024];
            int nf = sscanf(line, "%d\t%d\t%d\t%u\t%31[^\t]\t%7[^\t]\t%a\t%a\t%a\t%d\t%d\t%1023[^\t]\t%1023s",
                            &aid, &g_att[i].ref_frame_id, &g_att[i].cur_frame_id, &g_att[i].num_matches,
                            g_att[i].verdict, g_att[i].model, &g_att[i].cost_h, &g_att[i].cost_f, &g_att[i].rel_cost_h,
                            &g_att[i].h_valid, &g_att[i].f_valid, h21s, f21s);
            if (nf == 13) {
                parse_hex9(h21s, g_att[i].H21);
                parse_hex9(f21s, g_att[i].F21);
            }
            i++;
        }
    }
    free(line);
    fclose(f);
}

/* matches.tsv: attempt_id -> list of (ref_idx,cur_idx) */
typedef struct { int ref_idx, cur_idx; } match_pair_row;
static match_pair_row** g_matches = NULL; /* [attempt][k] */
static int* g_matches_n = NULL;

static void load_matches(const char* init_dir, int num_kp_max) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    (void)num_kp_max;
    snprintf(path, sizeof(path), "%s/matches.tsv", init_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    g_matches = (match_pair_row**)calloc((size_t)g_num_att, sizeof(match_pair_row*));
    g_matches_n = (int*)calloc((size_t)g_num_att, sizeof(int));
    {
        int* cap_arr = (int*)calloc((size_t)g_num_att, sizeof(int));
        while (getline(&line, &cap, f) != -1) {
            int aid, ri, ci;
            if (sscanf(line, "%d\t%d\t%d", &aid, &ri, &ci) != 3) continue;
            if (aid < 0 || aid >= g_num_att) continue;
            if (g_matches_n[aid] >= cap_arr[aid]) {
                cap_arr[aid] = cap_arr[aid] ? cap_arr[aid] * 2 : 16;
                g_matches[aid] = (match_pair_row*)realloc(g_matches[aid], sizeof(match_pair_row) * (size_t)cap_arr[aid]);
            }
            g_matches[aid][g_matches_n[aid]].ref_idx = ri;
            g_matches[aid][g_matches_n[aid]].cur_idx = ci;
            g_matches_n[aid]++;
        }
        free(cap_arr);
    }
    free(line);
    fclose(f);
}

typedef struct { unsigned char h, f; } inlier_row;
static inlier_row** g_inliers = NULL;

static void load_inliers(const char* init_dir) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/inliers.tsv", init_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    g_inliers = (inlier_row**)calloc((size_t)g_num_att, sizeof(inlier_row*));
    {
        int a;
        for (a = 0; a < g_num_att; ++a) {
            g_inliers[a] = (inlier_row*)calloc((size_t)(g_matches_n[a] > 0 ? g_matches_n[a] : 1), sizeof(inlier_row));
        }
    }
    while (getline(&line, &cap, f) != -1) {
        int aid, mi, val;
        char solver[4];
        if (sscanf(line, "%d\t%1s\t%d\t%d", &aid, solver, &mi, &val) != 4) continue;
        if (aid < 0 || aid >= g_num_att) continue;
        if (mi < 0 || mi >= g_matches_n[aid]) continue;
        if (solver[0] == 'h') g_inliers[aid][mi].h = (unsigned char)val;
        else g_inliers[aid][mi].f = (unsigned char)val;
    }
    free(line);
    fclose(f);
}

typedef struct {
    unsigned int num_valid_pts, num_triangulated_pts;
    float parallax_cos;
    double rot[9], trans[3];
} hyp_row;
static hyp_row** g_hyps = NULL;
static int* g_hyps_n = NULL;

static void load_hyps(const char* init_dir) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/hyps.tsv", init_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    g_hyps = (hyp_row**)calloc((size_t)g_num_att, sizeof(hyp_row*));
    g_hyps_n = (int*)calloc((size_t)g_num_att, sizeof(int));
    {
        int* cap_arr = (int*)calloc((size_t)g_num_att, sizeof(int));
        while (getline(&line, &cap, f) != -1) {
            int aid, hidx;
            char model[8];
            unsigned int nv, nt;
            float pcos;
            char rots[1024], transs[256];
            int nf = sscanf(line, "%d\t%d\t%7[^\t]\t%u\t%u\t%a\t%1023[^\t]\t%255s",
                            &aid, &hidx, model, &nv, &nt, &pcos, rots, transs);
            if (nf != 8 || aid < 0 || aid >= g_num_att) continue;
            if (g_hyps_n[aid] >= cap_arr[aid]) {
                cap_arr[aid] = cap_arr[aid] ? cap_arr[aid] * 2 : 8;
                g_hyps[aid] = (hyp_row*)realloc(g_hyps[aid], sizeof(hyp_row) * (size_t)cap_arr[aid]);
            }
            {
                hyp_row* hr = &g_hyps[aid][g_hyps_n[aid]];
                hr->num_valid_pts = nv;
                hr->num_triangulated_pts = nt;
                hr->parallax_cos = pcos;
                parse_hex9(rots, hr->rot);
                {
                    const char* p = transs;
                    int k;
                    for (k = 0; k < 3; ++k) {
                        hr->trans[k] = strtod(p, (char**)&p);
                        if (*p == ',') p++;
                    }
                }
            }
            g_hyps_n[aid]++;
        }
        free(cap_arr);
    }
    free(line);
    fclose(f);
}

typedef struct {
    int selected_hyp;
    double rot[9], trans[3];
} final_row;
static final_row* g_final = NULL;

static void load_final(const char* init_dir) {
    char path[4096];
    FILE* f;
    char* line = NULL;
    size_t cap = 0;
    snprintf(path, sizeof(path), "%s/final.tsv", init_dir);
    f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
    getline(&line, &cap, f);
    g_final = (final_row*)calloc((size_t)g_num_att, sizeof(final_row));
    {
        int a;
        for (a = 0; a < g_num_att; ++a) g_final[a].selected_hyp = -1;
    }
    while (getline(&line, &cap, f) != -1) {
        int aid, sel;
        char rots[1024], transs[256];
        int nf = sscanf(line, "%d\t%d\t%1023[^\t]\t%255s", &aid, &sel, rots, transs);
        if (nf < 2 || aid < 0 || aid >= g_num_att) continue;
        g_final[aid].selected_hyp = sel;
        if (sel >= 0 && nf == 4) {
            parse_hex9(rots, g_final[aid].rot);
            {
                const char* p = transs;
                int k;
                for (k = 0; k < 3; ++k) {
                    g_final[aid].trans[k] = strtod(p, (char**)&p);
                    if (*p == ',') p++;
                }
            }
        }
    }
    free(line);
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n", argv[0]);
        return 2;
    }
    const char* seq_label = argv[1];
    const char* dump_dir = argv[3];
    int max_frames = argc > 4 ? atoi(argv[4]) : -1;

    char* init_dir = derive_dir(dump_dir, "reference_init");
    if (!init_dir) { fprintf(stderr, "cannot derive reference_init dir from %s\n", dump_dir); return 2; }

    load_frames(dump_dir, max_frames);
    load_attempts(init_dir);
    load_matches(init_dir, 0);
    load_inliers(init_dir);
    load_hyps(init_dir);
    load_final(init_dir);

    sv_camera_perspective cam;
    cam.fx = CAM_U.fx; cam.fy = CAM_U.fy; cam.cx = CAM_U.cx; cam.cy = CAM_U.cy;
    cam.focal_x_baseline = 0.0;
    {
        sv_image_bounds b;
        sv_compute_image_bounds(&CAM_U, CAM_COLS, CAM_ROWS, &b);
        cam.min_x = b.min_x; cam.max_x = b.max_x; cam.min_y = b.min_y; cam.max_y = b.max_y;
    }
    double cam_matrix[9] = { CAM_U.fx, 0, 0, 0, CAM_U.fy, 0, CAM_U.cx, CAM_U.cy, 1 };

    /* per-frame bearings (once) */
    double** bearings = (double**)calloc((size_t)g_num_frames, sizeof(double*));
    {
        int i;
        for (i = 0; i < g_num_frames; ++i) {
            bearings[i] = (double*)malloc(sizeof(double) * g_frames[i].n * 3);
            unsigned int k;
            for (k = 0; k < g_frames[i].n; ++k) {
                sv_camera_convert_point_to_bearing(&cam, g_frames[i].kp[k].x, g_frames[i].kp[k].y, &bearings[i][k * 3]);
            }
        }
    }

    /* per-frame grid (for matcher) */
    sv_frame_grid* grids = (sv_frame_grid*)calloc((size_t)g_num_frames, sizeof(sv_frame_grid));
    {
        int i;
        sv_image_bounds b;
        sv_compute_image_bounds(&CAM_U, CAM_COLS, CAM_ROWS, &b);
        for (i = 0; i < g_num_frames; ++i) {
            sv_frame_build_grid(g_frames[i].kp, g_frames[i].n, &b, 64, 48, &grids[i]);
        }
    }

    sv_init_params params;
    params.num_ransac_iters = 100;
    params.min_num_valid_pts = 50;
    params.min_num_triangulated_pts = 50;
    params.parallax_deg_thr = 1.0f;
    params.reproj_err_thr = 4.0f;

    int ref_i = 0;
    int have_ref = 0;
    float *prev_x = NULL, *prev_y = NULL;
    int aid = 0;

    int i;
    for (i = 0; i < g_num_frames; ++i) {
        if (!have_ref) {
            ref_i = i;
            prev_x = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
            prev_y = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
            {
                unsigned int k;
                for (k = 0; k < g_frames[ref_i].n; ++k) {
                    prev_x[k] = g_frames[ref_i].kp[k].x;
                    prev_y[k] = g_frames[ref_i].kp[k].y;
                }
            }
            have_ref = 1;
            continue;
        }

        int* matched = (int*)malloc(sizeof(int) * g_frames[ref_i].n);
        float* px = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
        float* py = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
        memcpy(px, prev_x, sizeof(float) * g_frames[ref_i].n);
        memcpy(py, prev_y, sizeof(float) * g_frames[ref_i].n);

        unsigned int num_matches = sv_match_in_consistent_area(
            g_frames[ref_i].kp, g_frames[ref_i].n, g_frames[ref_i].desc,
            g_frames[i].kp, g_frames[i].n, g_frames[i].desc,
            &grids[i], px, py, 100, matched);

        if (aid >= g_num_att) { free(matched); free(px); free(py); break; }
        attempt_row* ar = &g_att[aid];
        check_eq_i(ref_i, ar->ref_frame_id, "ref_frame_id");
        check_eq_i(i, ar->cur_frame_id, "cur_frame_id");
        if (num_matches != ar->num_matches && getenv("SV_DEBUG_FIRST")) {
            fprintf(stderr, "FIRST num_matches diff at aid=%d ref=%d cur=%d got=%u want=%u\n",
                   aid, ref_i, i, num_matches, ar->num_matches);
        }
        check_eq_i((long)num_matches, (long)ar->num_matches, "num_matches");

        /* matches.tsv */
        {
            int k, expect_i = 0;
            for (k = 0; k < (int)g_frames[ref_i].n; ++k) {
                if (matched[k] >= 0) {
                    if (expect_i < g_matches_n[aid]) {
                        check_eq_i(k, g_matches[aid][expect_i].ref_idx, "match.ref_idx");
                        check_eq_i(matched[k], g_matches[aid][expect_i].cur_idx, "match.cur_idx");
                    }
                    else {
                        check_eq_i(1, 0, "match.extra_row");
                    }
                    expect_i++;
                }
            }
            check_eq_i(expect_i, g_matches_n[aid], "match.count");
        }

        if (num_matches < params.min_num_valid_pts) {
            free(matched);
            ref_i = i;
            free(prev_x); free(prev_y);
            prev_x = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
            prev_y = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
            {
                unsigned int k;
                for (k = 0; k < g_frames[ref_i].n; ++k) {
                    prev_x[k] = g_frames[ref_i].kp[k].x;
                    prev_y[k] = g_frames[ref_i].kp[k].y;
                }
            }
            free(px); free(py);
            aid++;
            continue;
        }

        sv_init_attempt_result res;
        res.matched_2_in_1 = (int*)malloc(sizeof(int) * g_frames[ref_i].n);
        res.inlier_h = (unsigned char*)malloc(sizeof(unsigned char) * num_matches);
        res.inlier_f = (unsigned char*)malloc(sizeof(unsigned char) * num_matches);
        res.triangulated_pts = (double*)malloc(sizeof(double) * g_frames[ref_i].n * 3);
        res.is_triangulated = (unsigned char*)malloc(g_frames[ref_i].n);

        sv_init_try_monocular(g_frames[ref_i].kp, g_frames[ref_i].n, bearings[ref_i],
                              g_frames[i].kp, g_frames[i].n, bearings[i],
                              matched, &cam, &cam, cam_matrix, cam_matrix, &params, &res);

        check_eq_hexf(res.cost_h, ar->cost_h, "cost_h");
        check_eq_hexf(res.cost_f, ar->cost_f, "cost_f");
        check_eq_hexf(res.rel_cost_h, ar->rel_cost_h, "rel_cost_h");
        check_eq_i(res.h_valid, ar->h_valid, "h_valid");
        check_eq_i(res.f_valid, ar->f_valid, "f_valid");
        {
            int k;
            for (k = 0; k < 9; ++k) {
                check_eq_hexd(res.best_H21[k], ar->H21[k], "H21");
                check_eq_hexd(res.best_F21[k], ar->F21[k], "F21");
            }
        }
        {
            unsigned int m;
            for (m = 0; m < num_matches; ++m) {
                check_eq_i(res.inlier_h[m], g_inliers[aid][m].h, "inlier_h");
                check_eq_i(res.inlier_f[m], g_inliers[aid][m].f, "inlier_f");
            }
        }
        check_eq_i((long)res.num_hyps, (long)g_hyps_n[aid], "num_hyps");
        {
            unsigned int h;
            unsigned int nh = res.num_hyps < (unsigned int)g_hyps_n[aid] ? res.num_hyps : (unsigned int)g_hyps_n[aid];
            for (h = 0; h < nh; ++h) {
                check_eq_i((long)res.hyps[h].num_valid_pts, (long)g_hyps[aid][h].num_valid_pts, "hyp.num_valid_pts");
                check_eq_i((long)res.hyps[h].num_triangulated_pts, (long)g_hyps[aid][h].num_triangulated_pts, "hyp.num_triangulated_pts");
                check_eq_hexf(res.hyps[h].parallax_cos, g_hyps[aid][h].parallax_cos, "hyp.parallax_cos");
                {
                    int k;
                    for (k = 0; k < 9; ++k) {
                        if (res.hyps[h].rot[k] != g_hyps[aid][h].rot[k] && getenv("SV_DEBUG_FIRST_HYP")) {
                            fprintf(stderr, "FIRST_HYP_ROT_MISMATCH aid=%u h=%u k=%d model=%d ref_frame=%d cur_frame=%d\n",
                                   aid, h, k, (int)res.model_chosen, ar->ref_frame_id, ar->cur_frame_id);
                        }
                        check_eq_hexd(res.hyps[h].rot[k], g_hyps[aid][h].rot[k], "hyp.rot");
                    }
                    for (k = 0; k < 3; ++k) check_eq_hexd(res.hyps[h].trans[k], g_hyps[aid][h].trans[k], "hyp.trans");
                }
            }
        }
        {
            int got_sel = (res.verdict == SV_INIT_SUCCESS) ? (int)res.selected_hyp : -1;
            check_eq_i(got_sel, g_final[aid].selected_hyp, "final.selected_hyp");
            if (got_sel >= 0 && g_final[aid].selected_hyp >= 0) {
                int k;
                for (k = 0; k < 9; ++k) check_eq_hexd(res.rot_ref_to_cur[k], g_final[aid].rot[k], "final.rot");
                for (k = 0; k < 3; ++k) check_eq_hexd(res.trans_ref_to_cur[k], g_final[aid].trans[k], "final.trans");
            }
        }

        free(res.matched_2_in_1);
        free(res.inlier_h);
        free(res.inlier_f);
        free(res.triangulated_pts);
        free(res.is_triangulated);
        free(matched);

        /* Match dump_stella_init.cc's (and, ultimately, create_initializer()'s)
         * reference-frame reset: prev_matched_coords_ is reseeded from the
         * NEW reference frame's own keypoints, not carried over from the
         * matcher's mutated output array (which only updates entries that
         * matched -- using it directly here would leak stale coordinates
         * from an earlier reference frame into unmatched slots). */
        free(px); free(py);
        ref_i = i;
        free(prev_x); free(prev_y);
        prev_x = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
        prev_y = (float*)malloc(sizeof(float) * g_frames[ref_i].n);
        {
            unsigned int k;
            for (k = 0; k < g_frames[ref_i].n; ++k) {
                prev_x[k] = g_frames[ref_i].kp[k].x;
                prev_y[k] = g_frames[ref_i].kp[k].y;
            }
        }

        aid++;
    }

    printf("%s: %ld/%ld\n", seq_label, g_mismatches, g_total);
    return g_mismatches == 0 ? 0 : 1;
}
