/* SV_PORT_SOURCES: sv_run.c sv_imu.c sv_imu_gyro.c sv_rot.c sv_system.c sv_loop.c sv_bow_db.c sv_g2o_sim3.c sv_sim3.c sv_eigen_lu3.c sv_map_match.c sv_eigen_svd.c sv_eigen_qr.c sv_rbtree.c sv_bundle_adjuster.c sv_g2o_ba.c sv_umap_order.c sv_eigen_amd.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_relocalizer.c sv_pnp.c sv_eigen_pnp.c sv_extract.c sv_fast.c sv_image.c sv_init.c sv_map.c sv_match_area.c sv_solve_homography.c sv_solve_fundamental.c sv_solve_essential.c sv_solve_common.c sv_triangulate.c
 * SPDX-License-Identifier: BSD-2-Clause AND MIT
 *
 * sv_run: driver of the pure-C stella_vslam port (sv_system). The ONLY file of the port that uses stdio.
 *
 * Reads a TUM RGB-D sequence the way the reference driver (stella_port/reference/driver/main.cc, tum_rgbd_util.cc)
 * does: rgb.txt / depth.txt, the nearest depth frame per RGB frame with the 0.1 s threshold, frame timestamp =
 * (rgb + depth) / 2. The image itself is NOT decoded here (PNG + cvtColor need OpenCV): the driver reads the exact
 * gray frames the reference feeds to orb_extractor::extract(), as 8-bit PGMs written by tools/dump_stella_fixtures.cc
 * (runs/stella_port/fixtures/<seq>/%06d.pgm, the same files every other harness of the port uses).
 *
 * Output (all in the reference's own text formats, see stella_port/reference/driver/main.cc):
 *   frames_before.tsv  frame_trace.tsv  kf_decision.tsv  matches.tsv  frames_after.tsv  trajectory.tum
 *   keyframes.tsv landmarks.tsv        (map snapshots, --snap-every N, default 1; --no-snap disables;
 *                                       --snap-loop: only right after an accepted loop and after the last frame)
 *   loop_log.tsv       (accepted loops, keyframe lifetime log, run statistics)
 *
 * Frame size: from the first fixture's PGM header (or --size WxH); the camera intrinsics default to TUM fr1 unless --camera.
 *
 * usage: sv_run <vocab.fbow> <tum_seq_dir> <fixtures_dir> <out_dir> [max_frames] [--snap-every N] [--no-snap]
 *               [--snap-from F] [--resume-mapper] [--no-loop]
 */
#define _POSIX_C_SOURCE 200809L /* nanosleep for --wait-fixtures */
#include "sv_system.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* formatting (identical to the reference driver's fmt_* helpers)      */
/* ------------------------------------------------------------------ */
static void fmt_mat44_9g(FILE* f, const double m[16]) { /* m column-major, printed row-major like Eigen's (r,c) loop */
    int r, c;
    for (r = 0; r < 4; ++r) {
        for (c = 0; c < 4; ++c) {
            fprintf(f, "%.9g", m[c * 4 + r]);
            if (!(r == 3 && c == 3)) {
                fputc(',', f);
            }
        }
    }
}

static void fmt_mat44_hex(FILE* f, const double m[16]) {
    int r, c;
    for (r = 0; r < 4; ++r) {
        for (c = 0; c < 4; ++c) {
            fprintf(f, "%a", m[c * 4 + r]);
            if (!(r == 3 && c == 3)) {
                fputc(',', f);
            }
        }
    }
}

static const char* path_name(int p) {
    switch (p) {
        case SV_TR_PATH_NONE: return "none";
        case SV_TR_PATH_MOTION: return "motion_model";
        case SV_TR_PATH_BOW: return "bow_match";
        case SV_TR_PATH_ROBUST: return "robust_match";
        case SV_TR_PATH_RELOC_BY_POSE: return "relocalize_by_pose";
        case SV_TR_PATH_RELOC_AUTO: return "relocalize_auto";
    }
    return "unknown";
}

static void hex_desc(FILE* f, const uint8_t* d) {
    int i;
    for (i = 0; i < 32; ++i) {
        fprintf(f, "%02x", d[i]);
    }
}

/* ------------------------------------------------------------------ */
/* input                                                              */
/* ------------------------------------------------------------------ */
static uint8_t* read_file(const char* path, size_t* len) {
    FILE* f = fopen(path, "rb");
    uint8_t* buf;
    long n;
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (uint8_t*)malloc((size_t)n + 1);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return buf;
}

static uint8_t* read_pgm(const char* path, int* w, int* h) {
    FILE* f = fopen(path, "rb");
    char magic[3] = {0};
    int maxval;
    uint8_t* buf;
    if (!f) {
        return NULL;
    }
    if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P5") != 0 || fscanf(f, "%d %d %d", w, h, &maxval) != 3) {
        fclose(f);
        return NULL;
    }
    fgetc(f);
    buf = (uint8_t*)malloc((size_t)(*w) * (size_t)(*h));
    if (fread(buf, 1, (size_t)(*w) * (size_t)(*h), f) != (size_t)(*w) * (size_t)(*h)) {
        free(buf);
        buf = NULL;
    }
    fclose(f);
    return buf;
}

#ifndef SV_RUN_NO_MAIN /* sequence association: only the stand-alone driver needs it (the harness takes exact timestamps from the dump) */
/* tum_rgbd_sequence::acquire_image_information skips the first 3 lines of rgb.txt / depth.txt (the TUM header).
 * Here every line starting with '#' is skipped instead, which is identical for TUM files (exactly 3 '#' lines) and
 * also correct for files with a different number of header lines (including none). */
static int read_index(const char* seq_dir, const char* name, double** ts_out, unsigned int* n_out) {
    char path[4096], line[1024];
    FILE* f;
    unsigned int n = 0, cap = 0;
    double* ts = NULL;
    snprintf(path, sizeof(path), "%s/%s", seq_dir, name);
    f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    while (fgets(line, sizeof(line), f)) {
        double t;
        char file[512];
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\0') {
            continue;
        }
        if (sscanf(line, "%lf %511s", &t, file) < 1) {
            continue;
        }
        if (n == cap) {
            cap = cap ? cap * 2 : 1024;
            ts = (double*)realloc(ts, cap * sizeof(double));
        }
        ts[n++] = t;
    }
    fclose(f);
    *ts_out = ts;
    *n_out = n;
    return 0;
}

/* tum_rgbd_sequence: nearest depth frame per RGB frame (strict <, first minimum), reject over the threshold */
static int associate(const char* seq_dir, double** frame_ts, unsigned int* n_frames) {
    double *rgb, *depth, *out;
    unsigned int nr, nd, i, j, n = 0;
    const double thr = 0.1;
    if (read_index(seq_dir, "rgb.txt", &rgb, &nr) || read_index(seq_dir, "depth.txt", &depth, &nd) || !nd) {
        return -1;
    }
    out = (double*)malloc((nr ? nr : 1) * sizeof(double));
    for (i = 0; i < nr; ++i) {
        double nearest = depth[0];
        double min_diff = fabs(rgb[i] - nearest);
        for (j = 0; j < nd; ++j) {
            const double diff = fabs(rgb[i] - depth[j]);
            if (diff < min_diff) {
                min_diff = diff;
                nearest = depth[j];
            }
        }
        if (thr < min_diff) {
            continue;
        }
        out[n++] = (rgb[i] + nearest) / 2.0;
    }
    free(rgb);
    free(depth);
    *frame_ts = out;
    *n_frames = n;
    return 0;
}

#endif

/* ------------------------------------------------------------------ */
/* the run                                                            */
/* ------------------------------------------------------------------ */
typedef struct sv_run_opts {
    long max_frames;
    long skip; /* --skip N: start at frame N (timestamps / fixtures keep their global index) */
    long snap_every; /* 0 = off */
    long snap_from;
    int resume_mapper;
    int enable_loop;
    int verbose;
    long blank_from, blank_to; /* --blank A-B: feed a flat gray image for frames A..B (forces Lost / relocalization / reset) */
    int wait_fx; /* --wait-fixtures: a missing fixture is waited for (up to 10 min; a streaming feeder writes and deletes the PGMs) instead of ending the run */
    int lean; /* --lean: the per-frame dump files go to /dev/null (only trajectory + loop_log are kept) */
    int n_set;
    const char* set[32]; /* --set key=value (stella_vio parameters, see apply_set) */
    int width, height; /* --size WxH; 0 = take the size from the first fixture image */
    int has_camera;
    const char* imu_path; /* --imu imu.csv (t_ns,gx,gy,gz,ax,ay,az) */
    const char* imu_ext;  /* --imu-ext ext.txt: 12 numbers, R_BC row-major then p_BC */
    double imu_toff;      /* --imu-toff s: IMU clock = camera clock + s */
    double imu_bg[3];     /* --imu-bg x,y,z */
    double camera[9]; /* --camera fx,fy,cx,cy,k1,k2,p1,p2,k3: override the default (fr1) camera, e.g. TUM_RGBD_mono_2/3.yaml */
} sv_run_opts;

static void write_snapshot(FILE* fk, FILE* fl, sv_system* sys, long frame) {
    const sv_tr_map* m = sv_system_map(sys);
    sv_mapping* mp = sv_system_mapping(sys);
    unsigned int id, j;
    for (id = 0; id < m->kf_cap; ++id) {
        const sv_tr_kf* kf = m->kfs[id];
        unsigned int *cid, *cw, nc;
        if (!kf || !kf->alive) {
            continue;
        }
        cid = (unsigned int*)malloc((kf->n_covis + 1) * sizeof(unsigned int));
        cw = (unsigned int*)malloc((kf->n_covis + 1) * sizeof(unsigned int));
        nc = sv_mapping_covisibilities(mp, kf, cid, cw);
        fprintf(fk, "%ld\t%u\t", frame, kf->id);
        fmt_mat44_9g(fk, kf->pose_cw);
        fputc('\t', fk);
        fmt_mat44_hex(fk, kf->pose_cw);
        fprintf(fk, "\t0\t");
        for (j = 0; j < nc; ++j) {
            fprintf(fk, "%s%u:%u", j ? "," : "", cid[j], cw[j]);
        }
        fprintf(fk, "\t%d\t", kf->parent >= 0 ? kf->parent : -1);
        for (j = 0; j < kf->n_children; ++j) {
            fprintf(fk, "%s%u", j ? "," : "", kf->children[j]);
        }
        fputc('\n', fk);
        free(cid);
        free(cw);
    }
    for (id = 0; id < m->lm_cap; ++id) {
        const sv_tr_lm* lm = m->lms[id];
        if (!lm || !lm->alive) {
            continue;
        }
        fprintf(fl, "%ld\t%u\t%.9g,%.9g,%.9g\t%a,%a,%a\t", frame, lm->id, lm->pos_w[0], lm->pos_w[1], lm->pos_w[2], lm->pos_w[0],
                lm->pos_w[1], lm->pos_w[2]);
        hex_desc(fl, lm->desc);
        fprintf(fl, "\t%.9g,%.9g,%.9g\t%a,%a,%a\t%.9g\t%.9g\t%u\t%u\t%d\t", lm->mean_normal[0], lm->mean_normal[1], lm->mean_normal[2],
                lm->mean_normal[0], lm->mean_normal[1], lm->mean_normal[2], (double)lm->min_valid_dist, (double)lm->max_valid_dist,
                lm->num_observed, lm->num_observable, lm->ref_kf);
        for (j = 0; j < lm->num_obs; ++j) {
            fprintf(fl, "%s%u:%u", j ? "," : "", lm->obs_kf[j], lm->obs_idx[j]);
        }
        fputc('\n', fl);
    }
}

/* stella_vio parameter overrides: --set key=value (repeatable). Unknown keys are an error.
 * R-frames: rframe rf_max_sec rf_init_after rf_hold_sec rf_scale rf_gyro_max rf_calib ; map merge: merge (see RESULTS.md) */
static int apply_set(sv_system_params* p, const char* kv) {
    char key[64];
    double v;
    if (sscanf(kv, "%63[^=]=%lf", key, &v) != 2) {
        return -1;
    }
    if (!strcmp(key, "reinit_sec")) {
        p->reinit_lost_sec = v;
    }
    else if (!strcmp(key, "init_parallax")) {
        p->init_parallax_deg = (float)v;
    }
    else if (!strcmp(key, "init_par_frac")) {
        p->init_par_frac = (float)v;
    }
    else if (!strcmp(key, "orb_min_area")) {
        p->orb.min_area = (unsigned int)v;
    }
    else if (!strcmp(key, "orb_levels")) {
        p->orb.num_levels = (int)v;
    }
    else if (!strcmp(key, "orb_scale")) {
        p->orb.scale_factor = (float)v;
    }
    else if (!strcmp(key, "orb_fast")) {
        p->orb.ini_fast_thr = (int)v;
    }
    else if (!strcmp(key, "orb_minfast")) {
        p->orb.min_fast_thr = (int)v;
    }
    else if (!strcmp(key, "init_hamm")) {
        p->init_hamm = (unsigned int)v;
    }
    else if (!strcmp(key, "init_ratio")) {
        p->init_ratio = (float)v;
    }
    else if (!strcmp(key, "init_max_level")) {
        p->init_max_level = (int)v;
    }
    else if (!strcmp(key, "init_min_valid")) {
        p->init_min_valid = (unsigned int)v;
    }
    else if (!strcmp(key, "init_confirm")) {
        p->init_confirm = (unsigned int)v;
    }
    else if (!strcmp(key, "gyro")) {
        p->gyro_mode = (int)v;
    }
    else if (!strcmp(key, "gravity")) {
        p->gravity = (int)v;
    }
    else if (!strcmp(key, "dr_sec")) {
        p->dr_max_sec = v;
    }
    else if (!strcmp(key, "init_seeds")) {
        p->init_seeds = (unsigned int)v;
    }
    else if (!strcmp(key, "init_tri")) {
        p->init_min_tri = (unsigned int)v;
    }
    else if (!strcmp(key, "merge")) {
        p->merge_maps = (int)v;
    }
    else if (!strcmp(key, "rframe")) {
        p->rframe = (int)v;
    }
    else if (!strcmp(key, "rf_max_sec")) {
        p->rframe_max_sec = v;
    }
    else if (!strcmp(key, "rf_init_after")) {
        p->rframe_init_after = v;
    }
    else if (!strcmp(key, "rf_hold_sec")) {
        p->rframe_hold_sec = v;
    }
    else if (!strcmp(key, "rf_scale")) {
        p->rframe_scale = (int)v;
    }
    else if (!strcmp(key, "rf_calib")) {
        p->rframe_calib_sec = v;
    }
    else if (!strcmp(key, "rf_gyro_max")) {
        p->rframe_gyro_max = (unsigned int)v;
    }
    else {
        return -1;
    }
    return 0;
}

/* Runs the whole sequence. ts: timestamps of every frame (frame_idx order). Returns 0 on success. */
static int sv_run_sequence(const char* vocab_path, const char* fixtures_dir, const double* ts, unsigned int n_frames,
                           const char* out_dir, const sv_run_opts* o, sv_system_stats* stats_out) {
    size_t vlen = 0;
    uint8_t* vbuf = read_file(vocab_path, &vlen);
    sv_bow_vocab vocab;
    sv_system_params p;
    sv_imu_buf imu;
    sv_system* sys;
    char path[4096];
    FILE *fb = NULL, *ft = NULL, *fdc = NULL, *fm = NULL, *fa = NULL, *fk = NULL, *fl = NULL, *flog = NULL;
    unsigned int i, k;
    long processed = 0, last_frame = -1;
    int rc = 0;

    if (!vbuf || sv_bow_load_memory(vbuf, vlen, &vocab) != 0) {
        fprintf(stderr, "sv_run: cannot load vocabulary %s\n", vocab_path);
        return 2;
    }
    sv_system_params_default(&p, &vocab);
    if (o->has_camera) {
        p.cam.fx = o->camera[0];
        p.cam.fy = o->camera[1];
        p.cam.cx = o->camera[2];
        p.cam.cy = o->camera[3];
        p.cam.k1 = o->camera[4];
        p.cam.k2 = o->camera[5];
        p.cam.p1 = o->camera[6];
        p.cam.p2 = o->camera[7];
        p.cam.k3 = o->camera[8];
    }
    if (o->width > 0 && o->height > 0) {
        p.cols = o->width;
        p.rows = o->height;
    }
    else { /* frame size from the first fixture's PGM header */
        int fw, fh;
        uint8_t* g0;
        snprintf(path, sizeof(path), "%s/%06u.pgm", fixtures_dir, 0u);
        g0 = read_pgm(path, &fw, &fh);
        if (g0) {
            p.cols = fw;
            p.rows = fh;
            free(g0);
        }
    }
    for (k = 0; k < (unsigned int)o->n_set; ++k) {
        if (apply_set(&p, o->set[k])) {
            fprintf(stderr, "sv_run: bad --set %s\n", o->set[k]);
            return 1;
        }
    }
    p.resume_mapper_after_loop = o->resume_mapper;
    p.enable_loop_closure = o->enable_loop;
    sv_imu_buf_init(&imu, 250000000LL);
    if (o->imu_path) {
        FILE* fi = fopen(o->imu_path, "r");
        char line[1024];
        if (!fi) {
            fprintf(stderr, "sv_run: cannot read %s\n", o->imu_path);
            return 2;
        }
        while (fgets(line, sizeof line, fi)) {
            sv_imu_sample smp;
            long long tn;
            if (line[0] == '#') {
                continue;
            }
            if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &tn, &smp.gyr[0], &smp.gyr[1], &smp.gyr[2], &smp.acc[0], &smp.acc[1], &smp.acc[2]) == 7) {
                smp.t_ns = (int64_t)tn;
                sv_imu_buf_push(&imu, &smp);
            }
        }
        fclose(fi);
        if (o->imu_ext) {
            double e[12];
            fi = fopen(o->imu_ext, "r");
            for (k = 0; fi && k < 12; ++k) {
                if (fscanf(fi, "%lf", &e[k]) != 1) {
                    break;
                }
            }
            if (!fi || k < 12) {
                fprintf(stderr, "sv_run: bad --imu-ext\n");
                return 2;
            }
            fclose(fi);
            memcpy(p.imu_R_BC, e, 9 * sizeof(double));
        }
        p.imu = &imu;
        p.imu_toff = o->imu_toff;
        memcpy(p.imu_bg, o->imu_bg, sizeof(p.imu_bg));
        fprintf(stderr, "sv_run: imu %zu samples (dup %ld back %ld gap %ld) toff %.4f gyro_mode %d gravity %d\n", imu.n, imu.n_dup, imu.n_back, imu.n_gap,
                o->imu_toff, p.gyro_mode, p.gravity);
    }
    sys = sv_system_create(&p);

#define OPEN(fp, name)                                              \
    snprintf(path, sizeof(path), "%s/%s", out_dir, o->lean && strcmp(name, "loop_log.tsv") ? "dump_null" : name); \
    fp = fopen(o->lean && strcmp(name, "loop_log.tsv") ? "/dev/null" : path, "w");                                          \
    if (!fp) {                                                      \
        fprintf(stderr, "sv_run: cannot write %s\n", path);         \
        return 2;                                                   \
    }
    OPEN(fb, "frames_before.tsv");
    OPEN(ft, "frame_trace.tsv");
    OPEN(fdc, "kf_decision.tsv");
    OPEN(fm, "matches.tsv");
    OPEN(fa, "frames_after.tsv");
    OPEN(flog, "loop_log.tsv");
    if (o->snap_every != 0) {
        OPEN(fk, "keyframes.tsv");
        OPEN(fl, "landmarks.tsv");
        fprintf(fk, "frame_idx\tkf_id\tpose_cw_9g\tpose_cw_hex\tbad\tcovisibilities\tspanning_parent\tspanning_children\n");
        fprintf(fl, "frame_idx\tlm_id\tpos_w_9g\tpos_w_hex\tdescriptor_hex\tmean_normal_9g\tmean_normal_hex\t"
                    "min_valid_dist_9g\tmax_valid_dist_9g\tnum_observed\tnum_observable\tref_keyfrm_id\tobservations\n");
    }
#undef OPEN
    fprintf(fb, "frame_idx\ttimestamp\ttracked\tpose_row_major_9g\tpose_row_major_hex\n");
    fprintf(ft, "frame_idx\ttrack_path\tref_keyfrm_id\tinitial_pose_valid\tinitial_pose_9g\tinitial_pose_hex\t"
                "final_pose_valid\tfinal_pose_9g\tfinal_pose_hex\tnum_tracked_lms\tnum_reliable_lms\n");
    fprintf(fdc, "frame_idx\tverdict\tmapper_paused_or_pausing\tnum_reliable_lms_ref\tnum_reliable_lms\tnum_tracked_lms\t"
                 "distance_traveled_9g\tmax_interval_elapsed\tmin_interval_elapsed\tmax_distance_traveled\tmin_distance_traveled\t"
                 "view_changed\tnot_enough_lms\tenough_keyfrms\ttracking_is_unstable\talmost_all_lms_are_tracked\tmapper_is_skipping_localBA\n");
    fprintf(fm, "frame_idx\tkp_idx\tlandmark_id\n");
    fprintf(fa, "frame_idx\tnum_keyframes\tnum_landmarks\n");
    fprintf(flog, "kind\tframe\ta\tb\n");

    for (i = (unsigned int)o->skip; i < n_frames; ++i) {
        sv_frame_result r;
        int w, h;
        uint8_t* gray;
        const int* lms;
        unsigned int nkp;
        if (o->max_frames >= 0 && (long)i - o->skip >= o->max_frames) {
            break;
        }
        snprintf(path, sizeof(path), "%s/%06u.pgm", fixtures_dir, i);
        gray = read_pgm(path, &w, &h);
        if (!gray && o->wait_fx) {
            int tries;
            for (tries = 0; !gray && tries < 60000; ++tries) {
                struct timespec ts;
                ts.tv_sec = 0; ts.tv_nsec = 10000000L;
                nanosleep(&ts, NULL);
                gray = read_pgm(path, &w, &h);
            }
        }
        if (!gray || w != p.cols || h != p.rows) {
            fprintf(stderr, "sv_run: cannot read fixture %s\n", path);
            rc = 2;
            break;
        }
        if (o->blank_to >= o->blank_from && o->blank_from > 0 && (long)i >= o->blank_from && (long)i <= o->blank_to) {
            memset(gray, 128, (size_t)w * (size_t)h);
        }
        sv_system_feed(sys, gray, ts[i], &r);
        free(gray);
        ++processed;

        /* frames_before.tsv */
        fprintf(fb, "%u\t%.9g\t%d\t", i, ts[i], r.pose_valid ? 1 : 0);
        if (r.pose_valid) {
            fmt_mat44_9g(fb, r.pose_wc);
            fputc('\t', fb);
            fmt_mat44_hex(fb, r.pose_wc);
        }
        else {
            fputs("-\t-", fb);
        }
        fputc('\n', fb);

        /* frame_trace.tsv */
        fprintf(ft, "%u\t%s\t%d\t%d\t", i, path_name(r.track_path), r.ref_kf, r.initial_pose_valid ? 1 : 0);
        fmt_mat44_9g(ft, r.initial_pose);
        fputc('\t', ft);
        fmt_mat44_hex(ft, r.initial_pose);
        fprintf(ft, "\t%d\t", r.pose_valid ? 1 : 0);
        if (r.pose_valid) {
            fmt_mat44_9g(ft, r.pose_wc);
            fputc('\t', ft);
            fmt_mat44_hex(ft, r.pose_wc);
        }
        else {
            fputs("-\t-", ft);
        }
        fprintf(ft, "\t%u\t%u\n", r.num_tracked, r.num_reliable);

        /* kf_decision.tsv */
        {
            const sv_tr_kf_decision* d = &r.decision;
            fprintf(fdc, "%u\t%d\t%d\t%u\t%u\t%u\t%.9g\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", i, d->verdict ? 1 : 0,
                    d->mapper_paused_or_pausing ? 1 : 0, d->num_reliable_lms_ref, d->num_reliable_lms, d->num_tracked_lms,
                    (double)d->distance_traveled, d->max_interval_elapsed ? 1 : 0, d->min_interval_elapsed ? 1 : 0,
                    d->max_distance_traveled ? 1 : 0, d->min_distance_traveled ? 1 : 0, d->view_changed ? 1 : 0,
                    d->not_enough_lms ? 1 : 0, d->enough_keyfrms ? 1 : 0, d->tracking_is_unstable ? 1 : 0,
                    d->almost_all_lms_are_tracked ? 1 : 0, d->mapper_is_skipping_localBA ? 1 : 0);
        }

        /* matches.tsv: landmark of every keypoint of the current frame */
        lms = sv_system_curr_landmarks(sys, &nkp);
        for (k = 0; k < nkp; ++k) {
            fprintf(fm, "%u\t%u\t%d\n", i, k, lms[k] >= 0 ? lms[k] : -1);
        }

        fprintf(fa, "%u\t%u\t%u\n", i, r.n_keyframes, r.n_landmarks);

        if (r.loop_accepted) {
            fprintf(flog, "loop\t%u\t%d\t%d\n", i, r.loop_cur_kf, r.loop_cand_kf);
        }
        if (r.reset_happened) {
            fprintf(flog, "reset\t%u\t0\t0\n", i);
        }
        if (r.cal_f != 0.0 && o->verbose) {
            fprintf(stderr, "sv_run: scale calibration frame %u f %.3f vold %.4f vnew %.4f\n", i, r.cal_f, r.cal_vold, r.cal_vnew);
        }
        if (r.rframe_kind && o->verbose) {
            fprintf(stderr, "sv_run: rframe frame %u kind %d inliers %u par %.3f\n", i, r.rframe_kind, r.rframe_inliers, r.rframe_par_deg);
        }
        if (r.initialized) {
            fprintf(flog, "init\t%u\t0\t0\n", i);
        }
        if (o->snap_every > 0 && (long)i >= o->snap_from && (r.n_keyframes > 0) && ((long)i % o->snap_every == 0 || r.inserted_kf >= 0 || r.initialized || r.loop_accepted)) {
            write_snapshot(fk, fl, sys, (long)i);
        }
        else if (o->snap_every < 0 && r.loop_accepted) { /* --snap-loop: the map right after an accepted loop */
            write_snapshot(fk, fl, sys, (long)i);
        }
        last_frame = (long)i;
        if (o->verbose && (i % 100) == 0) {
            fprintf(stderr, "sv_run: frame %u kfs=%u lms=%u\n", i, r.n_keyframes, r.n_landmarks);
        }
    }

    if (o->snap_every < 0 && last_frame >= 0) { /* final map */
        write_snapshot(fk, fl, sys, last_frame);
    }
    /* lifetime log: erase / destroy frame of every keyframe */
    {
        const sv_tr_map* m = sv_system_map(sys);
        for (k = 0; k < m->kf_cap; ++k) {
            const int e = sv_system_kf_erased_frame(sys, k), d = sv_system_kf_destroyed_frame(sys, k);
            if (e >= 0) {
                fprintf(flog, "erased\t%d\t%u\t%d\n", e, k, d);
            }
        }
    }
    fclose(fb);
    fclose(ft);
    fclose(fdc);
    fclose(fm);
    fclose(fa);
    fclose(flog);
    if (fk) {
        fclose(fk);
        fclose(fl);
    }

    /* system::save_frame_trajectory(path, "TUM") */
    {
        sv_traj_entry* e;
        unsigned int n, q;
        FILE *f, *fm2;
        sv_system_trajectory(sys, &e, &n);
        snprintf(path, sizeof(path), "%s/trajectory.tum", out_dir);
        f = fopen(path, "w");
        if (f) {
            for (q = 0; q < n; ++q) {
                fprintf(f, "%.15g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", e[q].timestamp, e[q].pose_wc[12], e[q].pose_wc[13],
                        e[q].pose_wc[14], e[q].quat_xyzw[0], e[q].quat_xyzw[1], e[q].quat_xyzw[2], e[q].quat_xyzw[3]);
            }
            fclose(f);
        }
        snprintf(path, sizeof(path), "%s/trajectory_maps.tum", out_dir); /* + 9th column: map id (stella_vio re-initializations) */
        fm2 = fopen(path, "w");
        if (fm2) {
            for (q = 0; q < n; ++q) {
                fprintf(fm2, "%.15g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d %d %d\n", e[q].timestamp, e[q].pose_wc[12], e[q].pose_wc[13],
                        e[q].pose_wc[14], e[q].quat_xyzw[0], e[q].quat_xyzw[1], e[q].quat_xyzw[2], e[q].quat_xyzw[3], e[q].map_id, e[q].rframe, e[q].seg);
            }
            fclose(fm2);
        }
        if (p.imu && p.gravity) { /* per-map up vector and the gravity-aligned trajectory (map frame rotated so that up = +z) */
            FILE *fg, *fz;
            int mid, maxmid = 0;
            double Rg[9], up[3];
            for (q = 0; q < n; ++q) {
                maxmid = e[q].map_id > maxmid ? e[q].map_id : maxmid;
            }
            snprintf(path, sizeof(path), "%s/gravity.txt", out_dir);
            fg = fopen(path, "w");
            snprintf(path, sizeof(path), "%s/trajectory_gz.tum", out_dir);
            fz = fopen(path, "w");
            for (mid = 0; fg && fz && mid <= maxmid; ++mid) {
                const unsigned int cnt = sv_system_map_up(sys, mid, up);
                double ax, ay, ang, ca = up[2], n2;
                if (!cnt) {
                    continue;
                }
                fprintf(fg, "%d %u %.6f %.6f %.6f\n", mid, cnt, up[0], up[1], up[2]);
                ax = up[1]; ay = -up[0]; /* u x z = (uy, -ux, 0) */
                n2 = sqrt(ax * ax + ay * ay);
                ang = acos(ca > 1.0 ? 1.0 : (ca < -1.0 ? -1.0 : ca));
                if (n2 < 1e-12) {
                    Rg[0] = Rg[4] = Rg[8] = 1.0; Rg[1] = Rg[2] = Rg[3] = Rg[5] = Rg[6] = Rg[7] = 0.0;
                }
                else { /* Rodrigues */
                    const double kx = ax / n2, ky = ay / n2, c = cos(ang), sn = sin(ang), v = 1.0 - c;
                    Rg[0] = c + kx * kx * v;   Rg[1] = kx * ky * v;       Rg[2] = ky * sn;
                    Rg[3] = kx * ky * v;       Rg[4] = c + ky * ky * v;   Rg[5] = -kx * sn;
                    Rg[6] = -ky * sn;          Rg[7] = kx * sn;           Rg[8] = c;
                }
                for (q = 0; q < n; ++q) {
                    double R[9], B[9], pz[3], tr, qx, qy, qz, qw;
                    int a, b, c2;
                    if (e[q].map_id != mid) {
                        continue;
                    }
                    for (a = 0; a < 3; ++a) {
                        for (b = 0; b < 3; ++b) {
                            R[a * 3 + b] = e[q].pose_wc[b * 4 + a]; /* column-major -> row-major R_wc */
                        }
                    }
                    for (a = 0; a < 3; ++a) {
                        pz[a] = Rg[a * 3 + 0] * e[q].pose_wc[12] + Rg[a * 3 + 1] * e[q].pose_wc[13] + Rg[a * 3 + 2] * e[q].pose_wc[14];
                        for (b = 0; b < 3; ++b) {
                            B[a * 3 + b] = 0.0;
                            for (c2 = 0; c2 < 3; ++c2) {
                                B[a * 3 + b] += Rg[a * 3 + c2] * R[c2 * 3 + b];
                            }
                        }
                    }
                    tr = B[0] + B[4] + B[8];
                    if (tr > 0.0) {
                        const double sq = sqrt(tr + 1.0) * 2.0;
                        qw = 0.25 * sq; qx = (B[7] - B[5]) / sq; qy = (B[2] - B[6]) / sq; qz = (B[3] - B[1]) / sq;
                    }
                    else if (B[0] > B[4] && B[0] > B[8]) {
                        const double sq = sqrt(1.0 + B[0] - B[4] - B[8]) * 2.0;
                        qw = (B[7] - B[5]) / sq; qx = 0.25 * sq; qy = (B[1] + B[3]) / sq; qz = (B[2] + B[6]) / sq;
                    }
                    else if (B[4] > B[8]) {
                        const double sq = sqrt(1.0 + B[4] - B[0] - B[8]) * 2.0;
                        qw = (B[2] - B[6]) / sq; qx = (B[1] + B[3]) / sq; qy = 0.25 * sq; qz = (B[5] + B[7]) / sq;
                    }
                    else {
                        const double sq = sqrt(1.0 + B[8] - B[0] - B[4]) * 2.0;
                        qw = (B[3] - B[1]) / sq; qx = (B[2] + B[6]) / sq; qy = (B[5] + B[7]) / sq; qz = 0.25 * sq;
                    }
                    fprintf(fz, "%.15g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d\n", e[q].timestamp, pz[0], pz[1], pz[2], qx, qy, qz, qw, mid);
                }
            }
            if (fg) fclose(fg);
            if (fz) fclose(fz);
        }
        free(e);
    }
    if (stats_out) {
        sv_system_get_stats(sys, stats_out);
    }
    if (o->verbose) {
        sv_system_stats st;
        sv_system_get_stats(sys, &st);
        fprintf(stderr, "sv_run: maps=%u reinits=%u merges=%u rframes=%u rframes_gyro=%u rbridges=%u rfail=%u\n", st.reinits + 1, st.reinits, st.merges, st.rframes, st.rframes_gyro, st.rbridges, st.rfail);
        fprintf(stderr, "sv_run: %ld frames, %u keyframes inserted, %u global steps, %u loops accepted, %u lost frames, %u resets, %u erased / %u destroyed keyframes\n",
                processed, st.keyframes_inserted, st.global_steps, st.loops_accepted, st.lost_frames, st.resets, st.erased_keyframes, st.destroyed_keyframes);
    }
    if (o->verbose && p.imu) {
        const sv_tracker* tk = sv_system_tracker(sys);
        fprintf(stderr, "sv_run: gyro_tracked=%u dead_reckon_tries=%u dead_reckon_ok=%u\n", tk->n_gyro_track, tk->n_dr_try, tk->n_dr_ok);
    }
    sv_system_destroy(sys);
    sv_imu_buf_free(&imu);
    free(vbuf);
    return rc;
}

#ifndef SV_RUN_NO_MAIN
int main(int argc, char** argv) {
    sv_run_opts o;
    double* ts = NULL;
    unsigned int n = 0;
    int i;
    if (argc < 5) {
        fprintf(stderr, "usage: sv_run <vocab.fbow> <tum_seq_dir> <fixtures_dir> <out_dir> [max_frames] [--snap-every N] [--no-snap] "
                        "[--snap-from F] [--snap-loop] [--resume-mapper] [--no-loop] [--blank A-B] [--size WxH] [--set key=val] [--camera fx,fy,cx,cy,k1,k2,p1,p2,k3]\n");
        return 1;
    }
    memset(&o, 0, sizeof(o));
    o.max_frames = -1;
    o.snap_every = 1;
    o.enable_loop = 1;
    o.verbose = 1;
    for (i = 5; i < argc; ++i) {
        if (!strcmp(argv[i], "--snap-every") && i + 1 < argc) {
            o.snap_every = atol(argv[++i]);
        }
        else if (!strcmp(argv[i], "--no-snap")) {
            o.snap_every = 0;
        }
        else if (!strcmp(argv[i], "--snap-loop")) {
            o.snap_every = -1;
        }
        else if (!strcmp(argv[i], "--snap-from") && i + 1 < argc) {
            o.snap_from = atol(argv[++i]);
        }
        else if (!strcmp(argv[i], "--resume-mapper")) {
            o.resume_mapper = 1;
        }
        else if (!strcmp(argv[i], "--no-loop")) {
            o.enable_loop = 0;
        }
        else if (!strcmp(argv[i], "--blank") && i + 1 < argc) {
            sscanf(argv[++i], "%ld-%ld", &o.blank_from, &o.blank_to);
        }
        else if (!strcmp(argv[i], "--wait-fixtures")) {
            o.wait_fx = 1;
        }
        else if (!strcmp(argv[i], "--lean")) {
            o.lean = 1;
        }
        else if (!strcmp(argv[i], "--skip") && i + 1 < argc) {
            o.skip = atol(argv[++i]);
        }
        else if (!strcmp(argv[i], "--set") && i + 1 < argc && o.n_set < 32) {
            o.set[o.n_set++] = argv[++i];
        }
        else if (!strcmp(argv[i], "--imu") && i + 1 < argc) {
            o.imu_path = argv[++i];
        }
        else if (!strcmp(argv[i], "--imu-ext") && i + 1 < argc) {
            o.imu_ext = argv[++i];
        }
        else if (!strcmp(argv[i], "--imu-toff") && i + 1 < argc) {
            o.imu_toff = atof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--imu-bg") && i + 1 < argc) {
            sscanf(argv[++i], "%lf,%lf,%lf", &o.imu_bg[0], &o.imu_bg[1], &o.imu_bg[2]);
        }
        else if (!strcmp(argv[i], "--size") && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &o.width, &o.height) != 2 || o.width <= 0 || o.height <= 0) {
                fprintf(stderr, "sv_run: --size needs WxH\n");
                return 1;
            }
        }
        else if (!strcmp(argv[i], "--camera") && i + 1 < argc) {
            o.has_camera = sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &o.camera[0], &o.camera[1], &o.camera[2],
                                  &o.camera[3], &o.camera[4], &o.camera[5], &o.camera[6], &o.camera[7], &o.camera[8]) == 9;
            if (!o.has_camera) {
                fprintf(stderr, "sv_run: --camera needs fx,fy,cx,cy,k1,k2,p1,p2,k3\n");
                return 1;
            }
        }
        else {
            o.max_frames = atol(argv[i]);
        }
    }
    if (associate(argv[2], &ts, &n) != 0) {
        fprintf(stderr, "sv_run: cannot read rgb.txt / depth.txt in %s\n", argv[2]);
        return 2;
    }
    {
        const int rc = sv_run_sequence(argv[1], argv[3], ts, n, argv[4], &o, NULL);
        free(ts);
        return rc;
    }
}
#endif
