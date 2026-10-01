/* SV_PORT_SOURCES: sv_run.c sv_system.c sv_loop.c sv_bow_db.c sv_g2o_sim3.c sv_sim3.c sv_eigen_lu3.c sv_map_match.c sv_eigen_svd.c sv_eigen_qr.c sv_rbtree.c sv_bundle_adjuster.c sv_g2o_ba.c sv_umap_order.c sv_eigen_amd.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_relocalizer.c sv_pnp.c ../reference_reloc/c/sv_eigen_pnp.c sv_extract.c sv_fast.c sv_image.c sv_init.c sv_map.c sv_match_area.c sv_solve_homography.c sv_solve_fundamental.c sv_solve_essential.c sv_solve_common.c sv_triangulate.c
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
 * usage: sv_run <vocab.fbow> <tum_seq_dir> <fixtures_dir> <out_dir> [max_frames] [--snap-every N] [--no-snap]
 *               [--snap-from F] [--resume-mapper] [--no-loop]
 */
#include "sv_system.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
/* tum_rgbd_sequence::acquire_image_information: 3 header lines, then `timestamp file` rows */
static int read_index(const char* seq_dir, const char* name, double** ts_out, unsigned int* n_out) {
    char path[4096], line[1024];
    FILE* f;
    unsigned int n = 0, cap = 0, skip = 3;
    double* ts = NULL;
    snprintf(path, sizeof(path), "%s/%s", seq_dir, name);
    f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    while (fgets(line, sizeof(line), f)) {
        double t;
        char file[512];
        if (skip) {
            --skip;
            continue;
        }
        if (line[0] == '\n' || line[0] == '\0') {
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
    long snap_every; /* 0 = off */
    long snap_from;
    int resume_mapper;
    int enable_loop;
    int verbose;
    long blank_from, blank_to; /* --blank A-B: feed a flat gray image for frames A..B (forces Lost / relocalization / reset) */
    int has_camera;
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

/* Runs the whole sequence. ts: timestamps of every frame (frame_idx order). Returns 0 on success. */
static int sv_run_sequence(const char* vocab_path, const char* fixtures_dir, const double* ts, unsigned int n_frames,
                           const char* out_dir, const sv_run_opts* o, sv_system_stats* stats_out) {
    size_t vlen = 0;
    uint8_t* vbuf = read_file(vocab_path, &vlen);
    sv_bow_vocab vocab;
    sv_system_params p;
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
    p.resume_mapper_after_loop = o->resume_mapper;
    p.enable_loop_closure = o->enable_loop;
    sys = sv_system_create(&p);

#define OPEN(fp, name)                                              \
    snprintf(path, sizeof(path), "%s/%s", out_dir, name);          \
    fp = fopen(path, "w");                                          \
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

    for (i = 0; i < n_frames; ++i) {
        sv_frame_result r;
        int w, h;
        uint8_t* gray;
        const int* lms;
        unsigned int nkp;
        if (o->max_frames >= 0 && (long)i >= o->max_frames) {
            break;
        }
        snprintf(path, sizeof(path), "%s/%06u.pgm", fixtures_dir, i);
        gray = read_pgm(path, &w, &h);
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
        FILE* f;
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
        free(e);
    }
    if (stats_out) {
        sv_system_get_stats(sys, stats_out);
    }
    if (o->verbose) {
        sv_system_stats st;
        sv_system_get_stats(sys, &st);
        fprintf(stderr, "sv_run: %ld frames, %u keyframes inserted, %u global steps, %u loops accepted, %u lost frames, %u resets, %u erased / %u destroyed keyframes\n",
                processed, st.keyframes_inserted, st.global_steps, st.loops_accepted, st.lost_frames, st.resets, st.erased_keyframes, st.destroyed_keyframes);
    }
    sv_system_destroy(sys);
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
                        "[--snap-from F] [--snap-loop] [--resume-mapper] [--no-loop] [--blank A-B] [--camera fx,fy,cx,cy,k1,k2,p1,p2,k3]\n");
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
