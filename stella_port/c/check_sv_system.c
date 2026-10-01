/* SV_PORT_SOURCES: check_sv_system.c sv_system.c sv_loop.c sv_bow_db.c sv_g2o_sim3.c sv_sim3.c sv_eigen_lu3.c sv_map_match.c sv_eigen_svd.c sv_eigen_qr.c sv_rbtree.c sv_bundle_adjuster.c sv_g2o_ba.c sv_umap_order.c sv_eigen_amd.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_relocalizer.c sv_pnp.c ../reference_reloc/c/sv_eigen_pnp.c sv_extract.c sv_fast.c sv_image.c sv_init.c sv_map.c sv_match_area.c sv_solve_homography.c sv_solve_fundamental.c sv_solve_essential.c sv_solve_common.c sv_triangulate.c
 * SPDX-License-Identifier: MIT
 *
 * Harness for the top-level system (sv_system): runs the port CONTINUOUSLY over the whole sequence -- extraction,
 * initialization, tracking, keyframe insertion, mapping, loop detection, relocalization, keyframe lifetimes, all from
 * its own previous state, no teacher forcing -- and compares EVERY frame against the canonical reference dump:
 *   frame_trace.tsv   track path, initial / final pose bits, reference keyframe id, tracked / reliable counts
 *   kf_decision.tsv   keyframe decision with all its sub-flags
 *   frames_before.tsv frames_after.tsv (pose text, keyframe / landmark counts)
 *   matches.tsv       landmark id of every keypoint of every frame
 *   keyframes.tsv landmarks.tsv   the whole map after every frame (an order independent digest of every row's text:
 *                     poses, covisibilities, spanning tree, landmark position / descriptor / normal / range / counters
 *                     / reference keyframe / observations)
 * Items = compared lines + compared frame blocks. Prints "<seq>: mismatches/total"; exit 0 iff 0 mismatches.
 * The full diagnostics (first divergent frame, ATE) come from tools/run_stella_port_replay.py.
 * usage: check_sv_system <seq_label> <fixtures_dir> <dump_dir> [max_frames]   (run from the repo root) */
#define SV_RUN_NO_MAIN
#include "sv_run.c"

#define LINE_CAP (1 << 20)

static uint64_t fnv1a(const char* s, size_t n) {
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    for (i = 0; i < n; ++i) {
        h ^= (unsigned char)s[i];
        h *= 1099511628211ULL;
    }
    return h;
}

typedef struct block {
    long frame;
    unsigned long count;
    uint64_t sum;
} block;

/* order independent per-frame digest of a frame-keyed snapshot TSV */
static unsigned int load_blocks(const char* path, long max_frame, block** out) {
    FILE* f = fopen(path, "r");
    char* line = (char*)malloc(LINE_CAP);
    block* b = NULL;
    unsigned int n = 0, cap = 0;
    if (!f) {
        *out = NULL;
        free(line);
        return 0;
    }
    if (!fgets(line, LINE_CAP, f)) {
        fclose(f);
        free(line);
        *out = NULL;
        return 0;
    }
    while (fgets(line, LINE_CAP, f)) {
        const long fr = atol(line);
        if (max_frame >= 0 && fr >= max_frame) {
            break;
        }
        if (n == 0 || b[n - 1].frame != fr) {
            if (n == cap) {
                cap = cap ? cap * 2 : 1024;
                b = (block*)realloc(b, cap * sizeof(block));
            }
            b[n].frame = fr;
            b[n].count = 0;
            b[n].sum = 0;
            ++n;
        }
        b[n - 1].count++;
        b[n - 1].sum += fnv1a(line, strlen(line));
    }
    fclose(f);
    free(line);
    *out = b;
    return n;
}

static unsigned long g_total, g_bad;

static void note(const char* file, long frame, const char* what) {
    ++g_bad;
    if (g_bad <= 20) {
        fprintf(stderr, "  mismatch: %s frame %ld: %s\n", file, frame, what);
    }
}

static void compare_snapshot(const char* name, const char* ref_dir, const char* out_dir, long max_frames) {
    char rp[4096], mp[4096];
    block *rb, *mb;
    unsigned int nr, nm, i = 0, j = 0;
    snprintf(rp, sizeof(rp), "%s/%s", ref_dir, name);
    snprintf(mp, sizeof(mp), "%s/%s", out_dir, name);
    nr = load_blocks(rp, max_frames, &rb);
    nm = load_blocks(mp, max_frames, &mb);
    while (i < nr || j < nm) {
        ++g_total;
        if (i < nr && j < nm && rb[i].frame == mb[j].frame) {
            if (rb[i].count != mb[j].count || rb[i].sum != mb[j].sum) {
                note(name, rb[i].frame, "map snapshot differs");
            }
            ++i;
            ++j;
        }
        else if (j >= nm || (i < nr && rb[i].frame < mb[j].frame)) {
            note(name, rb[i].frame, "snapshot only in the reference");
            ++i;
        }
        else {
            note(name, mb[j].frame, "snapshot only in the port");
            ++j;
        }
    }
    free(rb);
    free(mb);
}

static void compare_lines(const char* name, const char* ref_dir, const char* out_dir, long max_frames) {
    char rp[4096], mp[4096];
    FILE *fr, *fm;
    char *a, *b;
    long frame = -1;
    snprintf(rp, sizeof(rp), "%s/%s", ref_dir, name);
    snprintf(mp, sizeof(mp), "%s/%s", out_dir, name);
    fr = fopen(rp, "r");
    fm = fopen(mp, "r");
    a = (char*)malloc(LINE_CAP);
    b = (char*)malloc(LINE_CAP);
    if (!fr || !fm) {
        ++g_total;
        note(name, -1, "file missing");
        goto done;
    }
    for (;;) {
        char* ra = fgets(a, LINE_CAP, fr);
        char* rb = fgets(b, LINE_CAP, fm);
        if (!ra && !rb) {
            break;
        }
        if (ra && frame >= 0 && max_frames >= 0 && atol(a) >= max_frames) {
            break;
        }
        ++g_total;
        if (ra) {
            frame = atol(a);
        }
        if (!ra || !rb || strcmp(a, b) != 0) {
            note(name, frame, (!ra || !rb) ? "length differs" : "line differs");
        }
        if (frame < 0) {
            frame = 0; /* header line consumed */
        }
    }
done:
    if (fr) fclose(fr);
    if (fm) fclose(fm);
    free(a);
    free(b);
}

int main(int argc, char** argv) {
    const char* seq_label;
    const char* fixtures_dir;
    const char* dump_dir;
    long max_frames = -1;
    char out_dir[4096], cmd[8192], path[4096];
    double* ts = NULL;
    unsigned int n = 0, cap = 0;
    char* line;
    FILE* f;
    sv_run_opts o;
    int rc;
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_system <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 2;
    }
    seq_label = argv[1];
    fixtures_dir = argv[2];
    dump_dir = argv[3];
    if (argc >= 5) {
        max_frames = atol(argv[4]);
    }
    /* frame timestamps (exact): track_pre.tsv column 2 (hex) */
    snprintf(path, sizeof(path), "%s/track_pre.tsv", dump_dir);
    f = fopen(path, "r");
    line = (char*)malloc(LINE_CAP);
    if (!f || !fgets(line, LINE_CAP, f)) {
        fprintf(stderr, "check_sv_system: cannot read %s\n", path);
        return 2;
    }
    while (fgets(line, LINE_CAP, f)) {
        char* p = strchr(line, '\t');
        if (!p) {
            continue;
        }
        if (n == cap) {
            cap = cap ? cap * 2 : 1024;
            ts = (double*)realloc(ts, cap * sizeof(double));
        }
        ts[n++] = strtod(p + 1, NULL);
    }
    fclose(f);
    free(line);

    snprintf(out_dir, sizeof(out_dir), "runs/stella_port/replay_check/%s", seq_label);
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", out_dir);
    if (system(cmd) != 0) {
        return 2;
    }
    memset(&o, 0, sizeof(o));
    o.max_frames = max_frames;
    o.snap_every = 1;
    o.enable_loop = 1;
    rc = sv_run_sequence("external/candidates/orb_vocab.fbow", fixtures_dir, ts, n, out_dir, &o, NULL);
    if (rc != 0) {
        fprintf(stderr, "check_sv_system: sv_run_sequence failed (%d)\n", rc);
        return 2;
    }
    compare_lines("frame_trace.tsv", dump_dir, out_dir, max_frames);
    compare_lines("kf_decision.tsv", dump_dir, out_dir, max_frames);
    compare_lines("frames_after.tsv", dump_dir, out_dir, max_frames);
    compare_lines("frames_before.tsv", dump_dir, out_dir, max_frames);
    compare_lines("matches.tsv", dump_dir, out_dir, max_frames);
    compare_snapshot("keyframes.tsv", dump_dir, out_dir, max_frames);
    compare_snapshot("landmarks.tsv", dump_dir, out_dir, max_frames);
    /* the big scratch files are not kept */
    snprintf(cmd, sizeof(cmd), "rm -f %s/matches.tsv %s/keyframes.tsv %s/landmarks.tsv", out_dir, out_dir, out_dir);
    if (system(cmd) != 0) {
        return 2;
    }
    printf("%s: %lu/%lu\n", seq_label, g_bad, g_total);
    return g_bad == 0 ? 0 : 1;
}
