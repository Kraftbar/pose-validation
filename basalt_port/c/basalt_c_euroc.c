/* BS_PORT_SOURCES: basalt_c_euroc.c bs_app.c bs_svd.c bs_flow.c bs_patch.c bs_fast.c bs_image.c ../../okvis_port/c/ok_png.c bs_marg.c bs_vio_opt.c bs_linabsqr.c bs_lmdb.c bs_hashorder.c bs_imu.c bs_cam.c bs_lie.c bs_eigenf.c */
#define _POSIX_C_SOURCE 199309L
/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt pure-C99 port, module M9: the end-to-end EuRoC driver, the counterpart of basalt_port/reference/driver/basalt_ref_driver.cpp.
 *   basalt_c_euroc --dataset-path D --cam-calib C.json --config-path CFG.json --out traj.tum [--max-frames N] [--quiet 1]
 * EuRoC loader as basalt/io/dataset_io_euroc.h (cam0 data.csv drives the frame list, the cam1 image has the same file name, IMU csv parsed field
 * by field), frame by frame: optical flow (bs_flow) -> estimator (bs_app), then the TUM file exactly as the reference driver writes it
 * ("# timestamp tx ty tz qx qy qz qw", "%.18e", timestamp = t_ns * 1e-9). */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bs_app.h"
#include "bs_image.h"

static char* slurp(const char* path, size_t* n) {
    FILE* f = fopen(path, "rb");
    long len;
    char* b;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    b = (char*)malloc((size_t)len + 1);
    if (b && fread(b, 1, (size_t)len, f) != (size_t)len) { free(b); b = NULL; }
    if (b) { b[len] = 0; *n = (size_t)len; }
    fclose(f);
    return b;
}

static const char* skip_ws(const char* p) { while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\v' || *p == '\f') p++; return p; }

typedef struct frame_ref { int64_t t_ns; char name[96]; } frame_ref;

/* read_image_timestamps: `ss >> t_ns >> tmp >> path` for every line not starting with '#' */
static int read_frames(const char* path, frame_ref** out, size_t* n_out) {
    size_t n = 0, cap = 0, len;
    char* buf = slurp(path, &len);
    char *line, *next;
    frame_ref* fr = NULL;
    if (!buf) return 1;
    for (line = buf; line && *line; line = next) {
        char* nl = strchr(line, '\n');
        const char* p;
        char* end;
        int64_t t;
        size_t k = 0;
        if (nl) { *nl = 0; next = nl + 1; } else next = NULL;
        if (line[0] == '#') continue;
        p = skip_ws(line);
        t = (int64_t)strtoll(p, &end, 10);
        if (end == p) { fprintf(stderr, "bad line in %s: %s\n", path, line); free(buf); free(fr); return 1; }
        p = skip_ws(end);
        if (*p) p++;                                    /* the separator char */
        p = skip_ws(p);
        if (n == cap) { cap = cap ? 2 * cap : 4096; fr = (frame_ref*)realloc(fr, cap * sizeof *fr); }
        while (p[k] && p[k] != ' ' && p[k] != '\t' && p[k] != '\r' && k + 1 < sizeof fr[n].name) { fr[n].name[k] = p[k]; k++; }
        fr[n].name[k] = 0;
        fr[n].t_ns = t;
        n++;
    }
    free(buf);
    *out = fr; *n_out = n;
    return 0;
}

/* read_imu_data: timestamp, gyro x y z, accel x y z, `>> tmp` consumes the separator char */
static int read_imu(const char* path, bs_imu_raw** out, size_t* n_out) {
    size_t n = 0, cap = 0, len;
    char* buf = slurp(path, &len);
    char *line, *next;
    bs_imu_raw* r = NULL;
    if (!buf) return 1;
    for (line = buf; line && *line; line = next) {
        char* nl = strchr(line, '\n');
        const char* p;
        char* end;
        double v[6];
        uint64_t ts;
        int i;
        if (nl) { *nl = 0; next = nl + 1; } else next = NULL;
        if (line[0] == '#') continue;
        p = skip_ws(line);
        ts = (uint64_t)strtoull(p, &end, 10);
        if (end == p) { fprintf(stderr, "bad IMU line in %s: %s\n", path, line); free(buf); free(r); return 1; }
        p = end;
        for (i = 0; i < 6; ++i) {
            p = skip_ws(p);
            if (*p) p++;                                /* tmp */
            p = skip_ws(p);
            v[i] = strtod(p, &end);
            if (end == p) { fprintf(stderr, "bad IMU line in %s: %s\n", path, line); free(buf); free(r); return 1; }
            p = end;
        }
        if (n == cap) { cap = cap ? 2 * cap : 65536; r = (bs_imu_raw*)realloc(r, cap * sizeof *r); }
        r[n].t_ns = (int64_t)ts;
        for (i = 0; i < 3; ++i) { r[n].gyro[i] = v[i]; r[n].accel[i] = v[3 + i]; }
        n++;
    }
    free(buf);
    *out = r; *n_out = n;
    return 0;
}

static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec; }

int main(int argc, char** argv) {
    const char *dataset = NULL, *calib_path = NULL, *config_path = NULL, *out_path = "trajectory.tum";
    size_t max_frames = 0, i, n_frames = 0, n_imu = 0, n_out = 0, cap_out = 0;
    int quiet = 0, a;
    char err[256], path[1024];
    frame_ref* frames = NULL;
    bs_imu_raw* imu = NULL;
    bs_app_cfg cfg;
    bs_app app;
    bs_flow_config fcfg;
    bs_flow_calib fcal;
    bs_flow* flow;
    bs_app_out* outs = NULL;
    double t0;
    for (a = 1; a + 1 < argc; a += 2) {
        const char *k = argv[a], *v = argv[a + 1];
        if (!strcmp(k, "--dataset-path")) dataset = v;
        else if (!strcmp(k, "--cam-calib")) calib_path = v;
        else if (!strcmp(k, "--config-path")) config_path = v;
        else if (!strcmp(k, "--out")) out_path = v;
        else if (!strcmp(k, "--max-frames")) max_frames = (size_t)strtoul(v, NULL, 10);
        else if (!strcmp(k, "--quiet")) quiet = atoi(v);
        else { fprintf(stderr, "unknown option %s\n", k); return 2; }
    }
    if (!dataset || !calib_path || !config_path) { fprintf(stderr, "usage: %s --dataset-path D --cam-calib C --config-path CFG --out OUT [--max-frames N]\n", argv[0]); return 2; }
    if (bs_app_cfg_load(&cfg, config_path, calib_path, err, sizeof err)) { fprintf(stderr, "config: %s\n", err); return 2; }
    bs_flow_config_default(&fcfg);
    if (bs_flow_config_load(config_path, &fcfg)) { fprintf(stderr, "cannot read the optical flow keys of %s\n", config_path); return 2; }
    if (bs_flow_calib_load(calib_path, &fcal)) { fprintf(stderr, "cannot read %s\n", calib_path); return 2; }
    flow = bs_flow_new(&fcfg, &fcal);
    if (!flow) { fprintf(stderr, "bs_flow_new failed (unsupported flow configuration)\n"); return 2; }

    snprintf(path, sizeof path, "%s/mav0/cam0/data.csv", dataset);
    if (read_frames(path, &frames, &n_frames)) { fprintf(stderr, "cannot read %s\n", path); return 2; }
    snprintf(path, sizeof path, "%s/mav0/imu0/data.csv", dataset);
    if (read_imu(path, &imu, &n_imu)) { fprintf(stderr, "cannot read %s\n", path); return 2; }
    if (bs_app_init(&app, &cfg)) return 2;
    if (bs_app_set_imu(&app, imu, n_imu)) { fprintf(stderr, "%s\n", app.fatal_msg); return 1; }

    t0 = now_s();
    for (i = 0; i < n_frames; ++i) {
        uint16_t* img[2] = {NULL, NULL};
        int w[2], h[2], cam, rc;
        bs_app_out o;
        if (max_frames > 0 && i >= max_frames) break;
        for (cam = 0; cam < 2; ++cam) {
            FILE* f;
            snprintf(path, sizeof path, "%s/mav0/cam%d/data/%s", dataset, cam, frames[i].name);
            f = fopen(path, "rb");
            if (!f) break;                                  /* fs::exists false: res[i].img stays null, processFrame returns */
            fclose(f);
            rc = bs_image_load_euroc(path, &img[cam], &w[cam], &h[cam]);
            if (rc != BS_IMG_OK) { fprintf(stderr, "cannot decode %s (code %d)\n", path, rc); return 1; }
        }
        if (!img[0] || !img[1]) { free(img[0]); free(img[1]); continue; }
        if (w[0] != w[1] || h[0] != h[1]) { fprintf(stderr, "camera images differ in size at frame %zu\n", i); return 1; }
        {
            const uint16_t* cimg[2] = {img[0], img[1]};
            rc = bs_flow_process(flow, frames[i].t_ns, cimg, w[0], h[0]);
        }
        free(img[0]); free(img[1]);
        if (rc) { fprintf(stderr, "bs_flow_process failed at frame %zu (code %d)\n", i, rc); return 1; }
        {
            bs_flow_obs obs[2];
            obs[0] = *bs_flow_result(flow, 0);
            obs[1] = *bs_flow_result(flow, 1);
            rc = bs_app_frame(&app, frames[i].t_ns, obs, &o);
        }
        if (rc < 0) { fprintf(stderr, "frame %zu (t_ns %lld): %s\n", i, (long long)frames[i].t_ns, app.fatal_msg); return 1; }
        if (rc > 0) break;                                  /* the estimator's loop ended */
        if (n_out == cap_out) { cap_out = cap_out ? 2 * cap_out : 4096; outs = (bs_app_out*)realloc(outs, cap_out * sizeof *outs); }
        outs[n_out++] = o;
        if (!quiet && (i % 500) == 0) { fprintf(stderr, "frame %zu / %zu  %.1f s\n", i, n_frames, now_s() - t0); }
    }
    {
        FILE* f = fopen(out_path, "w");
        if (!f) { fprintf(stderr, "cannot write %s\n", out_path); return 1; }
        fputs("# timestamp tx ty tz qx qy qz qw\n", f);
        for (i = 0; i < n_out; ++i)
            fprintf(f, "%.18e %.18e %.18e %.18e %.18e %.18e %.18e %.18e\n", (double)outs[i].t_ns * 1e-9, outs[i].t[0], outs[i].t[1], outs[i].t[2],
                    outs[i].q[0], outs[i].q[1], outs[i].q[2], outs[i].q[3]);
        fclose(f);
    }
    fprintf(stderr, "states %zu wall_s %.3f kf %ld landmarks_added %ld triangulations %ld lmdb_ub %d\n", n_out, now_s() - t0, app.stat_kf, app.stat_landmarks_added,
            app.stat_triangulate, bs_lmdb_ub);
    bs_app_destroy(&app);
    bs_flow_free(flow);
    free(frames); free(imu); free(outs);
    return 0;
}
