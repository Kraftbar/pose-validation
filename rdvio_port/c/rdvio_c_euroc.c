/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port: the EuRoC driver of the C system (rd_sys.h), the counterpart of the reference driver
 * runs/rdvio_port/<tree>/rdvio_gray_driver.cpp (THREADING=OFF Handler).
 *
 *   rdvio_c_euroc <sensor.yaml> <setting.yaml> <mav0 dir> <out.tum> --gray <undistorted pack> [--masks <swt.bin>
 *                 [--logged-essential]] [--max-seconds s]
 *
 * imu0/data.csv and cam0/data.csv are read like the reference (timestamps through strtod, / 1e9 above 1e12). For every camera
 * frame, the IMU samples up to its time go in as track_gyroscope then track_accelerometer; then the image goes in. While the
 * system is tracking, one TUM line per frame: the camera time and the feature tracker's latest state, %.17g.
 * --gray: the undistorted images ("OKGRAY1\0" pack, rdvio_port/reference_tools/rd_undistort_pack.cc) until module M7b ports
 *         initUndistortRectifyMap + remap.
 * --masks: the PnP PARSAC inlier masks of the reference run (patch 0012 with RDVIO_PORT_SWT_MASKS=1) until EPnP is ported
 *          (without it judge_track_status is skipped). The essential PARSAC is native; --logged-essential takes its masks
 *          from the log too.
 */
#include "rd_sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef RD_PNP_OPENCV
void rd_pnp6_opencv(void* ctx, const double X[6][3], const double x[6][2], double T[16]);   /* reference_tools/rd_pnp6_opencv.cc */
#endif

typedef struct imu_row { double t, w[3], a[3]; } imu_row;
typedef struct cam_row { double t; unsigned long long stamp; } cam_row;

static double ts(const char* s) { const double v = strtod(s, NULL); return v > 1e12 ? v / 1e9 : v; }

/* ---- the PARSAC masks of the reference run ---- */
typedef struct masks { FILE* f; long used, bad; int essential; } masks;
static void read_mask(masks* m, uint32_t want, size_t n, char* mask) {
    uint32_t tag;
    uint64_t len, npts, nm;
    memset(mask, 1, n);
    if (!m->f) return;
    for (;;) {
        if (fread(&tag, 4, 1, m->f) != 1 || fread(&len, 8, 1, m->f) != 1 || len < 16 ||
            fread(&npts, 8, 1, m->f) != 1 || fread(&nm, 8, 1, m->f) != 1) {
            if (!m->bad++) fprintf(stderr, "masks: log ended at mask %ld\n", m->used);
            return;
        }
        if (tag == 6 && !m->essential && want != 6) { fseek(m->f, (long)(len - 16), SEEK_CUR); continue; }   /* native essential */
        break;
    }
    if (tag != want || npts != n || nm != n) {
        if (!m->bad++) fprintf(stderr, "masks: mask %ld: record %u with %llu points, C wants record %u with %zu\n", m->used, tag,
                               (unsigned long long)npts, want, n);
        fseek(m->f, (long)(len - 16), SEEK_CUR);
        return;
    }
    if (fread(mask, 1, n, m->f) != n) m->bad++;
    m->used++;
}
static void pnp_mask(void* ctx, size_t n, const double* p3d, const double* p2d, const size_t* lens, const double Rcw[9],
                     const double tcw[3], double inv_f, char* mask) {
    (void)p3d; (void)p2d; (void)lens; (void)Rcw; (void)tcw; (void)inv_f;
    read_mask((masks*)ctx, 5, n, mask);
}
static void ess_mask(void* ctx, size_t n, const double* pts1, const double* pts2, double threshold, char* mask) {
    (void)pts1; (void)pts2; (void)threshold;
    read_mask((masks*)ctx, 6, n, mask);
}

static char* read_line(FILE* f, char* buf, size_t n) {
    if (!fgets(buf, (int)n, f)) return NULL;
    return buf;
}

int main(int argc, char** argv) {
    const char *gray_path = NULL, *mask_path = NULL;
    double maxs = 1e18;
    rd_cfg cfg;
    char err[256], path[4096], line[4096];
    imu_row* imus = NULL;
    cam_row* cams = NULL;
    size_t nimu = 0, cimu = 0, ncam = 0, ccam = 0, ii = 0, frames = 0, poses = 0, k;
    FILE *f, *gray, *out;
    char magic[8];
    uint32_t gh[3];
    double t0, first_pose = -1;
    int prev = -1, a;
    masks mk;
    rd_swt_hooks parsac;
    rd_sys* sys;
    uint8_t* pix;
    memset(&mk, 0, sizeof mk);
    if (argc < 5) {
        fprintf(stderr, "usage: rdvio_c_euroc sensor.yaml setting.yaml mav0 out.tum --gray pack [--masks swt.bin] [--max-seconds s]\n");
        return 1;
    }
    for (a = 5; a < argc; ++a) {
        if (!strcmp(argv[a], "--gray") && a + 1 < argc) gray_path = argv[++a];
        else if (!strcmp(argv[a], "--masks") && a + 1 < argc) mask_path = argv[++a];
        else if (!strcmp(argv[a], "--max-seconds") && a + 1 < argc) maxs = atof(argv[++a]);
        else if (!strcmp(argv[a], "--logged-essential")) mk.essential = 1;
        else { fprintf(stderr, "unknown argument %s\n", argv[a]); return 1; }
    }
    if (!gray_path) { fprintf(stderr, "--gray is required (undistorted image pack)\n"); return 1; }
    if (rd_cfg_load(argv[2], argv[1], &cfg, err, sizeof err)) { fprintf(stderr, "config: %s\n", err); return 1; }
    snprintf(path, sizeof path, "%s/imu0/data.csv", argv[3]);
    if (!(f = fopen(path, "r"))) { fprintf(stderr, "cannot open %s\n", path); return 1; }
    read_line(f, line, sizeof line);
    while (read_line(f, line, sizeof line)) {
        char *p, *e;
        imu_row r;
        int i;
        if (line[0] == '\n' || line[0] == '\r' || !line[0]) continue;
        for (p = line; *p; ++p) if (*p == ',') *p = ' ';
        p = line;
        while (*p == ' ') ++p;
        e = p; while (*e && *e != ' ') ++e;
        { char c = *e; *e = 0; r.t = ts(p); *e = c; }
        p = e;
        for (i = 0; i < 3; ++i) r.w[i] = strtod(p, &p);
        for (i = 0; i < 3; ++i) r.a[i] = strtod(p, &p);
        if (nimu == cimu) { cimu = cimu * 2 + 1024; imus = (imu_row*)realloc(imus, cimu * sizeof *imus); }
        imus[nimu++] = r;
    }
    fclose(f);
    snprintf(path, sizeof path, "%s/cam0/data.csv", argv[3]);
    if (!(f = fopen(path, "r"))) { fprintf(stderr, "cannot open %s\n", path); return 1; }
    read_line(f, line, sizeof line);
    while (read_line(f, line, sizeof line)) {
        char* c = strchr(line, ',');
        cam_row r;
        if (line[0] == '\n' || line[0] == '\r' || !line[0] || !c) continue;
        *c = 0;
        r.t = ts(line);
        r.stamp = strtoull(c + 1, NULL, 10);      /* the file name's stem (stoull(c.second.substr(0, '.'))) */
        if (ncam == ccam) { ccam = ccam * 2 + 1024; cams = (cam_row*)realloc(cams, ccam * sizeof *cams); }
        cams[ncam++] = r;
    }
    fclose(f);
    if (!nimu || !ncam) { fprintf(stderr, "no data\n"); return 1; }
    gray = fopen(gray_path, "rb");
    if (!gray || fread(magic, 1, 8, gray) != 8 || memcmp(magic, "OKGRAY1\0", 8) || fread(gh, 4, 3, gray) != 3 ||
        (int)gh[0] != cfg.resolution[0] || (int)gh[1] != cfg.resolution[1] || gh[2] != ncam) { fprintf(stderr, "bad gray pack\n"); return 3; }
    memset(&parsac, 0, sizeof parsac);
    if (mask_path) {
        if (!(mk.f = fopen(mask_path, "rb"))) { fprintf(stderr, "cannot open %s\n", mask_path); return 1; }
        parsac.ctx = &mk; parsac.pnp_mask = pnp_mask;
        if (mk.essential) parsac.ess_mask = ess_mask;
    }
    sys = rd_sys_create(&cfg, &parsac, NULL, NULL);
#ifdef RD_PNP_OPENCV   /* reference tooling build: RD-VIO's own solve_pnp_6pt (real OpenCV) for the native IMU-PARSAC */
    if (!mask_path) rd_sys_set_pnp_solver(sys, rd_pnp6_opencv, NULL);
#endif
    t0 = imus[0].t < cams[0].t ? imus[0].t : cams[0].t;
    if (!(out = fopen(argv[4], "w"))) { fprintf(stderr, "cannot open %s\n", argv[4]); return 1; }
    pix = (uint8_t*)malloc((size_t)gh[0] * gh[1]);
    for (k = 0; k < ncam; ++k) {
        const cam_row* c = &cams[k];
        unsigned long long stamp;
        rd_sys_image* im;
        int st;
        if (c->t - t0 > maxs) break;
        while (ii < nimu && imus[ii].t <= c->t) {
            const imu_row* m = &imus[ii];
            rd_sys_track_gyroscope(sys, m->t, m->w[0], m->w[1], m->w[2], NULL);
            rd_sys_track_accelerometer(sys, m->t, m->a[0], m->a[1], m->a[2], NULL);
            ii++;
        }
        if (fread(&stamp, 8, 1, gray) != 1 || stamp != c->stamp || fread(pix, 1, (size_t)gh[0] * gh[1], gray) != (size_t)gh[0] * gh[1]) {
            fprintf(stderr, "gray pack out of step at frame %zu\n", k);
            return 4;
        }
        im = rd_sys_image_new(c->t, pix, (int)gh[0], (int)gh[1]);
        rd_sys_track_camera(sys, im, NULL);
        frames++;
        st = rd_sys_state(sys);
        if (st != prev) { fprintf(stderr, "t=%.3f state %d -> %d\n", c->t - t0, prev, st); prev = st; }
        if (st == RD_SYS_TRACKING) {
            double tt;
            rd_pose p;
            rd_sys_latest_state(sys, &tt, &p);
            fprintf(out, "%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", c->t, p.p[0], p.p[1], p.p[2], p.q.x, p.q.y, p.q.z, p.q.w);
            poses++;
            if (first_pose < 0) first_pose = c->t - t0;
        }
    }
    fprintf(stderr, "FRAMES %zu POSES %zu FIRST_POSE_S %.2f\n", frames, poses, first_pose);
    if (mask_path) fprintf(stderr, "masks: %ld used, %ld problems\n", mk.used, mk.bad);
    if (rd_sys_parsac_grid_errors(sys)) fprintf(stderr, "essential PARSAC: %ld calls with points off the grid\n", rd_sys_parsac_grid_errors(sys));
    fclose(out); fclose(gray);
    rd_sys_free(sys);
    rd_sys_image_reset_statics();
    if (mk.f) fclose(mk.f);
    free(pix); free(imus); free(cams);
    return 0;
}
