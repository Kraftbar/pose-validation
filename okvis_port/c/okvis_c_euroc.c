/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * okvis_c_euroc: the pure-C OKVIS2 port on a EuRoC sequence, the counterpart of okvis_app_synchronous (module 8).
 *
 *   okvis_c_euroc <config.yaml> <sequence dir> <vocabulary.bin> <out dir> [max frames]
 *
 * <sequence dir> holds mav0/imu0/data.csv and gray/cam<i>.gray (the images decoded by tools/okvis_port_images.py; the port
 * has no PNG decoder); <vocabulary.bin> is the DBoW2 payload of tools/convert_okvis_vocabulary.py. Writes
 * <out dir>/causal.csv (the state published after every frame, TrajectoryOutput) and <out dir>/final.csv
 * (ViSlamBackend::writeFinalCsvTrajectory), byte-identical to the deterministic reference build's outputs.
 *
 * Derived from OKVIS2 okvis_apps / DatasetReader.cpp (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich,
 * 2020 Smart Robotics Lab / Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt).
 */
#include "ok_system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct gpack { FILE* f; uint32_t w, h, n; uint64_t* ts; } gpack;

static int gpack_open(gpack* g, const char* path) {
    char magic[8];
    uint32_t i;
    memset(g, 0, sizeof *g);
    g->f = fopen(path, "rb");
    if (!g->f) return 0;
    if (fread(magic, 8, 1, g->f) != 1 || memcmp(magic, "OKGRAY1", 8) || fread(&g->w, 4, 1, g->f) != 1 ||
        fread(&g->h, 4, 1, g->f) != 1 || fread(&g->n, 4, 1, g->f) != 1) return 0;
    g->ts = (uint64_t*)malloc(8 * (size_t)(g->n ? g->n : 1));
    for (i = 0; i < g->n; ++i)
        if (fseek(g->f, 20 + (long)i * (8 + (long)g->w * (long)g->h), SEEK_SET) || fread(&g->ts[i], 8, 1, g->f) != 1) return 0;
    return 1;
}
static int gpack_read(gpack* g, uint64_t ts, uint8_t* out) {
    uint32_t lo = 0, hi = g->n;
    while (lo < hi) { const uint32_t mid = lo + (hi - lo) / 2; if (g->ts[mid] < ts) lo = mid + 1; else hi = mid; }
    if (lo == g->n || g->ts[lo] != ts) return 0;
    if (fseek(g->f, 20 + (long)lo * (8 + (long)g->w * (long)g->h) + 8, SEEK_SET)) return 0;
    return fread(out, (size_t)g->w * g->h, 1, g->f) == 1;
}
static void on_publish(void* ctx, const ok_sys_state* s) { ok_sys_write_state_csv((FILE*)ctx, s); }

int main(int argc, char** argv) {
    char path[1024], err[256], line[512];
    ok_cfg cfg;
    ok_sys* sys;
    gpack gp[OK_CFG_MAXCAM];
    uint8_t* img[OK_CFG_MAXCAM];
    unsigned char* voc;
    long nvoc, frames = 0;
    FILE *f, *imu, *causal;
    ok_time start;
    uint32_t i;
    int c, imu_done = 0;
    long max_frames;
    if (argc != 5 && argc != 6) { fprintf(stderr, "okvis_c_euroc <config.yaml> <sequence dir> <vocabulary.bin> <out dir> [max frames]\n"); return 2; }
    max_frames = argc == 6 ? atol(argv[5]) : -1;
    if (ok_cfg_load(argv[1], &cfg, err, sizeof err)) { fprintf(stderr, "%s: %s\n", argv[1], err); return 1; }
    f = fopen(argv[3], "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", argv[3]); return 1; }
    fseek(f, 0, SEEK_END); nvoc = ftell(f); fseek(f, 0, SEEK_SET);
    voc = (unsigned char*)malloc((size_t)nvoc);
    if (fread(voc, 1, (size_t)nvoc, f) != (size_t)nvoc) { fprintf(stderr, "cannot read %s\n", argv[3]); return 1; }
    fclose(f);
    for (c = 0; c < cfg.ncam; ++c) {
        snprintf(path, sizeof path, "%s/gray/cam%d.gray", argv[2], c);
        if (!gpack_open(&gp[c], path) || (int)gp[c].w != cfg.cam[c].w || (int)gp[c].h != cfg.cam[c].h) { fprintf(stderr, "cannot read %s\n", path); return 1; }
        img[c] = (uint8_t*)malloc((size_t)gp[c].w * gp[c].h);
    }
    snprintf(path, sizeof path, "%s/mav0/imu0/data.csv", argv[2]);
    imu = fopen(path, "r");
    if (!imu || !fgets(line, sizeof line, imu)) { fprintf(stderr, "cannot read %s\n", path); return 1; }
    sys = ok_sys_new(&cfg, NULL, NULL, NULL, voc, (size_t)nvoc, err, sizeof err);
    if (!sys) { fprintf(stderr, "%s\n", err); return 1; }
    snprintf(path, sizeof path, "%s/causal.csv", argv[4]);
    causal = fopen(path, "w");
    if (!causal) { fprintf(stderr, "cannot write %s\n", path); return 1; }
    ok_sys_write_csv_header(causal);
    ok_sys_set_publish(sys, on_publish, causal);
    /* DatasetReader: before every image time t, the IMU up to the first measurement later than t + 0.021 s */
    start = ok_time_from_nsec(gp[0].n ? gp[0].ts[0] : 0);
    for (i = 0; i < gp[0].n && !imu_done && (max_frames < 0 || frames < max_frames); ++i) {
        const ok_time t = ok_time_from_nsec(gp[0].ts[i]);
        const unsigned char* imgs[OK_CFG_MAXCAM];
        ok_time t_lim, t_imu;
        ok_time_add(t, ok_duration_from_sec(0.021), &t_lim);
        do {
            char* tok;
            double v[6];
            int j;
            if (!fgets(line, sizeof line, imu)) { imu_done = 1; break; }
            tok = strtok(line, ",");
            t_imu = ok_time_from_nsec(strtoull(tok, NULL, 10));
            for (j = 0; j < 6; ++j) { tok = strtok(NULL, ","); v[j] = (double)strtof(tok ? tok : "0", NULL); }   /* std::stof */
            if (ok_duration_to_nsec(ok_duration_add(ok_time_sub(t_imu, start), ok_duration_from_sec(1.0))) > 0)
                ok_sys_add_imu(sys, t_imu, v + 3, v);
        } while (ok_time_le(t_imu, t_lim));
        if (imu_done) break;
        for (c = 0; c < cfg.ncam; ++c) {
            if (!gpack_read(&gp[c], gp[0].ts[i], img[c])) { fprintf(stderr, "camera %d: no image at %llu\n", c, (unsigned long long)gp[0].ts[i]); return 1; }
            imgs[c] = img[c];
        }
        c = ok_sys_add_frame(sys, t, imgs);
        if (c < 0) { fprintf(stderr, "frame %llu: error %d\n", (unsigned long long)gp[0].ts[i], c); return 1; }
        frames += c;
        if (frames % 500 == 0 && c == 1) fprintf(stderr, "%ld frames\n", frames);
    }
    fclose(causal);
    snprintf(path, sizeof path, "%s/final.csv", argv[4]);
    f = fopen(path, "w");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return 1; }
    ok_sys_write_final_csv(sys, f);
    fclose(f);
    fprintf(stderr, "%ld frames processed, trajectories in %s\n", frames, argv[4]);
    ok_sys_free(sys);
    for (c = 0; c < cfg.ncam; ++c) { free(img[c]); free(gp[c].ts); fclose(gp[c].f); }
    fclose(imu);
    free(voc);
    return 0;
}
