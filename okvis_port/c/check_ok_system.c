/* OK_PORT_SOURCES: check_ok_system.c ok_system.c ok_config.c ok_brisk_detector.c ok_brisk_descriptor.c ok_brisk_camera.c ok_dbow.c ok_place.c ok_place_dist.c ok_opengv.c ok_opengv_gp3p_gen.c ok_opengv_stew_gen.c ok_eigen_eigsolver8.c ok_eigen_eigsolver10.c ok_eigen_cx.c ok_eigen_svd.c ok_eigen_qr.c ok_eigen_fullpivlu.c ok_frontend.c ok_vslam.c ok_vsb_geom.c ok_vsolve.c ok_solve.c ok_solve_linear.c ok_sparse.c ok_amd.c ok_vigraph.c ok_problem.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_triangulate.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_dense.c ok_blas.c */
/* End-to-end harness for okvis_port module 8 (the system driver, ok_system.c): the whole C pipeline runs from the dataset
 * (EuRoC imu0/data.csv + the images, decoded once by tools/okvis_port_images.py into gray/cam<i>.gray) and the OKVIS2 YAML
 * config of the reference run (read from the first line of <run>/log.txt), with BRISK, the frontend, RANSAC, place
 * recognition and the backend all native. Nothing of the log is fed in: the log only CHECKS.
 *
 *   check_ok_system <seq_label> <fixtures_dir (unused, "-")> <dump_dir>        env OK_SYSTEM_DATA=<dir with gray/ and
 *                                                                                mav0/imu0/data.csv>, OK_NATIVE_SOLVE=1
 *
 * Checked against the reference log of <dump_dir> (problem.bin and its add-on files, see check_ok_frontend.c): every
 * ViSlamBackend call ThreadedSlam makes (addImu, addCamera, addStates with the IMU deque and the BRISK keypoints of the
 * multiframe, setKeyframe, optimiseRealtimeGraph, synchroniseRealtimeAndFullGraph, applyStrategy, optimiseFullGraph; tag
 * and argument bytes, results), the BRISK descriptors (record 161), every backend call of the frontend and every graph /
 * Problem record they cause (as in check_ok_frontend), and finally the two trajectory files of the reference run,
 * <run>/causal.csv (TrajectoryOutput) and <run>/final.csv (writeFinalCsvTrajectory), byte for byte, row by row.
 * With OK_NATIVE_SOLVE=1 every graph solve is native, so no logged value enters the computation.
 * Last line: "<seq_label>: <mismatches>/<total>".
 */
#define OK_FRONTEND_AS_LIB
#include "check_ok_frontend.c"
#undef main
#include "ok_system.h"

static cnt C_sys_desc, C_sys_traj, C_sys_end;
static long G_sys_frames, G_sys_dropped, G_sys_imu, G_sys_causal_rows, G_sys_final_rows;
static unsigned char* G_sys_desc[OK_FE_MAXCAM]; static size_t G_sys_ndesc[OK_FE_MAXCAM]; static int G_sys_have_desc;

/* ---- ThreadedSlam's backend calls: compare with the next logged entry record, then execute ---- */
static void ob_f64(obuf* o, double v) { ob_raw(o, &v, 8); }
static void ob_time(obuf* o, ok_time t) { ob_u32(o, t.sec); ob_u32(o, t.nsec); }
static int s_add_imu(void* ctx, const ok_vg_imu_cfg* c) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)c->use); ob_f64n(&o, c->T_BS, 7);
    ob_f64(&o, c->a_max); ob_f64(&o, c->g_max); ob_f64(&o, c->sigma_g_c); ob_f64(&o, c->sigma_bg); ob_f64(&o, c->sigma_a_c);
    ob_f64(&o, c->sigma_ba); ob_f64(&o, c->sigma_gw_c); ob_f64(&o, c->sigma_aw_c); ob_f64n(&o, c->g0, 3); ob_f64n(&o, c->a0, 3); ob_f64(&o, c->g);
    fe_expect(OK_B_ADDIMU, &o); free(o.p);
    G_imu_cfg = *c;
    return ok_vsb_add_imu(V_b, c);
}
static int s_add_camera(void* ctx, int d, double sr, double sa) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)d); ob_f64(&o, sr); ob_f64(&o, sa);
    fe_expect(OK_B_ADDCAM, &o); free(o.p);
    return ok_vsb_add_camera(V_b, d, sr, sa);
}
static int s_add_states(void* ctx, ok_time t, const ok_imu_meas* m, size_t n, int kf, double kr, int nc, const ok_vsb_cam_in* cams) {
    obuf o; size_t i; int c, k, r; (void)ctx; memset(&o, 0, sizeof o);
    ob_time(&o, t);
    ob_u64(&o, (uint64_t)n);
    for (i = 0; i < n; ++i) { ob_time(&o, m[i].t); ob_f64n(&o, m[i].gyr, 3); ob_f64n(&o, m[i].acc, 3); }
    ob_u32(&o, (uint32_t)kf); ob_f64(&o, kr);
    ob_u32(&o, (uint32_t)nc);
    for (c = 0; c < nc; ++c) {
        ob_u32(&o, (uint32_t)cams[c].hlen); ob_raw(&o, cams[c].header, cams[c].hlen);
        ob_f64n(&o, cams[c].T_SC, 7);
        ob_u32(&o, (uint32_t)cams[c].rows); ob_u32(&o, (uint32_t)cams[c].cols); ob_u32(&o, (uint32_t)cams[c].nkp);
        ob_raw(&o, cams[c].kp, 12 * (size_t)cams[c].nkp);
        ob_u32(&o, (uint32_t)cams[c].nz);
        for (k = 0; k < cams[c].nz; ++k) { ob_u32(&o, cams[c].nz_kp[k]); ob_u64(&o, cams[c].nz_id[k]); }
    }
    fe_expect(OK_B_ADDSTATES, &o); free(o.p);
    r = ok_vsb_add_states(V_b, t, m, n, kf, kr, nc, cams);
    G_cur_frame = ok_vsb_current_state_id(V_b);
    return r;
}
static int s_set_keyframe(void* ctx, uint64_t id, int fl) {
    obuf o; int c; (void)ctx; memset(&o, 0, sizeof o);
    /* the descriptors of this frame (record 161 follows the ADDSTATES record and has been read by now) */
    if (G_sys_have_desc) {
        chk(&C_sys_desc, G_desc.valid && G_desc.ncam == ok_vsb_frame(V_b, id)->ncam);
        for (c = 0; c < G_desc.ncam && c < OK_FE_MAXCAM; ++c) {
            size_t k;
            chk(&C_sys_desc, (size_t)G_desc.nkp[c] == G_sys_ndesc[c]);
            if ((size_t)G_desc.nkp[c] == G_sys_ndesc[c]) for (k = 0; k < 48 * G_sys_ndesc[c]; ++k) chk(&C_sys_desc, G_desc.desc[c][k] == G_sys_desc[c][k]);
        }
        { static int shown; if (!shown && C_sys_desc.bad) { shown = 1; DBG("first BRISK descriptor mismatch at frame %llu", (unsigned long long)id); } }
        G_sys_have_desc = 0;
    }
    ob_u64(&o, id); ob_u32(&o, (uint32_t)fl);
    fe_expect(OK_B_SETKF, &o); free(o.p);
    G_fe_frames++;
    return ok_vsb_set_keyframe(V_b, id, fl);
}
static int s_optimise_realtime(void* ctx, int ni, int nt, int vb, int on, int ii, uint64_t** up, int* nu) {
    obuf o; int r; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)ni); ob_u32(&o, (uint32_t)nt); ob_u32(&o, (uint32_t)vb); ob_u32(&o, (uint32_t)on); ob_u32(&o, (uint32_t)ii);
    fe_expect(OK_B_OPTRT, &o); free(o.p);
    r = ok_vsb_optimise_realtime(V_b, ni, nt, vb, on, ii, up, nu);
    res_ids(OK_B_OPTRT, *up, *nu);
    return r;
}
static int s_synchronise(void* ctx, uint64_t** up, int* nu) {
    obuf e; int r; (void)ctx; memset(&e, 0, sizeof e);
    fe_expect(OK_B_SYNC, &e); free(e.p);
    r = ok_vsb_synchronise(V_b, up, nu);
    res_ids(OK_B_SYNC, *up, *nu);
    return r;
}
static int s_apply_strategy(void* ctx, size_t k, size_t l, size_t i, int ex, uint64_t** a, int* na) {
    obuf o; int r; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, (uint64_t)k); ob_u64(&o, (uint64_t)l); ob_u64(&o, (uint64_t)i); ob_u32(&o, (uint32_t)ex);
    fe_expect(OK_B_APPLYSTRATEGY, &o); free(o.p);
    r = ok_vsb_apply_strategy(V_b, k, l, i, ex, a, na);
    res_ids(OK_B_APPLYSTRATEGY, *a, *na);
    return r;
}
static int s_optimise_full(void* ctx, int ni, int nt, int vb) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)ni); ob_u32(&o, (uint32_t)nt); ob_u32(&o, (uint32_t)vb);
    fe_expect(OK_B_OPTFULL, &o); free(o.p);
    return ok_vsb_optimise_full(V_b, ni, nt, vb);
}
static void s_on_features(void* ctx, ok_time t, int cam, size_t n, const float* kp, const unsigned char* desc) {
    (void)ctx; (void)t; (void)kp;
    if (cam >= OK_FE_MAXCAM) return;
    free(G_sys_desc[cam]);
    G_sys_desc[cam] = (unsigned char*)malloc(48 * (n ? n : 1));
    memcpy(G_sys_desc[cam], desc, 48 * n);
    G_sys_ndesc[cam] = n;
    G_sys_have_desc = 1;
    G_br_kp += (long)n;
}

/* ---- dataset ---- */
static void on_publish(void* ctx, const ok_sys_state* st) { ok_sys_write_state_csv((FILE*)ctx, st); G_sys_causal_rows++; }

/* compare two text files line by line (one value per row) */
static long cmp_files(cnt* c, FILE* mine, const char* ref_path, const char* what) {
    FILE* r = fopen(ref_path, "rb");
    char a[2048], b[2048];
    long row = 0;
    if (!r) { chk(c, 0); DBG("cannot open %s", ref_path); return 0; }
    rewind(mine);
    for (;;) {
        char* pa = fgets(a, sizeof a, mine);
        char* pb = fgets(b, sizeof b, r);
        if (!pa && !pb) break;
        row++;
        chk(c, pa && pb && strcmp(a, b) == 0);
        if (!(pa && pb && strcmp(a, b) == 0)) {
            static int shown;
            if (shown++ < 3) DBG("%s row %ld differs:\n      C:   %s      ref: %s", what, row, pa ? pa : "(missing)\n", pb ? pb : "(missing)\n");
        }
    }
    fclose(r);
    return row;
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "system";
    const char* dir = argc > 3 ? argv[3] : ".";
    const char* data = getenv("OK_SYSTEM_DATA");
    char path[1024], cfg_path[1024], err[256], line[512];
    ok_fe_params fp;
    ok_fe_est est;
    ok_sys_be be;
    ok_cfg cfg;
    ok_sys* sys;
    gpack gp[OK_CFG_MAXCAM];
    FILE *lf, *imu, *causal, *fin;
    uint8_t* img[OK_CFG_MAXCAM];
    uint32_t i;
    int c, rc, imu_done = 0;
    ok_time start; int have_start = 0;
    lrec r;

    if (!data) { fprintf(stderr, "set OK_SYSTEM_DATA=<dir with gray/ and mav0/imu0/data.csv> (tools/okvis_port_images.py)\n"); printf("%s: 0/0\n", label); return 1; }
    /* the reference run's config: first line of <run>/log.txt */
    snprintf(path, sizeof path, "%s/../log.txt", dir);
    lf = fopen(path, "r");
    cfg_path[0] = 0;
    if (lf) {
        while (fgets(line, sizeof line, lf)) {
            char* p = strstr(line, "Opened configuration file: ");
            if (p) { size_t n; snprintf(cfg_path, sizeof cfg_path, "%s", p + 27); n = strlen(cfg_path); while (n && (cfg_path[n - 1] == '\n' || cfg_path[n - 1] == '\r')) cfg_path[--n] = 0; break; }
        }
        fclose(lf);
    }
    if (!cfg_path[0] || ok_cfg_load(cfg_path, &cfg, err, sizeof err)) { fprintf(stderr, "config: %s (%s)\n", cfg_path, err); printf("%s: 0/0\n", label); return 1; }
    memset(gp, 0, sizeof gp);
    for (c = 0; c < cfg.ncam; ++c) {
        snprintf(path, sizeof path, "%s/gray/cam%d.gray", data, c);
        if (!gpack_open(&gp[c], path) || (int)gp[c].w != cfg.cam[c].w || (int)gp[c].h != cfg.cam[c].h) { fprintf(stderr, "cannot read %s\n", path); printf("%s: 0/0\n", label); return 1; }
        img[c] = (uint8_t*)malloc((size_t)gp[c].w * gp[c].h);
    }
    snprintf(path, sizeof path, "%s/mav0/imu0/data.csv", data);
    imu = fopen(path, "r");
    if (!imu || !fgets(line, sizeof line, imu)) { fprintf(stderr, "cannot read %s\n", path); printf("%s: 0/0\n", label); return 1; }

    rc = fe_setup(label, dir, &est, &fp);
    if (rc) return rc < 0 ? 1 : rc;
    if (!G_place_native) { fprintf(stderr, "place.bin missing in %s (the native place recognition needs its vocabulary)\n", dir); printf("%s: 0/0\n", label); return 1; }
    memset(&be, 0, sizeof be);
    be.add_imu = s_add_imu; be.add_camera = s_add_camera; be.add_states = s_add_states; be.set_keyframe = s_set_keyframe;
    be.optimise_realtime = s_optimise_realtime; be.synchronise = s_synchronise; be.apply_strategy = s_apply_strategy;
    be.optimise_full = s_optimise_full; be.on_features = s_on_features;
    sys = ok_sys_new(&cfg, V_b, &est, &be, G_voc, G_voc_len, err, sizeof err);
    if (!sys) { fprintf(stderr, "ok_sys_new: %s\n", err); printf("%s: 0/0\n", label); return 1; }
    causal = tmpfile(); fin = tmpfile();
    ok_sys_write_csv_header(causal);
    ok_sys_set_publish(sys, on_publish, causal);

    /* DatasetReader::processing: images in time order (the synchronised cameras together); before each, the IMU up to and
     * including the first measurement later than t + 0.021 s (measurements older than start - 1 s are not delivered) */
    for (i = 0; i < gp[0].n && !imu_done; ++i) {
        const ok_time t = ok_time_from_nsec(gp[0].ts[i]);
        ok_time t_lim;
        const unsigned char* imgs[OK_CFG_MAXCAM];
        ok_time t_imu;
        int ret;
        ok_time_add(t, ok_duration_from_sec(0.021), &t_lim);              /* t + Duration(0.021) */
        if (!have_start) { start = t; have_start = 1; }
        do {
            char* tok;
            unsigned long long ns;
            double v[6];
            int j;
            if (!fgets(line, sizeof line, imu)) { imu_done = 1; break; }
            tok = strtok(line, ",");
            ns = strtoull(tok, NULL, 10);
            for (j = 0; j < 6; ++j) { tok = strtok(NULL, ","); v[j] = (double)strtof(tok ? tok : "0", NULL); }   /* std::stof */
            t_imu = ok_time_from_nsec(ns);
            if (ok_duration_to_nsec(ok_duration_add(ok_time_sub(t_imu, start), ok_duration_from_sec(1.0))) > 0) {
                ok_sys_add_imu(sys, t_imu, v + 3, v);   /* columns: w_x w_y w_z a_x a_y a_z */
                G_sys_imu++;
            }
        } while (ok_time_le(t_imu, t_lim));
        if (imu_done) break;
        for (c = 0; c < cfg.ncam; ++c) {
            if (!gpack_read(&gp[c], gp[0].ts[i], img[c])) { chk(&C_sys_end, 0); DBG("camera %d has no image at %llu", c, (unsigned long long)gp[0].ts[i]); }
            imgs[c] = img[c];
        }
        ret = ok_sys_add_frame(sys, t, imgs);
        if (ret == 1) G_sys_frames++;
        else if (ret == 0) G_sys_dropped++;
        else { chk(&C_sys_end, 0); DBG("ok_sys_add_frame returned %d at %llu", ret, (unsigned long long)gp[0].ts[i]); break; }
    }
    /* nothing of the log may be left over */
    while (get_rec(&r)) {
        chk(&C_sys_end, r.tag < 128);
        if (r.tag >= 128) { static int shown; if (!shown++) DBG("log record %u was not made by the C system", r.tag); }
        lrec_free(&r);
    }
    chk(&C_sys_end, 1);
    fclose(V_f);
    ok_sys_write_final_csv(sys, fin);
    snprintf(path, sizeof path, "%s/../causal.csv", dir);
    G_sys_causal_rows = cmp_files(&C_sys_traj, causal, path, "causal.csv");
    snprintf(path, sizeof path, "%s/../final.csv", dir);
    G_sys_final_rows = cmp_files(&C_sys_traj, fin, path, "final.csv");
    printf("  C system run from the images and imu0/data.csv (%s): %ld frames processed, %ld dropped at startup, %ld IMU measurements; "
           "trajectory rows compared: causal.csv %ld, final.csv %ld\n", cfg_path, G_sys_frames, G_sys_dropped, G_sys_imu, G_sys_causal_rows, G_sys_final_rows);
    PK2("system: BRISK descriptors", C_sys_desc); PK2("system: trajectory csv rows", C_sys_traj); PK2("system: log consumed", C_sys_end);
    G_extra_bad = C_sys_desc.bad + C_sys_traj.bad + C_sys_end.bad;
    G_extra_tot = C_sys_desc.tot + C_sys_traj.tot + C_sys_end.tot;
    ok_sys_free(sys);
    V_fe = NULL;
    return fe_report(label);
}
