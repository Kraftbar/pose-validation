/* SPDX-License-Identifier: Apache-2.0 */
/* Harness for module M2 (rd_factor.c, rd_geom.c): replays the dump records of the reference (patch 0004) or of the oracle program
 * (rdvio_port/reference_tools/rd_m2_oracle.cc, same layouts) through the C port and compares BITWISE.
 * usage: check_rd_m2 <dir> [rpe|rot|ess|hom|wahba|decess|dechom|tri2|trin|tritrack ...]   exit 0 iff no mismatch in the files found. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rd_factor.h"
#include "rd_geom.h"

static long g_cmp, g_bad;
static int g_report;
static void cmp(const char* what, const double* got, const double* exp, int n, long rec) {
    int i;
    for (i = 0; i < n; ++i) {
        g_cmp++;
        if (memcmp(&got[i], &exp[i], 8)) {
            g_bad++;
            if (g_report < 16) { g_report++; printf("  MISMATCH rec %ld %s[%d]: got %.17g exp %.17g\n", rec, what, i, got[i], exp[i]); }
        }
    }
}
static int rd(FILE* f, void* p, size_t n) { return fread(p, 1, n, f) == n; }
#define RD(p, n) do { if (!rd(f, (p), (n))) goto done; } while (0)
static FILE* open_kind(const char* dir, const char* name) { char path[512]; snprintf(path, sizeof path, "%s/%s.bin", dir, name); return fopen(path, "rb"); }
#define REPORT(name) printf("%s: %ld records, %ld values compared, %ld mismatches\n", name, rec, g_cmp - c0, g_bad - b0)

static void read_ext(FILE* f, rd_extrinsic* e, int* ok) {
    double q[4];
    *ok = rd(f, q, 32) && rd(f, e->p_cs, 24);
    e->q_cs.x = q[0]; e->q_cs.y = q[1]; e->q_cs.z = q[2]; e->q_cs.w = q[3];
}

static void check_rpe(const char* dir) {
    static const int SZ[5] = {4, 3, 4, 3, 1};
    FILE* f = open_kind(dir, "rpe"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t has_jac, mask, via; double P[15], z[3], zr[3], sic[4], res[2], got_r[2], expj[5][8], gotj[5][8];
        rd_extrinsic cr, ct; const double* pp[5]; double* jj[5]; int k, ok; const int o[5] = {0, 4, 7, 11, 14};
        RD(&has_jac, 4); RD(&mask, 4); RD(&via, 4); RD(P, 120); RD(z, 24); RD(zr, 24);
        read_ext(f, &cr, &ok); if (!ok) goto done;
        read_ext(f, &ct, &ok); if (!ok) goto done;
        RD(sic, 32); RD(res, 16);
        for (k = 0; k < 5; ++k) if (has_jac && (mask >> k & 1) && !rd(f, expj[k], (size_t)2 * SZ[k] * 8)) goto done;
        for (k = 0; k < 5; ++k) { pp[k] = P + o[k]; jj[k] = (has_jac && (mask >> k & 1)) ? gotj[k] : NULL; }
        rd_rpe_eval(z, zr, &cr, &ct, sic, pp, got_r, has_jac ? jj : NULL);
        cmp("residual", got_r, res, 2, rec);
        for (k = 0; k < 5; ++k) if (jj[k]) { char nm[16]; snprintf(nm, sizeof nm, "jac%d", k); cmp(nm, gotj[k], expj[k], 2 * SZ[k], rec); }
        rec++;
    }
done:
    REPORT("rpe");
    fclose(f);
}

static void check_rot(const char* dir) {
    FILE* f = open_kind(dir, "rot"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t has_jac; double qt[4], qr[4], z[3], zr[3], sic[4], res[2], got_r[2], ej[8], gj[8]; rd_extrinsic cr, ct; ok_quat qrc; int ok;
        RD(&has_jac, 4); RD(qt, 32); RD(qr, 32); RD(z, 24); RD(zr, 24);
        read_ext(f, &cr, &ok); if (!ok) goto done;
        read_ext(f, &ct, &ok); if (!ok) goto done;
        RD(sic, 32); RD(res, 16);
        if (has_jac) RD(ej, 64);
        qrc.x = qr[0]; qrc.y = qr[1]; qrc.z = qr[2]; qrc.w = qr[3];
        rd_rot_prior_eval(z, zr, &cr, &ct, sic, &qrc, qt, got_r, has_jac ? gj : NULL);
        cmp("residual", got_r, res, 2, rec);
        if (has_jac) cmp("jac", gj, ej, 8, rec);
        rec++;
    }
done:
    REPORT("rot");
    fclose(f);
}


static void check_wahba(const char* dir) {
    FILE* f = open_kind(dir, "wahba"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        double p1[2][3], p2[2][3], ex[9], got[9];
        RD(p1, 48); RD(p2, 48); RD(ex, 72);
        rd_solve_rotation_2pt((const double (*)[3])p1, (const double (*)[3])p2, got);
        cmp("R", got, ex, 9, rec); rec++;
    }
done:
    REPORT("wahba");
    fclose(f);
}

static void check_ess5(const char* dir) {
    FILE* f = open_kind(dir, "ess5"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        double p1[5][2], p2[5][2], ex[10][9], got[10][9]; uint32_t cnt; int n;
        RD(p1, 80); RD(p2, 80); RD(&cnt, 4);
        if (cnt > 10 || !rd(f, ex, (size_t)cnt * 72)) goto done;
        n = rd_solve_essential_5pt((const double (*)[2])p1, (const double (*)[2])p2, got);
        if (n != (int)cnt) { g_bad++; if (g_report < 16) { g_report++; printf("  MISMATCH rec %ld solution count got %d exp %u\n", rec, n, cnt); } }
        else cmp("E", &got[0][0], &ex[0][0], 9 * n, rec);
        rec++;
    }
done:
    REPORT("ess5");
    fclose(f);
}

static void check_hom4(const char* dir) {
    FILE* f = open_kind(dir, "hom4"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        double p1[4][2], p2[4][2], ex[9], got[9];
        RD(p1, 64); RD(p2, 64); RD(ex, 72);
        rd_solve_homography_4pt((const double (*)[2])p1, (const double (*)[2])p2, got);
        cmp("H", got, ex, 9, rec); rec++;
    }
done:
    REPORT("hom4");
    fclose(f);
}

static void check_decess(const char* dir) {
    FILE* f = open_kind(dir, "decess"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        double E[9], e1[9], e2[9], et[3], r1[9], r2[9], t[3];
        RD(E, 72); RD(e1, 72); RD(e2, 72); RD(et, 24);
        rd_decompose_essential(E, r1, r2, t);
        cmp("R1", r1, e1, 9, rec); cmp("R2", r2, e2, 9, rec); cmp("T", t, et, 3, rec); rec++;
    }
done:
    REPORT("decess");
    fclose(f);
}

static void check_dechom(const char* dir) {
    FILE* f = open_kind(dir, "dechom"); long rec = 0, b0 = g_bad, c0 = g_cmp, nrot = 0;
    if (!f) return;
    for (;;) {
        double H[9], e1[9], e2[9], et1[3], et2[3], en1[3], en2[3], r1[9], r2[9], t1[3], t2[3], n1[3], n2[3]; uint32_t ret; int got;
        RD(H, 72); RD(&ret, 4); RD(e1, 72); RD(e2, 72); RD(et1, 24); RD(et2, 24); RD(en1, 24); RD(en2, 24);
        got = rd_decompose_homography(H, r1, r2, t1, t2, n1, n2);
        if (got != (int)ret) { g_bad++; printf("  MISMATCH rec %ld ret\n", rec); }
        else if (!ret) nrot++;
        cmp("R1", r1, e1, 9, rec); cmp("R2", r2, e2, 9, rec); cmp("T1", t1, et1, 3, rec); cmp("T2", t2, et2, 3, rec); cmp("n1", n1, en1, 3, rec); cmp("n2", n2, en2, 3, rec);
        rec++;
    }
done:
    REPORT("dechom"); printf("  (pure-rotation branch records: %ld)\n", nrot);
    fclose(f);
}

static void check_tri2(const char* dir) {
    FILE* f = open_kind(dir, "tri2"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        double P1[12], P2[12], a[3], b[3], ex[4], got[4];
        RD(P1, 96); RD(P2, 96); RD(a, 24); RD(b, 24); RD(ex, 32);
        rd_triangulate_point2(P1, P2, a, b, got);
        cmp("X", got, ex, 4, rec); rec++;
    }
done:
    REPORT("tri2");
    fclose(f);
}

static void check_trin(const char* dir) {
    FILE* f = open_kind(dir, "trin"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t n; double *Ps, *pts, ex[4], got[4];
        RD(&n, 4);
        if (n > 4096) goto done;
        Ps = (double*)malloc(96 * (size_t)n + 8); pts = (double*)malloc(24 * (size_t)n + 8);
        if (!rd(f, Ps, 96 * (size_t)n) || !rd(f, pts, 24 * (size_t)n) || !rd(f, ex, 32)) { free(Ps); free(pts); goto done; }
        rd_triangulate_point_n((int)n, Ps, pts, got);
        cmp("X", got, ex, 4, rec); rec++;
        free(Ps); free(pts);
    }
done:
    REPORT("trin");
    fclose(f);
}

static void check_trk(const char* dir) {
    FILE* f = open_kind(dir, "trk"); long rec = 0, b0 = g_bad, c0 = g_cmp, nvalid = 0;
    if (!f) return;
    for (;;) {
        uint32_t n, valid; rd_obs* o; double ex[3], got[3]; uint32_t i; int gv;
        RD(&n, 4);
        if (n > 4096) goto done;
        o = (rd_obs*)malloc(sizeof(rd_obs) * (n ? n : 1));
        for (i = 0; i < n; ++i) {
            double q[4], c[4];
            if (!rd(f, q, 32) || !rd(f, o[i].pose_p, 24) || !rd(f, c, 32) || !rd(f, o[i].cam_p, 24) || !rd(f, o[i].keypoint, 24)) { free(o); goto done; }
            o[i].pose_q.x = q[0]; o[i].pose_q.y = q[1]; o[i].pose_q.z = q[2]; o[i].pose_q.w = q[3];
            o[i].cam_q.x = c[0]; o[i].cam_q.y = c[1]; o[i].cam_q.z = c[2]; o[i].cam_q.w = c[3];
        }
        if (!rd(f, &valid, 4) || (valid && !rd(f, ex, 24))) { free(o); goto done; }
        gv = rd_track_triangulate((int)n, o, got);
        if (gv != (int)valid) { g_bad++; if (g_report < 16) { g_report++; printf("  MISMATCH rec %ld validity got %d exp %u\n", rec, gv, valid); } }
        else if (valid) { nvalid++; cmp("landmark", got, ex, 3, rec); }
        rec++; free(o);
    }
done:
    REPORT("trk"); printf("  (valid: %ld)\n", nvalid);
    fclose(f);
}

static int read_obs(FILE* f, rd_obs* o) {
    double q[4], c[4];
    if (!rd(f, q, 32) || !rd(f, o->pose_p, 24) || !rd(f, c, 32) || !rd(f, o->cam_p, 24) || !rd(f, o->keypoint, 24)) return 0;
    o->pose_q.x = q[0]; o->pose_q.y = q[1]; o->pose_q.z = q[2]; o->pose_q.w = q[3];
    o->cam_q.x = c[0]; o->cam_q.y = c[1]; o->cam_q.z = c[2]; o->cam_q.w = c[3];
    return 1;
}

static void check_tang(const char* dir) {
    FILE* f = open_kind(dir, "tang"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t n, i; rd_obs* o; double p[3], ang, ga;
        RD(&n, 4);
        if (n == 0 || n > 4096) goto done;
        o = (rd_obs*)malloc(sizeof(rd_obs) * n);
        for (i = 0; i < n; ++i) if (!read_obs(f, &o[i])) { free(o); goto done; }
        if (!rd(f, p, 24) || !rd(f, &ang, 8)) { free(o); goto done; }
        ga = rd_track_triangulation_angle((int)n, o, p);
        cmp("angle", &ga, &ang, 1, rec);
        rec++; free(o);
    }
done:
    REPORT("tang");
    fclose(f);
}

static void check_glp(const char* dir) {
    FILE* f = open_kind(dir, "glp"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        rd_obs o; double invd, ex[3], got[3];
        if (!read_obs(f, &o)) goto done;
        RD(&invd, 8); RD(ex, 24);
        rd_track_get_landmark_point(&o, invd, got);
        cmp("landmark", got, ex, 3, rec); rec++;
    }
done:
    REPORT("glp");
    fclose(f);
}

static void check_slp(const char* dir) {
    FILE* f = open_kind(dir, "slp"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        rd_obs o; double p[3], ex, got;
        if (!read_obs(f, &o)) goto done;
        RD(p, 24); RD(&ex, 8);
        got = rd_track_set_landmark_point(&o, p);
        cmp("inv_depth", &got, &ex, 1, rec); rec++;
    }
done:
    REPORT("slp");
    fclose(f);
}

int main(int argc, char** argv) {
    int i; const char* dir;
    if (argc < 2) { fprintf(stderr, "usage: check_rd_m2 <dir> [kinds]\n"); return 2; }
    dir = argv[1];
    {
        int all = argc == 2;
        for (i = 2; i < argc || all; ++i) {
            const char* k = all ? "all" : argv[i];
            if (all || !strcmp(k, "rpe")) check_rpe(dir);
            if (all || !strcmp(k, "rot")) check_rot(dir);
            if (all || !strcmp(k, "wahba")) check_wahba(dir);
            if (all || !strcmp(k, "ess5")) check_ess5(dir);
            if (all || !strcmp(k, "hom4")) check_hom4(dir);
            if (all || !strcmp(k, "decess")) check_decess(dir);
            if (all || !strcmp(k, "dechom")) check_dechom(dir);
            if (all || !strcmp(k, "tri2")) check_tri2(dir);
            if (all || !strcmp(k, "trin")) check_trin(dir);
            if (all || !strcmp(k, "trk")) check_trk(dir);
            if (all || !strcmp(k, "tang")) check_tang(dir);
            if (all || !strcmp(k, "glp")) check_glp(dir);
            if (all || !strcmp(k, "slp")) check_slp(dir);
            if (all) break;
        }
    }
    printf("TOTAL: %ld values compared, %ld mismatches\n", g_cmp, g_bad);
    return g_bad ? 1 : 0;
}
