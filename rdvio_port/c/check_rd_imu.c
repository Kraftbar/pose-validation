/* SPDX-License-Identifier: Apache-2.0 */
/* Harness for module M1 (rd_imu.c): replays the dump records of the reference (patch 0003) or of the oracle program
 * (rdvio_port/reference_tools/rd_imu_oracle.cc, same layouts plus incr.bin / sqrt.bin) through the C port and compares BITWISE.
 * usage: check_rd_imu <dir> [integ|pred|pie|plus|incr|sqrt ...]   exit 0 iff no mismatch in the files found. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rd_imu.h"
#include "rd_eigen.h"

static long g_cmp, g_bad;
static int g_report;
static void cmp(const char* what, const double* got, const double* exp, int n, long rec) {
    int i, bad = 0;
    for (i = 0; i < n; ++i) {
        g_cmp++;
        if (memcmp(&got[i], &exp[i], 8)) {
            bad++; g_bad++;
            if (g_report < 12) { g_report++; printf("  MISMATCH rec %ld %s[%d]: got %.17g exp %.17g\n", rec, what, i, got[i], exp[i]); }
        }
    }
    (void)bad;
}
static int rd(FILE* f, void* p, size_t n) { return fread(p, 1, n, f) == n; }
#define RD(p, n) do { if (!rd(f, (p), (n))) goto done; } while (0)

static void read_state(FILE* f, rd_preint* s, int* ok) {
    double q[4];
    *ok = 0;
    if (!rd(f, &s->delta.t, 8) || !rd(f, q, 32) || !rd(f, s->delta.p, 24) || !rd(f, s->delta.v, 24) || !rd(f, s->delta.cov, 1800) ||
        !rd(f, s->delta.sqrt_inv_cov, 1800) || !rd(f, &s->jac, 360))
        return;
    s->delta.q.x = q[0]; s->delta.q.y = q[1]; s->delta.q.z = q[2]; s->delta.q.w = q[3];
    *ok = 1;
}
static void cmp_state(const rd_preint* got, const rd_preint* exp, long rec, int cov, int sq) {
    double qg[4] = {got->delta.q.x, got->delta.q.y, got->delta.q.z, got->delta.q.w};
    double qe[4] = {exp->delta.q.x, exp->delta.q.y, exp->delta.q.z, exp->delta.q.w};
    cmp("delta.t", &got->delta.t, &exp->delta.t, 1, rec);
    cmp("delta.q", qg, qe, 4, rec);
    cmp("delta.p", got->delta.p, exp->delta.p, 3, rec);
    cmp("delta.v", got->delta.v, exp->delta.v, 3, rec);
    if (cov) cmp("delta.cov", got->delta.cov, exp->delta.cov, 225, rec);
    if (sq) cmp("sqrt_inv_cov", got->delta.sqrt_inv_cov, exp->delta.sqrt_inv_cov, 225, rec);
    cmp("jac", (const double*)&got->jac, (const double*)&exp->jac, 45, rec);
}

static void check_integ(const char* dir) {
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/integ.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        uint32_t n, flags, ret; double t, bg[3], ba[3];
        rd_preint pi, ex; rd_imu_sample* d; uint32_t i; int ok;
        RD(&n, 4); RD(&flags, 4); RD(&t, 8); RD(bg, 24); RD(ba, 24);
        RD(pi.cov_w, 72); RD(pi.cov_a, 72); RD(pi.cov_bg, 72); RD(pi.cov_ba, 72);
        d = (rd_imu_sample*)malloc(sizeof(rd_imu_sample) * (n ? n : 1));
        for (i = 0; i < n; ++i) { if (!rd(f, &d[i], 56)) { free(d); goto done; } }
        if (!rd(f, &ret, 4)) { free(d); goto done; }
        ex = pi;
        if (ret) { read_state(f, &ex, &ok); if (!ok) { free(d); goto done; } }
        {
            int r2 = rd_pi_integrate(&pi, d, (int)n, t, bg, ba, flags & 1, (flags >> 1) & 1);
            if (r2 != (int)ret) { printf("  MISMATCH rec %ld ret\n", rec); g_bad++; }
            else if (ret) cmp_state(&pi, &ex, rec, 1, 1);
        }
        free(d); rec++;
    }
done:
    printf("integ: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

static void check_incr(const char* dir) {
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/incr.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        double dt, bg[3], ba[3]; uint32_t flags; rd_imu_sample s; rd_preint pi, ex; int ok;
        RD(&dt, 8); RD(&s, 56); RD(bg, 24); RD(ba, 24); RD(&flags, 4);
        RD(pi.cov_w, 72); RD(pi.cov_a, 72); RD(pi.cov_bg, 72); RD(pi.cov_ba, 72);
        read_state(f, &pi, &ok); if (!ok) goto done;
        ex = pi; read_state(f, &ex, &ok); if (!ok) goto done;
        rd_pi_increment(&pi, dt, &s, bg, ba, flags & 1, (flags >> 1) & 1);
        cmp_state(&pi, &ex, rec, 1, 0);
        rec++;
    }
done:
    printf("incr: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

static void check_sqrt(const char* dir) {
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/sqrt.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        rd_preint pi; double ex[225];
        RD(pi.delta.cov, 1800); RD(ex, 1800);
        rd_pi_compute_sqrt_inv_cov(&pi);
        cmp("sqrt_inv_cov", pi.delta.sqrt_inv_cov, ex, 225, rec);
        rec++;
    }
done:
    printf("sqrt: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

static void check_pred(const char* dir) {
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/pred.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        double oq[4], op[3], ov[3], obg[3], oba[3], dt, dq[4], dp[3], dv[3], nq[4], np[3], nv[3], nbg[3], nba[3];
        rd_preint pi; rd_pose o, n; rd_motion om, nm; double got[4];
        RD(oq, 32); RD(op, 24); RD(ov, 24); RD(obg, 24); RD(oba, 24); RD(&dt, 8); RD(dq, 32); RD(dp, 24); RD(dv, 24);
        RD(nq, 32); RD(np, 24); RD(nv, 24); RD(nbg, 24); RD(nba, 24);
        o.q.x = oq[0]; o.q.y = oq[1]; o.q.z = oq[2]; o.q.w = oq[3]; memcpy(o.p, op, 24);
        memcpy(om.v, ov, 24); memcpy(om.bg, obg, 24); memcpy(om.ba, oba, 24);
        pi.delta.t = dt; pi.delta.q.x = dq[0]; pi.delta.q.y = dq[1]; pi.delta.q.z = dq[2]; pi.delta.q.w = dq[3];
        memcpy(pi.delta.p, dp, 24); memcpy(pi.delta.v, dv, 24);
        rd_pi_predict(&pi, &o, &om, &n, &nm);
        got[0] = n.q.x; got[1] = n.q.y; got[2] = n.q.z; got[3] = n.q.w;
        cmp("q", got, nq, 4, rec); cmp("p", n.p, np, 3, rec); cmp("v", nm.v, nv, 3, rec); cmp("bg", nm.bg, nbg, 3, rec); cmp("ba", nm.ba, nba, 3, rec);
        rec++;
    }
done:
    printf("pred: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

static void check_plus(const char* dir) {
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/plus.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        double q[4], dq[3], ex[4], got[4];
        RD(q, 32); RD(dq, 24); RD(ex, 32);
        rd_quat_plus(q, dq, got);
        cmp("plus", got, ex, 4, rec); rec++;
    }
done:
    printf("plus: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

static void check_pie(const char* dir) {
    static const int SZ[10] = {4, 3, 3, 3, 3, 4, 3, 3, 3, 3};
    char path[512]; FILE* f; long rec = 0, b0 = g_bad, c0 = g_cmp;
    snprintf(path, sizeof path, "%s/pie.bin", dir);
    f = fopen(path, "rb"); if (!f) return;
    for (;;) {
        uint32_t has_jac, mask, via; double prm[32], iq[4], ip[3], jq[4], jp[3], bg0[3], ba0[3], dt, dq[4], dp[3], dv[3], res[15], got_r[15];
        double sic[225]; rd_preint pre; ok_quat qi, qj; const double* pp[10]; double* jj[10]; double expj[10][60], gotj[10][60]; int k, off = 0;
        RD(&has_jac, 4); RD(&mask, 4); RD(&via, 4); RD(prm, 256); RD(iq, 32); RD(ip, 24); RD(jq, 32); RD(jp, 24); RD(bg0, 24); RD(ba0, 24);
        RD(&dt, 8); RD(dq, 32); RD(dp, 24); RD(dv, 24); RD(&pre.jac, 360); RD(sic, 1800); RD(res, 120);
        for (k = 0; k < 10; ++k) if ((has_jac && (mask >> k & 1)) && !rd(f, expj[k], (size_t)15 * SZ[k] * 8)) goto done;
        pre.delta.t = dt; pre.delta.q.x = dq[0]; pre.delta.q.y = dq[1]; pre.delta.q.z = dq[2]; pre.delta.q.w = dq[3];
        memcpy(pre.delta.p, dp, 24); memcpy(pre.delta.v, dv, 24); memcpy(pre.delta.sqrt_inv_cov, sic, 1800);
        qi.x = iq[0]; qi.y = iq[1]; qi.z = iq[2]; qi.w = iq[3]; qj.x = jq[0]; qj.y = jq[1]; qj.z = jq[2]; qj.w = jq[3];
        for (k = 0; k < 10; ++k) { pp[k] = prm + off; off += SZ[k]; jj[k] = (has_jac && (mask >> k & 1)) ? gotj[k] : NULL; }
        rd_pie_eval(&pre, &qi, ip, &qj, jp, bg0, ba0, pp, got_r, has_jac ? jj : NULL);
        cmp("residual", got_r, res, 15, rec);
        for (k = 0; k < 10; ++k) if (jj[k] && has_jac) { char nm[16]; snprintf(nm, sizeof nm, "jac%d", k); cmp(nm, gotj[k], expj[k], 15 * SZ[k], rec); }
        (void)via; rec++;
    }
done:
    printf("pie: %ld records, %ld values compared, %ld mismatches\n", rec, g_cmp - c0, g_bad - b0);
    fclose(f);
}

int main(int argc, char** argv) {
    int i; const char* dir;
    if (argc < 2) { fprintf(stderr, "usage: check_rd_imu <dir> [kinds]\n"); return 2; }
    dir = argv[1];
    if (argc == 2 || strstr(" integ ", " integ ")) { /* default: all */ }
    {
        int all = argc == 2;
        for (i = 2; i < argc || all; ++i) {
            const char* k = all ? "all" : argv[i];
            if (all || !strcmp(k, "integ")) check_integ(dir);
            if (all || !strcmp(k, "pred")) check_pred(dir);
            if (all || !strcmp(k, "pie")) check_pie(dir);
            if (all || !strcmp(k, "plus")) check_plus(dir);
            if (all || !strcmp(k, "incr")) check_incr(dir);
            if (all || !strcmp(k, "sqrt")) check_sqrt(dir);
            if (all) break;
        }
    }
    printf("TOTAL: %ld values compared, %ld mismatches\n", g_cmp, g_bad);
    return g_bad ? 1 : 0;
}
