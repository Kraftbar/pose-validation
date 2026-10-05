/* SPDX-License-Identifier: Apache-2.0 */
/* Harness for module M5 (rd_marg.c, rd_seig.c): replays marg.bin (CeresMarginalizationFactor::marginalize inputs and outputs, dump patch 0007 /
 * oracle run) and eval.bin (random CeresMarginalizationFactor::Evaluate calls, oracle only) through the C port and compares BITWISE
 * (memcmp of doubles, tolerance 0). Layouts: rd_marg.h.
 * usage: check_rd_m5 <dir> [marg|eval]   exit 0 iff no mismatch in the files found. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rd_marg.h"

static long g_cmp, g_bad;
static int g_report;
static void cmp(const char* what, const double* got, const double* exp, long n, long rec) {
    long i;
    for (i = 0; i < n; ++i) {
        g_cmp++;
        if (memcmp(&got[i], &exp[i], 8)) {
            g_bad++;
            if (g_report < 24) { g_report++; printf("  MISMATCH rec %ld %s[%ld]: got %.17g exp %.17g\n", rec, what, i, got[i], exp[i]); }
        }
    }
}
static int rd(FILE* f, void* p, size_t n) { return n == 0 || fread(p, 1, n, f) == n; }
#define RD(p, n) do { if (!rd(f, (p), (n))) goto bad; } while (0)
static FILE* open_kind(const char* dir, const char* name) { char path[512]; snprintf(path, sizeof path, "%s/%s.bin", dir, name); return fopen(path, "rb"); }
static double* dz(size_t n) { return (double*)calloc(n ? n : 1, sizeof(double)); }

static int rd_pose_motion(FILE* f, rd_pose* p, rd_motion* m) {
    double v[16];
    if (!rd(f, v, 128)) return 0;
    p->q.x = v[0]; p->q.y = v[1]; p->q.z = v[2]; p->q.w = v[3];
    memcpy(p->p, v + 4, 24); memcpy(m->v, v + 7, 24); memcpy(m->bg, v + 10, 24); memcpy(m->ba, v + 13, 24);
    return 1;
}
static void flat(const rd_pose* p, const rd_motion* m, double v[16]) {
    v[0] = p->q.x; v[1] = p->q.y; v[2] = p->q.z; v[3] = p->q.w;
    memcpy(v + 4, p->p, 24); memcpy(v + 7, m->v, 24); memcpy(v + 10, m->bg, 24); memcpy(v + 13, m->ba, 24);
}

/* ---- marg.bin ---- */
static void check_marg(const char* dir) {
    FILE* f = open_kind(dir, "marg");
    long rec = 0, full = 0, c0 = g_cmp, b0 = g_bad, ntrk = 0, nobs = 0, maxn = 0, cut = 0;
    long c_lin = 0, c_sic = 0, c_iv = 0, c_im = 0, c_ib = 0, c_ev = 0, c_ve = 0, b_lin = 0, b_sic = 0, b_iv = 0, b_im = 0, b_ib = 0, b_ev = 0, b_ve = 0;
    long hist[16] = {0}, nonconv = 0;
    if (!f) return;
    for (;;) {
        uint32_t nf, index, flags, nff, ntracks, nff2, i, j;
        rd_marg m, ref;
        rd_marg_frame* fr = NULL;
        rd_preint* pre = NULL;
        rd_marg_track* tr = NULL;
        rd_marg_obs** obs = NULL;
        rd_marg_dbg dbg;
        int* mpos = NULL;
        double* eb = NULL;
        int ret;
        long N, N2;
        long cb;
        memset(&m, 0, sizeof m); memset(&ref, 0, sizeof ref); memset(&dbg, 0, sizeof dbg);
        if (fread(&nf, 4, 1, f) != 1) break;   /* clean EOF */
        RD(&index, 4); RD(&flags, 4); RD(&nff, 4);
        m.nf = (int)nff;
        N = (long)nff * 15;
        m.ids = (uint64_t*)calloc(nff ? nff : 1, 8); m.lin_pose = (rd_pose*)calloc(nff ? nff : 1, sizeof(rd_pose)); m.lin_motion = (rd_motion*)calloc(nff ? nff : 1, sizeof(rd_motion));
        m.sqrt_inv_cov = dz((size_t)(N * N)); m.infovec = dz((size_t)N);
        mpos = (int*)calloc(nff ? nff : 1, sizeof(int));
        fr = (rd_marg_frame*)calloc(nf ? nf : 1, sizeof(rd_marg_frame));
        pre = (rd_preint*)calloc(nf ? nf : 1, sizeof(rd_preint));
        for (i = 0; i < nff; ++i) {
            int32_t pos;
            RD(&pos, 4);
            mpos[i] = pos;
            if (!rd_pose_motion(f, &m.lin_pose[i], &m.lin_motion[i])) goto bad;
        }
        RD(m.sqrt_inv_cov, (size_t)(N * N) * 8); RD(m.infovec, (size_t)N * 8);
        for (i = 0; i < nf; ++i) {
            double v[22];
            uint32_t has_pre;
            RD(&fr[i].id, 8); RD(v, 16 * 8 + 0);
            fr[i].pose.q.x = v[0]; fr[i].pose.q.y = v[1]; fr[i].pose.q.z = v[2]; fr[i].pose.q.w = v[3];
            memcpy(fr[i].pose.p, v + 4, 24); memcpy(fr[i].motion.v, v + 7, 24); memcpy(fr[i].motion.bg, v + 10, 24); memcpy(fr[i].motion.ba, v + 13, 24);
            RD(v, 7 * 8);
            fr[i].imu.q_cs.x = v[0]; fr[i].imu.q_cs.y = v[1]; fr[i].imu.q_cs.z = v[2]; fr[i].imu.q_cs.w = v[3]; memcpy(fr[i].imu.p_cs, v + 4, 24);
            RD(&has_pre, 4);
            if (has_pre) {
                rd_preint* p = &pre[i];
                double t[4];
                RD(&p->delta.t, 8); RD(t, 32); p->delta.q.x = t[0]; p->delta.q.y = t[1]; p->delta.q.z = t[2]; p->delta.q.w = t[3];
                RD(p->delta.p, 24); RD(p->delta.v, 24);
                RD(p->jac.dq_dbg, 72); RD(p->jac.dp_dbg, 72); RD(p->jac.dp_dba, 72); RD(p->jac.dv_dbg, 72); RD(p->jac.dv_dba, 72);
                RD(p->delta.sqrt_inv_cov, 225 * 8);
                fr[i].kpre = p;
            }
        }
        for (i = 0; i < nff; ++i) m.ids[i] = mpos[i] >= 0 ? fr[mpos[i]].id : (uint64_t)-1;
        RD(&ntracks, 4);
        tr = (rd_marg_track*)calloc(ntracks ? ntracks : 1, sizeof(rd_marg_track));
        obs = (rd_marg_obs**)calloc(ntracks ? ntracks : 1, sizeof(rd_marg_obs*));
        for (i = 0; i < ntracks; ++i) {
            uint32_t ref_pos, no;
            RD(&tr[i].id, 8); RD(&ref_pos, 4); RD(&tr[i].inv_depth, 8); RD(&no, 4);
            tr[i].ref = (int)ref_pos; tr[i].nobs = (int)no;
            obs[i] = (rd_marg_obs*)calloc(no ? no : 1, sizeof(rd_marg_obs));
            for (j = 0; j < no; ++j) {
                rd_marg_obs* o = &obs[i][j];
                double v[7];
                int32_t tgt;
                RD(&tgt, 4); o->tgt = tgt;
                RD(o->z, 24); RD(o->z_ref, 24);
                RD(v, 56); o->cam_ref.q_cs.x = v[0]; o->cam_ref.q_cs.y = v[1]; o->cam_ref.q_cs.z = v[2]; o->cam_ref.q_cs.w = v[3]; memcpy(o->cam_ref.p_cs, v + 4, 24);
                RD(v, 56); o->cam_tgt.q_cs.x = v[0]; o->cam_tgt.q_cs.y = v[1]; o->cam_tgt.q_cs.z = v[2]; o->cam_tgt.q_cs.w = v[3]; memcpy(o->cam_tgt.p_cs, v + 4, 24);
                RD(o->sqrt_inv_cov, 32);
                nobs++;
            }
            tr[i].obs = obs[i];
            ntrk++;
        }
        /* outputs */
        RD(&nff2, 4);
        N2 = (long)nff2 * 15;
        ref.nf = (int)nff2;
        ref.lin_pose = (rd_pose*)calloc(nff2 ? nff2 : 1, sizeof(rd_pose)); ref.lin_motion = (rd_motion*)calloc(nff2 ? nff2 : 1, sizeof(rd_motion));
        for (i = 0; i < nff2; ++i) if (!rd_pose_motion(f, &ref.lin_pose[i], &ref.lin_motion[i])) goto bad;
        ref.sqrt_inv_cov = dz((size_t)(N2 * N2)); ref.infovec = dz((size_t)N2);
        RD(ref.sqrt_inv_cov, (size_t)(N2 * N2) * 8); RD(ref.infovec, (size_t)N2 * 8);
        if (flags & 1) {
            eb = dz((size_t)(N2 * N2 * 2 + N2 * 2));
            RD(eb, (size_t)(N2 * N2 * 2 + N2 * 2) * 8);
        }
        ret = rd_marg_marginalize(&m, (int)nf, fr, (int)index, (int)ntracks, tr, (flags & 1) ? &dbg : NULL);
        if (ret != 0) { printf("  rec %ld: rd_marg_marginalize returned %d\n", rec, ret); g_bad++; g_cmp++; }
        else {
            cb = g_bad;
            if (m.nf != (int)nff2) { printf("  rec %ld: nf %d vs %u\n", rec, m.nf, nff2); g_bad++; }
            else {
                double a[16], b[16];
                for (i = 0; i < nff2; ++i) {
                    flat(&m.lin_pose[i], &m.lin_motion[i], a); flat(&ref.lin_pose[i], &ref.lin_motion[i], b);
                    cmp("lin", a, b, 16, rec);
                }
                c_lin += (long)nff2 * 16; b_lin += g_bad - cb; cb = g_bad;
                cmp("sqrt_inv_cov", m.sqrt_inv_cov, ref.sqrt_inv_cov, N2 * N2, rec); c_sic += N2 * N2; b_sic += g_bad - cb; cb = g_bad;
                cmp("infovec", m.infovec, ref.infovec, N2, rec); c_iv += N2; b_iv += g_bad - cb; cb = g_bad;
                if (flags & 1) {
                    const double* x = eb;
                    cmp("stage.infomat", dbg.infomat, x, N2 * N2, rec); c_im += N2 * N2; b_im += g_bad - cb; cb = g_bad; x += N2 * N2;
                    cmp("stage.infovec", dbg.infovec, x, N2, rec); c_ib += N2; b_ib += g_bad - cb; cb = g_bad; x += N2;
                    cmp("stage.evals", dbg.evals, x, N2, rec); c_ev += N2; b_ev += g_bad - cb; cb = g_bad; x += N2;
                    cmp("stage.evecs", dbg.evecs, x, N2 * N2, rec); c_ve += N2 * N2; b_ve += g_bad - cb; cb = g_bad;
                    for (i = 0; i < (uint32_t)N2; ++i) if (dbg.evals[i] <= 1.0e-8) cut++;
                    full++;
                }
            }
            if (N2 / 15 < 16) hist[N2 / 15]++;
            if (N2 > maxn) maxn = N2;
        }
        if (ret == 0 && m.eig_info) nonconv++;
        rd_marg_dbg_free(&dbg);
        rd_marg_free(&m); rd_marg_free(&ref);
        free(mpos); free(fr); free(pre); free(eb);
        if (obs) { for (i = 0; i < ntracks; ++i) free(obs[i]); free(obs); }
        free(tr);
        rec++;
        continue;
bad:
        printf("  truncated record %ld\n", rec);
        g_bad++;
        break;
    }
    printf("marg: %ld records (%ld with stage matrices), %ld values compared, %ld mismatches; max prior %ldx%ld, %ld tracks / %ld observations, %ld eigenvalues cut, %ld eigensolver NoConvergence\n",
           rec, full, g_cmp - c0, g_bad - b0, maxn, maxn, ntrk, nobs, cut, nonconv);
    printf("  by output: lin %ld/%ld  sqrt_inv_cov %ld/%ld  infovec %ld/%ld  stage.infomat %ld/%ld  stage.infovec %ld/%ld  stage.evals %ld/%ld  stage.evecs %ld/%ld (values/mismatches)\n",
           c_lin, b_lin, c_sic, b_sic, c_iv, b_iv, c_im, b_im, c_ib, b_ib, c_ev, b_ev, c_ve, b_ve);
    {
        int s;
        printf("  new prior frames:");
        for (s = 0; s < 16; ++s) if (hist[s]) printf(" %d:%ld", s, hist[s]);
        printf("\n");
    }
    fclose(f);
}

/* ---- eval.bin ---- */
static void check_eval(const char* dir) {
    FILE* f = open_kind(dir, "eval");
    long rec = 0, c0 = g_cmp, b0 = g_bad, withjac = 0, nj = 0;
    if (!f) return;
    for (;;) {
        uint32_t nff, have_jac, i, k;
        uint64_t mask;
        rd_marg m;
        long N;
        double *prm = NULL, *res = NULL, *got = NULL;
        double** jac = NULL;
        double** expj = NULL;
        const double** pp = NULL;
        memset(&m, 0, sizeof m);
        if (fread(&nff, 4, 1, f) != 1) break;
        RD(&i, 4);   /* the factor payload starts with its own frame count */
        if (i != nff) goto bad;
        N = (long)nff * 15;
        m.nf = (int)nff;
        m.lin_pose = (rd_pose*)calloc(nff ? nff : 1, sizeof(rd_pose)); m.lin_motion = (rd_motion*)calloc(nff ? nff : 1, sizeof(rd_motion));
        m.sqrt_inv_cov = dz((size_t)(N * N)); m.infovec = dz((size_t)N);
        for (i = 0; i < nff; ++i) if (!rd_pose_motion(f, &m.lin_pose[i], &m.lin_motion[i])) goto bad;
        RD(m.sqrt_inv_cov, (size_t)(N * N) * 8); RD(m.infovec, (size_t)N * 8);
        prm = dz((size_t)nff * 16);
        RD(prm, (size_t)nff * 16 * 8);
        RD(&have_jac, 4); RD(&mask, 8);
        res = dz((size_t)N); got = dz((size_t)N);
        RD(res, (size_t)N * 8);
        jac = (double**)calloc(5 * nff + 1, sizeof(double*));
        expj = (double**)calloc(5 * nff + 1, sizeof(double*));
        pp = (const double**)calloc(5 * nff + 1, sizeof(double*));
        for (k = 0; k < 5 * nff; ++k) {
            const long sz = N * (k % 5 == 0 ? 4 : 3);
            const long off = (long)(k / 5) * 16 + (k % 5 == 0 ? 0 : (k % 5 == 1 ? 4 : 7 + 3 * (long)(k % 5 - 2)));
            pp[k] = prm + off;
            if (have_jac && (mask >> k & 1)) {
                expj[k] = dz((size_t)sz); jac[k] = dz((size_t)sz);
                RD(expj[k], (size_t)sz * 8);
                memset(jac[k], 0xAB, (size_t)sz * 8);   /* the port must overwrite everything */
            }
        }
        rd_marg_eval(&m, pp, got, have_jac ? jac : NULL);
        cmp("residual", got, res, N, rec);
        if (have_jac) {
            withjac++;
            for (k = 0; k < 5 * nff; ++k) if (jac[k]) { cmp("jac", jac[k], expj[k], N * (k % 5 == 0 ? 4 : 3), rec); nj++; }
        }
        for (k = 0; k < 5 * nff; ++k) { free(jac[k]); free(expj[k]); }
        free(jac); free(expj); free(pp); free(prm); free(res); free(got);
        rd_marg_free(&m);
        rec++;
        continue;
bad:
        printf("  truncated eval record %ld\n", rec);
        g_bad++;
        break;
    }
    printf("eval: %ld records (%ld with Jacobians, %ld Jacobian blocks), %ld values compared, %ld mismatches\n", rec, withjac, nj, g_cmp - c0, g_bad - b0);
    fclose(f);
}

int main(int argc, char** argv) {
    int i; const char* dir;
    if (argc < 2) { fprintf(stderr, "usage: check_rd_m5 <dir> [marg|eval]\n"); return 2; }
    dir = argv[1];
    {
        int all = argc == 2;
        for (i = 2; i < argc || all; ++i) {
            const char* k = all ? "all" : argv[i];
            if (all || !strcmp(k, "marg")) check_marg(dir);
            if (all || !strcmp(k, "eval")) check_eval(dir);
            if (all) break;
        }
    }
    printf("TOTAL: %ld values compared, %ld mismatches\n", g_cmp, g_bad);
    return g_bad ? 1 : 0;
}
