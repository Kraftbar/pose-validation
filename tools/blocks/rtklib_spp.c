/* SPDX-License-Identifier: BSD-2-Clause (this harness; links RTKLIB, BSD-2-Clause, in external/blocks/src/RTKLIB)
 * Own harness: text dump of u-blox raw observations/ephemerides (gnss_raw_decode.py) -> RTKLIB pntpos() SPP + Doppler velocity.
 *   rtklib_spp <txtdir> <out.txt> [mode=l1|if] [sys=GECR] [elmin=15] [snrmin=0] [dopp_sign=1] [fde=0]
 * out line: tow stat ns x y z vx vy vz dtr(s) qxx qyy qzz qxy qyz qzx qvxx qvyy qvzz pdop(unused=0)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rtklib.h"

#define MAXEPH 1024
typedef struct { int week; double tow, p, d, snr; int freqhz; } sig_t;

static int cmpd(const void *a, const void *b) { return 0; }

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage\n"); return 1; }
    char path[1024]; const char *dir = argv[1];
    int dual = 0, elmin = 15, fde = 0; double snrmin = 0, dsign = 1; const char *syss = "GECR"; int use_bdsgeo = 0; (void)use_bdsgeo;
    for (int i = 3; i < argc; ++i) {
        if (!strncmp(argv[i], "mode=", 5)) dual = !strcmp(argv[i] + 5, "if");
        else if (!strncmp(argv[i], "sys=", 4)) syss = argv[i] + 4;
        else if (!strncmp(argv[i], "elmin=", 6)) elmin = atoi(argv[i] + 6);
        else if (!strncmp(argv[i], "snrmin=", 7)) snrmin = atof(argv[i] + 7);
        else if (!strncmp(argv[i], "dopp_sign=", 10)) dsign = atof(argv[i] + 10);
        else if (!strncmp(argv[i], "fde=", 4)) fde = atoi(argv[i] + 4);
    }
    int navsys = 0;
    for (const char *c = syss; *c; ++c) navsys |= (*c == 'G' ? SYS_GPS : *c == 'E' ? SYS_GAL : *c == 'C' ? SYS_CMP : *c == 'R' ? SYS_GLO : 0);

    nav_t nav; memset(&nav, 0, sizeof nav);
    nav.eph = (eph_t *)calloc(MAXEPH, sizeof(eph_t)); nav.geph = (geph_t *)calloc(MAXEPH, sizeof(geph_t));
    int fcn[MAXPRNGLO + 2]; for (int i = 0; i < MAXPRNGLO + 2; ++i) fcn[i] = 99;
    /* satellite id of the message stream -> RTKLIB satno (mapping inferred from ephemeris semi-major axes: GPS 1-32, GLO 33-59, GAL 60-95, BDS 97+) */
    #define SATMAP(id, sys_out, prn_out) do { int _i = (id); \
        if (_i <= 32) { sys_out = SYS_GPS; prn_out = _i; } else if (_i <= 59) { sys_out = SYS_GLO; prn_out = _i - 32; } \
        else if (_i <= 95) { sys_out = SYS_GAL; prn_out = _i - 59; } else { sys_out = SYS_CMP; prn_out = _i - 97; } } while (0)

    /* ephemerides */
    snprintf(path, sizeof path, "%s/eph.txt", dir); FILE *f = fopen(path, "r"); char line[2048];
    while (f && fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        double v[31]; char *s = line; int k = 0;
        for (; k < 31; ++k) { char *e; v[k] = strtod(s, &e); if (e == s) break; s = e; }
        if (k < 31) continue;
        int sys, prn; SATMAP((int)v[0], sys, prn);
        if (!(navsys & sys) || sys == SYS_GLO) continue;
        int sat = satno(sys, prn); if (!sat) continue;
        eph_t *e = &nav.eph[nav.n++];
        e->sat = sat; e->week = (int)v[1]; e->iode = (int)v[5]; e->iodc = (int)v[6]; e->svh = (int)v[7]; e->code = (int)v[8];
        e->sva = 0; e->toes = v[2]; e->A = v[10]; e->e = v[11]; e->i0 = v[12]; e->omg = v[13]; e->OMG0 = v[14]; e->M0 = v[15]; e->deln = v[16];
        e->OMGd = v[17]; e->idot = v[18]; e->cuc = v[19]; e->cus = v[20]; e->crc = v[21]; e->crs = v[22]; e->cic = v[23]; e->cis = v[24];
        e->f0 = v[25]; e->f1 = v[26]; e->f2 = v[27]; e->tgd[0] = v[28]; e->tgd[1] = v[29];
        int wk = (int)v[30]; if (sys == SYS_CMP) { e->week = wk; }
        e->toe = gpst2time(e->week, v[2]); e->toc = gpst2time(e->week, v[3]); e->ttr = gpst2time(e->week, v[4]);
        if (sys == SYS_CMP) { e->toes = v[2] - 14.0; e->flag = 1; } /* message stream gives BDS epochs already in GPST (toe = BDT + 14 s); RTKLIB wants toes in BDT */
        e->fit = 0;
        if (sys == SYS_GAL) e->tgd[2] = 0;
        if (nav.n >= MAXEPH) break;
    }
    if (f) fclose(f);
    /* GLONASS */
    snprintf(path, sizeof path, "%s/glo.txt", dir); f = fopen(path, "r");
    while ((navsys & SYS_GLO) && f && fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        double v[21]; char *s = line; int k = 0;
        for (; k < 21; ++k) { char *e; v[k] = strtod(s, &e); if (e == s) break; s = e; }
        if (k < 21) continue;
        int sys, prn; SATMAP((int)v[0], sys, prn);
        int sat = satno(SYS_GLO, prn); if (!sat) continue;
        geph_t *g = &nav.geph[nav.ng++];
        memset(g, 0, sizeof *g);
        g->sat = sat; g->frq = (int)v[4]; g->iode = (int)v[5]; g->svh = (int)v[6]; g->age = (int)v[7]; g->sva = 0;
        g->toe = gpst2time((int)v[1], v[2]); g->tof = gpst2time((int)v[1], v[3]);
        for (int i = 0; i < 3; ++i) { g->pos[i] = v[9 + i]; g->vel[i] = v[12 + i]; g->acc[i] = v[15 + i]; }
        g->taun = v[18]; g->gamn = v[19]; g->dtaun = v[20];
        if (prn >= 1 && prn <= MAXPRNGLO) nav.glo_fcn[prn - 1] = g->frq + 8;
        if (nav.ng >= MAXEPH) break;
    }
    if (f) fclose(f);
    /* Klobuchar from the last broadcast set */
    snprintf(path, sizeof path, "%s/iono.txt", dir); f = fopen(path, "r");
    if (f) { double t; double a[8]; while (fgets(line, sizeof line, f)) { if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &t, a, a + 1, a + 2, a + 3, a + 4, a + 5, a + 6, a + 7) == 9) memcpy(nav.ion_gps, a, sizeof a); } fclose(f); }
    fprintf(stderr, "eph %d geph %d ion %g %g\n", nav.n, nav.ng, nav.ion_gps[0], nav.ion_gps[4]);

    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_SINGLE; opt.nf = dual ? 2 : 1; opt.navsys = navsys; opt.elmin = elmin * D2R;
    opt.ionoopt = dual ? IONOOPT_IFLC : IONOOPT_BRDC; opt.tropopt = TROPOPT_SAAS; opt.sateph = EPHOPT_BRDC;
    opt.posopt[4] = fde; opt.dynamics = 0;
    for (int i = 0; i < NFREQ; ++i) for (int j = 0; j < 9; ++j) opt.snrmask.mask[i][j] = snrmin;
    if (snrmin > 0) opt.snrmask.ena[0] = 1;

    /* observations: epoch grouping by tow */
    snprintf(path, sizeof path, "%s/obs.txt", dir); f = fopen(path, "r"); if (!f) return 2;
    FILE *fo = fopen(argv[2], "w");
    obsd_t obs[MAXOBS]; int no = 0; double cur_tow = -1; int cur_week = 0;
    sol_t sol; memset(&sol, 0, sizeof sol); ssat_t *ssat = (ssat_t *)calloc(MAXSAT, sizeof(ssat_t)); double azel[2 * MAXOBS]; char msg[256];
    int nep = 0, nok = 0;
    for (;;) {
        char *r = fgets(line, sizeof line, f);
        int week = 0; double tow = 0, fr, cn0, psr, ps, cp, cps, dop, dps; int sat = 0, lli, code, st;
        int have = r && line[0] != '#' && sscanf(line, "%d %lf %d %lf %lf %d %d %lf %lf %lf %lf %lf %lf %d", &week, &tow, &sat, &fr, &cn0, &lli, &code, &psr, &ps, &cp, &cps, &dop, &dps, &st) == 14;
        if (r && !have) continue;
        if (!r || tow != cur_tow) {
            if (no > 0) {
                ++nep; int ok = pntpos(obs, no, &nav, &opt, &sol, azel, ssat, msg);
                if (ok) ++nok;
                fprintf(fo, "%.4f %d %d %.4f %.4f %.4f %.5f %.5f %.5f %.9e %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g\n", cur_tow, ok ? sol.stat : -1, sol.ns, sol.rr[0], sol.rr[1], sol.rr[2],
                        sol.rr[3], sol.rr[4], sol.rr[5], sol.dtr[0], sol.qr[0], sol.qr[1], sol.qr[2], sol.qr[3], sol.qr[4], sol.qr[5], sol.qv[0], sol.qv[1], sol.qv[2]);
            }
            no = 0; if (!r) break; cur_tow = tow; cur_week = week;
        }
        if (!(st & 1) || psr == 0.0) continue;
        int sys, prn; SATMAP(sat, sys, prn);
        if (!(navsys & sys)) continue;
        int s = satno(sys, prn); if (!s) continue;
        int idx = -1; for (int i = 0; i < no; ++i) if (obs[i].sat == s) idx = i;
        if (idx < 0) { if (no >= MAXOBS) continue; idx = no++; memset(&obs[idx], 0, sizeof(obsd_t)); obs[idx].sat = (uint8_t)s; obs[idx].time = gpst2time(cur_week, cur_tow); obs[idx].rcv = 1; }
        int first = fr > 1.5e9; /* L1/E1/B1/G1 band vs second band */
        int fi = first ? 0 : 1;
        int c1, c2;
        switch (sys) { case SYS_GPS: c1 = CODE_L1C; c2 = CODE_L2X; break; case SYS_GAL: c1 = CODE_L1C; c2 = CODE_L7Q; break; case SYS_CMP: c1 = CODE_L2I; c2 = CODE_L7I; break; default: c1 = CODE_L1C; c2 = CODE_L2C; }
        obs[idx].code[fi] = (uint8_t)(first ? c1 : c2); obs[idx].P[fi] = psr; obs[idx].D[fi] = (float)(dsign * dop); obs[idx].SNR[fi] = (float)cn0;
        obs[idx].Pstd[fi] = (float)ps; obs[idx].SNR[fi] = (float)cn0;
        if (sys == SYS_GLO) { for (int i = 0; i < nav.ng; ++i) if (nav.geph[i].sat == s) { obs[idx].freq = (uint8_t)(nav.geph[i].frq + 7); } }
    }
    fprintf(stderr, "epochs %d ok %d\n", nep, nok);
    fclose(fo); (void)cmpd; return 0;
}
