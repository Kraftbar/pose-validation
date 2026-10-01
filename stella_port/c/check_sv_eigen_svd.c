/* SV_PORT_SOURCES: check_sv_eigen_svd.c sv_eigen_svd.c sv_eigen_qr.c
 * SPDX-License-Identifier: MIT
 *
 * Reads runs/stella_port/reference_init/svd_fixtures.txt (written by the
 * standalone stella_port/reference_tools/dump_eigen_svd.cc, real Eigen
 * built with the reference's flags -- see provenance.json), runs the C99
 * port (sv_eigen_svd.{h,c} / sv_eigen_qr.{h,c}) on each fixture, and
 * compares bit-for-bit every output stella_vslam's solvers actually read:
 * NX9 case -> matrixV().col(8) is what solvers use, but we compare all of
 * V (9x9), all singular values and rank() for full coverage; MAT3 case ->
 * full U, V, singular values (homography/fundamental/essential decompose()
 * read all three).
 */
#include "sv_eigen_svd.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double read_hex(FILE *f) {
    double v;
    if (fscanf(f, "%lf", &v) != 1) { fprintf(stderr, "unexpected EOF\n"); exit(2); }
    return v;
}

static int bits_equal(double a, double b) {
    uint64_t ua, ub;
    memcpy(&ua, &a, 8);
    memcpy(&ub, &b, 8);
    return ua == ub;
}

static int ulp_diff(double a, double b) {
    if (bits_equal(a, b)) return 0;
    int64_t ia, ib;
    memcpy(&ia, &a, 8);
    memcpy(&ib, &b, 8);
    if (ia < 0) ia = (int64_t)0x8000000000000000ULL - ia;
    if (ib < 0) ib = (int64_t)0x8000000000000000ULL - ib;
    int64_t d = ia - ib;
    if (d < 0) d = -d;
    return d > 1000000000 ? 1000000000 : (int)d;
}

int main(int argc, char** argv) {
    /* tools/check_stella_port.py's shared runner invokes every check_*.c
     * harness as `<seq_label> <fixtures_dir> <dump_dir> [max_frames]`; SVD
     * fixtures aren't per-sequence, so seq_label is only used to label the
     * summary line below (the fixture path is fixed, same as check_sv_rng.c's
     * approach for its own non-per-sequence fixture). */
    const char* seq_label = argc > 1 ? argv[1] : "svd";
    const char *path = "runs/stella_port/reference_init/svd_fixtures.txt";
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "missing %s -- build & run stella_port/reference_tools/dump_eigen_svd.cc first\n", path);
        return 2;
    }

    long n_fixtures = 0;
    long n_mismatch_fixtures = 0;
    long max_ulp = 0;
    long total_values = 0, total_mismatch_values = 0;
    char tag[64];

    while (fscanf(f, "%63s", tag) == 1) {
        if (strcmp(tag, "NX9") == 0) {
            int N;
            if (fscanf(f, "%d", &N) != 1) break;
            double *A = (double *)malloc((size_t)N * 9 * sizeof(double));
            for (int i = 0; i < N * 9; ++i) A[i] = read_hex(f);
            double Vref[81], svref[9];
            for (int i = 0; i < 81; ++i) Vref[i] = read_hex(f);
            for (int i = 0; i < 9; ++i) svref[i] = read_hex(f);
            char rtag[16]; int rank_ref;
            if (fscanf(f, "%15s %d", rtag, &rank_ref) != 2) { fprintf(stderr, "bad RANK line\n"); return 2; }

            double Vgot[81], svgot[9];
            int rank_got = -1;
            sv_eigen_jacobisvd_Nx9_v(A, N, Vgot, svgot, &rank_got);

            int fixture_bad = 0;
            for (int i = 0; i < 81; ++i) {
                total_values++;
                if (!bits_equal(Vref[i], Vgot[i])) {
                    total_mismatch_values++;
                    fixture_bad = 1;
                    int u = ulp_diff(Vref[i], Vgot[i]);
                    if (u > max_ulp) max_ulp = u;
                }
            }
            for (int i = 0; i < 9; ++i) {
                total_values++;
                if (!bits_equal(svref[i], svgot[i])) {
                    total_mismatch_values++;
                    fixture_bad = 1;
                    int u = ulp_diff(svref[i], svgot[i]);
                    if (u > max_ulp) max_ulp = u;
                }
            }
            total_values++;
            if (rank_ref != rank_got) { total_mismatch_values++; fixture_bad = 1; }

            fprintf(stdout, "SUMMARY NX9 N=%d bad=%d\n", N, fixture_bad);
            if (fixture_bad) {
                n_mismatch_fixtures++;
                if (n_mismatch_fixtures <= 5) {
                    fprintf(stderr, "[NX9 N=%d] mismatch (rank ref=%d got=%d)\n", N, rank_ref, rank_got);
                    for (int i = 0; i < 81; ++i) {
                        if (!bits_equal(Vref[i], Vgot[i]))
                            fprintf(stderr, "  V[%d]=%d,%d ref=%.17g got=%.17g ulp=%d\n",
                                    i, i % 9, i / 9, Vref[i], Vgot[i], ulp_diff(Vref[i], Vgot[i]));
                    }
                    for (int i = 0; i < 9; ++i) {
                        if (!bits_equal(svref[i], svgot[i]))
                            fprintf(stderr, "  sv[%d] ref=%.17g got=%.17g ulp=%d\n",
                                    i, svref[i], svgot[i], ulp_diff(svref[i], svgot[i]));
                    }
                }
            }
            n_fixtures++;
            free(A);
        } else if (strcmp(tag, "MAT3") == 0) {
            double A[9];
            for (int i = 0; i < 9; ++i) A[i] = read_hex(f);
            double Uref[9], Vref[9], svref[3];
            for (int i = 0; i < 9; ++i) Uref[i] = read_hex(f);
            for (int i = 0; i < 9; ++i) Vref[i] = read_hex(f);
            for (int i = 0; i < 3; ++i) svref[i] = read_hex(f);

            double Ugot[9], Vgot[9], svgot[3];
            sv_eigen_jacobisvd_3x3(A, Ugot, Vgot, svgot);

            int fixture_bad = 0;
            for (int i = 0; i < 9; ++i) {
                total_values++;
                if (!bits_equal(Uref[i], Ugot[i])) { total_mismatch_values++; fixture_bad = 1; int u = ulp_diff(Uref[i], Ugot[i]); if (u > max_ulp) max_ulp = u; }
                total_values++;
                if (!bits_equal(Vref[i], Vgot[i])) { total_mismatch_values++; fixture_bad = 1; int u = ulp_diff(Vref[i], Vgot[i]); if (u > max_ulp) max_ulp = u; }
            }
            for (int i = 0; i < 3; ++i) {
                total_values++;
                if (!bits_equal(svref[i], svgot[i])) { total_mismatch_values++; fixture_bad = 1; int u = ulp_diff(svref[i], svgot[i]); if (u > max_ulp) max_ulp = u; }
            }
            if (fixture_bad) {
                n_mismatch_fixtures++;
                if (n_mismatch_fixtures <= 5) {
                    fprintf(stderr, "[MAT3] mismatch\n");
                    for (int i = 0; i < 9; ++i) {
                        if (!bits_equal(Uref[i], Ugot[i])) fprintf(stderr, "  U[%d] ref=%.17g got=%.17g ulp=%d\n", i, Uref[i], Ugot[i], ulp_diff(Uref[i], Ugot[i]));
                        if (!bits_equal(Vref[i], Vgot[i])) fprintf(stderr, "  V[%d] ref=%.17g got=%.17g ulp=%d\n", i, Vref[i], Vgot[i], ulp_diff(Vref[i], Vgot[i]));
                    }
                    for (int i = 0; i < 3; ++i)
                        if (!bits_equal(svref[i], svgot[i])) fprintf(stderr, "  sv[%d] ref=%.17g got=%.17g ulp=%d\n", i, svref[i], svgot[i], ulp_diff(svref[i], svgot[i]));
                }
            }
            n_fixtures++;
        } else {
            fprintf(stderr, "unknown tag '%s'\n", tag);
            break;
        }
    }
    fclose(f);

    fprintf(stdout, "fixtures=%ld mismatched_fixtures=%ld values=%ld mismatched_values=%ld max_ulp=%ld\n",
            n_fixtures, n_mismatch_fixtures, total_values, total_mismatch_values, max_ulp);
    /* Shared-runner line (tools/check_stella_port.py's SOURCES_RE/regex
     * expects the LAST stdout line to match "<label>: <mismatches>/<total>"). */
    fprintf(stdout, "%s: %ld/%ld\n", seq_label, total_mismatch_values, total_values);
    return n_mismatch_fixtures == 0 ? 0 : 1;
}
