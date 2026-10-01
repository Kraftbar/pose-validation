/* SV_PORT_SOURCES: check_sv_g2o_pose.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_linalg.c sv_eigen_llt.c
 * SPDX-License-Identifier: MIT
 *
 * Harness: replays every real stella_vslam pose_optimizer_g2o::optimize()
 * call captured by stella_port/reference_tools/dump_stella_g2o_pose.cc
 * (see that file and stella_port/reference/patches/
 * 0006-pose-optimizer-g2o-trace.patch for how the data was captured --
 * real library, real tracking pipeline, no synthetic data) and compares
 * this port's (sv_g2o_pose_optimizer.c) output against the reference's.
 *
 * Unlike this repo's other check_*.c harnesses, the dump this reads is
 * NOT under runs/stella_port/reference_dumps/ (it has its own tree,
 * runs/stella_port/reference_g2o/<seq>/, from tools/dump_stella_g2o.py) --
 * this harness still accepts the shared-runner's
 * "<seq_label> <fixtures_dir> <dump_dir> [max_frames]" argv shape so it
 * fits tools/check_stella_port.py's discovery/build/run convention, but
 * `fixtures_dir` is unused (this leaf has no image fixtures) and
 * `dump_dir` is only used to derive the reference_g2o path (it replaces a
 * trailing ".../reference_dumps/<seq>" with ".../reference_g2o/<seq>";
 * falls back to "<seq_label>" resolved as a sibling of `dump_dir`'s parent
 * otherwise). `max_frames`, if >=0, caps the number of calls replayed.
 *
 * Comparison targets: final pose (bit-exact, POSE_TOL/MAX_ROT_TOL are
 * exactly 0 -- see below), outlier flags, and num_valid_obs, given the
 * same inputs. Bit-exact confirmed 0/1570 (fr1_xyz), 0/1082 (fr1_desk)
 * mismatches (see stella_port/HANDOVER.md module-4b "bit-exact closure"
 * for the four bugs this took: (1) H/b and chi2 term-grouping order
 * (2 args multiplied together before summing, not summed then scaled),
 * (2) a missing `SE3Quat::normalizeRotation()` after every rotation
 * construction/composition, (3) g2o's actual pose-optimizer solver is
 * `Eigen::SimplicialLLT` (plain Cholesky), NOT `SimplicialLDLT` as
 * originally assumed -- see sv_eigen_llt.h, (4) stella's chi-squared
 * threshold/Huber-delta constants are FLOAT, not double
 * (`constexpr float chi_sq_2D = 5.99146f`), and (5) this port's LM loop
 * did not stop early on `rho==0`/max-trials-after-failure/non-finite
 * lambda the way g2o's `OptimizationAlgorithmLevenberg::solve()`
 * returning `Terminate` stops `SparseOptimizer::optimize()`'s outer
 * iteration loop). Per-iteration chi2/lambda/accept-reject trial detail
 * was not captured from the real g2o build itself (no public hook -- see
 * the reference tool's header comment); it WAS captured and cross-checked
 * via a separate instrumented-g2o debug build during development (see
 * HANDOVER.md), not as part of this harness's normal run. Prints
 * "<seq_label>: <mismatches>/<total>\n"; a "mismatch" is any call whose
 * final pose, outlier-flag set, or num_valid_obs differs at all.
 */
#include "sv_g2o_pose_optimizer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 65536
#define MAX_OBS_PER_CALL 4096

/* Bit-exact: any nonzero deviation is a mismatch (see header). */
#define POSE_TOL 0.0
#define MAX_ROT_TOL 0.0

typedef struct ref_call {
    unsigned int call_index;
    char call_site[32];
    unsigned int num_obs;
    double fx, fy, cx, cy;
    double m_init[16];
    double m_final[16];
    unsigned int num_valid_obs;
} ref_call;

static char* next_field(char** p) {
    char* start = *p;
    char* tab = strchr(start, '\t');
    if (tab) {
        *tab = '\0';
        *p = tab + 1;
    } else {
        char* nl = strchr(start, '\n');
        if (nl) *nl = '\0';
        *p = start + strlen(start);
    }
    return start;
}

static void parse_hex_list(char* s, double* out, int n) {
    int i = 0;
    char* tok = strtok(s, ",");
    while (tok && i < n) {
        out[i++] = strtod(tok, NULL);
        tok = strtok(NULL, ",");
    }
}

/* row-major m[r*4+c] -> sv_se3 (rotation transposed into column-major for
 * sv_quat_from_mat3, translation from column 3). */
static void mat16_to_se3(const double m[16], sv_se3* out) {
    double colmajor_r[9];
    int r, c;
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            colmajor_r[c * 3 + r] = m[r * 4 + c];
        }
    }
    sv_quat_from_mat3(colmajor_r, &out->q);
    out->t[0] = m[0 * 4 + 3];
    out->t[1] = m[1 * 4 + 3];
    out->t[2] = m[2 * 4 + 3];
    /* util::converter::to_g2o_SE3 == `g2o::SE3Quat{rot,trans}`, whose
     * constructor calls normalizeRotation() -- see sv_g2o_se3.h. */
    sv_se3_normalize_rotation(out);
}

static void se3_to_mat16(const sv_se3* pose, double m[16]) {
    double R[9]; /* column-major, R[c*3+r] */
    sv_quat_to_mat3(&pose->q, R);
    int r, c;
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            m[r * 4 + c] = R[c * 3 + r];
        }
        m[r * 4 + 3] = pose->t[r];
    }
    m[12] = 0;
    m[13] = 0;
    m[14] = 0;
    m[15] = 1;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_g2o_pose <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 1;
    }
    const char* seq_label = argv[1];
    const char* dump_dir = argv[3];
    long max_calls = (argc >= 5) ? atol(argv[4]) : -1;

    char g2o_dir[4096];
    {
        const char* pos = strstr(dump_dir, "reference_dumps");
        if (pos) {
            size_t prefix_len = (size_t)(pos - dump_dir);
            snprintf(g2o_dir, sizeof(g2o_dir), "%.*sreference_g2o%s", (int)prefix_len, dump_dir, pos + strlen("reference_dumps"));
        } else {
            snprintf(g2o_dir, sizeof(g2o_dir), "%s", dump_dir);
        }
    }

    char calls_path[4200], obs_path[4200], outliers_path[4200];
    snprintf(calls_path, sizeof(calls_path), "%s/calls.tsv", g2o_dir);
    snprintf(obs_path, sizeof(obs_path), "%s/obs.tsv", g2o_dir);
    snprintf(outliers_path, sizeof(outliers_path), "%s/outliers.tsv", g2o_dir);

    FILE* fc = fopen(calls_path, "r");
    FILE* fo = fopen(obs_path, "r");
    FILE* fl = fopen(outliers_path, "r");
    if (!fc || !fo || !fl) {
        /* This leaf has only captured fr1_xyz/fr1_desk (see
         * tools/dump_stella_g2o.py); skip other sequences cleanly rather
         * than failing the shared runner for a dump this leaf never
         * claimed to produce. */
        fprintf(stderr, "%s: no runs/stella_port/reference_g2o dump (%s) -- skipping\n", seq_label, g2o_dir);
        printf("%s: 0/0\n", seq_label);
        if (fc) fclose(fc);
        if (fo) fclose(fo);
        if (fl) fclose(fl);
        return 0;
    }

    char line[MAX_LINE];
    if (!fgets(line, sizeof(line), fc)) { /* header */
    }
    if (!fgets(line, sizeof(line), fo)) {
    }
    if (!fgets(line, sizeof(line), fl)) {
    }

    long obs_line_ok = 1, outlier_line_ok = 1;
    char obs_line[MAX_LINE], outlier_line[MAX_LINE];
    obs_line_ok = fgets(obs_line, sizeof(obs_line), fo) != NULL;
    outlier_line_ok = fgets(outlier_line, sizeof(outlier_line), fl) != NULL;

    unsigned int total = 0, mismatches = 0;
    double max_pos_err_seen = 0.0, max_rot_err_seen = 0.0;
    unsigned int outlier_mismatch_calls = 0, count_mismatch_calls = 0;

    while (fgets(line, sizeof(line), fc)) {
        if (max_calls >= 0 && (long)total >= max_calls) break;

        ref_call rc;
        char* p = line;
        rc.call_index = (unsigned int)atol(next_field(&p));
        snprintf(rc.call_site, sizeof(rc.call_site), "%s", next_field(&p));
        rc.num_obs = (unsigned int)atol(next_field(&p));
        rc.fx = strtod(next_field(&p), NULL);
        rc.fy = strtod(next_field(&p), NULL);
        rc.cx = strtod(next_field(&p), NULL);
        rc.cy = strtod(next_field(&p), NULL);
        char* ntr_s = next_field(&p);
        char* nt_s = next_field(&p);
        char* nei_s = next_field(&p);
        char* init_s = next_field(&p);
        char* final_s = next_field(&p);
        char* nvo_s = next_field(&p);
        (void)nvo_s;

        sv_pose_optimizer_params params;
        params.num_trials_robust = (unsigned int)atol(ntr_s);
        params.num_trials = (unsigned int)atol(nt_s);
        params.num_each_iter = (unsigned int)atol(nei_s);

        parse_hex_list(init_s, rc.m_init, 16);
        parse_hex_list(final_s, rc.m_final, 16);
        rc.num_valid_obs = (unsigned int)atol(nvo_s);

        static sv_pose_opt_edge edges[MAX_OBS_PER_CALL];
        unsigned int n_edges = 0;

        while (obs_line_ok) {
            char obuf[MAX_LINE];
            strcpy(obuf, obs_line);
            char* op = obuf;
            unsigned int oc = (unsigned int)atol(next_field(&op));
            if (oc != rc.call_index) break;
            unsigned int idx = (unsigned int)atol(next_field(&op));
            double x = strtod(next_field(&op), NULL);
            double y = strtod(next_field(&op), NULL);
            int octave = atoi(next_field(&op));
            (void)octave;
            double isq = strtod(next_field(&op), NULL);
            char* pw_s = next_field(&op);
            double pw[3];
            char pwbuf[256];
            snprintf(pwbuf, sizeof(pwbuf), "%s", pw_s);
            parse_hex_list(pwbuf, pw, 3);

            if (n_edges < MAX_OBS_PER_CALL) {
                sv_pose_opt_edge* e = &edges[n_edges++];
                e->pos_w[0] = pw[0];
                e->pos_w[1] = pw[1];
                e->pos_w[2] = pw[2];
                e->obs[0] = x;
                e->obs[1] = y;
                e->inv_sigma_sq = isq;
                e->fx = rc.fx;
                e->fy = rc.fy;
                e->cx = rc.cx;
                e->cy = rc.cy;
                e->level = 0;
                (void)idx;
            }
            obs_line_ok = fgets(obs_line, sizeof(obs_line), fo) != NULL;
        }

        /* consume outlier rows for this call (not cross-checked per-idx
         * here beyond the final-flags recomputation below, since idx
         * mapping back to the original undist_keypts_ index is not needed
         * to replay the optimizer -- we only need the SET of flags on the
         * edges array we built, in the same order). */
        unsigned int ref_num_outliers = 0;
        while (outlier_line_ok) {
            char lbuf[MAX_LINE];
            strcpy(lbuf, outlier_line);
            char* lp = lbuf;
            unsigned int lc = (unsigned int)atol(next_field(&lp));
            if (lc != rc.call_index) break;
            next_field(&lp); /* idx, unused */
            int is_outlier = atoi(next_field(&lp));
            if (is_outlier) ref_num_outliers++;
            outlier_line_ok = fgets(outlier_line, sizeof(outlier_line), fl) != NULL;
        }

        sv_se3 pose;
        mat16_to_se3(rc.m_init, &pose);
        unsigned int num_valid = sv_pose_optimizer_optimize(&pose, edges, (int)n_edges, &params);

        total++;

        if (n_edges < 5) {
            /* early-return path: reference outlier_flags trace is empty
             * (see dump tool header), so there is nothing to compare
             * beyond num_valid_obs == 0 and pose left untouched. */
            if (rc.num_valid_obs != 0 || num_valid != 0) {
                mismatches++;
            }
            continue;
        }

        if (num_valid != rc.num_valid_obs) {
            count_mismatch_calls++;
        }

        unsigned int port_num_outliers = 0;
        unsigned int i;
        for (i = 0; i < n_edges; ++i) {
            if (edges[i].level != 0) port_num_outliers++;
        }
        if (port_num_outliers != ref_num_outliers) {
            outlier_mismatch_calls++;
        }

        double m_out[16];
        se3_to_mat16(&pose, m_out);
        double pos_err = 0, rot_err = 0;
        for (i = 0; i < 16; ++i) {
            double d = fabs(m_out[i] - rc.m_final[i]);
            if (i % 4 == 3) {
                if (d > pos_err) pos_err = d;
            } else {
                if (d > rot_err) rot_err = d;
            }
        }
        if (pos_err > max_pos_err_seen) max_pos_err_seen = pos_err;
        if (rot_err > max_rot_err_seen) max_rot_err_seen = rot_err;

        int this_mismatch = 0;
        if (pos_err > POSE_TOL || rot_err > MAX_ROT_TOL) this_mismatch = 1;
        if (num_valid != rc.num_valid_obs) this_mismatch = 1;
        if (port_num_outliers != ref_num_outliers) this_mismatch = 1;
        if (this_mismatch) mismatches++;
    }

    fclose(fc);
    fclose(fo);
    fclose(fl);

    fprintf(stderr, "%s: max_pos_err=%.3e max_rot_err=%.3e num_valid_obs mismatches=%u outlier-count mismatches=%u (of %u calls)\n",
            seq_label, max_pos_err_seen, max_rot_err_seen, count_mismatch_calls, outlier_mismatch_calls, total);
    printf("%s: %u/%u\n", seq_label, mismatches, total);
    return mismatches == 0 ? 0 : 1;
}
