/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_solve_fundamental.h (BSD-2, AIST 2019 + stella-cv 2022; SVD via
 * sv_eigen_svd.h, MPL-2.0, used not reproduced). */
#include "sv_solve_fundamental.h"
#include "sv_solve_common.h"
#include "sv_solve_essential.h"
#include "sv_linalg.h"
#include "sv_eigen_svd.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

void sv_solve_compute_F21(const float* x1, const float* y1,
                           const float* x2, const float* y2, int n,
                           double F21[9]) {
    int i;
    double* A = (double*)malloc(sizeof(double) * (size_t)n * 9);
    double V[81], sv9[9];
    int rank;
    double v[9], init_F[9];
    double U[9], V3[9], Vt3[9], sv3[3];

#define AC(col, row) A[(col) * n + (row)]
    for (i = 0; i < n; ++i) {
        double X1 = (double)x1[i], Y1 = (double)y1[i];
        double X2 = (double)x2[i], Y2 = (double)y2[i];
        AC(0, i) = X2 * X1; AC(1, i) = X2 * Y1; AC(2, i) = X2 * 1.0;
        AC(3, i) = Y2 * X1; AC(4, i) = Y2 * Y1; AC(5, i) = Y2 * 1.0;
        AC(6, i) = X1;      AC(7, i) = Y1;      AC(8, i) = 1.0;
    }
#undef AC

    sv_eigen_jacobisvd_Nx9_v(A, n, V, sv9, &rank);
    free(A);

    for (i = 0; i < 9; ++i) {
        v[i] = V[8 * 9 + i];
    }
    {
        int r, c;
        for (r = 0; r < 3; ++r) {
            for (c = 0; c < 3; ++c) {
                init_F[c * 3 + r] = v[r * 3 + c];
            }
        }
    }

    sv_eigen_jacobisvd_3x3(init_F, U, V3, sv3);
    sv_mat3_transpose(V3, Vt3);
    sv3[2] = 0.0;

    {
        double scaledU[9];
        int c, r;
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                scaledU[c * 3 + r] = U[c * 3 + r] * sv3[c];
            }
        }
        sv_mat3_mul(scaledU, Vt3, F21);
    }
}

int sv_solve_fundamental_decompose(const double F21[9], const double cam1[9], const double cam2[9],
                                    double rots[4][9], double transes[4][3]) {
    double tmp[9], E21[9];
    /* cam_matrix_2.transpose() * F_21 * cam_matrix_1: the FIRST multiply's
     * LHS is the inline transpose -- bisected against real Eigen
     * (stella_port/reference_tools/debug_decompose.cc): ALL rows
     * left-associative (sv_mat3_mul_lhs_transposed), NOT the mixed rule
     * (that applies to A*B and A*B.transpose(), not A.transpose()*B). The
     * second multiply's LHS is the (materialized) result of the first, a
     * plain matrix, so it uses the normal mixed-rule sv_mat3_mul. */
    sv_mat3_mul_lhs_transposed(cam2, F21, tmp);
    sv_mat3_mul(tmp, cam1, E21);
    sv_solve_essential_decompose(E21, rots, transes);
    return 1;
}

static unsigned int check_inliers_F(const sv_keypoint* undist_1, const sv_keypoint* undist_2,
                                     const sv_match_pair* matches, unsigned int num_matches,
                                     const double F21[9], float sigma,
                                     unsigned char* is_inlier, float* cost_out) {
    unsigned int i, num_inliers = 0;
    const float chi_sq = 5.991f;
    float sigma_sq = sigma * sigma;
    float cost = 0.0f;

    for (i = 0; i < num_matches; ++i) {
        const sv_keypoint* k1 = &undist_1[matches[i].idx1];
        const sv_keypoint* k2 = &undist_2[matches[i].idx2];
        double pt1[3], pt2[3];
        double F21pt1[3];
        double pt2F21[3]; /* pt_2^T * F_21, as a row (1x3) */
        double pt2F21pt1;
        double dist_sq;
        float thr;

        pt1[0] = (double)k1->x; pt1[1] = (double)k1->y; pt1[2] = 1.0;
        pt2[0] = (double)k2->x; pt2[1] = (double)k2->y; pt2[2] = 1.0;

        sv_mat3_mulv(F21, pt1, F21pt1);

        /* pt2F21[j] = sum_i pt2[i] * F21(i,j): pt_2.transpose() is an
         * inline (lazy, non-materialized) Transpose<Vector3d>, and being a
         * 1-row LHS disables the packet path entirely regardless of the
         * RHS -- pure scalar coeff() path, right-associative (see
         * sv_mat3_mul's header comment). pt2F21pt1 = pt_2_F_21 * pt_1 is
         * then an InnerProduct (1x3 . 3x1), likewise right-associative. */
        {
            int j;
            for (j = 0; j < 3; ++j) {
                pt2F21[j] = pt2[0] * F21[j * 3 + 0] + (pt2[1] * F21[j * 3 + 1] + pt2[2] * F21[j * 3 + 2]);
            }
        }
        pt2F21pt1 = pt2F21[0] * pt1[0] + (pt2F21[1] * pt1[1] + pt2F21[2] * pt1[2]);
        {
            double num = pt2F21pt1 * pt2F21pt1;
            double denom = (F21pt1[0] * F21pt1[0] + F21pt1[1] * F21pt1[1])
                          + (pt2F21[0] * pt2F21[0] + pt2F21[1] * pt2F21[1]);
            dist_sq = num / denom;
        }

        thr = chi_sq * sigma_sq;
        if ((double)thr > dist_sq) {
            is_inlier[i] = 1;
            cost = (float)(cost + dist_sq);
            num_inliers++;
        }
        else {
            is_inlier[i] = 0;
            cost = cost + thr;
        }
    }
    *cost_out = cost;
    return num_inliers;
}

void sv_solve_fundamental_find_via_ransac(
    const sv_keypoint* undist_1, unsigned int num_kp1,
    const sv_keypoint* undist_2, unsigned int num_kp2,
    const sv_match_pair* matches, unsigned int num_matches,
    float sigma, unsigned int max_num_iter, int recompute,
    sv_mt19937* engine,
    sv_fundamental_ransac_result* out) {
    unsigned int i;
    const unsigned int min_set_size = 8;
    /* normalize() over the FULL keypoint arrays -- see the analogous
     * comment in sv_solve_homography_find_via_ransac. */
    float* px1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* py1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* px2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* py2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* nx1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* ny1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* nx2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* ny2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    double transform1[9], transform2[9], transform2_t[9];
    unsigned char* inlier_sac = (unsigned char*)malloc(num_matches ? num_matches : 1);
    unsigned int iter;
    uint32_t idxbuf[64];

    out->solution_valid = 0;
    out->best_cost = FLT_MAX;
    memset(out->is_inlier, 0, num_matches);

    for (i = 0; i < num_kp1; ++i) {
        px1[i] = undist_1[i].x;
        py1[i] = undist_1[i].y;
    }
    for (i = 0; i < num_kp2; ++i) {
        px2[i] = undist_2[i].x;
        py2[i] = undist_2[i].y;
    }
    sv_solve_normalize(px1, py1, (int)num_kp1, nx1, ny1, transform1);
    sv_solve_normalize(px2, py2, (int)num_kp2, nx2, ny2, transform2);
    sv_mat3_transpose(transform2, transform2_t);

    if (num_matches < min_set_size) {
        free(nx1); free(ny1); free(nx2); free(ny2);
        free(px1); free(py1); free(px2); free(py2);
        free(inlier_sac);
        return;
    }

    for (iter = 0; iter < max_num_iter; ++iter) {
        float ms1x[8], ms1y[8], ms2x[8], ms2y[8];
        double F_norm[9], F_tmp[9], F_sac[9];
        unsigned int k;

        sv_create_random_array(min_set_size, 0U, num_matches - 1, engine, idxbuf);
        for (k = 0; k < min_set_size; ++k) {
            uint32_t idx = idxbuf[k];
            ms1x[k] = nx1[matches[idx].idx1]; ms1y[k] = ny1[matches[idx].idx1];
            ms2x[k] = nx2[matches[idx].idx2]; ms2y[k] = ny2[matches[idx].idx2];
        }
        sv_solve_compute_F21(ms1x, ms1y, ms2x, ms2y, (int)min_set_size, F_norm);
        sv_mat3_mul(transform2_t, F_norm, F_tmp);
        sv_mat3_mul(F_tmp, transform1, F_sac);

        {
            float cost_sac;
            unsigned int num_inliers = check_inliers_F(undist_1, undist_2, matches, num_matches,
                                                        F_sac, sigma, inlier_sac, &cost_sac);
            if (num_inliers > min_set_size && out->best_cost > cost_sac) {
                out->best_cost = cost_sac;
                memcpy(out->best_F21, F_sac, sizeof(F_sac));
                memcpy(out->is_inlier, inlier_sac, num_matches);
            }
        }
    }

    out->solution_valid = out->best_cost < FLT_MAX;

    if (recompute && out->solution_valid) {
        unsigned int n_inl = 0;
        float* ix1 = (float*)malloc(sizeof(float) * num_matches);
        float* iy1 = (float*)malloc(sizeof(float) * num_matches);
        float* ix2 = (float*)malloc(sizeof(float) * num_matches);
        float* iy2 = (float*)malloc(sizeof(float) * num_matches);
        double F_norm[9], F_tmp[9], F_sac[9];
        for (i = 0; i < num_matches; ++i) {
            if (out->is_inlier[i]) {
                ix1[n_inl] = nx1[matches[i].idx1]; iy1[n_inl] = ny1[matches[i].idx1];
                ix2[n_inl] = nx2[matches[i].idx2]; iy2[n_inl] = ny2[matches[i].idx2];
                n_inl++;
            }
        }
        sv_solve_compute_F21(ix1, iy1, ix2, iy2, (int)n_inl, F_norm);
        sv_mat3_mul(transform2_t, F_norm, F_tmp);
        sv_mat3_mul(F_tmp, transform1, F_sac);
        memcpy(out->best_F21, F_sac, sizeof(F_sac));
        {
            float cost;
            check_inliers_F(undist_1, undist_2, matches, num_matches, out->best_F21, sigma, out->is_inlier, &cost);
            out->best_cost = cost;
        }
        free(ix1); free(iy1); free(ix2); free(iy2);
    }

    free(nx1); free(ny1); free(nx2); free(ny2);
    free(px1); free(py1); free(px2); free(py2);
    free(inlier_sac);
}
