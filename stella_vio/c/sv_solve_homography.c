/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_solve_homography.h (BSD-2, AIST 2019 + stella-cv 2022; SVD via
 * sv_eigen_svd.h, MPL-2.0, used not reproduced). */
#include "sv_solve_homography.h"
#include "sv_solve_common.h"
#include "sv_linalg.h"
#include "sv_eigen_svd.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

int sv_solve_compute_H21(const float* x1, const float* y1,
                          const float* x2, const float* y2, int n,
                          double H21[9]) {
    int i;
    int N = 2 * n;
    double* A = (double*)malloc(sizeof(double) * (size_t)N * 9);
    double V[81], sv[9];
    int rank;
    double v[9];

#define AC(col, row) A[(col) * N + (row)]
    for (i = 0; i < n; ++i) {
        int r0 = 2 * i, r1 = 2 * i + 1;
        double X1 = (double)x1[i], Y1 = (double)y1[i];
        double X2 = (double)x2[i], Y2 = (double)y2[i];
        AC(0, r0) = 0.0;
        AC(1, r0) = 0.0;
        AC(2, r0) = 0.0;
        AC(3, r0) = -X1;
        AC(4, r0) = -Y1;
        AC(5, r0) = -1.0;
        AC(6, r0) = Y2 * X1;
        AC(7, r0) = Y2 * Y1;
        AC(8, r0) = Y2 * 1.0;

        AC(0, r1) = X1;
        AC(1, r1) = Y1;
        AC(2, r1) = 1.0;
        AC(3, r1) = 0.0;
        AC(4, r1) = 0.0;
        AC(5, r1) = 0.0;
        AC(6, r1) = -X2 * X1;
        AC(7, r1) = -X2 * Y1;
        AC(8, r1) = -X2 * 1.0;
    }
#undef AC

    sv_eigen_jacobisvd_Nx9_v(A, N, V, sv, &rank);
    free(A);

    if (rank < 8) {
        return 0;
    }
    /* v = V.col(8) */
    for (i = 0; i < 9; ++i) {
        v[i] = V[8 * 9 + i];
    }
    /* Mat33_t(v.data()) is column-major fill == v itself; H21 = that .transpose() */
    {
        int r, c;
        for (r = 0; r < 3; ++r) {
            for (c = 0; c < 3; ++c) {
                H21[c * 3 + r] = v[r * 3 + c];
            }
        }
    }
    return 1;
}

int sv_solve_homography_decompose(const double H21[9], const double cam1[9], const double cam2[9],
                                   double rots[8][9], double transes[8][3], double normals[8][3]) {
    double cam2_inv[9], tmp[9], A[9];
    double U[9], V[9], Vt[9], sv3[3];
    float d1, d2, d3;
    float s_f;
    double s_d;
    float aux_1, aux_3;
    float x1s[4], x3s[4];
    float aux_sin_theta, cos_theta;
    float aux_sin_thetas[4];
    float aux_sin_phi, cos_phi;
    float sin_phis[4];
    int i;

    sv_mat3_inverse(cam2, cam2_inv);
    sv_mat3_mul(cam2_inv, H21, tmp);
    { double H1[9]; memcpy(H1, tmp, sizeof(H1)); sv_mat3_mul(H1, cam1, A); }

    sv_eigen_jacobisvd_3x3(A, U, V, sv3);
    sv_mat3_transpose(V, Vt);

    d1 = (float)sv3[0];
    d2 = (float)sv3[1];
    d3 = (float)sv3[2];

    {
        float r1 = d1 / d2;
        float r2 = d2 / d3;
        if ((double)r1 < 1.0001 || (double)r2 < 1.0001) {
            return 0;
        }
    }

    {
        double detU = sv_mat3_det(U);
        double detVt = sv_mat3_det(Vt);
        double sprod = detU * detVt;
        s_f = (float)sprod;
        s_d = (double)s_f;
    }

    aux_1 = sqrtf((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    aux_3 = sqrtf((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    x1s[0] = aux_1; x1s[1] = aux_1; x1s[2] = -aux_1; x1s[3] = -aux_1;
    x3s[0] = aux_3; x3s[1] = -aux_3; x3s[2] = aux_3; x3s[3] = -aux_3;

    aux_sin_theta = sqrtf((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
    cos_theta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    aux_sin_thetas[0] = aux_sin_theta;
    aux_sin_thetas[1] = -aux_sin_theta;
    aux_sin_thetas[2] = -aux_sin_theta;
    aux_sin_thetas[3] = aux_sin_theta;

    for (i = 0; i < 4; ++i) {
        double aux_rot[9];
        double sU[9], sUaux[9], init_rot[9];
        double aux_trans[3], diff_d;
        double init_trans[3], nrm;
        double aux_normal[3], init_normal[3];

        memset(aux_rot, 0, sizeof(aux_rot));
        aux_rot[0 * 3 + 0] = 1.0;
        aux_rot[1 * 3 + 1] = 1.0;
        aux_rot[2 * 3 + 2] = 1.0;
        aux_rot[0 * 3 + 0] = (double)cos_theta;
        aux_rot[2 * 3 + 0] = -(double)aux_sin_thetas[i]; /* (0,2) */
        aux_rot[0 * 3 + 2] = (double)aux_sin_thetas[i]; /* (2,0) */
        aux_rot[2 * 3 + 2] = (double)cos_theta;

        {
            int k;
            for (k = 0; k < 9; ++k) sU[k] = s_d * U[k];
        }
        sv_mat3_mul(sU, aux_rot, sUaux);
        sv_mat3_mul(sUaux, Vt, init_rot);
        memcpy(rots[i], init_rot, sizeof(init_rot));

        diff_d = (double)(d1 - d3);
        aux_trans[0] = (double)x1s[i] * diff_d;
        aux_trans[1] = 0.0 * diff_d;
        aux_trans[2] = (double)(-x3s[i]) * diff_d;
        sv_mat3_mulv(U, aux_trans, init_trans);
        nrm = sv_vec3_norm(init_trans);
        transes[i][0] = init_trans[0] / nrm;
        transes[i][1] = init_trans[1] / nrm;
        transes[i][2] = init_trans[2] / nrm;

        aux_normal[0] = (double)x1s[i];
        aux_normal[1] = 0.0;
        aux_normal[2] = (double)x3s[i];
        sv_mat3_mulv(V, aux_normal, init_normal);
        if (init_normal[2] < 0) {
            init_normal[0] = -init_normal[0];
            init_normal[1] = -init_normal[1];
            init_normal[2] = -init_normal[2];
        }
        normals[i][0] = init_normal[0];
        normals[i][1] = init_normal[1];
        normals[i][2] = init_normal[2];
    }

    aux_sin_phi = sqrtf((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
    cos_phi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    sin_phis[0] = aux_sin_phi;
    sin_phis[1] = -aux_sin_phi;
    sin_phis[2] = -aux_sin_phi;
    sin_phis[3] = aux_sin_phi;

    for (i = 0; i < 4; ++i) {
        double aux_rot[9];
        double sU[9], sUaux[9], init_rot[9];
        double aux_trans[3], sum_d;
        double init_trans[3], nrm;
        double aux_normal[3], init_normal[3];

        memset(aux_rot, 0, sizeof(aux_rot));
        aux_rot[0 * 3 + 0] = (double)cos_phi;
        aux_rot[2 * 3 + 0] = (double)sin_phis[i]; /* (0,2) */
        aux_rot[1 * 3 + 1] = -1.0;
        aux_rot[0 * 3 + 2] = (double)sin_phis[i]; /* (2,0) */
        aux_rot[2 * 3 + 2] = -(double)cos_phi;

        {
            int k;
            for (k = 0; k < 9; ++k) sU[k] = s_d * U[k];
        }
        sv_mat3_mul(sU, aux_rot, sUaux);
        sv_mat3_mul(sUaux, Vt, init_rot);
        memcpy(rots[4 + i], init_rot, sizeof(init_rot));

        sum_d = (double)(d1 + d3);
        aux_trans[0] = (double)x1s[i] * sum_d;
        aux_trans[1] = (double)0.0f * sum_d;
        aux_trans[2] = (double)x3s[i] * sum_d;
        sv_mat3_mulv(U, aux_trans, init_trans);
        nrm = sv_vec3_norm(init_trans);
        transes[4 + i][0] = init_trans[0] / nrm;
        transes[4 + i][1] = init_trans[1] / nrm;
        transes[4 + i][2] = init_trans[2] / nrm;

        aux_normal[0] = (double)x1s[i];
        aux_normal[1] = 0.0;
        aux_normal[2] = (double)x3s[i];
        sv_mat3_mulv(V, aux_normal, init_normal);
        if (init_normal[2] < 0) {
            init_normal[0] = -init_normal[0];
            init_normal[1] = -init_normal[1];
            init_normal[2] = -init_normal[2];
        }
        normals[4 + i][0] = init_normal[0];
        normals[4 + i][1] = init_normal[1];
        normals[4 + i][2] = init_normal[2];
    }

    return 1;
}

static unsigned int check_inliers_H(const sv_keypoint* undist_1, const sv_keypoint* undist_2,
                                     const sv_match_pair* matches, unsigned int num_matches,
                                     const double H21[9], float sigma,
                                     unsigned char* is_inlier, float* cost_out) {
    unsigned int i, num_inliers = 0;
    double H12[9];
    const float chi_sq = 5.991f;
    float sigma_sq = sigma * sigma;
    float cost = 0.0f;

    sv_mat3_inverse(H21, H12);

    for (i = 0; i < num_matches; ++i) {
        const sv_keypoint* k1 = &undist_1[matches[i].idx1];
        const sv_keypoint* k2 = &undist_2[matches[i].idx2];
        double pt1[3], pt2[3];
        double tp1[3], tp2[3];
        float dsq1, dsq2, dsq;
        double thr;

        pt1[0] = (double)k1->x; pt1[1] = (double)k1->y; pt1[2] = 1.0;
        pt2[0] = (double)k2->x; pt2[1] = (double)k2->y; pt2[2] = 1.0;

        sv_mat3_mulv(H21, pt1, tp1);
        tp1[0] = tp1[0] / tp1[2]; tp1[1] = tp1[1] / tp1[2]; tp1[2] = tp1[2] / tp1[2];
        {
            double dx = pt2[0] - tp1[0], dy = pt2[1] - tp1[1], dz = pt2[2] - tp1[2];
            dsq1 = (float)(dx * dx + dy * dy + dz * dz);
        }

        sv_mat3_mulv(H12, pt2, tp2);
        tp2[0] = tp2[0] / tp2[2]; tp2[1] = tp2[1] / tp2[2]; tp2[2] = tp2[2] / tp2[2];
        {
            double dx = pt1[0] - tp2[0], dy = pt1[1] - tp2[1], dz = pt1[2] - tp2[2];
            dsq2 = (float)(dx * dx + dy * dy + dz * dz);
        }

        dsq = dsq1 > dsq2 ? dsq1 : dsq2;
        thr = (double)(chi_sq * sigma_sq);
        if (thr > (double)dsq) {
            is_inlier[i] = 1;
            cost = cost + dsq;
            num_inliers++;
        }
        else {
            is_inlier[i] = 0;
            cost = (float)(cost + thr);
        }
    }
    *cost_out = cost;
    return num_inliers;
}

void sv_solve_homography_find_via_ransac(
    const sv_keypoint* undist_1, unsigned int num_kp1,
    const sv_keypoint* undist_2, unsigned int num_kp2,
    const sv_match_pair* matches, unsigned int num_matches,
    float sigma, unsigned int max_num_iter, int recompute,
    sv_mt19937* engine,
    sv_homography_ransac_result* out) {
    unsigned int i;
    const unsigned int min_set_size = 4;
    /* normalize() runs over the FULL undist_keypts_1_/2_ arrays (every
     * keypoint in the frame), not just the matched subset -- see
     * solve/homography_solver.cc find_via_ransac() step 0. Minimal sets
     * and check_inliers then index into these full normalized arrays via
     * matches_12_.at(idx).first/second (original keypoint indices). */
    float* px1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* py1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* px2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* py2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* nx1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* ny1 = (float*)malloc(sizeof(float) * (num_kp1 ? num_kp1 : 1));
    float* nx2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    float* ny2 = (float*)malloc(sizeof(float) * (num_kp2 ? num_kp2 : 1));
    double transform1[9], transform2[9], transform2_inv[9];
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
    sv_mat3_inverse(transform2, transform2_inv);

    if (num_matches < min_set_size * 2) {
        free(nx1); free(ny1); free(nx2); free(ny2);
        free(px1); free(py1); free(px2); free(py2);
        free(inlier_sac);
        return;
    }

    for (iter = 0; iter < max_num_iter; ++iter) {
        float ms1x[4], ms1y[4], ms2x[4], ms2y[4];
        double H_norm[9], H_tmp[9], H_sac[9];
        int ok;
        unsigned int k;

        sv_create_random_array(min_set_size, 0U, num_matches - 1, engine, idxbuf);
        for (k = 0; k < min_set_size; ++k) {
            uint32_t idx = idxbuf[k];
            ms1x[k] = nx1[matches[idx].idx1]; ms1y[k] = ny1[matches[idx].idx1];
            ms2x[k] = nx2[matches[idx].idx2]; ms2y[k] = ny2[matches[idx].idx2];
        }
        ok = sv_solve_compute_H21(ms1x, ms1y, ms2x, ms2y, (int)min_set_size, H_norm);
        if (!ok) {
            continue;
        }
        sv_mat3_mul(transform2_inv, H_norm, H_tmp);
        sv_mat3_mul(H_tmp, transform1, H_sac);

        {
            float cost_sac;
            unsigned int num_inliers = check_inliers_H(undist_1, undist_2, matches, num_matches,
                                                        H_sac, sigma, inlier_sac, &cost_sac);
            if (num_inliers > min_set_size && out->best_cost > cost_sac) {
                out->best_cost = cost_sac;
                memcpy(out->best_H21, H_sac, sizeof(H_sac));
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
        double H_norm[9], H_tmp[9], H_sac[9];
        int ok;
        for (i = 0; i < num_matches; ++i) {
            if (out->is_inlier[i]) {
                ix1[n_inl] = nx1[matches[i].idx1]; iy1[n_inl] = ny1[matches[i].idx1];
                ix2[n_inl] = nx2[matches[i].idx2]; iy2[n_inl] = ny2[matches[i].idx2];
                n_inl++;
            }
        }
        ok = sv_solve_compute_H21(ix1, iy1, ix2, iy2, (int)n_inl, H_norm);
        if (ok) {
            sv_mat3_mul(transform2_inv, H_norm, H_tmp);
            sv_mat3_mul(H_tmp, transform1, H_sac);
            memcpy(out->best_H21, H_sac, sizeof(H_sac));
            {
                float cost;
                check_inliers_H(undist_1, undist_2, matches, num_matches, out->best_H21, sigma, out->is_inlier, &cost);
                out->best_cost = cost;
            }
        }
        free(ix1); free(iy1); free(ix2); free(iy2);
    }

    free(nx1); free(ny1); free(nx2); free(ny2);
    free(px1); free(py1); free(px2); free(py2);
    free(inlier_sac);
    (void)idxbuf;
}
