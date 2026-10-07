/* SPDX-License-Identifier: BSD-3-Clause
 * Port of basalt-headers camera/double_sphere_camera.hpp (BSD-3-Clause, (c) 2019 Vladyslav Usenko, Nikolaus Demmel).
 * See bs_cam.h. The body is compiled twice (float, double) by including this file from itself. Every expression keeps the
 * C++ parenthesisation / literal types: Scalar(1), Scalar(0.5), `2 * mx` (int promoted to Scalar), sqrt overloaded on Scalar
 * (`using std::sqrt` in the header: sqrtf for float). */
#ifndef BS_CAM_BODY
#include <math.h>
#include "bs_cam.h"

void bs_ds_cast_f(bs_ds_f* out, const double p[6]) {
    int i;
    for (i = 0; i < 6; ++i) out->p[i] = (float)p[i];
}

#define BS_CAM_BODY 1
#define BS_T float
#define BS_SQRT sqrtf
#define BS_CAM bs_ds_f
#define BS_PROJECT bs_ds_project_f
#define BS_UNPROJECT bs_ds_unproject_f
#include "bs_cam.c"
#undef BS_T
#undef BS_SQRT
#undef BS_CAM
#undef BS_PROJECT
#undef BS_UNPROJECT
#define BS_T double
#define BS_SQRT sqrt
#define BS_CAM bs_ds_d
#define BS_PROJECT bs_ds_project_d
#define BS_UNPROJECT bs_ds_unproject_d
#include "bs_cam.c"

#else /* ---- template body ---- */

int BS_PROJECT(const BS_CAM* cam, const BS_T p3d[3], BS_T proj[2], BS_T* J3, BS_T* Jp) {
    const BS_T fx = cam->p[0], fy = cam->p[1], cx = cam->p[2], cy = cam->p[3], xi = cam->p[4], alpha = cam->p[5];
    const BS_T x = p3d[0], y = p3d[1], z = p3d[2];

    const BS_T xx = x * x;
    const BS_T yy = y * y;
    const BS_T zz = z * z;
    const BS_T r2 = xx + yy;
    const BS_T d1_2 = r2 + zz;
    const BS_T d1 = BS_SQRT(d1_2);

    const BS_T w1 = alpha > (BS_T)0.5 ? ((BS_T)1 - alpha) / alpha : alpha / ((BS_T)1 - alpha);
    const BS_T w2 = (w1 + xi) / BS_SQRT((BS_T)2 * w1 * xi + xi * xi + (BS_T)1);

    const int is_valid = (z > -w2 * d1);

    const BS_T k = xi * d1 + z;
    const BS_T kk = k * k;
    const BS_T d2_2 = r2 + kk;
    const BS_T d2 = BS_SQRT(d2_2);
    const BS_T norm = alpha * d2 + ((BS_T)1 - alpha) * k;
    const BS_T mx = x / norm;
    const BS_T my = y / norm;

    proj[0] = fx * mx + cx;
    proj[1] = fy * my + cy;

    if (J3) {
        const BS_T norm2 = norm * norm;
        const BS_T xy = x * y;
        const BS_T tt2 = xi * z / d1 + (BS_T)1;
        const BS_T d_norm_d_r2 = (xi * ((BS_T)1 - alpha) / d1 + alpha * (xi * k / d1 + (BS_T)1) / d2) / norm2;
        const BS_T tmp2 = (((BS_T)1 - alpha) * tt2 + alpha * k * tt2 / d2) / norm2;
        int i;
        for (i = 0; i < 8; ++i) J3[i] = 0;
#define J3_(r, c) J3[(r) + 2 * (c)]
        J3_(0, 0) = fx * ((BS_T)1 / norm - xx * d_norm_d_r2);
        J3_(1, 0) = -fy * xy * d_norm_d_r2;
        J3_(0, 1) = -fx * xy * d_norm_d_r2;
        J3_(1, 1) = fy * ((BS_T)1 / norm - yy * d_norm_d_r2);
        J3_(0, 2) = -fx * x * tmp2;
        J3_(1, 2) = -fy * y * tmp2;
#undef J3_
    }
    if (Jp) {
        const BS_T norm2 = norm * norm;
        const BS_T tmp4 = (alpha - (BS_T)1 - alpha * k / d2) * d1 / norm2;
        const BS_T tmp5 = (k - d2) / norm2;
        int i;
        for (i = 0; i < 12; ++i) Jp[i] = 0;
#define JP_(r, c) Jp[(r) + 2 * (c)]
        JP_(0, 0) = mx;
        JP_(0, 2) = (BS_T)1;
        JP_(1, 1) = my;
        JP_(1, 3) = (BS_T)1;
        JP_(0, 4) = fx * x * tmp4;
        JP_(1, 4) = fy * y * tmp4;
        JP_(0, 5) = fx * x * tmp5;
        JP_(1, 5) = fy * y * tmp5;
#undef JP_
    }
    return is_valid;
}

int BS_UNPROJECT(const BS_CAM* cam, const BS_T proj[2], BS_T p3d[4], BS_T* Jproj, BS_T* Jparam) {
    const BS_T fx = cam->p[0], fy = cam->p[1], cx = cam->p[2], cy = cam->p[3], xi = cam->p[4], alpha = cam->p[5];

    const BS_T mx = (proj[0] - cx) / fx;
    const BS_T my = (proj[1] - cy) / fy;
    const BS_T r2 = mx * mx + my * my;

    const int is_valid = !(alpha > (BS_T)0.5 && (r2 >= (BS_T)1 / ((BS_T)2 * alpha - (BS_T)1)));

    const BS_T xi2_2 = alpha * alpha;
    const BS_T xi1_2 = xi * xi;
    const BS_T sqrt2 = BS_SQRT((BS_T)1 - ((BS_T)2 * alpha - (BS_T)1) * r2);
    const BS_T norm2 = alpha * sqrt2 + (BS_T)1 - alpha;
    const BS_T mz = ((BS_T)1 - xi2_2 * r2) / norm2;
    const BS_T mz2 = mz * mz;
    const BS_T norm1 = mz2 + r2;
    const BS_T sqrt1 = BS_SQRT(mz2 + ((BS_T)1 - xi1_2) * r2);
    const BS_T k = (mz * xi + sqrt1) / norm1;

    p3d[0] = k * mx;
    p3d[1] = k * my;
    p3d[2] = k * mz - xi;
    p3d[3] = 0;

    if (Jproj || Jparam) {
        const BS_T norm2_2 = norm2 * norm2;
        const BS_T norm1_2 = norm1 * norm1;
        const BS_T d_mz_d_r2 = ((BS_T)0.5 * alpha - xi2_2) * (r2 * xi2_2 - (BS_T)1) / (sqrt2 * norm2_2) - xi2_2 / norm2;
        const BS_T d_mz_d_mx = (BS_T)2 * mx * d_mz_d_r2;
        const BS_T d_mz_d_my = (BS_T)2 * my * d_mz_d_r2;
        const BS_T d_k_d_mz = (norm1 * (xi * sqrt1 + mz) - (BS_T)2 * mz * (mz * xi + sqrt1) * sqrt1) / (norm1_2 * sqrt1);
        const BS_T d_k_d_r2 = (xi * d_mz_d_r2 + (BS_T)0.5 / sqrt1 * ((BS_T)2 * mz * d_mz_d_r2 + (BS_T)1 - xi1_2)) / norm1 -
                              (mz * xi + sqrt1) * ((BS_T)2 * mz * d_mz_d_r2 + (BS_T)1) / norm1_2;
        const BS_T d_k_d_mx = d_k_d_r2 * (BS_T)2 * mx;
        const BS_T d_k_d_my = d_k_d_r2 * (BS_T)2 * my;
        BS_T c0[4], c1[4];
        int i;

        c0[0] = (mx * d_k_d_mx + k);
        c0[1] = my * d_k_d_mx;
        c0[2] = (mz * d_k_d_mx + k * d_mz_d_mx);
        c0[3] = 0;
        for (i = 0; i < 4; ++i) c0[i] /= fx;

        c1[0] = mx * d_k_d_my;
        c1[1] = (my * d_k_d_my + k);
        c1[2] = (mz * d_k_d_my + k * d_mz_d_my);
        c1[3] = 0;
        for (i = 0; i < 4; ++i) c1[i] /= fy;

        if (Jproj) {
            for (i = 0; i < 4; ++i) { Jproj[i] = c0[i]; Jproj[4 + i] = c1[i]; }
        }
        if (Jparam) {
            const BS_T d_k_d_xi1 = (mz * sqrt1 - xi * r2) / (sqrt1 * norm1);
            const BS_T d_mz_d_xi2 = (((BS_T)1 - r2 * xi2_2) * (r2 * alpha / sqrt2 - sqrt2 + (BS_T)1) / norm2 - (BS_T)2 * r2 * alpha) / norm2;
            const BS_T d_k_d_xi2 = d_k_d_mz * d_mz_d_xi2;
            for (i = 0; i < 24; ++i) Jparam[i] = 0;
            for (i = 0; i < 4; ++i) {
                Jparam[0 * 4 + i] = -c0[i] * mx;
                Jparam[1 * 4 + i] = -c1[i] * my;
                Jparam[2 * 4 + i] = -c0[i];
                Jparam[3 * 4 + i] = -c1[i];
            }
            Jparam[4 * 4 + 0] = mx * d_k_d_xi1;
            Jparam[4 * 4 + 1] = my * d_k_d_xi1;
            Jparam[4 * 4 + 2] = mz * d_k_d_xi1 - 1;
            Jparam[5 * 4 + 0] = mx * d_k_d_xi2;
            Jparam[5 * 4 + 1] = my * d_k_d_xi2;
            Jparam[5 * 4 + 2] = mz * d_k_d_xi2 + k * d_mz_d_xi2;
        }
    }
    return is_valid;
}

#endif
