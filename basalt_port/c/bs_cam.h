/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt double-sphere camera model (basalt-headers DoubleSphereCamera, BSD-3-Clause, (c) 2019 Usenko, Demmel),
 * C99 port, bit-exact against g++ -O2 -ffp-contract=off -fno-fast-math (SSE, no FMA).
 *
 * Executed path (basalt_port/PLAN.md section 2, checked by grep over src/ + include/): the estimator and the frontend run
 * with Scalar = float (calibration is cast with `calib.cast<float>()` = round-to-nearest of the double parameters).
 *   project   : ba_utils.h linearizePoint -> cam.project(Vec4 p_t_3d, Vec2 res, Mat24* Jp)   (Jp has a zero 4th column,
 *               no intrinsics Jacobian), also with Jp == nullptr (computeError / computeProjections)
 *   unproject : sqrt_keypoint_vio.cpp measure(): Vec2 -> Vec4, no Jacobian;
 *               frontend epipolar check: Vec2f -> Vec4f through GenericCamera::unproject on the float calibration, no Jacobian.
 * The Jacobian variants of unproject / the intrinsics Jacobian of project are ported and tested for completeness (not on the path).
 * A double instantiation is provided too (same source, compiled twice); only the float one is on the VIO path.
 *
 * Layouts: parameters p[6] = {fx, fy, cx, cy, xi, alpha}; matrices column-major like Eigen:
 *   d_proj_d_p3d  2x4 (col 3 = 0), d_proj_d_param 2x6, d_p3d_d_proj 4x2, d_p3d_d_param 4x6.
 * Return value = is_valid of the C++ function (all outputs are written regardless).
 */
#ifndef BS_CAM_H
#define BS_CAM_H

typedef struct bs_ds_f { float p[6]; } bs_ds_f;
typedef struct bs_ds_d { double p[6]; } bs_ds_d;

/* calib.cast<float>(): each double parameter rounded to float */
void bs_ds_cast_f(bs_ds_f* out, const double p[6]);

/* p3d[0..2] used (a Vec4 argument's 4th component is ignored); Jacobian pointers may be NULL */
int bs_ds_project_f(const bs_ds_f* cam, const float p3d[3], float proj[2], float* d_proj_d_p3d /*2x4*/, float* d_proj_d_param /*2x6*/);
int bs_ds_project_d(const bs_ds_d* cam, const double p3d[3], double proj[2], double* d_proj_d_p3d, double* d_proj_d_param);

/* p3d is a Vec4 (4th component set to 0) */
int bs_ds_unproject_f(const bs_ds_f* cam, const float proj[2], float p3d[4], float* d_p3d_d_proj /*4x2*/, float* d_p3d_d_param /*4x6*/);
int bs_ds_unproject_d(const bs_ds_d* cam, const double proj[2], double p3d[4], double* d_p3d_d_proj, double* d_p3d_d_param);

#endif
