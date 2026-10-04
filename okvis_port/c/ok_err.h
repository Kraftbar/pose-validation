/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 3b: error terms (okvis_ceres: ReprojectionError<PinholeCamera<...>>, PoseError,
 * SpeedAndBiasError, RelativePoseError, HomogeneousPointError) including their information / square-root-information
 * (LLT) set-up, residuals and Jacobians (full and minimal).
 *
 * Derived from OKVIS2 (okvis_ceres/include/okvis/ceres/{ReprojectionError,PoseError,SpeedAndBiasError,RelativePoseError,
 * HomogeneousPointError}.hpp, implementation/ReprojectionError.hpp, src/<same names>.cpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions (small lazy products, GEBP/GEMV kernels, LLT, redux) are MPL-2.0 (Eigen, Copyright (C) Gael Guennebaud,
 *   Benoit Jacob and the Eigen authors). Redistribution requires retaining these notices; the names of ETH Zurich,
 *   Imperial College London and TUM may not be used to endorse derived products.
 *
 * C99, <math.h> <stdint.h> <stdlib.h> <string.h> only. All matrices stored in the structs are COLUMN-major (m[row +
 * nrows*col]); Jacobian output buffers are ROW-major like the Eigen::RowMajor maps of the C++ code. `jac` /
 * `jacmin` mimic `double** jacobians` / `double** jacobiansMinimal`: NULL = nullptr, entries may be NULL too. The C++
 * code writes a minimal Jacobian only inside the `jacobians[k] != nullptr` branch (SpeedAndBiasError: independently);
 * the C code does exactly the same.
 * ReprojectionError: the C++ code ignores the ProjectionStatus of projectHomogeneous, and when the point is
 * degenerate (|z| < 1e-12, status Invalid) kp / Jh stay UNINITIALISED there; the C code zero-fills them (the
 * reference dumps and the random tests exclude that case). `covariance_` (information.inverse()) is never read by the
 * pipeline and is not computed.
 * Not ported: DepthError (depth cameras only, never constructed by the pipeline), the numeric-diff checker
 * jacobiansCorrect, ReprojectionError over RadialTangential8 (see ok_cam.h), TwoPoseGraphError (module 5).
 *
 * ---- record layouts of the reference dumps (patch 0007; native endian; f64/u32) ----
 * Evaluate records end with the shared tail (nb parameter blocks, sizes sz[], residual dim nres, minimal sizes msz[]):
 *   u32 nb, nb x {u32 size, f64 params[size]}, u32 have_jac, nb x {u32 jac_nonnull, u32 have_jacmin, u32 jacmin_nonnull},
 *   u32 return, f64 residuals[nres], for every block with jac_nonnull: f64 J[nres*size] (row-major),
 *   for every block with a written minimal Jacobian (jacmin_nonnull && (jac_nonnull || SpeedAndBiasError)): f64 Jm[nres*msz] (row-major)
 *   err_reproj.bin : cam header (ok_cam.h), f64 meas[2], f64 information_[4] (row-major), f64 sqrt_info[4] (row-major), tail
 *   err_pose.bin   : f64 meas coeffs[7] (r, q xyzw), f64 sqrt_info[36] (row-major), tail
 *   err_sab.bin    : f64 meas[9], f64 sqrt_info[81] (row-major), tail
 *   err_relpose.bin: f64 T_AB coeffs[7], f64 sqrt_info[36] (row-major), tail
 *   err_hpoint.bin : f64 meas[4], f64 sqrt_info[9] (row-major), tail
 *   err_llt.bin    : u32 n, f64 information[n*n] (row-major), f64 squareRootInformation[n*n] (row-major)   setInformation
 *   err_ctor.bin   : u32 tag, u32 nin, f64 in[nin], u32 n, f64 information_[n*n], f64 squareRootInformation_[n*n] (row-major)
 *                    tag 0 PoseError(T, diag6): in = T coeffs[7], diag[6]
 *                    tag 1 PoseError(T, translationVariance, rotationVariance): in = tv, rv
 *                    tag 2 SpeedAndBiasError(sb, speedVar, gyrVar, accVar): in = sb[9], sv, gv, av
 *                    tag 3 RelativePoseError(translationVariance, rotationVariance, T_AB): in = tv, rv
 *                    tag 4 HomogeneousPointError(m, variance): in = m[4], variance
 */
#ifndef OK_ERR_H
#define OK_ERR_H
#include "ok_cam.h"
#include "ok_kin.h"

/* Eigen::LLT<Matrix<double,n,n>>(info) followed by matrixL().transpose(): out = L^T (upper triangle, zeros below).
 * n <= 9 (unblocked path). Returns -1 on success, else the index k of the failing pivot (out then holds the partially
 * factorised matrix exactly like Eigen's m_matrix). info / out column-major n*n; out may alias info. */
int ok_llt_sqrt_information(int n, const double* info, double* out);

/* ---- Eigen small lazy-product models shared with the later modules (ok_twopose.c, ok_graph.c); see the header
 * comment of ok_err.c for the measured cases A/B/C/D. Column-major unless stated; D <= 8. ---- */
double ok_red_tree(const double* p, int n);                 /* redux_novec_unroller halving tree */
void ok_red_vec_lanes(const double* p, int np, double lane[2]);
double ok_red_vec(const double* p, int n);                  /* vectorised redux (2 lanes, horizontal add, odd tail) */
void ok_lazy_a(int R, int C, int D, const double* A, const double* B, double* out);        /* A) lhs col-major */
void ok_lazy_tree_rm(int R, int C, int D, const double* A, const double* B, double* out);  /* B) row-major, tree */
void ok_lazy_c(int R, int C, int D, const double* A, const double* B, double* out);        /* C) lhs row-major */
void ok_lazy_tree(int R, int C, int D, const double* A, const double* B, double* out);     /* D) col-major, tree */

/* ---------------------------------------------- ReprojectionError ---------------------------------------------- */
typedef struct ok_reproj_err {
    ok_cam cam;
    double meas[2];
    double info[4];       /* information_, 2x2 */
    double sqrt_info[4];  /* squareRootInformation_ */
} ok_reproj_err;
/* ctor (cameraGeometry, cameraId, measurement, information) == setCameraGeometry + setMeasurement + setInformation */
void ok_reproj_err_init(ok_reproj_err* e, const ok_cam* cam, const double meas[2], const double info[4]);
void ok_reproj_err_set_information(ok_reproj_err* e, const double info[4]);
/* params: pose[7] (T_WS), homogeneous point[4] (hp_W), extrinsics[7] (T_SC); residuals[2];
 * jac[3] row-major 2x7, 2x4, 2x7; jacmin[3] row-major 2x6, 2x3, 2x6. Returns 1. */
int ok_reproj_err_evaluate(const ok_reproj_err* e, const double* const params[3], double res[2],
                           double* const* jac, double* const* jacmin);

/* ------------------------------------------------- PoseError --------------------------------------------------- */
typedef struct ok_pose_err {
    ok_tf meas;           /* cached Transformation */
    double info[36];
    double sqrt_info[36];
} ok_pose_err;
void ok_pose_err_init_info(ok_pose_err* e, const ok_tf* T, const double info[36]);
void ok_pose_err_init_diag(ok_pose_err* e, const ok_tf* T, const double diag[6]);
void ok_pose_err_init_var(ok_pose_err* e, const ok_tf* T, double translation_variance, double rotation_variance);
void ok_pose_err_set_information(ok_pose_err* e, const double info[36]);
/* params: pose[7]; residuals[6]; jac[1] row-major 6x7; jacmin[1] 6x6 */
int ok_pose_err_evaluate(const ok_pose_err* e, const double* const params[1], double res[6], double* const* jac,
                         double* const* jacmin);

/* ---------------------------------------------- SpeedAndBiasError ---------------------------------------------- */
typedef struct ok_sab_err {
    double meas[9];
    double info[81];
    double sqrt_info[81];
} ok_sab_err;
void ok_sab_err_init_info(ok_sab_err* e, const double meas[9], const double info[81]);
void ok_sab_err_init_var(ok_sab_err* e, const double meas[9], double speed_var, double gyr_bias_var, double acc_bias_var);
void ok_sab_err_set_information(ok_sab_err* e, const double info[81]);
/* params: speed-and-bias[9]; residuals[9]; jac[1] and jacmin[1] row-major 9x9 */
int ok_sab_err_evaluate(const ok_sab_err* e, const double* const params[1], double res[9], double* const* jac,
                        double* const* jacmin);

/* ---------------------------------------------- RelativePoseError ---------------------------------------------- */
typedef struct ok_relpose_err {
    ok_tf T_AB;           /* cached Transformation */
    double info[36];
    double sqrt_info[36];
} ok_relpose_err;
void ok_relpose_err_init_info(ok_relpose_err* e, const double info[36], const ok_tf* T_AB);
void ok_relpose_err_init_var(ok_relpose_err* e, double translation_variance, double rotation_variance, const ok_tf* T_AB);
void ok_relpose_err_set_information(ok_relpose_err* e, const double info[36]);
/* params: T_WA[7], T_WB[7]; residuals[6]; jac[2] row-major 6x7; jacmin[2] 6x6 */
int ok_relpose_err_evaluate(const ok_relpose_err* e, const double* const params[2], double res[6], double* const* jac,
                            double* const* jacmin);

/* -------------------------------------------- HomogeneousPointError -------------------------------------------- */
typedef struct ok_hpoint_err {
    double meas[4];
    double info[9];
    double sqrt_info[9];
} ok_hpoint_err;
void ok_hpoint_err_init_info(ok_hpoint_err* e, const double meas[4], const double info[9]);
void ok_hpoint_err_init_var(ok_hpoint_err* e, const double meas[4], double variance);
void ok_hpoint_err_set_information(ok_hpoint_err* e, const double info[9]);
/* params: hp[4]; residuals[3]; jac[1] row-major 3x4; jacmin[1] 3x3 */
int ok_hpoint_err_evaluate(const ok_hpoint_err* e, const double* const params[1], double res[3], double* const* jac,
                           double* const* jacmin);
#endif
