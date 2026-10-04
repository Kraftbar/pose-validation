/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M2a: visual factors (rdvio_estimation/ceres/reprojection_factor.h: CeresReprojectionErrorFactor,
 * CeresReprojectionPriorFactor; rdvio_estimation/ceres/rotation_factor.h: CeresRotationPriorFactor) and the stereo.h helpers
 * (dproj_dp, apply_k, remove_k).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified.
 * Eigen 3.4.0 evaluation-order models: MPL-2.0. See rdvio_port/NOTICE. Column-major matrices; Jacobians are ROW-major rows x size
 * (the factors write them through Map<Matrix<2,N,RowMajor>>); quaternions (x,y,z,w).
 *
 * Dump record layouts (rdvio_port/reference/patches/0004-m2-factor-geometry-dump.patch), native endian, f64 unless noted:
 *   rpe.bin : u32 has_jac, u32 mask(5 bits), u32 via_prior, params q_tgt[4] p_tgt[3] q_ref[4] p_ref[3] inv_depth, z[3] (this frame's keypoint),
 *             z_ref[3], cam_ref q_cs[4] p_cs[3], cam_tgt q_cs[4] p_cs[3], sqrt_inv_cov[4] (col-major), residual[2],
 *             then for every set mask bit k (has_jac): 2 x size(k) doubles, row-major (sizes 4,3,4,3,1)
 *   rot.bin : u32 has_jac, q_tgt[4], q_ref[4], z[3], z_ref[3], cam_ref (7), cam_tgt (7), sqrt_inv_cov[4], residual[2], [has_jac: 8]
 */
#ifndef RD_FACTOR_H
#define RD_FACTOR_H
#include "rd_lie.h"

typedef struct rd_extrinsic { ok_quat q_cs; double p_cs[3]; } rd_extrinsic;

/* dproj_dp(p): 2x3 column-major */
void rd_dproj_dp(const double p[3], double out[6]);

/* CeresReprojectionErrorFactor::Evaluate. params: q_tgt(4) p_tgt(3) q_ref(4) p_ref(3) inv_depth(1). jac[k] may be NULL; otherwise it
 * receives the row-major 2 x size(k) Jacobian (sizes 4,3,4,3,1). sqrt_inv_cov is the 2x2 column-major Frame::sqrt_inv_cov.
 * z = the target frame's keypoint bearing (the factor's local tangent basis is built from it), z_ref = the reference observation. */
void rd_rpe_eval(const double z[3], const double z_ref[3], const rd_extrinsic* cam_ref, const rd_extrinsic* cam_tgt,
                 const double sqrt_inv_cov[4], const double* const params[5], double residual[2], double* jac[5]);

/* CeresRotationPriorFactor::Evaluate (one parameter block q_tgt; q_ref_center = reference frame pose.q). jac0: row-major 2x4 or NULL. */
void rd_rot_prior_eval(const double z[3], const double z_ref[3], const rd_extrinsic* cam_ref, const rd_extrinsic* cam_tgt,
                       const double sqrt_inv_cov[4], const ok_quat* q_ref_center, const double q_tgt[4], double residual[2], double* jac0);

#endif
