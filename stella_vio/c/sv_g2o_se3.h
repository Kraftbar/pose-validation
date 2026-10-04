/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_G2O_SE3_H
#define SV_G2O_SE3_H

#include "sv_eigen_quaternion.h"

/* g2o::SE3Quat and stella_vslam's se3::shot_vertex / landmark_vertex
 * (setToOriginImpl/oplusImpl) -- BSD (g2o's BSD-2 notice):
 *   external/candidates/g2o/g2o/types/slam3d/se3quat.h
 *     (SE3Quat::exp, SE3Quat::operator*, SE3Quat::map)
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/internal/
 *     se3/shot_vertex.h, landmark_vertex.h (oplusImpl)
 * Uses sv_eigen_quaternion.h (MPL-2.0) for the quaternion primitives and
 * sv_linalg.h (MPL-2.0) for the 3x3 mat/vec helpers (Omega, Omega^2, V).
 * Only the monocular/perspective path stella_vslam's pose optimizer and
 * local BA use is ported here (no equirectangular/stereo).
 */

typedef struct sv_se3 {
    sv_quat q;   /* rotation, cam_pose_cw convention (world -> camera) */
    double t[3]; /* translation */
} sv_se3;

/* SE3Quat::exp(update): update = [omega(3); upsilon(3)] (angle-axis,
 * translation-generator) six-vector, g2o's Lie-algebra tangent order. */
void sv_se3_exp(const double update[6], sv_se3* out);

/* SE3Quat::normalizeRotation(): sign-canonicalize (w>=0) then
 * unit-normalize the rotation quaternion in place -- called by every
 * SE3Quat constructor and by operator*; callers building an sv_se3
 * directly from a rotation matrix (e.g. util::converter::to_g2o_SE3's
 * `SE3Quat{rot,trans}`, as pose_optimizer_g2o.cc does for its initial
 * vertex estimate) MUST call this afterward too -- a Shoemake-converted
 * quaternion from a not-perfectly-orthogonal matrix is not bit-identical
 * to its normalized form, and every downstream computation depends on
 * having the SAME (normalized) quaternion g2o actually uses. */
void sv_se3_normalize_rotation(sv_se3* pose);

/* SE3Quat::operator*(tr2): compose so that (a*b).map(p) == a.map(b.map(p)). */
void sv_se3_compose(const sv_se3* a, const sv_se3* b, sv_se3* out);

/* SE3Quat::map: rotate+translate a world point into this frame. */
void sv_se3_map(const sv_se3* pose, const double p_w[3], double p_c[3]);

/* shot_vertex::oplusImpl: new = exp(update) * old. */
void sv_shot_vertex_oplus(const sv_se3* old_pose, const double update[6], sv_se3* out);

/* landmark_vertex::oplusImpl: new = old + update. */
void sv_landmark_vertex_oplus(const double old_pos[3], const double update[3], double out[3]);

#endif /* SV_G2O_SE3_H */
