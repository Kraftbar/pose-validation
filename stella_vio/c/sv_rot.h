/* SPDX-License-Identifier: MIT */
/* stella_vio rotation-only two-view estimation ("R-frames", idea of RD-VIO, arXiv:2310.15072: a frame that mostly rotates is tracked by a
 * pure-rotation model instead of the map). Own implementation: predicted-window descriptor matching, minimal 2-pair rotation RANSAC on
 * bearing vectors, closed-form (Horn 1987 unit-quaternion) least-squares refit on the inliers. C99, libm only.
 * Rotations are ROW-major double[9]; R_ab maps a bearing of camera A to camera B: b = R_ab a (so R_cw(B) = R_ab R_cw(A) for a pure rotation). */
#ifndef SV_ROT_H
#define SV_ROT_H

#include "sv_track.h"

/* Least-squares rotation R (b_i ~ R a_i) of n unit-vector pairs (Horn); w may be NULL (all 1). Returns 0, -1 if degenerate. */
int sv_rot_fit(const double* a, const double* b, unsigned int n, double R[9]);

/* RANSAC over n bearing pairs: R_ab and the inlier flags (angular error < thr_rad). Deterministic (fixed LCG). Returns the inlier count. */
unsigned int sv_rot_ransac(const double* a, const double* b, unsigned int n, double thr_rad, unsigned int iters, double R_ab[9],
                           unsigned char* inl);

typedef struct sv_rot_result {
    unsigned int n_match, n_inlier;   /* matches of the second pass, inliers of the final fit */
    double R_ab[9];
    double parallax_med;              /* median angle between b and R_ab a over the inliers [rad]: ~0 for a pure rotation of a far scene */
} sv_rot_result;

/* Rotation from frame A to frame B: matches A's keypoints into B inside a window around the prediction R_pred (radius_px), RANSAC,
 * re-match with the estimate (radius 12 px), refit. Returns 1 iff n_inlier >= min_inl. */
int sv_rot_estimate(const sv_tr_config* cfg, const sv_tr_obs* A, const sv_tr_obs* B, const double R_pred[9], double radius_px,
                    unsigned int min_inl, sv_rot_result* out);

#endif
