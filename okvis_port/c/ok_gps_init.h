/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2-X GNSS initialisation helpers (ViGraph.cpp: umeyamaTransform, estimateRigidRansac and the numeric core of
 * ViGraph::checkForGpsInit), bit-exact C99 port.
 *
 * Derived from OKVIS2-X (ethz-mrl/OKVIS2-X, okvis_ceres/src/ViGraph.cpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich and ETH Zurich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions are MPL-2.0 (Eigen, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors); the
 *   std::mt19937 / std::uniform_int_distribution<int> models are written from the algorithm description of
 *   libstdc++ (GCC 13). The names of the copyright holders may not be used to endorse derived products.
 *
 * All 3-vector lists are flat arrays [x0 y0 z0 x1 ...], covariances are 3x3 column-major (9 doubles each).
 * Reference flags: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0 SSE2, GCC 13 libstdc++.
 * Verified by okvis_port/reference_tools/okvis_gps_init_test.cc (tolerance 0).
 */
#ifndef OK_GPS_INIT_H
#define OK_GPS_INIT_H

#include "ok_kin.h"

/* std::mt19937 (32-bit Mersenne twister, the seeding recurrence of the standard) */
typedef struct ok_mt19937 { unsigned int mt[624]; int idx; } ok_mt19937;
void ok_mt19937_seed(ok_mt19937* g, unsigned int seed);
unsigned int ok_mt19937_next(ok_mt19937* g);
/* std::uniform_int_distribution<int>(a, b)(g) as libstdc++ 13 evaluates it for mt19937 (b >= a, b - a < 2^32 - 1) */
int ok_uniform_int(ok_mt19937* g, int a, int b);

/* Eigen building blocks (exposed for the oracle) */
void ok_gps_inverse3(const double m[9], double out[9]);   /* Matrix3d::inverse() (cofactors, size-3 helper) */
void ok_gps_inverse4(const double m[16], double out[16]); /* Matrix4d::inverse() (SSE2 Packet2d path) */

/* kinematics::Transformation umeyamaTransform(gps, world): closed-form yaw + centroid translation.
 * n < 3 returns the identity (the C++ logs an error); returns 0 on success, 1 for that error. C = block verbatim. */
int ok_gps_umeyama(int n, const double* gps, const double* world, ok_tf* T);

/* RigidResult estimateRigidRansac(gps, world, iterations, n_points, inlierThreshold, requiredInlierRatio).
 * inliers_out (n ints, may be NULL) receives the inlier indices of the best model. */
typedef struct ok_rigid_result { double R[9]; double t[3]; double inlier_ratio; int n_inliers; } ok_rigid_result;
void ok_gps_estimate_rigid_ransac(int n, const double* gps, const double* world, int iterations, int n_points,
                                  double inlier_threshold, double required_inlier_ratio, ok_rigid_result* out,
                                  int* inliers_out);

/* The 4x4 information of (translation, yaw): sum_i Ei^T * cov_i^-1 * Ei, Ei = [-I | crossMx(C*world_i).col(2)]. */
void ok_gps_yaw_hessian(int n, const double* world, const double* cov, const double C[9], double Hess[16]);

/* Numeric core of ViGraph::checkForGpsInit, after the propagated points have been gathered.
 *   robust != 0: estimateRigidRansac(.., 20, 20, 4.0, 0.7); inlier_ratio < 0.25 returns 1 (T_GW untouched);
 *                otherwise T_GW.set(R, t) (rotation re-derived through the quaternion);
 *   robust == 0: T_GW = umeyamaTransform.
 * Then the Hessian, Hess.inverse() and yaw sigma = sqrt(P(3,3)) / pi * 180 (degrees, *yaw_error_deg).
 * Returns 0 when the yaw sigma was computed (the caller compares it with yawErrorThreshold; the robust path then
 * continues with the Ceres refinement, not part of this module), 1 on the RANSAC rejection.
 * ransac_ratio (may be NULL) receives the best inlier ratio in the robust path. */
int ok_gps_init_core(int n, const double* gps, const double* world, const double* cov, int robust, ok_tf* T_GW,
                     double* yaw_error_deg, double* ransac_ratio);

#endif
