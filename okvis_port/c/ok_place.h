/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 7d (part 2): the numerical parts of Frontend::verifyRecognisedPlace -- the pose refinement
 * (`quickSolver`: a ceres::Problem with one free pose block, constant extrinsics / landmarks, reprojection errors with
 * CauchyLoss(3), default ceres::Solver::Options except num_threads / max_num_iterations, i.e. LEVENBERG_MARQUARDT with
 * SPARSE_NORMAL_CHOLESKY) run through the module-4 solver, the information matrix H and the extra-outlier count, and the
 * descriptor distinctiveness statistic of the surviving matches (Eigen float expression).
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; Frontend.cpp verifyRecognisedPlace),
 * Ceres Solver 2.2.0 (BSD-3-Clause, Google Inc.: the Levenberg-Marquardt strategy of ok_solve.c) and, for the Eigen 3.4.0
 * evaluation-order models, MPL-2.0. Redistribution requires retaining these notices (okvis_port/NOTICE).
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- place.bin record 173 (patch 0014): one per verifyRecognisedPlace call ----
 *   u64 frame (current multiframe id), u64 old frame id, u32 minInliers, u32 exit code (0 too few matches / points, 1 < 7
 *   correspondences, 2 RANSAC rejected, 3 indistinctive descriptors, 4 refinement rejected, 5 accepted), u32 ctr, u32 number
 *   of landmark points, u32 correspondences, u32 RANSAC inliers, f64 avg, u32 have T0, f64 T0[7] (r, q xyzw: the RANSAC model
 *   as Transformation), u32 have T1, f64 T1[7] (after the Ceres solve), u32 have H, f64 H[36] (column-major),
 *   u32 additionalOutliers, u32 numFinalInliers, u32 ceres iterations (summary.iterations.size()), u32 ceres termination type,
 *   f64 initial cost, f64 final cost, u32 nlandmarks, nlandmarks x { u64 landmark id, f64 hp[4] } (ascending id: the landmarks
 *   of the old frame), u32 nmatches, nmatches x { u64 frame, u32 cam, u32 kp, u64 landmark id } (std::map order)
 */
#ifndef OK_PLACE_H
#define OK_PLACE_H
#include <stdint.h>
#include "ok_cam.h"

typedef struct ok_place_term {
    const ok_cam* cam;              /* camera geometry of the keypoint's camera */
    int cam_idx;
    double meas[2];                 /* keypoint position */
    double size;                    /* keypoint size (information 64 / size^2 * I) */
    uint64_t lm_id;                 /* landmark (blocks are shared between terms with the same id) */
    double hp[4];                   /* the landmark (homogeneous, old sensor frame) */
} ok_place_term;

typedef struct ok_place_refine_out {
    double T[7];                    /* the refined T_Sold_Snew (r, q xyzw, q normalised like Transformation(r, q)) */
    double H[36];                   /* column-major */
    int additional_outliers;
    int iterations, termination;    /* summary.iterations.size(), summary.termination_type */
    double initial_cost, final_cost;
} ok_place_refine_out;

/* the refinement + information of verifyRecognisedPlace: T0 = the RANSAC model (r, q xyzw), T_SC = the extrinsics of every camera
 * at the current state (r, q xyzw), max_iters = realtime_max_iterations. Returns 0 on success. */
int ok_place_refine(const ok_place_term* terms, int nterms, int ncam, const double T_SC[][7], const double T0[7], int max_iters,
                    ok_place_refine_out* out);

/* the distinctiveness statistic of one camera: the inlier matches' descriptors (n x 48 bytes) -> stdev.sum() times n;
 * Eigen::Matrix<float, Dynamic, 384> expression, see ok_place.c. Returns sum += float(n) * stdev.sum() as float. */
float ok_place_distinctiveness(const unsigned char* desc, int n);

#endif
