/* SV_PORT_SOURCES: check_sv_track_bow.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_tracking.c sv_kf_insert.c sv_landmark_descriptor.c sv_match_robust.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_linalg.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c
 * SPDX-License-Identifier: MIT
 *
 * Same replay as check_sv_track.c, on the opt-in fault-injection dumps
 * runs/stella_port/reference_dumps_force_bow/<seq>/ (reference patch 0010,
 * STELLA_PORT_FORCE_PATH=bow:5: every 5th frame skips the motion-model
 * tracker, so bow_match_based_track -- one frame per sequence in the canonical
 * dumps -- runs on ~150 / ~80 frames, followed by the normal local-map
 * tracking, keyframe decision and keyframe insertion). Generate with
 *   python3 tools/dump_stella_reference.py fr1_xyz fr1_desk --force-path bow:5
 * Prints 0/0 (skipped) when those dumps do not exist. */
#define SV_TRACK_DUMP_SUBST "reference_dumps_force_bow"
#define SV_TRACK_FORCE_PERIOD 5
#define SV_TRACK_FORCE_SKIP_BOW 0
#include "check_sv_track.c"
