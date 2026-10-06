/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M8-M11 support: rdvio::Solver (rdvio_estimation/src/solver.cpp) on the C map layer.
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; see rdvio_port/LICENSES and NOTICE). C99.
 *
 * A Solver collects parameter blocks and residual blocks exactly in the order of the C++ calls (AddParameterBlock /
 * AddResidualBlock = the Ceres program order; a residual that references a block not added yet appends it, as
 * ProblemImpl::AddResidualBlock does) and runs module M4 (rd_sv_solve). Parameter blocks are identified by the address of the
 * C state (Frame::pose.q / pose.p / motion.v / bg / ba, Track::landmark.inv_depth); the solution is written back into that
 * state. Factors are built at the call, with what the Ceres factor would read (rdvio_port/c/rd_solve.h payloads):
 *   ReprojectionError(frame, kp)   : this keypoint, the track's first keypoint, both cameras, frame->sqrt_inv_cov; Cauchy(1)
 *   ReprojectionPrior(frame, track): the same + live first-frame pose and inverse depth
 *   RotationPrior(frame, track)    : the same + live first-frame rotation
 *   PreIntegrationError(i, j, pre) : a copy of the preintegrator, both IMU extrinsics, live bg_i / ba_i
 *   PreIntegrationPrior(i, j, pre) : the same, live pose / motion of frame i
 *   Marginalization(factor)        : the map's factor (module M5), its linearization frames looked up in the map by id
 * Options are rdvio::Solver::solve's: SPARSE_SCHUR, DOGLEG, max_num_iterations = solver.iteration_limit, num_threads 1,
 * update_state_every_iteration, Ceres defaults otherwise. max_solver_time_in_seconds (wall clock) is not modelled.
 */
#ifndef RD_SOLVER_GLUE_H
#define RD_SOLVER_GLUE_H
#include "rd_map.h"
#include "rd_marg.h"
#include "rd_solve.h"

typedef struct rd_solver rd_solver;

rd_solver* rd_solver_create(int max_num_iterations);
void rd_solver_free(rd_solver* s);
void rd_solver_add_frame_states(rd_solver* s, rd_frame* f, int with_motion);
void rd_solver_add_track_states(rd_solver* s, rd_track* t);
void rd_solver_add_rpe(rd_solver* s, rd_frame* f, size_t kp);   /* frame->reprojection_error_factors[kp] (track = f->track[kp]) */
void rd_solver_add_rpp(rd_solver* s, rd_frame* f, rd_track* t);
void rd_solver_add_rop(rd_solver* s, rd_frame* f, rd_track* t);
void rd_solver_add_pie(rd_solver* s, rd_frame* fi, rd_frame* fj, const rd_preint* pre);
void rd_solver_add_pip(rd_solver* s, rd_frame* fi, rd_frame* fj, const rd_preint* pre);
/* the map's marginalization factor: frames of the factor are found in `map` by id (CeresMarginalizationFactor keeps Frame*) */
void rd_solver_add_marg(rd_solver* s, rd_marg* m, rd_map* map);
/* Solver::solve: returns IsSolutionUsable(). hooks may be NULL (a harness observes the solve through them) */
int rd_solver_solve(rd_solver* s, const rd_sv_hooks* hooks);
const rd_sv_problem* rd_solver_problem(const rd_solver* s);

#endif
