/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_G2O_BA_H
#define SV_G2O_BA_H

#include "sv_g2o_se3.h"

/* g2o graph optimization core for stella_vslam's bundle adjusters: shot
 * (SE3Quat) + landmark (Vector3d, marginalized) vertices, binary monocular
 * reprojection edges with optional Huber kernel, BlockSolver_6_3 (Schur
 * complement over the marginalized landmarks, back-substitution), the
 * Levenberg-Marquardt loop of OptimizationAlgorithmLevenberg and the
 * SparseOptimizer::optimize() driver with stella_vslam's terminate_action
 * post-iteration action -- BSD (g2o BSD-2 notice; stella-vslam BSD-2, AIST
 * 2019 / stella-cv 2022):
 *   external/candidates/g2o/g2o/core/{block_solver.hpp,
 *     sparse_optimizer.cpp, optimization_algorithm_levenberg.cpp,
 *     optimization_algorithm_with_hessian.cpp, base_fixed_sized_edge.hpp,
 *     sparse_block_matrix.hpp, sparse_block_matrix_ccs.h,
 *     sparse_block_matrix_diagonal.h, robust_kernel_impl.cpp,
 *     sparse_optimizer_terminate_action.cpp}
 *   external/candidates/g2o/g2o/solvers/eigen/linear_solver_eigen.h
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/
 *     internal/{landmark_vertex.h, se3/shot_vertex.h,
 *     se3/perspective_reproj_edge.h, se3/reproj_edge_wrapper.h},
 *     terminate_action.cc
 * The reduced camera system is solved with Eigen's SimplicialLLT + AMD
 * ordering on the block pattern (sv_eigen_llt.h / sv_eigen_amd.h, MPL-2.0),
 * which is what g2o's LinearSolverEigen does (local BA and the
 * pose-optimizer). global_bundle_adjuster uses LinearSolverCSparse
 * upstream; this port deliberately uses the same SimplicialLLT path for it
 * (no CSparse), see HANDOVER.md.
 *
 * Bit-exactness notes (the Eigen evaluation-order rules this file
 * replicates -- SSE2 baseline, -ffp-contract=off) are documented next to
 * each kernel in sv_g2o_ba.c and in stella_port/HANDOVER.md.
 */

typedef struct sv_ba_vertex {
    unsigned int id;   /* g2o vertex id == creation order */
    int is_landmark;   /* landmark_vertex (marginalized) vs shot_vertex */
    int fixed;
    sv_se3 pose;       /* shot estimate (cam_pose_cw as SE3Quat) */
    double pos[3];     /* landmark estimate */
    /* internal (SparseOptimizer / BlockSolver state) */
    int hidx;          /* hessian index, -1 when fixed / not in the index mapping */
    int active;        /* has >= 1 level-0 edge (in _activeVertices) */
    sv_se3 pose_bak;   /* push()/pop() single-level backup */
    double pos_bak[3];
    double b[6];       /* vertex quadratic form b */
    double hessian[6][6]; /* shot: 6x6 block, landmark: top-left 3x3 */
} sv_ba_vertex;

typedef struct sv_ba_edge {
    int lm;   /* index (into g->v) of the landmark vertex (vertex 0) */
    int kf;   /* index (into g->v) of the shot vertex (vertex 1) */
    double obs[2];
    double info;        /* information = Identity * info (inv_sigma_sq, float widened) */
    double huber_delta; /* RobustKernelHuber delta (float widened); valid iff has_kernel */
    int has_kernel;
    int level;          /* 0 = active, 1 = outlier */
    double err[2];      /* stored _error (stale while the edge is inactive) */
} sv_ba_edge;

typedef struct sv_ba_iter {
    double chi2;   /* activeRobustChi2() after the iteration's post-iteration action */
    double lambda; /* OptimizationAlgorithmLevenberg::currentLambda() */
    int lev_iter;  /* levenbergIteration() */
    unsigned int flag; /* optimizer stop flag as seen after terminate_action */
} sv_ba_iter;

typedef struct sv_ba_graph {
    sv_ba_vertex* v;
    int nv, cap_v;
    sv_ba_edge* e;
    int ne, cap_e;
    double fx, fy, cx, cy;
    /* SparseOptimizer + terminate_action state */
    int* stop_flag;        /* forceStopFlag pointer (external flag, or &aux_flag once terminate_action installed it) */
    int aux_flag;
    double gain_threshold; /* terminate_action gain threshold */
    int stopped_by_terminate;
    double last_chi;
    /* OptimizationAlgorithmLevenberg state */
    double current_lambda, ni;
    int lev_iterations;
    /* recorder for the most recent optimize() call */
    sv_ba_iter* iters;
    int n_iters, cap_iters;
    /* active structure (rebuilt by sv_ba_initialize_optimization) */
    int* active_edges; /* indices into e[] in EdgeIDCompare order */
    int n_active_edges;
    int* ivmap;        /* vertex indices in hessian-index order */
    int n_ivmap;
    /* BlockSolver workspace */
    void* solver;
} sv_ba_graph;

void sv_ba_graph_init(sv_ba_graph* g);
void sv_ba_graph_free(sv_ba_graph* g);
/* Both return the new element's index. Vertex ids must be added in
 * increasing id order (they are: creation order). */
int sv_ba_add_shot_vertex(sv_ba_graph* g, unsigned int id, const sv_se3* pose, int fixed);
int sv_ba_add_landmark_vertex(sv_ba_graph* g, unsigned int id, const double pos[3], int fixed);
int sv_ba_add_edge(sv_ba_graph* g, int lm_vertex, int kf_vertex, const double obs[2], double info,
                   int has_kernel, double huber_delta);

/* SparseOptimizer::setForceStopFlag(flag) (may be NULL). */
void sv_ba_set_force_stop_flag(sv_ba_graph* g, int* flag);
/* SparseOptimizer::terminate() */
int sv_ba_terminate(const sv_ba_graph* g);

/* SparseOptimizer::initializeOptimization() (level 0), including the
 * terminate_action call for iteration -1 (which resets the stop flag). */
int sv_ba_initialize_optimization(sv_ba_graph* g);

/* SparseOptimizer::optimize(iterations) with OptimizationAlgorithmLevenberg
 * and terminate_action(gain_threshold). Returns the number of iterations
 * run (or -1 if the index mapping is empty). g->iters holds the per-
 * iteration record. */
int sv_ba_optimize(sv_ba_graph* g, int iterations);

/* Per-edge accessors matching mono_perspective_reproj_edge. */
double sv_ba_edge_chi2(const sv_ba_edge* e);
int sv_ba_edge_depth_positive(const sv_ba_graph* g, const sv_ba_edge* e);
/* SparseOptimizer::computeActiveErrors() */
void sv_ba_compute_active_errors(sv_ba_graph* g);
/* SparseOptimizer::activeRobustChi2() (uses stored errors) */
double sv_ba_active_robust_chi2(const sv_ba_graph* g);

/* --- kernels exposed for the Eigen shape self-tests --- */
/* xl-style y.segment<3> += A * x.segment<3> (A column-major 3x3). */
void sv_ba_k_axpy3(const double A[9], const double x[3], double y[3]);
/* y.segment<3> += A^T * x.segment<6>, A a 6x3 block [row][col]. */
void sv_ba_k_atxpy63(const double A[6][3], const double x[6], double y[3]);
/* BDinv(6x3) = B(6x3) * Dinv(3x3 col-major). */
void sv_ba_k_bdinv(const double B[6][3], const double Dinv[9], double out[6][3]);
/* y(6) += B(6x3) * d(3). */
void sv_ba_k_bb(const double B[6][3], const double d[3], double y[6]);
/* H(6x6) -= BD(6x3) * Bj(6x3)^T. */
void sv_ba_k_schur_sub(double H[6][6], const double BD[6][3], const double Bj[6][3]);

#endif /* SV_G2O_BA_H */
