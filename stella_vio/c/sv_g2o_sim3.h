/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_G2O_SIM3_H
#define SV_G2O_SIM3_H

#include "sv_sim3.h"

/* g2o optimization core for stella_vslam's two Sim3 problems -- BSD (g2o BSD-2
 * notice; stella-vslam BSD-2, AIST 2019 / stella-cv 2022):
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/
 *     graph_optimizer.cc            (pose graph over 7-dof shot vertices, binary graph_opt_edge,
 *                                    terminate_action(1e-3), optimize(50))
 *     transform_optimizer.cc        (one 7-dof transform_vertex, unary forward/backward reprojection
 *                                    edges with Huber kernel, optimize(5) + optimize(num_iter))
 *     internal/sim3/{shot_vertex.h, transform_vertex.h, graph_opt_edge.h,
 *       forward_reproj_edge.h, backward_reproj_edge.h, mutual_reproj_edge_wrapper.h}
 *   external/candidates/g2o/g2o/core/{base_fixed_sized_edge.hpp (numeric linearizeOplus:
 *     central differences with delta = 1e-9, constructQuadraticForm), block_solver.hpp (no landmarks:
 *     the Schur copy of Hpp), sparse_optimizer.cpp, optimization_algorithm_levenberg.cpp,
 *     robust_kernel_impl.cpp}
 *   external/candidates/g2o/g2o/solvers/eigen/linear_solver_eigen.h
 * The normal equations are solved with Eigen's SimplicialLLT + AMD (sv_eigen_llt / sv_eigen_amd, MPL-2.0),
 * i.e. g2o's LinearSolverEigen. graph_optimizer.cc uses LinearSolverCSparse upstream (LGPL, not ported);
 * the reference offers STELLA_PORT_EIGEN_SOLVER=1 (patch 0013) so that the port can be checked bit for bit.
 *
 * Eigen evaluation orders reproduced (measured against real Eigen 3.4, eigen_shape_tests_sim3opt.cc):
 *   7-term dot / chi2 of a 7-vector : ((p0+(p2+p4))+(p1+(p3+p5)))+p6
 *   b += A^T*we  (A 7x7)            : b + ((p0+(p2+p4))+(p1+(p3+p5)))+p6)
 *   H += AtO*A   (7x7)              : rows 0..5 H + ((((((p0+p1)+p2)+p3)+p4)+p5)+p6), row 6 H + ((p0+(p1+p2))+((p3+p4)+(p5+p6)))
 *   hessian^T += B^T*AtO^T          : every element H + ((p0+(p1+p2))+((p3+p4)+(p5+p6)))
 *   2-dim errors                    : depth-2 rule  dst + (p0+p1)
 */

typedef enum {
    SV_S3_EDGE_GRAPH = 0,    /* graph_opt_edge: binary, 7-dim error, information = I7, no kernel */
    SV_S3_EDGE_FORWARD = 1,  /* perspective_forward_reproj_edge: unary on the transform vertex */
    SV_S3_EDGE_BACKWARD = 2  /* perspective_backward_reproj_edge */
} sv_s3_edge_kind;

typedef struct sv_s3_vertex {
    unsigned int id;
    int fixed;
    sv_sim3 est, bak;
    int hidx;
    int active;
    double b[7];
    double H[7][7];
} sv_s3_vertex;

typedef struct sv_s3_edge {
    int kind;
    int v0, v1;         /* vertex indices; v1 = -1 for unary edges */
    sv_sim3 meas;       /* GRAPH: measurement */
    double obs[2];      /* reprojection: measurement */
    double info;        /* reprojection: information = Identity * info */
    double fx, fy, cx, cy;
    double pos_w[3];
    double rot[9];      /* FORWARD: rot_2w, BACKWARD: rot_1w (column-major) */
    double trans[3];
    int has_kernel;
    double huber_delta;
    int level;          /* 0 = active */
    double err[7];      /* stored _error (stale while inactive) */
} sv_s3_edge;

typedef struct sv_s3_iter {
    double chi2;   /* activeRobustChi2() after the iteration's post-iteration action */
    double lambda;
    int lev_iter;
    unsigned int flag;
} sv_s3_iter;

typedef struct sv_s3_graph {
    sv_s3_vertex* v;
    int nv, cap_v;
    sv_s3_edge* e;
    int ne, cap_e;
    int fix_scale;        /* vertex->fix_scale_ (update(6) = 0) */
    /* terminate_action */
    int use_terminate;
    double gain_threshold;
    int aux_flag;
    int stopped_by_terminate;
    double last_chi;
    /* OptimizationAlgorithmLevenberg */
    double current_lambda, ni;
    int lev_iterations;
    /* recorder of the last optimize() call */
    sv_s3_iter* iters;
    int n_iters, cap_iters;
    /* active structure */
    int* active_edges;
    int n_active_edges;
    int* ivmap;
    int n_ivmap;
    void* solver;
} sv_s3_graph;

void sv_s3_graph_init(sv_s3_graph* g);
void sv_s3_graph_free(sv_s3_graph* g);
/* returns the vertex index; vertices must be added in increasing id order */
int sv_s3_add_vertex(sv_s3_graph* g, unsigned int id, const sv_sim3* est, int fixed);
int sv_s3_add_graph_edge(sv_s3_graph* g, int v0, int v1, const sv_sim3* measurement);
/* unary reprojection edge on vertex v0 (kind FORWARD or BACKWARD) */
int sv_s3_add_reproj_edge(sv_s3_graph* g, int kind, int v0, const double obs[2], double info, double fx, double fy,
                          double cx, double cy, const double pos_w[3], const double rot[9], const double trans[3],
                          int has_kernel, double huber_delta);

/* SparseOptimizer::initializeOptimization() (level 0) incl. the terminate_action call for iteration -1 */
int sv_s3_initialize_optimization(sv_s3_graph* g);
/* SparseOptimizer::optimize(iterations): number of iterations run (-1 if empty) */
int sv_s3_optimize(sv_s3_graph* g, int iterations);

double sv_s3_edge_chi2(const sv_s3_edge* e);
void sv_s3_compute_active_errors(sv_s3_graph* g);
double sv_s3_active_robust_chi2(const sv_s3_graph* g);

/* ---- transform_optimizer::optimize -------------------------------------------------------------- */
typedef struct sv_transform_match {
    unsigned int idx1;
    double obs1[2];     /* keypoint of keyframe 1 (edge_12 measurement) */
    double info1;       /* keyfrm_1->orb_params_->inv_level_sigma_sq_[octave] (float widened) */
    double pos_w_2[3];  /* lm_2->get_pos_in_world() (edge_12) */
    double obs2[2];
    double info2;
    double pos_w_1[3];  /* lm_1->get_pos_in_world() (edge_21) */
} sv_transform_match;

typedef struct sv_transform_camera {
    double fx, fy, cx, cy;
} sv_transform_camera;

/* rej[i]: 0 = kept, 1 = rejected after the first optimization (edges set to level 1), 2 = rejected after the
 * second one. Returns num_inliers (0 when fewer than 10 matches survive the first stage; then *sim3_12 is
 * left untouched, like upstream). `mid` (may be NULL) receives the vertex estimate after the first
 * optimize(5). */
unsigned int sv_transform_optimize(const sv_transform_camera* cam1, const sv_transform_camera* cam2,
                                   const double rot_1w[9], const double trans_1w[3],
                                   const double rot_2w[9], const double trans_2w[3],
                                   const sv_transform_match* m, unsigned int n, unsigned char* rej,
                                   sv_sim3* sim3_12, float chi_sq, int fix_scale, unsigned int num_iter,
                                   sv_sim3* mid);

#endif
