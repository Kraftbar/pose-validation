/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_g2o_ba.h (BSD, g2o/stella_vslam-derived; Eigen-order kernels
 * follow MPL-2.0 Eigen's evaluation order as measured). */
#include "sv_g2o_ba.h"
#include "sv_eigen_amd.h"
#include "sv_eigen_llt.h"
#include "sv_g2o_edge.h"
#include "sv_linalg.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TAU 1e-5
#define GOOD_STEP_LOWER (1.0 / 3.0)
#define GOOD_STEP_UPPER (2.0 / 3.0)
#define MAX_TRIALS_AFTER_FAILURE 10

/* ==========================================================================
 * Eigen-order kernels (each verified bit-exact against real Eigen 3.4 by
 * stella_port/reference_tools/eigen_shape_tests_ba.cc, see HANDOVER.md).
 * ========================================================================== */

/* DInv->multiply(xl, cl) -> internal::axpy<Matrix3d>:
 *   y.segment<3>(off) += A * x.segment<3>(off)
 * dst is a fixed-size-3 block with PacketAccess: complete-unrolled linear
 * vectorized assignment = packet [0,1] + scalar [2]. Rows 0-1 come from the
 * packet path (k-ascending pmadd chain: ((a0*x0)+a1*x1)+a2*x2), row 2 from
 * the scalar coeff() path (redux_novec_unroller: a0 + (a1+a2)). Same rule
 * as the Matrix3d*Vector3d row of HANDOVER.md's table. */
void sv_ba_k_axpy3(const double A_[9], const double x[3], double y[3]) {
#define AA(i, j) A_[(j) * 3 + (i)]
    double s0 = AA(0, 0) * x[0];
    s0 = s0 + AA(0, 1) * x[1];
    s0 = s0 + AA(0, 2) * x[2];
    double s1 = AA(1, 0) * x[0];
    s1 = s1 + AA(1, 1) * x[1];
    s1 = s1 + AA(1, 2) * x[2];
    const double s2 = AA(2, 0) * x[0] + (AA(2, 1) * x[1] + AA(2, 2) * x[2]);
    y[0] += s0;
    y[1] += s1;
    y[2] += s2;
#undef AA
}

/* SparseBlockMatrixCCS::rightMultiply -> internal::atxpy:
 *   y.segment<3>(off) += A.transpose() * x.segment<6>(off)
 * Each output coefficient is the scalar coeff() of a lazy product whose
 * lhs.row(i).transpose().cwiseProduct(rhs) redux is complete-unrolled
 * linear-vectorized over 6 doubles: redux_vec_unroller splits the three
 * 2-lane packets as pkt0 + (pkt1 + pkt2), then predux adds the two lanes:
 *   (a0 + (a2 + a4)) + (a1 + (a3 + a5)). */
void sv_ba_k_atxpy63(const double A_[6][3], const double x[6], double y[3]) {
    int c;
    for (c = 0; c < 3; ++c) {
        const double a0 = A_[0][c] * x[0], a1 = A_[1][c] * x[1], a2 = A_[2][c] * x[2];
        const double a3 = A_[3][c] * x[3], a4 = A_[4][c] * x[4], a5 = A_[5][c] * x[5];
        const double lane0 = a0 + (a2 + a4);
        const double lane1 = a1 + (a3 + a5);
        y[c] += lane0 + lane1;
    }
}

/* PoseLandmarkMatrixType BDinv = (*Bi) * Dinv : 6x3 * 3x3, 6 rows =
 * three full packets, every entry the k-ascending chain ((a0*b0)+a1*b1)+a2*b2. */
void sv_ba_k_bdinv(const double B[6][3], const double Dinv[9], double out[6][3]) {
    int r, c;
    for (r = 0; r < 6; ++r) {
        for (c = 0; c < 3; ++c) {
            double s = B[r][0] * Dinv[c * 3 + 0];
            s = s + B[r][1] * Dinv[c * 3 + 1];
            s = s + B[r][2] * Dinv[c * 3 + 2];
            out[r][c] = s;
        }
    }
}

/* Bb.noalias() += (*Bi) * db : Map<Matrix<6,1>> += 6x3 * 3-vector,
 * inner-vectorized (3 packets): dst + (((a0*d0)+a1*d1)+a2*d2). */
void sv_ba_k_bb(const double B[6][3], const double d[3], double y[6]) {
    int r;
    for (r = 0; r < 6; ++r) {
        double s = B[r][0] * d[0];
        s = s + B[r][1] * d[1];
        s = s + B[r][2] * d[2];
        y[r] += s;
    }
}

/* (*Hi1i2).noalias() -= BDinv * Bj->transpose() : 6x6 -= 6x3 * 3x6,
 * packets along the 6 rows: dst - (((a0*b0)+a1*b1)+a2*b2). */
void sv_ba_k_schur_sub(double H[6][6], const double BD[6][3], const double Bj[6][3]) {
    int r, c;
    for (r = 0; r < 6; ++r) {
        for (c = 0; c < 6; ++c) {
            double s = BD[r][0] * Bj[c][0];
            s = s + BD[r][1] * Bj[c][1];
            s = s + BD[r][2] * Bj[c][2];
            H[r][c] = H[r][c] - s;
        }
    }
}

/* ==========================================================================
 * Graph containers
 * ========================================================================== */

void sv_ba_graph_init(sv_ba_graph* g) {
    memset(g, 0, sizeof(*g));
    g->gain_threshold = 1e-6;
}

typedef struct sv_ba_solver sv_ba_solver;
static void solver_free(sv_ba_solver* s);

void sv_ba_graph_free(sv_ba_graph* g) {
    free(g->v);
    free(g->e);
    free(g->iters);
    free(g->active_edges);
    free(g->ivmap);
    if (g->solver) {
        solver_free((sv_ba_solver*)g->solver);
        free(g->solver);
    }
    memset(g, 0, sizeof(*g));
}

int sv_ba_add_shot_vertex(sv_ba_graph* g, unsigned int id, const sv_se3* pose, int fixed) {
    if (g->nv == g->cap_v) {
        g->cap_v = g->cap_v ? g->cap_v * 2 : 64;
        g->v = (sv_ba_vertex*)realloc(g->v, sizeof(sv_ba_vertex) * (size_t)g->cap_v);
    }
    sv_ba_vertex* v = &g->v[g->nv];
    memset(v, 0, sizeof(*v));
    v->id = id;
    v->is_landmark = 0;
    v->fixed = fixed;
    v->pose = *pose;
    v->hidx = -1;
    return g->nv++;
}

int sv_ba_add_landmark_vertex(sv_ba_graph* g, unsigned int id, const double pos[3], int fixed) {
    if (g->nv == g->cap_v) {
        g->cap_v = g->cap_v ? g->cap_v * 2 : 64;
        g->v = (sv_ba_vertex*)realloc(g->v, sizeof(sv_ba_vertex) * (size_t)g->cap_v);
    }
    sv_ba_vertex* v = &g->v[g->nv];
    memset(v, 0, sizeof(*v));
    v->id = id;
    v->is_landmark = 1;
    v->fixed = fixed;
    v->pos[0] = pos[0];
    v->pos[1] = pos[1];
    v->pos[2] = pos[2];
    v->hidx = -1;
    return g->nv++;
}

int sv_ba_add_edge(sv_ba_graph* g, int lm_vertex, int kf_vertex, const double obs[2], double info,
                   int has_kernel, double huber_delta) {
    if (g->ne == g->cap_e) {
        g->cap_e = g->cap_e ? g->cap_e * 2 : 256;
        g->e = (sv_ba_edge*)realloc(g->e, sizeof(sv_ba_edge) * (size_t)g->cap_e);
    }
    sv_ba_edge* e = &g->e[g->ne];
    memset(e, 0, sizeof(*e));
    e->lm = lm_vertex;
    e->kf = kf_vertex;
    e->obs[0] = obs[0];
    e->obs[1] = obs[1];
    e->info = info;
    e->has_kernel = has_kernel;
    e->huber_delta = huber_delta;
    e->level = 0;
    return g->ne++;
}

void sv_ba_set_force_stop_flag(sv_ba_graph* g, int* flag) { g->stop_flag = flag; }
int sv_ba_terminate(const sv_ba_graph* g) { return g->stop_flag ? (*g->stop_flag != 0) : 0; }

/* ==========================================================================
 * Edge: error / chi2 / robust chi2 / linearization / quadratic form
 * (mono_perspective_reproj_edge + BaseFixedSizedEdge<2,Vec2,landmark,shot>)
 * ========================================================================== */

static void edge_pos_c(const sv_ba_graph* g, const sv_ba_edge* e, double pos_c[3]) {
    sv_se3_map(&g->v[e->kf].pose, g->v[e->lm].pos, pos_c);
}

/* computeError(): _error = obs - cam_project(pose.map(pos_w)) */
static void edge_error(const sv_ba_graph* g, const sv_ba_edge* e, double err[2]) {
    double pc[3];
    edge_pos_c(g, e, pc);
    const double px = g->fx * pc[0] / pc[2] + g->cx;
    const double py = g->fy * pc[1] / pc[2] + g->cy;
    err[0] = e->obs[0] - px;
    err[1] = e->obs[1] - py;
}

double sv_ba_edge_chi2(const sv_ba_edge* e) {
    /* _error.dot(information() * _error), see sv_pose_opt_edge_chi2 */
    const double ie0 = e->info * e->err[0];
    const double ie1 = e->info * e->err[1];
    return ie0 * e->err[0] + ie1 * e->err[1];
}

int sv_ba_edge_depth_positive(const sv_ba_graph* g, const sv_ba_edge* e) {
    double pc[3];
    edge_pos_c(g, e, pc);
    return 0.0 < pc[2];
}

static int edge_all_fixed(const sv_ba_graph* g, const sv_ba_edge* e) {
    return g->v[e->lm].fixed && g->v[e->kf].fixed;
}

void sv_ba_compute_active_errors(sv_ba_graph* g) {
    int k;
    for (k = 0; k < g->n_active_edges; ++k) {
        sv_ba_edge* e = &g->e[g->active_edges[k]];
        edge_error(g, e, e->err);
    }
}

double sv_ba_active_robust_chi2(const sv_ba_graph* g) {
    double chi = 0.0;
    int k;
    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_ba_edge* e = &g->e[g->active_edges[k]];
        if (e->has_kernel) {
            double rho[3];
            sv_huber_robustify(e->huber_delta, sv_ba_edge_chi2(e), rho);
            chi += rho[0];
        } else {
            chi += sv_ba_edge_chi2(e);
        }
    }
    return chi;
}

/* linearizeOplus(): Xi (2x3, landmark) and Xj (2x6, shot). */
static void edge_linearize(const sv_ba_graph* g, const sv_ba_edge* e, double Xi[2][3], double Xj[2][6]) {
    const sv_se3* pose = &g->v[e->kf].pose;
    const double fx = g->fx, fy = g->fy;
    double pc[3];
    edge_pos_c(g, e, pc);
    const double x = pc[0], y = pc[1], z = pc[2];
    const double z_sq = z * z;

    double R[9]; /* rot_cw column-major R[c*3+r] */
    sv_quat_to_mat3(&pose->q, R);
#define ROT(r, c) R[(c) * 3 + (r)]
    Xi[0][0] = -fx * ROT(0, 0) / z + fx * x * ROT(2, 0) / z_sq;
    Xi[0][1] = -fx * ROT(0, 1) / z + fx * x * ROT(2, 1) / z_sq;
    Xi[0][2] = -fx * ROT(0, 2) / z + fx * x * ROT(2, 2) / z_sq;
    Xi[1][0] = -fy * ROT(1, 0) / z + fy * y * ROT(2, 0) / z_sq;
    Xi[1][1] = -fy * ROT(1, 1) / z + fy * y * ROT(2, 1) / z_sq;
    Xi[1][2] = -fy * ROT(1, 2) / z + fy * y * ROT(2, 2) / z_sq;
#undef ROT

    Xj[0][0] = x * y / z_sq * fx;
    Xj[0][1] = -(1.0 + (x * x / z_sq)) * fx;
    Xj[0][2] = y / z * fx;
    Xj[0][3] = -1.0 / z * fx;
    Xj[0][4] = 0.0;
    Xj[0][5] = x / z_sq * fx;

    Xj[1][0] = (1.0 + y * y / z_sq) * fy;
    Xj[1][1] = -x * y / z_sq * fy;
    Xj[1][2] = -x / z * fy;
    Xj[1][3] = 0.0;
    Xj[1][4] = -1.0 / z * fy;
    Xj[1][5] = y / z_sq * fy;
}

/* ==========================================================================
 * BlockSolver_6_3 workspace
 * ========================================================================== */

struct sv_ba_solver {
    int P, L;                 /* non-marginalized / marginalized vertex counts */
    int size_poses, size_landmarks;
    int do_schur;
    /* Hpl in CCS-by-landmark form (poses ascending per landmark) */
    int* hpl_col;             /* L+1 */
    int* hpl_pose;            /* n_hpl */
    double (*hpl_blk)[6][3];  /* n_hpl */
    int n_hpl;
    int* edge_slot;           /* per active-edge position: hpl slot or -1 */
    /* Hschur upper block pattern in CCS (rows ascending per column) */
    int* hs_col;              /* P+1 */
    int* hs_row;              /* n_hs */
    double (*hs_blk)[6][6];   /* n_hs */
    int n_hs;
    /* scalar CCS of the upper triangle handed to SimplicialLLT */
    int* sc_p;
    int* sc_i;
    double* sc_x;
    int sllt_ready;
    sv_sllt sllt;
    /* vectors */
    double* x;
    double* b;
    double* coef;
    double* bschur;
    size_t x_cap;
    double* dinv;             /* L * 9 (column-major) */
    double (*bak_pose)[6];    /* diagonal backups */
    double (*bak_lm)[3];
    int x_initialized;
};

static void solver_free_structure(sv_ba_solver* s) {
    free(s->hpl_col);
    free(s->hpl_pose);
    free(s->hpl_blk);
    free(s->edge_slot);
    free(s->hs_col);
    free(s->hs_row);
    free(s->hs_blk);
    free(s->sc_p);
    free(s->sc_i);
    free(s->sc_x);
    free(s->dinv);
    free(s->bak_pose);
    free(s->bak_lm);
    s->hpl_col = s->hpl_pose = s->edge_slot = s->hs_col = s->hs_row = s->sc_p = s->sc_i = NULL;
    s->hpl_blk = NULL;
    s->hs_blk = NULL;
    s->sc_x = s->dinv = NULL;
    s->bak_pose = NULL;
    s->bak_lm = NULL;
    if (s->sllt_ready) {
        sv_sllt_free(&s->sllt);
        s->sllt_ready = 0;
    }
}

static void solver_free(sv_ba_solver* s) {
    solver_free_structure(s);
    free(s->x);
    free(s->b);
    free(s->coef);
    free(s->bschur);
    s->x = s->b = s->coef = s->bschur = NULL;
}

static int cmp_pair(const void* a, const void* b) {
    const int* x = (const int*)a;
    const int* y = (const int*)b;
    if (x[0] != y[0]) {
        return x[0] < y[0] ? -1 : 1;
    }
    return (x[1] > y[1]) - (x[1] < y[1]);
}

static sv_ba_solver* get_solver(sv_ba_graph* g) {
    if (!g->solver) {
        g->solver = calloc(1, sizeof(sv_ba_solver));
    }
    return (sv_ba_solver*)g->solver;
}

/* SparseOptimizer::initializeOptimization(level=0) */
int sv_ba_initialize_optimization(sv_ba_graph* g) {
    int i, k;
    if (g->ne == 0) {
        return 0; /* "Attempt to initialize an empty graph" */
    }
    /* clearIndexMapping */
    for (i = 0; i < g->n_ivmap; ++i) {
        g->v[g->ivmap[i]].hidx = -1;
    }
    free(g->ivmap);
    g->ivmap = NULL;
    g->n_ivmap = 0;
    for (i = 0; i < g->nv; ++i) {
        g->v[i].active = 0;
    }
    free(g->active_edges);
    g->active_edges = (int*)malloc(sizeof(int) * (size_t)(g->ne > 0 ? g->ne : 1));
    g->n_active_edges = 0;
    for (k = 0; k < g->ne; ++k) {
        const sv_ba_edge* e = &g->e[k];
        if (e->level == 0 && !edge_all_fixed(g, e)) {
            g->active_edges[g->n_active_edges++] = k;
            g->v[e->lm].active = 1;
            g->v[e->kf].active = 1;
        }
    }
    /* buildIndexMapping over _activeVertices (vertex-id order == array order) */
    g->ivmap = (int*)malloc(sizeof(int) * (size_t)(g->nv > 0 ? g->nv : 1));
    {
        int n = 0, kk;
        for (kk = 0; kk < 2; ++kk) {
            for (i = 0; i < g->nv; ++i) {
                sv_ba_vertex* v = &g->v[i];
                if (!v->active) {
                    continue;
                }
                if (!v->fixed) {
                    if (v->is_landmark == kk) {
                        v->hidx = n;
                        g->ivmap[n] = i;
                        ++n;
                    }
                } else {
                    v->hidx = -1;
                }
            }
        }
        g->n_ivmap = n;
    }
    /* postIteration(-1): terminate_action, iteration < 0 */
    sv_ba_compute_active_errors(g);
    if (g->stop_flag) {
        *g->stop_flag = 0;
    } else {
        g->aux_flag = 0;
        g->stop_flag = &g->aux_flag;
    }
    g->stopped_by_terminate = 0;
    return g->n_ivmap > 0;
}

/* BlockSolver::buildStructure */
static int build_structure(sv_ba_graph* g) {
    sv_ba_solver* s = get_solver(g);
    int i, k;
    solver_free_structure(s);

    s->P = s->L = 0;
    s->size_poses = s->size_landmarks = 0;
    for (i = 0; i < g->n_ivmap; ++i) {
        if (!g->v[g->ivmap[i]].is_landmark) {
            ++s->P;
            s->size_poses += 6;
        } else {
            ++s->L;
            s->size_landmarks += 3;
        }
    }
    s->do_schur = 1;
    if (s->P == 0) {
        return 0; /* no free pose: not a shape stella's BA produces */
    }
    const size_t vec_size = (size_t)(s->size_poses + s->size_landmarks);
    if (s->x_cap < vec_size) {
        s->x_cap = 2 * vec_size;
        free(s->x);
        free(s->b);
        free(s->coef);
        free(s->bschur);
        s->x = (double*)calloc(s->x_cap, sizeof(double));
        s->b = (double*)calloc(s->x_cap, sizeof(double));
        s->coef = (double*)calloc(s->x_cap, sizeof(double));
        s->bschur = (double*)calloc(s->x_cap, sizeof(double));
    }

    /* Hpl pattern from the active edges */
    int* pairs = (int*)malloc(sizeof(int) * 2 * (size_t)(g->n_active_edges > 0 ? g->n_active_edges : 1));
    int np = 0;
    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_ba_edge* e = &g->e[g->active_edges[k]];
        const int pi = g->v[e->kf].hidx;
        const int li = g->v[e->lm].hidx;
        if (pi >= 0 && li >= 0) {
            pairs[2 * np + 0] = li - s->P;
            pairs[2 * np + 1] = pi;
            ++np;
        }
    }
    qsort(pairs, (size_t)np, 2 * sizeof(int), cmp_pair);
    s->hpl_col = (int*)calloc((size_t)s->L + 1, sizeof(int));
    s->hpl_pose = (int*)malloc(sizeof(int) * (size_t)(np > 0 ? np : 1));
    s->n_hpl = 0;
    for (i = 0; i < np; ++i) {
        if (i > 0 && pairs[2 * i] == pairs[2 * (i - 1)] && pairs[2 * i + 1] == pairs[2 * (i - 1) + 1]) {
            continue;
        }
        s->hpl_pose[s->n_hpl] = pairs[2 * i + 1];
        s->hpl_col[pairs[2 * i] + 1]++;
        ++s->n_hpl;
    }
    for (i = 0; i < s->L; ++i) {
        s->hpl_col[i + 1] += s->hpl_col[i];
    }
    s->hpl_blk = (double(*)[6][3])calloc((size_t)(s->n_hpl > 0 ? s->n_hpl : 1), sizeof(double[6][3]));
    s->edge_slot = (int*)malloc(sizeof(int) * (size_t)(g->n_active_edges > 0 ? g->n_active_edges : 1));
    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_ba_edge* e = &g->e[g->active_edges[k]];
        const int pi = g->v[e->kf].hidx;
        const int li = g->v[e->lm].hidx;
        s->edge_slot[k] = -1;
        if (pi >= 0 && li >= 0) {
            int lo = s->hpl_col[li - s->P], hi = s->hpl_col[li - s->P + 1] - 1;
            while (lo <= hi) {
                const int mid = (lo + hi) / 2;
                if (s->hpl_pose[mid] == pi) {
                    s->edge_slot[k] = mid;
                    break;
                }
                if (s->hpl_pose[mid] < pi) {
                    lo = mid + 1;
                } else {
                    hi = mid - 1;
                }
            }
        }
    }
    free(pairs);

    /* Hschur pattern: for every marginalized vertex v in the index mapping,
     * over ALL of v's edges (any level -- v->edges() is the full edge set),
     * every pair (i1 <= i2) of poses with a hessian index. Diagonal blocks
     * come along because each such pose pairs with itself. */
    {
        int* vlm_cnt = (int*)calloc((size_t)g->nv + 1, sizeof(int));
        for (k = 0; k < g->ne; ++k) {
            vlm_cnt[g->e[k].lm + 1]++;
        }
        for (i = 0; i < g->nv; ++i) {
            vlm_cnt[i + 1] += vlm_cnt[i];
        }
        int* vlm_edges = (int*)malloc(sizeof(int) * (size_t)(g->ne > 0 ? g->ne : 1));
        int* fill = (int*)malloc(sizeof(int) * ((size_t)g->nv + 1));
        memcpy(fill, vlm_cnt, sizeof(int) * ((size_t)g->nv + 1));
        for (k = 0; k < g->ne; ++k) {
            vlm_edges[fill[g->e[k].lm]++] = k;
        }
        free(fill);

        size_t cap = 64, n_pairs = 0;
        int* pp = (int*)malloc(sizeof(int) * 2 * cap);
        int* poses = (int*)malloc(sizeof(int) * (size_t)(g->ne > 0 ? g->ne : 1));
        for (i = 0; i < g->n_ivmap; ++i) {
            const int vi = g->ivmap[i];
            if (!g->v[vi].is_landmark) {
                continue;
            }
            int npose = 0, a, b;
            for (a = vlm_cnt[vi]; a < vlm_cnt[vi + 1]; ++a) {
                const int h = g->v[g->e[vlm_edges[a]].kf].hidx;
                if (h != -1) {
                    poses[npose++] = h;
                }
            }
            /* every (edge1, edge2) combination, i1 <= i2 */
            for (a = 0; a < npose; ++a) {
                for (b = 0; b < npose; ++b) {
                    if (poses[a] <= poses[b]) {
                        if (n_pairs == cap) {
                            cap *= 2;
                            pp = (int*)realloc(pp, sizeof(int) * 2 * cap);
                        }
                        pp[2 * n_pairs + 0] = poses[b]; /* column */
                        pp[2 * n_pairs + 1] = poses[a]; /* row */
                        ++n_pairs;
                    }
                }
            }
        }
        qsort(pp, n_pairs, 2 * sizeof(int), cmp_pair);
        s->hs_col = (int*)calloc((size_t)s->P + 1, sizeof(int));
        s->hs_row = (int*)malloc(sizeof(int) * (n_pairs > 0 ? n_pairs : 1));
        s->n_hs = 0;
        for (i = 0; i < (int)n_pairs; ++i) {
            if (i > 0 && pp[2 * i] == pp[2 * (i - 1)] && pp[2 * i + 1] == pp[2 * (i - 1) + 1]) {
                continue;
            }
            s->hs_row[s->n_hs] = pp[2 * i + 1];
            s->hs_col[pp[2 * i] + 1]++;
            ++s->n_hs;
        }
        for (i = 0; i < s->P; ++i) {
            s->hs_col[i + 1] += s->hs_col[i];
        }
        free(pp);
        free(poses);
        free(vlm_cnt);
        free(vlm_edges);
    }
    s->hs_blk = (double(*)[6][6])calloc((size_t)(s->n_hs > 0 ? s->n_hs : 1), sizeof(double[6][6]));

    /* scalar CCS (upper triangle) -- SparseBlockMatrixCCS::fillCCS(Cp,Ci,Cx,true) */
    {
        const int n = s->size_poses;
        int nz = 0, bc, c, bi;
        s->sc_p = (int*)malloc(sizeof(int) * ((size_t)n + 1));
        /* count */
        for (bc = 0; bc < s->P; ++bc) {
            for (c = 0; c < 6; ++c) {
                for (bi = s->hs_col[bc]; bi < s->hs_col[bc + 1]; ++bi) {
                    nz += (s->hs_row[bi] == bc) ? c + 1 : 6;
                }
            }
        }
        s->sc_i = (int*)malloc(sizeof(int) * (size_t)(nz > 0 ? nz : 1));
        s->sc_x = (double*)malloc(sizeof(double) * (size_t)(nz > 0 ? nz : 1));
        nz = 0;
        for (bc = 0; bc < s->P; ++bc) {
            for (c = 0; c < 6; ++c) {
                s->sc_p[bc * 6 + c] = nz;
                for (bi = s->hs_col[bc]; bi < s->hs_col[bc + 1]; ++bi) {
                    const int rstart = s->hs_row[bi] * 6;
                    const int elems = (s->hs_row[bi] == bc) ? c + 1 : 6;
                    int r;
                    for (r = 0; r < elems; ++r) {
                        s->sc_i[nz++] = rstart + r;
                    }
                }
            }
        }
        s->sc_p[n] = nz;
    }

    s->dinv = (double*)malloc(sizeof(double) * 9 * (size_t)(s->L > 0 ? s->L : 1));
    s->bak_pose = (double(*)[6])malloc(sizeof(double[6]) * (size_t)s->P);
    s->bak_lm = (double(*)[3])malloc(sizeof(double[3]) * (size_t)(s->L > 0 ? s->L : 1));
    return 1;
}

static double* hs_block(sv_ba_solver* s, int row, int col) {
    int lo = s->hs_col[col], hi = s->hs_col[col + 1] - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (s->hs_row[mid] == row) {
            return &s->hs_blk[mid][0][0];
        }
        if (s->hs_row[mid] < row) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return NULL;
}

/* BlockSolver::buildSystem */
static void build_system(sv_ba_graph* g) {
    sv_ba_solver* s = get_solver(g);
    int i, k, r, c;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_ba_vertex* v = &g->v[g->ivmap[i]];
        memset(v->b, 0, sizeof(v->b));
        memset(v->hessian, 0, sizeof(v->hessian));
    }
    memset(s->hpl_blk, 0, sizeof(double[6][3]) * (size_t)s->n_hpl);

    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_ba_edge* e = &g->e[g->active_edges[k]];
        sv_ba_vertex* vl = &g->v[e->lm];
        sv_ba_vertex* vs = &g->v[e->kf];
        double Xi[2][3], Xj[2][6];
        edge_linearize(g, e, Xi, Xj);

        /* constructQuadraticForm */
        double omega_diag, we0, we1;
        if (e->has_kernel) {
            const double chi2 = sv_ba_edge_chi2(e);
            double rho[3];
            sv_huber_robustify(e->huber_delta, chi2, rho);
            /* omega_r = -_information * _error; omega_r *= rho[1]; omega = rho[1] * _information */
            we0 = (-(e->info * e->err[0])) * rho[1];
            we1 = (-(e->info * e->err[1])) * rho[1];
            omega_diag = rho[1] * e->info;
        } else {
            we0 = -(e->info * e->err[0]);
            we1 = -(e->info * e->err[1]);
            omega_diag = e->info;
        }

        /* N = 0 : landmark (from), 3 dims */
        if (!vl->fixed) {
            double AtO0[3], AtO1[3];
            for (r = 0; r < 3; ++r) {
                AtO0[r] = Xi[0][r] * omega_diag;
                AtO1[r] = Xi[1][r] * omega_diag;
            }
            for (r = 0; r < 3; ++r) {
                vl->b[r] += Xi[0][r] * we0 + Xi[1][r] * we1;
                for (c = 0; c < 3; ++c) {
                    vl->hessian[r][c] += AtO0[r] * Xi[0][c] + AtO1[r] * Xi[1][c];
                }
            }
            /* off-diagonal towards the shot vertex (if not fixed):
             * hessianTransposed(6x3) += B^T (6x2) * AtO^T (2x3) */
            if (!vs->fixed) {
                double (*blk)[3] = s->hpl_blk[s->edge_slot[k]];
                for (r = 0; r < 6; ++r) {
                    for (c = 0; c < 3; ++c) {
                        blk[r][c] += Xj[0][r] * AtO0[c] + Xj[1][r] * AtO1[c];
                    }
                }
            }
        }
        /* N = 1 : shot (from), 6 dims */
        if (!vs->fixed) {
            double AtO0[6], AtO1[6];
            for (r = 0; r < 6; ++r) {
                AtO0[r] = Xj[0][r] * omega_diag;
                AtO1[r] = Xj[1][r] * omega_diag;
            }
            for (r = 0; r < 6; ++r) {
                vs->b[r] += Xj[0][r] * we0 + Xj[1][r] * we1;
                for (c = 0; c < 6; ++c) {
                    vs->hessian[r][c] += AtO0[r] * Xj[0][c] + AtO1[r] * Xj[1][c];
                }
            }
        }
    }

    /* copyB */
    {
        int off_p = 0, off_l = s->size_poses;
        for (i = 0; i < g->n_ivmap; ++i) {
            sv_ba_vertex* v = &g->v[g->ivmap[i]];
            if (!v->is_landmark) {
                memcpy(s->b + off_p, v->b, sizeof(double) * 6);
                off_p += 6;
            } else {
                memcpy(s->b + off_l, v->b, sizeof(double) * 3);
                off_l += 3;
            }
        }
    }
}

/* BlockSolver::setLambda(lambda, backup=true) */
static void set_lambda(sv_ba_graph* g, double lambda) {
    sv_ba_solver* s = get_solver(g);
    int i, d, pi = 0, li = 0;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_ba_vertex* v = &g->v[g->ivmap[i]];
        if (!v->is_landmark) {
            for (d = 0; d < 6; ++d) {
                s->bak_pose[pi][d] = v->hessian[d][d];
                v->hessian[d][d] += lambda;
            }
            ++pi;
        } else {
            for (d = 0; d < 3; ++d) {
                s->bak_lm[li][d] = v->hessian[d][d];
                v->hessian[d][d] += lambda;
            }
            ++li;
        }
    }
}

static void restore_diagonal(sv_ba_graph* g) {
    sv_ba_solver* s = get_solver(g);
    int i, d, pi = 0, li = 0;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_ba_vertex* v = &g->v[g->ivmap[i]];
        if (!v->is_landmark) {
            for (d = 0; d < 6; ++d) {
                v->hessian[d][d] = s->bak_pose[pi][d];
            }
            ++pi;
        } else {
            for (d = 0; d < 3; ++d) {
                v->hessian[d][d] = s->bak_lm[li][d];
            }
            ++li;
        }
    }
}

/* BlockSolver::solve() with Schur complement. Returns 1 on success. */
static int solver_solve(sv_ba_graph* g) {
    sv_ba_solver* s = get_solver(g);
    int i, j, li, bi, bj, r, c;
    double* xp = s->x;
    double* xl = s->x + s->size_poses;
    double* cp = s->coef;
    double* cl = s->coef + s->size_poses;
    double* bl = s->b + s->size_poses;

    /* _Hschur->clear(); _Hpp->add(*_Hschur); */
    memset(s->hs_blk, 0, sizeof(double[6][6]) * (size_t)s->n_hs);
    {
        int pi = 0;
        for (i = 0; i < g->n_ivmap; ++i) {
            const sv_ba_vertex* v = &g->v[g->ivmap[i]];
            if (v->is_landmark) {
                continue;
            }
            double (*blk)[6] = (double(*)[6])hs_block(s, pi, pi);
            for (r = 0; r < 6; ++r) {
                for (c = 0; c < 6; ++c) {
                    blk[r][c] += v->hessian[r][c];
                }
            }
            ++pi;
        }
    }
    memset(s->coef, 0, sizeof(double) * (size_t)s->size_poses);

    /* landmark loop (block order = landmark hessian index) */
    {
        int lidx = 0;
        for (i = 0; i < g->n_ivmap; ++i) {
            const sv_ba_vertex* v = &g->v[g->ivmap[i]];
            if (!v->is_landmark) {
                continue;
            }
            li = lidx++;
            double D[9]; /* column-major */
            for (r = 0; r < 3; ++r) {
                for (c = 0; c < 3; ++c) {
                    D[c * 3 + r] = v->hessian[r][c];
                }
            }
            double* Dinv = s->dinv + 9 * li;
            sv_mat3_inverse(D, Dinv);
            double db[3], db2[3];
            db[0] = bl[3 * li + 0];
            db[1] = bl[3 * li + 1];
            db[2] = bl[3 * li + 2];
            sv_mat3_mulv(Dinv, db, db2);
            db[0] = db2[0];
            db[1] = db2[1];
            db[2] = db2[2];

            for (bi = s->hpl_col[li]; bi < s->hpl_col[li + 1]; ++bi) {
                const int i1 = s->hpl_pose[bi];
                double BDinv[6][3];
                sv_ba_k_bdinv(s->hpl_blk[bi], Dinv, BDinv);
                sv_ba_k_bb(s->hpl_blk[bi], db, s->coef + 6 * i1);
                for (bj = bi; bj < s->hpl_col[li + 1]; ++bj) {
                    const int i2 = s->hpl_pose[bj];
                    double (*Hi1i2)[6] = (double(*)[6])hs_block(s, i1, i2);
                    sv_ba_k_schur_sub(Hi1i2, BDinv, s->hpl_blk[bj]);
                }
            }
        }
    }

    /* _bschur = _b - _coefficients */
    memcpy(s->bschur, s->b, sizeof(double) * (size_t)s->size_poses);
    for (i = 0; i < s->size_poses; ++i) {
        s->bschur[i] -= s->coef[i];
    }

    /* _linearSolver->solve(*_Hschur, _x, _bschur): LinearSolverEigen */
    {
        const int n = s->size_poses;
        int bc, cc, k;
        /* fillCCS values (upper triangle) */
        int nz = 0;
        for (bc = 0; bc < s->P; ++bc) {
            for (cc = 0; cc < 6; ++cc) {
                for (k = s->hs_col[bc]; k < s->hs_col[bc + 1]; ++k) {
                    const int elems = (s->hs_row[k] == bc) ? cc + 1 : 6;
                    for (r = 0; r < elems; ++r) {
                        s->sc_x[nz++] = s->hs_blk[k][r][cc];
                    }
                }
            }
        }
        if (!s->sllt_ready) {
            /* computeSymbolicDecomposition: AMD on the block pattern, then
             * blockToScalarPermutation, then analyzePatternWithPermutation */
            int* blockP = (int*)malloc(sizeof(int) * (size_t)s->P);
            sv_amd_order(s->P, s->hs_col, s->hs_row, blockP);
            int* scalarP = (int*)malloc(sizeof(int) * (size_t)n);
            int sidx = 0;
            for (bc = 0; bc < s->P; ++bc) {
                int base = blockP[bc] * 6;
                for (cc = 0; cc < 6; ++cc) {
                    scalarP[sidx++] = base++;
                }
            }
            sv_sllt_analyze(&s->sllt, n, s->sc_p, s->sc_i, scalarP);
            s->sllt_ready = 1;
            free(blockP);
            free(scalarP);
        }
        if (!sv_sllt_factorize(&s->sllt, s->sc_x, s->sc_p, s->sc_i)) {
            return 0;
        }
        sv_sllt_solve(&s->sllt, s->bschur, xp);
    }

    /* landmarks: cp = -xp ; cl = bl ; cl += Hpl^T cp ; xl = Dinv cl */
    for (i = 0; i < s->size_poses; ++i) {
        cp[i] = -xp[i];
    }
    memcpy(cl, bl, sizeof(double) * (size_t)s->size_landmarks);
    for (li = 0; li < s->L; ++li) {
        for (bi = s->hpl_col[li]; bi < s->hpl_col[li + 1]; ++bi) {
            sv_ba_k_atxpy63(s->hpl_blk[bi], cp + 6 * s->hpl_pose[bi], cl + 3 * li);
        }
    }
    memset(xl, 0, sizeof(double) * (size_t)s->size_landmarks);
    for (j = 0; j < s->L; ++j) {
        sv_ba_k_axpy3(s->dinv + 9 * j, cl + 3 * j, xl + 3 * j);
    }
    return 1;
}

/* ==========================================================================
 * Levenberg + SparseOptimizer::optimize
 * ========================================================================== */

static void push_all(sv_ba_graph* g) {
    int i;
    for (i = 0; i < g->nv; ++i) {
        g->v[i].pose_bak = g->v[i].pose;
        memcpy(g->v[i].pos_bak, g->v[i].pos, sizeof(double) * 3);
    }
}

static void pop_all(sv_ba_graph* g) {
    int i;
    for (i = 0; i < g->nv; ++i) {
        g->v[i].pose = g->v[i].pose_bak;
        memcpy(g->v[i].pos, g->v[i].pos_bak, sizeof(double) * 3);
    }
}

static void update_estimates(sv_ba_graph* g, const double* upd) {
    int i;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_ba_vertex* v = &g->v[g->ivmap[i]];
        if (!v->is_landmark) {
            sv_se3 out;
            sv_shot_vertex_oplus(&v->pose, upd, &out);
            v->pose = out;
            upd += 6;
        } else {
            double out[3];
            sv_landmark_vertex_oplus(v->pos, upd, out);
            v->pos[0] = out[0];
            v->pos[1] = out[1];
            v->pos[2] = out[2];
            upd += 3;
        }
    }
}

static double compute_lambda_init(sv_ba_graph* g) {
    double max_diag = 0.0;
    int i, j;
    for (i = 0; i < g->n_ivmap; ++i) {
        const sv_ba_vertex* v = &g->v[g->ivmap[i]];
        const int dim = v->is_landmark ? 3 : 6;
        for (j = 0; j < dim; ++j) {
            const double a = fabs(v->hessian[j][j]);
            max_diag = a > max_diag ? a : max_diag; /* std::max(fabs(..), maxDiagonal) */
        }
    }
    return TAU * max_diag;
}

/* OptimizationAlgorithmLevenberg::solve. Returns 1 = OK, 0 = Terminate. */
static int levenberg_solve(sv_ba_graph* g, int iteration) {
    sv_ba_solver* s = get_solver(g);
    if (iteration == 0) {
        if (!build_structure(g)) {
            return 0;
        }
    }
    sv_ba_compute_active_errors(g);
    double current_chi = sv_ba_active_robust_chi2(g);
    build_system(g);

    if (iteration == 0) {
        g->current_lambda = compute_lambda_init(g);
        g->ni = 2.0;
    }

    double rho = 0.0;
    int qmax = 0;
    do {
        push_all(g);
        set_lambda(g, g->current_lambda);
        const int ok2 = solver_solve(g);
        update_estimates(g, s->x);
        restore_diagonal(g);

        sv_ba_compute_active_errors(g);
        double temp_chi = sv_ba_active_robust_chi2(g);
        if (!ok2) {
            temp_chi = DBL_MAX;
        }

        rho = current_chi - temp_chi;
        double scale = 0.0;
        {
            const size_t vs = (size_t)(s->size_poses + s->size_landmarks);
            size_t j;
            for (j = 0; j < vs; ++j) {
                scale += s->x[j] * (g->current_lambda * s->x[j] + s->b[j]);
            }
        }
        scale += 1e-3;
        rho /= scale;

        if (rho > 0 && isfinite(temp_chi)) {
            double alpha = 1.0 - pow(2 * rho - 1, 3);
            alpha = alpha < GOOD_STEP_UPPER ? alpha : GOOD_STEP_UPPER; /* std::min(alpha, upper) */
            const double scale_factor = GOOD_STEP_LOWER > alpha ? GOOD_STEP_LOWER : alpha; /* std::max(lower, alpha) */
            g->current_lambda *= scale_factor;
            g->ni = 2.0;
            current_chi = temp_chi;
            /* discardTop(): keep the new estimate */
        } else {
            g->current_lambda *= g->ni;
            g->ni *= 2.0;
            pop_all(g);
            if (!isfinite(g->current_lambda)) {
                break;
            }
        }
        qmax++;
    } while (rho < 0 && qmax < MAX_TRIALS_AFTER_FAILURE && !sv_ba_terminate(g));

    g->lev_iterations = qmax;
    if (qmax == MAX_TRIALS_AFTER_FAILURE || rho == 0 || !isfinite(g->current_lambda)) {
        return 0;
    }
    return 1;
}

static void record_iter(sv_ba_graph* g) {
    if (g->n_iters == g->cap_iters) {
        g->cap_iters = g->cap_iters ? g->cap_iters * 2 : 32;
        g->iters = (sv_ba_iter*)realloc(g->iters, sizeof(sv_ba_iter) * (size_t)g->cap_iters);
    }
    sv_ba_iter* it = &g->iters[g->n_iters++];
    it->chi2 = sv_ba_active_robust_chi2(g);
    it->lambda = g->current_lambda;
    it->lev_iter = g->lev_iterations;
    it->flag = sv_ba_terminate(g) ? 1u : 0u;
}

/* stella_vslam terminate_action::operator() for iteration >= 0 */
static void terminate_action_post(sv_ba_graph* g, int iteration) {
    sv_ba_compute_active_errors(g);
    if (iteration == 0) {
        g->last_chi = sv_ba_active_robust_chi2(g);
    } else {
        int stop = 0;
        if (iteration < INT_MAX) {
            const double current_chi = sv_ba_active_robust_chi2(g);
            const double gain = (g->last_chi - current_chi) / current_chi;
            g->last_chi = current_chi;
            if (gain >= 0 && gain < g->gain_threshold) {
                stop = 1;
            }
        } else {
            stop = 1;
        }
        if (stop) {
            if (g->stop_flag) {
                *g->stop_flag = 1;
            } else {
                g->aux_flag = 1;
                g->stop_flag = &g->aux_flag;
            }
            g->stopped_by_terminate = 1;
        }
    }
}

int sv_ba_optimize(sv_ba_graph* g, int iterations) {
    int i;
    g->n_iters = 0;
    if (g->n_ivmap == 0) {
        return -1;
    }
    /* algorithm->init(online=false): solver init -- fresh symbolic analysis */
    {
        sv_ba_solver* s = get_solver(g);
        if (s->sllt_ready) {
            sv_sllt_free(&s->sllt);
            s->sllt_ready = 0;
        }
    }
    int ok = 1;
    int cj = 0;
    for (i = 0; i < iterations && !sv_ba_terminate(g) && ok; i++) {
        const int result = levenberg_solve(g, i);
        ok = (result == 1);
        ++cj;
        terminate_action_post(g, i);
        record_iter(g);
    }
    return cj;
}
