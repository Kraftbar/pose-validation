/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_g2o_sim3.h (BSD, g2o/stella_vslam-derived; Eigen-order kernels follow MPL-2.0 Eigen's evaluation
 * order as measured). */
#include "sv_g2o_sim3.h"
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
#define BD 7

/* ---- Eigen reduction orders (see sv_g2o_sim3.h) ---- */
static double sum7_v(const double p[7]) { /* linear-vectorized redux + tail */
    return ((p[0] + (p[2] + p[4])) + (p[1] + (p[3] + p[5]))) + p[6];
}
static double sum7_l(const double p[7]) { /* packet pmadd chain */
    return ((((((p[0] + p[1]) + p[2]) + p[3]) + p[4]) + p[5]) + p[6]);
}
static double sum7_n(const double p[7]) { /* redux_novec_unroller */
    return (p[0] + (p[1] + p[2])) + ((p[3] + p[4]) + (p[5] + p[6]));
}

/* ==========================================================================
 * containers
 * ========================================================================== */
typedef struct sv_s3_solver sv_s3_solver;
static void solver_free(sv_s3_solver* s);

void sv_s3_graph_init(sv_s3_graph* g) {
    memset(g, 0, sizeof(*g));
}

void sv_s3_graph_free(sv_s3_graph* g) {
    free(g->v);
    free(g->e);
    free(g->iters);
    free(g->active_edges);
    free(g->ivmap);
    if (g->solver) {
        solver_free((sv_s3_solver*)g->solver);
        free(g->solver);
    }
    memset(g, 0, sizeof(*g));
}

int sv_s3_add_vertex(sv_s3_graph* g, unsigned int id, const sv_sim3* est, int fixed) {
    sv_s3_vertex* v;
    if (g->nv == g->cap_v) {
        g->cap_v = g->cap_v ? g->cap_v * 2 : 64;
        g->v = (sv_s3_vertex*)realloc(g->v, sizeof(sv_s3_vertex) * (size_t)g->cap_v);
    }
    v = &g->v[g->nv];
    memset(v, 0, sizeof(*v));
    v->id = id;
    v->fixed = fixed;
    v->est = *est;
    v->bak = *est;
    v->hidx = -1;
    return g->nv++;
}

static sv_s3_edge* new_edge(sv_s3_graph* g) {
    sv_s3_edge* e;
    if (g->ne == g->cap_e) {
        g->cap_e = g->cap_e ? g->cap_e * 2 : 256;
        g->e = (sv_s3_edge*)realloc(g->e, sizeof(sv_s3_edge) * (size_t)g->cap_e);
    }
    e = &g->e[g->ne++];
    memset(e, 0, sizeof(*e));
    e->v1 = -1;
    return e;
}

int sv_s3_add_graph_edge(sv_s3_graph* g, int v0, int v1, const sv_sim3* measurement) {
    sv_s3_edge* e = new_edge(g);
    e->kind = SV_S3_EDGE_GRAPH;
    e->v0 = v0;
    e->v1 = v1;
    e->meas = *measurement;
    return g->ne - 1;
}

int sv_s3_add_reproj_edge(sv_s3_graph* g, int kind, int v0, const double obs[2], double info, double fx, double fy,
                          double cx, double cy, const double pos_w[3], const double rot[9], const double trans[3],
                          int has_kernel, double huber_delta) {
    sv_s3_edge* e = new_edge(g);
    e->kind = kind;
    e->v0 = v0;
    e->obs[0] = obs[0];
    e->obs[1] = obs[1];
    e->info = info;
    e->fx = fx;
    e->fy = fy;
    e->cx = cx;
    e->cy = cy;
    memcpy(e->pos_w, pos_w, sizeof(double) * 3);
    memcpy(e->rot, rot, sizeof(double) * 9);
    memcpy(e->trans, trans, sizeof(double) * 3);
    e->has_kernel = has_kernel;
    e->huber_delta = huber_delta;
    return g->ne - 1;
}

/* ==========================================================================
 * edges: computeError / chi2
 * ========================================================================== */
static int edge_dim(const sv_s3_edge* e) { return e->kind == SV_S3_EDGE_GRAPH ? 7 : 2; }

/* 'rot * pos_w + trans' (mixed matrix*vector rule, then the add) */
static void rt_apply(const double rot[9], const double trans[3], const double p[3], double out[3]) {
    double t[3];
    sv_mat3_mulv(rot, p, t);
    out[0] = t[0] + trans[0];
    out[1] = t[1] + trans[1];
    out[2] = t[2] + trans[2];
}

static void project_err(const sv_s3_edge* e, const double pos_c[3], double err[2]) {
    const double px = e->fx * pos_c[0] / pos_c[2] + e->cx;
    const double py = e->fy * pos_c[1] / pos_c[2] + e->cy;
    err[0] = e->obs[0] - px;
    err[1] = e->obs[1] - py;
}

static void edge_error_v(const sv_s3_graph* g, const sv_s3_edge* e, const sv_sim3* est0, const sv_sim3* est1, double* err) {
    if (e->kind == SV_S3_EDGE_GRAPH) {
        sv_sim3 t, inv, r;
        sv_sim3_mul(&e->meas, est0, &t);
        sv_sim3_inverse(est1, &inv);
        sv_sim3_mul(&t, &inv, &r);
        sv_sim3_log(&r, err);
    }
    else if (e->kind == SV_S3_EDGE_FORWARD) {
        double pos_2[3], pos_1[3];
        rt_apply(e->rot, e->trans, e->pos_w, pos_2);
        sv_sim3_map(est0, pos_2, pos_1);
        project_err(e, pos_1, err);
    }
    else {
        sv_sim3 inv;
        double pos_1[3], pos_2[3];
        sv_sim3_inverse(est0, &inv);
        rt_apply(e->rot, e->trans, e->pos_w, pos_1);
        sv_sim3_map(&inv, pos_1, pos_2);
        project_err(e, pos_2, err);
    }
    (void)g;
}

double sv_s3_edge_chi2(const sv_s3_edge* e) {
    if (e->kind == SV_S3_EDGE_GRAPH) {
        double p[7];
        int k;
        for (k = 0; k < 7; ++k) {
            p[k] = e->err[k] * e->err[k];
        }
        return sum7_v(p);
    }
    else {
        const double ie0 = e->info * e->err[0];
        const double ie1 = e->info * e->err[1];
        return ie0 * e->err[0] + ie1 * e->err[1];
    }
}

static int edge_all_fixed(const sv_s3_graph* g, const sv_s3_edge* e) {
    return g->v[e->v0].fixed && (e->v1 < 0 || g->v[e->v1].fixed);
}

void sv_s3_compute_active_errors(sv_s3_graph* g) {
    int k;
    for (k = 0; k < g->n_active_edges; ++k) {
        sv_s3_edge* e = &g->e[g->active_edges[k]];
        edge_error_v(g, e, &g->v[e->v0].est, e->v1 >= 0 ? &g->v[e->v1].est : NULL, e->err);
    }
}

double sv_s3_active_robust_chi2(const sv_s3_graph* g) {
    double chi = 0.0;
    int k;
    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_s3_edge* e = &g->e[g->active_edges[k]];
        if (e->has_kernel) {
            double rho[3];
            sv_huber_robustify(e->huber_delta, sv_s3_edge_chi2(e), rho);
            chi += rho[0];
        }
        else {
            chi += sv_s3_edge_chi2(e);
        }
    }
    return chi;
}

/* shot_vertex::oplusImpl / transform_vertex::oplusImpl */
static void vertex_oplus(const sv_s3_graph* g, sv_s3_vertex* v, const double* upd) {
    double u[7];
    sv_sim3 s, out;
    memcpy(u, upd, sizeof(u));
    if (g->fix_scale) {
        u[6] = 0.0;
    }
    sv_sim3_exp(u, &s);
    sv_sim3_mul(&s, &v->est, &out);
    v->est = out;
}

/* BaseFixedSizedEdge::linearizeOplusN: central differences on vertex 'which' */
static void numeric_jacobian(sv_s3_graph* g, const sv_s3_edge* e, int which, double J[7][7]) {
    const double delta = 1e-9;
    const double scalar = 1.0 / (2.0 * delta);
    sv_s3_vertex* vx = &g->v[which == 0 ? e->v0 : e->v1];
    const int dim = edge_dim(e);
    int d, r;
    for (d = 0; d < 7; ++d) {
        double upd[7] = {0, 0, 0, 0, 0, 0, 0};
        double ep[7], em[7];
        const sv_sim3 bak = vx->est; /* push */
        upd[d] = delta;
        vertex_oplus(g, vx, upd);
        edge_error_v(g, e, &g->v[e->v0].est, e->v1 >= 0 ? &g->v[e->v1].est : NULL, ep);
        vx->est = bak; /* pop */
        upd[d] = -delta;
        vertex_oplus(g, vx, upd);
        edge_error_v(g, e, &g->v[e->v0].est, e->v1 >= 0 ? &g->v[e->v1].est : NULL, em);
        vx->est = bak;
        for (r = 0; r < dim; ++r) {
            J[r][d] = scalar * (ep[r] - em[r]);
        }
    }
}

/* ==========================================================================
 * BlockSolver workspace (pose vertices only, 7-dim blocks)
 * ========================================================================== */
struct sv_s3_solver {
    int P;
    int size;
    int* hs_col; /* P+1 */
    int* hs_row; /* n_hs */
    int n_hs;
    double (*hpp)[7][7]; /* off-diagonal Hpp blocks (diagonal ones stay in the vertices) */
    double (*hs_blk)[7][7];
    int* edge_slot; /* per active-edge position: slot of the (min,max) block or -1 */
    int* sc_p;
    int* sc_i;
    double* sc_x;
    int sllt_ready;
    sv_sllt sllt;
    double* x;
    double* b;
    double* bschur;
    size_t x_cap;
    double (*bak)[7];
};

static void solver_free_structure(sv_s3_solver* s) {
    free(s->hs_col);
    free(s->hs_row);
    free(s->hpp);
    free(s->hs_blk);
    free(s->edge_slot);
    free(s->sc_p);
    free(s->sc_i);
    free(s->sc_x);
    free(s->bak);
    s->hs_col = s->hs_row = s->edge_slot = s->sc_p = s->sc_i = NULL;
    s->hpp = s->hs_blk = NULL;
    s->sc_x = NULL;
    s->bak = NULL;
    if (s->sllt_ready) {
        sv_sllt_free(&s->sllt);
        s->sllt_ready = 0;
    }
}

static void solver_free(sv_s3_solver* s) {
    solver_free_structure(s);
    free(s->x);
    free(s->b);
    free(s->bschur);
    s->x = s->b = s->bschur = NULL;
}

static int cmp_pair(const void* a, const void* b) {
    const int* x = (const int*)a;
    const int* y = (const int*)b;
    if (x[0] != y[0]) {
        return x[0] < y[0] ? -1 : 1;
    }
    return (x[1] > y[1]) - (x[1] < y[1]);
}

static sv_s3_solver* get_solver(sv_s3_graph* g) {
    if (!g->solver) {
        g->solver = calloc(1, sizeof(sv_s3_solver));
    }
    return (sv_s3_solver*)g->solver;
}

/* SparseOptimizer::initializeOptimization(level = 0) */
int sv_s3_initialize_optimization(sv_s3_graph* g) {
    int i, k;
    if (g->ne == 0) {
        return 0;
    }
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
        const sv_s3_edge* e = &g->e[k];
        if (e->level == 0 && !edge_all_fixed(g, e)) {
            g->active_edges[g->n_active_edges++] = k;
            g->v[e->v0].active = 1;
            if (e->v1 >= 0) {
                g->v[e->v1].active = 1;
            }
        }
    }
    /* buildIndexMapping over _activeVertices (id order == array order) */
    g->ivmap = (int*)malloc(sizeof(int) * (size_t)(g->nv > 0 ? g->nv : 1));
    {
        int n = 0;
        for (i = 0; i < g->nv; ++i) {
            sv_s3_vertex* v = &g->v[i];
            if (!v->active) {
                continue;
            }
            if (!v->fixed) {
                v->hidx = n;
                g->ivmap[n] = i;
                ++n;
            }
            else {
                v->hidx = -1;
            }
        }
        g->n_ivmap = n;
    }
    /* postIteration(-1): terminate_action (only when the graph has one), iteration < 0 */
    if (g->use_terminate) {
        sv_s3_compute_active_errors(g);
        g->aux_flag = 0;
    }
    g->stopped_by_terminate = 0;
    return g->n_ivmap > 0;
}

static int terminate_requested(const sv_s3_graph* g) { return g->use_terminate ? (g->aux_flag != 0) : 0; }

/* BlockSolver::buildStructure */
static int build_structure(sv_s3_graph* g) {
    sv_s3_solver* s = get_solver(g);
    int i, k;
    solver_free_structure(s);
    s->P = g->n_ivmap;
    s->size = BD * s->P;
    if (s->P == 0) {
        return 0;
    }
    if (s->x_cap < (size_t)s->size) {
        s->x_cap = 2 * (size_t)s->size;
        free(s->x);
        free(s->b);
        free(s->bschur);
        s->x = (double*)calloc(s->x_cap, sizeof(double));
        s->b = (double*)calloc(s->x_cap, sizeof(double));
        s->bschur = (double*)calloc(s->x_cap, sizeof(double));
    }
    {
        /* pattern: diagonal blocks + (min,max) block of every active edge with two free vertices */
        int* pp = (int*)malloc(sizeof(int) * 2 * (size_t)(s->P + g->n_active_edges + 1));
        int np = 0, nuniq = 0;
        for (i = 0; i < s->P; ++i) {
            pp[2 * np + 0] = i; /* column */
            pp[2 * np + 1] = i; /* row */
            ++np;
        }
        for (k = 0; k < g->n_active_edges; ++k) {
            const sv_s3_edge* e = &g->e[g->active_edges[k]];
            if (e->v1 >= 0) {
                const int a = g->v[e->v0].hidx, b = g->v[e->v1].hidx;
                if (a >= 0 && b >= 0 && a != b) {
                    pp[2 * np + 0] = a > b ? a : b;
                    pp[2 * np + 1] = a > b ? b : a;
                    ++np;
                }
            }
        }
        qsort(pp, (size_t)np, 2 * sizeof(int), cmp_pair);
        s->hs_col = (int*)calloc((size_t)s->P + 1, sizeof(int));
        s->hs_row = (int*)malloc(sizeof(int) * (size_t)np);
        for (i = 0; i < np; ++i) {
            if (i > 0 && pp[2 * i] == pp[2 * (i - 1)] && pp[2 * i + 1] == pp[2 * (i - 1) + 1]) {
                continue;
            }
            s->hs_row[nuniq] = pp[2 * i + 1];
            s->hs_col[pp[2 * i] + 1]++;
            ++nuniq;
        }
        s->n_hs = nuniq;
        for (i = 0; i < s->P; ++i) {
            s->hs_col[i + 1] += s->hs_col[i];
        }
        free(pp);
    }
    s->hpp = (double(*)[7][7])calloc((size_t)(s->n_hs > 0 ? s->n_hs : 1), sizeof(double[7][7]));
    s->hs_blk = (double(*)[7][7])calloc((size_t)(s->n_hs > 0 ? s->n_hs : 1), sizeof(double[7][7]));
    s->edge_slot = (int*)malloc(sizeof(int) * (size_t)(g->n_active_edges > 0 ? g->n_active_edges : 1));
    for (k = 0; k < g->n_active_edges; ++k) {
        const sv_s3_edge* e = &g->e[g->active_edges[k]];
        s->edge_slot[k] = -1;
        if (e->v1 >= 0) {
            const int a = g->v[e->v0].hidx, b = g->v[e->v1].hidx;
            if (a >= 0 && b >= 0 && a != b) {
                const int row = a > b ? b : a, col = a > b ? a : b;
                int lo = s->hs_col[col], hi = s->hs_col[col + 1] - 1;
                while (lo <= hi) {
                    const int mid = (lo + hi) / 2;
                    if (s->hs_row[mid] == row) {
                        s->edge_slot[k] = mid;
                        break;
                    }
                    if (s->hs_row[mid] < row) {
                        lo = mid + 1;
                    }
                    else {
                        hi = mid - 1;
                    }
                }
            }
        }
    }
    /* scalar CCS (upper triangle) -- SparseBlockMatrixCCS::fillCCS(Cp,Ci,Cx,true) */
    {
        const int n = s->size;
        int nz = 0, bc, c, bi;
        s->sc_p = (int*)malloc(sizeof(int) * ((size_t)n + 1));
        for (bc = 0; bc < s->P; ++bc) {
            for (c = 0; c < BD; ++c) {
                for (bi = s->hs_col[bc]; bi < s->hs_col[bc + 1]; ++bi) {
                    nz += (s->hs_row[bi] == bc) ? c + 1 : BD;
                }
            }
        }
        s->sc_i = (int*)malloc(sizeof(int) * (size_t)(nz > 0 ? nz : 1));
        s->sc_x = (double*)malloc(sizeof(double) * (size_t)(nz > 0 ? nz : 1));
        nz = 0;
        for (bc = 0; bc < s->P; ++bc) {
            for (c = 0; c < BD; ++c) {
                s->sc_p[bc * BD + c] = nz;
                for (bi = s->hs_col[bc]; bi < s->hs_col[bc + 1]; ++bi) {
                    const int rstart = s->hs_row[bi] * BD;
                    const int elems = (s->hs_row[bi] == bc) ? c + 1 : BD;
                    int r;
                    for (r = 0; r < elems; ++r) {
                        s->sc_i[nz++] = rstart + r;
                    }
                }
            }
        }
        s->sc_p[n] = nz;
    }
    s->bak = (double(*)[7])malloc(sizeof(double[7]) * (size_t)s->P);
    return 1;
}

static double* hs_block(sv_s3_solver* s, int row, int col) {
    int lo = s->hs_col[col], hi = s->hs_col[col + 1] - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (s->hs_row[mid] == row) {
            return &s->hs_blk[mid][0][0];
        }
        if (s->hs_row[mid] < row) {
            lo = mid + 1;
        }
        else {
            hi = mid - 1;
        }
    }
    return NULL;
}

/* BaseFixedSizedEdge<7,Sim3,shot,shot>::constructQuadraticForm (information = I7, no kernel) */
static void quad_form_graph(sv_s3_graph* g, sv_s3_solver* s, int k, const sv_s3_edge* e, double J0[7][7], double J1[7][7]) {
    sv_s3_vertex* va = &g->v[e->v0];
    sv_s3_vertex* vb = &g->v[e->v1];
    double we[7], p[7];
    int i, j, kk;
    for (i = 0; i < 7; ++i) {
        we[i] = -e->err[i]; /* -I*e */
    }
    if (!va->fixed) {
        for (i = 0; i < 7; ++i) {
            for (kk = 0; kk < 7; ++kk) {
                p[kk] = J0[kk][i] * we[kk];
            }
            va->b[i] = va->b[i] + sum7_v(p);
        }
        for (i = 0; i < 7; ++i) {
            for (j = 0; j < 7; ++j) {
                for (kk = 0; kk < 7; ++kk) {
                    p[kk] = J0[kk][i] * J0[kk][j];
                }
                va->H[i][j] = va->H[i][j] + (i < 6 ? sum7_l(p) : sum7_n(p));
            }
        }
        if (!vb->fixed && s->edge_slot[k] >= 0) {
            double (*blk)[7] = s->hpp[s->edge_slot[k]];
            if (va->hidx > vb->hidx) {
                /* transposed block: hessianTransposed(v1 params, v0 params) += J1^T * J0 (all-N) */
                for (i = 0; i < 7; ++i) {
                    for (j = 0; j < 7; ++j) {
                        for (kk = 0; kk < 7; ++kk) {
                            p[kk] = J1[kk][i] * J0[kk][j];
                        }
                        blk[i][j] = blk[i][j] + sum7_n(p);
                    }
                }
            }
            else {
                /* hessian(v0 params, v1 params) += AtO * J1 */
                for (i = 0; i < 7; ++i) {
                    for (j = 0; j < 7; ++j) {
                        for (kk = 0; kk < 7; ++kk) {
                            p[kk] = J0[kk][i] * J1[kk][j];
                        }
                        blk[i][j] = blk[i][j] + (i < 6 ? sum7_l(p) : sum7_n(p));
                    }
                }
            }
        }
    }
    if (!vb->fixed) {
        for (i = 0; i < 7; ++i) {
            for (kk = 0; kk < 7; ++kk) {
                p[kk] = J1[kk][i] * we[kk];
            }
            vb->b[i] = vb->b[i] + sum7_v(p);
        }
        for (i = 0; i < 7; ++i) {
            for (j = 0; j < 7; ++j) {
                for (kk = 0; kk < 7; ++kk) {
                    p[kk] = J1[kk][i] * J1[kk][j];
                }
                vb->H[i][j] = vb->H[i][j] + (i < 6 ? sum7_l(p) : sum7_n(p));
            }
        }
    }
}

/* unary 2-dim edge on a 7-dim vertex (same arithmetic as the pose leaf / BA, 7 columns) */
static void quad_form_reproj(sv_s3_graph* g, const sv_s3_edge* e, double J[7][7]) {
    sv_s3_vertex* v = &g->v[e->v0];
    double omega_diag, we0, we1, AtO0[7], AtO1[7];
    int r, c;
    if (v->fixed) {
        return;
    }
    if (e->has_kernel) {
        double rho[3];
        sv_huber_robustify(e->huber_delta, sv_s3_edge_chi2(e), rho);
        we0 = (-(e->info * e->err[0])) * rho[1];
        we1 = (-(e->info * e->err[1])) * rho[1];
        omega_diag = rho[1] * e->info;
    }
    else {
        we0 = -(e->info * e->err[0]);
        we1 = -(e->info * e->err[1]);
        omega_diag = e->info;
    }
    for (r = 0; r < 7; ++r) {
        AtO0[r] = J[0][r] * omega_diag;
        AtO1[r] = J[1][r] * omega_diag;
    }
    for (r = 0; r < 7; ++r) {
        v->b[r] += J[0][r] * we0 + J[1][r] * we1;
        for (c = 0; c < 7; ++c) {
            v->H[r][c] += AtO0[r] * J[0][c] + AtO1[r] * J[1][c];
        }
    }
}

/* BlockSolver::buildSystem */
static void build_system(sv_s3_graph* g) {
    sv_s3_solver* s = get_solver(g);
    int i, k;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_s3_vertex* v = &g->v[g->ivmap[i]];
        memset(v->b, 0, sizeof(v->b));
        memset(v->H, 0, sizeof(v->H));
    }
    memset(s->hpp, 0, sizeof(double[7][7]) * (size_t)s->n_hs);
    for (k = 0; k < g->n_active_edges; ++k) {
        sv_s3_edge* e = &g->e[g->active_edges[k]];
        double J0[7][7], J1[7][7];
        memset(J0, 0, sizeof(J0));
        memset(J1, 0, sizeof(J1));
        if (edge_all_fixed(g, e)) {
            continue;
        }
        if (!g->v[e->v0].fixed) {
            numeric_jacobian(g, e, 0, J0);
        }
        if (e->v1 >= 0 && !g->v[e->v1].fixed) {
            numeric_jacobian(g, e, 1, J1);
        }
        if (e->kind == SV_S3_EDGE_GRAPH) {
            quad_form_graph(g, s, k, e, J0, J1);
        }
        else {
            quad_form_reproj(g, e, J0);
        }
    }
    {
        int off = 0;
        for (i = 0; i < g->n_ivmap; ++i) {
            memcpy(s->b + off, g->v[g->ivmap[i]].b, sizeof(double) * BD);
            off += BD;
        }
    }
}

static void set_lambda(sv_s3_graph* g, double lambda) {
    sv_s3_solver* s = get_solver(g);
    int i, d;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_s3_vertex* v = &g->v[g->ivmap[i]];
        for (d = 0; d < BD; ++d) {
            s->bak[i][d] = v->H[d][d];
            v->H[d][d] += lambda;
        }
    }
}

static void restore_diagonal(sv_s3_graph* g) {
    sv_s3_solver* s = get_solver(g);
    int i, d;
    for (i = 0; i < g->n_ivmap; ++i) {
        sv_s3_vertex* v = &g->v[g->ivmap[i]];
        for (d = 0; d < BD; ++d) {
            v->H[d][d] = s->bak[i][d];
        }
    }
}

/* BlockSolver::solve() with _doSchur and no landmarks: Hschur = Hpp, solved with LinearSolverEigen */
static int solver_solve(sv_s3_graph* g) {
    sv_s3_solver* s = get_solver(g);
    int i, r, c, k, bc, cc, nz = 0;
    const int n = s->size;
    memset(s->hs_blk, 0, sizeof(double[7][7]) * (size_t)s->n_hs);
    for (i = 0; i < g->n_ivmap; ++i) { /* _Hpp->add(*_Hschur): diagonal blocks */
        double (*blk)[7] = (double(*)[7])hs_block(s, i, i);
        const sv_s3_vertex* v = &g->v[g->ivmap[i]];
        for (r = 0; r < 7; ++r) {
            for (c = 0; c < 7; ++c) {
                blk[r][c] += v->H[r][c];
            }
        }
    }
    for (bc = 0; bc < s->P; ++bc) { /* off-diagonal blocks */
        for (k = s->hs_col[bc]; k < s->hs_col[bc + 1]; ++k) {
            if (s->hs_row[k] != bc) {
                for (r = 0; r < 7; ++r) {
                    for (c = 0; c < 7; ++c) {
                        s->hs_blk[k][r][c] += s->hpp[k][r][c];
                    }
                }
            }
        }
    }
    memcpy(s->bschur, s->b, sizeof(double) * (size_t)n);
    for (i = 0; i < n; ++i) {
        s->bschur[i] -= 0.0; /* _bschur[i] -= _coefficients[i] (zero: no landmarks) */
    }
    for (bc = 0; bc < s->P; ++bc) {
        for (cc = 0; cc < BD; ++cc) {
            for (k = s->hs_col[bc]; k < s->hs_col[bc + 1]; ++k) {
                const int elems = (s->hs_row[k] == bc) ? cc + 1 : BD;
                for (r = 0; r < elems; ++r) {
                    s->sc_x[nz++] = s->hs_blk[k][r][cc];
                }
            }
        }
    }
    if (!s->sllt_ready) {
        int* blockP = (int*)malloc(sizeof(int) * (size_t)s->P);
        int* scalarP = (int*)malloc(sizeof(int) * (size_t)n);
        int sidx = 0;
        sv_amd_order(s->P, s->hs_col, s->hs_row, blockP);
        for (bc = 0; bc < s->P; ++bc) {
            int base = blockP[bc] * BD;
            for (cc = 0; cc < BD; ++cc) {
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
    sv_sllt_solve(&s->sllt, s->bschur, s->x);
    return 1;
}

/* ==========================================================================
 * Levenberg + SparseOptimizer::optimize
 * ========================================================================== */
static void push_all(sv_s3_graph* g) {
    int i;
    for (i = 0; i < g->nv; ++i) {
        g->v[i].bak = g->v[i].est;
    }
}

static void pop_all(sv_s3_graph* g) {
    int i;
    for (i = 0; i < g->nv; ++i) {
        g->v[i].est = g->v[i].bak;
    }
}

static void update_estimates(sv_s3_graph* g, const double* upd) {
    int i;
    for (i = 0; i < g->n_ivmap; ++i) {
        vertex_oplus(g, &g->v[g->ivmap[i]], upd);
        upd += BD;
    }
}

static double compute_lambda_init(sv_s3_graph* g) {
    double max_diag = 0.0;
    int i, j;
    for (i = 0; i < g->n_ivmap; ++i) {
        const sv_s3_vertex* v = &g->v[g->ivmap[i]];
        for (j = 0; j < BD; ++j) {
            const double a = fabs(v->H[j][j]);
            max_diag = a > max_diag ? a : max_diag;
        }
    }
    return TAU * max_diag;
}

/* OptimizationAlgorithmLevenberg::solve. Returns 1 = OK, 0 = Terminate. */
static int levenberg_solve(sv_s3_graph* g, int iteration) {
    sv_s3_solver* s = get_solver(g);
    double current_chi, rho = 0.0;
    int qmax = 0;
    if (iteration == 0) {
        if (!build_structure(g)) {
            return 0;
        }
    }
    sv_s3_compute_active_errors(g);
    current_chi = sv_s3_active_robust_chi2(g);
    build_system(g);
    if (iteration == 0) {
        g->current_lambda = compute_lambda_init(g);
        g->ni = 2.0;
    }
    do {
        int ok2;
        double temp_chi, scale = 0.0;
        size_t j;
        push_all(g);
        set_lambda(g, g->current_lambda);
        ok2 = solver_solve(g);
        update_estimates(g, s->x);
        restore_diagonal(g);
        sv_s3_compute_active_errors(g);
        temp_chi = sv_s3_active_robust_chi2(g);
        if (!ok2) {
            temp_chi = DBL_MAX;
        }
        rho = current_chi - temp_chi;
        for (j = 0; j < (size_t)s->size; ++j) {
            scale += s->x[j] * (g->current_lambda * s->x[j] + s->b[j]);
        }
        scale += 1e-3;
        rho /= scale;
        if (rho > 0 && isfinite(temp_chi)) {
            double alpha = 1.0 - pow(2 * rho - 1, 3);
            double scale_factor;
            alpha = alpha < GOOD_STEP_UPPER ? alpha : GOOD_STEP_UPPER;
            scale_factor = GOOD_STEP_LOWER > alpha ? GOOD_STEP_LOWER : alpha;
            g->current_lambda *= scale_factor;
            g->ni = 2.0;
            current_chi = temp_chi;
        }
        else {
            g->current_lambda *= g->ni;
            g->ni *= 2.0;
            pop_all(g);
            if (!isfinite(g->current_lambda)) {
                break;
            }
        }
        qmax++;
    } while (rho < 0 && qmax < MAX_TRIALS_AFTER_FAILURE && !terminate_requested(g));
    g->lev_iterations = qmax;
    if (qmax == MAX_TRIALS_AFTER_FAILURE || rho == 0 || !isfinite(g->current_lambda)) {
        return 0;
    }
    return 1;
}

static void record_iter(sv_s3_graph* g) {
    sv_s3_iter* it;
    if (g->n_iters == g->cap_iters) {
        g->cap_iters = g->cap_iters ? g->cap_iters * 2 : 32;
        g->iters = (sv_s3_iter*)realloc(g->iters, sizeof(sv_s3_iter) * (size_t)g->cap_iters);
    }
    it = &g->iters[g->n_iters++];
    it->chi2 = sv_s3_active_robust_chi2(g);
    it->lambda = g->current_lambda;
    it->lev_iter = g->lev_iterations;
    it->flag = terminate_requested(g) ? 1u : 0u;
}

/* stella_vslam terminate_action::operator() for iteration >= 0 */
static void terminate_action_post(sv_s3_graph* g, int iteration) {
    sv_s3_compute_active_errors(g);
    if (iteration == 0) {
        g->last_chi = sv_s3_active_robust_chi2(g);
    }
    else {
        int stop = 0;
        if (iteration < INT_MAX) {
            const double current_chi = sv_s3_active_robust_chi2(g);
            const double gain = (g->last_chi - current_chi) / current_chi;
            g->last_chi = current_chi;
            if (gain >= 0 && gain < g->gain_threshold) {
                stop = 1;
            }
        }
        else {
            stop = 1;
        }
        if (stop) {
            g->aux_flag = 1;
            g->stopped_by_terminate = 1;
        }
    }
}

int sv_s3_optimize(sv_s3_graph* g, int iterations) {
    int i, ok = 1, cj = 0;
    sv_s3_solver* s;
    g->n_iters = 0;
    if (g->n_ivmap == 0) {
        return -1;
    }
    s = get_solver(g);
    if (s->sllt_ready) { /* algorithm->init(online = false): fresh symbolic analysis */
        sv_sllt_free(&s->sllt);
        s->sllt_ready = 0;
    }
    for (i = 0; i < iterations && !terminate_requested(g) && ok; i++) {
        const int result = levenberg_solve(g, i);
        ok = (result == 1);
        ++cj;
        if (g->use_terminate) {
            terminate_action_post(g, i);
        }
        record_iter(g);
    }
    return cj;
}

/* ==========================================================================
 * transform_optimizer::optimize
 * ========================================================================== */
unsigned int sv_transform_optimize(const sv_transform_camera* cam1, const sv_transform_camera* cam2,
                                   const double rot_1w[9], const double trans_1w[3],
                                   const double rot_2w[9], const double trans_2w[3],
                                   const sv_transform_match* m, unsigned int n, unsigned char* rej,
                                   sv_sim3* sim3_12, float chi_sq, int fix_scale, unsigned int num_iter,
                                   sv_sim3* mid) {
    const float sqrt_chi_sq = sqrtf(chi_sq);
    sv_s3_graph g;
    unsigned int i, num_outliers = 0, num_inliers = 0;
    int v0;
    sv_s3_graph_init(&g);
    g.fix_scale = fix_scale;
    v0 = sv_s3_add_vertex(&g, 0, sim3_12, 0);
    for (i = 0; i < n; ++i) {
        rej[i] = 0;
        /* forward edge: keyframe-1 keypoint, landmark of keyframe 2 through (rot_2w, trans_2w) */
        sv_s3_add_reproj_edge(&g, SV_S3_EDGE_FORWARD, v0, m[i].obs1, m[i].info1, cam1->fx, cam1->fy, cam1->cx, cam1->cy,
                              m[i].pos_w_2, rot_2w, trans_2w, 1, (double)sqrt_chi_sq);
        sv_s3_add_reproj_edge(&g, SV_S3_EDGE_BACKWARD, v0, m[i].obs2, m[i].info2, cam2->fx, cam2->fy, cam2->cx, cam2->cy,
                              m[i].pos_w_1, rot_1w, trans_1w, 1, (double)sqrt_chi_sq);
    }
    sv_s3_initialize_optimization(&g);
    sv_s3_optimize(&g, 5);
    if (mid) {
        *mid = g.v[v0].est;
    }
    for (i = 0; i < n; ++i) {
        sv_s3_edge* e12 = &g.e[2 * i];
        sv_s3_edge* e21 = &g.e[2 * i + 1];
        if (sv_s3_edge_chi2(e12) < (double)chi_sq && sv_s3_edge_chi2(e21) < (double)chi_sq) {
            continue;
        }
        rej[i] = 1;
        e12->level = 1;
        e21->level = 1;
        ++num_outliers;
    }
    if (n - num_outliers < 10) {
        sv_s3_graph_free(&g);
        return 0;
    }
    sv_s3_initialize_optimization(&g);
    sv_s3_optimize(&g, (int)num_iter);
    for (i = 0; i < n; ++i) {
        const sv_s3_edge* e12 = &g.e[2 * i];
        const sv_s3_edge* e21 = &g.e[2 * i + 1];
        if (rej[i]) {
            continue;
        }
        if ((double)chi_sq < sv_s3_edge_chi2(e12) || (double)chi_sq < sv_s3_edge_chi2(e21)) {
            rej[i] = 2;
            continue;
        }
        ++num_inliers;
    }
    *sim3_12 = g.v[v0].est;
    sv_s3_graph_free(&g);
    return num_inliers;
}
