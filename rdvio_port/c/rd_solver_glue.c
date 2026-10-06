/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port: rdvio::Solver on the C map layer. See rd_solver_glue.h. */
#include "rd_solver_glue.h"
#include <stdlib.h>
#include <string.h>

struct rd_solver {
    rd_sv_problem pb;
    int pcap, rcap;
    double** user;                 /* the C state of every parameter block (write-back) */
    uint64_t next_rb;
};

rd_solver* rd_solver_create(int max_num_iterations) {
    rd_solver* s = (rd_solver*)calloc(1, sizeof *s);
    rd_sv_options* o = &s->pb.opt;
    o->linear_solver_type = RD_SV_SPARSE_SCHUR;
    o->max_num_iterations = max_num_iterations;
    o->function_tolerance = 1e-6; o->gradient_tolerance = 1e-10; o->parameter_tolerance = 1e-8;
    o->initial_trust_region_radius = 1e4; o->max_trust_region_radius = 1e16; o->min_trust_region_radius = 1e-32;
    o->min_relative_decrease = 1e-3; o->min_lm_diagonal = 1e-6; o->max_lm_diagonal = 1e32;
    o->jacobi_scaling = 1; o->max_num_consecutive_invalid_steps = 5; o->update_state_every_iteration = 1;
    return s;
}
void rd_solver_free(rd_solver* s) {
    int i;
    if (!s) return;
    for (i = 0; i < s->pb.np; ++i) free(s->pb.p[i].x);
    free(s->pb.p); free(s->pb.r); free(s->user); free(s);
}
const rd_sv_problem* rd_solver_problem(const rd_solver* s) { return &s->pb; }

static int find_param(const rd_solver* s, const double* user) {
    int i;
    for (i = s->pb.np - 1; i >= 0; --i) if (s->user[i] == user) return i;
    return -1;
}
/* AddParameterBlock (or the implicit add of AddResidualBlock): appended in call order, a known block is left as it is */
static int add_param(rd_solver* s, double* user, int size, int quaternion) {
    rd_sv_param* p;
    int i = find_param(s, user);
    if (i >= 0) return i;
    if (s->pb.np == s->pcap) {
        s->pcap = s->pcap ? 2 * s->pcap : 64;
        s->pb.p = (rd_sv_param*)realloc(s->pb.p, sizeof(rd_sv_param) * (size_t)s->pcap);
        s->user = (double**)realloc(s->user, sizeof(double*) * (size_t)s->pcap);
    }
    p = &s->pb.p[s->pb.np];
    memset(p, 0, sizeof *p);
    p->ptr = (uint64_t)(uintptr_t)user;
    p->size = size;
    p->tangent = quaternion ? 3 : size;
    p->kind = quaternion ? RD_SV_KIND_QUAT : RD_SV_KIND_NONE;
    p->x = (double*)malloc(sizeof(double) * (size_t)size);
    memcpy(p->x, user, sizeof(double) * (size_t)size);
    p->index = -1;
    s->user[s->pb.np] = user;
    return s->pb.np++;
}
static void set_constant(rd_solver* s, const double* user) { const int i = find_param(s, user); if (i >= 0) s->pb.p[i].constant = 1; }

void rd_solver_add_frame_states(rd_solver* s, rd_frame* f, int with_motion) {
    add_param(s, &f->pose_q.x, 4, 1);
    add_param(s, f->pose_p, 3, 0);
    if (f->tags & RD_TAG(RD_FT_FIX_POSE)) { set_constant(s, &f->pose_q.x); set_constant(s, f->pose_p); }
    if (with_motion) {
        add_param(s, f->motion.v, 3, 0);
        add_param(s, f->motion.bg, 3, 0);
        add_param(s, f->motion.ba, 3, 0);
        if (f->tags & RD_TAG(RD_FT_FIX_MOTION)) { set_constant(s, f->motion.v); set_constant(s, f->motion.bg); set_constant(s, f->motion.ba); }
    }
}
void rd_solver_add_track_states(rd_solver* s, rd_track* t) { add_param(s, &t->inv_depth, 1, 0); }

/* AddResidualBlock: the blocks in the order of the call (implicitly added when unknown) */
static rd_sv_resid* new_resid(rd_solver* s, int type, int loss, int nres, int nb, double* const* blocks, const int* sizes) {
    rd_sv_resid* rb;
    int k;
    if (s->pb.nr == s->rcap) {
        s->rcap = s->rcap ? 2 * s->rcap : 64;
        s->pb.r = (rd_sv_resid*)realloc(s->pb.r, sizeof(rd_sv_resid) * (size_t)s->rcap);
    }
    rb = &s->pb.r[s->pb.nr++];
    memset(rb, 0, sizeof *rb);
    rb->ptr = ++s->next_rb;
    rb->type = type; rb->loss = loss; rb->nres = nres; rb->nb = nb;
    for (k = 0; k < nb; ++k) rb->blk[k] = add_param(s, blocks[k], sizes[k], 0);
    return rb;
}
static void live(rd_sv_resid* rb, const double* p, int n) {
    rd_sv_live* l = &rb->live[rb->nlive++];
    l->ptr = (uint64_t)(uintptr_t)p; l->size = n; l->pidx = -1;
    memcpy(l->v, p, sizeof(double) * (size_t)n);
}
static void extr(rd_extrinsic* e, const ok_quat* q, const double p[3]) { e->q_cs = *q; memcpy(e->p_cs, p, sizeof e->p_cs); }
/* CeresReprojectionErrorFactor(frame, track): payload of the factor of keypoint kp of frame */
static void vis(rd_sv_vis* v, const rd_frame* f, size_t kp, const rd_track* t) {
    const rd_frame* fr = t->ref[0].frame;
    memcpy(v->z, f->bearing + 3 * kp, sizeof v->z);
    memcpy(v->z_ref, fr->bearing + 3 * t->ref[0].kp, sizeof v->z_ref);
    extr(&v->cam_ref, &fr->cam_q, fr->cam_p);
    extr(&v->cam_tgt, &f->cam_q, f->cam_p);
    memcpy(v->sqrt_inv_cov, f->sqrt_inv_cov, sizeof v->sqrt_inv_cov);
}

void rd_solver_add_rpe(rd_solver* s, rd_frame* f, size_t kp) {
    rd_track* t = f->track[kp];
    rd_frame* fr = t->ref[0].frame;
    double* b[5]; int sz[5] = {4, 3, 4, 3, 1};
    rd_sv_resid* rb;
    b[0] = &f->pose_q.x; b[1] = f->pose_p; b[2] = &fr->pose_q.x; b[3] = fr->pose_p; b[4] = &t->inv_depth;
    rb = new_resid(s, RD_SV_T_RPE, RD_SV_LOSS_CAUCHY, 2, 5, b, sz);
    vis(&rb->term.vis, f, kp, t);
}
void rd_solver_add_rpp(rd_solver* s, rd_frame* f, rd_track* t) {
    rd_frame* fr = t->ref[0].frame;
    const size_t kp = rd_track_keypoint_index(t, f);
    double* b[2]; int sz[2] = {4, 3};
    rd_sv_resid* rb;
    b[0] = &f->pose_q.x; b[1] = f->pose_p;
    rb = new_resid(s, RD_SV_T_RPP, RD_SV_LOSS_CAUCHY, 2, 2, b, sz);
    vis(&rb->term.vis, f, kp, t);
    live(rb, &fr->pose_q.x, 4); live(rb, fr->pose_p, 3); live(rb, &t->inv_depth, 1);
}
void rd_solver_add_rop(rd_solver* s, rd_frame* f, rd_track* t) {
    rd_frame* fr = t->ref[0].frame;
    const size_t kp = rd_track_keypoint_index(t, f);
    double* b[1]; int sz[1] = {4};
    rd_sv_resid* rb;
    b[0] = &f->pose_q.x;
    rb = new_resid(s, RD_SV_T_ROP, RD_SV_LOSS_CAUCHY, 2, 1, b, sz);
    vis(&rb->term.vis, f, kp, t);
    live(rb, &fr->pose_q.x, 4);
}
static void pie(rd_sv_pie* e, const rd_frame* fi, const rd_frame* fj, const rd_preint* pre) {
    e->pre = *pre;
    e->imu_i_q = fi->imu_q; memcpy(e->imu_i_p, fi->imu_p, sizeof e->imu_i_p);
    e->imu_j_q = fj->imu_q; memcpy(e->imu_j_p, fj->imu_p, sizeof e->imu_j_p);
}
void rd_solver_add_pie(rd_solver* s, rd_frame* fi, rd_frame* fj, const rd_preint* pre) {
    double* b[10]; int sz[10] = {4, 3, 3, 3, 3, 4, 3, 3, 3, 3};
    rd_sv_resid* rb;
    b[0] = &fi->pose_q.x; b[1] = fi->pose_p; b[2] = fi->motion.v; b[3] = fi->motion.bg; b[4] = fi->motion.ba;
    b[5] = &fj->pose_q.x; b[6] = fj->pose_p; b[7] = fj->motion.v; b[8] = fj->motion.bg; b[9] = fj->motion.ba;
    rb = new_resid(s, RD_SV_T_PIE, RD_SV_LOSS_NONE, 15, 10, b, sz);
    pie(&rb->term.pie, fi, fj, pre);
    live(rb, fi->motion.bg, 3); live(rb, fi->motion.ba, 3);
}
void rd_solver_add_pip(rd_solver* s, rd_frame* fi, rd_frame* fj, const rd_preint* pre) {
    double* b[5]; int sz[5] = {4, 3, 3, 3, 3};
    rd_sv_resid* rb;
    b[0] = &fj->pose_q.x; b[1] = fj->pose_p; b[2] = fj->motion.v; b[3] = fj->motion.bg; b[4] = fj->motion.ba;
    rb = new_resid(s, RD_SV_T_PIP, RD_SV_LOSS_NONE, 15, 5, b, sz);
    pie(&rb->term.pie, fi, fj, pre);
    live(rb, &fi->pose_q.x, 4); live(rb, fi->pose_p, 3); live(rb, fi->motion.v, 3); live(rb, fi->motion.bg, 3); live(rb, fi->motion.ba, 3);
}
void rd_solver_add_marg(rd_solver* s, rd_marg* m, rd_map* map) {
    double* b[RD_SV_MAXB]; int sz[RD_SV_MAXB];
    int i, nb = 0;
    size_t j;
    rd_sv_resid* rb;
    for (i = 0; i < m->nf && nb + 5 <= RD_SV_MAXB; ++i) {
        rd_frame* f = NULL;
        for (j = 0; j < rd_map_frame_num(map); ++j) if (rd_map_get_frame(map, j)->id == m->ids[i]) { f = rd_map_get_frame(map, j); break; }
        if (!f) return;                               /* a linearization frame left the map: not reachable upstream */
        b[nb] = &f->pose_q.x; sz[nb++] = 4; b[nb] = f->pose_p; sz[nb++] = 3;
        b[nb] = f->motion.v; sz[nb++] = 3; b[nb] = f->motion.bg; sz[nb++] = 3; b[nb] = f->motion.ba; sz[nb++] = 3;
    }
    rb = new_resid(s, RD_SV_T_MAR, RD_SV_LOSS_NONE, 15 * m->nf, nb, b, sz);
    rb->marg = m;                                     /* owned by the map, not by the problem */
}

int rd_solver_solve(rd_solver* s, const rd_sv_hooks* hooks) {
    rd_sv_hooks none;
    int i, k, j, term;
    for (i = 0; i < s->pb.nr; ++i)                    /* live memory that is a parameter block reads its user state */
        for (k = 0; k < s->pb.r[i].nlive; ++k) {
            s->pb.r[i].live[k].pidx = -1;
            for (j = 0; j < s->pb.np; ++j) if (s->pb.p[j].ptr == s->pb.r[i].live[k].ptr) { s->pb.r[i].live[k].pidx = j; break; }
        }
    if (!hooks) { memset(&none, 0, sizeof none); hooks = &none; }
    term = rd_sv_solve(&s->pb, hooks);
    for (i = 0; i < s->pb.np; ++i) memcpy(s->user[i], s->pb.p[i].x, sizeof(double) * (size_t)s->pb.p[i].size);
    return term == RD_SV_CONVERGENCE || term == RD_SV_NO_CONVERGENCE || term == RD_SV_USER_SUCCESS;
}
