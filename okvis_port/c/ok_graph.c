/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 5b: ViGraph::updateLandmarks and the module-5 dump readers. See ok_graph.h. */
#include "ok_graph.h"

#include <math.h>
#include <string.h>

#include "ok_cam.h"
#include "ok_kin.h"

double ok_v4_norm(const double v[4]) {
    const double p0 = v[0] * v[0], p1 = v[1] * v[1], p2 = v[2] * v[2], p3 = v[3] * v[3];
    return sqrt((p0 + p2) + (p1 + p3));
}

void ok_graph_update_landmark(double hp[4], const ok_lm_obs* obs, int nobs, double* quality_out, int* initialised_out) {
    const int num = nobs;
    int isInitialised = 0, behind = 0, o, i, j;
    double quality = 0.0, best_err = 1.0e12, best_pos[4] = {0.0, 0.0, 0.0, 0.0}, minD = 1.0e12, H[9];
    double hp_W[4];
    memcpy(hp_W, hp, sizeof hp_W);
    for (i = 0; i < 9; ++i) H[i] = 0.0;
    if (num > 0) {
        for (o = 0; o < num; ++o) {
            const ok_lm_obs* ob = &obs[o];
            ok_tf T_WS, T_SCi, T_WCi, T_CiW;
            double pos_Ci[4], dir[3], dir_W[3], err[2], J1[8], J1m[6], err_norm;
            double* jac[3];
            double* jacmin[3];
            const double* params[3];
            ok_tf_convert(&T_WS, ob->pose);   /* T_WS = state.pose->estimate() */
            ok_tf_convert(&T_SCi, ob->extr);  /* T_SCi = state.extrinsics.at(ci)->estimate() */
            ok_tf_mul(&T_WS, &T_SCi, &T_WCi, 1);
            ok_tf_inverse(&T_WCi, &T_CiW, 1);
            ok_tf_mul_v4(&T_CiW, hp_W, pos_Ci, 1); /* pos_Ci = T_WCi.inverse() * hp_W */
            ok_m3_mulv(T_WCi.C, pos_Ci, dir);       /* T_WCi.C() * pos_Ci.head<3>() */
            ok_v3_normalized(dir, dir_W);
            if (fabs(pos_Ci[3]) > 1.0e-12) {
                const double w = pos_Ci[3];
                pos_Ci[0] = pos_Ci[0] / w;
                pos_Ci[1] = pos_Ci[1] / w;
                pos_Ci[2] = pos_Ci[2] / w;
                pos_Ci[3] = pos_Ci[3] / w;
            }
            if (pos_Ci[2] < 0.1) behind = 1;
            if (pos_Ci[2] < 0.0) { dir_W[0] = -dir_W[0]; dir_W[1] = -dir_W[1]; dir_W[2] = -dir_W[2]; }
            params[0] = ob->pose;
            params[1] = hp;
            params[2] = ob->extr;
            jac[0] = NULL; jac[1] = J1; jac[2] = NULL;
            jacmin[0] = NULL; jacmin[1] = J1m; jacmin[2] = NULL;
            ok_reproj_err_evaluate(&ob->err, params, err, jac, jacmin);
            err_norm = sqrt(err[0] * err[0] + err[1] * err[1]);
            if (err_norm > 2.5) continue;
            if (fabs(pos_Ci[3]) > 1.0e-12) {
                if (pos_Ci[2] < minD) minD = pos_Ci[2];
            }
            if (err_norm < best_err && ok_v4_norm(pos_Ci) > 0.0001) {
                const double nrm = ok_v4_norm(pos_Ci);
                const double dist = (0.1 < nrm) ? nrm : 0.1; /* std::max(0.1, pos_Ci.norm()) */
                best_pos[0] = T_WCi.r[0] + dist * dir_W[0];
                best_pos[1] = T_WCi.r[1] + dist * dir_W[1];
                best_pos[2] = T_WCi.r[2] + dist * dir_W[2];
                best_pos[3] = 1.0;
                best_err = err_norm;
            }
            /* H += J1_minimal.transpose() * J1_minimal: J1_minimal is a column-major Matrix<2,3> whose buffer the error
             * term filled row-major, so its logical (i,a) is J1m[i + 2a] */
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) {
                    const double p0 = J1m[0 + 2 * i] * J1m[0 + 2 * j];
                    const double p1 = J1m[1 + 2 * i] * J1m[1 + 2 * j];
                    H[i + 3 * j] = H[i + 3 * j] + (p0 + p1);
                }
        }
        {
            double ev[3], V[9], s0;
            ok_selfadjoint_eig(3, H, ev, V, 0);
            s0 = sqrt((1.0e-12 < ev[0]) ? ev[0] : 1.0e-12);
            quality = (minD - 3.0 / s0) / fabs((1.0e-12 < minD) ? minD : 1.0e-12);
            if (behind) quality = 0.0;
            if (quality > 0.15) {
                isInitialised = 1;
            } else {
                if (behind && ok_v4_norm(best_pos) > 1.0e-12) memcpy(hp, best_pos, sizeof best_pos); /* reset along best ray */
            }
        }
    }
    *initialised_out = isInitialised;
    *quality_out = (0.0 < quality) ? quality : 0.0;
}

/* ---- dump readers ---- */
typedef struct cur { const unsigned char* p; size_t off, len; int bad; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); else c->bad = 1; c->off += 4; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static void cf64n(cur* c, double* out, size_t n) { size_t i; for (i = 0; i < n; ++i) out[i] = cf64(c); }

int ok_reproj_payload_read(const unsigned char* p, size_t len, ok_reproj_err* e) {
    cur c; ok_cam cam; uint32_t tag, w, h, nd, i; double f[4], d[OK_CAM_MAX_DIST], meas[2], info[4];
    c.p = p; c.off = 0; c.len = len; c.bad = 0;
    memset(d, 0, sizeof d);
    tag = cu32(&c); w = cu32(&c); h = cu32(&c);
    for (i = 0; i < 4; ++i) f[i] = cf64(&c);
    nd = cu32(&c);
    if (nd > OK_CAM_MAX_DIST) return -1;
    for (i = 0; i < nd; ++i) d[i] = cf64(&c);
    if (tag != OK_CAM_RADTAN && tag != OK_CAM_EQUIDISTANT && tag != OK_CAM_NODIST) return -1;
    cf64n(&c, meas, 2);
    cf64n(&c, info, 4); /* row-major 2x2 == column-major (symmetric) */
    if (c.bad) return -1;
    ok_cam_init(&cam, (int)tag, (int)w, (int)h, f[0], f[1], f[2], f[3], d);
    ok_reproj_err_init(e, &cam, meas, info);
    return (int)c.off;
}

static void read_t7(cur* c, ok_tf* T) {
    double r[3]; ok_quat q;
    cf64n(c, r, 3);
    q.x = cf64(c); q.y = cf64(c); q.z = cf64(c); q.w = cf64(c);
    /* the stored linearisation point is a cached Transformation: C from q (no renormalisation: the stored q is unit) */
    memcpy(T->r, r, sizeof r);
    T->q = q;
    ok_quat_to_mat3(&q, T->C);
}

int ok_tp_payload_read(const unsigned char* p, size_t len, int kind, ok_tp_std* s, ok_tp_ext* x) {
    cur c;
    c.p = p; c.off = 0; c.len = len; c.bad = 0;
    if (kind == 7 || kind == 8) {
        memset(s, 0, sizeof *s);
        s->is_computed = (int)cu32(&c);
        cf64n(&c, s->DeltaX, 6);
        cf64n(&c, s->J, 36);
        read_t7(&c, &s->lin_T_S0S1);
        return c.bad ? -1 : (int)c.off;
    } else if (kind == 9 || kind == 10) {
        uint32_t n, ne, i;
        memset(x, 0, sizeof *x);
        x->is_computed = (int)cu32(&c);
        n = cu32(&c);
        if (n < 6 || n > 6 + 6 * OK_TP_MAXEXTR || (n - 6) % 6 != 0) return -1;
        x->n = (int)n;
        cf64n(&c, x->DeltaX, n);
        cf64n(&c, x->J, (size_t)n * n);
        read_t7(&c, &x->lin_T_S0S1);
        ne = cu32(&c);
        if (ne > OK_TP_MAXEXTR || ne != (n - 6) / 6) return -1;
        x->nextr = (int)ne;
        for (i = 0; i < ne; ++i) {
            x->extr_present[i] = (int)cu32(&c);
            read_t7(&c, &x->lin_T_SC[i]);
        }
        return c.bad ? -1 : (int)c.off;
    }
    return -1;
}
