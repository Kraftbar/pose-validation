/* OK_PORT_SOURCES: check_ok_kin.c ok_kin.c ok_eigen.c */
/* Bit-exactness harness for okvis_port module 2b (kinematics: Transformation, sinc/deltaQ/rightJacobian).
 *
 *   check_ok_kin <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records_per_kind]
 *
 * Replays every record of <dump_dir>/kin_<kind>.bin (layouts in ok_kin.h) through the C module and compares all
 * outputs bitwise (memcmp, so signed zeros count). Prints one line per kind
 *   "  <kind>: <mismatching values>/<compared values> (<failing records>/<records> records)"
 * and as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0.
 * (Dumps come from tools/run_okvis_reference.py --dump; EuRoC-derived, never committed.)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_kin.h"

static FILE* G_f;
static int G_eof;

static void rd(void* p, size_t n) {
    if (n && fread(p, 1, n, G_f) != n) G_eof = 1;
}
static uint32_t rd_u32(void) { uint32_t v = 0; rd(&v, 4); return v; }
static void rd_f64n(double* p, size_t n) { rd(p, 8 * n); }

typedef struct pose { int cache; double c[7]; double C[9]; } pose;
static void rd_pose(pose* p) {
    memset(p, 0, sizeof *p);
    p->cache = (int)rd_u32();
    rd_f64n(p->c, 7);
    if (p->cache) rd_f64n(p->C, 9);
}
static ok_tf to_tf(const pose* p) {
    ok_tf t;
    t.r[0] = p->c[0]; t.r[1] = p->c[1]; t.r[2] = p->c[2];
    t.q.x = p->c[3]; t.q.y = p->c[4]; t.q.z = p->c[5]; t.q.w = p->c[6];
    memcpy(t.C, p->C, sizeof t.C);
    return t;
}
static void tf_coeffs(const ok_tf* t, double c[7]) {
    c[0] = t->r[0]; c[1] = t->r[1]; c[2] = t->r[2];
    c[3] = t->q.x; c[4] = t->q.y; c[5] = t->q.z; c[6] = t->q.w;
}

typedef struct counts { long bad, tot, recs, badrecs; } counts;
static int cmpv(counts* c, const double* a, const double* b, size_t n) {
    size_t i;
    int bad = 0;
    for (i = 0; i < n; ++i) {
        c->tot++;
        if (memcmp(&a[i], &b[i], 8) != 0) { c->bad++; bad++; }
    }
    return bad;
}
static int cmp_pose(counts* c, const ok_tf* got, const pose* want) {
    double gc[7];
    int bad;
    tf_coeffs(got, gc);
    bad = cmpv(c, gc, want->c, 7);
    if (want->cache) bad += cmpv(c, got->C, want->C, 9);
    return bad;
}

static const char* G_dir;
static long G_max;
static long G_total_bad, G_total;

#define KIND_BEGIN(name)                                                                      \
    static void check_##name(void) {                                                          \
        counts c;                                                                             \
        char path[1024];                                                                      \
        long rec = 0;                                                                         \
        memset(&c, 0, sizeof c);                                                              \
        snprintf(path, sizeof path, "%s/kin_%s.bin", G_dir, #name);                           \
        G_f = fopen(path, "rb");                                                              \
        if (!G_f) { printf("  %s: (no dump file)\n", #name); return; }                       \
        G_eof = 0;                                                                            \
        while (!G_eof && (G_max < 0 || rec < G_max)) {                                        \
            int bad = 0;
#define KIND_END(name)                                                                        \
            if (G_eof) break;                                                                 \
            rec++; c.recs++;                                                                  \
            if (bad) c.badrecs++;                                                             \
        }                                                                                     \
        fclose(G_f);                                                                          \
        printf("  %s: %ld/%ld (%ld/%ld records)\n", #name, c.bad, c.tot, c.badrecs, c.recs);  \
        G_total_bad += c.bad; G_total += c.tot;                                               \
    }

KIND_BEGIN(ctor_rq) {
    double r[3], q[4]; pose out; ok_tf t; ok_quat qq;
    rd_f64n(r, 3); rd_f64n(q, 4); rd_pose(&out);
    if (G_eof) break;
    qq.x = q[0]; qq.y = q[1]; qq.z = q[2]; qq.w = q[3];
    ok_tf_from_rq(&t, r, &qq, out.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(ctor_rq)

KIND_BEGIN(set_rq) {
    double r[3], q[4]; pose out; ok_tf t; ok_quat qq;
    rd_f64n(r, 3); rd_f64n(q, 4); rd_pose(&out);
    if (G_eof) break;
    qq.x = q[0]; qq.y = q[1]; qq.z = q[2]; qq.w = q[3];
    ok_tf_from_rq(&t, r, &qq, out.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(set_rq)

KIND_BEGIN(ctor_m4) {
    double m[16]; pose out; ok_tf t;
    rd_f64n(m, 16); rd_pose(&out);
    if (G_eof) break;
    ok_tf_from_m4(&t, m, out.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(ctor_m4)

KIND_BEGIN(set_m4) {
    double m[16]; pose out; ok_tf t;
    rd_f64n(m, 16); rd_pose(&out);
    if (G_eof) break;
    memset(&t, 0, sizeof t);
    ok_tf_set_m4(&t, m, out.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(set_m4)

KIND_BEGIN(setcoeffs) {
    double in[7]; pose out; ok_tf t;
    rd_f64n(in, 7); rd_pose(&out);
    if (G_eof) break;
    memset(&t, 0, sizeof t);
    ok_tf_set_coeffs(&t, in, out.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(setcoeffs)

KIND_BEGIN(convert) {
    double in[7]; pose out; ok_tf t;
    rd_f64n(in, 7); rd_pose(&out);
    if (G_eof) break;
    memset(&t, 0, sizeof t);
    ok_tf_convert(&t, in);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(convert)

KIND_BEGIN(inv) {
    pose in, out; ok_tf a, t;
    rd_pose(&in); rd_pose(&out);
    if (G_eof) break;
    a = to_tf(&in);
    ok_tf_inverse(&a, &t, in.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(inv)

KIND_BEGIN(mul_t) {
    pose l, r, out; ok_tf a, b, t;
    rd_pose(&l); rd_pose(&r); rd_pose(&out);
    if (G_eof) break;
    a = to_tf(&l); b = to_tf(&r);
    ok_tf_mul(&a, &b, &t, l.cache);
    bad = cmp_pose(&c, &t, &out);
} KIND_END(mul_t)

KIND_BEGIN(mul_v3) {
    pose p; double v[3], want[3], got[3]; ok_tf a;
    rd_pose(&p); rd_f64n(v, 3); rd_f64n(want, 3);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_mul_v3(&a, v, got, p.cache);
    bad = cmpv(&c, got, want, 3);
} KIND_END(mul_v3)

KIND_BEGIN(mul_v4) {
    pose p; double v[4], want[4], got[4]; ok_tf a;
    rd_pose(&p); rd_f64n(v, 4); rd_f64n(want, 4);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_mul_v4(&a, v, got, p.cache);
    bad = cmpv(&c, got, want, 4);
} KIND_END(mul_v4)

KIND_BEGIN(oplus) {
    pose in, out; double delta[6]; ok_tf a;
    rd_pose(&in); rd_f64n(delta, 6); rd_pose(&out);
    if (G_eof) break;
    a = to_tf(&in);
    ok_tf_oplus(&a, delta, in.cache);
    bad = cmp_pose(&c, &a, &out);
} KIND_END(oplus)

KIND_BEGIN(oplusj) {
    pose p; double want[42], got[42]; ok_tf a;
    rd_pose(&p); rd_f64n(want, 42);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_oplus_jacobian(&a, got);
    bad = cmpv(&c, got, want, 42);
} KIND_END(oplusj)

KIND_BEGIN(liftj) {
    pose p; double want[42], got[42]; ok_tf a;
    rd_pose(&p); rd_f64n(want, 42);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_lift_jacobian(&a, got);
    bad = cmpv(&c, got, want, 42);
} KIND_END(liftj)

KIND_BEGIN(t4) {
    pose p; double want[16], got[16]; ok_tf a;
    rd_pose(&p); rd_f64n(want, 16);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_T4(&a, got, p.cache);
    bad = cmpv(&c, got, want, 16);
} KIND_END(t4)

KIND_BEGIN(t3x4) {
    pose p; double want[12], got[12]; ok_tf a;
    rd_pose(&p); rd_f64n(want, 12);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_T3x4(&a, got, p.cache);
    bad = cmpv(&c, got, want, 12);
} KIND_END(t3x4)

KIND_BEGIN(c) {
    pose p; double want[9], got[9]; ok_tf a;
    rd_pose(&p); rd_f64n(want, 9);
    if (G_eof) break;
    a = to_tf(&p);
    ok_tf_C(&a, got, p.cache);
    bad = cmpv(&c, got, want, 9);
} KIND_END(c)

KIND_BEGIN(sinc) {
    double x, want, got;
    rd_f64n(&x, 1); rd_f64n(&want, 1);
    if (G_eof) break;
    got = ok_kin_sinc(x);
    bad = cmpv(&c, &got, &want, 1);
} KIND_END(sinc)

KIND_BEGIN(deltaq) {
    double d[3], want[4], got[4]; ok_quat q;
    rd_f64n(d, 3); rd_f64n(want, 4);
    if (G_eof) break;
    q = ok_kin_delta_q(d);
    got[0] = q.x; got[1] = q.y; got[2] = q.z; got[3] = q.w;
    bad = cmpv(&c, got, want, 4);
} KIND_END(deltaq)

KIND_BEGIN(rjac) {
    double phi[3], want[9], got[9];
    rd_f64n(phi, 3); rd_f64n(want, 9);
    if (G_eof) break;
    ok_kin_right_jacobian(phi, got);
    bad = cmpv(&c, got, want, 9);
} KIND_END(rjac)

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "kin";
    G_dir = argc > 3 ? argv[3] : ".";
    G_max = argc > 4 ? atol(argv[4]) : -1;
    check_ctor_rq(); check_ctor_m4(); check_set_m4(); check_set_rq(); check_setcoeffs(); check_convert();
    check_inv(); check_mul_t(); check_mul_v3(); check_mul_v4(); check_oplus(); check_oplusj(); check_liftj();
    check_t4(); check_t3x4(); check_c(); check_sinc(); check_deltaq(); check_rjac();
    printf("%s: %ld/%ld\n", label, G_total_bad, G_total);
    return G_total_bad == 0 ? 0 : 1;
}
