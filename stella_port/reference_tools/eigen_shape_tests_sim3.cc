// Bit-exactness self-test of stella_port/c/sv_sim3.c (g2o::Sim3: exp/log/inverse/operator*/map,
// constructor from a rotation matrix) against REAL g2o + Eigen 3.4. Random data in every branch of
// exp()/log() (sigma and theta above/below eps, d above/below 1-eps), 300k cases per kernel.
// Build (repo root):
//   mkdir -p /tmp/sim3_objs && for f in sv_sim3 sv_eigen_lu3 sv_linalg sv_eigen_quaternion; do
//     gcc -std=c99 -O2 -ffp-contract=off -fno-fast-math -c stella_port/c/$f.c -o /tmp/sim3_objs/$f.o; done
//   g++ -std=c++14 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -Iexternal/eigen \
//       -Iexternal/candidates/deps/root/usr/include stella_port/reference_tools/eigen_shape_tests_sim3.cc \
//       /tmp/sim3_objs/*.o -lm -o /tmp/eigen_shape_tests_sim3
#include <g2o/types/sim3/sim3.h>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/sv_sim3.h"
}
static std::mt19937_64 rng(12345);
static double urand(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double nrand(double sd) { return std::normal_distribution<double>(0.0, sd)(rng); }

static bool same(double a, double b) { return std::memcmp(&a, &b, 8) == 0 || (a == 0.0 && b == 0.0); }

static long n_bad = 0;
static void report(const char* what, long i, bool ok) {
    if (!ok) {
        if (n_bad < 10) std::printf("MISMATCH %s case %ld\n", what, i);
        ++n_bad;
    }
}

static void random_update(g2o::Vector7& u, int regime) {
    const double th = regime % 4 == 0 ? urand(0, 3.0) : regime % 4 == 1 ? urand(0, 2e-5) : regime % 4 == 2 ? 0.0 : urand(0, 1e-3);
    g2o::Vector3 ax(nrand(1), nrand(1), nrand(1));
    ax.normalize();
    const double sg = regime % 3 == 0 ? nrand(0.3) : regime % 3 == 1 ? nrand(3e-6) : 0.0;
    u << ax * th, nrand(1.0), nrand(1.0), nrand(1.0), sg;
    if (regime % 5 == 4) u.template segment<3>(3).setZero();
}

static g2o::Sim3 random_sim3(int regime) {
    g2o::Vector7 u;
    random_update(u, regime);
    return g2o::Sim3(u);
}

static void to_c(const g2o::Sim3& a, sv_sim3* c) {
    c->r.x = a.rotation().x();
    c->r.y = a.rotation().y();
    c->r.z = a.rotation().z();
    c->r.w = a.rotation().w();
    for (int i = 0; i < 3; ++i) c->t[i] = a.translation()(i);
    c->s = a.scale();
}

static bool eq(const g2o::Sim3& a, const sv_sim3& c) {
    return same(a.rotation().x(), c.r.x) && same(a.rotation().y(), c.r.y) && same(a.rotation().z(), c.r.z) &&
           same(a.rotation().w(), c.r.w) && same(a.translation()(0), c.t[0]) && same(a.translation()(1), c.t[1]) &&
           same(a.translation()(2), c.t[2]) && same(a.scale(), c.s);
}

int main() {
    const long N = 300000;
    for (long i = 0; i < N; ++i) { // exp
        g2o::Vector7 u;
        random_update(u, (int)i);
        double uc[7];
        for (int k = 0; k < 7; ++k) uc[k] = u(k);
        sv_sim3 c;
        sv_sim3_exp(uc, &c);
        report("exp", i, eq(g2o::Sim3(u), c));
    }
    for (long i = 0; i < N; ++i) { // log of exp-ed and of composed / normalized sim3
        g2o::Sim3 a = random_sim3((int)i);
        if (i % 3 == 1) a = a * random_sim3((int)i + 1);
        if (i % 3 == 2) a = a.inverse();
        sv_sim3 c;
        to_c(a, &c);
        const g2o::Vector7 l = a.log();
        double lc[7];
        sv_sim3_log(&c, lc);
        bool ok = true;
        for (int k = 0; k < 7; ++k) ok = ok && same(l(k), lc[k]);
        report("log", i, ok);
    }
    for (long i = 0; i < N; ++i) { // inverse, mul, map
        g2o::Sim3 a = random_sim3((int)i), b = random_sim3((int)i * 7 + 3);
        sv_sim3 ca, cb, cr;
        to_c(a, &ca);
        to_c(b, &cb);
        sv_sim3_inverse(&ca, &cr);
        report("inverse", i, eq(a.inverse(), cr));
        sv_sim3_mul(&ca, &cb, &cr);
        report("mul", i, eq(a * b, cr));
        g2o::Vector3 p(nrand(3), nrand(3), nrand(3));
        double pc[3] = {p(0), p(1), p(2)}, oc[3];
        sv_sim3_map(&ca, pc, oc);
        const g2o::Vector3 q = a.map(p);
        report("map", i, same(q(0), oc[0]) && same(q(1), oc[1]) && same(q(2), oc[2]));
    }
    for (long i = 0; i < N; ++i) { // constructor from a (not exactly orthogonal) rotation matrix
        g2o::Sim3 a = random_sim3((int)i);
        g2o::Matrix3 R = a.rotation().toRotationMatrix();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) R(r, c) += (i % 2) ? nrand(1e-9) : 0.0;
        g2o::Vector3 t(nrand(1), nrand(1), nrand(1));
        const double s = i % 3 == 0 ? 1.0 : std::exp(nrand(0.3));
        g2o::Sim3 ref(R, t, s);
        double Rc[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Rc[c * 3 + r] = R(r, c);
        double tc[3] = {t(0), t(1), t(2)};
        sv_sim3 c;
        sv_sim3_from_rot(Rc, tc, s, &c);
        report("from_rot", i, eq(ref, c));
    }
    std::printf("eigen_shape_tests_sim3: %ld/%ld mismatches\n", n_bad, 5 * N);
    return n_bad ? 1 : 0;
}
