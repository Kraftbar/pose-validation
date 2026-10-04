// OK_PORT_TEST_SRC: okvis_time/src/Time.cpp okvis_time/src/Duration.cpp okvis_cv/src/NCameraSystem.cpp okvis_cv/src/CameraBase.cpp
// OK_PORT_TEST_C: ok_time.c ok_kin.c ok_cam.c ok_eigen.c
// Random-case, tolerance-0 comparison of the C modules ok_time / ok_kin / ok_cam against the *real* OKVIS2 classes
// (okvis_time, okvis_kinematics Transformation / operators, okvis_cv PinholeCamera<{RadialTangential, Equidistant,
// NoDistortion}> and NCameraSystem::computeOverlaps), compiled with the reference flags (-O2 -DNDEBUG
// -ffp-contract=off -fno-fast-math, Eigen 3.4.0, SSE2). Covers every variant of the API, including those the
// EuRoC mono/stereo pipelines never call, and edge values (+-0, tiny, huge, singular points).
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <okvis/Time.hpp>
#include <okvis/Duration.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/NoDistortion.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
extern "C" {
#include "../c/ok_time.h"
#include "../c/ok_kin.h"
#include "../c/ok_cam.h"
}

using namespace okvis;
static std::mt19937_64 rng(20261002);
static double U() { return std::uniform_real_distribution<double>(-1.0, 1.0)(rng); }
static double N() { return std::normal_distribution<double>(0.0, 1.0)(rng); }
// value generator with special values
static double V(double scale = 1.0) {
  switch (rng() % 16) {
    case 0: return 0.0;
    case 1: return -0.0;
    case 2: return scale * 1e-9 * N();
    case 3: return scale * 1e-4 * N();
    case 4: return scale * 100.0 * N();
    default: return scale * N();
  }
}

struct Sec { std::string name; long bad = 0, tot = 0; };
static std::vector<Sec> g_secs;
static Sec& sec(const char* n) {
  for (auto& s : g_secs) if (s.name == n) return s;
  g_secs.push_back(Sec{std::string(n)}); return g_secs.back();
}
static int cmp(const char* n, const double* a, const double* b, int k) {
  Sec& s = sec(n); int bad = 0;
  for (int i = 0; i < k; ++i) { s.tot++; if (std::memcmp(a + i, b + i, 8)) { s.bad++; bad++; } }
  return bad;
}
static int cmpi(const char* n, long a, long b) { Sec& s = sec(n); s.tot++; if (a != b) { s.bad++; return 1; } return 0; }

// ------------------------------------------------------------------------------------------- time
static void test_time() {
  for (int it = 0; it < 200000; ++it) {
    uint32_t s1 = rng() % 3 == 0 ? rng() : rng() % 2000000, n1 = rng() % 4 == 0 ? rng() : rng() % 1100000000u;
    uint32_t s2 = rng() % 2000000, n2 = rng() % 1100000000u;
    int32_t ds = int32_t(rng() % 4000) - 2000, dn = rng() % 3 == 0 ? int32_t(rng()) % 2000000000 : int32_t(rng() % 3000000000u) - 1500000000;
    okvis::Time a(s1, n1), b(s2, n2);
    ok_time ca = ok_time_make(s1, n1), cb = ok_time_make(s2, n2);
    cmpi("time.make", (long)a.sec * 4294967296L + a.nsec, (long)ca.sec * 4294967296L + ca.nsec);
    double ts = std::floor(U() * 1e6 + 1e6) + (rng() % 2 ? 0 : N() * 1e-3);
    if (rng() % 7 == 0) ts = double(rng() % 100000) * 0.5;
    if (rng() % 11 == 0) ts = double(rng() % 100000) + 0.9999999999;
    { okvis::Time t(ts); ok_time c = ok_time_from_sec(ts);
      cmpi("time.fromSec", (long)t.sec * 4294967296L + t.nsec, (long)c.sec * 4294967296L + c.nsec); }
    { double x = a.toSec(), y = ok_time_to_sec(ca); cmp("time.toSec", &x, &y, 1); }
    { uint64_t v = rng() >> (20 + rng() % 40);
      okvis::Time t; bool thrown = false; try { t.fromNSec(v); } catch (...) { thrown = true; }
      ok_time c = ok_time_from_nsec(v);
      if (!thrown) cmpi("time.fromNSec", (long)t.sec * 4294967296L + t.nsec, (long)c.sec * 4294967296L + c.nsec);
      cmpi("time.toNSec", (long)a.toNSec(), (long)ok_time_to_nsec(ca)); }
    { okvis::Duration d = a - b; ok_duration cd = ok_time_sub(ca, cb);
      cmpi("time.sub", (long)d.sec * 4294967296L + d.nsec, (long)cd.sec * 4294967296L + cd.nsec);
      double x = d.toSec(), y = ok_time_diff_sec(ca, cb); cmp("time.diffSec", &x, &y, 1); }
    cmpi("time.lt", a < b, ok_time_lt(ca, cb)); cmpi("time.gt", a > b, ok_time_gt(ca, cb));
    cmpi("time.le", a <= b, ok_time_le(ca, cb)); cmpi("time.ge", a >= b, ok_time_ge(ca, cb));
    cmpi("time.eq", a == b, ok_time_eq(ca, cb)); cmpi("time.eq", a == a, ok_time_eq(ca, ca));
    { okvis::Duration d(ds, dn); ok_duration cd = ok_duration_make(ds, dn);
      cmpi("dur.make", (long)d.sec * 4294967296L + d.nsec, (long)cd.sec * 4294967296L + cd.nsec);
      bool thrown = false; okvis::Time r;
      try { r = a + d; } catch (...) { thrown = true; }
      ok_time cr; int rc = ok_time_add(ca, cd, &cr);
      cmpi("time.add.throws", thrown, rc != 0);
      if (!thrown && rc == 0) cmpi("time.add", (long)r.sec * 4294967296L + r.nsec, (long)cr.sec * 4294967296L + cr.nsec);
      thrown = false;
      try { r = a - d; } catch (...) { thrown = true; }
      rc = ok_time_sub_duration(ca, cd, &cr);
      cmpi("time.subDur.throws", thrown, rc != 0);
      if (!thrown && rc == 0) cmpi("time.subDur", (long)r.sec * 4294967296L + r.nsec, (long)cr.sec * 4294967296L + cr.nsec);
      double x = d.toSec(), y = ok_duration_to_sec(cd); cmp("dur.toSec", &x, &y, 1);
      okvis::Duration e(int32_t(rng() % 100) - 50, int32_t(rng() % 1500000000u) - 200000000); ok_duration ce = ok_duration_make(e.sec, e.nsec);
      okvis::Duration s = d + e; ok_duration cs = ok_duration_add(cd, ce);
      cmpi("dur.add", (long)s.sec * 4294967296L + s.nsec, (long)cs.sec * 4294967296L + cs.nsec);
      s = d - e; cs = ok_duration_sub(cd, ce);
      cmpi("dur.sub", (long)s.sec * 4294967296L + s.nsec, (long)cs.sec * 4294967296L + cs.nsec);
      s = -d; cs = ok_duration_neg(cd);
      cmpi("dur.neg", (long)s.sec * 4294967296L + s.nsec, (long)cs.sec * 4294967296L + cs.nsec);
      double sc = N() * 3; s = d * sc; cs = ok_duration_mul(cd, sc);
      cmpi("dur.mul", (long)s.sec * 4294967296L + s.nsec, (long)cs.sec * 4294967296L + cs.nsec);
      cmpi("dur.lt", d < e, ok_duration_lt(cd, ce)); cmpi("dur.ge", d >= e, ok_duration_ge(cd, ce));
      cmpi("dur.le", d <= e, ok_duration_le(cd, ce)); cmpi("dur.gt", d > e, ok_duration_gt(cd, ce));
    }
    { double t = (rng() % 5 == 0) ? double(int(U() * 50)) : U() * 100.0;   // includes negative whole numbers
      okvis::Duration d(t); ok_duration cd = ok_duration_from_sec(t);
      cmpi("dur.fromSec", (long)d.sec * 4294967296L + d.nsec, (long)cd.sec * 4294967296L + cd.nsec);
      int64_t nv = int64_t(rng() >> 20) * (rng() % 2 ? 1 : -1);
      okvis::Duration d2; bool thrown = false; try { d2.fromNSec(nv); } catch (...) { thrown = true; }
      ok_duration c2 = ok_duration_from_nsec(nv);
      if (!thrown) cmpi("dur.fromNSec", (long)d2.sec * 4294967296L + d2.nsec, (long)c2.sec * 4294967296L + c2.nsec);
      cmpi("dur.toNSec", (long)d.toNSec(), (long)ok_duration_to_nsec(cd)); }
  }
}

// ------------------------------------------------------------------------------------------- kinematics
static void coeffs_of(const kinematics::Transformation& t, double c[7]) { std::memcpy(c, t.coeffs().data(), 56); }
static void coeffs_of(const kinematics::TransformationCacheless& t, double c[7]) { std::memcpy(c, t.coeffs().data(), 56); }
static void tf_c(const ok_tf& t, double c[7]) {
  c[0] = t.r[0]; c[1] = t.r[1]; c[2] = t.r[2]; c[3] = t.q.x; c[4] = t.q.y; c[5] = t.q.z; c[6] = t.q.w;
}
static ok_tf from_cpp(const kinematics::Transformation& t) {
  ok_tf o; double c[7]; coeffs_of(t, c);
  o.r[0] = c[0]; o.r[1] = c[1]; o.r[2] = c[2]; o.q.x = c[3]; o.q.y = c[4]; o.q.z = c[5]; o.q.w = c[6];
  Eigen::Matrix3d C = t.C(); std::memcpy(o.C, C.data(), 72); return o;
}
static ok_tf from_cpp(const kinematics::TransformationCacheless& t) {
  ok_tf o; double c[7]; coeffs_of(t, c);
  o.r[0] = c[0]; o.r[1] = c[1]; o.r[2] = c[2]; o.q.x = c[3]; o.q.y = c[4]; o.q.z = c[5]; o.q.w = c[6];
  std::memset(o.C, 0, 72); return o;
}
template <class T> static void cmp_tf(const char* n, const T& cpp, const ok_tf& c, bool cached) {
  double a[7], b[7]; coeffs_of(cpp, a); tf_c(c, b); cmp(n, a, b, 7);
  if (cached) { Eigen::Matrix3d C = cpp.C(); cmp(n, C.data(), c.C, 9); }
}
static Eigen::Quaterniond rq() {
  switch (rng() % 8) {
    case 0: return Eigen::Quaterniond(1, 0, 0, 0);
    case 1: return Eigen::Quaterniond(-0.0, 0.0, -0.0, 1.0);
    case 2: return Eigen::Quaterniond(V(), V(), V(), V());
    default: return Eigen::Quaterniond(N(), N(), N(), N());
  }
}
static Eigen::Vector3d rv(double s = 1.0) { return Eigen::Vector3d(V(s), V(s), V(s)); }

template <bool CACHE>
static void test_tf() {
  typedef kinematics::TransformationT<CACHE> T;
  const char* sfx = CACHE ? "" : ".cl";
  auto nm = [&](const char* b) { static char buf[8][64]; static int k = 0; k = (k + 1) % 8; std::snprintf(buf[k], 64, "tf%s.%s", sfx, b); return (const char*)buf[k]; };
  for (int it = 0; it < 60000; ++it) {
    Eigen::Vector3d r = rv(3.0); Eigen::Quaterniond q = rq();
    T a(r, q);
    ok_tf ca; ok_quat cq = {q.x(), q.y(), q.z(), q.w()};
    ok_tf_from_rq(&ca, r.data(), &cq, CACHE);
    cmp_tf(nm("ctor_rq"), a, ca, CACHE);
    // setters
    { T b; b.set(r, q); ok_tf cb; ok_tf_from_rq(&cb, r.data(), &cq, CACHE); cmp_tf(nm("set_rq"), b, cb, CACHE); }
    // Matrix4d constructors / set
    { Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d R;
      switch (rng() % 4) { case 0: R = q.normalized().toRotationMatrix(); break;
        case 1: R = rq().normalized().toRotationMatrix(); break;
        case 2: R = Eigen::Matrix3d::Identity() * -1.0; R(0, 0) = 1.0; break;   // trace <= 0 branches
        default: R << V(), V(), V(), V(), V(), V(), V(), V(), V(); }
      if (rng() % 3 == 0) { Eigen::Quaterniond qq(N(), N(), N(), N()); qq.normalize(); R = qq.toRotationMatrix(); R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)).toRotationMatrix() * R; }
      M.topLeftCorner<3, 3>() = R; M.topRightCorner<3, 1>() = r;
      T b(M); ok_tf cb; ok_tf_from_m4(&cb, M.data(), CACHE); cmp_tf(nm("ctor_m4"), b, cb, CACHE);
      T s; s.set(M); ok_tf cs; ok_tf_set_m4(&cs, M.data(), CACHE); cmp_tf(nm("set_m4"), s, cs, CACHE);
      double Tm[16]; Eigen::Matrix4d TT = a.T(); ok_tf_T4(&ca, Tm, CACHE); cmp(nm("T4"), TT.data(), Tm, 16);
      Eigen::Matrix<double, 3, 4> T34 = a.T3x4(); double t34[12]; ok_tf_T3x4(&ca, t34, CACHE); cmp(nm("T3x4"), T34.data(), t34, 12);
      Eigen::Matrix3d Cm = a.C(); double cc[9]; ok_tf_C(&ca, cc, CACHE); cmp(nm("C"), Cm.data(), cc, 9); }
    // setCoeffs (unnormalised coefficients, as written by the solver)
    { Eigen::Matrix<double, 7, 1> co; co << rv(2.0), rq().coeffs(); T b; b.setCoeffs(co); ok_tf cb; memset(&cb, 0, sizeof cb);
      ok_tf_set_coeffs(&cb, co.data(), CACHE); cmp_tf(nm("setCoeffs"), b, cb, CACHE); }
    // inverse, product, vector products
    { T inv = a.inverse(); ok_tf ci; ok_tf_inverse(&ca, &ci, CACHE); cmp_tf(nm("inverse"), inv, ci, CACHE); }
    { Eigen::Vector3d r2 = rv(2.0); T b(r2, rq()); ok_tf cb = from_cpp(b), cr;
      T pr = a * b; ok_tf_mul(&ca, &cb, &cr, CACHE); cmp_tf(nm("mul_t"), pr, cr, CACHE); }
    { Eigen::Vector3d v = rv(2.0); Eigen::Vector3d w = a * v; double o[3]; ok_tf_mul_v3(&ca, v.data(), o, CACHE); cmp(nm("mul_v3"), w.data(), o, 3); }
    { Eigen::Vector4d v(V(), V(), V(), rng() % 3 ? 1.0 : V()); Eigen::Vector4d w = a * v; double o[4]; ok_tf_mul_v4(&ca, v.data(), o, CACHE); cmp(nm("mul_v4"), w.data(), o, 4); }
    // oplus: random small and large deltas (including exactly zero rotation)
    { Eigen::Matrix<double, 6, 1> d; d << rv(0.1), (rng() % 6 == 0 ? Eigen::Vector3d::Zero() : rv(rng() % 2 ? 1e-3 : 0.5));
      if (rng() % 9 == 0) d.tail<3>() = Eigen::Vector3d(1e-8 * N(), -0.0, 1e-9);
      T b(a); b.oplus(d); ok_tf cb = ca; ok_tf_oplus(&cb, d.data(), CACHE); cmp_tf(nm("oplus"), b, cb, CACHE);
      Eigen::Matrix<double, 7, 6> J; T b2(a); b2.oplus(d, J); double Jc[42]; ok_tf_oplus_jacobian(&cb, Jc); cmp(nm("oplusJacobian"), J.data(), Jc, 42);
      Eigen::Matrix<double, 6, 7> L; b2.liftJacobian(L); ok_tf_lift_jacobian(&cb, Jc); cmp(nm("liftJacobian"), L.data(), Jc, 42);
      Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Jr; b2.oplusJacobian(Jr); // same values, RowMajor storage
      Eigen::Matrix<double, 7, 6> Jr2 = Jr; ok_tf_oplus_jacobian(&cb, Jc); cmp(nm("oplusJacobian.rm"), Jr2.data(), Jc, 42); }
    // helpers
    { Eigen::Vector3d d = rv(rng() % 2 ? 1e-5 : 1.0); if (rng() % 8 == 0) d = Eigen::Vector3d(1e-7, -1e-7, 0.0);
      double x = kinematics::sinc(V(3.0)); (void)x;
      double xs = V(rng() % 2 ? 1e-6 : 3.0); double s1 = kinematics::sinc(xs), s2 = ok_kin_sinc(xs); cmp("sinc", &s1, &s2, 1);
      Eigen::Quaterniond dq = kinematics::deltaQ(d); ok_quat cqq = ok_kin_delta_q(d.data());
      double e[4] = {dq.x(), dq.y(), dq.z(), dq.w()}, g[4] = {cqq.x, cqq.y, cqq.z, cqq.w}; cmp("deltaQ", e, g, 4);
      Eigen::Matrix3d J = kinematics::rightJacobian(d); double g9[9]; ok_kin_right_jacobian(d.data(), g9); cmp("rightJacobian", J.data(), g9, 9);
      Eigen::Matrix3d X = kinematics::crossMx(d); ok_kin_cross_mx(d.data(), g9); cmp("crossMx", X.data(), g9, 9);
      Eigen::Quaterniond pq = rq(); ok_quat cp = {pq.x(), pq.y(), pq.z(), pq.w()};
      Eigen::Matrix4d P = kinematics::plus(pq), O = kinematics::oplus(pq); double g16[16];
      ok_kin_plus(&cp, g16); cmp("plus", P.data(), g16, 16); ok_kin_oplus(&cp, g16); cmp("oplus.op", O.data(), g16, 16); }
    // cross-cache conversion
    { kinematics::TransformationT<!CACHE> o(r, q); T c1(o); ok_tf cb; double co[7]; coeffs_of(o, co);
      if (CACHE) { memset(&cb, 0, sizeof cb); ok_tf_convert(&cb, co); cmp_tf(nm("convert"), c1, cb, CACHE); } }
  }
}

// ------------------------------------------------------------------------------------------- cameras
struct CamPair { std::shared_ptr<cameras::CameraBase> cpp; ok_cam c; int dist; int nd; };

template <class D>
static CamPair make_cam(int dist, int nd, bool eurocLike) {
  int w = 752, h = 480;
  double fu = eurocLike ? 458.654880721 + N() : 300 + 400 * std::fabs(U()), fv = eurocLike ? 457.296696463 + N() : 300 + 400 * std::fabs(U());
  double cu = eurocLike ? 367.215803962 : 300 + 150 * U(), cv = eurocLike ? 248.37534061 : 240 + 100 * U();
  double d[4] = {0, 0, 0, 0};
  if (dist == OK_CAM_RADTAN) { d[0] = -0.28 + 0.05 * N(); d[1] = 0.07 + 0.02 * N(); d[2] = 2e-4 * N(); d[3] = 2e-5 * N(); }
  else if (dist == OK_CAM_EQUIDISTANT) { d[0] = 0.03 * N(); d[1] = 0.01 * N(); d[2] = 0.003 * N(); d[3] = 0.001 * N(); }
  CamPair p; p.dist = dist; p.nd = nd;
  if (dist == OK_CAM_RADTAN) p.cpp = std::make_shared<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(w, h, fu, fv, cu, cv, cameras::RadialTangentialDistortion(d[0], d[1], d[2], d[3]));
  else if (dist == OK_CAM_EQUIDISTANT) p.cpp = std::make_shared<cameras::PinholeCamera<cameras::EquidistantDistortion>>(w, h, fu, fv, cu, cv, cameras::EquidistantDistortion(d[0], d[1], d[2], d[3]));
  else p.cpp = std::make_shared<cameras::PinholeCamera<cameras::NoDistortion>>(w, h, fu, fv, cu, cv, cameras::NoDistortion());
  ok_cam_init(&p.c, dist, w, h, fu, fv, cu, cv, d);
  return p;
}

static Eigen::Vector3d rpoint() {
  Eigen::Vector3d p(V(2.0), V(2.0), V(3.0) + 3.0 * (rng() % 4 != 0));
  switch (rng() % 12) {
    case 0: p[2] = 0.0; break;
    case 1: p[2] = 1e-13 * N(); break;
    case 2: p[0] = 0.0; p[1] = 0.0; break;
    case 3: p[2] = -std::fabs(p[2]); break;
    case 4: p = Eigen::Vector3d(0.3 * N(), 0.2 * N(), 1.0); break;
    default: break;
  }
  return p;
}

static void test_cam(int dist, const char* tag) {
  auto nm = [&](const char* b) { static char buf[8][64]; static int k = 0; k = (k + 1) % 8; std::snprintf(buf[k], 64, "cam.%s.%s", tag, b); return (const char*)buf[k]; };
  for (int it = 0; it < 30000; ++it) {
    CamPair cp = make_cam<cameras::NoDistortion>(dist, 0, it % 3 == 0);
    // rebuilding with the right class happens inside make_cam via dist; NoDistortion template arg above is unused
    cameras::CameraBase& cam = *cp.cpp;
    const ok_cam& cc = cp.c;
    const int ni = ok_cam_num_intrinsics(&cc);
    Eigen::Vector3d p = rpoint();
    Eigen::Vector4d ph(p[0], p[1], p[2], rng() % 4 ? 1.0 : (rng() % 2 ? -1.0 : V()));
    // project (2 args)
    { Eigen::Vector2d ip(-7.0, -7.0); double ic[2] = {-7.0, -7.0};
      cameras::ProjectionStatus st = cam.project(p, &ip); ok_proj_status cs = ok_cam_project(&cc, p.data(), ic);
      cmpi(nm("project.status"), (int)st, (int)cs); cmp(nm("project.img"), ip.data(), ic, 2);   // untouched outputs compare too
    }
    // project with Jacobians
    { Eigen::Vector2d ip(-7.0, -7.0); Eigen::Matrix<double, 2, 3> J; J.setConstant(-7.0); Eigen::Matrix2Xd Ji;
      double ic[2] = {-7.0, -7.0}, Jc[6] = {-7, -7, -7, -7, -7, -7}; std::vector<double> Jic(2 * ni + 1, -7.0);
      bool wantI = rng() % 2;
      cameras::ProjectionStatus st = cam.project(p, &ip, &J, wantI ? &Ji : nullptr);
      ok_proj_status cs = ok_cam_project_j(&cc, p.data(), ic, Jc, wantI ? Jic.data() : nullptr);
      cmpi(nm("projectJ.status"), (int)st, (int)cs);
      if (std::fabs(p[2]) >= 1e-12) { cmp(nm("projectJ.img"), ip.data(), ic, 2); cmp(nm("projectJ.J"), J.data(), Jc, 6);
        if (wantI) { cmpi(nm("projectJ.cols"), Ji.cols(), ni); cmp(nm("projectJ.Ji"), Ji.data(), Jic.data(), 2 * ni); } }
    }
    // with external parameters
    { Eigen::VectorXd par; cam.getIntrinsics(par); for (int i = 0; i < par.size(); ++i) par[i] *= 1.0 + 0.01 * N();
      Eigen::Vector2d ip(-7.0, -7.0); Eigen::Matrix<double, 2, 3> J; J.setConstant(-7.0); Eigen::Matrix2Xd Ji;
      double ic[2] = {-7.0, -7.0}, Jc[6] = {-7, -7, -7, -7, -7, -7}; std::vector<double> Jic(2 * ni + 1, -7.0);
      bool wantI = rng() % 2, wantJ = rng() % 4 != 0;
      cameras::ProjectionStatus st = cam.projectWithExternalParameters(p, par, &ip, wantJ ? &J : nullptr, wantI ? &Ji : nullptr);
      ok_proj_status cs = ok_cam_project_ext(&cc, p.data(), par.data(), ic, wantJ ? Jc : nullptr, wantI ? Jic.data() : nullptr);
      cmpi(nm("projectX.status"), (int)st, (int)cs);
      if (std::fabs(p[2]) >= 1e-12) { cmp(nm("projectX.img"), ip.data(), ic, 2); if (wantJ) cmp(nm("projectX.J"), J.data(), Jc, 6);
        if (wantI) cmp(nm("projectX.Ji"), Ji.data(), Jic.data(), 2 * ni); }
    }
    // homogeneous
    { Eigen::Vector2d ip(-7.0, -7.0); double ic[2] = {-7.0, -7.0};
      cameras::ProjectionStatus st = cam.projectHomogeneous(ph, &ip); ok_proj_status cs = ok_cam_project_h(&cc, ph.data(), ic);
      cmpi(nm("projectH.status"), (int)st, (int)cs); cmp(nm("projectH.img"), ip.data(), ic, 2); }
    if (std::fabs(ph[2]) >= 1e-12) {
      Eigen::Vector2d ip; Eigen::Matrix<double, 2, 4> J; Eigen::Matrix2Xd Ji; double ic[2], Jc[8]; std::vector<double> Jic(2 * ni + 1);
      bool wantI = rng() % 2;
      cameras::ProjectionStatus st = cam.projectHomogeneous(ph, &ip, &J, wantI ? &Ji : nullptr);
      ok_proj_status cs = ok_cam_project_h_j(&cc, ph.data(), ic, Jc, wantI ? Jic.data() : nullptr);
      cmpi(nm("projectHJ.status"), (int)st, (int)cs); cmp(nm("projectHJ.img"), ip.data(), ic, 2); cmp(nm("projectHJ.J"), J.data(), Jc, 8);
      if (wantI) cmp(nm("projectHJ.Ji"), Ji.data(), Jic.data(), 2 * ni);
      Eigen::VectorXd par; cam.getIntrinsics(par); for (int i = 0; i < par.size(); ++i) par[i] *= 1.0 + 0.01 * N();
      Eigen::Vector2d ip2; Eigen::Matrix<double, 2, 4> J2; Eigen::Matrix2Xd Ji2; double ic2[2], Jc2[8]; std::vector<double> Jic2(2 * ni + 1);
      st = cam.projectHomogeneousWithExternalParameters(ph, par, &ip2, &J2, wantI ? &Ji2 : nullptr);
      cs = ok_cam_project_h_ext(&cc, ph.data(), par.data(), ic2, Jc2, wantI ? Jic2.data() : nullptr);
      cmpi(nm("projectHX.status"), (int)st, (int)cs); cmp(nm("projectHX.img"), ip2.data(), ic2, 2); cmp(nm("projectHX.J"), J2.data(), Jc2, 8);
      if (wantI) cmp(nm("projectHX.Ji"), Ji2.data(), Jic2.data(), 2 * ni);
    }
    // back-projection (pixel in and slightly outside the image; exactly at the principal point sometimes)
    { Eigen::Vector2d ip(U() * 400 + 376, U() * 260 + 240);
      if (rng() % 10 == 0) ip = Eigen::Vector2d(cc.cu, cc.cv);
      if (rng() % 10 == 0) ip = Eigen::Vector2d(std::floor(ip[0]), std::floor(ip[1]));
      Eigen::Vector3d d; double dc[3];
      bool ok = cam.backProject(ip, &d); int okc = ok_cam_back_project(&cc, ip.data(), dc);
      cmpi(nm("back.ok"), ok, okc); cmp(nm("back.dir"), d.data(), dc, 3);
      Eigen::Matrix<double, 3, 2> J; double Jc[6]; Eigen::Vector3d d2; double dc2[3];
      ok = cam.backProject(ip, &d2, &J); okc = ok_cam_back_project_j(&cc, ip.data(), dc2, Jc);
      cmpi(nm("backJ.ok"), ok, okc); cmp(nm("backJ.dir"), d2.data(), dc2, 3); cmp(nm("backJ.J"), J.data(), Jc, 6);
      Eigen::Vector4d h; double hc[4]; ok = cam.backProjectHomogeneous(ip, &h); okc = ok_cam_back_project_h(&cc, ip.data(), hc);
      cmpi(nm("backH.ok"), ok, okc); cmp(nm("backH.dir"), h.data(), hc, 4);
      Eigen::Matrix<double, 4, 2> JH; double JHc[8]; ok = cam.backProjectHomogeneous(ip, &h, &JH); okc = ok_cam_back_project_h_j(&cc, ip.data(), hc, JHc);
      cmpi(nm("backHJ.ok"), ok, okc); cmp(nm("backHJ.dir"), h.data(), hc, 4); cmp(nm("backHJ.J"), JH.data(), JHc, 8); }
  }
}

// ------------------------------------------------------------------------------------------- NCameraSystem
static void test_overlaps(int dist0, int dist1, const char* tag) {
  auto mk = [&](int dist, bool second) {
    CamPair p = make_cam<cameras::NoDistortion>(dist, 0, true); (void)second; return p; };
  CamPair a = mk(dist0, false), b = mk(dist1, true);
  Eigen::Vector3d r0(-0.02, -0.06, 0.01), r1(-0.02, 0.045, 0.008);
  kinematics::Transformation T0(r0, Eigen::Quaterniond(0.7, 0.1, -0.5, 0.2)), T1(r1, Eigen::Quaterniond(0.69, 0.12, -0.45, 0.25));
  cameras::NCameraSystem ncs;
  ncs.addCamera(std::make_shared<kinematics::Transformation>(T0), a.cpp, cameras::NCameraSystem::RadialTangential, true);
  ncs.addCamera(std::make_shared<kinematics::Transformation>(T1), b.cpp, cameras::NCameraSystem::RadialTangential, true);
  ok_ncam s; ok_ncam_init(&s);
  ok_tf t0 = from_cpp(T0), t1 = from_cpp(T1);
  ok_ncam_add(&s, &a.c, &t0); ok_ncam_add(&s, &b.c, &t1);
  ok_ncam_compute_overlaps(&s);
  char nmb[64];
  for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
    const cv::Mat m = ncs.overlap(i, j);
    std::snprintf(nmb, sizeof nmb, "overlap.%s", tag);
    long bad = 0;
    for (int v = 0; v < m.rows; ++v) for (int u = 0; u < m.cols; ++u) bad += (m.at<uchar>(v, u) != s.mask[i][j][(size_t)v * m.cols + u]);
    Sec& sc = sec("overlap.mask"); sc.tot += (long)m.rows * m.cols; sc.bad += bad;
    cmpi("overlap.flag", ncs.hasOverlap(i, j), s.overlaps[i][j]);
  }
  ok_ncam_free(&s);
}

int main() {
  test_time();
  test_tf<true>();
  test_tf<false>();
  test_cam(OK_CAM_RADTAN, "radtan");
  test_cam(OK_CAM_EQUIDISTANT, "equi");
  test_cam(OK_CAM_NODIST, "nodist");
  test_overlaps(OK_CAM_RADTAN, OK_CAM_RADTAN, "rr");
  test_overlaps(OK_CAM_EQUIDISTANT, OK_CAM_EQUIDISTANT, "ee");
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("  %-26s %ld/%ld\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("okvis_kin_cam_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
