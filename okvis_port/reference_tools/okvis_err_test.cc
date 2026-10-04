// OK_PORT_TEST_SRC: okvis_time/src/Time.cpp okvis_time/src/Duration.cpp okvis_cv/src/CameraBase.cpp okvis_ceres/src/PoseError.cpp okvis_ceres/src/SpeedAndBiasError.cpp okvis_ceres/src/RelativePoseError.cpp okvis_ceres/src/HomogeneousPointError.cpp okvis_ceres/src/PoseLocalParameterization.cpp okvis_ceres/src/HomogeneousPointLocalParameterization.cpp okvis_ceres/src/PoseParameterBlock.cpp okvis_ceres/src/SpeedAndBiasParameterBlock.cpp okvis_ceres/src/HomogeneousPointParameterBlock.cpp
// OK_PORT_TEST_C: ok_time.c ok_kin.c ok_cam.c ok_eigen.c ok_param.c ok_err.c
// OK_PORT_TEST_LIBS: ceres glog
// Random-case, tolerance-0 comparison of the C modules ok_param / ok_err against the *real* OKVIS2 okvis_ceres classes
// (PoseManifold, HomogeneousPointManifold, Pose/SpeedAndBias/HomogeneousPoint parameter blocks, ReprojectionError over
// PinholeCamera<{RadialTangential, Equidistant, NoDistortion}>, PoseError, SpeedAndBiasError, RelativePoseError,
// HomogeneousPointError incl. all constructors / setInformation (LLT) and every Jacobian-pointer combination), compiled
// with the reference flags (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0, SSE2). Covers the terms and
// variants the EuRoC mono/stereo pipelines never construct (HomogeneousPointError, NoDistortion, partial Jacobian
// requests, minimal Jacobians without full ones, failing LLT) and edge values (+-0, tiny, huge, unnormalised and
// zero quaternions). Unwritten Jacobian buffers are pre-filled with a sentinel and compared as well.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <okvis/Time.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/NoDistortion.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
extern "C" {
#include "../c/ok_kin.h"
#include "../c/ok_cam.h"
#include "../c/ok_param.h"
#include "../c/ok_err.h"
}

using namespace okvis;
namespace oc = okvis::ceres;  // (plain `ceres` would be ambiguous with ::ceres)
static std::mt19937_64 rng(20261003);
static double U() { return std::uniform_real_distribution<double>(-1.0, 1.0)(rng); }
static double N() { return std::normal_distribution<double>(0.0, 1.0)(rng); }
static double V(double scale = 1.0) {  // value generator with special values
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
static Sec& sec(const std::string& n) {
  for (auto& s : g_secs) if (s.name == n) return s;
  g_secs.push_back(Sec{n}); return g_secs.back();
}
static int cmp(const std::string& n, const double* a, const double* b, int k) {
  Sec& s = sec(n); int bad = 0;
  for (int i = 0; i < k; ++i) { s.tot++; if (std::memcmp(a + i, b + i, 8)) { s.bad++; bad++; } }
  return bad;
}
static int cmpi(const std::string& n, long a, long b) { Sec& s = sec(n); s.tot++; if (a != b) { s.bad++; return 1; } return 0; }

// ------------------------------------------------------------------------------------------- helpers
static Eigen::Quaterniond rq() {
  Eigen::Quaterniond q(V(), V(), V(), V());
  if (rng() % 3 != 0) q = Eigen::Quaterniond(N(), N(), N(), N());
  if (rng() % 6 == 0) { q.coeffs() *= 0.3 + std::fabs(U()); }
  if (q.coeffs().squaredNorm() == 0.0) q = Eigen::Quaterniond::Identity();
  return q;
}
static Eigen::Quaterniond rqn() { Eigen::Quaterniond q(N(), N(), N(), N()); q.normalize(); if (rng() % 8 == 0) q = Eigen::Quaterniond(1, 0, 0, 0); return q; }
static void pose_params(double p[7]) {
  Eigen::Quaterniond q = rq();
  p[0] = V(2); p[1] = V(2); p[2] = V(2); p[3] = q.x(); p[4] = q.y(); p[5] = q.z(); p[6] = q.w();
}
static void coeffs_of(const kinematics::Transformation& t, double c[7]) { std::memcpy(c, t.coeffs().data(), 56); }
static ok_tf tf_c(const kinematics::Transformation& t) {
  ok_tf o; double c[7]; coeffs_of(t, c);
  ok_tf_set_coeffs(&o, c, 1);  // cached: C from q (the Transformation was built from (r, q))
  Eigen::Matrix3d C = t.C(); std::memcpy(o.C, C.data(), 72);
  return o;
}
static kinematics::Transformation rT() {
  return kinematics::Transformation(Eigen::Vector3d(V(2), V(2), V(2)), rq());  // ctor normalises
}
// random SPD / special information matrices of size n (column-major, symmetric)
static Eigen::MatrixXd rinfo(int n) {
  Eigen::MatrixXd I(n, n);
  switch (rng() % 8) {
    case 0: case 1: {  // diagonal like the pipeline priors
      I.setZero(); for (int i = 0; i < n; ++i) I(i, i) = std::pow(10.0, 4.0 * U()); break; }
    case 2: { I.setZero(); for (int i = 0; i < n; ++i) I(i, i) = std::fabs(V(10.0)) + 1e-3; break; }
    case 3: {  // not positive definite (LLT fails somewhere)
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, n); for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) B(i, j) = V();
      I = B + B.transpose(); break; }
    default: {
      Eigen::MatrixXd B(n, n); for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) B(i, j) = N();
      I = B.transpose() * B + std::pow(10.0, 2.0 * U()) * Eigen::MatrixXd::Identity(n, n);
      if (rng() % 3 == 0) for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) if (rng() % 3 == 0) I(i, j) = I(j, i) = 0.0;
      break; }
  }
  return I;
}
template <int N_> static Eigen::Matrix<double, N_, N_> fixed(const Eigen::MatrixXd& m) { return Eigen::Matrix<double, N_, N_>(m); }

// pointer-configuration helper for Evaluate
struct Cfg { bool haveJac, haveMin; bool jn[3], mn[3]; };
static Cfg rcfg(int nb) {
  Cfg c; c.haveJac = rng() % 6 != 0; c.haveMin = rng() % 2;
  for (int k = 0; k < 3; ++k) { c.jn[k] = k < nb && rng() % 6 != 0; c.mn[k] = k < nb && rng() % 5 != 0; }
  if (rng() % 3 == 0 && c.haveJac) for (int k = 0; k < nb; ++k) c.jn[k] = true;
  return c;
}
static const double SENT = -1.2345678901234e-300;
struct EvalBufs {
  double res[16]; std::vector<double> J[3], M[3]; double* jp[3]; double* mp[3];
  EvalBufs(const Cfg& c) {
    for (int k = 0; k < 3; ++k) {
      J[k].assign(100, SENT); M[k].assign(100, SENT);
      jp[k] = c.jn[k] ? J[k].data() : nullptr; mp[k] = c.mn[k] ? M[k].data() : nullptr;
    }
    for (int i = 0; i < 16; ++i) res[i] = SENT;
  }
};
template <class CppF, class CF>
static void check_eval(const std::string& tag, int nres, int nb, CppF cppf, CF cf) {
  Cfg cfg = rcfg(nb);
  EvalBufs a(cfg), b(cfg);
  double** ja = cfg.haveJac ? a.jp : nullptr; double** ma = cfg.haveMin ? a.mp : nullptr;
  double* const* jb = cfg.haveJac ? b.jp : nullptr; double* const* mb = cfg.haveMin ? b.mp : nullptr;
  bool r1 = cppf(a.res, ja, ma);
  bool r2 = cf(b.res, jb, mb) != 0;
  cmpi(tag + ".ret", r1, r2);
  cmp(tag + ".res", a.res, b.res, nres);
  for (int k = 0; k < nb; ++k) {
    cmp(tag + ".J" + std::to_string(k), a.J[k].data(), b.J[k].data(), 100);
    cmp(tag + ".Jmin" + std::to_string(k), a.M[k].data(), b.M[k].data(), 100);
  }
}

// ------------------------------------------------------------------------------------------- manifolds / blocks
static void test_param() {
  for (int it = 0; it < 200000; ++it) {
    double x[7], y[7], d[6], o1[7], o2[7], e6[6], g6[6];
    pose_params(x); pose_params(y);
    for (int i = 0; i < 6; ++i) d[i] = V(rng() % 4 ? 1.0 : 1e-5);
    if (rng() % 10 == 0) d[3] = d[4] = d[5] = 0.0;
    oc::PoseManifold::plus(x, d, o1); ok_pose_plus(x, d, o2); cmp("pose.plus", o1, o2, 7);
    oc::PoseManifold pm; pm.Plus(x, d, o1); cmp("pose.Plus", o1, o2, 7);
    oc::PoseManifold::minus(y, x, e6); ok_pose_minus(y, x, g6); cmp("pose.minus", e6, g6, 6);
    double J1[42], J2[42];
    oc::PoseManifold::plusJacobian(x, J1); ok_pose_plus_jacobian(x, J2); cmp("pose.plusJacobian", J1, J2, 42);
    oc::PoseManifold::minusJacobian(x, J1); ok_pose_minus_jacobian(x, J2); cmp("pose.minusJacobian", J1, J2, 42);
    // pose block
    { okvis::kinematics::Transformation T = rT(); oc::PoseParameterBlock b(T, 1); const double* pp = b.parameters();
      ok_tf ct = tf_c(T); double blk[7]; ok_pose_block_set_estimate(blk, &ct); cmp("poseblock.set", pp, blk, 7);
      ok_tf est; ok_pose_block_estimate(blk, &est); const kinematics::TransformationCacheless& e = b.estimate();
      double ec[7]; std::memcpy(ec, e.coeffs().data(), 56); double gc[7] = {est.r[0], est.r[1], est.r[2], est.q.x, est.q.y, est.q.z, est.q.w};
      cmp("poseblock.estimate", ec, gc, 7);
      Eigen::Matrix3d Cc = e.C(); cmp("poseblock.estimateC", Cc.data(), est.C, 9);
      double np[7]; pose_params(np); b.setParameters(np); double ec2[7]; std::memcpy(ec2, b.estimate().coeffs().data(), 56);
      double blk2[7]; ok_pose_block_set_parameters(blk2, np); cmp("poseblock.setParameters", ec2, blk2, 7);
      double a1[7], a2[7]; b.plus(x, d, a1); ok_pose_plus(x, d, a2); cmp("poseblock.plus", a1, a2, 7);
      double m1[6], m2[6]; b.minus(x, y, m1); ok_pose_minus(y, x, m2); cmp("poseblock.minus", m1, m2, 6);
      b.plusJacobian(x, J1); ok_pose_plus_jacobian(x, J2); cmp("poseblock.plusJacobian", J1, J2, 42);
      b.liftJacobian(x, J1); ok_pose_minus_jacobian(x, J2); cmp("poseblock.liftJacobian", J1, J2, 42); }
    // homogeneous point
    { double hx[4], hy[4], hd[3], h1[4], h2[4], m1[3], m2[3], K1[12], K2[12];
      for (int i = 0; i < 4; ++i) { hx[i] = V(3); hy[i] = V(3); } for (int i = 0; i < 3; ++i) hd[i] = V();
      oc::HomogeneousPointManifold::plus(hx, hd, h1); ok_hpoint_plus(hx, hd, h2); cmp("hpoint.plus", h1, h2, 4);
      oc::HomogeneousPointManifold::minus(hy, hx, m1); ok_hpoint_minus(hy, hx, m2); cmp("hpoint.minus", m1, m2, 3);
      oc::HomogeneousPointManifold::plusJacobian(hx, K1); ok_hpoint_plus_jacobian(hx, K2); cmp("hpoint.plusJacobian", K1, K2, 12);
      oc::HomogeneousPointManifold::minusJacobian(hx, K1); ok_hpoint_minus_jacobian(hx, K2); cmp("hpoint.minusJacobian", K1, K2, 12);
      oc::HomogeneousPointParameterBlock pb(Eigen::Vector4d(hx[0], hx[1], hx[2], hx[3]), 1);
      pb.plus(hx, hd, h1); cmp("hpointblock.plus", h1, h2, 4);
      pb.minus(hx, hy, m1); ok_hpoint_minus(hy, hx, m2); cmp("hpointblock.minus", m1, m2, 3);
      pb.plusJacobian(hx, K1); ok_hpoint_plus_jacobian(hx, K2); cmp("hpointblock.plusJacobian", K1, K2, 12);
      pb.liftJacobian(hx, K1); ok_hpoint_minus_jacobian(hx, K2); cmp("hpointblock.liftJacobian", K1, K2, 12);
      oc::HomogeneousPointParameterBlock p3(Eigen::Vector3d(hx[0], hx[1], hx[2]), 2); double blk[4]; ok_hpoint_block_from_v3(blk, hx);
      cmp("hpointblock.fromV3", p3.parameters(), blk, 4); }
    // speed and bias
    { double sx[9], sd[9], sy[9], s1[9], s2[9], K1[81], K2[81];
      for (int i = 0; i < 9; ++i) { sx[i] = V(); sd[i] = V(); sy[i] = V(); }
      SpeedAndBias est; for (int i = 0; i < 9; ++i) est[i] = sx[i];
      oc::SpeedAndBiasParameterBlock b(est, 1);
      b.plus(sx, sd, s1); ok_sab_plus(sx, sd, s2); cmp("sabblock.plus", s1, s2, 9);
      b.minus(sx, sy, s1); ok_sab_minus(sx, sy, s2); cmp("sabblock.minus", s1, s2, 9);
      b.plusJacobian(sx, K1); ok_sab_plus_jacobian(K2); cmp("sabblock.plusJacobian", K1, K2, 81);
      b.liftJacobian(sx, K1); ok_sab_minus_jacobian(K2); cmp("sabblock.liftJacobian", K1, K2, 81); }
  }
}

// ------------------------------------------------------------------------------------------- error terms
template <class E> struct Expose : public E {
  using E::E;
  const auto& sq() const { return this->squareRootInformation_; }
};
template <class E> struct ExposeH : public E {
  using E::E;
  const auto& sq() const { return this->_squareRootInformation; }
};

template <int Nn> static void flat_cmp(const std::string& n, const Eigen::Matrix<double, Nn, Nn>& a, const double* cm) { cmp(n, a.data(), cm, Nn * Nn); }

static void test_pose_err() {
  for (int it = 0; it < 120000; ++it) {
    kinematics::Transformation T = rT(); ok_tf cT = tf_c(T);
    // construction variants
    const int variant = int(rng() % 3);
    std::unique_ptr<Expose<oc::PoseError>> e; ok_pose_err c;
    Eigen::MatrixXd I = rinfo(6);
    Eigen::Matrix<double, 6, 1> dg; for (int i = 0; i < 6; ++i) dg[i] = std::fabs(V(50.0)) + (rng() % 7 ? 1e-3 : 0.0);
    double tv = std::fabs(V(0.5)) + 1e-3, rv = std::fabs(V(0.5)) + 1e-3; if (rng() % 9 == 0) { tv = 0.0; }
    if (variant == 0) { e.reset(new Expose<oc::PoseError>(T, fixed<6>(I))); ok_pose_err_init_info(&c, &cT, I.data()); }
    else if (variant == 1) { e.reset(new Expose<oc::PoseError>(T, dg)); ok_pose_err_init_diag(&c, &cT, dg.data()); }
    else { e.reset(new Expose<oc::PoseError>(T, tv, rv)); ok_pose_err_init_var(&c, &cT, tv, rv); }
    cmp("pose.ctor.info", e->information().data(), c.info, 36);
    cmp("pose.ctor.sqrt", e->sq().data(), c.sqrt_info, 36);
    for (int rep = 0; rep < 4; ++rep) {
      double p[7]; pose_params(p);
      if (rng() % 8 == 0) { Eigen::Quaterniond q = rqn(); Eigen::Quaterniond m = rqn(); (void)m; p[3] = q.x(); p[4] = q.y(); p[5] = q.z(); p[6] = q.w(); }
      const double* params[1] = {p};
      check_eval("pose.eval", 6, 1,
        [&](double* res, double** jac, double** jm) { return e->EvaluateWithMinimalJacobians(params, res, jac, jm); },
        [&](double* res, double* const* jac, double* const* jm) { return ok_pose_err_evaluate(&c, params, res, jac, jm); });
    }
    { double p[7]; pose_params(p); const double* params[1] = {p}; double r1[6], r2[6];
      e->Evaluate(params, r1, nullptr); ok_pose_err_evaluate(&c, params, r2, nullptr, nullptr); cmp("pose.Evaluate.res", r1, r2, 6); }
  }
}

static void test_sab_err() {
  for (int it = 0; it < 120000; ++it) {
    SpeedAndBias m; for (int i = 0; i < 9; ++i) m[i] = V();
    Eigen::MatrixXd I = rinfo(9);
    const int variant = int(rng() % 2);
    double sv = std::fabs(V(0.5)) + 1e-3, gv = std::fabs(V(1e-3)) + 1e-9, av = std::fabs(V(1e-2)) + 1e-9;
    if (rng() % 3 == 0) { sv = 0.1; gv = 3.0e-3 * 3.0e-3; av = 2.0e-2 * 2.0e-2; }
    std::unique_ptr<Expose<oc::SpeedAndBiasError>> e; ok_sab_err c;
    if (variant == 0) { e.reset(new Expose<oc::SpeedAndBiasError>(m, fixed<9>(I))); ok_sab_err_init_info(&c, m.data(), I.data()); }
    else { e.reset(new Expose<oc::SpeedAndBiasError>(m, sv, gv, av)); ok_sab_err_init_var(&c, m.data(), sv, gv, av); }
    cmp("sab.ctor.info", e->information().data(), c.info, 81);
    cmp("sab.ctor.sqrt", e->sq().data(), c.sqrt_info, 81);
    for (int rep = 0; rep < 3; ++rep) {
      double p[9]; for (int i = 0; i < 9; ++i) p[i] = V();
      const double* params[1] = {p};
      check_eval("sab.eval", 9, 1,
        [&](double* res, double** jac, double** jm) { return e->EvaluateWithMinimalJacobians(params, res, jac, jm); },
        [&](double* res, double* const* jac, double* const* jm) { return ok_sab_err_evaluate(&c, params, res, jac, jm); });
    }
  }
}

static void test_relpose_err() {
  for (int it = 0; it < 120000; ++it) {
    kinematics::Transformation T = rT(); ok_tf cT = tf_c(T);
    Eigen::MatrixXd I = rinfo(6);
    const int variant = int(rng() % 2);
    double tv = std::fabs(V(0.5)) + 1e-3, rv = std::fabs(V(0.5)) + 1e-3;
    std::unique_ptr<Expose<oc::RelativePoseError>> e; ok_relpose_err c;
    if (variant == 0) { e.reset(new Expose<oc::RelativePoseError>(fixed<6>(I), T)); ok_relpose_err_init_info(&c, I.data(), &cT); }
    else { e.reset(new Expose<oc::RelativePoseError>(tv, rv, T)); ok_relpose_err_init_var(&c, tv, rv, &cT); }
    cmp("relpose.ctor.info", e->information().data(), c.info, 36);
    cmp("relpose.ctor.sqrt", e->sq().data(), c.sqrt_info, 36);
    for (int rep = 0; rep < 4; ++rep) {
      double a[7], b[7]; pose_params(a); pose_params(b);
      if (rng() % 3 == 0) { for (int i = 0; i < 7; ++i) b[i] = a[i] + (rng() % 2 ? 1e-3 * N() : 0.0); }
      const double* params[2] = {a, b};
      check_eval("relpose.eval", 6, 2,
        [&](double* res, double** jac, double** jm) { return e->EvaluateWithMinimalJacobians(params, res, jac, jm); },
        [&](double* res, double* const* jac, double* const* jm) { return ok_relpose_err_evaluate(&c, params, res, jac, jm); });
    }
  }
}

static void test_hpoint_err() {
  for (int it = 0; it < 120000; ++it) {
    Eigen::Vector4d m(V(3), V(3), V(3), V(2));
    Eigen::MatrixXd I = rinfo(3);
    const int variant = int(rng() % 2);
    double var = std::fabs(V(2.0)) + 1e-3;
    std::unique_ptr<ExposeH<oc::HomogeneousPointError>> e; ok_hpoint_err c;
    if (variant == 0) { e.reset(new ExposeH<oc::HomogeneousPointError>(m, fixed<3>(I))); ok_hpoint_err_init_info(&c, m.data(), I.data()); }
    else { e.reset(new ExposeH<oc::HomogeneousPointError>(m, var)); ok_hpoint_err_init_var(&c, m.data(), var); }
    cmp("hpoint.ctor.info", e->information().data(), c.info, 9);
    cmp("hpoint.ctor.sqrt", e->sq().data(), c.sqrt_info, 9);
    for (int rep = 0; rep < 3; ++rep) {
      double p[4]; for (int i = 0; i < 4; ++i) p[i] = V(3);
      const double* params[1] = {p};
      check_eval("hpoint.eval", 3, 1,
        [&](double* res, double** jac, double** jm) { return e->EvaluateWithMinimalJacobians(params, res, jac, jm); },
        [&](double* res, double* const* jac, double* const* jm) { return ok_hpoint_err_evaluate(&c, params, res, jac, jm); });
    }
  }
}

template <class D, class MakeD>
static void test_reproj(int dist, const char* tag, MakeD makeD) {
  const std::string t = std::string("reproj.") + tag;
  for (int it = 0; it < 100000; ++it) {
    const int w = 752, h = 480;
    const bool euroc = it % 3 == 0;
    double fu = euroc ? 458.654880721 : 300 + 400 * std::fabs(U()), fv = euroc ? 457.296696463 : 300 + 400 * std::fabs(U());
    double cu = euroc ? 367.215803962 : 300 + 150 * U(), cv = euroc ? 248.37534061 : 240 + 100 * U();
    double d[4] = {0, 0, 0, 0};
    if (dist == OK_CAM_RADTAN) { d[0] = -0.28 + 0.05 * N(); d[1] = 0.07 + 0.02 * N(); d[2] = 2e-4 * N(); d[3] = 2e-5 * N(); }
    else if (dist == OK_CAM_EQUIDISTANT) { d[0] = 0.03 * N(); d[1] = 0.01 * N(); d[2] = 0.003 * N(); d[3] = 0.001 * N(); }
    auto cam = std::make_shared<cameras::PinholeCamera<D>>(w, h, fu, fv, cu, cv, makeD(d));
    ok_cam cc; ok_cam_init(&cc, dist, w, h, fu, fv, cu, cv, d);
    Eigen::Vector2d meas(V(300) + 376, V(200) + 240);
    Eigen::MatrixXd I;
    const double size = 8.0 + 40.0 * std::fabs(U());
    Eigen::Matrix2d info;
    if (rng() % 3 != 0) info = 64.0 / (size * size) * Eigen::Matrix2d::Identity(); else info = fixed<2>(rinfo(2));
    std::unique_ptr<Expose<oc::ReprojectionError<cameras::PinholeCamera<D>>>> e(
        new Expose<oc::ReprojectionError<cameras::PinholeCamera<D>>>(cam, 0, meas, info));
    ok_reproj_err c; ok_reproj_err_init(&c, &cc, meas.data(), info.data());
    cmp(t + ".ctor.info", e->information().data(), c.info, 4);
    cmp(t + ".ctor.sqrt", e->sq().data(), c.sqrt_info, 4);
    if (rng() % 5 == 0) {  // setInformation path (as called by the estimator)
      e->setInformation(e->information()); ok_reproj_err_set_information(&c, c.info);
      cmp(t + ".setInfo.sqrt", e->sq().data(), c.sqrt_info, 4);
    }
    for (int rep = 0; rep < 3; ++rep) {
      double p0[7], p2[7], hp[4]; pose_params(p0);
      if (rng() % 2) { Eigen::Quaterniond q = rqn(); Eigen::Quaterniond q2(q); p2[3] = q2.x(); p2[4] = q2.y(); p2[5] = q2.z(); p2[6] = q2.w();
        p2[0] = 0.1 * N(); p2[1] = 0.1 * N(); p2[2] = 0.1 * N(); } else pose_params(p2);
      if (rng() % 6 == 0) { p0[0] = p0[1] = p0[2] = 0.0; p0[3] = p0[4] = p0[5] = 0.0; p0[6] = 1.0; }
      if (rng() % 6 == 0) { p2[0] = p2[1] = p2[2] = 0.0; p2[3] = p2[4] = p2[5] = 0.0; p2[6] = 1.0; }
      hp[0] = V(3); hp[1] = V(3); hp[2] = V(3) + (rng() % 3 ? 5.0 : 0.0); hp[3] = rng() % 3 ? 1.0 : (rng() % 2 ? 0.0 : V(0.5));
      // skip degenerate projections (|z| ~ 0: the C++ code leaves the keypoint uninitialised there)
      { kinematics::Transformation Twc = kinematics::Transformation(Eigen::Vector3d(p0[0], p0[1], p0[2]), Eigen::Quaterniond(p0[6], p0[3], p0[4], p0[5]).normalized());
        kinematics::Transformation Tsc = kinematics::Transformation(Eigen::Vector3d(p2[0], p2[1], p2[2]), Eigen::Quaterniond(p2[6], p2[3], p2[4], p2[5]).normalized());
        Eigen::Vector4d hc = Tsc.inverse().T() * Twc.inverse().T() * Eigen::Vector4d(hp[0], hp[1], hp[2], hp[3]);
        if (!(std::fabs(hc[2]) > 1e-6)) continue; }
      const double* params[3] = {p0, hp, p2};
      check_eval(t + ".eval", 2, 3, [&](double* res, double** jac, double** jm) { return e->EvaluateWithMinimalJacobians(params, res, jac, jm); },
                 [&](double* res, double* const* jac, double* const* jm) { return ok_reproj_err_evaluate(&c, params, res, jac, jm); });
    }
  }
}

int main() {
  test_param();
  test_pose_err();
  test_sab_err();
  test_relpose_err();
  test_hpoint_err();
  test_reproj<cameras::RadialTangentialDistortion>(OK_CAM_RADTAN, "radtan", [](const double* d) { return cameras::RadialTangentialDistortion(d[0], d[1], d[2], d[3]); });
  test_reproj<cameras::EquidistantDistortion>(OK_CAM_EQUIDISTANT, "equi", [](const double* d) { return cameras::EquidistantDistortion(d[0], d[1], d[2], d[3]); });
  test_reproj<cameras::NoDistortion>(OK_CAM_NODIST, "nodist", [](const double*) { return cameras::NoDistortion(); });
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("  %-28s %ld/%ld\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("okvis_err_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
