// OK_PORT_TEST_SRC: okvis_time/src/Time.cpp okvis_time/src/Duration.cpp okvis_cv/src/CameraBase.cpp okvis_ceres/src/PoseLocalParameterization.cpp okvis_ceres/src/HomogeneousPointLocalParameterization.cpp okvis_ceres/src/PoseParameterBlock.cpp okvis_ceres/src/HomogeneousPointParameterBlock.cpp okvis_ceres/src/TwoPoseGraphError.cpp okvis_ceres/src/TwoPoseExtrinsicsGraphError.cpp
// OK_PORT_TEST_C: ok_time.c ok_kin.c ok_cam.c ok_eigen.c ok_param.c ok_err.c ok_dense.c ok_blas.c ok_twopose.c ok_graph.c
// OK_PORT_TEST_LIBS: ceres glog
// Random-case, tolerance-0 comparison of the C module ok_twopose (module M5a) against the *real* OKVIS2 classes
// TwoPoseStandardGraphError / TwoPoseStandardGraphErrorConst (addObservation + compute + EvaluateWithMinimalJacobians
// with every Jacobian-pointer combination + convertToReprojectionErrors), TwoPoseExtrinsicsGraphError(Const) (the
// never-used online-extrinsics variant: real compute, ported Evaluate), okvis::PseudoInverse::symm / symmSqrt /
// symmSqrtU and the Vector4d norm used by ViGraph::updateLandmarks; compiled with the reference flags
// (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0, SSE2). Scenarios: 1-2 cameras, 3-40 landmarks,
// outliers (residual norm > 3), Cauchy and no loss, duplications, reference pose moved between addObservation and
// compute (live vs snapshot parameters), perturbed and unnormalised quaternions at Evaluate.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <ceres/loss_function.h>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/PseudoInverse.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/TwoPoseGraphError.hpp>
#include <okvis/ceres/TwoPoseExtrinsicsGraphError.hpp>
extern "C" {
#include "../c/ok_kin.h"
#include "../c/ok_cam.h"
#include "../c/ok_param.h"
#include "../c/ok_err.h"
#include "../c/ok_twopose.h"
#include "../c/ok_graph.h"
}

using namespace okvis;
namespace oc = okvis::ceres;
typedef cameras::PinholeCamera<cameras::RadialTangentialDistortion> Cam;
typedef oc::ReprojectionError<Cam> Reproj;

static std::mt19937_64 rng(20261003);
static double U() { return std::uniform_real_distribution<double>(-1.0, 1.0)(rng); }
static double N() { return std::normal_distribution<double>(0.0, 1.0)(rng); }
static std::map<std::string, std::pair<long, long>> stats;  // name -> (mismatches, compared)
static int printed = 0;
static void cmp(const std::string& name, const double* got, const double* want, long n) {
  auto& s = stats[name];
  for (long i = 0; i < n; ++i) {
    s.second++;
    if (std::memcmp(&got[i], &want[i], 8) != 0) {
      s.first++;
      if (printed < 40) { printed++; std::printf("  MISMATCH %s[%ld]: got %.17g want %.17g\n", name.c_str(), i, got[i], want[i]); }
    }
  }
}
static void cmpi(const std::string& name, long got, long want) {
  auto& s = stats[name];
  s.second++;
  if (got != want) { s.first++; if (printed < 40) { printed++; std::printf("  MISMATCH %s: got %ld want %ld\n", name.c_str(), got, want); } }
}

static Eigen::Quaterniond randq(double scale = 1.0) {
  Eigen::Vector4d v(N(), N(), N(), N());
  if (scale < 1.0) { v = Eigen::Vector4d(scale * N(), scale * N(), scale * N(), 1.0); }
  v.normalize();
  return Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
}

// a derived class exposing the protected state
struct ProbeStd : public oc::TwoPoseStandardGraphError {
  ProbeStd(StateId a, StateId b, size_t n) : oc::TwoPoseStandardGraphError(a, b, n) {}
  const Eigen::Matrix<double, 6, 6>& H00() const { return H00_; }
  const Eigen::Matrix<double, 6, 1>& b0() const { return b0_; }
  const Eigen::Matrix<double, 6, 6>& J() const { return J_; }
  const Eigen::Matrix<double, 6, 1>& DeltaX() const { return DeltaX_; }
  const kinematics::Transformation& lin() const { return linearisationPoint_T_S0S1_; }
  bool computed() const { return isComputed_; }
  const AlignedMap<uint64_t, Eigen::Vector4d>& lms() const { return landmarks_; }
  const std::map<uint64_t, std::vector<Observation>>& obs() const { return observations_; }
};
struct ProbeExt : public oc::TwoPoseExtrinsicsGraphError {
  ProbeExt(StateId a, StateId b, size_t n) : oc::TwoPoseExtrinsicsGraphError(a, b, n) {}
  const Eigen::MatrixXd& J() const { return J_; }
  const Eigen::VectorXd& DeltaX() const { return DeltaX_; }
  const kinematics::Transformation& lin() const { return linearisationPoint_T_S0S1_; }
  const std::vector<std::shared_ptr<kinematics::Transformation>>& linSC() const { return linearisationPoints_T_SC_; }
  bool computed() const { return isComputed_; }
};

static void tf_to_c(const kinematics::Transformation& T, ok_tf* out) {
  double c[7];
  for (int i = 0; i < 3; ++i) c[i] = T.r()[i];
  for (int i = 0; i < 4; ++i) c[3 + i] = T.q().coeffs()[i];
  ok_tf_set_coeffs(out, c, 1);
}

struct Scenario {
  int numCams;
  std::vector<std::shared_ptr<Cam>> cams;
  std::vector<ok_cam> ccams;
  std::vector<std::shared_ptr<oc::PoseParameterBlock>> extr;
  std::shared_ptr<oc::PoseParameterBlock> pose[2];
  std::vector<std::shared_ptr<oc::HomogeneousPointParameterBlock>> lms;
  struct Ob { KeypointIdentifier kid; std::shared_ptr<Reproj> err; ok_reproj_err cerr; bool loss, dup; int lm; };
  std::vector<Ob> obs;
};

static ::ceres::CauchyLoss cauchy(1.0);

static Scenario make_scenario(int numCams) {
  Scenario S;
  S.numCams = numCams;
  for (int c = 0; c < numCams; ++c) {
    const int w = 752, h = 480;
    const double fu = 458.65 + 20 * U(), fv = 457.29 + 20 * U(), cu = 367.2 + 10 * U(), cv = 248.4 + 10 * U();
    double d[4] = {-0.28 + 0.05 * N(), 0.07 + 0.02 * N(), 2e-4 * N(), 2e-5 * N()};
    S.cams.push_back(std::make_shared<Cam>(w, h, fu, fv, cu, cv, cameras::RadialTangentialDistortion(d[0], d[1], d[2], d[3])));
    ok_cam cc; ok_cam_init(&cc, OK_CAM_RADTAN, w, h, fu, fv, cu, cv, d);
    S.ccams.push_back(cc);
    kinematics::Transformation T_SC(Eigen::Vector3d(0.1 * c + 0.02 * N(), 0.02 * N(), 0.02 * N()), randq(0.05));
    S.extr.push_back(std::make_shared<oc::PoseParameterBlock>(T_SC, 100 + c, Time(0)));
  }
  for (int k = 0; k < 2; ++k) {
    kinematics::Transformation T_WS(Eigen::Vector3d(N(), N(), N()) * (k ? 0.5 : 1.0), k ? randq(0.3) : randq());
    S.pose[k] = std::make_shared<oc::PoseParameterBlock>(T_WS, 1 + k, Time(0));
  }
  const int K = 3 + int(rng() % 38);
  for (int l = 0; l < K; ++l) {
    const int c = int(rng() % numCams);
    const double z = 0.5 + 8.0 * std::fabs(U()), x = z * 0.6 * U(), y = z * 0.4 * U();
    Eigen::Vector4d hp_C(x, y, z, 1.0);
    if (rng() % 7 == 0) hp_C *= 0.5 + std::fabs(N());  // non-unit w
    Eigen::Vector4d hp_W = S.pose[0]->estimate() * (S.extr[c]->estimate() * hp_C);
    S.lms.push_back(std::make_shared<oc::HomogeneousPointParameterBlock>(hp_W, 1000 + l, rng() % 2 == 0));
  }
  // observations: landmark-major or pose-major insertion order
  const bool lmMajor = rng() % 2 == 0;
  std::vector<std::pair<int, int>> order;  // (lm, pose)
  for (int l = 0; l < K; ++l) for (int k = 0; k < 2; ++k) order.push_back({l, k});
  if (!lmMajor) std::stable_sort(order.begin(), order.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) { return a.second < b.second; });
  int kp = 0;
  for (auto& lk : order) {
    const int l = lk.first, k = lk.second;
    for (int c = 0; c < numCams; ++c) {
      if (rng() % 4 == 0 && S.obs.size() > 2) continue;  // not every landmark is seen everywhere
      kinematics::Transformation T_WC = S.pose[k]->estimate() * S.extr[c]->estimate();
      Eigen::Vector4d hp_C = T_WC.inverse() * S.lms[l]->estimate();
      Eigen::Vector2d px;
      auto st = S.cams[c]->projectHomogeneous(hp_C, &px);
      if (st != cameras::ProjectionStatus::Successful) continue;
      px += Eigen::Vector2d(0.5 * N(), 0.5 * N());
      if (rng() % 10 == 0) px += Eigen::Vector2d(20 * U(), 20 * U());  // outlier
      const double size = 8.0 + 40.0 * std::fabs(U());
      Eigen::Matrix2d info = 64.0 / (size * size) * Eigen::Matrix2d::Identity();
      Scenario::Ob ob;
      ob.kid = KeypointIdentifier(uint64_t(1 + k), size_t(c), size_t(kp++));
      ob.err = std::make_shared<Reproj>(S.cams[c], uint64_t(c), px, info);
      ok_reproj_err_init(&ob.cerr, &S.ccams[c], px.data(), info.data());
      ob.loss = rng() % 5 != 0;
      ob.dup = rng() % 5 == 0;
      ob.lm = l;
      S.obs.push_back(ob);
    }
  }
  {  // both poses must be observed (compute() asserts otherwise)
    bool seen[2] = {false, false};
    for (auto& ob : S.obs) seen[ob.kid.frameId - 1] = true;
    if (!seen[0] || !seen[1] || S.obs.empty()) return make_scenario(numCams);
  }
  return S;
}

static void add_all(Scenario& S, oc::TwoPoseGraphError& term, ok_twopose& t) {
  for (auto& ob : S.obs) {
    const int k = int(ob.kid.frameId) - 1, c = int(ob.kid.cameraIndex);
    term.addObservation(ob.kid, ob.err, ob.loss ? &cauchy : nullptr, S.pose[k], S.lms[ob.lm], S.extr[c], ob.dup);
    ok_twopose_add_observation(&t, ob.kid.frameId, c, int(ob.kid.keypointIndex), &ob.cerr, ob.loss ? 1 : 0, S.pose[k]->id(),
                               S.pose[k]->parameters(), S.lms[ob.lm]->id(), S.lms[ob.lm]->parameters(), S.lms[ob.lm]->initialized(),
                               S.extr[c]->id(), S.extr[c]->parameters(), ob.dup ? 1 : 0);
  }
}

// evaluate a 2-pose term (real vs C) with every pointer combination
template <class RealEval, class CEval>
static void eval_combos(const std::string& tag, int nb, int nres, const double* const* params, RealEval real, CEval cfun) {
  const double SENT = -7.25e300;
  std::vector<std::vector<double>> J(nb), Jc(nb), Jm(nb), Jmc(nb);
  for (int k = 0; k < nb; ++k) { J[k].resize(nres * 7); Jc[k].resize(nres * 7); Jm[k].resize(nres * 6); Jmc[k].resize(nres * 6); }
  std::vector<double> res(nres), resc(nres);
  const int ncombo = 1 << nb;
  for (int jc = 0; jc <= ncombo; ++jc)        // jc == ncombo: jacobians == nullptr
    for (int mc = 0; mc <= ncombo; ++mc) {    // mc == ncombo: jacobiansMinimal == nullptr
      if (jc == ncombo && mc != ncombo) continue;  // the C++ code dereferences jacobians[] then
      std::vector<double*> jp(nb), jpc(nb), mp(nb), mpc(nb);
      for (int k = 0; k < nb; ++k) {
        std::fill(J[k].begin(), J[k].end(), SENT); std::fill(Jc[k].begin(), Jc[k].end(), SENT);
        std::fill(Jm[k].begin(), Jm[k].end(), SENT); std::fill(Jmc[k].begin(), Jmc[k].end(), SENT);
        jp[k] = (jc & (1 << k)) ? J[k].data() : nullptr; jpc[k] = (jc & (1 << k)) ? Jc[k].data() : nullptr;
        mp[k] = (mc & (1 << k)) ? Jm[k].data() : nullptr; mpc[k] = (mc & (1 << k)) ? Jmc[k].data() : nullptr;
      }
      std::fill(res.begin(), res.end(), SENT); std::fill(resc.begin(), resc.end(), SENT);
      const bool r0 = real(params, res.data(), jc == ncombo ? nullptr : jp.data(), mc == ncombo ? nullptr : mp.data());
      const int r1 = cfun(params, resc.data(), jc == ncombo ? nullptr : jpc.data(), mc == ncombo ? nullptr : mpc.data());
      cmpi(tag + ".ret", r1, r0 ? 1 : 0);
      cmp(tag + ".res", resc.data(), res.data(), nres);
      for (int k = 0; k < nb; ++k) { cmp(tag + ".J" + std::to_string(k), Jc[k].data(), J[k].data(), nres * 7); cmp(tag + ".Jmin" + std::to_string(k), Jmc[k].data(), Jm[k].data(), nres * 6); }
    }
}

static void perturb_params(const Scenario& S, int nb, std::vector<std::array<double, 7>>& p) {
  p.resize(nb);
  for (int k = 0; k < nb; ++k) {
    const double* src = k < 2 ? S.pose[k]->parameters() : S.extr[k - 2]->parameters();
    const int mode = int(rng() % 4);
    if (mode == 0) { std::memcpy(p[k].data(), src, 56); }
    else if (mode == 1) { double d[6]; for (int i = 0; i < 6; ++i) d[i] = 0.05 * N(); ok_pose_plus(src, d, p[k].data()); }
    else if (mode == 2) { for (int i = 0; i < 7; ++i) p[k][i] = src[i] + 0.01 * N(); }   // unnormalised quaternion
    else { for (int i = 0; i < 3; ++i) p[k][i] = src[i] + 0.3 * N(); Eigen::Quaterniond q = randq(0.2) * Eigen::Quaterniond(src[6], src[3], src[4], src[5]); for (int i = 0; i < 4; ++i) p[k][3 + i] = q.coeffs()[i] * (1.0 + 0.1 * U()); }
  }
}

static void test_standard(int iters) {
  for (int it = 0; it < iters; ++it) {
    Scenario S = make_scenario(1 + int(rng() % 2));
    ProbeStd term(StateId(1), StateId(2), size_t(S.numCams));
    ok_twopose t; ok_twopose_init(&t, 1, 2, S.numCams, 1);
    add_all(S, term, t);
    if (rng() % 3 == 0) {  // the reference pose block moves between addObservation and compute (live != snapshot)
      double d[6], np[7]; for (int i = 0; i < 6; ++i) d[i] = 0.1 * N();
      ok_pose_plus(S.pose[0]->parameters(), d, np);
      S.pose[0]->setParameters(np);
      std::memcpy(t.pose_live[0], np, 56);
    }
    // structural bookkeeping checks are implicit in the numeric comparison below
    const bool ok = term.compute();
    const int okc = ok_twopose_compute(&t);
    cmpi("std.compute.ret", okc, ok ? 1 : 0);
    cmpi("std.compute.isComputed", t.term.is_computed, term.computed() ? 1 : 0);
    cmp("std.compute.H00", t.H00, term.H00().data(), 36);
    cmp("std.compute.b0", t.b0, term.b0().data(), 6);
    cmp("std.compute.J", t.term.J, term.J().data(), 36);
    cmp("std.compute.DeltaX", t.term.DeltaX, term.DeltaX().data(), 6);
    { ok_tf L; tf_to_c(term.lin(), &L); double a[7] = {t.term.lin_T_S0S1.r[0], t.term.lin_T_S0S1.r[1], t.term.lin_T_S0S1.r[2], t.term.lin_T_S0S1.q.x, t.term.lin_T_S0S1.q.y, t.term.lin_T_S0S1.q.z, t.term.lin_T_S0S1.q.w};
      double b[7] = {L.r[0], L.r[1], L.r[2], L.q.x, L.q.y, L.q.z, L.q.w}; cmp("std.compute.lin", a, b, 7); cmp("std.compute.linC", t.term.lin_T_S0S1.C, L.C, 9); }
    cmpi("std.compute.nlm", t.nlm_S0, long(term.lms().size()));
    { int i = 0; for (auto& kv : term.lms()) { if (i < t.nlm_S0) { cmpi("std.compute.lmid", long(t.lm_S0[i].id), long(kv.first)); cmp("std.compute.lmS0", t.lm_S0[i].hp_S0, kv.second.data(), 4); } ++i; } }
    { int g = 0; for (auto& grp : term.obs()) { int o = 0; for (auto& ob : grp.second) { if (g < t.ngroups && o < t.groups[g].nobs) cmpi("std.compute.marg", t.groups[g].obs[o].is_marginalised, ob.isMarginalised ? 1 : 0); ++o; } ++g; } }
    // Evaluate (standard) at perturbed parameters, all pointer combinations; and the Const clone
    std::shared_ptr<oc::TwoPoseGraphErrorConst> cl = term.cloneTwoPoseGraphErrorConst();
    auto* clc = dynamic_cast<oc::TwoPoseStandardGraphErrorConst*>(cl.get());
    for (int e = 0; e < 3; ++e) {
      std::vector<std::array<double, 7>> p; perturb_params(S, 2, p);
      const double* params[2] = {p[0].data(), p[1].data()};
      eval_combos("std.eval", 2, 6, params,
                  [&](const double* const* pr, double* res, double** jac, double** jm) { return term.EvaluateWithMinimalJacobians(pr, res, jac, jm); },
                  [&](const double* const* pr, double* res, double* const* jac, double* const* jm) { return ok_tp_std_evaluate(&t.term, pr, res, jac, jm); });
      ok_tp_std cc = t.term; cc.is_computed = 1;
      eval_combos("const.eval", 2, 6, params,
                  [&](const double* const* pr, double* res, double** jac, double** jm) { return clc->EvaluateWithMinimalJacobians(pr, res, jac, jm); },
                  [&](const double* const* pr, double* res, double* const* jac, double* const* jm) { return ok_tp_std_evaluate(&cc, pr, res, jac, jm); });
      if (e == 0) {  // plain Evaluate (jacobians only) through the ceres interface
        double res[6], J0[42], J1[42], resc[6], J0c[42], J1c[42]; double* jac[2] = {J0, J1}; double* jacc[2] = {J0c, J1c};
        term.Evaluate(params, res, jac); ok_tp_std_evaluate(&t.term, params, resc, jacc, nullptr);
        cmp("std.Evaluate.res", resc, res, 6); cmp("std.Evaluate.J0", J0c, J0, 42); cmp("std.Evaluate.J1", J1c, J1, 42);
      }
    }
    // not computed: Evaluate returns false
    { ProbeStd fresh(StateId(1), StateId(2), size_t(S.numCams)); ok_tp_std cc = t.term; cc.is_computed = 0; double res[6], resc[6];
      const double* params[2] = {S.pose[0]->parameters(), S.pose[1]->parameters()};
      cmpi("std.notcomputed", ok_tp_std_evaluate(&cc, params, resc, nullptr, nullptr), fresh.EvaluateWithMinimalJacobians(params, res, nullptr, nullptr) ? 1 : 0); }
    // convertToReprojectionErrors (with a moved reference pose in some cases)
    if (rng() % 2 == 0) { double d[6], np[7]; for (int i = 0; i < 6; ++i) d[i] = 0.1 * N(); ok_pose_plus(S.pose[0]->parameters(), d, np); S.pose[0]->setParameters(np); }
    {
      std::vector<oc::TwoPoseGraphError::Observation> out; std::vector<KeypointIdentifier> dup;
      term.convertToReprojectionErrors(out, dup);
      std::vector<std::array<double, 4>> hp(out.size() + 1);
      int cdup = 0;
      const int n = ok_twopose_convert(&t, S.pose[0]->parameters(), reinterpret_cast<double(*)[4]>(hp.data()), int(hp.size()), &cdup);
      cmpi("std.convert.n", n, long(out.size()));
      cmpi("std.convert.dup", cdup, long(dup.size()));
      for (size_t i = 0; i < out.size() && int(i) < n; ++i) cmp("std.convert.hp", hp[i].data(), out[i].hPoint->estimate().data(), 4);
      cmpi("std.convert.cleared", t.ngroups + t.nlm + t.nlm_S0, 0);
    }
    ok_twopose_free(&t);
  }
}

static void test_extrinsics(int iters) {
  for (int it = 0; it < iters; ++it) {
    Scenario S = make_scenario(1 + int(rng() % 2));
    ProbeExt term(StateId(1), StateId(2), size_t(S.numCams));
    ok_twopose t; ok_twopose_init(&t, 1, 2, S.numCams, 1);
    add_all(S, term, t);
    ok_twopose_free(&t);
    term.compute();  // the real computation (its compute() is not ported)
    ok_tp_ext x; std::memset(&x, 0, sizeof x);
    x.is_computed = term.computed() ? 1 : 0;
    x.n = int(term.DeltaX().size());
    x.nextr = int(term.linSC().size());
    for (int i = 0; i < x.n; ++i) x.DeltaX[i] = term.DeltaX()[i];
    for (int j = 0; j < x.n; ++j) for (int i = 0; i < x.n; ++i) x.J[i + x.n * j] = term.J()(i, j);
    tf_to_c(term.lin(), &x.lin_T_S0S1);
    for (int c = 0; c < x.nextr; ++c) { x.extr_present[c] = term.linSC()[c] ? 1 : 0; if (term.linSC()[c]) tf_to_c(*term.linSC()[c], &x.lin_T_SC[c]); }
    std::shared_ptr<oc::TwoPoseGraphErrorConst> cl = term.cloneTwoPoseGraphErrorConst();
    auto* clc = dynamic_cast<oc::TwoPoseExtrinsicsGraphErrorConst*>(cl.get());
    const int nb = 2 + x.nextr;
    for (int e = 0; e < 3; ++e) {
      std::vector<std::array<double, 7>> p; perturb_params(S, nb, p);
      std::vector<const double*> params(nb); for (int k = 0; k < nb; ++k) params[k] = p[k].data();
      eval_combos("ext.eval", nb, x.n, params.data(),
                  [&](const double* const* pr, double* res, double** jac, double** jm) { return term.EvaluateWithMinimalJacobians(pr, res, jac, jm); },
                  [&](const double* const* pr, double* res, double* const* jac, double* const* jm) { return ok_tp_ext_evaluate(&x, pr, res, jac, jm); });
      ok_tp_ext xc = x; xc.is_computed = 1;
      eval_combos("extconst.eval", nb, x.n, params.data(),
                  [&](const double* const* pr, double* res, double** jac, double** jm) { return clc->EvaluateWithMinimalJacobians(pr, res, jac, jm); },
                  [&](const double* const* pr, double* res, double* const* jac, double* const* jm) { return ok_tp_ext_evaluate(&xc, pr, res, jac, jm); });
    }
  }
}

template <int DIM>
static void test_pinv(int iters, const char* tag) {
  typedef Eigen::Matrix<double, DIM, DIM> M;
  for (int it = 0; it < iters; ++it) {
    M B; for (int k = 0; k < DIM * DIM; ++k) B.data()[k] = N();
    Eigen::Matrix<double, DIM, 1> d; for (int k = 0; k < DIM; ++k) d[k] = (rng() % 3 == 0) ? 0.0 : std::exp(6.0 * N());
    M A = B * d.asDiagonal() * B.transpose();
    if (it % 4 == 0) A = (A + A.transpose()).eval();
    if (it % 9 == 0) A.setZero();
    const double eps = (rng() % 2) ? 1.0e-7 : 1.0e-6;
    M R1, R2, R3; int r1, r2, r3;
    PseudoInverse::symm(A, R1, eps, &r1); PseudoInverse::symmSqrt(A, R2, eps, &r2); PseudoInverse::symmSqrtU(A, R3, eps, &r3);
    double C1[DIM * DIM], C2[DIM * DIM], C3[DIM * DIM]; int c1, c2, c3;
    ok_pinv_symm(DIM, A.data(), C1, eps, &c1); ok_pinv_symm_sqrt(DIM, A.data(), C2, eps, &c2); ok_pinv_symm_sqrt_u(DIM, A.data(), C3, eps, &c3);
    cmp(std::string("pinv.symm.") + tag, C1, R1.data(), DIM * DIM); cmpi(std::string("pinv.symm.rank.") + tag, c1, r1);
    cmp(std::string("pinv.symmSqrt.") + tag, C2, R2.data(), DIM * DIM); cmpi(std::string("pinv.symmSqrt.rank.") + tag, c2, r2);
    cmp(std::string("pinv.symmSqrtU.") + tag, C3, R3.data(), DIM * DIM); cmpi(std::string("pinv.symmSqrtU.rank.") + tag, c3, r3);
  }
}

int main(int argc, char** argv) {
  const int n = argc > 1 ? atoi(argv[1]) : 300;
  test_standard(n);
  test_extrinsics(n / 3);
  test_pinv<3>(n * 20, "3");
  test_pinv<6>(n * 10, "6");
  for (int it = 0; it < n * 100; ++it) {
    Eigen::Vector4d v(N() * std::exp(3 * U()), N(), N() * 1e-3, (rng() % 3) ? 1.0 : N());
    const double want = v.norm(), got = ok_v4_norm(v.data());
    cmp("v4norm", &got, &want, 1);
  }
  long bad = 0, tot = 0;
  for (auto& kv : stats) { std::printf("  %-28s %ld/%ld\n", kv.first.c_str(), kv.second.first, kv.second.second); bad += kv.second.first; tot += kv.second.second; }
  std::printf("okvis_twopose_test: %ld/%ld mismatches\n", bad, tot);
  return bad != 0 || tot == 0;
}
