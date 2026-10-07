// OK_PORT_TEST_C: ok_align4.c ok_solve.c ok_solve_linear.c ok_blas.c ok_dense.c ok_sparse.c ok_amd.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_twopose.c ok_graph.c ok_gps.c ok_gps_init.c
// OK_PORT_TEST_LIBS: ceres
// Tolerance-0 (memcmp) comparison of okvis_port/c/ok_align4.{h,c} (+ the DENSE_QR path of ok_solve.c / ok_solve_linear.c) against
// the REAL Ceres 2.2.0 / Eigen 3.4.0 for OKVIS2-X's Align4DoF_Ceres (ViGraph.cpp:35-110, commit 38043e4):
//   1. FourDoFResidual (copied verbatim from ViGraph.cpp between the BEGIN/END VERBATIM markers) through
//      ceres::AutoDiffCostFunction<FourDoFResidual, 3, 7>::Evaluate, with jacobians (Jet instantiation) and without (double);
//   2. ceres::internal::DenseQR (EIGEN) FactorAndSolve = Eigen HouseholderQR::solve, matrices rows 4..700 x cols 1..8, with
//      special values (+-0, tiny, huge), the augmented [J; D] shape of the LM step included;
//   3. the whole Problem (one PoseManifold4d block, N residual blocks with CauchyLoss(3.0), DENSE_QR, default
//      LEVENBERG_MARQUARDT, max_num_iterations 100) through ::ceres::Solve: every IterationSummary (cost, cost_change,
//      gradient_max_norm, gradient_norm, step_norm, relative_decrease, trust_region_radius, step flags), the termination type
//      and the final parameters.
// PoseManifold4d is copied from okvis_ceres/src/PoseLocalParameterization.cpp (plus / plusJacobian only, as Ceres calls them).
// Usage: okvis_align4_test [cases_per_seed [seed ...]]  (default 3000 problems, seeds 1 2 3); OKA4_VERBOSE=1 prints mismatches.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <ceres/ceres.h>
#include "ceres/dense_qr.h"
#include "ceres/linear_solver.h"
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
extern "C" {
#include "../c/ok_align4.h"
#include "../c/ok_kin.h"
#include "../c/ok_solve.h"
}

// ---- BEGIN VERBATIM (okvis_ceres/src/ViGraph.cpp:37-62) ----
struct FourDoFResidual {
  FourDoFResidual(const Eigen::Vector3d& point_G, const Eigen::Vector3d& point_W)
      : pG_(point_G), pW_(point_W) {}

  template <typename T>
  bool operator()(const T* const params, T* residuals) const {

    const Eigen::Matrix<T,3,1> r_GW(params[0], params[1], params[2]);
    const Eigen::Quaternion<T> q_GW(params[6], params[3],params[4], params[5]);

    Eigen::Matrix<T,3,1> pW_in_G = q_GW * pW_.template cast<T>() + r_GW;
    Eigen::Matrix<T,3,1> error = pG_.template cast<T>() - pW_in_G;

    residuals[0] = error[0];
    residuals[1] = error[1];
    residuals[2] = error[2];
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  const Eigen::Vector3d pG_;
  const Eigen::Vector3d pW_;
};
// ---- END VERBATIM ----

// ---- BEGIN VERBATIM (okvis_ceres/src/PoseLocalParameterization.cpp, PoseManifold4d::plus / plusJacobian) ----
class PoseManifold4d : public ::ceres::Manifold {
 public:
  virtual ~PoseManifold4d() override = default;
  virtual int AmbientSize() const override { return 7; }
  virtual int TangentSize() const override { return 4; }
  static bool plus(const double* x, const double* delta, double* x_plus_delta) {
    Eigen::Matrix<double, 6, 1> delta_;
    delta_.setZero();
    delta_[0] = delta[0];
    delta_[1] = delta[1];
    delta_[2] = delta[2];
    delta_[5] = delta[3];
    okvis::kinematics::Transformation T(Eigen::Vector3d(x[0], x[1], x[2]), Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
    T.oplus(delta_);
    x_plus_delta[0] = T.r()[0];
    x_plus_delta[1] = T.r()[1];
    x_plus_delta[2] = T.r()[2];
    x_plus_delta[3] = T.q().coeffs()[0];
    x_plus_delta[4] = T.q().coeffs()[1];
    x_plus_delta[5] = T.q().coeffs()[2];
    x_plus_delta[6] = T.q().coeffs()[3];
    return true;
  }
  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override { return plus(x, delta, x_plus_delta); }
  static bool plusJacobian(const double* x, double* jacobian) {
    Eigen::Map<Eigen::Matrix<double, 7, 4, Eigen::RowMajor> > Jp(jacobian);
    Eigen::Matrix<double, 7, 6, Eigen::RowMajor> Jp_full;
    okvis::kinematics::Transformation T(Eigen::Vector3d(x[0], x[1], x[2]), Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
    T.oplusJacobian(Jp_full);
    Jp.topLeftCorner<7, 3>() = Jp_full.topLeftCorner<7, 3>();
    Jp.bottomRightCorner<7, 1>() = Jp_full.bottomRightCorner<7, 1>();
    return true;
  }
  virtual bool PlusJacobian(const double* x, double* jacobian) const override { return plusJacobian(x, jacobian); }
  virtual bool Minus(const double*, const double*, double*) const override { return false; }
  virtual bool MinusJacobian(const double*, double*) const override { return false; }
};
// ---- END VERBATIM ----

namespace {
struct Sec { std::string name; long bad = 0, tot = 0; };
std::vector<Sec> g_secs;
Sec& sec(const std::string& n) { for (auto& s : g_secs) if (s.name == n) return s; g_secs.push_back({n}); return g_secs.back(); }
bool g_verbose = false;
std::mt19937_64 rng;
double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
double N01() { return std::normal_distribution<double>(0.0, 1.0)(rng); }
int IR(int a, int b) { return a + int(rng() % uint64_t(b - a + 1)); }
double rnd() {
  const int k = int(rng() % 24);
  if (k == 0) return 0.0;
  if (k == 1) return -0.0;
  if (k == 2) return U(-1, 1) * 1e-9;
  if (k == 3) return U(-1, 1) * 1e6;
  return U(-3, 3);
}
void cmpd(Sec& s, const double* a, const double* b, long n, const char* what) {
  for (long i = 0; i < n; ++i) {
    s.tot++;
    if (std::memcmp(&a[i], &b[i], 8) != 0) {
      if (g_verbose && s.bad < 5) std::printf("    %s[%ld]: C %.17g real %.17g\n", what, i, a[i], b[i]);
      s.bad++;
    }
  }
}
void cmpi(Sec& s, long a, long b, const char* what) {
  s.tot++;
  if (a != b) { if (g_verbose && s.bad < 5) std::printf("    %s: C %ld real %ld\n", what, a, b); s.bad++; }
}

void rand_quat(double q[4]) {  // x y z w, sometimes unnormalised / axis-aligned / near identity
  const int k = int(rng() % 6);
  if (k == 0) { q[0] = q[1] = q[2] = 0; q[3] = 1; return; }
  if (k == 1) { const double a = U(-3.2, 3.2); q[0] = q[1] = 0; q[2] = std::sin(a / 2); q[3] = std::cos(a / 2); return; }
  if (k == 2) { for (int i = 0; i < 4; ++i) q[i] = rnd(); return; }
  Eigen::Quaterniond e(N01(), N01(), N01(), N01());
  e.normalize();
  q[0] = e.x(); q[1] = e.y(); q[2] = e.z(); q[3] = e.w();
}

void test_residual(int cases) {
  Sec& sr = sec("residual (Jet + double)");
  for (int it = 0; it < cases; ++it) {
    double x[7], pG[3], pW[3], q[4];
    rand_quat(q);
    for (int i = 0; i < 3; ++i) { x[i] = rnd(); pG[i] = rnd(); pW[i] = rnd(); }
    x[3] = q[0]; x[4] = q[1]; x[5] = q[2]; x[6] = q[3];
    FourDoFResidual* f = new FourDoFResidual(Eigen::Vector3d(pG[0], pG[1], pG[2]), Eigen::Vector3d(pW[0], pW[1], pW[2]));
    ::ceres::AutoDiffCostFunction<FourDoFResidual, 3, 7> cf(f);
    double res[3], jac[21], res2[3];
    double* jacs[1] = {jac};
    const double* params[1] = {x};
    cf.Evaluate(params, res, jacs);
    cf.Evaluate(params, res2, nullptr);
    ok_align4_term t;
    std::memcpy(t.pG, pG, 24); std::memcpy(t.pW, pW, 24);
    double cres[3], cjac[21], cres2[3];
    ok_align4_residual(&t, x, cres, cjac);
    ok_align4_residual(&t, x, cres2, nullptr);
    cmpd(sr, cres, res, 3, "res(jet)"); cmpd(sr, cjac, jac, 21, "jac"); cmpd(sr, cres2, res2, 3, "res(double)");
  }
}

void test_qr(int cases) {
  Sec& s = sec("DenseQR (Eigen HouseholderQR::solve)");
  ::ceres::internal::LinearSolver::Options opt;
  opt.dense_linear_algebra_library_type = ::ceres::EIGEN;
  for (int it = 0; it < cases; ++it) {
    const int cols = (it % 4 == 3) ? IR(1, 8) : 4;
    const int rows = (it < 60) ? cols + it % 12 : IR(cols, 700);
    std::vector<double> A(size_t(rows) * cols), b(rows), x(cols), xc(cols);
    const bool smooth = rng() % 3 != 0;
    for (auto& v : A) v = smooth ? U(-1, 1) : rnd();
    for (auto& v : b) v = smooth ? U(-1, 1) : rnd();
    if (cols > 1 && rng() % 7 == 0) for (int r = 0; r < rows; ++r) A[size_t(1) * rows + r] = 0.0;  // zero column
    std::vector<double> A2 = A;
    auto qr = ::ceres::internal::DenseQR::Create(opt);
    std::string msg;
    qr->FactorAndSolve(rows, cols, A2.data(), b.data(), x.data(), &msg);
    ok_eigen_hqr_solve(rows, cols, A.data(), b.data(), xc.data());
    cmpd(s, xc.data(), x.data(), cols, "x");
  }
}

struct Ctx { std::vector<ok_sv_iter> its; };
void on_iter(void* c, const ok_sv_iter* i) { static_cast<Ctx*>(c)->its.push_back(*i); }

void test_solve(int cases) {
  Sec& s = sec("Align4DoF problem (LM + DENSE_QR + Cauchy(3))");
  Sec& s_it = sec("  iterations");
  long total_iters = 0, max_iters = 0, nonconv = 0;
  for (int it = 0; it < cases; ++it) {
    const int n = (it < 30) ? 2 + it : IR(2, 220);
    const int mode = int(rng() % 5);
    double qt[4]; double yaw = U(-3.1, 3.1);
    if (mode == 4) rand_quat(qt); else { qt[0] = qt[1] = 0; qt[2] = std::sin(yaw / 2); qt[3] = std::cos(yaw / 2); }
    Eigen::Quaterniond Qt(qt[3], qt[0], qt[1], qt[2]); Qt.normalize();
    const Eigen::Vector3d tt(U(-50, 50), U(-50, 50), U(-5, 5));
    std::vector<double> G(3 * n), W(3 * n);
    const double span = (mode == 0) ? 0.3 : U(1, 40);
    const double noise = (mode == 1) ? 0.5 : U(0.005, 0.1);
    for (int i = 0; i < n; ++i) {
      Eigen::Vector3d w(U(-span, span), U(-span, span), U(-span * 0.2, span * 0.2));
      Eigen::Vector3d g = Qt * w + tt + Eigen::Vector3d(N01() * noise, N01() * noise, N01() * noise);
      if (rng() % 9 == 0) g += Eigen::Vector3d(U(-6, 6), U(-6, 6), U(-6, 6));  // outlier
      for (int k = 0; k < 3; ++k) { W[3 * i + k] = w[k]; G[3 * i + k] = g[k]; }
    }
    // initial guess: perturbed truth (yaw-only quaternion as the RANSAC / Umeyama result)
    double x0[7];
    const double dy = yaw + U(-0.3, 0.3);
    x0[0] = tt[0] + U(-1, 1); x0[1] = tt[1] + U(-1, 1); x0[2] = tt[2] + U(-0.5, 0.5);
    x0[3] = 0; x0[4] = 0; x0[5] = std::sin(dy / 2); x0[6] = std::cos(dy / 2);
    if (mode == 3) { x0[0] = x0[1] = x0[2] = 0; x0[5] = 0; x0[6] = 1; }

    // real Ceres
    double xr[7]; std::memcpy(xr, x0, sizeof xr);
    ::ceres::Problem problem;
    auto* manifold = new PoseManifold4d();
    problem.AddParameterBlock(xr, 7, manifold);
    for (int i = 0; i < n; ++i) {
      auto* cf = new ::ceres::AutoDiffCostFunction<FourDoFResidual, 3, 7>(
          new FourDoFResidual(Eigen::Vector3d(G[3 * i], G[3 * i + 1], G[3 * i + 2]), Eigen::Vector3d(W[3 * i], W[3 * i + 1], W[3 * i + 2])));
      problem.AddResidualBlock(cf, new ::ceres::CauchyLoss(3.0), xr);
    }
    ::ceres::Solver::Options options;
    options.linear_solver_type = ::ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, &problem, &summary);

    // C port
    ok_tf T0; double r0[3] = {x0[0], x0[1], x0[2]}; ok_quat q0 = {x0[3], x0[4], x0[5], x0[6]};
    ok_tf_from_rq(&T0, r0, &q0, 1);
    T0.q.x = x0[3]; T0.q.y = x0[4]; T0.q.z = x0[5]; T0.q.w = x0[6];   // the block holds the coefficients verbatim
    ok_tf Tout;
    Ctx ctx; ok_sv_hooks hooks; std::memset(&hooks, 0, sizeof hooks); hooks.ctx = &ctx; hooks.on_iter = on_iter;
    const int term = ok_align4dof_ceres(n, G.data(), W.data(), &T0, &Tout, &hooks);

    cmpi(s, term, int(summary.termination_type), "termination");
    cmpi(s, long(ctx.its.size()), long(summary.iterations.size()), "num iteration records");
    const size_t m = std::min(ctx.its.size(), summary.iterations.size());
    for (size_t k = 0; k < m; ++k) {
      const auto& a = ctx.its[k]; const auto& b = summary.iterations[k];
      const double ca[8] = {a.cost, a.cost_change, a.gradient_max_norm, a.gradient_norm, a.step_norm, a.relative_decrease, a.trust_region_radius, double(a.step_is_successful)};
      const double cb[8] = {b.cost, b.cost_change, b.gradient_max_norm, b.gradient_norm, b.step_norm, b.relative_decrease, b.trust_region_radius, double(b.step_is_successful)};
      cmpd(s_it, ca, cb, 8, "iteration fields");
    }
    double xc[7] = {Tout.r[0], Tout.r[1], Tout.r[2], Tout.q.x, Tout.q.y, Tout.q.z, Tout.q.w};
    // PoseParameterBlock::estimate() -> Transformation: compare against the same conversion of the real block
    okvis::kinematics::TransformationCacheless Tcl;           // PoseParameterBlock: estimate_.setCoeffs(parameters)
    Tcl.setCoeffs(Eigen::Map<const Eigen::Matrix<double, 7, 1>>(xr));
    okvis::kinematics::Transformation Te = Tcl;               // T_GW_refined = gpsParameterBlock.estimate()
    double xe[7] = {Te.r()[0], Te.r()[1], Te.r()[2], Te.q().coeffs()[0], Te.q().coeffs()[1], Te.q().coeffs()[2], Te.q().coeffs()[3]};
    cmpd(s, xc, xe, 7, "T_GW estimate");
    total_iters += long(summary.iterations.size()); max_iters = std::max<long>(max_iters, long(summary.iterations.size()));
    if (summary.termination_type != ::ceres::CONVERGENCE) nonconv++;
  }
  std::printf("  (Ceres iterations in total %ld, max %ld per problem, non-CONVERGENCE terminations %ld)\n", total_iters, max_iters, nonconv);
}
}  // namespace

int main(int argc, char** argv) {
  g_verbose = std::getenv("OKA4_VERBOSE") != nullptr;
  int cases = argc > 1 ? std::atoi(argv[1]) : 3000;
  std::vector<uint64_t> seeds;
  for (int i = 2; i < argc; ++i) seeds.push_back(std::strtoull(argv[i], nullptr, 10));
  if (seeds.empty()) seeds = {1, 2, 3};
  for (uint64_t sd : seeds) {
    rng.seed(sd);
    test_residual(cases * 2);
    test_qr(cases);
    test_solve(cases / 3 + 1);
  }
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("%-52s %ld/%ld mismatches\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("okvis_align4_test: %ld/%ld mismatches\n", bad, tot);
  return bad ? 1 : 0;
}
