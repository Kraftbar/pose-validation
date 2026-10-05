// OK_PORT_TEST_C: ok_opengv.c ok_opengv_gp3p_gen.c ok_opengv_stew_gen.c ok_eigen_eigsolver8.c ok_eigen_eigsolver10.c ok_eigen_cx.c ok_eigen.c ok_eigen_svd.c ok_eigen_qr.c ok_eigen_fullpivlu.c
// OK_PORT_TEST_LIBS: opengv
// Random-case, tolerance-0 comparison of module M7c (okvis_port/c/ok_opengv.c and friends) against the REAL OpenGV library
// (libopengv.a of the reference build) and the UNMODIFIED OKVIS2 Frame*SacProblem headers (driven through the shadow adapters of
// reference_tools/shadow, same class names over plain vectors), Eigen 3.4.0, libstdc++ and the reference flags
// -O2 -DNDEBUG -ffp-contract=off -fno-fast-math:
//   rng        std::mt19937(12345) + std::uniform_int_distribution<int>(0, INT_MAX) bound with std::bind as SampleConsensusProblem
//   complex    std::complex<double> operator/, operator*, std::sqrt vs ok_cdiv / ok_cmul / ok_csqrt
//   eigsolver  EigenSolver<Matrix<double,8,8>> / <10,10> eigenvalues and eigenvectors()
//   gp3p       opengv::absolute_pose::modules::gp3p_main
//   abs        FrameAbsolutePoseSacProblem<GP3P>: computeModelCoefficients, getSelectedDistancesToModel, Ransac::computeModel
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <climits>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <random>
#include <opengv/types.hpp>
#include <opengv/absolute_pose/modules/main.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/triangulation/methods.hpp>

extern "C" {
#include "../c/ok_opengv.h"
#include "../c/ok_eigen_eigsolver.h"
}

static bool same(double a, double b) { return std::memcmp(&a, &b, 8) == 0; }
struct Tally {
  const char* name; long bad = 0, tot = 0;
  explicit Tally(const char* n) : name(n) {}
  void add(bool ok) { ++tot; if (!ok) ++bad; }
  void print() const { std::printf("  %s: %ld/%ld\n", name, bad, tot); }
};

typedef opengv::absolute_pose::FrameNoncentralAbsoluteAdapter AbsAdapter;
typedef opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem<AbsAdapter> AbsProblem;

static std::mt19937_64 g_rng(2026);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(g_rng); }
static double N01() { return std::normal_distribution<double>(0.0, 1.0)(g_rng); }

// random absolute-pose problem: points in front of 1-2 cameras, noisy bearings, a fraction of outliers
struct AbsData {
  AbsAdapter ad;
  std::vector<double> bearing, point, offset, rot, sigma;
  void finish() {
    const int n = int(ad.points_.size());
    bearing.resize(3 * n); point.resize(3 * n); offset.resize(3 * n); rot.resize(9 * n); sigma.resize(n);
    for (int i = 0; i < n; ++i) {
      for (int k = 0; k < 3; ++k) { bearing[3 * i + k] = ad.bearingVectors_[i][k]; point[3 * i + k] = ad.points_[i][k]; offset[3 * i + k] = ad.camOffsets_[i][k]; }
      for (int k = 0; k < 9; ++k) rot[9 * i + k] = ad.camRotations_[i].data()[k];
      sigma[i] = ad.sigmaAngles_[i];
    }
  }
  ok_og_abs view() const { ok_og_abs a; a.n = int(ad.points_.size()); a.bearing = bearing.data(); a.point = point.data(); a.offset = offset.data(); a.rot = rot.data(); a.sigma = sigma.data(); return a; }
};
static Eigen::Matrix3d randomRotation() {
  Eigen::Vector3d ax(N01(), N01(), N01()); ax.normalize();
  return Eigen::AngleAxisd(U(-3.0, 3.0), ax).toRotationMatrix();
}
static void makeAbs(AbsData& d, int n, double outlierFrac, int ncam, bool degenerateTilt) {
  const Eigen::Matrix3d R_WS = randomRotation();
  const Eigen::Vector3d t_WS(U(-5, 5), U(-5, 5), U(-5, 5));
  std::vector<Eigen::Matrix3d> Rc(ncam); std::vector<Eigen::Vector3d> tc(ncam);
  for (int c = 0; c < ncam; ++c) {
    // a camera looking along +z of the sensor frame with a random small mounting rotation
    Rc[c] = Eigen::AngleAxisd(U(-0.3, 0.3), Eigen::Vector3d(N01(), N01(), N01()).normalized()).toRotationMatrix() * (c ? Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix() : Eigen::Matrix3d::Identity());
    tc[c] = Eigen::Vector3d(U(-0.1, 0.1) + 0.11 * c, U(-0.05, 0.05), U(-0.05, 0.05));
  }
  const double f = U(300, 800);
  for (int i = 0; i < n; ++i) {
    const int c = i % ncam;
    Eigen::Vector3d pc(U(-0.6, 0.6) * 5, U(-0.4, 0.4) * 5, U(2, 12));            // point in the camera frame
    if (degenerateTilt) pc = Eigen::Vector3d(pc[0], pc[1], 5.0 + 0.01 * pc[2]);
    const Eigen::Vector3d p_S = Rc[c] * pc + tc[c];
    const Eigen::Vector3d p_W = R_WS * p_S + t_WS;
    Eigen::Vector3d b = pc;
    b = (b + Eigen::Vector3d(N01() / f, N01() / f, 0)).normalized();
    if (U(0, 1) < outlierFrac) b = Eigen::Vector3d(N01(), N01(), std::fabs(N01()) + 0.5).normalized();
    d.ad.bearingVectors_.push_back(b);
    d.ad.points_.push_back(p_W);
    d.ad.camOffsets_.push_back(tc[c]);
    d.ad.camRotations_.push_back(Rc[c]);
    const double size = U(6, 40), sd = 0.8 * size / 12.0;
    d.ad.sigmaAngles_.push_back(std::sqrt(2.0) * sd * sd / (f * f));
  }
  d.finish();
}

typedef opengv::relative_pose::FrameRelativeAdapter RelAdapter;
typedef opengv::sac_problems::relative_pose::FrameRotationOnlySacProblem RotProblem;
typedef opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem RelProblem;

struct RelData {
  RelAdapter ad;
  std::vector<double> f1, f2, s1, s2;
  void finish() {
    const int n = int(ad.bearingVectors1_.size());
    f1.resize(3 * n); f2.resize(3 * n); s1.resize(n); s2.resize(n);
    for (int i = 0; i < n; ++i) {
      for (int k = 0; k < 3; ++k) { f1[3 * i + k] = ad.bearingVectors1_[i][k]; f2[3 * i + k] = ad.bearingVectors2_[i][k]; }
      s1[i] = ad.sigmaAngles1_[i]; s2[i] = ad.sigmaAngles2_[i];
    }
  }
  ok_og_rel view() const { ok_og_rel a; a.n = int(ad.bearingVectors1_.size()); a.f1 = f1.data(); a.f2 = f2.data(); a.s1 = s1.data(); a.s2 = s2.data(); return a; }
};
// two views: p1 = R12 * p2 + t12; mode 0 general, 1 near-pure rotation, 2 many outliers
static void makeRel(RelData& d, int n, int mode) {
  const Eigen::Matrix3d R12 = Eigen::AngleAxisd(U(-0.4, 0.4), Eigen::Vector3d(N01(), N01(), N01()).normalized()).toRotationMatrix();
  Eigen::Vector3d t12(U(-1, 1), U(-1, 1), U(-1, 1)); t12 *= (mode == 1 ? 1e-4 : U(0.1, 0.5));
  const double f = U(300, 800);
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d p1(U(-3, 3), U(-2, 2), U(2, 15));
    Eigen::Vector3d p2 = R12.transpose() * (p1 - t12);
    Eigen::Vector3d b1 = (p1.normalized() + Eigen::Vector3d(N01() / f, N01() / f, 0)).normalized();
    Eigen::Vector3d b2 = (p2.normalized() + Eigen::Vector3d(N01() / f, N01() / f, 0)).normalized();
    if (U(0, 1) < (mode == 2 ? 0.4 : 0.15)) b2 = Eigen::Vector3d(N01(), N01(), std::fabs(N01()) + 0.5).normalized();
    d.ad.bearingVectors1_.push_back(b1); d.ad.bearingVectors2_.push_back(b2);
    const double sz1 = U(6, 40), sz2 = U(6, 40), sd1 = 0.8 * sz1 / 12.0, sd2 = 0.8 * sz2 / 12.0;
    d.ad.sigmaAngles1_.push_back(std::sqrt(2.0) * sd1 * sd1 / (f * f));
    d.ad.sigmaAngles2_.push_back(std::sqrt(2.0) * sd2 * sd2 / (f * f));
  }
  d.finish();
}
static std::vector<int> distinct(int k, int n) {
  std::vector<int> v(k);
  for (int i = 0; i < k; ++i) { bool dup; do { v[i] = int(U(0, n - 1e-9)); dup = false; for (int j = 0; j < i; ++j) dup = dup || v[j] == v[i]; } while (dup); }
  return v;
}

static void testTri(Tally& t, int iters) {
  for (int it = 0; it < iters; ++it) {
    RelData d; makeRel(d, 12, it % 3);
    ok_og_rel view = d.view();
    // scores of a Stewenius model exercise triangulate2 + the reprojection; the pose is random
    double model[12];
    const Eigen::Matrix3d R = randomRotation(); const Eigen::Vector3d tt(N01(), N01(), N01());
    for (int k = 0; k < 9; ++k) model[k] = R.data()[k];
    for (int k = 0; k < 3; ++k) model[9 + k] = tt[k];
    RelProblem prob(d.ad, RelProblem::STEWENIUS);
    std::vector<int> all(view.n); for (int k = 0; k < view.n; ++k) all[k] = k;
    RelProblem::model_t m; for (int k = 0; k < 12; ++k) m.data()[k] = model[k];
    std::vector<double> s0; prob.getSelectedDistancesToModel(m, all, s0);
    std::vector<double> s1(view.n); ok_og_stewenius_scores(&view, model, s1.data());
    bool ok = true; for (int k = 0; k < view.n; ++k) ok = ok && same(s0[k], s1[k]);
    t.add(ok);
  }
}

static void testRot(Tally& tm, Tally& td, Tally& tr, int iters) {
  for (int it = 0; it < iters; ++it) {
    RelData d; makeRel(d, 10 + int(U(0, 70)), it % 3);
    ok_og_rel view = d.view();
    RotProblem prob(d.ad);
    for (int s = 0; s < 4; ++s) {
      std::vector<int> idx = distinct(2, view.n);
      RotProblem::model_t m0; const bool ok0 = prob.computeModelCoefficients(idx, m0);
      double m1[9]; const int ok1 = ok_og_rotation_model(&view, idx.data(), m1);
      bool ok = ok0 && ok1; for (int k = 0; k < 9; ++k) ok = ok && same(m0.data()[k], m1[k]);
      tm.add(ok);
      std::vector<int> all(view.n); for (int k = 0; k < view.n; ++k) all[k] = k;
      std::vector<double> s0; prob.getSelectedDistancesToModel(m0, all, s0);
      std::vector<double> s1(view.n); ok_og_rotation_scores(&view, m0.data(), s1.data());
      bool ok2 = true; for (int k = 0; k < view.n; ++k) ok2 = ok2 && same(s0[k], s1[k]);
      td.add(ok2);
    }
    opengv::sac::Ransac<RotProblem> ransac;
    ransac.sac_model_ = std::shared_ptr<RotProblem>(new RotProblem(d.ad));
    ransac.threshold_ = 9; ransac.max_iterations_ = 50;
    const bool r0 = ransac.computeModel(0);
    ok_og_result res; const int r1 = ok_og_ransac_rotation(&view, 9, 50, &res);
    bool ok = (r0 == (r1 != 0)) && ransac.iterations_ == res.iterations && int(ransac.inliers_.size()) == res.ninliers;
    for (int k = 0; ok && k < res.ninliers; ++k) ok = ok && ransac.inliers_[k] == res.inliers[k];
    if (r0 && r1) for (int k = 0; k < 9; ++k) ok = ok && same(ransac.model_coefficients_.data()[k], res.model[k]);
    tr.add(ok);
    free(res.inliers);
  }
}

static void testStew(Tally& te, Tally& tm, Tally& td, Tally& tr, int iters) {
  for (int it = 0; it < iters; ++it) {
    RelData d; makeRel(d, 10 + int(U(0, 70)), it % 3);
    ok_og_rel view = d.view();
    RelProblem prob(d.ad, RelProblem::STEWENIUS);
    for (int s = 0; s < 3; ++s) {
      std::vector<int> idx = distinct(8, view.n);
      // the minimal solver output (real parts of the 10 essentials)
      std::vector<int> sub5(idx.begin(), idx.begin() + 5);
      opengv::complexEssentials_t ce = opengv::relative_pose::fivept_stewenius(d.ad, sub5);
      double Er[10][9]; const int nE = ok_og_stewenius_essentials(&view, idx.data(), Er);
      bool oke = (nE == int(ce.size()));
      for (int c = 0; oke && c < nE; ++c) for (int r = 0; r < 3; ++r) for (int k = 0; k < 3; ++k) oke = oke && same(ce[c](r, k).real(), Er[c][3 * r + k]);
      te.add(oke);
      RelProblem::model_t m0; const bool ok0 = prob.computeModelCoefficients(idx, m0);
      double m1[12]; const int ok1 = ok_og_stewenius_model(&view, idx.data(), m1);
      bool ok = (ok0 == (ok1 != 0));
      if (ok0 && ok1) for (int k = 0; k < 12; ++k) ok = ok && same(m0.data()[k], m1[k]);
      tm.add(ok);
      if (!ok && tm.bad < 4) { std::printf("model mismatch ok0=%d ok1=%d\n", int(ok0), ok1); if (ok0 && ok1) for (int k = 0; k < 12; ++k) std::printf("  [%d] %.17g %.17g%s\n", k, m0.data()[k], m1[k], same(m0.data()[k], m1[k]) ? "" : "  <--"); }
      if (ok0) {
        std::vector<int> all(view.n); for (int k = 0; k < view.n; ++k) all[k] = k;
        std::vector<double> s0; prob.getSelectedDistancesToModel(m0, all, s0);
        std::vector<double> s1(view.n); ok_og_stewenius_scores(&view, m0.data(), s1.data());
        bool ok2 = true; for (int k = 0; k < view.n; ++k) ok2 = ok2 && same(s0[k], s1[k]);
        td.add(ok2);
      }
    }
    opengv::sac::Ransac<RelProblem> ransac;
    ransac.sac_model_ = std::shared_ptr<RelProblem>(new RelProblem(d.ad, RelProblem::STEWENIUS));
    ransac.threshold_ = 9; ransac.max_iterations_ = 50;
    const bool r0 = ransac.computeModel(0);
    ok_og_result res; const int r1 = ok_og_ransac_stewenius(&view, 9, 50, &res);
    bool ok = (r0 == (r1 != 0)) && ransac.iterations_ == res.iterations && int(ransac.inliers_.size()) == res.ninliers;
    for (int k = 0; ok && k < res.ninliers; ++k) ok = ok && ransac.inliers_[k] == res.inliers[k];
    if (r0 && r1) for (int k = 0; k < 12; ++k) ok = ok && same(ransac.model_coefficients_.data()[k], res.model[k]);
    tr.add(ok);
    free(res.inliers);
  }
}

static void testRng(Tally& t) {
  std::mt19937 eng; eng.seed(12345u);
  std::uniform_int_distribution<> dist(0, std::numeric_limits<int>::max());
  std::function<int()> gen = std::bind(dist, eng);
  ok_og_rng r; ok_og_rng_seed(&r, 12345u);
  for (int i = 0; i < 3000000; ++i) t.add(gen() == ok_og_rng_rnd(&r));
  std::mt19937 e2; e2.seed(777u); ok_og_rng r2; ok_og_rng_seed(&r2, 777u);
  for (int i = 0; i < 100000; ++i) t.add(uint32_t(e2()) == ok_og_rng_next(&r2));
}

static void testComplex(Tally& t) {
  for (int i = 0; i < 4000000; ++i) {
    auto mag = [&]() { return (i % 9 == 0) ? std::pow(10.0, U(-300, 300)) : (i % 3 == 0 ? std::pow(10.0, U(-8, 8)) : 1.0); };
    const double a = N01() * mag(), b = N01() * mag(), c = N01() * mag(), d = (i % 11 == 0 ? 0.0 : N01() * mag());
    const std::complex<double> x(a, b), y(c, d);
    const std::complex<double> q = x / y, m = x * y, s = std::sqrt(x);
    if (std::isfinite(q.real()) && std::isfinite(q.imag())) { double re, im; ok_cdiv(a, b, c, d, &re, &im); t.add(same(re, q.real()) && same(im, q.imag())); }
    if (std::isfinite(m.real()) && std::isfinite(m.imag())) { double re, im; ok_cmul(a, b, c, d, &re, &im); t.add(same(re, m.real()) && same(im, m.imag())); }
    { double re, im; ok_csqrt(a, b, &re, &im); t.add(same(re, s.real()) && same(im, s.imag())); }
  }
}

template <int N> static void testEig(Tally& t, int iters) {
  typedef Eigen::Matrix<double, N, N> M;
  for (int it = 0; it < iters; ++it) {
    M A;
    const int mode = it % 4;
    for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) A(i, j) = N01();
    if (mode == 1) { A.setZero(); for (int i = 0; i < N - 4; ++i) for (int j = 0; j < N; ++j) A(i, j) = N01(); for (int i = N - 4; i < N; ++i) A(i, i - (N - 4) + 0) = 1.0; }  // companion-like
    if (mode == 2) A *= std::pow(10.0, U(-6, 6));
    if (mode == 3) { for (int i = 0; i < N; ++i) for (int j = 0; j < i - 1; ++j) A(i, j) = 0.0; }                                    // Hessenberg
    Eigen::EigenSolver<M> eig(A, true);
    double a[N * N], er[N], ei[N], vr[N * N], vi[N * N];
    std::memcpy(a, A.data(), sizeof a);
    const int rc = (N == 8) ? ok_eigensolver8(a, er, ei, vr, vi) : ok_eigensolver10(a, er, ei, vr, vi);
    if (eig.info() != Eigen::Success || rc != 0) { t.add(eig.info() != Eigen::Success && rc != 0); continue; }
    const auto D = eig.eigenvalues();
    const Eigen::Matrix<std::complex<double>, N, N> V = eig.eigenvectors();
    bool ok = true;
    for (int i = 0; i < N; ++i) ok = ok && same(D[i].real(), er[i]) && same(D[i].imag(), ei[i]);
    for (int j = 0; j < N; ++j) for (int i = 0; i < N; ++i) ok = ok && same(V(i, j).real(), vr[i + N * j]) && same(V(i, j).imag(), vi[i + N * j]);
    t.add(ok);
  }
}

static void testGp3p(Tally& t, int iters) {
  for (int it = 0; it < iters; ++it) {
    AbsData d; makeAbs(d, 4, 0.0, 1 + it % 2, it % 7 == 0);
    Eigen::Matrix3d f, v, p;
    for (int i = 0; i < 3; ++i) { f.col(i) = d.ad.camRotations_[i] * d.ad.bearingVectors_[i]; v.col(i) = d.ad.camOffsets_[i]; p.col(i) = d.ad.points_[i]; }
    opengv::transformations_t sols;
    opengv::absolute_pose::modules::gp3p_main(f, v, p, sols);
    double sc[8][12];
    const int n = ok_og_gp3p_main(f.data(), v.data(), p.data(), sc);
    bool ok = n == int(sols.size());
    for (int s = 0; ok && s < n; ++s) for (int k = 0; k < 12; ++k) ok = ok && same(sols[s].data()[k], sc[s][k]);
    t.add(ok);
  }
}

static void testAbs(Tally& tm, Tally& td, Tally& tr, int iters) {
  for (int it = 0; it < iters; ++it) {
    AbsData d; makeAbs(d, 10 + int(U(0, 100)), (it % 3) * 0.15, 1 + it % 2, it % 13 == 0);
    ok_og_abs view = d.view();
    typedef opengv::sac::Ransac<AbsProblem> Ransac;
    Ransac ransac;
    std::shared_ptr<AbsProblem> prob(new AbsProblem(d.ad, AbsProblem::Algorithm::GP3P));
    ransac.sac_model_ = prob;
    ransac.threshold_ = 16; ransac.max_iterations_ = 50;
    // model coefficients + distances of a few random samples
    for (int s = 0; s < 4; ++s) {
      std::vector<int> idx(4);
      for (int k = 0; k < 4; ++k) { bool dup; do { idx[k] = int(U(0, view.n - 1e-9)); dup = false; for (int j = 0; j < k; ++j) dup = dup || idx[j] == idx[k]; } while (dup); }
      AbsProblem::model_t m0; const bool ok0 = prob->computeModelCoefficients(idx, m0);
      double m1[12]; const int ok1 = ok_og_abs_model(&view, idx.data(), m1);
      bool ok = (ok0 == (ok1 != 0));
      if (ok0 && ok1) for (int k = 0; k < 12; ++k) ok = ok && same(m0.data()[k], m1[k]);
      tm.add(ok);
      if (ok0) {
        std::vector<int> all(view.n); for (int k = 0; k < view.n; ++k) all[k] = k;
        std::vector<double> s0; prob->getSelectedDistancesToModel(m0, all, s0);
        std::vector<double> s1(view.n); ok_og_abs_scores(&view, m0.data(), s1.data());
        bool ok2 = true; for (int k = 0; k < view.n; ++k) { const bool e = same(s0[k], s1[k]); if (!e && td.bad < 3) std::printf("dist mismatch k=%d %.17g %.17g (size %zu/%d)\n", k, s0[k], s1[k], s0.size(), view.n); ok2 = ok2 && e; }
        td.add(ok2);
      }
    }
    const bool r0 = ransac.computeModel(0);
    ok_og_result res; const int r1 = ok_og_ransac_abs(&view, 16, 50, &res);
    bool ok = (r0 == (r1 != 0)) && ransac.iterations_ == res.iterations && int(ransac.inliers_.size()) == res.ninliers;
    for (int k = 0; ok && k < res.ninliers; ++k) ok = ok && ransac.inliers_[k] == res.inliers[k];
    if (r0 && r1) for (int k = 0; k < 12; ++k) ok = ok && same(ransac.model_coefficients_.data()[k], res.model[k]);
    tr.add(ok);
    free(res.inliers);
  }
}

int main() {
  Tally trng("rng draws"), tcx("complex ops"), te8("EigenSolver 8x8"), te10("EigenSolver 10x10"), tg("gp3p_main"),
      tam("abs computeModelCoefficients"), tad("abs distances"), tar("abs Ransac::computeModel");
  testRng(trng);
  testComplex(tcx);
  testEig<8>(te8, 100000);
  testEig<10>(te10, 100000);
  testGp3p(tg, 20000);
  testAbs(tam, tad, tar, 3000);
  Tally ttri("triangulate2 (Stewenius scores)"), trm("rotation-only model"), trd("rotation-only distances"), trr("rotation-only Ransac"),
      tse("Stewenius essentials"), tsm("Stewenius model"), tsd("Stewenius distances"), tsr("Stewenius Ransac");
  testTri(ttri, 3000);
  testRot(trm, trd, trr, 3000);
  testStew(tse, tsm, tsd, tsr, 1500);
  const Tally* all[] = {&trng, &tcx, &te8, &te10, &tg, &tam, &tad, &tar, &ttri, &trm, &trd, &trr, &tse, &tsm, &tsd, &tsr};
  long bad = 0, tot = 0;
  for (const Tally* x : all) { x->print(); bad += x->bad; tot += x->tot; }
  std::printf("okvis_opengv_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
