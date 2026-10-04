// OK_PORT_TEST_C: ok_dense.c ok_blas.c
// OK_PORT_TEST_LIBS: ceres glog
// Random-case, tolerance-0 comparison of the module-4 dense kernels (okvis_port/c/ok_dense.c, ok_blas.c) against the
// REAL Eigen 3.4.0 statements Ceres 2.2.0 executes and the real Ceres header-only kernels (small_blas.h naive paths,
// InvertPSDMatrix<Eigen::Dynamic>), compiled with the reference flags (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math,
// SSE2): dynamic-size redux (squaredNorm / norm / dot / model-cost Dot / lpNorm<Infinity>), colwise squaredNorm with
// the destination-address peeling, column- and row-major GEMV, LLT<Ref<MatrixXd>, Lower> (unblocked and blocked
// sizes incl. non-multiples of 4 / 8) with its vector solve, failing LLT, InvertPSDMatrix<Dynamic> on 1..12,
// and MatrixMatrixMultiply / MatrixTransposeMatrixMultiply / MatrixVectorMultiply / MatrixTransposeVectorMultiply
// with every kOperation. Values include +-0 and tiny / huge magnitudes so that signed zeros and roundings matter.
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "ceres/internal/eigen.h"
#include "ceres/invert_psd_matrix.h"
#include "ceres/small_blas.h"
extern "C" {
#include "../c/ok_blas.h"
#include "../c/ok_dense.h"
}

namespace {
struct Sec { std::string name; long bad = 0, tot = 0; };
std::vector<Sec> g_secs;
Sec& sec(const std::string& n) { for (auto& s : g_secs) if (s.name == n) return s; g_secs.push_back({n}); return g_secs.back(); }
std::mt19937_64 rng(20261003);
double rnd() {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const int k = int(rng() % 16);
  if (k == 0) return 0.0;
  if (k == 1) return -0.0;
  if (k == 2) return u(rng) * 1e-9;
  if (k == 3) return u(rng) * 1e9;
  return u(rng);
}
void cmpd(Sec& s, const double* a, const double* b, long n, const char* what, long idx0 = 0) {
  for (long i = 0; i < n; ++i) {
    s.tot++;
    if (std::memcmp(&a[i], &b[i], 8) != 0) {
      if (s.bad < 5) std::printf("    %s[%ld]: C %.17g real %.17g\n", what, idx0 + i, a[i], b[i]);
      s.bad++;
    }
  }
}
using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void test_redux() {
  Sec& s = sec("redux");
  for (int it = 0; it < 4000; ++it) {
    const int n = it < 80 ? it : int(rng() % 300) + 1;
    Eigen::VectorXd a(n + 2), b(n + 2), m(n + 2), r(n + 2);
    for (int i = 0; i < n + 2; ++i) { a[i] = rnd(); b[i] = rnd(); m[i] = rnd(); r[i] = rnd(); }
    if (n == 0) continue;
    // squaredNorm / norm of a Vector, of an unaligned Map (offset 1) and of a difference expression
    double c, e;
    c = ok_dyn_sqnorm(a.data(), n); e = a.head(n).squaredNorm(); cmpd(s, &c, &e, 1, "sqnorm");
    c = ok_dyn_sqnorm(a.data() + 1, n); e = Eigen::Map<Eigen::VectorXd>(a.data() + 1, n).squaredNorm(); cmpd(s, &c, &e, 1, "sqnorm.map1");
    c = ok_dyn_norm(a.data(), n); e = Eigen::Map<const Eigen::VectorXd>(a.data(), n).norm(); cmpd(s, &c, &e, 1, "norm");
    c = ok_dyn_dot(a.data(), b.data(), n); e = a.head(n).dot(b.head(n)); cmpd(s, &c, &e, 1, "dot");
    c = ok_dyn_dot(a.data() + 1, b.data() + 1, n); e = Eigen::Map<const Eigen::VectorXd>(a.data() + 1, n).dot(Eigen::Map<const Eigen::VectorXd>(b.data() + 1, n)); cmpd(s, &c, &e, 1, "dot.map1");
    {  // Ceres: model_cost_change_ = -Dot(model_residuals_, residuals_ + model_residuals_ / 2.0, context, 1)
      Eigen::VectorXd mm = m.head(n), rr = r.head(n);
      double dots = 0.;
      const auto& x_block = mm.segment(0, n);
      const auto& y_block = (rr + mm / 2.0).segment(0, n);
      dots += x_block.dot(y_block);
      e = -std::accumulate(&dots, &dots + 1, 0.);
      c = -(0.0 + (0.0 + ok_dyn_dot_model(mm.data(), rr.data(), n)));
      cmpd(s, &c, &e, 1, "dot.model");
    }
    {
      Eigen::VectorXd aa = a.head(n), bb = b.head(n);
      c = ok_dyn_norm_diff(aa.data(), bb.data(), n); e = (aa - bb).norm(); cmpd(s, &c, &e, 1, "norm.diff");
      c = ok_dyn_maxabs_diff(aa.data(), bb.data(), n); e = (aa - bb).lpNorm<Eigen::Infinity>(); cmpd(s, &c, &e, 1, "maxabs.diff");
    }
  }
}

void test_colwise() {
  Sec& s = sec("colwise_sqnorm");
  for (int it = 0; it < 6000; ++it) {
    const int rows = 1 + int(rng() % 20), cols = 1 + int(rng() % 12), pos = int(rng() % 13);
    std::vector<double> m((size_t)rows * cols);
    for (auto& v : m) v = rnd();
    Eigen::VectorXd x = Eigen::VectorXd::Zero(32), xc = Eigen::VectorXd::Zero(32);
    for (int i = 0; i < 32; ++i) { x[i] = rnd(); xc[i] = x[i]; }
    Eigen::Map<Eigen::VectorXd>(x.data() + pos, cols) += Eigen::Map<const RowMajorMatrix>(m.data(), rows, cols).colwise().squaredNorm();
    ok_colwise_sqnorm_add(m.data(), rows, cols, pos & 1, xc.data() + pos);
    cmpd(s, xc.data(), x.data(), 32, "colwise");
  }
}

void test_gemv() {
  Sec& s = sec("gemv");
  for (int it = 0; it < 4000; ++it) {
    const int rows = 1 + int(rng() % 40), cols = 1 + int(rng() % 40), lda = rows + int(rng() % 3);
    std::vector<double> A((size_t)lda * cols), x(cols), y(rows), yc(rows);
    for (auto& v : A) v = rnd();
    for (auto& v : x) v = rnd();
    for (int i = 0; i < rows; ++i) { y[i] = rnd(); yc[i] = y[i]; }
    const double alpha = (it & 1) ? -1.0 : 1.0;
    {  // column-major GEMV: Map<VectorXd> -= / += Map<const MatrixXd, OuterStride> * Map<const VectorXd>
      Eigen::Map<const Eigen::MatrixXd, 0, Eigen::OuterStride<>> Am(A.data(), rows, cols, Eigen::OuterStride<>(lda));
      Eigen::Map<Eigen::VectorXd> ym(y.data(), rows);
      if (alpha < 0) ym.noalias() -= Am * Eigen::Map<const Eigen::VectorXd>(x.data(), cols);
      else ym.noalias() += Am * Eigen::Map<const Eigen::VectorXd>(x.data(), cols);
      ok_gemv_col(rows, cols, A.data(), lda, x.data(), 1, yc.data(), alpha);
      cmpd(s, yc.data(), y.data(), rows, "gemv.col");
    }
    for (int i = 0; i < rows; ++i) { y[i] = rnd(); yc[i] = y[i]; }
    {  // row-major GEMV
      Eigen::Map<const RowMajorMatrix, 0, Eigen::OuterStride<>> Am(A.data(), rows, cols, Eigen::OuterStride<>(cols + 0));
      std::vector<double> Ar((size_t)rows * cols);
      for (auto& v : Ar) v = rnd();
      Eigen::Map<const RowMajorMatrix> Arm(Ar.data(), rows, cols);
      Eigen::Map<Eigen::VectorXd> ym(y.data(), rows);
      if (alpha < 0) ym.noalias() -= Arm * Eigen::Map<const Eigen::VectorXd>(x.data(), cols);
      else ym.noalias() += Arm * Eigen::Map<const Eigen::VectorXd>(x.data(), cols);
      ok_gemv_row(rows, cols, Ar.data(), cols, x.data(), yc.data(), alpha);
      { const long b0 = s.bad; cmpd(s, yc.data(), y.data(), rows, "gemv.row"); if (s.bad != b0 && s.bad < 12) std::printf("      (rows %d cols %d alpha %g)\n", rows, cols, alpha); }
      (void)Am;
    }
  }
}

Eigen::MatrixXd random_spd(int n, bool pd = true) {
  Eigen::MatrixXd B(n, n + 3);
  for (int i = 0; i < n; ++i) for (int j = 0; j < n + 3; ++j) B(i, j) = rnd();
  Eigen::MatrixXd A = B * B.transpose();
  for (int i = 0; i < n; ++i) A(i, i) += pd ? 0.5 + std::fabs(rnd()) : -std::fabs(A(i, i)) * (rng() % 4 == 0 ? 1.0 : 0.0);
  return A;
}

void test_llt() {
  Sec& s = sec("llt_lower");
  Sec& s2 = sec("llt_solve");
  Sec& s3 = sec("llt_fail");
  const int sizes[] = {1, 2, 3, 5, 6, 8, 9, 15, 16, 31, 32, 33, 40, 47, 63, 64, 100, 120, 127, 128, 129, 150, 200, 231, 233, 256, 300};
  for (int rep = 0; rep < 2; ++rep) {
    for (int n : sizes) {
      Eigen::MatrixXd A = random_spd(n);
      // Ceres: a row-major upper-filled buffer viewed as a column-major Map -> lower triangle
      std::vector<double> buf((size_t)n * n), bufc((size_t)n * n);
      for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) buf[i * n + j] = (i <= j) ? A(j, i) : rnd();  // row-major upper = colmajor lower
      bufc = buf;
      Eigen::Map<Eigen::MatrixXd> m(buf.data(), n, n);
      Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>, Eigen::Lower> llt(m);
      const int ret = ok_llt_lower(n, bufc.data(), n);
      s.tot++;
      if ((llt.info() == Eigen::Success) != (ret < 0)) { s.bad++; std::printf("    llt n=%d: C ret %d, Eigen info %d\n", n, ret, int(llt.info())); }
      // only the lower triangle (column-major) is written by Eigen (the strict upper is untouched: compare all)
      cmpd(s, bufc.data(), buf.data(), (long)n * n, "llt.factor");
      Eigen::VectorXd b(n), x(n), xc(n);
      for (int i = 0; i < n; ++i) b[i] = rnd();
      Eigen::Map<Eigen::VectorXd>(x.data(), n) = llt.solve(Eigen::Map<const Eigen::VectorXd>(b.data(), n));
      ok_llt_lower_solve(n, bufc.data(), n, b.data(), xc.data());
      cmpd(s2, xc.data(), x.data(), n, "llt.solve");
    }
  }
  for (int it = 0; it < 200; ++it) {  // failing factorisations
    const int n = 1 + int(rng() % 40);
    Eigen::MatrixXd A = random_spd(n, false);
    std::vector<double> buf((size_t)n * n), bufc((size_t)n * n);
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) buf[i * n + j] = (i <= j) ? A(j, i) : rnd();
    bufc = buf;
    Eigen::Map<Eigen::MatrixXd> m(buf.data(), n, n);
    Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>, Eigen::Lower> llt(m);
    const int ret = ok_llt_lower(n, bufc.data(), n);
    s3.tot++;
    if ((llt.info() == Eigen::Success) != (ret < 0)) s3.bad++;
    cmpd(s3, bufc.data(), buf.data(), (long)n * n, "llt.fail.factor");
  }
}

void test_invert_psd() {
  Sec& s = sec("invert_psd_dyn");
  for (int it = 0; it < 3000; ++it) {
    const int n = 1 + int(rng() % 12);
    Eigen::MatrixXd A = random_spd(n);
    ceres::Matrix m(n, n);  // row-major (EigenTypes<Dynamic,Dynamic>::Matrix)
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) m(i, j) = (i <= j) ? A(i, j) : rnd();  // lower never read
    const ceres::Matrix inv = ceres::internal::InvertPSDMatrix<Eigen::Dynamic>(true, m);
    std::vector<double> mc((size_t)n * n), out((size_t)n * n);
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) mc[i * n + j] = m(i, j);
    ok_invert_psd_dyn(n, mc.data(), out.data());
    cmpd(s, out.data(), inv.data(), (long)n * n, "invert_psd");
  }
}

void test_small_blas() {
  Sec& s = sec("small_blas");
  using namespace ceres::internal;
  for (int it = 0; it < 6000; ++it) {
    const int ra = 1 + int(rng() % 9), ca = 1 + int(rng() % 9), cb = 1 + int(rng() % 9), depth = 1 + int(rng() % 16);
    const int op = int(rng() % 3) - 1;
    const int sr = int(rng() % 3), sc = int(rng() % 3), stride = 20;
    std::vector<double> A((size_t)ra * ca), B((size_t)depth * cb), C((size_t)stride * stride), Cc;
    for (auto& v : A) v = rnd();
    for (auto& v : B) v = rnd();
    for (auto& v : C) v = rnd();
    // MMM: A (ra x ca) * B (ca x cb)
    {
      std::vector<double> Bm((size_t)ca * cb);
      for (auto& v : Bm) v = rnd();
      Cc = C;
      if (op > 0) MatrixMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 1>(A.data(), ra, ca, Bm.data(), ca, cb, C.data(), sr, sc, stride, stride);
      else if (op < 0) MatrixMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, -1>(A.data(), ra, ca, Bm.data(), ca, cb, C.data(), sr, sc, stride, stride);
      else MatrixMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 0>(A.data(), ra, ca, Bm.data(), ca, cb, C.data(), sr, sc, stride, stride);
      ok_mmm(A.data(), ra, ca, Bm.data(), ca, cb, Cc.data(), sr, sc, stride, op);
      cmpd(s, Cc.data(), C.data(), stride * stride, "mmm");
    }
    // MTM: A^T (A is depth x ca) * B (depth x cb)
    {
      std::vector<double> At((size_t)depth * ca);
      for (auto& v : At) v = rnd();
      Cc = C;
      if (op > 0) MatrixTransposeMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 1>(At.data(), depth, ca, B.data(), depth, cb, C.data(), sr, sc, stride, stride);
      else if (op < 0) MatrixTransposeMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, -1>(At.data(), depth, ca, B.data(), depth, cb, C.data(), sr, sc, stride, stride);
      else MatrixTransposeMatrixMultiply<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 0>(At.data(), depth, ca, B.data(), depth, cb, C.data(), sr, sc, stride, stride);
      ok_mtm(At.data(), depth, ca, B.data(), depth, cb, Cc.data(), sr, sc, stride, op);
      cmpd(s, Cc.data(), C.data(), stride * stride, "mtm");
    }
    // MV / MTV
    {
      std::vector<double> b(ca), bt(ra), c(ra), cc(ra), d(ca), dc(ca);
      for (auto& v : b) v = rnd();
      for (auto& v : bt) v = rnd();
      for (int i = 0; i < ra; ++i) { c[i] = rnd(); cc[i] = c[i]; }
      for (int i = 0; i < ca; ++i) { d[i] = rnd(); dc[i] = d[i]; }
      if (op > 0) { MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 1>(A.data(), ra, ca, b.data(), c.data()); MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 1>(A.data(), ra, ca, bt.data(), d.data()); }
      else if (op < 0) { MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, -1>(A.data(), ra, ca, b.data(), c.data()); MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, -1>(A.data(), ra, ca, bt.data(), d.data()); }
      else { MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 0>(A.data(), ra, ca, b.data(), c.data()); MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 0>(A.data(), ra, ca, bt.data(), d.data()); }
      ok_mv(A.data(), ra, ca, b.data(), cc.data(), op);
      ok_mtv(A.data(), ra, ca, bt.data(), dc.data(), op);
      cmpd(s, cc.data(), c.data(), ra, "mv");
      cmpd(s, dc.data(), d.data(), ca, "mtv");
    }
  }
}
}  // namespace

int main() {
  g_secs.reserve(64);
  test_redux();
  test_colwise();
  test_gemv();
  test_llt();
  test_invert_psd();
  test_small_blas();
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("  %-28s %ld/%ld\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("okvis_solve_dense_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
