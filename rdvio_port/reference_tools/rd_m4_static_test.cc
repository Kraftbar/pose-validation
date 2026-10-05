// OK_PORT_TEST_C: rd_static.c ok_blas.c
// Random-case, tolerance-0 comparison of the statically-sized kernels of SchurEliminator<2,3,3> (rdvio_port/c/rd_static.c) against the REAL Ceres
// 2.2.0 header-only kernels (small_blas.h with every dimension static -> the Eigen path, invert_psd_matrix.h InvertPSDMatrix<3>) and the Eigen 3.4.0
// product `InvertPSDMatrix<3>(..) * y_block`, compiled with the reference flags (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math, SSE2).
// Values include +-0 and tiny / huge magnitudes so that signed zeros and roundings matter.
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include "ceres/internal/eigen.h"
#include "ceres/invert_psd_matrix.h"
#include "ceres/small_blas.h"
extern "C" {
#include "../c/rd_static.h"
#include "../../okvis_port/c/ok_blas.h"
}
namespace {
struct Sec { std::string name; long bad = 0, tot = 0; };
std::vector<Sec> g_secs;
Sec& sec(const std::string& n) { for (auto& s : g_secs) if (s.name == n) return s; g_secs.push_back({n}); return g_secs.back(); }
std::mt19937_64 rng(20261005);
double rnd() {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const int k = int(rng() % 16);
  if (k == 0) return 0.0;
  if (k == 1) return -0.0;
  if (k == 2) return u(rng) * 1e-9;
  if (k == 3) return u(rng) * 1e9;
  return u(rng);
}
void cmpd(Sec& s, const double* a, const double* b, long n, const char* what) {
  for (long i = 0; i < n; ++i) {
    s.tot++;
    if (std::memcmp(&a[i], &b[i], 8) != 0) {
      if (s.bad < 5) std::printf("    %s[%ld]: C %.17g real %.17g\n", what, i, a[i], b[i]);
      s.bad++;
    }
  }
}
}  // namespace
using namespace ceres::internal;

int main() {
  const int N = 20000;
  for (int t = 0; t < N; ++t) {
    double A[9], B[9], C0[16], b[3], c0[3];
    for (auto& v : A) v = rnd();
    for (auto& v : B) v = rnd();
    for (auto& v : C0) v = rnd();
    for (auto& v : b) v = rnd();
    for (auto& v : c0) v = rnd();
    const int cs = (t % 3 == 0) ? 3 : 4;  // the lhs cells are 3 wide, but also exercise a wider destination
    {  // MatrixTransposeMatrixMultiply<2,3,2,3,1>
      double Cr[16], Cc[16];
      std::memcpy(Cr, C0, sizeof Cr); std::memcpy(Cc, C0, sizeof Cc);
      MatrixTransposeMatrixMultiply<2, 3, 2, 3, 1>(A, 2, 3, B, 2, 3, Cr, 0, 0, 3, cs);
      rd_st_mtm_2_3(A, B, Cc, cs, 1);
      cmpd(sec("mtm<2,3,2,3,+1>"), Cc, Cr, 3 * cs, "C");
    }
    {  // MatrixTransposeVectorMultiply<2,3,1>
      double cr[3], cc[3];
      std::memcpy(cr, c0, sizeof cr); std::memcpy(cc, c0, sizeof cc);
      MatrixTransposeVectorMultiply<2, 3, 1>(A, 2, 3, b, cr);
      ok_mtv(A, 2, 3, b, cc, 1);
      cmpd(sec("mtv<2,3,+1>"), cc, cr, 3, "c");
    }
    for (int op : {-1, 0, 1}) {  // MatrixVectorMultiply<2,3,op>
      double cr[3], cc[3];
      std::memcpy(cr, c0, sizeof cr); std::memcpy(cc, c0, sizeof cc);
      if (op == 1) MatrixVectorMultiply<2, 3, 1>(A, 2, 3, b, cr);
      else if (op == -1) MatrixVectorMultiply<2, 3, -1>(A, 2, 3, b, cr);
      else MatrixVectorMultiply<2, 3, 0>(A, 2, 3, b, cr);
      ok_mv(A, 2, 3, b, cc, op);
      cmpd(sec("mv<2,3,op>"), cc, cr, 2, "c");
    }
    for (int op : {-1, 0, 1}) {  // MatrixVectorMultiply<3,3,op>
      double cr[3], cc[3];
      std::memcpy(cr, c0, sizeof cr); std::memcpy(cc, c0, sizeof cc);
      if (op == 1) MatrixVectorMultiply<3, 3, 1>(A, 3, 3, b, cr);
      else if (op == -1) MatrixVectorMultiply<3, 3, -1>(A, 3, 3, b, cr);
      else MatrixVectorMultiply<3, 3, 0>(A, 3, 3, b, cr);
      ok_mv(A, 3, 3, b, cc, op);
      cmpd(sec("mv<3,3,op>"), cc, cr, 3, "c");
    }
    for (int op : {-1, 0, 1}) {  // MatrixTransposeMatrixMultiply<3,3,3,3,op>, MatrixMatrixMultiply<3,3,3,3,op>
      double Cr[16], Cc[16];
      std::memcpy(Cr, C0, sizeof Cr); std::memcpy(Cc, C0, sizeof Cc);
      if (op == 1) MatrixTransposeMatrixMultiply<3, 3, 3, 3, 1>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      else if (op == -1) MatrixTransposeMatrixMultiply<3, 3, 3, 3, -1>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      else MatrixTransposeMatrixMultiply<3, 3, 3, 3, 0>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      rd_st_mtm_3_3(A, B, Cc, cs, op);
      cmpd(sec("mtm<3,3,3,3,op>"), Cc, Cr, 3 * cs, "C");
      std::memcpy(Cr, C0, sizeof Cr); std::memcpy(Cc, C0, sizeof Cc);
      if (op == 1) MatrixMatrixMultiply<3, 3, 3, 3, 1>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      else if (op == -1) MatrixMatrixMultiply<3, 3, 3, 3, -1>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      else MatrixMatrixMultiply<3, 3, 3, 3, 0>(A, 3, 3, B, 3, 3, Cr, 0, 0, 3, cs);
      rd_st_mmm_3_3(A, B, Cc, cs, op);
      cmpd(sec("mmm<3,3,3,3,op>"), Cc, Cr, 3 * cs, "C");
    }
    {  // InvertPSDMatrix<3> on E^T E-like SPD matrices (sums of 2x3 outer products + optional diagonal), and inverse * y_block
      double E[6 * 4], ete[9] = {0};
      const int nrow = 1 + int(rng() % 4);
      for (int r = 0; r < nrow; ++r) {
        for (int k = 0; k < 6; ++k) E[k] = std::exp(std::uniform_real_distribution<double>(-4, 4)(rng)) * (rng() & 1 ? 1 : -1) * (rng() % 5 == 0 ? 0.0 : 1.0);
        rd_st_mtm_2_3(E, E, ete, 3, 1);
      }
      if (t % 4 == 0) for (int k = 0; k < 3; ++k) ete[k * 4] += std::exp(std::uniform_real_distribution<double>(-6, 3)(rng));
      for (int i = 0; i < 3; ++i) for (int j = 0; j < i; ++j) ete[i * 3 + j] = ete[j * 3 + i];  // E^T E sums are exactly symmetric; m.inverse() reads both triangles
      ceres::EigenTypes<3, 3>::Matrix m = ceres::EigenTypes<3, 3>::ConstMatrixRef(ete);
      const bool spd = [&] { Eigen::LLT<Eigen::Matrix3d> l(Eigen::Matrix3d(m.selfadjointView<Eigen::Upper>())); return l.info() == Eigen::Success; }();
      if (spd) {
        ceres::EigenTypes<3, 3>::Matrix inv = InvertPSDMatrix<3>(true, m);
        double inv_c[9];
        rd_st_invert_psd3(ete, inv_c);
        cmpd(sec("InvertPSDMatrix<3>"), inv_c, inv.data(), 9, "inv");
        double y[3] = {rnd(), rnd(), rnd()}, yr[3], yc[3];
        std::memcpy(yr, y, sizeof y);
        ceres::EigenTypes<3>::VectorRef y_block(yr, 3);
        y_block = InvertPSDMatrix<3>(true, m) * y_block;
        rd_st_inv_times_y(inv_c, y, yc);
        cmpd(sec("inverse * y_block"), yc, yr, 3, "y");
      }
    }
  }
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("  %s: %ld/%ld\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("rd_m4_static_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
