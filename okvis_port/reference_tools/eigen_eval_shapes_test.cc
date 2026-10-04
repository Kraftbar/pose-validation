// Eigen shapes used by ImuError::EvaluateWithMinimalJacobians vs the C models (bit-exact check).
#include <Eigen/Dense>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/ok_eigen.h"
}
using namespace Eigen;
typedef Matrix<double, 15, 15> M15;
static std::mt19937_64 rng(4242);
static std::normal_distribution<double> nd(0.0, 1.0);
template <class M> void rnd(M& m) { for (int j = 0; j < m.cols(); ++j) for (int i = 0; i < m.rows(); ++i) m(i, j) = nd(rng) * std::exp(nd(rng)); }
static long cnt(const double* a, const double* b, int n) { long k = 0; for (int i = 0; i < n; ++i) if (std::memcmp(a + i, b + i, 8)) ++k; return k; }

__attribute__((noinline)) void gemv15(const M15& S, const Matrix<double, 15, 1>& e, double* out) {
  Eigen::Map<Eigen::Matrix<double, 15, 1> > w(out);
  w = S * e;
}
__attribute__((noinline)) void blk36(const M15& F, const Matrix<double, 6, 1>& d, double* out) {
  Matrix<double, 3, 1> r; r = F.block<3, 6>(0, 9) * d; std::memcpy(out, r.data(), 24);
}
__attribute__((noinline)) void blk36b(const M15& F, const Matrix<double, 6, 1>& d, double* out) {
  Matrix<double, 3, 1> r; r = F.block<3, 6>(6, 9) * d; std::memcpy(out, r.data(), 24);
}
__attribute__((noinline)) void j0(const M15& S, const M15& F, const Matrix<double, 6, 7, RowMajor>& Jl, double* out) {
  Matrix<double, 15, 6> Jm = S * F.block<15, 6>(0, 0);
  Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J(out);
  J = Jm * Jl;
}
__attribute__((noinline)) void j1(const M15& S, const M15& F, double* out) {
  Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J(out);
  J = S * F.block<15, 9>(0, 6);
}
__attribute__((noinline)) void jm(const M15& S, const M15& F, double* out) {
  Matrix<double, 15, 6> Jm = S * F.block<15, 6>(0, 0); std::memcpy(out, Jm.data(), 90 * 8);
}
int main() {
  long b[6] = {0}, t[6] = {0};
  for (int it = 0; it < 20000; ++it) {
    M15 S, F; rnd(S); rnd(F); Matrix<double, 15, 1> e; rnd(e); Matrix<double, 6, 1> d; rnd(d);
    Matrix<double, 6, 7, RowMajor> Jl; rnd(Jl);
    { double out[15], m[15]; gemv15(S, e, out);
      for (int i = 0; i < 15; ++i) { double acc = 0.0; for (int j = 0; j < 15; ++j) acc = acc + S(i, j) * e[j]; m[i] = 0.0 + 1.0 * acc; }
      b[0] += cnt(out, m, 15); t[0] += 15; }
    { double out[3]; blk36(F, d, out); double m[3];
      for (int i = 0; i < 2; ++i) { double s = F(i, 9) * d[0]; for (int k = 1; k < 6; ++k) s = s + F(i, 9 + k) * d[k]; m[i] = s; }
      m[2] = (F(2, 9) * d[0] + (F(2, 10) * d[1] + F(2, 11) * d[2])) + (F(2, 12) * d[3] + (F(2, 13) * d[4] + F(2, 14) * d[5]));
      b[1] += cnt(out, m, 3); t[1] += 3; }
    { double out[3]; blk36b(F, d, out); double m[3];
      for (int i = 0; i < 2; ++i) { double s = F(6 + i, 9) * d[0]; for (int k = 1; k < 6; ++k) s = s + F(6 + i, 9 + k) * d[k]; m[i] = s; }
      m[2] = (F(8, 9) * d[0] + (F(8, 10) * d[1] + F(8, 11) * d[2])) + (F(8, 12) * d[3] + (F(8, 13) * d[4] + F(8, 14) * d[5]));
      b[2] += cnt(out, m, 3); t[2] += 3; }
    { double out[105], mm[90], m[105]; j0(S, F, Jl, out); ok_gemm(15, 6, 15, S.data(), F.data(), mm);
      // J = Jm*Jl evaluates into a col-major 15x7 temp, SliceVectorized: even columns -> packets over rows 0..13 and a
      // scalar row 14, odd columns -> scalar row 0 then packets over rows 1..14. Scalar coefficient: pairwise
      // (a0+(a1+a2))+(a3+(a4+a5)); packet rows: left fold.
      for (int j = 0; j < 7; ++j) for (int i = 0; i < 15; ++i) {
        const bool scalar_row = (j & 1) ? (i == 0) : (i == 14);
        double s;
        if (scalar_row) { double a[6]; for (int k = 0; k < 6; ++k) a[k] = mm[i + 15 * k] * Jl(k, j); s = (a[0] + (a[1] + a[2])) + (a[3] + (a[4] + a[5])); }
        else { s = mm[i] * Jl(0, j); for (int k = 1; k < 6; ++k) s = s + mm[i + 15 * k] * Jl(k, j); }
        m[i * 7 + j] = s; }
      b[3] += cnt(out, m, 105); t[3] += 105; }
    { double out[135], mm[135], m[135]; j1(S, F, out); ok_gemm(15, 9, 15, S.data(), F.data() + 15 * 6, mm);
      for (int j = 0; j < 9; ++j) for (int i = 0; i < 15; ++i) m[i * 9 + j] = mm[i + 15 * j];
      b[4] += cnt(out, m, 135); t[4] += 135; }
    { double out[90], mm[90]; jm(S, F, out); ok_gemm(15, 6, 15, S.data(), F.data(), mm); b[5] += cnt(out, mm, 90); t[5] += 90; }
  }
  const char* nm[] = {"gemv15", "blk3x6(0,9)*v6", "blk3x6(6,9)*v6", "J0=Jm*Jlift(rowmajor)", "J1 rowmajor gemm", "Jm 15x6"};
  int fail = 0;
  for (int k = 0; k < 6; ++k) { std::printf("%-24s %ld/%ld\n", nm[k], b[k], t[k]); fail |= b[k] != 0; }
  return fail;
}
