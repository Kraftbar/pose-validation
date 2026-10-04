// Bit-exactness test of ok_gemm (okvis_port/c/ok_eigen.c) against real Eigen 3.4.0 for the product
// shapes used in OKVIS2's ImuError (statement-for-statement, incl. assignment vs construction).
// Build/run: tools/check_okvis_port.py --eigen-tests  (g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math)
#include <Eigen/Dense>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
extern "C" {
#include "../c/ok_eigen.h"
}
using namespace Eigen;
typedef Matrix<double, 15, 15> M15;
static std::mt19937_64 rng(12345);
static std::normal_distribution<double> nd(0.0, 1.0);
template <class M> void rnd(M& m) { for (int j = 0; j < m.cols(); ++j) for (int i = 0; i < m.rows(); ++i) m(i, j) = nd(rng) * std::exp(nd(rng)); }
template <class A, class B> long diff(const A& a, const B& b) {
  long n = 0;
  for (int j = 0; j < a.cols(); ++j) for (int i = 0; i < a.rows(); ++i) { double x = a(i, j), y = b(i, j); if (std::memcmp(&x, &y, 8) != 0) ++n; }
  return n;
}
// model of  dst = P*Q*P.transpose()  (assignment): inner product plain, outer product on the transposed problem
static M15 pqpt(const M15& P, const M15& Q) {
  M15 t, tT, r, o;
  ok_gemm(15, 15, 15, P.data(), Q.data(), t.data());
  tT = t.transpose();
  ok_gemm(15, 15, 15, P.data(), tT.data(), r.data());
  o = r.transpose();
  return o;
}
__attribute__((noinline)) void st_S1(std::vector<M15, aligned_allocator<M15> >& v, const M15& F, const M15& K) { v.at(0) = F * v.at(0) * F.transpose() + K; }
__attribute__((noinline)) void st_S2(M15& info, const M15& s) { info = s.transpose() * s; }
__attribute__((noinline)) void st_S3(M15& P, const M15& F) { P = F * P * F.transpose(); }
__attribute__((noinline)) void st_S4(M15& P, const M15& T, const M15& Pd) { P = T * Pd * T.transpose(); }
__attribute__((noinline)) void st_j1(const M15& S, const M15& F, double* out) { Map<Matrix<double, 15, 9, RowMajor> > J(out); J = S * F.block<15, 9>(0, 6); }
int main() {
  const int N = 20000;
  const char* names[] = {"A*B", "A*B^T", "A^T*B", "construct A*Blk15x6", "construct A*Blk15x9",
                         "S1 v.at(0)=F*P*F^T+K", "S2 info=s^T*s", "S3 P=F*P*F^T", "S4 P=T*Pd*T^T", "S5 Map<RowMajor15x9>=S*Blk"};
  long bad[10] = {0}, tot[10] = {0};
  for (int it = 0; it < N; ++it) {
    M15 A, B, C, K, T; rnd(A); rnd(B); rnd(C); rnd(K); rnd(T);
    { M15 e = A * B; M15 m; ok_gemm(15, 15, 15, A.data(), B.data(), m.data()); bad[0] += diff(e, m); tot[0] += 225; }
    { M15 e = A * B.transpose(); M15 Bt = B.transpose(); M15 m; ok_gemm(15, 15, 15, A.data(), Bt.data(), m.data()); bad[1] += diff(e, m); tot[1] += 225; }
    { M15 e = A.transpose() * B; M15 At = A.transpose(); M15 m; ok_gemm(15, 15, 15, At.data(), B.data(), m.data()); bad[2] += diff(e, m); tot[2] += 225; }
    { Matrix<double, 15, 6> e = A * B.block<15, 6>(0, 0); Matrix<double, 15, 6> m; ok_gemm(15, 6, 15, A.data(), B.data(), m.data()); bad[3] += diff(e, m); tot[3] += 90; }
    { Matrix<double, 15, 9> e = A * B.block<15, 9>(0, 6); Matrix<double, 15, 9> m; ok_gemm(15, 9, 15, A.data(), B.data() + 15 * 6, m.data()); bad[4] += diff(e, m); tot[4] += 135; }
    { std::vector<M15, aligned_allocator<M15> > v(4); v.at(0) = B; st_S1(v, A, K); M15 m = pqpt(A, B); for (int i = 0; i < 225; ++i) m.data()[i] = m.data()[i] + K.data()[i]; bad[5] += diff(v[0], m); tot[5] += 225; }
    { M15 info; st_S2(info, A); M15 At = A.transpose(); M15 m; ok_gemm(15, 15, 15, At.data(), A.data(), m.data()); bad[6] += diff(info, m); tot[6] += 225; }
    { M15 P = B; st_S3(P, A); bad[7] += diff(P, pqpt(A, B)); tot[7] += 225; }
    { M15 P; st_S4(P, T, B); bad[8] += diff(P, pqpt(T, B)); tot[8] += 225; }
    { double out[135]; st_j1(A, B, out); Matrix<double, 15, 9> m; ok_gemm(15, 9, 15, A.data(), B.data() + 15 * 6, m.data());
      Map<Matrix<double, 15, 9, RowMajor> > e(out); bad[9] += diff(e, m); tot[9] += 135; }
  }
  int fail = 0;
  for (int k = 0; k < 10; ++k) { std::printf("%-30s %ld/%ld\n", names[k], bad[k], tot[k]); fail |= bad[k] != 0; }
  return fail;
}
