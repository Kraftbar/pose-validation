// ok_selfadjoint_eig vs Eigen::SelfAdjointEigenSolver, bit-exact: Matrix<double,15,15> (module M1), Matrix3d (the
// closed-form 3x3 tridiagonalisation: ViGraph::updateLandmarks, PseudoInverse::symmSqrt), Matrix<double,6,6>
// (TwoPoseStandardGraphError::compute) and MatrixXd n = 12 / 18 (TwoPoseExtrinsicsGraphError, heap-aligned hcoeffs).
#include <Eigen/Dense>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/ok_eigen.h"
}
using namespace Eigen;
static std::mt19937_64 rng(777);
static std::normal_distribution<double> nd(0.0, 1.0);
template <int N>
static int run_fixed(int iters, const char* name) {
  typedef Matrix<double, N, N> M;
  long badv = 0, badq = 0, badcase = 0, tot = 0;
  for (int it = 0; it < iters; ++it) {
    M B; for (int k = 0; k < N * N; ++k) B.data()[k] = nd(rng);
    Matrix<double, N, 1> d; for (int k = 0; k < N; ++k) d[k] = (it % 5 == 1 && k == 0) ? 0.0 : std::exp(8.0 * nd(rng));
    M A = B * d.asDiagonal() * B.transpose();
    if (it % 3 == 0) A = (A + A.transpose()).eval();
    if (it % 7 == 3) A(N - 1, 0) = A(0, N - 1) = 0.0;  // exercises the v1norm2 <= tol branch for N == 3
    if (it % 11 == 5) A.setZero();
    SelfAdjointEigenSolver<M> saes(A);
    double ev[N], vec[N * N];
    int info = ok_selfadjoint_eig(N, A.data(), ev, vec, 0);
    long bv = 0, bq = 0;
    for (int k = 0; k < N; ++k) if (std::memcmp(&ev[k], &saes.eigenvalues().data()[k], 8)) ++bv;
    for (int k = 0; k < N * N; ++k) if (std::memcmp(&vec[k], &saes.eigenvectors().data()[k], 8)) ++bq;
    badv += bv; badq += bq; badcase += (bv || bq || info != 0); tot++;
  }
  std::printf("eig%s: eigenvalue mismatches %ld/%ld, eigenvector mismatches %ld/%ld, bad cases %ld/%ld\n", name, badv, tot * N, badq, tot * N * N, badcase, tot);
  return badcase != 0;
}
static int run_dyn(int n, int iters) {
  long badv = 0, badq = 0, badcase = 0, tot = 0;
  for (int it = 0; it < iters; ++it) {
    MatrixXd B(n, n); for (int k = 0; k < n * n; ++k) B.data()[k] = nd(rng);
    VectorXd d(n); for (int k = 0; k < n; ++k) d[k] = std::exp(8.0 * nd(rng));
    MatrixXd A = B * d.asDiagonal() * B.transpose();
    if (it % 3 == 0) A = (A + A.transpose()).eval();
    SelfAdjointEigenSolver<MatrixXd> saes(A);
    std::vector<double> ev(n), vec(n * n);
    int info = ok_selfadjoint_eig(n, A.data(), ev.data(), vec.data(), 0);
    long bv = 0, bq = 0;
    for (int k = 0; k < n; ++k) if (std::memcmp(&ev[k], &saes.eigenvalues().data()[k], 8)) ++bv;
    for (int k = 0; k < n * n; ++k) if (std::memcmp(&vec[k], &saes.eigenvectors().data()[k], 8)) ++bq;
    badv += bv; badq += bq; badcase += (bv || bq || info != 0); tot++;
  }
  std::printf("eigX%d: eigenvalue mismatches %ld/%ld, eigenvector mismatches %ld/%ld, bad cases %ld/%ld\n", n, badv, tot * n, badq, tot * n * n, badcase, tot);
  return badcase != 0;
}
int main(int argc, char** argv) {
  int N = argc > 1 ? atoi(argv[1]) : 2000;
  int bad = 0;
  bad |= run_fixed<15>(N, "15");
  bad |= run_fixed<3>(N * 5, "3");
  bad |= run_fixed<6>(N * 2, "6");
  bad |= run_fixed<9>(N, "9");
  bad |= run_dyn(12, N);
  bad |= run_dyn(18, N / 2);
  return bad;
}
