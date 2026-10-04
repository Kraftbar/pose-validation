// Executable record of the Eigen 3.4.0 small-product evaluation orders the okvis_port module 3 relies on (error terms).
// Each case runs the exact C++ statement shape used by okvis_ceres (destination type matters: a Map<RowMajor> destination
// evaluates differently from a plain local) on random operands and requires every output coefficient to equal ONE of the
// summation orders over the D products p_k = a_ik * b_kj:
//   LF   left fold ((p0+p1)+p2)+...      (a packet lane: lhs column-major, rows packetised)
//   TREE redux_novec halving tree        (scalar coefficient: odd last row, or a scalar assignment loop)
//   VEC  packet lanes + horizontal add   (lhs row-major, rhs column-major, both contiguous)
// The expected order per coefficient is part of the test. No C code is linked (these are the rules ok_err.c encodes;
// the okvis_err_test.cc cross-check then verifies ok_err.c itself against the real classes).
#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
using namespace Eigen;
static std::mt19937_64 rng(7);
static double N() { return std::normal_distribution<double>(0, 1)(rng) * std::pow(10.0, int(rng() % 5) - 2); }
static double lf(const double* p, int n) { double s = p[0]; for (int i = 1; i < n; ++i) s += p[i]; return s; }
static double tr(const double* p, int n) { if (n == 1) return p[0]; return tr(p, n / 2) + tr(p + n / 2, n - n / 2); }
static void vl(const double* p, int np, double l[2]) { if (np == 1) { l[0] = p[0]; l[1] = p[1]; return; } double a[2], b[2]; vl(p, np / 2, a); vl(p + 2 * (np / 2), np - np / 2, b); l[0] = a[0] + b[0]; l[1] = a[1] + b[1]; }
static double vr(const double* p, int n) { if (n < 2) return p[0]; double l[2]; vl(p, n / 2, l); double r = l[0] + l[1]; if (n & 1) r += p[n - 1]; return r; }
typedef double (*F)(const double*, int);
static F ord[] = {lf, tr, vr};
enum { LF = 0, TREE = 1, VEC = 2 };
template <typename T> void fill(T& m) { for (int j = 0; j < m.cols(); ++j) for (int i = 0; i < m.rows(); ++i) m(i, j) = N(); }
static long g_bad = 0, g_tot = 0;
// EXPECT(i, j) -> LF/TREE/VEC
#define PROBE(NAME, R, C, D, EXPECT, ARES, AOP, BOP, ...)                                  \
  {                                                                                       \
    const int NT = 3000; long bad = 0;                                                    \
    for (int t = 0; t < NT; ++t) {                                                        \
      __VA_ARGS__                                                                         \
      for (int i = 0; i < R; ++i) for (int j = 0; j < C; ++j) {                           \
        double p[16]; for (int k = 0; k < D; ++k) p[k] = (AOP) * (BOP);                   \
        g_tot++; if (ord[EXPECT] (p, D) != (ARES)) { bad++; g_bad++; }                    \
      }                                                                                   \
    }                                                                                     \
    std::printf("  %-56s %ld/%d\n", NAME, bad, NT * R * C);                               \
  }

int main() {
  std::printf("eigen_product_modes_test (Eigen %d.%d.%d)\n", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
  // lhs column-major: rows packetised (left fold), an odd last row is a scalar coefficient (tree)
  PROBE("A  Matrix<2,4> = M2*M<2,4>", 2, 4, 2, LF, Y(i,j), A(i,k), B(k,j), Matrix2d A; Matrix<double,2,4> B; fill(A); fill(B); Matrix<double,2,4> Y; Y = A * B;)
  PROBE("A  M44*M44", 4, 4, 4, LF, Y(i,j), A(i,k), B(k,j), Matrix4d A, B; fill(A); fill(B); Matrix4d Y = A * B;)
  PROBE("A  Vector4d = M44*V4", 4, 1, 4, LF, Y(i,j), A(i,k), B(k,j), Matrix4d A; Vector4d B; fill(A); fill(B); Vector4d Y = A * B;)
  PROBE("A  Vector2d = M22*V2 (any order)", 2, 1, 2, LF, Y(i,j), A(i,k), B(k,j), Matrix2d A; Vector2d B; fill(A); fill(B); Vector2d Y = A * B;)
  PROBE("A  Map<V6> = M66*V6", 6, 1, 6, LF, Y(i,j), A(i,k), B(k,j), Matrix<double,6,6> A; Matrix<double,6,1> B; fill(A); fill(B); double buf[6]; Map<Matrix<double,6,1>> Y(buf); Y = A * B;)
  PROBE("A  M<3,4>*M44 into Map<RM>.bottomRightCorner<3,4>", 3, 4, 4, (i < 2 ? LF : TREE), Y(i,j), A(i,k), B(k,j),
        Matrix<double,3,4> A; Matrix4d B; fill(A); fill(B); double buf[42]; Map<Matrix<double,6,7,RowMajor>> Jl(buf); Jl.setZero(); Jl.bottomRightCorner<3,4>() = A * B; Matrix<double,3,4> Y = Jl.bottomRightCorner<3,4>();)
  PROBE("A  Map<7x6 RM>.bottomRightCorner<4,3>() = M44*M<4,3>", 4, 3, 4, LF, Y(i,j), A(i,k), B(k,j),
        Matrix4d A; Matrix<double,4,3> B; fill(A); fill(B); double buf[42]; Map<Matrix<double,7,6,RowMajor>> Jp(buf); Jp.setZero(); Jp.bottomRightCorner<4,3>() = A * B; Matrix<double,4,3> Y = Jp.bottomRightCorner<4,3>();)
  PROBE("A  RM26 = (Jw*T)*J nested: outer", 2, 6, 4, LF, Y(i,j), X(i,k), B(k,j), Matrix<double,2,4> Jw; Matrix4d T; Matrix<double,4,6> B; fill(Jw); fill(T); fill(B); Matrix<double,2,6,RowMajor> Y; Y = Jw * T * B; Matrix<double,2,4> X = Jw * T;)
  PROBE("A  RM26 = Jw*J (local RowMajor destination)", 2, 6, 4, LF, Y(i,j), A(i,k), B(k,j), Matrix<double,2,4> A; Matrix<double,4,6> B; fill(A); fill(B); Matrix<double,2,6,RowMajor> Y; Y = A * B;)
  PROBE("A  Map<2x4 RM> = -Jw*T (negation stays inside)", 2, 4, 4, LF, Y(i,j), nA(i,k), B(k,j), Matrix<double,2,4> A; Matrix4d B; fill(A); fill(B); double buf[8]; Map<Matrix<double,2,4,RowMajor>> Y(buf); Y = -A * B; Matrix<double,2,4> nA = -A;)
  PROBE("A  block<3,1> = -C*t (3x3 lhs: rows 0-1 LF, row 2 tree)", 3, 1, 3, (i < 2 ? LF : TREE), Y(i,j), nA(i,k), B(k,j),
        Matrix3d A; Vector3d B; fill(A); fill(B); Matrix4d T; T.setIdentity(); T.topRightCorner<3,1>() = -A * B; Matrix3d nA = -A; Matrix<double,3,1> Y = T.topRightCorner<3,1>();)
  PROBE("A  block<3,3> = -C*crossMx", 3, 3, 3, (i < 2 ? LF : TREE), Y(i,j), nA(i,k), B(k,j),
        Matrix3d A, B; fill(A); fill(B); Matrix<double,4,6> J; J.setZero(); J.topRightCorner<3,3>() = -A * B; Matrix3d nA = -A; Matrix3d Y = J.topRightCorner<3,3>();)
  PROBE("A  RM66 = (CM66*RM66).eval()", 6, 6, 6, LF, Y(i,j), A(i,k), Y0(k,j), Matrix<double,6,6> A; Matrix<double,6,6,RowMajor> Y0; fill(A); fill(Y0); Matrix<double,6,6,RowMajor> Y = Y0; Y = (A * Y).eval();)
  PROBE("A  RM33 = (CM33*RM33).eval()", 3, 3, 3, (i < 2 ? LF : TREE), Y(i,j), A(i,k), Y0(k,j), Matrix3d A; Matrix<double,3,3,RowMajor> Y0; fill(A); fill(Y0); Matrix<double,3,3,RowMajor> Y = Y0; Y = (A * Y).eval();)
  PROBE("A  Map<V3> = M33*V3", 3, 1, 3, (i < 2 ? LF : TREE), Y(i,j), A(i,k), B(k,j), Matrix3d A; Vector3d B; fill(A); fill(B); double buf[3]; Map<Vector3d> Y(buf); Y = A * B;)
  // lhs row-major, rhs column-major: vectorised redux inside every coefficient
  PROBE("C  Map<2x3 RM> = Map<2x4 RM>*M<4,3>", 2, 3, 4, VEC, Y(i,j), A(i,k), B(k,j), double ab[8]; Map<Matrix<double,2,4,RowMajor>> A(ab); Matrix<double,4,3> B; fill(A); fill(B); double buf[6]; Map<Matrix<double,2,3,RowMajor>> Y(buf); Y = A * B;)
  // both row-major, scalar assignment loop: every coefficient the halving tree
  PROBE("B  Map<2x7 RM> = RM26*RM67", 2, 7, 6, TREE, Y(i,j), A(i,k), B(k,j), Matrix<double,2,6,RowMajor> A; Matrix<double,6,7,RowMajor> B; fill(A); fill(B); double buf[14]; Map<Matrix<double,2,7,RowMajor>> Y(buf); Y = A * B;)
  PROBE("B  Map<6x7 RM> = RM66*RM67", 6, 7, 6, TREE, Y(i,j), A(i,k), B(k,j), Matrix<double,6,6,RowMajor> A; Matrix<double,6,7,RowMajor> B; fill(A); fill(B); double buf[42]; Map<Matrix<double,6,7,RowMajor>> Y(buf); Y = A * B;)
  PROBE("B  RM33 = RM34*RM43 (local)", 3, 3, 4, TREE, Y(i,j), A(i,k), B(k,j), Matrix<double,3,4,RowMajor> A; Matrix<double,4,3,RowMajor> B; fill(A); fill(B); Matrix<double,3,3,RowMajor> Y; Y = A * B;)
  PROBE("B  Map<3x4 RM> = RM33*RM34", 3, 4, 3, TREE, Y(i,j), A(i,k), B(k,j), Matrix<double,3,3,RowMajor> A; Matrix<double,3,4,RowMajor> B; fill(A); fill(B); double buf[12]; Map<Matrix<double,3,4,RowMajor>> Y(buf); Y = A * B;)
  // Large dimension (9): GEMV column kernel, a left fold from 0 per row
  PROBE("G  Map<V9> = M99*V9 (GEMV)", 9, 1, 9, LF, Y(i,j), A(i,k), B(k,j), Matrix<double,9,9> A; Matrix<double,9,1> B; fill(A); fill(B); double buf[9]; Map<Matrix<double,9,1>> Y(buf); Y = A * B;)
  std::printf("eigen_product_modes_test: %ld/%ld\n", g_bad, g_tot);
  return g_bad == 0 ? 0 : 1;
}
