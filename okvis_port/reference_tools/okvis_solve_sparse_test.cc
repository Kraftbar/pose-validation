// OK_PORT_TEST_C: ok_sparse.c ok_amd.c
// Random-case, tolerance-0 comparison of the module-4 sparse kernels (okvis_port/c/ok_sparse.c, ok_amd.c) against
// the real Eigen 3.4.0 classes Ceres 2.2.0 uses with EIGEN_SPARSE: SimplicialLDLT<SparseMatrix<double>, Upper,
// NaturalOrdering<int>> driven exactly as ceres::internal::EigenSparseCholeskyTemplate does (a column-major Map of
// Ceres' LOWER_TRIANGULAR block CRS, i.e. the upper triangle plus the strictly-lower parts of the diagonal blocks;
// analyzePattern once, factorize + solve), and AMDOrdering<int> on random symmetric block patterns (Ceres' block
// Hessian). Compares the L / D factors, the solutions and the permutations bitwise.
#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>
extern "C" {
#include "../c/ok_sparse.h"
}

namespace {
struct Sec { std::string name; long bad = 0, tot = 0; };
std::vector<Sec> g_secs;
Sec& sec(const std::string& n) { for (auto& s : g_secs) if (s.name == n) return s; g_secs.push_back({n}); return g_secs.back(); }
std::mt19937_64 rng(31337);
double rnd() { std::uniform_real_distribution<double> u(-1.0, 1.0); return u(rng); }
void cmpd(Sec& s, const double* a, const double* b, long n, const char* what) {
  for (long i = 0; i < n; ++i) {
    s.tot++;
    if (std::memcmp(&a[i], &b[i], 8) != 0) { if (s.bad < 5) std::printf("    %s[%ld]: C %.17g real %.17g\n", what, i, a[i], b[i]); s.bad++; }
  }
}
void cmpi(Sec& s, const int* a, const int* b, long n, const char* what) {
  for (long i = 0; i < n; ++i) { s.tot++; if (a[i] != b[i]) { if (s.bad < 5) std::printf("    %s[%ld]: C %d real %d\n", what, i, a[i], b[i]); s.bad++; } }
}

// Build Ceres' LOWER_TRIANGULAR block CRS of a random block-sparse SPD matrix: rows/cols/vals arrays and the dense
// matrix it represents (for the right-hand sides).
struct BlockCrs { int n = 0; std::vector<int> rows, cols; std::vector<double> vals; Eigen::MatrixXd dense; };
BlockCrs random_block_spd(int nb, double density) {
  std::vector<int> bs(nb), pos(nb);
  int n = 0;
  for (int i = 0; i < nb; ++i) { const int c = int(rng() % 3); bs[i] = c == 0 ? 3 : (c == 1 ? 6 : 9); pos[i] = n; n += bs[i]; }
  std::vector<std::vector<char>> pat(nb, std::vector<char>(nb, 0));
  for (int i = 0; i < nb; ++i) { pat[i][i] = 1; for (int j = 0; j < i; ++j) if (std::uniform_real_distribution<double>(0, 1)(rng) < density) pat[i][j] = pat[j][i] = 1; }
  // SPD values respecting the pattern: A = sum over blocks of local SPD contributions
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < nb; ++i) {
    for (int j = 0; j <= i; ++j) {
      if (!pat[i][j]) continue;
      Eigen::MatrixXd M(bs[i], bs[j]);
      for (int r = 0; r < bs[i]; ++r) for (int c = 0; c < bs[j]; ++c) M(r, c) = rnd() * (i == j ? 1.0 : 0.05);
      A.block(pos[i], pos[j], bs[i], bs[j]) += M;
      if (i != j) A.block(pos[j], pos[i], bs[j], bs[i]) += M.transpose();
    }
  }
  A = (A + A.transpose()).eval() * 0.5;
  for (int i = 0; i < n; ++i) A(i, i) += 2.0 + double(n) * 0.1;
  BlockCrs out;
  out.n = n;
  out.dense = A;
  out.rows.assign(n + 1, 0);
  for (int i = 0; i < nb; ++i) {
    int row_nnz = 0;
    for (int j = 0; j <= i; ++j) if (pat[i][j]) row_nnz += bs[j];
    for (int r = 0; r < bs[i]; ++r) {
      out.rows[pos[i] + r + 1] = out.rows[pos[i] + r] + row_nnz;
      for (int j = 0; j <= i; ++j) {
        if (!pat[i][j]) continue;
        for (int c = 0; c < bs[j]; ++c) { out.cols.push_back(pos[j] + c); out.vals.push_back(A(pos[i] + r, pos[j] + c)); }
      }
    }
  }
  return out;
}

void test_ldlt() {
  Sec& sL = sec("ldlt_factor");
  Sec& sS = sec("ldlt_solve");
  for (int it = 0; it < 300; ++it) {
    const int nb = 1 + int(rng() % 40);
    BlockCrs m = random_block_spd(nb, it % 3 == 0 ? 0.9 : 0.15);
    const int n = m.n, nnz = int(m.vals.size());
    Eigen::Map<const Eigen::SparseMatrix<double, Eigen::ColMajor>> eigen_lhs(n, n, nnz, m.rows.data(), m.cols.data(), m.vals.data());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Upper, Eigen::NaturalOrdering<int>> solver;
    solver.analyzePattern(eigen_lhs);
    solver.factorize(eigen_lhs);
    ok_ldlt f;
    ok_ldlt_analyze(&f, n, m.rows.data(), m.cols.data());
    const int ok = ok_ldlt_factorize(&f, m.rows.data(), m.cols.data(), m.vals.data());
    sL.tot++;
    if ((solver.info() == Eigen::Success) != (ok != 0)) { sL.bad++; std::printf("    ldlt it=%d: C ok=%d Eigen info=%d\n", it, ok, int(solver.info())); }
    if (solver.info() == Eigen::Success && ok) {
      const Eigen::SparseMatrix<double> L = solver.matrixL();
      { const int nz = int(L.nonZeros()); cmpi(sL, &f.nnz_l, &nz, 1, "ldlt.nnz"); }
      if (f.nnz_l == int(L.nonZeros())) {
        cmpi(sL, f.Lp, L.outerIndexPtr(), n + 1, "ldlt.Lp");
        cmpi(sL, f.Li, L.innerIndexPtr(), f.nnz_l, "ldlt.Li");
        cmpd(sL, f.Lx, L.valuePtr(), f.nnz_l, "ldlt.Lx");
      }
      const Eigen::VectorXd D = solver.vectorD();
      cmpd(sL, f.D, D.data(), n, "ldlt.D");
      for (int k = 0; k < 3; ++k) {
        Eigen::VectorXd b(n), x(n), xc(n);
        for (int i = 0; i < n; ++i) b[i] = rnd();
        Eigen::Map<Eigen::VectorXd>(x.data(), n) = solver.solve(Eigen::Map<const Eigen::VectorXd>(b.data(), n));
        ok_ldlt_solve(&f, b.data(), xc.data());
        cmpd(sS, xc.data(), x.data(), n, "ldlt.solve");
      }
    }
    ok_ldlt_free(&f);
  }
}

void test_amd() {
  Sec& s = sec("amd");
  for (int it = 0; it < 2000; ++it) {
    const int n = 1 + int(rng() % 120);
    const double density = (it % 4 == 0) ? 0.8 : (it % 4 == 1 ? 0.3 : 0.05);
    std::vector<std::vector<char>> pat(n, std::vector<char>(n, 0));
    for (int i = 0; i < n; ++i) { pat[i][i] = 1; for (int j = 0; j < i; ++j) if (std::uniform_real_distribution<double>(0, 1)(rng) < density) pat[i][j] = pat[j][i] = 1; }
    std::vector<Eigen::Triplet<int>> trip;
    std::vector<int> Ap(n + 1, 0), Ai;
    for (int j = 0; j < n; ++j) { Ap[j] = int(Ai.size()); for (int i = 0; i < n; ++i) if (pat[i][j]) { Ai.push_back(i); trip.emplace_back(i, j, 1); } }
    Ap[n] = int(Ai.size());
    Eigen::SparseMatrix<int> block_hessian(n, n);
    block_hessian.setFromTriplets(trip.begin(), trip.end());
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> perm;
    Eigen::AMDOrdering<int> amd;
    amd(block_hessian, perm);
    std::vector<int> pc(n);
    ok_amd_order(n, Ap.data(), Ai.data(), pc.data());
    cmpi(s, pc.data(), perm.indices().data(), n, "amd.perm");
  }
}
}  // namespace

int main() {
  test_ldlt();
  test_amd();
  long bad = 0, tot = 0;
  for (auto& s : g_secs) { std::printf("  %-28s %ld/%ld\n", s.name.c_str(), s.bad, s.tot); bad += s.bad; tot += s.tot; }
  std::printf("okvis_solve_sparse_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
