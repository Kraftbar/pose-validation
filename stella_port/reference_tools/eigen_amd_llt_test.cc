// Checks stella_port/c/sv_eigen_amd.c (sv_amd_order) and the general sparse
// LLT in sv_eigen_llt.c (sv_sllt_*) against REAL Eigen 3.4 driven exactly
// the way g2o's LinearSolverEigen drives them (block AMD ordering from the
// upper block pattern -> blockToScalarPermutation -> analyzePattern-
// WithPermutation -> factorize -> solve), on random block patterns with
// random SPD block-structured values. Bit-exact expected (0 mismatches).
//
// Build (from repo root; Eigen headers from external/eigen):
//   gcc -std=c99 -O2 -ffp-contract=off -c stella_port/c/sv_eigen_amd.c -o /tmp/sv_amd.o
//   gcc -std=c99 -O2 -ffp-contract=off -c stella_port/c/sv_eigen_llt.c -o /tmp/sv_llt.o
//   g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -Iexternal/eigen \
//       stella_port/reference_tools/eigen_amd_llt_test.cc /tmp/sv_amd.o /tmp/sv_llt.o -lm -o /tmp/eigen_amd_llt_test
//   /tmp/eigen_amd_llt_test [patterns.txt]   # optional extra real patterns: "n nnz" then col-major (col row) pairs
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include <fstream>
extern "C" {
#include "../c/sv_eigen_amd.h"
#include "../c/sv_eigen_llt.h"
}
using namespace Eigen;

struct G2oLikeSolver : public SimplicialLLT<SparseMatrix<double, ColMajor>, Upper> {
    typedef SimplicialLLT<SparseMatrix<double, ColMajor>, Upper> Base;
    typedef SparseMatrix<double, ColMajor> SM;
    void analyzePatternWithPermutation(SM& a, const PermutationMatrix<Dynamic, Dynamic>& permutation) {
        m_Pinv = permutation;
        m_P = permutation.inverse();
        int size = a.cols();
        SM ap(size, size);
        ap.selfadjointView<Upper>() = a.selfadjointView<Upper>().twistedBy(m_P);
        analyzePattern_preordered(ap, false);
    }
    using Base::analyzePattern_preordered;
};

static long g_amd_bad = 0, g_amd_n = 0, g_llt_bad = 0, g_llt_n = 0;

static void run_pattern(int nb, const std::vector<std::vector<int>>& cols /* per block column: rows<=col ascending */,
                        std::mt19937& rng, int bs) {
    // block CCS
    std::vector<int> Ap(nb + 1, 0), Ai;
    for (int c = 0; c < nb; ++c) {
        for (int r : cols[c]) Ai.push_back(r);
        Ap[c + 1] = (int)Ai.size();
    }
    // Eigen: AMD on the block pattern exactly like LinearSolverEigen
    SparseMatrix<double, ColMajor> aux(nb, nb);
    aux.resizeNonZeros((int)Ai.size());
    for (int i = 0; i <= nb; ++i) aux.outerIndexPtr()[i] = Ap[i];
    for (size_t i = 0; i < Ai.size(); ++i) { aux.innerIndexPtr()[i] = Ai[i]; aux.valuePtr()[i] = 1.0; }
    AMDOrdering<int> ordering;
    PermutationMatrix<Dynamic, Dynamic> blockP;
    ordering(aux, blockP);
    std::vector<int> mine(nb);
    sv_amd_order(nb, Ap.data(), Ai.data(), mine.data());
    ++g_amd_n;
    bool same = (int)blockP.indices().size() == nb;
    for (int i = 0; same && i < nb; ++i) same = blockP.indices()(i) == mine[i];
    if (!same) { ++g_amd_bad; return; }

    // scalar CCS (upper) with random SPD values
    int n = nb * bs;
    // dense SPD block matrix M = pattern-respecting: diagonally dominant random symmetric
    std::uniform_real_distribution<double> ud(-1.0, 1.0);
    std::vector<double> full((size_t)n * n, 0.0);
    for (int c = 0; c < nb; ++c)
        for (int r : cols[c]) {
            for (int cc = 0; cc < bs; ++cc)
                for (int rr = 0; rr < bs; ++rr) {
                    double v = ud(rng);
                    full[(size_t)(r * bs + rr) + (size_t)(c * bs + cc) * n] = v;
                    full[(size_t)(c * bs + cc) + (size_t)(r * bs + rr) * n] = v; // symmetric
                }
        }
    for (int i = 0; i < n; ++i) {
        double s = 0;
        for (int j = 0; j < n; ++j) s += std::fabs(full[(size_t)i + (size_t)j * n]);
        full[(size_t)i + (size_t)i * n] = s + 1.0;
    }
    std::vector<int> sp(n + 1, 0), si;
    std::vector<double> sx;
    for (int c = 0; c < nb; ++c)
        for (int cc = 0; cc < bs; ++cc) {
            for (int r : cols[c]) {
                int elems = (r == c) ? cc + 1 : bs;
                for (int rr = 0; rr < elems; ++rr) {
                    si.push_back(r * bs + rr);
                    sx.push_back(full[(size_t)(r * bs + rr) + (size_t)(c * bs + cc) * n]);
                }
            }
            sp[c * bs + cc + 1] = (int)si.size();
        }
    SparseMatrix<double, ColMajor> A(n, n);
    A.resizeNonZeros((int)si.size());
    for (int i = 0; i <= n; ++i) A.outerIndexPtr()[i] = sp[i];
    for (size_t i = 0; i < si.size(); ++i) { A.innerIndexPtr()[i] = si[i]; A.valuePtr()[i] = sx[i]; }

    PermutationMatrix<Dynamic, Dynamic> scalarP(n);
    int sidx = 0;
    for (int i = 0; i < nb; ++i) {
        int base = blockP.indices()(i) * bs;
        for (int j = 0; j < bs; ++j) scalarP.indices()(sidx++) = base++;
    }
    G2oLikeSolver chol;
    chol.analyzePatternWithPermutation(A, scalarP);
    chol.factorize(A);
    std::vector<double> b(n);
    for (auto& v : b) v = ud(rng);
    VectorXd x = chol.solve(Map<VectorXd>(b.data(), n));

    // mine
    std::vector<int> sperm(n);
    sidx = 0;
    for (int i = 0; i < nb; ++i) { int base = mine[i] * bs; for (int j = 0; j < bs; ++j) sperm[sidx++] = base++; }
    sv_sllt f;
    sv_sllt_analyze(&f, n, sp.data(), si.data(), sperm.data());
    int ok = sv_sllt_factorize(&f, sx.data(), sp.data(), si.data());
    std::vector<double> xm(n);
    sv_sllt_solve(&f, b.data(), xm.data());
    ++g_llt_n;
    bool lsame = ok && chol.info() == Success;
    for (int i = 0; lsame && i < n; ++i) lsame = std::memcmp(&x(i), &xm[i], 8) == 0;
    if (!lsame) ++g_llt_bad;
    sv_sllt_free(&f);
}

int main(int argc, char** argv) {
    std::mt19937 rng(777);
    for (int t = 0; t < 4000; ++t) {
        int nb = 1 + (int)(rng() % 60);
        int mode = t % 6;
        std::vector<std::vector<int>> cols(nb);
        for (int c = 0; c < nb; ++c) {
            for (int r = 0; r <= c; ++r) {
                bool on = (r == c);
                switch (mode) {
                    case 0: on = true; break;                                   // dense
                    case 1: on = on || (rng() % 100) < 15; break;               // sparse random
                    case 2: on = on || (c - r) <= 2; break;                     // banded
                    case 3: on = on || r == 0 || c == nb - 1; break;            // arrow
                    case 4: on = on || (rng() % 100) < 60; break;               // dense-ish
                    default: on = on || ((rng() % 100) < 5 + (c % 7) * 6); break;
                }
                if (on) cols[c].push_back(r);
            }
        }
        run_pattern(nb, cols, rng, (t % 3 == 0) ? 3 : 6);
    }
    for (int a = 1; a < argc; ++a) {
        std::ifstream in(argv[a]);
        int nb, nnz;
        while (in >> nb >> nnz) {
            std::vector<std::vector<int>> cols(nb);
            for (int i = 0; i < nnz; ++i) { int c, r; in >> c >> r; cols[c].push_back(r); }
            run_pattern(nb, cols, rng, 6);
        }
    }
    std::printf("eigen_amd_llt_test: AMD %ld/%ld mismatching patterns, sparse LLT solve %ld/%ld mismatching\n",
                g_amd_bad, g_amd_n, g_llt_bad, g_llt_n);
    return (g_amd_bad || g_llt_bad) ? 1 : 0;
}
