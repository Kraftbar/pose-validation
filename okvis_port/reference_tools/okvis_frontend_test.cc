// OK_PORT_TEST_SRC: okvis_frontend/src/stereo_triangulation.cpp
// OK_PORT_TEST_C: ok_triangulate.c ok_eigen.c
// Random-case, tolerance-0 comparison of ok_fe_triangulate_fast (module M7b) against the REAL
// okvis::triangulation::triangulateFast (compiled from okvis_frontend/src/stereo_triangulation.cpp with Eigen 3.4.0 and the
// reference flags -O2 -DNDEBUG -ffp-contract=off -fno-fast-math): rays that intersect, nearly parallel rays, divergent
// rays, rays with random noise, tiny baselines, random sigma. Compares isValid, isParallel and all four coordinates.
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <okvis/triangulation/stereo_triangulation.hpp>

extern "C" {
#include "../c/ok_frontend.h"
}

static long g_tot = 0, g_bad = 0;
static bool same(double a, double b) { return std::memcmp(&a, &b, 8) == 0; }

int main() {
  std::mt19937_64 rng(777);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  long nbad[4] = {0, 0, 0, 0}, ntot[4] = {0, 0, 0, 0}, npar = 0, ninv = 0, nvalid = 0;
  for (int it = 0; it < 2000000; ++it) {
    const int mode = it % 4;
    Eigen::Vector3d p1(U(rng) * 3, U(rng) * 3, U(rng) * 3);
    const double bl = (mode == 2) ? 1.0e-3 * std::fabs(U(rng)) : 0.05 + 0.5 * std::fabs(U(rng));
    Eigen::Vector3d dir(U(rng), U(rng), U(rng));
    if (dir.norm() < 1e-3) dir = Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d p2 = p1 + bl * dir.normalized();
    Eigen::Vector3d X(U(rng) * 5, U(rng) * 5, 5.0 + 20.0 * std::fabs(U(rng)));
    if (mode == 3) X *= 1.0e4;                                   // nearly parallel rays
    Eigen::Vector3d e1 = (X - p1).normalized(), e2 = (X - p2).normalized();
    const double noise = (it % 7 == 0) ? 0.05 : (it % 5 == 0 ? 1.0e-3 : 0.0);
    if (noise > 0) {
      e1 = (e1 + noise * Eigen::Vector3d(U(rng), U(rng), U(rng))).normalized();
      e2 = (e2 + noise * Eigen::Vector3d(U(rng), U(rng), U(rng))).normalized();
    }
    if (it % 11 == 0) e2 = -e2;                                  // divergent
    const double sigma = (0.5 + std::fabs(U(rng))) * 1.0e-2 * ((it % 3) + 1);
    bool v0 = false, par0 = false;
    const Eigen::Vector4d h0 = okvis::triangulation::triangulateFast(p1, e1, p2, e2, sigma, v0, par0);
    int v1 = -1, par1 = -1;
    double h1[4];
    ok_fe_triangulate_fast(p1.data(), e1.data(), p2.data(), e2.data(), sigma, &v1, &par1, h1);
    bool ok = (int(v0) == v1) && (int(par0) == par1);
    for (int k = 0; k < 4; ++k) ok = ok && same(h0[k], h1[k]);
    ++g_tot; ++ntot[mode];
    if (!ok) { ++g_bad; ++nbad[mode]; if (g_bad < 5) std::printf("MISMATCH mode %d iter %d valid %d/%d par %d/%d\n", mode, it, int(v0), v1, int(par0), par1); }
    npar += par0; ninv += !v0; nvalid += v0;
  }
  std::printf("  triangulateFast: %ld/%ld (valid %ld, invalid %ld, parallel %ld)\n", g_bad, g_tot, nvalid, ninv, npar);
  std::printf("okvis_frontend_test: %ld/%ld\n", g_bad, g_tot);
  return g_bad == 0 ? 0 : 1;
}
