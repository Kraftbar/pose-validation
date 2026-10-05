// OK_PORT_TEST_C: ok_place_dist.c ok_dbow.c
// Random-case, tolerance-0 comparison of the module-7d pieces that have a library oracle (Eigen 3.4.0, libstdc++, reference flags):
//   distinctiveness  the Eigen float expression of Frontend::verifyRecognisedPlace
//                    (Matrix<float,Dynamic,384> rowwise mean-subtraction, colwise squaredNorm, / (rows - 1), cwiseSqrt, sum)
//   std::sort        the introsort of ok_dbow.c (QueryResults sorting) vs std::sort on random Result vectors incl. ties
#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

extern "C" {
#include "../c/ok_place.h"
#include "../c/ok_dbow.h"
}

struct Tally {
  const char* name; long bad = 0, tot = 0;
  explicit Tally(const char* n) : name(n) {}
  void add(bool ok) { ++tot; if (!ok) ++bad; }
  void print() const { std::printf("  %s: %ld/%ld\n", name, bad, tot); }
};

int main() {
  std::mt19937 rng(11);
  Tally td("distinctiveness (float)"), ts("std::sort of query results");
  for (int rows = 1; rows <= 200; ++rows) for (int rep = 0; rep < 30; ++rep) {
    std::vector<unsigned char> desc(48 * size_t(rows));
    const unsigned density = 1 + rep % 7;
    for (auto& b : desc) { unsigned char v = 0; for (int k = 0; k < 8; ++k) if (rng() % 8 < density) v |= (1u << k); b = v; }
    // the C++ of verifyRecognisedPlace: an oversized matrix filled row by row, then conservativeResize
    const int bigRows = rows + (rep % 4 == 0 ? 3 : 0);
    Eigen::Matrix<float, Eigen::Dynamic, 48 * 8> descriptorMatrix(bigRows, 48 * 8);
    for (int r = 0; r < rows; ++r)
      for (size_t b = 0; b < 48; b++)
        for (size_t c = 0; c < 8; c++) descriptorMatrix(r, b * 8 + c) = (desc[48 * size_t(r) + b] & (1 << c)) ? 1.0f : 0.0f;
    descriptorMatrix.conservativeResize(rows, 48 * 8);
    Eigen::Matrix<float, 1, 48 * 8> stdev =
        ((descriptorMatrix.rowwise() - descriptorMatrix.colwise().mean()).colwise().squaredNorm() / (descriptorMatrix.rows() - 1)).cwiseSqrt();
    const float expect = float(rows) * stdev.sum();
    const float got = ok_place_distinctiveness(desc.data(), rows);
    uint32_t a, b; std::memcpy(&a, &expect, 4); std::memcpy(&b, &got, 4);
    const bool nan_both = (expect != expect) && (got != got);
    td.add(a == b || nan_both);
  }
  for (int it = 0; it < 30000; ++it) {
    const int n = 1 + int(rng() % (it % 5 == 0 ? 900 : 60));
    struct R { unsigned id; double s; };
    std::vector<ok_dbow_result> a;
    std::vector<R> b;
    const int levels = 1 + int(rng() % 12);   // few distinct scores -> many ties
    for (int i = 0; i < n; ++i) {
      const double s = -double(rng() % levels) / levels - (it % 3 ? 0.0 : double(rng() % 1000) * 1e-6);
      ok_dbow_result r; r.id = unsigned(i); r.score = s; a.push_back(r); b.push_back({unsigned(i), s});
    }
    std::sort(b.begin(), b.end(), [](const R& x, const R& y) { return x.s < y.s; });
    ok_dbow_sort_results(a.data(), n);
    bool ok = true;
    for (int i = 0; ok && i < n; ++i) ok = a[size_t(i)].id == b[size_t(i)].id;
    ts.add(ok);
  }
  td.print(); ts.print();
  const long bad = td.bad + ts.bad, tot = td.tot + ts.tot;
  std::printf("okvis_place_test: %ld/%ld\n", bad, tot);
  return bad == 0 ? 0 : 1;
}
