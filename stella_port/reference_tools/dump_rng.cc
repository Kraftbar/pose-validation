// Standalone RNG ground-truth generator for stella_port module 3.
// Reproduces stella_vslam's util::random_array.{h,cc} (create_random_engine
// with use_fixed_seed=true, create_random_array<unsigned int>) using the
// *actual* std::mt19937 / std::uniform_int_distribution / std::shuffle from
// this machine's libstdc++ (GCC 13, matching the reference build's
// toolchain) -- no stella library link needed, so this does not require the
// reference build. Writes a fixture the C port's harness compares against
// bit-exact (integers, so exact equality).
//
// Usage: dump_rng <out_file>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <random>
#include <vector>

// Verbatim algorithm from stella_vslam/src/stella_vslam/util/random_array.cc
// (BSD-2, AIST 2019 + stella-cv 2022) -- reproduced here (read-only
// reference tool, not part of the port) so the ground truth is exactly
// what the reference build executes.
template <typename T>
static std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max,
                                           std::mt19937& random_engine) {
    std::uniform_int_distribution<T> uniform_int_distribution(rand_min, rand_max);
    const auto make_size = static_cast<size_t>(size * 1.2);
    std::vector<T> v;
    v.reserve(size);
    while (v.size() != size) {
        while (v.size() < make_size) {
            v.push_back(uniform_int_distribution(random_engine));
        }
        std::sort(v.begin(), v.end());
        auto unique_end = std::unique(v.begin(), v.end());
        if (size < static_cast<size_t>(std::distance(v.begin(), unique_end))) {
            unique_end = std::next(v.begin(), size);
        }
        v.erase(unique_end, v.end());
    }
    std::shuffle(v.begin(), v.end(), random_engine);
    return v;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        return 1;
    }
    std::ofstream out(argv[1]);

    // 1. Raw engine outputs from a default-seeded mt19937 (use_fixed_seed=true).
    {
        std::mt19937 e; // default seed 5489
        out << "raw " << 2000 << "\n";
        for (int i = 0; i < 2000; ++i) {
            out << e() << (i + 1 < 2000 ? ' ' : '\n');
        }
    }

    // 2. create_random_array<unsigned int> replaying realistic RANSAC call
    // patterns: homography_solver uses min_set_size=4, fundamental_solver
    // uses min_set_size=8 (see solve/{homography,fundamental}_solver.cc),
    // called num_ransac_iters (default 100, config sets it explicitly)
    // times in a row on one engine, num_matches ranging over what real
    // frame pairs produce. Also cover edge sizes.
    struct Case { uint32_t size; uint32_t rand_max; int reps; };
    std::vector<Case> cases = {
        {4, 49, 100}, {8, 49, 100}, {4, 99, 100}, {8, 99, 100},
        {4, 199, 100}, {8, 199, 100}, {4, 499, 100}, {8, 499, 100},
        {4, 999, 100}, {8, 999, 100}, {4, 1499, 100}, {8, 1499, 100},
        {4, 7, 50}, {8, 15, 50}, {4, 3, 20},
        /* exhaustive small ranges (every rand_max 3..300), size 4 and 8,
         * to close the gap the curated list above left (arbitrary
         * num_matches-1 values, not just round numbers). */
    };
    for (uint32_t rm = 3; rm <= 300; ++rm) {
        cases.push_back({4, rm, 20});
    }
    for (uint32_t rm = 7; rm <= 300; ++rm) {
        cases.push_back({8, rm, 20});
    }
    for (auto& c : cases) {
        std::mt19937 e; // fresh default-seeded engine per case, like each
                         // homography_solver/fundamental_solver instance
        out << "arr " << c.size << ' ' << c.rand_max << ' ' << c.reps << "\n";
        for (int r = 0; r < c.reps; ++r) {
            auto v = create_random_array<unsigned int>(c.size, 0U, c.rand_max, e);
            for (size_t i = 0; i < v.size(); ++i) {
                out << v[i] << (i + 1 < v.size() ? ' ' : '\n');
            }
        }
    }
    return 0;
}
