// Checks stella_port/c/sv_umap_order.{h,c} (behavioural model of libstdc++'s
// std::unordered_map<unsigned,T> iteration order) against the real
// container: random insert (+ occasional erase) sequences with key patterns
// like stella's ids (dense ranges, holes, large ids). Build (from repo root):
//   gcc -std=c99 -O2 -c stella_port/c/sv_umap_order.c -o /tmp/sv_umap_order.o
//   g++ -O2 stella_port/reference_tools/umap_order_test.cc /tmp/sv_umap_order.o -o /tmp/umap_order_test
#include <cstdio>
#include <cstdlib>
#include <random>
#include <unordered_map>
#include <vector>
extern "C" {
#include "../c/sv_umap_order.h"
}
int main() {
    std::mt19937 rng(12345);
    long mismatches = 0, checks = 0;
    for (int t = 0; t < 60000; ++t) {
        std::unordered_map<unsigned, int> ref;
        sv_umap_order m;
        sv_umap_init(&m);
        const int mode = t % 6;
        const int nmax = (mode == 5) ? 3000 : (t % 300) + 1;
        unsigned base = rng() % 5000;
        for (int i = 0; i < nmax; ++i) {
            unsigned key;
            switch (mode) {
                case 0: key = base + i; break;                      // dense ascending
                case 1: key = base + i * 3 + (rng() % 3); break;    // holes
                case 2: key = rng() % 100000; break;                // random
                case 3: key = base + (rng() % (nmax + 5)); break;   // dense random with repeats
                case 4: key = (rng() % 20) * 4096 + rng() % 50; break; // collisions
                default: key = base + i + (rng() % 2) * 100000; break;
            }
            ref[key] = 1;
            sv_umap_insert(&m, key);
            if ((rng() % 25) == 0 && !ref.empty()) {
                unsigned ek = ref.begin()->first;
                if (rng() % 2) { auto it = ref.begin(); std::advance(it, rng() % ref.size()); ek = it->first; }
                ref.erase(ek);
                sv_umap_erase(&m, ek);
            }
        }
        // compare full order
        std::vector<unsigned> a, b;
        for (auto& kv : ref) a.push_back(kv.first);
        for (int i = m.first; i != -1; i = m.next[i]) b.push_back(m.keys[i]);
        ++checks;
        if (a != b) ++mismatches;
        sv_umap_free(&m);
    }
    printf("umap_order_test: %ld/%ld sequences differ\n", mismatches, checks);
    return mismatches != 0;
}
