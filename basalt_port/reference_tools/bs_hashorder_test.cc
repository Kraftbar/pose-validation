// Oracle for basalt_port/c/bs_hashorder.{h,c}: the real libstdc++ std::unordered_map / unordered_set (g++ 13.3) with the real key types and
// hashers (std::hash<size_t>, std::hash<int>, std::hash<basalt::TimeCamId>) vs the C node-order model, exact iteration order (tolerance 0),
// element count and bucket count, after every operation batch. Operations: operator[]/insert, erase(key), erase(iterator) while iterating
// (as LandmarkDatabase::removeFrame / removeKeyframes do), occasional bulk erase to (near) empty and refill.
//   bs_hashorder_test <seed> <scenarios>
#include <basalt/utils/common_types.h>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>
extern "C" {
#include "bs_hashorder.h"
}
typedef std::mt19937_64 Rng;
static long g_cmp = 0, g_bad = 0, g_ops = 0, g_rehash = 0;

template <class Map, class KeyOf>
static bool same(const Map& m, const bs_htab& t, KeyOf keyof, const char* what, long sc) {
  ++g_cmp;
  bool ok = m.size() == t.nelem && m.bucket_count() == t.nbuckets;
  if (ok) {
    const bs_hnode* n = t.before_begin.next;
    for (auto it = m.begin(); it != m.end(); ++it, n = n->next) {
      int64_t a0, a1;
      keyof(*it, a0, a1);
      if (!n || n->k0 != a0 || n->k1 != a1) { ok = false; break; }
    }
    if (ok && n) ok = false;
  }
  if (!ok) { ++g_bad; if (g_bad < 20) std::printf("MISMATCH %s scenario %ld size %zu/%zu buckets %zu/%zu\n", what, sc, m.size(), t.nelem, m.bucket_count(), t.nbuckets); }
  return ok;
}

struct KeyGen {
  int mode;
  int64_t next;
  Rng* r;
  int64_t gen() {
    switch (mode) {
      case 0: next += 1 + (*r)() % 4; return next;                 // ascending ids with small gaps (landmark ids)
      case 1: return (int64_t)((*r)() % 5000);                      // small range, many repeats
      case 2: return (int64_t)(*r)();                               // random 64-bit
      case 3: return (int64_t)((*r)() % 100000) * 1021;             // multiples of a prime-ish
      default: return -(int64_t)((*r)() % 3000);                    // negative ints
    }
  }
};

static void run_u64(Rng& r, long sc, bool is_int) {
  std::unordered_map<size_t, int> m;
  std::unordered_set<int> s;
  bs_htab t;
  bs_htab_init(&t, BS_HK_U64);
  KeyGen kg{(int)(r() % 5), (int64_t)(r() % 100000), &r};
  if (is_int && kg.mode == 2) kg.mode = 4;
  int nb = 3 + r() % 40;
  size_t cap = (r() % 3 == 0) ? 6000 : 400;
  std::vector<int64_t> live;
  for (int b = 0; b < nb; ++b) {
    int ops = 1 + r() % 60;
    for (int o = 0; o < ops; ++o) {
      ++g_ops;
      int kind = r() % 10;
      if (kind < 6 && live.size() < cap) {
        int64_t k = kg.gen();
        if (is_int) k = (int)k;
        int ins;
        bs_htab_insert(&t, k, 0, &ins);
        if (is_int) { bool x = s.insert((int)k).second; if (x != (bool)ins) ++g_bad; }
        else { bool x = m.emplace((size_t)k, 0).second; if (x != (bool)ins) ++g_bad; if (is_int == false) {} }
        if (ins) live.push_back(k);
      } else if (kind < 8 && !live.empty()) {
        size_t i = r() % live.size();
        int64_t k = live[i];
        live.erase(live.begin() + i);
        if (is_int) s.erase((int)k); else m.erase((size_t)k);
        if (!bs_htab_erase_key(&t, k, 0)) ++g_bad;
      } else if (!live.empty()) {
        // iterator erase pass: erase every node whose key satisfies a random predicate, while iterating (removeFrame pattern)
        uint64_t mask = r() % 7 + 2;
        if (is_int) {
          for (auto it = s.begin(); it != s.end();) if ((uint64_t)(int64_t)*it % mask == 0) it = s.erase(it); else ++it;
        } else {
          for (auto it = m.begin(); it != m.end();) if (it->first % mask == 0) it = m.erase(it); else ++it;
        }
        for (bs_hnode* n = t.before_begin.next; n;) if ((uint64_t)(is_int ? (int64_t)(int)n->k0 : n->k0) % mask == 0) n = bs_htab_erase_node(&t, n); else n = n->next;
        live.clear();
        for (bs_hnode* n = t.before_begin.next; n; n = n->next) live.push_back(n->k0);
      }
      if (r() % 4 == 0) {
        if (is_int) same(s, t, [](const int& k, int64_t& a, int64_t& b) { a = k; b = 0; }, "uset<int>", sc);
        else same(m, t, [](const std::pair<const size_t, int>& kv, int64_t& a, int64_t& b) { a = (int64_t)kv.first; b = 0; }, "umap<size_t>", sc);
      }
    }
    if (r() % 15 == 0) {   // wipe most elements
      if (is_int) { s.clear(); } else { m.clear(); }
      bs_htab_destroy(&t, nullptr);
      bs_htab_init(&t, BS_HK_U64);
      // clear() keeps bucket count and policy in libstdc++; the model's destroy resets: only compare after re-creating both
      if (is_int) { std::unordered_set<int> fresh; s.swap(fresh); } else { std::unordered_map<size_t, int> fresh; m.swap(fresh); }
      live.clear();
    }
    if (is_int) same(s, t, [](const int& k, int64_t& a, int64_t& b) { a = k; b = 0; }, "uset<int>", sc);
    else same(m, t, [](const std::pair<const size_t, int>& kv, int64_t& a, int64_t& b) { a = (int64_t)kv.first; b = 0; }, "umap<size_t>", sc);
  }
  bs_htab_destroy(&t, nullptr);
}

static void run_tcid(Rng& r, long sc) {
  std::unordered_map<basalt::TimeCamId, int> m;
  bs_htab t;
  bs_htab_init(&t, BS_HK_TCID);
  int64_t t0 = 1403636579763555584LL + (int64_t)(r() % 1000) * 1000000;
  int64_t dt = (r() % 2) ? 50000000LL : 5000000LL;
  size_t cap = (r() % 3 == 0) ? 3000 : 40;
  std::vector<std::pair<int64_t, int64_t>> live;
  int nb = 3 + r() % 40;
  for (int b = 0; b < nb; ++b) {
    int ops = 1 + r() % 40;
    for (int o = 0; o < ops; ++o) {
      ++g_ops;
      int kind = r() % 10;
      if (kind < 6 && live.size() < cap) {
        int64_t f = t0 + (int64_t)(r() % (cap * 4 + 30)) * dt;
        int64_t c = r() % 2;
        if (r() % 20 == 0) { f = (int64_t)r(); c = r() % 5; }
        int ins;
        bs_htab_insert(&t, f, c, &ins);
        bool x = m.emplace(basalt::TimeCamId(f, c), 0).second;
        if (x != (bool)ins) ++g_bad;
        if (ins) live.push_back({f, c});
      } else if (kind < 8 && !live.empty()) {
        size_t i = r() % live.size();
        auto k = live[i];
        live.erase(live.begin() + i);
        m.erase(basalt::TimeCamId(k.first, k.second));
        if (!bs_htab_erase_key(&t, k.first, k.second)) ++g_bad;
      } else if (!live.empty()) {
        uint64_t mask = r() % 5 + 2;
        for (auto it = m.begin(); it != m.end();) if ((uint64_t)(it->first.frame_id / dt + it->first.cam_id) % mask == 0) it = m.erase(it); else ++it;
        for (bs_hnode* n = t.before_begin.next; n;) if ((uint64_t)(n->k0 / dt + n->k1) % mask == 0) n = bs_htab_erase_node(&t, n); else n = n->next;
        live.clear();
        for (bs_hnode* n = t.before_begin.next; n; n = n->next) live.push_back({n->k0, n->k1});
      }
      if (r() % 4 == 0) same(m, t, [](const std::pair<const basalt::TimeCamId, int>& kv, int64_t& a, int64_t& b) { a = kv.first.frame_id; b = (int64_t)kv.first.cam_id; }, "umap<TimeCamId>", sc);
    }
    same(m, t, [](const std::pair<const basalt::TimeCamId, int>& kv, int64_t& a, int64_t& b) { a = kv.first.frame_id; b = (int64_t)kv.first.cam_id; }, "umap<TimeCamId>", sc);
  }
  bs_htab_destroy(&t, nullptr);
}

int main(int argc, char** argv) {
  long seed = argc > 1 ? atol(argv[1]) : 1, n = argc > 2 ? atol(argv[2]) : 200;
  Rng r(seed);
  for (long i = 0; i < n; ++i) {
    switch (i % 3) {
      case 0: run_u64(r, i, false); break;
      case 1: run_tcid(r, i); break;
      default: run_u64(r, i, true); break;
    }
  }
  std::printf("hashorder seed %ld scenarios %ld ops %ld comparisons %ld mismatches %ld\n", seed, n, g_ops, g_cmp, g_bad);
  return g_bad != 0;
}
