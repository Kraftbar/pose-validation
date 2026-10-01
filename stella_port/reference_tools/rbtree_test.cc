// Validates stella_port/c/sv_rbtree.c (behavioural model of the std::map red-black
// tree) against the real libstdc++ std::map<unsigned, unsigned, id_less-like Cmp>,
// where Cmp mimics id_less<std::weak_ptr<keyframe>>:
//     comp(a, b) = !expired(a) && (expired(b) || a < b)
// with keys that become expired while they are still in the tree. Random operation
// sequences (operator[] insert, find/count, erase(key), expiry of a key, rebuild from
// a sorted range as graph_node::update_connections does); after every operation the
// results and the COMPLETE tree structure (pre-order, key, colour) are compared,
// so the rotations / relinking / hint-insertion positions must agree exactly.
// Build (repo root):
//   gcc -std=c99 -O2 -c stella_port/c/sv_rbtree.c -o rbtree.o
//   g++ -O2 -std=c++14 stella_port/reference_tools/rbtree_test.cc rbtree.o -o rbtree_test
#include <cstdio>
#include <map>
#include <random>
#include <vector>
extern "C" {
#include "../c/sv_rbtree.h"
}

static std::vector<char> g_exp(64, 0);

struct Cmp {
    bool operator()(unsigned a, unsigned b) const { return !g_exp[a] && (g_exp[b] || a < b); }
};
static int c_less(void*, unsigned a, unsigned b) { return !g_exp[a] && (g_exp[b] || a < b); }

typedef std::map<unsigned, unsigned, Cmp> Map;
typedef std::_Rb_tree_node<std::pair<const unsigned, unsigned>> Node;

static void dump_std(const std::_Rb_tree_node_base* x, std::vector<long>& out) {
    if (!x) {
        out.push_back(-1);
        return;
    }
    const Node* n = static_cast<const Node*>(x);
    out.push_back((long)n->_M_valptr()->first * 4 + (x->_M_color == std::_S_red ? 1 : 0));
    out.push_back((long)n->_M_valptr()->second);
    dump_std(x->_M_left, out);
    dump_std(x->_M_right, out);
}

static void dump_c(const sv_rbtree* t, int x, std::vector<long>& out) {
    if (x < 0) {
        out.push_back(-1);
        return;
    }
    out.push_back((long)t->n[x].key * 4 + (t->n[x].red ? 1 : 0));
    out.push_back((long)t->n[x].val);
    dump_c(t, t->n[x].left, out);
    dump_c(t, t->n[x].right, out);
}

static bool same_state(Map& m, const sv_rbtree* t) {
    std::vector<long> a, b;
    dump_std(m.end()._M_node->_M_parent, a);
    dump_c(t, t->n[0].parent, b);
    if (a != b || m.size() != t->count) return false;
    // in-order
    int i = sv_rb_first(t);
    for (auto it = m.begin(); it != m.end(); ++it) {
        if (i < 0 || t->n[i].key != it->first || t->n[i].val != it->second) return false;
        i = sv_rb_next(t, i);
    }
    if (i >= 0) return false;
    // leftmost / rightmost bookkeeping
    if (m.size()) {
        if (t->n[t->n[0].left].key != m.begin()->first) return false;
        if (t->n[t->n[0].right].key != std::prev(m.end())->first) return false;
    }
    return true;
}

int main() {
    std::mt19937 rng(20260929);
    long ops = 0, bad = 0, nonfound_alive = 0;
    for (int seq = 0; seq < 4000 && bad == 0; ++seq) {
        std::fill(g_exp.begin(), g_exp.end(), 0);
        Map m;
        sv_rbtree t;
        sv_rb_init(&t);
        const unsigned range = 8 + rng() % 50;
        const int nops = 50 + rng() % 250;
        for (int op = 0; op < nops && bad == 0; ++op) {
            const unsigned k = rng() % range;
            const int kind = rng() % 100;
            ++ops;
            if (g_exp[k] && kind < 90) continue; // only live keys are looked up / inserted
            if (kind < 40) { // operator[] insert-if-absent, then assign
                if (!m.count(k)) {
                    const unsigned v = rng() % 1000;
                    m[k] = v;
                    const int nd = sv_rb_index(&t, k, c_less, nullptr);
                    t.n[nd].val = v;
                }
            }
            else if (kind < 60) { // find
                const bool f = m.count(k) != 0;
                const int nd = sv_rb_find(&t, k, c_less, nullptr);
                if (f != (nd >= 0)) { ++bad; std::printf("seq %d op %d: find(%u) std=%d c=%d\n", seq, op, k, (int)f, nd >= 0); }
                if (f && nd >= 0 && m.at(k) != t.n[nd].val) ++bad;
                if (!f) ++nonfound_alive;
            }
            else if (kind < 78) { // erase(key)
                const size_t e = m.erase(k);
                const unsigned ce = sv_rb_erase_key(&t, k, c_less, nullptr);
                if (e != ce) { ++bad; std::printf("seq %d op %d: erase(%u) std=%zu c=%u\n", seq, op, k, e, ce); }
            }
            else if (kind < 90) { // a key expires (its object is destroyed)
                g_exp[k] = 1;
            }
            else { // graph_node::update_connections(): map = std::map(sorted range of live keys)
                std::vector<std::pair<unsigned, unsigned>> v;
                for (unsigned q = 0; q < range; ++q) {
                    if (!g_exp[q] && rng() % 3 == 0) v.emplace_back(q, rng() % 100);
                }
                m = Map(v.begin(), v.end());
                sv_rb_clear(&t);
                for (auto& p : v) sv_rb_insert_at_end(&t, p.first, p.second, c_less, nullptr);
            }
            if (!same_state(m, &t)) {
                ++bad;
                std::printf("seq %d op %d (kind %d key %u): structure mismatch\n", seq, op, kind, k);
            }
        }
        sv_rb_free(&t);
    }
    std::printf("rbtree_test: %ld operations, %ld mismatches (%ld lookups of live absent-or-lost keys)\n", ops, bad, nonfound_alive);
    return bad != 0;
}
