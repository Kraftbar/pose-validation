// Reference observer for the REAL installed stella_vslam bow_database.
// No substituted keyframe/database/scoring implementation and no library patch.
// Protected methods are exposed by inheritance only to observe intermediates.
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/bow_database.h>
#include <stella_vslam/data/bow_vocabulary.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

using namespace stella_vslam;
using namespace stella_vslam::data;
class observed_db : public bow_database {
public:
    observed_db() : bow_database(nullptr) {} // FBoW scoring ignores vocabulary.
    using bow_database::compute_num_common_words;
    using bow_database::compute_scores;
    void snapshot(std::ostream& out) const {
        std::map<unsigned, std::vector<unsigned>> rows;
        for (const auto& b : keyfrms_in_node_) {
            auto& r = rows[b.first];
            for (const auto& k : b.second) r.push_back(k->id_);
        }
        out << "S " << rows.size() << '\n';
        for (const auto& r : rows) {
            out << "W " << r.first << ' ' << r.second.size();
            for (unsigned k : r.second) out << ' ' << k;
            out << '\n';
        }
    }
};
static std::string bits(float v) {
    uint32_t u; std::memcpy(&u, &v, sizeof(u));
    std::ostringstream s; s << std::hex << std::setw(8) << std::setfill('0') << u;
    return s.str();
}
static float read_float(std::istream& in) {
    std::string s; in >> s; size_t n;
    float v = std::stof(s, &n);
    if (n != s.size()) throw std::runtime_error("invalid float");
    return v;
}
int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::runtime_error("usage: dump_bow_db commands canonical raw_order");
        std::ifstream in(argv[1]); std::ofstream out(argv[2]), raw(argv[3]);
        if (!in || !out || !raw) throw std::runtime_error("cannot open fixture files");
        spdlog::set_level(spdlog::level::off);
        observed_db db;
        std::map<unsigned, std::shared_ptr<keyframe>> keys;
        char op;
        size_t queries = 0;
        while (in >> op) {
            if (op == 'K') {
                unsigned id, n; in >> id >> n;
                if (keys.count(id)) throw std::runtime_error("duplicate fixture keyframe ID");
                bow_vector v;
                for (unsigned i = 0; i < n; ++i) {
                    unsigned word; in >> word;
                    float weight = read_float(in);
                    v[word] = weight;
                }
                frame_observation obs{};
                keys[id] = keyframe::make_keyframe(id, 0, Mat44_t::Identity(),
                    nullptr, nullptr, obs, v, bow_feature_vector{});
            }
            else if (op == 'A' || op == 'E') {
                unsigned id; in >> id;
                if (op == 'A') db.add_keyframe(keys.at(id));
                else db.erase_keyframe(keys.at(id));
            }
            else if (op == 'C') db.clear();
            else if (op == 'S') db.snapshot(out);
            else if (op == 'Q') {
                unsigned qid, id, nr; in >> qid >> id;
                float minimum = read_float(in), ratio = read_float(in);
                in >> nr;
                std::set<std::shared_ptr<keyframe>> reject;
                for (unsigned i = 0; i < nr; ++i) { unsigned k; in >> k; reject.insert(keys.at(k)); }
                const auto& query = keys.at(id)->bow_vec_;
                const auto counts = db.compute_num_common_words(query, reject);
                unsigned maximum = 0;
                for (const auto& p : counts) maximum = std::max(maximum, p.second);
                unsigned threshold = static_cast<unsigned>(ratio * maximum);
                float best = minimum, unused;
                const auto accepted_scores = db.compute_scores(counts, query, threshold, minimum, best);
                const auto all_scores = db.compute_scores(counts, query, threshold,
                    -std::numeric_limits<float>::max(), unused);
                const auto candidates = db.acquire_keyframes(query, minimum, ratio, reject);
                std::set<unsigned> accepted;
                raw << qid << ' ' << candidates.size();
                for (const auto& k : candidates) { accepted.insert(k->id_); raw << ' ' << k->id_; }
                raw << '\n';
                if (accepted.size() != accepted_scores.size()) throw std::runtime_error("observer membership disagreement");
                for (const auto& s : accepted_scores)
                    if (!accepted.count(s.first->id_)) throw std::runtime_error("observer score disagreement");
                out << "Q " << qid << ' ' << maximum << ' ' << threshold << ' ' << bits(best)
                    << ' ' << counts.size() << ' ' << accepted.size() << '\n';
                std::map<unsigned, std::shared_ptr<keyframe>> ordered;
                for (const auto& p : counts) ordered[p.first->id_] = p.first;
                for (const auto& p : ordered) {
                    auto s = all_scores.find(p.second);
                    out << "R " << p.first << ' ' << counts.at(p.second) << ' ' << (s != all_scores.end())
                        << ' ' << bits(s != all_scores.end() ? s->second : 0.0f)
                        << ' ' << accepted.count(p.first) << '\n';
                }
                ++queries;
            }
            else throw std::runtime_error("unknown command");
            if (!in) throw std::runtime_error("truncated command");
        }
        if (!queries || !out || !raw) throw std::runtime_error("empty or failed fixture");
        std::cerr << "queries=" << queries << '\n';
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
