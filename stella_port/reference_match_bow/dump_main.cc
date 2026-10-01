// Observer calling the real installed stella_vslam match::bow_tree methods.
// No native source patch or substitute matcher.
#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/match/bow_tree.h>
#include <spdlog/spdlog.h>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace stella_vslam;
using namespace stella_vslam::data;
int main(int argc, char **argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: dump_match_bow commands output");
        std::ifstream in(argv[1]); std::ofstream out(argv[2]);
        if (!in || !out) throw std::runtime_error("cannot open files");
        spdlog::set_level(spdlog::level::off);
        map_database db(15);
        std::map<unsigned, std::shared_ptr<keyframe>> keys;
        char op; size_t queries = 0;
        while (in >> op) {
            if (op == 'F') {
                unsigned id, n; in >> id >> n;
                if (keys.count(id)) throw std::runtime_error("duplicate frame");
                frame_observation obs{};
                obs.undist_keypts_.resize(n);
                obs.descriptors_ = cv::Mat(n, 32, CV_8UC1);
                std::vector<uint64_t> tokens(n); std::vector<int> erased(n);
                for (unsigned i=0; i<n; ++i) {
                    std::string angle, hex;
                    in >> angle >> hex >> tokens[i] >> erased[i];
                    obs.undist_keypts_[i].angle = std::stof(angle);
                    if (hex.size()!=64 || tokens[i]>UINT64_C(4294967296)) throw std::runtime_error("invalid feature");
                    for (unsigned j=0; j<32; ++j)
                        obs.descriptors_.at<uint8_t>(i,j) = std::stoul(hex.substr(j*2,2),nullptr,16);
                }
                bow_feature_vector feat;
                unsigned nodes; in >> nodes;
                for (unsigned i=0; i<nodes; ++i) {
                    unsigned node, count; in >> node >> count;
                    for (unsigned j=0; j<count; ++j) { unsigned idx; in >> idx; feat[node].push_back(idx); }
                    if (!count) feat[node];
                }
                auto k = keyframe::make_keyframe(id, 0, Mat44_t::Identity(), nullptr, nullptr, obs, bow_vector{}, feat);
                std::map<uint64_t,std::shared_ptr<landmark>> lms;
                for (unsigned i=0; i<n; ++i) {
                    if (!tokens[i]) continue;
                    auto &lm = lms[tokens[i]];
                    if (!lm) {
                        lm = std::make_shared<landmark>(static_cast<unsigned>(tokens[i]-1), Vec3_t::Zero(), k);
                        if (erased[i]) lm->prepare_for_erasing(&db);
                    }
                    if (lm->will_be_erased() != bool(erased[i])) throw std::runtime_error("inconsistent alias state");
                    k->add_landmark(lm,i);
                }
                keys[id] = k;
            }
            else if (op == 'Q') {
                unsigned id,a,b; int mode,orient; std::string ratio;
                in >> id >> mode >> a >> b >> ratio >> orient;
                match::bow_tree matcher(std::stof(ratio),orient);
                std::vector<std::shared_ptr<landmark>> result;
                unsigned count;
                if (mode == 0) {
                    frame f; f.frm_obs_ = keys.at(b)->frm_obs_; f.bow_feat_vec_ = keys.at(b)->bow_feat_vec_;
                    // Pre-filled output verifies the real method replaces it.
                    result = keys.at(a)->get_landmarks();
                    count = matcher.match_frame_and_keyframe(keys.at(a), f, result);
                }
                else if (mode == 1) count = matcher.match_keyframes(keys.at(a),keys.at(b),result);
                else throw std::runtime_error("bad mode");
                out << "Q " << id << ' ' << count << ' ' << result.size();
                for (auto &lm : result) out << ' ' << (lm ? uint64_t(lm->id_)+1 : 0);
                out << '\n'; ++queries;
            }
            else throw std::runtime_error("bad command");
            if (!in) throw std::runtime_error("truncated command");
        }
        if (!queries || !out) throw std::runtime_error("empty output");
        std::cout << queries << " queries\n";
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 2; }
}
