// stella_port reference tool, module 4a: map data model + monocular
// initial-map creation dumper.
//
// Clean-room: written against stella_vslam's own public API (system.h,
// config.h, data/frame.h, data/keyframe.h, data/landmark.h,
// data/map_database.h, initialize/perspective.h,
// optimize/global_bundle_adjuster.h -- all BSD-2/public classes) and the
// BSD-2 example utility tum_rgbd_util.h/.cc, same pattern as module 3's
// dump_stella_init.cc (this file reuses its matcher/reset loop verbatim
// up to the first successful initialize::perspective::initialize() call).
// Nothing here is derived from orb_port/ or ORB-SLAM2.
//
// module::initializer::create_map_for_monocular() itself is a PRIVATE
// method of an internal (tracking_module-owned) module::initializer
// instance -- not reachable directly. Every individual step it performs
// is reachable through PUBLIC API, though (data::keyframe::make_keyframe,
// data::landmark's public constructor/connect_to_keyframe/
// compute_descriptor/update_mean_normal_and_obs_scale_variance,
// data::graph_node's public spanning-tree setters, data::map_database's
// public add_keyframe/add_landmark, and optimize::global_bundle_adjuster::
// optimize_for_initialization) -- this tool calls those real methods
// itself, in create_map_for_monocular's exact program order (read from
// module/initializer.cc), which gives genuine library ground truth (not
// reimplemented map/BA math) without patching the shared reference build
// other module workers depend on staying unpatched. The one private
// helper this tool DOES reimplement is module::initializer::scale_map(),
// which has no interesting math of its own (translation *= scalar,
// landmark positions *= scalar, then the same public
// update_mean_normal_and_obs_scale_variance() this tool already calls
// elsewhere) -- see the header comment above scale_map_here() below.
//
// Dumps per sequence, matching the C port's two-stage harness:
//   - runs/stella_port/reference_map_init/<seq>/{keyframes,landmarks}_pre.tsv:
//     state right after the landmark-creation loop, before global BA.
//   - .../keyframes_postba.tsv + landmarks_postba_pos.tsv: curr_keyfrm's
//     pose and every landmark's position immediately after a REAL
//     global_bundle_adjuster::optimize_for_initialization() call (BEFORE
//     scale_map_here()) -- this is the injected BA-result FIXTURE the
//     C-port harness feeds into sv_map_apply_post_ba(), since the real
//     g2o BA is a separate, concurrently-developed module (sv_g2o*), not
//     ported here.
//   - .../{keyframes,landmarks}_post.tsv + scale.tsv: final state, after
//     BA AND scale_map_here() -- the harness's POST-BA comparison target
//     (what create_map_for_monocular computes AFTER the BA call returns:
//     mean normal, ORB scale variance, median-depth scaling, the
//     wrong-init verdict -- see sv_map.h).
//   - .../init_state.tsv: ref_frame_id, cur_frame_id, num_kp_ref, num_kp_cur.
//   - .../matches.tsv: ref_idx, cur_idx, is_triangulated, tri_pt hex x/y/z
//     (module 3's already-exact initializer output -- rot_ref_to_cur/
//     trans_ref_to_cur go in init_state.tsv; this is the harness's INPUT,
//     not something it re-derives).
//
// Determinism: run twice, diff (tools/dump_stella_map_init.py does this).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/match/area.h"
#include "stella_vslam/initialize/perspective.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string fmt_hex(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}

std::string vec3_hex(const stella_vslam::Vec3_t& v) {
    std::ostringstream oss;
    oss << fmt_hex(v(0)) << ',' << fmt_hex(v(1)) << ',' << fmt_hex(v(2));
    return oss.str();
}

std::string mat44_hex_colmajor(const stella_vslam::Mat44_t& m) {
    std::ostringstream oss;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            if (c || r) oss << ',';
            oss << fmt_hex(m(r, c));
        }
    }
    return oss.str();
}

std::string desc_hex(const cv::Mat& row) {
    static const char* hexd = "0123456789abcdef";
    std::string out;
    out.resize(64);
    const unsigned char* p = row.ptr<unsigned char>(0);
    for (int i = 0; i < 32; ++i) {
        out[2 * i] = hexd[(p[i] >> 4) & 0xf];
        out[2 * i + 1] = hexd[p[i] & 0xf];
    }
    return out;
}

// module::initializer::scale_map() (private) -- see file header comment.
// Pure orchestration: no new math beyond scalar multiplication, calling
// only the same public setters/recompute this tool already exercises.
void scale_map_here(const std::shared_ptr<stella_vslam::data::keyframe>& init_keyfrm,
                    const std::shared_ptr<stella_vslam::data::keyframe>& curr_keyfrm,
                    double scale) {
    stella_vslam::Mat44_t cam_pose_cw = curr_keyfrm->get_pose_cw();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    curr_keyfrm->set_pose_cw(cam_pose_cw);

    const auto landmarks = init_keyfrm->get_landmarks();
    for (const auto& lm : landmarks) {
        if (!lm) continue;
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
        lm->update_mean_normal_and_obs_scale_variance();
    }
}

void dump_keyframes(std::ofstream& f,
                    const std::shared_ptr<stella_vslam::data::keyframe>& init_keyfrm,
                    const std::shared_ptr<stella_vslam::data::keyframe>& curr_keyfrm) {
    for (const auto& kf : {init_keyfrm, curr_keyfrm}) {
        auto parent = kf->graph_node_->get_spanning_parent();
        auto children = kf->graph_node_->get_spanning_children();
        std::ostringstream childs;
        bool first = true;
        for (const auto& c : children) {
            if (!first) childs << ',';
            first = false;
            childs << c->id_;
        }
        f << kf->id_ << '\t' << mat44_hex_colmajor(kf->get_pose_cw()) << '\t'
          << (parent ? (long)parent->id_ : -1L) << '\t'
          << kf->graph_node_->get_spanning_root()->id_ << '\t'
          << childs.str() << '\n';
    }
}

void dump_landmarks(std::ofstream& f, const std::vector<std::shared_ptr<stella_vslam::data::landmark>>& lms) {
    for (const auto& lm : lms) {
        if (!lm) continue; // keyframe::get_landmarks() includes nullptr entries
                            // for keypoints without an associated landmark.
        std::ostringstream obs;
        bool first = true;
        for (const auto& kf_idx : lm->get_observations()) {
            if (!first) obs << ',';
            first = false;
            obs << kf_idx.first.lock()->id_ << ':' << kf_idx.second;
        }
        f << lm->id_ << '\t' << vec3_hex(lm->get_pos_in_world()) << '\t'
          << desc_hex(lm->get_descriptor()) << '\t'
          << vec3_hex(lm->get_obs_mean_normal()) << '\t'
          << fmt_hex(lm->get_min_valid_distance()) << '\t'
          << fmt_hex(lm->get_max_valid_distance()) << '\t'
          << lm->get_num_observed() << '\t' << lm->get_num_observable() << '\t'
          << lm->get_ref_keyframe()->id_ << '\t' << obs.str() << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "usage: dump_stella_map_init <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir>\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string config_path = argv[2];
    const std::string seq_dir = argv[3];
    const std::string out_dir = argv[4];

    auto cfg = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup_single_threaded(true);
    auto* tracker = slam->get_tracker();

    tum_rgbd_sequence sequence(seq_dir);
    const auto frames = sequence.get_frames();

    // Params (deterministic config: defaults except use_fixed_seed=true).
    const unsigned int num_ransac_iters = 100;
    const unsigned int min_num_valid_pts = 50;
    const unsigned int min_num_triangulated_pts = 50;
    const float parallax_deg_thr = 1.0f;
    const float reproj_err_thr = 4.0f;
    const bool use_fixed_seed = true;
    const unsigned int num_ba_iters = 100;
    const float gain_threshold = 1e-5f;
    const double scaling_factor = 1.0;

    stella_vslam::data::frame ref_frm;
    bool have_ref = false;
    std::vector<cv::Point2f> prev_matched;

    bool done = false;
    for (size_t i = 0; i < frames.size() && !done; ++i) {
        const auto& f = frames[i];
        cv::Mat img = cv::imread(f.rgb_img_path_, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "failed to read " << f.rgb_img_path_ << "\n";
            return 2;
        }
        slam->feed_monocular_frame(img, f.timestamp_);
        slam->synchronize_background_modules();
        if (!tracker) continue;
        const auto& curr_frm = tracker->curr_frm_;

        if (!have_ref) {
            ref_frm = stella_vslam::data::frame(curr_frm);
            prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
            for (size_t k = 0; k < prev_matched.size(); ++k) {
                prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
            }
            have_ref = true;
            continue;
        }

        stella_vslam::match::area matcher(0.9, true);
        std::vector<int> local_init_matches(ref_frm.frm_obs_.undist_keypts_.size(), -1);
        auto local_prev_matched = prev_matched;
        unsigned int num_matches = matcher.match_in_consistent_area(
            ref_frm, const_cast<stella_vslam::data::frame&>(curr_frm), local_prev_matched, local_init_matches, 100);

        if (num_matches < min_num_valid_pts) {
            ref_frm = stella_vslam::data::frame(curr_frm);
            prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
            for (size_t k = 0; k < prev_matched.size(); ++k) {
                prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
            }
            continue;
        }

        auto persp = stella_vslam::initialize::perspective(ref_frm, num_ransac_iters, min_num_triangulated_pts,
                                                            min_num_valid_pts, parallax_deg_thr, reproj_err_thr, use_fixed_seed);
        // NOTE: perspective::initialize() takes local_init_matches only to
        // read ref_cur_matches_ from it -- it does NOT invalidate
        // non-triangulated entries itself; that invalidation loop belongs
        // to module::initializer::create_map_for_monocular() (see below,
        // right after a successful initialize() call).
        bool ok = persp.initialize(const_cast<stella_vslam::data::frame&>(curr_frm), local_init_matches);

        if (!ok) {
            ref_frm = stella_vslam::data::frame(curr_frm);
            prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
            for (size_t k = 0; k < prev_matched.size(); ++k) {
                prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
            }
            continue;
        }

        // ---- success: replicate create_map_for_monocular from here ----
        auto init_triangulated_pts = persp.get_triangulated_pts();
        auto is_triangulated = persp.get_triangulated_flags();
        stella_vslam::data::frame& curr_frm_mut = const_cast<stella_vslam::data::frame&>(curr_frm);

        // create_map_for_monocular's own invalidation loop: matches lacking
        // a triangulated point never become landmarks (module/initializer.cc).
        for (unsigned int i = 0; i < local_init_matches.size(); ++i) {
            if (local_init_matches.at(i) < 0) continue;
            if (is_triangulated.at(i)) continue;
            local_init_matches.at(i) = -1;
        }

        ref_frm.set_pose_cw(stella_vslam::Mat44_t::Identity());
        stella_vslam::Mat44_t cam_pose_cw = stella_vslam::Mat44_t::Identity();
        cam_pose_cw.block<3, 3>(0, 0) = persp.get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = persp.get_translation_ref_to_cur();
        curr_frm_mut.set_pose_cw(cam_pose_cw);

        stella_vslam::data::map_database map_db(15);
        auto init_keyfrm = stella_vslam::data::keyframe::make_keyframe(map_db.next_keyframe_id_++, ref_frm);
        auto curr_keyfrm = stella_vslam::data::keyframe::make_keyframe(map_db.next_keyframe_id_++, curr_frm_mut);
        curr_keyfrm->graph_node_->set_spanning_parent(init_keyfrm);
        init_keyfrm->graph_node_->add_spanning_child(curr_keyfrm);
        init_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
        curr_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
        map_db.add_spanning_root(init_keyfrm);

        map_db.add_keyframe(init_keyfrm);
        map_db.add_keyframe(curr_keyfrm);

        std::vector<std::shared_ptr<stella_vslam::data::landmark>> lms;
        for (unsigned int init_idx = 0; init_idx < local_init_matches.size(); ++init_idx) {
            const auto curr_idx = local_init_matches.at(init_idx);
            if (curr_idx < 0) continue;

            auto lm = std::make_shared<stella_vslam::data::landmark>(
                map_db.next_landmark_id_++, init_triangulated_pts.at(init_idx), curr_keyfrm);
            lm->connect_to_keyframe(init_keyfrm, init_idx);
            lm->connect_to_keyframe(curr_keyfrm, curr_idx);
            lm->compute_descriptor();
            lm->update_mean_normal_and_obs_scale_variance();

            curr_frm_mut.add_landmark(lm, curr_idx);
            map_db.add_landmark(lm);
            lms.push_back(lm);
        }

        // ---- dump PRE-BA ----
        {
            std::ofstream kf_f(out_dir + "/keyframes_pre.tsv");
            std::ofstream lm_f(out_dir + "/landmarks_pre.tsv");
            kf_f << "kf_id\tpose_cw_hex16\tspanning_parent\tspanning_root\tspanning_children\n";
            lm_f << "lm_id\tpos_w_hex\tdescriptor_hex\tmean_normal_hex\tmin_valid_dist_hex\tmax_valid_dist_hex\tnum_observed\tnum_observable\tref_keyfrm_id\tobservations\n";
            dump_keyframes(kf_f, init_keyfrm, curr_keyfrm);
            dump_landmarks(lm_f, lms);
        }

        // ---- REAL global bundle adjustment (public API) ----
        const auto gba = stella_vslam::optimize::global_bundle_adjuster(num_ba_iters, true, false);
        std::vector<std::shared_ptr<stella_vslam::data::keyframe>> keyfrms{init_keyfrm, curr_keyfrm};
        gba.optimize_for_initialization(keyfrms, lms, {}, gain_threshold, false);

        // ---- dump POST-BA, PRE-SCALE (the C port's injected fixture: the
        // real global_bundle_adjuster is a separate module -- its output,
        // taken here straight from the real library, is what the harness
        // feeds into sv_map_apply_post_ba() as already-computed input) ----
        {
            std::ofstream kf_f(out_dir + "/keyframes_postba.tsv");
            std::ofstream lm_f(out_dir + "/landmarks_postba_pos.tsv");
            kf_f << "kf_id\tpose_cw_hex16\n";
            for (const auto& kf : {init_keyfrm, curr_keyfrm}) {
                kf_f << kf->id_ << '\t' << mat44_hex_colmajor(kf->get_pose_cw()) << '\n';
            }
            lm_f << "lm_id\tpos_w_hex\n";
            for (const auto& lm : lms) {
                lm_f << lm->id_ << '\t' << vec3_hex(lm->get_pos_in_world()) << '\n';
            }
        }

        // ---- module::initializer::scale_map() (see header comment) ----
        float median_scale = init_keyfrm->compute_median_depth(true);
        double inv_median_scale = 1.0 / (double)median_scale;
        bool reset_wrong = (curr_keyfrm->get_num_tracked_landmarks(1) < min_num_triangulated_pts) && (median_scale < 0);
        std::string verdict = reset_wrong ? "wrong_init" : "success";
        double applied_scale = reset_wrong ? 0.0 : (inv_median_scale * scaling_factor);
        if (!reset_wrong) {
            scale_map_here(init_keyfrm, curr_keyfrm, applied_scale);
        }
        curr_frm_mut.set_pose_cw(curr_keyfrm->get_pose_cw());

        // ---- dump POST-BA ----
        {
            std::ofstream kf_f(out_dir + "/keyframes_post.tsv");
            std::ofstream lm_f(out_dir + "/landmarks_post.tsv");
            std::ofstream sc_f(out_dir + "/scale.tsv");
            kf_f << "kf_id\tpose_cw_hex16\tspanning_parent\tspanning_root\tspanning_children\n";
            lm_f << "lm_id\tpos_w_hex\tdescriptor_hex\tmean_normal_hex\tmin_valid_dist_hex\tmax_valid_dist_hex\tnum_observed\tnum_observable\tref_keyfrm_id\tobservations\n";
            dump_keyframes(kf_f, init_keyfrm, curr_keyfrm);
            dump_landmarks(lm_f, init_keyfrm->get_landmarks());
            sc_f << "median_scale_hex\tinv_median_scale_hex\tapplied_scale_hex\tverdict\n";
            sc_f << fmt_hex(median_scale) << '\t' << fmt_hex(inv_median_scale) << '\t' << fmt_hex(applied_scale) << '\t' << verdict << '\n';
        }

        // ---- dump init_state.tsv / matches.tsv (module-3 output, taken
        // as exact input by the C-port harness -- see file header) ----
        {
            std::ofstream is_f(out_dir + "/init_state.tsv");
            is_f << "ref_frame_id\tcur_frame_id\tnum_kp_ref\tnum_kp_cur\trot_ref_to_cur_hex9\ttrans_ref_to_cur_hex3\n";
            is_f << ref_frm.id_ << '\t' << curr_frm.id_ << '\t' << ref_frm.frm_obs_.undist_keypts_.size() << '\t'
                 << curr_frm.frm_obs_.undist_keypts_.size() << '\t';
            auto R = persp.get_rotation_ref_to_cur();
            auto t = persp.get_translation_ref_to_cur();
            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r)
                    is_f << fmt_hex(R(r, c)) << (c == 2 && r == 2 ? "" : ",");
            is_f << '\t' << vec3_hex(t) << '\n';

            std::ofstream mf(out_dir + "/matches.tsv");
            mf << "ref_idx\tcur_idx\tis_triangulated\tx_hex\ty_hex\tz_hex\n";
            for (unsigned int ref_idx = 0; ref_idx < local_init_matches.size(); ++ref_idx) {
                int curr_idx = local_init_matches[ref_idx];
                if (curr_idx < 0) continue;
                bool tri = is_triangulated.at(ref_idx);
                // init_triangulated_pts entries for non-triangulated matches
                // are never written by initialize::base::triangulate() and
                // are left at whatever eigen_alloc_vector's default Vec3_t
                // construction leaves them (uninitialized, not zeroed) --
                // module::initializer::create_map_for_monocular() itself
                // never reads them either (its own is_triangulated loop
                // invalidates those matches first). Dump 0 for determinism;
                // the C-port harness only ever consumes tri==1 rows anyway.
                stella_vslam::Vec3_t p = tri ? init_triangulated_pts.at(ref_idx) : stella_vslam::Vec3_t::Zero();
                mf << ref_idx << '\t' << curr_idx << '\t' << (tri ? 1 : 0) << '\t'
                   << fmt_hex(p(0)) << '\t' << fmt_hex(p(1)) << '\t' << fmt_hex(p(2)) << '\n';
            }
        }

        std::cerr << "dump_stella_map_init: success at ref_frame_id=" << ref_frm.id_
                  << " cur_frame_id=" << curr_frm.id_ << " (" << lms.size() << " landmarks), verdict=" << verdict << "\n";
        done = true;
    }

    if (!done) {
        std::cerr << "dump_stella_map_init: sequence never initialized\n";
        return 3;
    }
    return 0;
}
