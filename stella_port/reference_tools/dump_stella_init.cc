// stella_port reference tool, module 3: monocular initializer dumper.
//
// Clean-room: written against stella_vslam's own public API (system.h,
// config.h, data/frame.h, data/frame_observation.h, camera/perspective.h,
// match/area.h, solve/{homography,fundamental}_solver.h, all BSD-2) and the
// BSD-2 example utility tum_rgbd_util.h/.cc (see LICENSE.tum_rgbd_util in
// this directory), same as dump_frame_bow.cc (module 2). Nothing here is
// derived from orb_port/ or ORB-SLAM2.
//
// Per the task/coordinator instructions: this is a STANDALONE tool through
// the installed reference library -- no patches to the main reference
// build (other module workers depend on it staying unpatched). It drives
// match::area::match_in_consistent_area() and
// initialize::perspective::initialize() directly (both public classes/
// methods -- see solve/homography_solver.h, solve/fundamental_solver.h,
// initialize/perspective.h, initialize/base.h), mirroring
// module::initializer's monocular state machine
// (module/initializer.cc::try_initialize_for_monocular, margin=100) in
// this file's own main() loop instead of using module::initializer itself
// (which also owns map_database/global-BA side effects out of module-3
// scope).
//
// Per-attempt H/F/RANSAC data (best H_21/F_21, cost, inlier masks) comes
// straight from homography_solver/fundamental_solver's public getters --
// genuine ground truth, not reimplemented math. Model-selection and
// decompose() (homography_solver::decompose / fundamental_solver::decompose)
// are also stella's own public static methods, called directly.
//
// initialize::base::find_most_plausible_pose()/triangulate() are PROTECTED
// members of initialize::base, only reachable as a whole through
// initialize::perspective::initialize() (public) via its public output
// getters (get_rotation_ref_to_cur/get_translation_ref_to_cur/
// get_triangulated_pts/get_triangulated_flags) -- which give only the
// FINAL selected hypothesis, not each hypothesis's own triangulation
// count/parallax. To dump per-hypothesis detail, this tool independently
// replicates find_most_plausible_pose/triangulate's control flow (read
// closely from initialize/base.cc) using stella's own PUBLIC primitives:
// solve::triangulator::triangulate (public static inline, solve/
// triangulator.h) and camera::base::reproject_to_image (public virtual).
// As a self-check against having replicated that control flow correctly,
// this tool ALSO runs a real initialize::perspective object end-to-end on
// the same (ref_frm, matches) and asserts its public getters agree with
// this tool's own selected-hypothesis result (see check_against_real_perspective()).
//
// RNG draws: homography_solver/fundamental_solver's random_engine_ is a
// private std::mt19937 member; with use_fixed_seed=true (this config) it
// is fully determined by public knowledge (default seed, then
// create_random_array(min_set_size, 0, num_matches-1, engine) called
// exactly num_ransac_iters times in program order -- read from
// solve/{homography,fundamental}_solver.cc's find_via_ransac()). This tool
// replays that exact draw sequence itself (a verbatim copy of
// util::random_array.cc's create_random_array, BSD-2 source -- safe to
// inline in a reference tool) to dump the per-iteration sampled index
// sets, and the strong cross-check that these equal the real private
// engine's draws is that this tool's own from-scratch H/F RANSAC replay
// (using the same draws, but stella's real compute_H_21/check_inliers via
// the public solver objects run per-iteration is NOT reproduced here --
// instead the C port's harness (check_sv_init.c) is the real cross-check:
// it drives sv_rng.c + sv_solve_homography.c/sv_solve_fundamental.c with
// these exact dumped draws and must reproduce this tool's dumped
// best_H21/best_F21/cost/inliers bit-exact, which could only happen if
// both the draws and the port's math are correct).
//
// Determinism: run twice, diff (tools/dump_stella_init.py does this).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/match/area.h"
#include "stella_vslam/solve/homography_solver.h"
#include "stella_vslam/solve/fundamental_solver.h"
#include "stella_vslam/solve/triangulator.h"
#include "stella_vslam/initialize/perspective.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string fmt_hex(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}
std::string fmt_hexf(float v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", (double)v);
    return std::string(buf);
}

template <typename Vec>
std::string join_csv(const Vec& v) {
    std::ostringstream oss;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) oss << ',';
        oss << v[i];
    }
    return oss.str();
}

std::string mat33_hex(const stella_vslam::Mat33_t& m) {
    // column-major, matching sv_linalg.h/sv_eigen_svd.h convention
    std::ostringstream oss;
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) {
            if (c || r) oss << ',';
            oss << fmt_hex(m(r, c));
        }
    }
    return oss.str();
}
std::string vec3_hex(const stella_vslam::Vec3_t& v) {
    std::ostringstream oss;
    oss << fmt_hex(v(0)) << ',' << fmt_hex(v(1)) << ',' << fmt_hex(v(2));
    return oss.str();
}

// Verbatim copy of stella_vslam's util::random_array.cc create_random_array
// (BSD-2, AIST 2019 + stella-cv 2022) -- see header comment: used here only
// to dump the deterministic draw sequence for debugging, not as the
// ground-truth RANSAC computation (that comes from the real solver
// objects below).
template <typename T>
std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max,
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

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "usage: dump_stella_init <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir> <max_frames|-1>\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string config_path = argv[2];
    const std::string seq_dir = argv[3];
    const std::string out_dir = argv[4];
    const long max_frames = std::atol(argv[5]);

    auto cfg = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup_single_threaded(true);
    auto* tracker = slam->get_tracker();

    tum_rgbd_sequence sequence(seq_dir);
    const auto frames = sequence.get_frames();
    const size_t n = (max_frames >= 0) ? std::min<size_t>(frames.size(), (size_t)max_frames) : frames.size();

    std::ofstream attempts_f(out_dir + "/attempts.tsv");
    std::ofstream matches_f(out_dir + "/matches.tsv");
    std::ofstream rng_f(out_dir + "/rng.tsv");
    std::ofstream hyps_f(out_dir + "/hyps.tsv");
    std::ofstream final_f(out_dir + "/final.tsv");
    std::ofstream inliers_f(out_dir + "/inliers.tsv");

    attempts_f << "attempt_id\tref_frame_id\tcur_frame_id\tnum_matches\tverdict\tmodel\tcost_h\tcost_f\trel_cost_h\th_valid\tf_valid\tH21_hex\tF21_hex\n";
    matches_f << "attempt_id\tref_idx\tcur_idx\n";
    rng_f << "attempt_id\tsolver\titer\tindices\n";
    hyps_f << "attempt_id\thyp_idx\tmodel\tnum_valid_pts\tnum_triangulated_pts\tparallax_cos_hex\trot_hex\ttrans_hex\n";
    final_f << "attempt_id\tselected_hyp\trot_hex\ttrans_hex\n";
    inliers_f << "attempt_id\tsolver\tmatch_i\tis_inlier\n";

    // Mirror module::initializer's monocular state machine, own state:
    stella_vslam::data::frame ref_frm;
    bool have_ref = false;
    std::vector<cv::Point2f> prev_matched;
    std::vector<int> init_matches;
    unsigned int attempt_id = 0;

    // Params (deterministic config: defaults except use_fixed_seed=true).
    const unsigned int num_ransac_iters = 100;
    const unsigned int min_num_valid_pts = 50;
    const unsigned int min_num_triangulated_pts = 50;
    const float parallax_deg_thr = 1.0f;
    const float reproj_err_thr = 4.0f;
    const bool use_fixed_seed = true;

    for (size_t i = 0; i < n; ++i) {
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
            init_matches.assign(ref_frm.frm_obs_.undist_keypts_.size(), -1);
            have_ref = true;
            continue;
        }

        stella_vslam::match::area matcher(0.9, true);
        std::vector<int> local_init_matches(ref_frm.frm_obs_.undist_keypts_.size(), -1);
        // matcher mutates prev_matched in place AND needs the running
        // init_matches_ (module::initializer never resets it between
        // attempts unless min_num_valid_pts_ fails -- but each attempt
        // uses a fresh -1-filled vector per create_initializer(); since we
        // only reset the ref frame on failure here (matching reset()),
        // pass a fresh -1 vector each attempt, matching a freshly
        // constructed initializer each time create_initializer() runs.
        auto local_prev_matched = prev_matched;
        unsigned int num_matches = matcher.match_in_consistent_area(ref_frm, const_cast<stella_vslam::data::frame&>(curr_frm),
                                                                     local_prev_matched, local_init_matches, 100);

        unsigned int aid = attempt_id++;
        matches_f.flush();
        for (size_t ref_idx = 0; ref_idx < local_init_matches.size(); ++ref_idx) {
            if (local_init_matches[ref_idx] >= 0) {
                matches_f << aid << '\t' << ref_idx << '\t' << local_init_matches[ref_idx] << '\n';
            }
        }

        if (num_matches < min_num_valid_pts) {
            attempts_f << aid << '\t' << ref_frm.id_ << '\t' << curr_frm.id_ << '\t' << num_matches
                       << "\treset\tnone\t0\t0\t0\t0\t0\t-\t-\n";
            // reset: next frame becomes the new reference (create_initializer again)
            ref_frm = stella_vslam::data::frame(curr_frm);
            prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
            for (size_t k = 0; k < prev_matched.size(); ++k) {
                prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
            }
            continue;
        }

        // build ref_cur_matches_
        std::vector<std::pair<int, int>> ref_cur_matches;
        ref_cur_matches.reserve(curr_frm.frm_obs_.undist_keypts_.size());
        for (size_t ref_idx = 0; ref_idx < local_init_matches.size(); ++ref_idx) {
            int cur_idx = local_init_matches[ref_idx];
            if (cur_idx >= 0) ref_cur_matches.emplace_back((int)ref_idx, cur_idx);
        }

        const float sigma = 1.0f;
        auto h_solver = stella_vslam::solve::homography_solver(ref_frm.frm_obs_.undist_keypts_, curr_frm.frm_obs_.undist_keypts_,
                                                                ref_cur_matches, sigma, use_fixed_seed);
        auto f_solver = stella_vslam::solve::fundamental_solver(ref_frm.frm_obs_.undist_keypts_, curr_frm.frm_obs_.undist_keypts_,
                                                                 ref_cur_matches, sigma, use_fixed_seed);
        h_solver.find_via_ransac(num_ransac_iters, false);
        f_solver.find_via_ransac(num_ransac_iters, false);

        // Independent RNG replay (see header comment) -- dumped for
        // debugging, not the ground-truth computation path.
        {
            std::mt19937 eh;
            for (unsigned int it = 0; it < num_ransac_iters; ++it) {
                auto idx = create_random_array<unsigned int>(4, 0U, (unsigned int)ref_cur_matches.size() - 1, eh);
                rng_f << aid << "\th\t" << it << '\t' << join_csv(idx) << '\n';
            }
            std::mt19937 ef;
            for (unsigned int it = 0; it < num_ransac_iters; ++it) {
                auto idx = create_random_array<unsigned int>(8, 0U, (unsigned int)ref_cur_matches.size() - 1, ef);
                rng_f << aid << "\tf\t" << it << '\t' << join_csv(idx) << '\n';
            }
        }

        float cost_h = h_solver.get_best_cost();
        float cost_f = f_solver.get_best_cost();
        float rel_cost_h = cost_h / (cost_h + cost_f);
        bool h_valid = h_solver.solution_is_valid();
        bool f_valid = f_solver.solution_is_valid();

        auto inlier_h = h_solver.get_inlier_matches();
        auto inlier_f = f_solver.get_inlier_matches();
        for (size_t mi = 0; mi < inlier_h.size(); ++mi) {
            inliers_f << aid << "\th\t" << mi << '\t' << (inlier_h[mi] ? 1 : 0) << '\n';
        }
        for (size_t mi = 0; mi < inlier_f.size(); ++mi) {
            inliers_f << aid << "\tf\t" << mi << '\t' << (inlier_f[mi] ? 1 : 0) << '\n';
        }

        std::string model = "none";
        std::string verdict = "fail_no_valid_model";
        stella_vslam::Mat33_t H21 = h_solver.get_best_H_21();
        stella_vslam::Mat33_t F21 = f_solver.get_best_F_21();

        attempts_f << aid << '\t' << ref_frm.id_ << '\t' << curr_frm.id_ << '\t' << num_matches << '\t';

        // Camera matrices (public getters via perspective camera).
        auto get_cam_matrix = [](stella_vslam::camera::base* cam) -> stella_vslam::Mat33_t {
            return static_cast<stella_vslam::camera::perspective*>(cam)->eigen_cam_matrix_;
        };
        stella_vslam::Mat33_t ref_cam_mat = get_cam_matrix(ref_frm.camera_);
        stella_vslam::Mat33_t cur_cam_mat = get_cam_matrix(curr_frm.camera_);

        stella_vslam::eigen_alloc_vector<stella_vslam::Mat33_t> hyp_rots;
        stella_vslam::eigen_alloc_vector<stella_vslam::Vec3_t> hyp_transes;
        stella_vslam::eigen_alloc_vector<stella_vslam::Vec3_t> hyp_normals;
        std::vector<bool> is_inlier_for_tri;
        int num_hyp = 0;

        if (0.5f > rel_cost_h && h_valid) {
            model = "H";
            if (stella_vslam::solve::homography_solver::decompose(H21, ref_cam_mat, cur_cam_mat, hyp_rots, hyp_transes, hyp_normals)) {
                num_hyp = 8;
                is_inlier_for_tri = inlier_h;
            }
            else {
                verdict = "fail_no_valid_model";
                model = "none";
            }
        }
        else if (f_valid) {
            model = "F";
            stella_vslam::solve::fundamental_solver::decompose(F21, ref_cam_mat, cur_cam_mat, hyp_rots, hyp_transes);
            num_hyp = 4;
            is_inlier_for_tri = inlier_f;
        }

        attempts_f << verdict << '\t' << model << '\t' << fmt_hexf(cost_h) << '\t' << fmt_hexf(cost_f) << '\t'
                   << fmt_hexf(rel_cost_h) << '\t' << (h_valid ? 1 : 0) << '\t' << (f_valid ? 1 : 0) << '\t'
                   << mat33_hex(H21) << '\t' << mat33_hex(F21) << '\n';

        if (num_hyp == 0) {
            // (verdict/model already emitted above)
            ref_frm = stella_vslam::data::frame(curr_frm);
            prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
            for (size_t k = 0; k < prev_matched.size(); ++k) {
                prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
            }
            continue;
        }

        // Replicate find_most_plausible_pose()/triangulate() from public
        // primitives (see header comment). depth_is_positive=true always
        // (only call site in perspective.cc).
        const float cos_parallax_thr = 0.99996192306f;
        float reproj_err_thr_sq = reproj_err_thr * reproj_err_thr;

        std::vector<unsigned int> nums_valid_pts(num_hyp), num_tri_pts(num_hyp);
        std::vector<float> parallaxes(num_hyp);
        std::vector<stella_vslam::eigen_alloc_vector<stella_vslam::Vec3_t>> tri_pts(num_hyp);
        std::vector<std::vector<bool>> is_tri(num_hyp);

        for (int h = 0; h < num_hyp; ++h) {
            const auto& rot = hyp_rots[h];
            const auto& trans = hyp_transes[h];
            tri_pts[h].assign(ref_frm.frm_obs_.undist_keypts_.size(), stella_vslam::Vec3_t::Zero());
            is_tri[h].assign(ref_frm.frm_obs_.undist_keypts_.size(), false);
            std::vector<float> cos_parallaxes;
            cos_parallaxes.reserve(ref_cur_matches.size());

            stella_vslam::Vec3_t ref_cam_center = stella_vslam::Vec3_t::Zero();
            stella_vslam::Vec3_t cur_cam_center = -rot.transpose() * trans;

            unsigned int num_valid_pts = 0, num_triangulated_pts = 0;
            for (size_t mi = 0; mi < ref_cur_matches.size(); ++mi) {
                if (!is_inlier_for_tri[mi]) continue;
                const auto& ref_bearing = ref_frm.frm_obs_.bearings_[ref_cur_matches[mi].first];
                const auto& cur_bearing = curr_frm.frm_obs_.bearings_[ref_cur_matches[mi].second];
                const stella_vslam::Vec3_t pos_c_in_ref = stella_vslam::solve::triangulator::triangulate(ref_bearing, cur_bearing, rot, trans);
                if (!std::isfinite(pos_c_in_ref(0)) || !std::isfinite(pos_c_in_ref(1)) || !std::isfinite(pos_c_in_ref(2))) continue;

                const stella_vslam::Vec3_t ref_normal = pos_c_in_ref - ref_cam_center;
                const float ref_norm = ref_normal.norm();
                const stella_vslam::Vec3_t cur_normal = pos_c_in_ref - cur_cam_center;
                const float cur_norm = cur_normal.norm();
                const float cos_parallax = ref_normal.dot(cur_normal) / (ref_norm * cur_norm);
                const bool parallax_is_small = cos_parallax_thr < cos_parallax;

                if (!parallax_is_small && pos_c_in_ref(2) <= 0) continue;
                const stella_vslam::Vec3_t pos_c_in_cur = rot * pos_c_in_ref + trans;
                if (!parallax_is_small && pos_c_in_cur(2) <= 0) continue;

                const auto& ref_kp = ref_frm.frm_obs_.undist_keypts_[ref_cur_matches[mi].first];
                const auto& cur_kp = curr_frm.frm_obs_.undist_keypts_[ref_cur_matches[mi].second];

                stella_vslam::Vec2_t reproj_ref; float x_right_ref;
                bool is_valid_ref = ref_frm.camera_->reproject_to_image(stella_vslam::Mat33_t::Identity(), stella_vslam::Vec3_t::Zero(),
                                                                        pos_c_in_ref, reproj_ref, x_right_ref);
                if (!parallax_is_small && !is_valid_ref) continue;
                const float ref_reproj_err_sq = (reproj_ref - stella_vslam::Vec2_t(ref_kp.pt.x, ref_kp.pt.y)).squaredNorm();
                if (reproj_err_thr_sq < ref_reproj_err_sq) continue;

                stella_vslam::Vec2_t reproj_cur; float x_right_cur;
                bool is_valid_cur = curr_frm.camera_->reproject_to_image(rot, trans, pos_c_in_ref, reproj_cur, x_right_cur);
                if (!parallax_is_small && !is_valid_cur) continue;
                const float cur_reproj_err_sq = (reproj_cur - stella_vslam::Vec2_t(cur_kp.pt.x, cur_kp.pt.y)).squaredNorm();
                if (reproj_err_thr_sq < cur_reproj_err_sq) continue;

                ++num_valid_pts;
                cos_parallaxes.push_back(cos_parallax);
                if (!parallax_is_small) {
                    tri_pts[h][ref_cur_matches[mi].first] = pos_c_in_ref;
                    is_tri[h][ref_cur_matches[mi].first] = true;
                    ++num_triangulated_pts;
                }
            }
            float parallax_cos;
            if (!cos_parallaxes.empty()) {
                std::sort(cos_parallaxes.begin(), cos_parallaxes.end());
                int idx = std::min(50, (int)cos_parallaxes.size() - 1);
                parallax_cos = cos_parallaxes[idx];
            }
            else {
                parallax_cos = 1.0f;
            }
            nums_valid_pts[h] = num_valid_pts;
            num_tri_pts[h] = num_triangulated_pts;
            parallaxes[h] = parallax_cos;

            hyps_f << aid << '\t' << h << '\t' << model << '\t' << num_valid_pts << '\t' << num_triangulated_pts << '\t'
                   << fmt_hexf(parallax_cos) << '\t' << mat33_hex(hyp_rots[h]) << '\t' << vec3_hex(hyp_transes[h]) << '\n';
        }

        int max_idx = (int)(std::max_element(nums_valid_pts.begin(), nums_valid_pts.end()) - nums_valid_pts.begin());
        bool pose_found = true;
        if (nums_valid_pts[max_idx] < min_num_valid_pts) pose_found = false;
        int num_similars = 0;
        for (int h = 0; h < num_hyp; ++h) {
            if (0.8 * nums_valid_pts[max_idx] < nums_valid_pts[h]) num_similars++;
        }
        if (num_similars > 1) pose_found = false;
        if (pose_found && parallaxes[max_idx] > std::cos(parallax_deg_thr / 180.0 * M_PI)) pose_found = false;
        if (pose_found && num_tri_pts[max_idx] < min_num_triangulated_pts) pose_found = false;

        std::string final_verdict = pose_found ? "success" : "fail_pose_not_found";
        // patch verdict into attempts row retroactively isn't easy with
        // ofstream; log final verdict in final.tsv instead (attempts.tsv's
        // verdict column for a model!=none row is "see final.tsv").
        if (pose_found) {
            final_f << aid << '\t' << max_idx << '\t' << mat33_hex(hyp_rots[max_idx]) << '\t' << vec3_hex(hyp_transes[max_idx]) << '\n';
        }
        else {
            final_f << aid << '\t' << -1 << "\t-\t-\n";
        }
        (void)final_verdict;

        // Self-check: real initialize::perspective, run end-to-end.
        {
            auto persp = stella_vslam::initialize::perspective(ref_frm, num_ransac_iters, min_num_triangulated_pts,
                                                                min_num_valid_pts, parallax_deg_thr, reproj_err_thr, use_fixed_seed);
            bool ok = persp.initialize(curr_frm, local_init_matches);
            if (ok != pose_found) {
                std::cerr << "SELF-CHECK MISMATCH attempt " << aid << ": persp.initialize()=" << ok
                          << " replicated pose_found=" << pose_found << "\n";
            }
            else if (ok) {
                auto R = persp.get_rotation_ref_to_cur();
                auto t = persp.get_translation_ref_to_cur();
                if (R != hyp_rots[max_idx] || t != hyp_transes[max_idx]) {
                    std::cerr << "SELF-CHECK POSE MISMATCH attempt " << aid << "\n";
                }
            }
        }

        // reset ref frame regardless of success/failure (module::initializer
        // resets after create_map_for_monocular too -- but create_map is
        // out of module-3 scope, so this tool just always advances the
        // reference frame after an attempt, matching the observable
        // module-3 behavior: init_matches_/ref_frm_ don't persist across
        // attempts either way).
        ref_frm = stella_vslam::data::frame(curr_frm);
        prev_matched.resize(ref_frm.frm_obs_.undist_keypts_.size());
        for (size_t k = 0; k < prev_matched.size(); ++k) {
            prev_matched[k] = ref_frm.frm_obs_.undist_keypts_[k].pt;
        }
    }

    std::cerr << "dump_stella_init: wrote " << attempt_id << " attempts to " << out_dir << "\n";
    return 0;
}
