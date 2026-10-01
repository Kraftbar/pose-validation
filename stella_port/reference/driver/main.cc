// stella_port reference driver -- single-threaded, deterministic.
//
// Clean-room: written against stella_vslam's own public API
// (system.h/config.h, both BSD-2) and the BSD-2 example utility
// tum_rgbd_util.h/.cc (copied verbatim from stella_vslam_examples, same
// license). Nothing here is derived from orb_port/ or ORB-SLAM2.
//
// Unlike stella_vslam_examples/src/run_tum_rgbd_slam.cc, this driver never
// calls system::startup() (which spawns the mapping_module and
// global_optimization_module threads). Instead it calls
// system::startup_single_threaded() once, then after every
// feed_monocular_frame() call drains both modules synchronously via
// system::synchronize_background_modules() -- see the
// 0001-synchronous-step-entry-points.patch rationale in
// stella_port/reference/README.md.
//
// Dump timing (see docs/orb_port_handover.md "Decisions worth knowing" #7,
// used only as a checklist item): everything written to the *_before.tsv
// files below is captured BEFORE synchronize_background_modules() is called
// for that frame (i.e. it reflects tracking only, not that frame's mapping
// pass); everything in *_after.tsv is captured AFTER it returns (i.e. after
// that frame's synchronous mapping_module + global_optimization_module
// passes have both fully run).
//
// --light (the default output when passed): frames_before.tsv /
// frames_after.tsv only -- pose + tracked flag, keyframe/landmark counts.
// Without --light: the fuller per-frame schema below, see
// stella_port/reference/README.md "Dumps" for the exact column list of each
// file and what is NOT covered (mapping-pass detail beyond what
// frames_after.tsv's counts show, and erased-object final-state capture).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/mapping_module.h"
#include "stella_vslam/global_optimization_module.h"
#include "stella_vslam/module/keyframe_inserter.h"
#include "stella_vslam/module/local_map_cleaner.h"
#include "stella_vslam/publish/map_publisher.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/util/random_array.h"
#include "stella_vslam/util/erasure_log.h"
#include "stella_vslam/util/loop_trace.h" // patch 0013 (module 7)

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

// %.9g + IEEE hex, per the dump-format instructions, so two runs can be
// byte-diffed with no ambiguity about rounding in the text form.
std::string fmt_double(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return std::string(buf);
}

std::string fmt_double_hex(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}

std::string fmt_mat44_9g(const stella_vslam::Mat44_t& m) {
    std::ostringstream oss;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            oss << fmt_double(m(r, c));
            if (!(r == 3 && c == 3)) {
                oss << ',';
            }
        }
    }
    return oss.str();
}

std::string fmt_mat44_hex(const stella_vslam::Mat44_t& m) {
    std::ostringstream oss;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            oss << fmt_double_hex(m(r, c));
            if (!(r == 3 && c == 3)) {
                oss << ',';
            }
        }
    }
    return oss.str();
}

std::string fmt_vec3_9g(const stella_vslam::Vec3_t& v) {
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%.9g,%.9g,%.9g", v(0), v(1), v(2));
    return std::string(buf);
}

std::string fmt_vec3_hex(const stella_vslam::Vec3_t& v) {
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%a,%a,%a", v(0), v(1), v(2));
    return std::string(buf);
}

// ORB descriptors are CV_8U, 32 bytes/row (256 bits). Dumped as lowercase hex.
std::string fmt_descriptor_hex(const cv::Mat& descriptors, int row) {
    if (descriptors.empty() || row >= descriptors.rows) {
        return "";
    }
    const auto* p = descriptors.ptr<uint8_t>(row);
    std::string out;
    out.reserve(descriptors.cols * 2);
    static const char* hexd = "0123456789abcdef";
    for (int c = 0; c < descriptors.cols; ++c) {
        out.push_back(hexd[(p[c] >> 4) & 0xF]);
        out.push_back(hexd[p[c] & 0xF]);
    }
    return out;
}

std::string fmt_bytes_hex(const std::vector<uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size() * 2);
    static const char* hexd = "0123456789abcdef";
    for (auto b : bytes) {
        out.push_back(hexd[(b >> 4) & 0xF]);
        out.push_back(hexd[b & 0xF]);
    }
    return out;
}

const char* track_path_name(stella_vslam::tracking_module::track_path_t p) {
    using T = stella_vslam::tracking_module::track_path_t;
    switch (p) {
        case T::none: return "none";
        case T::motion_model: return "motion_model";
        case T::bow_match: return "bow_match";
        case T::robust_match: return "robust_match";
        case T::relocalize_by_pose: return "relocalize_by_pose";
        case T::relocalize_auto: return "relocalize_auto";
    }
    return "unknown";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "usage: run_stella_reference <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir> <max_frames|-1> "
                     "[--light] [--snapshot-interval N]\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string config_path = argv[2];
    const std::string seq_dir = argv[3];
    const std::string out_dir = argv[4];
    const long max_frames = std::atol(argv[5]);
    bool light = false;
    bool loop_dump = false; // patch 0013 (module 7): --loop-dump, implies --light
    unsigned int loop_snap_from = 0; // patch 0013: --loop-snap-from N: K/L snapshot rows only for steps >= N
    // --snapshot-interval N (default 1 = every frame, the full/default
    // behavior): keyframes.tsv/landmarks.tsv are only written on a frame
    // where the map actually changed (num_keyframes or num_landmarks
    // differs from the last snapshot written) OR frame_idx % N == 0. See
    // README "Dumps" / "Snapshot interval".
    long snapshot_interval = 1;
    for (int i = 6; i < argc; ++i) {
        if (std::strcmp(argv[i], "--light") == 0) {
            light = true;
        }
        else if (std::strcmp(argv[i], "--loop-snap-from") == 0 && i + 1 < argc) { // patch 0013
            loop_snap_from = (unsigned int)std::atol(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--loop-dump") == 0) { // patch 0013
            loop_dump = true;
            light = true;
        }
        else if (std::strcmp(argv[i], "--snapshot-interval") == 0 && i + 1 < argc) {
            snapshot_interval = std::atol(argv[++i]);
            if (snapshot_interval < 1) {
                snapshot_interval = 1;
            }
        }
    }

    auto cfg = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup_single_threaded(true);
    auto* tracker = slam->get_tracker();

    // ---- patch 0013 (module 7, loop closing): compact per-step dump --------------------------------
    // loop_events.tsv : library trace lines (util/loop_trace.h), one event per line
    // loop_snap.tsv   : map snapshot at every global-optimization step (phase 0, before the step) and,
    //                   for an accepted loop, after the pose graph (phase 1) and after the loop BA (phase 2)
    //                   step \t phase \t K \t id \t pose_cw(16 hex, row-major) \t bad \t root \t parent \t children \t
    //                   loop_edges \t connected(id:n in map order, -1 = expired) \t ordered(id:n)
    //                   step \t phase \t L \t id \t pos(3) \t descriptor \t mean_normal(3) \t min \t max \t observed \t
    //                   observable \t ref_kf \t obs(kf:idx,...)
    // loop_kfobs.tsv  : keypoints/descriptors of every keyframe, once: kf \t idx \t x \t y \t octave \t angle \t desc
    // kf_destroyed.tsv: as the full dump (patch 0011)
    std::FILE* loop_events_fp = nullptr;
    std::ofstream loop_snap_f, loop_kfobs_f, loop_destroyed_f;
    std::set<unsigned int> loop_kfobs_seen;
    if (loop_dump) {
        loop_events_fp = std::fopen((out_dir + "/loop_events.tsv").c_str(), "w");
        loop_snap_f.open(out_dir + "/loop_snap.tsv");
        loop_kfobs_f.open(out_dir + "/loop_kfobs.tsv");
        loop_destroyed_f.open(out_dir + "/kf_destroyed.tsv");
        loop_destroyed_f << "frame_idx\tphase\tkf_id\n";
        stella_vslam::util::g_loop_trace = loop_events_fp;
        stella_vslam::util::g_log_destroyed = true;
        stella_vslam::util::g_loop_hook = [&](int phase) {
            std::vector<std::shared_ptr<stella_vslam::data::keyframe>> kfs;
            std::vector<std::shared_ptr<stella_vslam::data::landmark>> lms;
            std::set<std::shared_ptr<stella_vslam::data::landmark>> local_lms;
            slam->get_map_publisher()->get_keyframes(kfs);
            slam->get_map_publisher()->get_landmarks(lms, local_lms);
            std::sort(kfs.begin(), kfs.end(), [](const auto& a, const auto& b) { return a->id_ < b->id_; });
            std::sort(lms.begin(), lms.end(), [](const auto& a, const auto& b) { return a->id_ < b->id_; });
            const unsigned int step = stella_vslam::util::g_loop_step_index;
            const bool write_rows = step >= loop_snap_from;
            for (const auto& kf : kfs) {
                if (loop_kfobs_seen.insert(kf->id_).second) {
                    const auto& kpts = kf->frm_obs_.undist_keypts_;
                    for (size_t k = 0; k < kpts.size(); ++k) {
                        loop_kfobs_f << kf->id_ << '\t' << k << '\t' << fmt_double_hex(kpts[k].pt.x) << '\t'
                                     << fmt_double_hex(kpts[k].pt.y) << '\t' << kpts[k].octave << '\t'
                                     << fmt_double_hex(kpts[k].angle) << '\t'
                                     << fmt_descriptor_hex(kf->frm_obs_.descriptors_, (int)k) << '\n';
                    }
                }
                if (!write_rows) {
                    continue;
                }
                std::vector<std::pair<int, unsigned int>> raw_conn, raw_ordered;
                kf->graph_node_->get_raw_state(raw_conn, raw_ordered);
                const auto parent = kf->graph_node_->get_spanning_parent();
                std::ostringstream ch, le, cn, od;
                bool first = true;
                for (const auto& c : kf->graph_node_->get_spanning_children()) {
                    ch << (first ? "" : ",") << (c ? (long long)c->id_ : -1LL);
                    first = false;
                }
                first = true;
                for (const auto& e : kf->graph_node_->get_loop_edges()) {
                    le << (first ? "" : ",") << (e ? (long long)e->id_ : -1LL);
                    first = false;
                }
                for (size_t k = 0; k < raw_conn.size(); ++k) {
                    cn << (k ? "," : "") << raw_conn[k].first << ':' << raw_conn[k].second;
                }
                for (size_t k = 0; k < raw_ordered.size(); ++k) {
                    od << (k ? "," : "") << raw_ordered[k].first << ':' << raw_ordered[k].second;
                }
                loop_snap_f << step << '\t' << phase << "\tK\t" << kf->id_ << '\t' << fmt_mat44_hex(kf->get_pose_cw()) << '\t'
                            << (kf->will_be_erased() ? 1 : 0) << '\t' << (kf->graph_node_->is_spanning_root() ? 1 : 0) << '\t'
                            << (parent ? (long long)parent->id_ : -1LL) << '\t' << ch.str() << '\t' << le.str() << '\t'
                            << cn.str() << '\t' << od.str() << '\n';
            }
            for (const auto& lm : lms) {
                if (!write_rows) {
                    break;
                }
                const auto ref_kf = lm->get_ref_keyframe();
                std::ostringstream obs;
                bool first = true;
                for (const auto& o : lm->get_observations()) {
                    auto kf = o.first.lock();
                    if (!kf) {
                        continue;
                    }
                    obs << (first ? "" : ",") << kf->id_ << ':' << o.second;
                    first = false;
                }
                loop_snap_f << step << '\t' << phase << "\tL\t" << lm->id_ << '\t' << fmt_vec3_hex(lm->get_pos_in_world()) << '\t'
                            << (lm->has_representative_descriptor() ? fmt_descriptor_hex(lm->get_descriptor(), 0) : "") << '\t'
                            << fmt_vec3_hex(lm->get_obs_mean_normal()) << '\t'
                            << fmt_double_hex(lm->get_min_valid_distance()) << '\t' << fmt_double_hex(lm->get_max_valid_distance()) << '\t'
                            << lm->get_num_observed() << '\t' << lm->get_num_observable() << '\t'
                            << (ref_kf ? (long long)ref_kf->id_ : -1LL) << '\t' << obs.str() << '\n';
            }
        };
    }

    tum_rgbd_sequence sequence(seq_dir);
    const auto frames = sequence.get_frames();

    std::ofstream before(out_dir + "/frames_before.tsv");
    std::ofstream after(out_dir + "/frames_after.tsv");
    before << "frame_idx\ttimestamp\ttracked\tpose_row_major_9g\tpose_row_major_hex\n";
    after << "frame_idx\tnum_keyframes\tnum_landmarks\n";

    // --- full-schema outputs (skipped in --light mode) ---
    std::ofstream keypoints_f, descriptors_f, matches_f, frame_trace_f, local_map_f, kf_decision_f, rng_f;
    std::ofstream keyframes_f, landmarks_f;
    std::ofstream culled_landmarks_f, triangulation_f, fused_landmarks_f, local_ba_f, culled_keyframes_f, global_opt_step_f;
    std::ofstream erased_keyframes_f, erased_landmarks_f;
    std::ofstream track_pre_f, keyframe_meta_f, kf_insert_f, kf_insert_lm_f;
    std::ofstream kf_destroyed_f, conn_f; // [reference-port] patch 0011 (module 6)
    std::set<unsigned int> keyframe_meta_seen;
    if (!light) {
        keypoints_f.open(out_dir + "/keypoints.tsv");
        keypoints_f << "frame_idx\tkp_idx\tx_9g\tx_hex\ty_9g\ty_hex\toctave\tangle_9g\tangle_hex\tresponse_9g\tresponse_hex\n";

        descriptors_f.open(out_dir + "/descriptors.tsv");
        descriptors_f << "frame_idx\tkp_idx\tdescriptor_hex\n";

        matches_f.open(out_dir + "/matches.tsv");
        matches_f << "frame_idx\tkp_idx\tlandmark_id\n";

        frame_trace_f.open(out_dir + "/frame_trace.tsv");
        frame_trace_f << "frame_idx\ttrack_path\tref_keyfrm_id\t"
                      << "initial_pose_valid\tinitial_pose_9g\tinitial_pose_hex\t"
                      << "final_pose_valid\tfinal_pose_9g\tfinal_pose_hex\t"
                      << "num_tracked_lms\tnum_reliable_lms\n";

        local_map_f.open(out_dir + "/local_map.tsv");
        local_map_f << "frame_idx\tlocal_keyframe_ids\tlocal_landmark_ids\n";

        kf_decision_f.open(out_dir + "/kf_decision.tsv");
        kf_decision_f << "frame_idx\tverdict\tmapper_paused_or_pausing\t"
                      << "num_reliable_lms_ref\tnum_reliable_lms\tnum_tracked_lms\tdistance_traveled_9g\t"
                      << "max_interval_elapsed\tmin_interval_elapsed\tmax_distance_traveled\tmin_distance_traveled\t"
                      << "view_changed\tnot_enough_lms\tenough_keyfrms\ttracking_is_unstable\t"
                      << "almost_all_lms_are_tracked\tmapper_is_skipping_localBA\n";

        rng_f.open(out_dir + "/rng.tsv");
        rng_f << "frame_idx\tengines_created_delta\tarray_calls_delta\telements_requested_delta\t"
              << "engines_created_total\tarray_calls_total\telements_requested_total\n";

        keyframes_f.open(out_dir + "/keyframes.tsv");
        keyframes_f << "frame_idx\tkf_id\tpose_cw_9g\tpose_cw_hex\tbad\tcovisibilities\tspanning_parent\tspanning_children\n";

        landmarks_f.open(out_dir + "/landmarks.tsv");
        landmarks_f << "frame_idx\tlm_id\tpos_w_9g\tpos_w_hex\tdescriptor_hex\tmean_normal_9g\tmean_normal_hex\t"
                    << "min_valid_dist_9g\tmax_valid_dist_9g\tnum_observed\tnum_observable\tref_keyfrm_id\tobservations\n";

        culled_landmarks_f.open(out_dir + "/culled_landmarks.tsv");
        culled_landmarks_f << "frame_idx\tmapping_keyfrm_id\tlandmark_id\treason\n";

        triangulation_f.open(out_dir + "/triangulation.tsv");
        triangulation_f << "frame_idx\tmapping_keyfrm_id\tneighbor_order\tneighbor_keyfrm_id\t"
                        << "num_matches\tmatches\taccepted_landmark_ids\taccepted_landmark_positions_9g\taccepted_landmark_positions_hex\n";

        fused_landmarks_f.open(out_dir + "/fused_landmarks.tsv");
        fused_landmarks_f << "frame_idx\tmapping_keyfrm_id\treplaced_landmark_id\treplaced_by_landmark_id\n";

        local_ba_f.open(out_dir + "/local_ba.tsv");
        local_ba_f << "frame_idx\tmapping_keyfrm_id\tinvoked\tlocal_keyfrm_ids\tfixed_keyfrm_ids\t"
                   << "num_iters_first\tnum_iters_second\tchi2_before_9g\tchi2_after_9g\tnum_outlier_observations_removed\n";

        culled_keyframes_f.open(out_dir + "/culled_keyframes.tsv");
        culled_keyframes_f << "frame_idx\tmapping_keyfrm_id\tculled_keyframe_id\treason\n";

        global_opt_step_f.open(out_dir + "/global_opt_step.tsv");
        global_opt_step_f << "frame_idx\tran\tkeyfrm_id\tcandidate_ids_considered\tloop_accepted\taccepted_candidate_id\t"
                          << "num_keyframes_sim3_corrected\tnum_landmarks_position_corrected\tloop_ba_invoked\n";

        erased_keyframes_f.open(out_dir + "/erased_keyframes.tsv");
        erased_keyframes_f << "frame_idx\tkf_id\tpose_cw_9g\tpose_cw_hex\tcovisibilities\tspanning_parent\tspanning_children\n";

        // [reference-port] patch 0008 / module-5 tracking harness inputs
        track_pre_f.open(out_dir + "/track_pre.tsv");
        track_pre_f << "frame_idx\ttimestamp_hex\ttracking_state\ttwist_valid\ttwist_hex\tlast_cam_pose_from_ref_hex\t"
                    << "last_reloc_frm_id\tlast_reloc_frm_ts_hex\tlast_frm_pose_valid\tlast_frm_id\tlast_frm_pose_hex\t"
                    << "last_frm_ref_keyfrm_id\tlast_frm_num_lms\tlast_frm_lm_hash\t"
                    << "last_ins_kf_id\tlast_ins_kf_ts_hex\tlast_ins_kf_trans_wc_hex\tnum_keyframes\tfixed_kf_thr\n";
        kf_insert_f.open(out_dir + "/kf_insert.tsv");
        kf_insert_f << "frame_idx\tkf_id\ttimestamp_hex\tpose_cw_hex\tpose_wc_hex\ttrans_wc_hex\n";
        kf_insert_lm_f.open(out_dir + "/kf_insert_lms.tsv");
        kf_insert_lm_f << "frame_idx\tkf_id\tkp_idx\tlm_id\tpos_w_hex\tdescriptor_hex\tmean_normal_hex\t"
                       << "min_valid_dist_9g\tmax_valid_dist_9g\tnum_observations\tnum_observed\tnum_observable\tref_keyfrm_id\n";
        keyframe_meta_f.open(out_dir + "/keyframe_meta.tsv");
        keyframe_meta_f << "first_seen_frame_idx\tkf_id\ttimestamp_hex\n";

        // [reference-port] patch 0011: keyframe destruction schedule + raw covisibility state
        kf_destroyed_f.open(out_dir + "/kf_destroyed.tsv");
        kf_destroyed_f << "frame_idx\tphase\tkf_id\n";
        conn_f.open(out_dir + "/conn.tsv");
        conn_f << "frame_idx\tkf_id\tconnected_in_order\tordered_raw\n";
        stella_vslam::util::g_log_destroyed = true;

        erased_landmarks_f.open(out_dir + "/erased_landmarks.tsv");
        erased_landmarks_f << "frame_idx\tlm_id\tpos_w_9g\tpos_w_hex\tdescriptor_hex\tmean_normal_9g\tmean_normal_hex\t"
                           << "min_valid_dist_9g\tmax_valid_dist_9g\tnum_observed\tnum_observable\tref_keyfrm_id\t"
                           << "observations\treplaced_by_id\n";
    }

    uint64_t prev_engines = 0, prev_calls = 0, prev_elems = 0;
    unsigned int last_snapshot_n_keyfrms = 0, last_snapshot_n_lms = 0;

    const size_t n = (max_frames >= 0) ? std::min<size_t>(frames.size(), (size_t)max_frames) : frames.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& f = frames[i];
        cv::Mat img = cv::imread(f.rgb_img_path_, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "failed to read " << f.rgb_img_path_ << "\n";
            return 2;
        }

        // [reference-port] reset here, not after synchronize_background_modules():
        // in synchronous mode a keyframe insertion drains inline, *during*
        // feed_monocular_frame() below (see system::synchronize_background_modules()'s
        // comment) -- so this is the only point in the loop guaranteed to run
        // before that frame's mapping/global-opt activity, and resetting after
        // sync would wipe the trace before we read it.
        if (!light) {
            auto* mapper_for_reset = slam->get_mapper();
            if (mapper_for_reset) {
                mapper_for_reset->reset_last_pass_trace();
            }
            auto* global_opt_for_reset = slam->get_global_optimizer();
            if (global_opt_for_reset) {
                global_opt_for_reset->reset_last_step_trace();
            }
        }

        stella_vslam::util::g_phase = 0; // [reference-port] patch 0011
        stella_vslam::util::g_loop_frame_idx = (int)i; // patch 0013
        if (!light && tracker) {
            const auto ps = tracker->get_pre_track_state();
            track_pre_f << i << '\t' << fmt_double_hex(f.timestamp_) << '\t' << ps.tracking_state << '\t'
                        << (ps.twist_is_valid ? 1 : 0) << '\t' << fmt_mat44_hex(ps.twist) << '\t'
                        << fmt_mat44_hex(ps.last_cam_pose_from_ref_keyfrm) << '\t'
                        << ps.last_reloc_frm_id << '\t' << fmt_double_hex(ps.last_reloc_frm_timestamp) << '\t'
                        << (ps.last_frm_pose_valid ? 1 : 0) << '\t' << ps.last_frm_id << '\t' << fmt_mat44_hex(ps.last_frm_pose_cw) << '\t'
                        << ps.last_frm_ref_keyfrm_id << '\t' << ps.last_frm_num_landmarks << '\t' << ps.last_frm_landmark_hash << '\t'
                        << ps.last_inserted_keyfrm_id << '\t' << fmt_double_hex(ps.last_inserted_keyfrm_timestamp) << '\t'
                        << fmt_vec3_hex(ps.last_inserted_keyfrm_trans_wc) << '\t'
                        << ps.num_keyframes << '\t' << ps.fixed_keyframe_id_threshold << '\n';
        }

        auto pose_cw = slam->feed_monocular_frame(img, f.timestamp_);

        // ---- frame-level dump: BEFORE the synchronous mapping/global-opt pass ----
        before << i << '\t' << fmt_double(f.timestamp_) << '\t' << (pose_cw ? 1 : 0) << '\t';
        if (pose_cw) {
            before << fmt_mat44_9g(*pose_cw) << '\t' << fmt_mat44_hex(*pose_cw);
        }
        else {
            before << "-\t-";
        }
        before << '\n';

        if (!light && tracker) {
            const auto& frm = tracker->curr_frm_;
            const auto& kpts = frm.frm_obs_.undist_keypts_;
            const auto& descs = frm.frm_obs_.descriptors_;

            for (size_t k = 0; k < kpts.size(); ++k) {
                const auto& kp = kpts[k];
                keypoints_f << i << '\t' << k << '\t'
                            << fmt_double(kp.pt.x) << '\t' << fmt_double_hex(kp.pt.x) << '\t'
                            << fmt_double(kp.pt.y) << '\t' << fmt_double_hex(kp.pt.y) << '\t'
                            << kp.octave << '\t'
                            << fmt_double(kp.angle) << '\t' << fmt_double_hex(kp.angle) << '\t'
                            << fmt_double(kp.response) << '\t' << fmt_double_hex(kp.response) << '\n';
                descriptors_f << i << '\t' << k << '\t' << fmt_descriptor_hex(descs, (int)k) << '\n';
                const auto lm = frm.get_landmark((unsigned int)k);
                matches_f << i << '\t' << k << '\t' << (lm ? (long long)lm->id_ : -1) << '\n';
            }

            frame_trace_f << i << '\t' << track_path_name(tracker->last_track_path_) << '\t'
                          << (frm.ref_keyfrm_ ? (long long)frm.ref_keyfrm_->id_ : -1) << '\t'
                          << (tracker->last_pose_after_initial_track_valid_ ? 1 : 0) << '\t'
                          << fmt_mat44_9g(tracker->last_pose_after_initial_track_) << '\t'
                          << fmt_mat44_hex(tracker->last_pose_after_initial_track_) << '\t'
                          << (pose_cw ? 1 : 0) << '\t';
            if (pose_cw) {
                frame_trace_f << fmt_mat44_9g(*pose_cw) << '\t' << fmt_mat44_hex(*pose_cw);
            }
            else {
                frame_trace_f << "-\t-";
            }
            frame_trace_f << '\t' << tracker->last_num_tracked_lms_ << '\t' << tracker->last_num_reliable_lms_ << '\n';

            local_map_f << i << '\t';
            for (size_t k = 0; k < tracker->last_local_keyframe_ids_.size(); ++k) {
                local_map_f << (k ? "," : "") << tracker->last_local_keyframe_ids_[k];
            }
            local_map_f << '\t';
            for (size_t k = 0; k < tracker->get_local_landmarks().size(); ++k) {
                local_map_f << (k ? "," : "") << (tracker->get_local_landmarks()[k] ? tracker->get_local_landmarks()[k]->id_ : 0);
            }
            local_map_f << '\n';

            const auto& d = tracker->get_last_kf_decision();
            kf_decision_f << i << '\t' << (d.verdict ? 1 : 0) << '\t' << (d.mapper_paused_or_pausing ? 1 : 0) << '\t'
                          << d.num_reliable_lms_ref << '\t' << d.num_reliable_lms << '\t' << d.num_tracked_lms << '\t'
                          << fmt_double(d.distance_traveled) << '\t'
                          << (d.max_interval_elapsed ? 1 : 0) << '\t' << (d.min_interval_elapsed ? 1 : 0) << '\t'
                          << (d.max_distance_traveled ? 1 : 0) << '\t' << (d.min_distance_traveled ? 1 : 0) << '\t'
                          << (d.view_changed ? 1 : 0) << '\t' << (d.not_enough_lms ? 1 : 0) << '\t'
                          << (d.enough_keyfrms ? 1 : 0) << '\t' << (d.tracking_is_unstable ? 1 : 0) << '\t'
                          << (d.almost_all_lms_are_tracked ? 1 : 0) << '\t' << (d.mapper_is_skipping_localBA ? 1 : 0) << '\n';

            const uint64_t engines = stella_vslam::util::g_random_engines_created.load();
            const uint64_t calls = stella_vslam::util::g_random_array_calls.load();
            const uint64_t elems = stella_vslam::util::g_random_array_elements_requested.load();
            rng_f << i << '\t' << (engines - prev_engines) << '\t' << (calls - prev_calls) << '\t' << (elems - prev_elems)
                  << '\t' << engines << '\t' << calls << '\t' << elems << '\n';
            prev_engines = engines;
            prev_calls = calls;
            prev_elems = elems;
        }

        // [reference-port] patch 0009: keyframe_inserter::create_new_keyframe()
        // post-update_landmarks() records of this frame (at most one)
        if (!light) {
            for (const auto& rec : stella_vslam::util::g_kf_inserts) {
                kf_insert_f << i << '\t' << rec.keyfrm_id << '\t' << fmt_double_hex(rec.timestamp) << '\t'
                            << fmt_mat44_hex(rec.pose_cw) << '\t' << fmt_mat44_hex(rec.pose_wc) << '\t'
                            << fmt_vec3_hex(rec.trans_wc) << '\n';
                for (const auto& lr : rec.landmarks) {
                    kf_insert_lm_f << i << '\t' << rec.keyfrm_id << '\t' << lr.kp_idx << '\t' << lr.id << '\t'
                                   << fmt_vec3_hex(lr.pos_w) << '\t' << fmt_bytes_hex(lr.descriptor) << '\t'
                                   << fmt_vec3_hex(lr.mean_normal) << '\t'
                                   << fmt_double(lr.min_valid_dist) << '\t' << fmt_double(lr.max_valid_dist) << '\t'
                                   << lr.num_observations << '\t' << lr.num_observed << '\t' << lr.num_observable << '\t'
                                   << lr.ref_keyfrm_id << '\n';
                }
            }
            stella_vslam::util::g_kf_inserts.clear();
        }

        // ---- run the mapping_module / global_optimization_module steps this
        //      frame produced, synchronously, on this thread ----
        slam->synchronize_background_modules();

        // [reference-port] patch 0011: keyframe destructions of this frame, in order
        if (!light) {
            for (const auto& d : stella_vslam::util::g_destroyed_keyframes) {
                kf_destroyed_f << i << '\t' << d.phase << '\t' << d.id << '\n';
            }
            stella_vslam::util::g_destroyed_keyframes.clear();
            stella_vslam::util::g_phase = 3;
        }

        if (loop_dump) { // patch 0013
            for (const auto& d : stella_vslam::util::g_destroyed_keyframes) {
                loop_destroyed_f << i << '\t' << d.phase << '\t' << d.id << '\n';
            }
            stella_vslam::util::g_destroyed_keyframes.clear();
            stella_vslam::util::g_phase = 3;
        }

        // ---- mapping-pass / global-optimization-step dump: AFTER the
        //      synchronous pass (this is what that pass actually did) ----
        if (!light) {
            auto* mapper = slam->get_mapper();
            if (mapper) {
                const auto& mp = mapper->get_last_pass_trace();
                if (mp.ran) {
                    for (auto lm_id : mp.culled_landmark_ids) {
                        culled_landmarks_f << i << '\t' << mp.keyfrm_id << '\t' << lm_id << '\t'
                                           << stella_vslam::module::local_map_cleaner::culled_landmark_reason() << '\n';
                    }
                    for (size_t nb = 0; nb < mp.triangulation.size(); ++nb) {
                        const auto& t = mp.triangulation[nb];
                        std::ostringstream matches_s, ids_s, pos9g_s, poshex_s;
                        for (size_t m = 0; m < t.matches.size(); ++m) {
                            matches_s << (m ? "," : "") << t.matches[m].first << ':' << t.matches[m].second;
                        }
                        for (size_t a = 0; a < t.accepted_landmark_ids.size(); ++a) {
                            ids_s << (a ? "," : "") << t.accepted_landmark_ids[a];
                            pos9g_s << (a ? ";" : "") << fmt_vec3_9g(t.accepted_landmark_positions[a]);
                            poshex_s << (a ? ";" : "") << fmt_vec3_hex(t.accepted_landmark_positions[a]);
                        }
                        triangulation_f << i << '\t' << mp.keyfrm_id << '\t' << nb << '\t' << t.neighbor_keyfrm_id << '\t'
                                        << t.matches.size() << '\t' << matches_s.str() << '\t'
                                        << ids_s.str() << '\t' << pos9g_s.str() << '\t' << poshex_s.str() << '\n';
                    }
                    for (const auto& rep : mp.replaced_landmark_ids) {
                        fused_landmarks_f << i << '\t' << mp.keyfrm_id << '\t' << rep.first << '\t' << rep.second << '\n';
                    }
                    for (auto kf_id : mp.culled_keyframe_ids) {
                        culled_keyframes_f << i << '\t' << mp.keyfrm_id << '\t' << kf_id << '\t'
                                           << stella_vslam::module::local_map_cleaner::culled_keyframe_reason() << '\n';
                    }
                    const auto& ba = mapper->get_last_local_ba_trace();
                    local_ba_f << i << '\t' << mp.keyfrm_id << '\t' << (mp.local_ba_invoked ? 1 : 0) << '\t';
                    if (mp.local_ba_invoked) {
                        for (size_t k = 0; k < ba.local_keyfrm_ids.size(); ++k) {
                            local_ba_f << (k ? "," : "") << ba.local_keyfrm_ids[k];
                        }
                        local_ba_f << '\t';
                        for (size_t k = 0; k < ba.fixed_keyfrm_ids.size(); ++k) {
                            local_ba_f << (k ? "," : "") << ba.fixed_keyfrm_ids[k];
                        }
                        local_ba_f << '\t' << ba.num_iters_first << '\t' << ba.num_iters_second << '\t'
                                   << fmt_double(ba.chi2_before) << '\t' << fmt_double(ba.chi2_after) << '\t'
                                   << ba.num_outlier_observations_removed;
                    }
                    else {
                        local_ba_f << "\t\t\t\t\t";
                    }
                    local_ba_f << '\n';
                }
            }
            auto* global_opt = slam->get_global_optimizer();
            if (global_opt) {
                const auto& g = global_opt->get_last_step_trace();
                if (g.ran) {
                    std::ostringstream cand_s;
                    for (size_t k = 0; k < g.candidate_ids_considered.size(); ++k) {
                        cand_s << (k ? "," : "") << g.candidate_ids_considered[k];
                    }
                    global_opt_step_f << i << '\t' << 1 << '\t' << g.keyfrm_id << '\t' << cand_s.str() << '\t'
                                      << (g.loop_accepted ? 1 : 0) << '\t' << g.accepted_candidate_id << '\t'
                                      << g.num_keyframes_sim3_corrected << '\t' << g.num_landmarks_position_corrected << '\t'
                                      << (g.loop_ba_invoked ? 1 : 0) << '\n';
                }
            }

            // ---- erased-object dump: everything data::keyframe::prepare_for_erasing()/
            //      data::landmark::prepare_for_erasing() recorded since the last
            //      frame (see util/erasure_log.h); cleared after dumping so each
            //      record is attributed to exactly one frame ----
            for (const auto& kf : stella_vslam::util::g_erased_keyframes) {
                std::ostringstream covis_s;
                for (size_t k = 0; k < kf.covisibilities.size(); ++k) {
                    covis_s << (k ? "," : "") << kf.covisibilities[k].first << ':' << kf.covisibilities[k].second;
                }
                std::ostringstream children_s;
                for (size_t k = 0; k < kf.spanning_children_ids.size(); ++k) {
                    children_s << (k ? "," : "") << kf.spanning_children_ids[k];
                }
                erased_keyframes_f << i << '\t' << kf.id << '\t' << fmt_mat44_9g(kf.pose_cw) << '\t' << fmt_mat44_hex(kf.pose_cw) << '\t'
                                   << covis_s.str() << '\t' << kf.spanning_parent_id << '\t' << children_s.str() << '\n';
            }
            for (const auto& lm : stella_vslam::util::g_erased_landmarks) {
                std::ostringstream obs_s;
                for (size_t k = 0; k < lm.observations.size(); ++k) {
                    obs_s << (k ? "," : "") << lm.observations[k].first << ':' << lm.observations[k].second;
                }
                erased_landmarks_f << i << '\t' << lm.id << '\t' << fmt_vec3_9g(lm.pos_w) << '\t' << fmt_vec3_hex(lm.pos_w) << '\t'
                                   << fmt_bytes_hex(lm.descriptor) << '\t' << fmt_vec3_9g(lm.mean_normal) << '\t' << fmt_vec3_hex(lm.mean_normal) << '\t'
                                   << fmt_double(lm.min_valid_dist) << '\t' << fmt_double(lm.max_valid_dist) << '\t'
                                   << lm.num_observed << '\t' << lm.num_observable << '\t' << lm.ref_keyfrm_id << '\t'
                                   << obs_s.str() << '\t' << lm.replaced_by_id << '\n';
            }
            stella_vslam::util::g_erased_keyframes.clear();
            stella_vslam::util::g_erased_landmarks.clear();
        }

        // ---- map-level dump: AFTER the synchronous pass ----
        std::vector<std::shared_ptr<stella_vslam::data::keyframe>> all_keyfrms;
        std::vector<std::shared_ptr<stella_vslam::data::landmark>> all_lms;
        std::set<std::shared_ptr<stella_vslam::data::landmark>> local_lms;
        const auto n_keyfrms = slam->get_map_publisher()->get_keyframes(all_keyfrms);
        const auto n_lms = slam->get_map_publisher()->get_landmarks(all_lms, local_lms);
        after << i << '\t' << n_keyfrms << '\t' << n_lms << '\n';

        const bool map_changed = (n_keyfrms != last_snapshot_n_keyfrms) || (n_lms != last_snapshot_n_lms);
        const bool take_snapshot = map_changed || (snapshot_interval <= 1) || (i % (size_t)snapshot_interval == 0);
        if (!light && take_snapshot) {
            last_snapshot_n_keyfrms = n_keyfrms;
            last_snapshot_n_lms = n_lms;
            const bool conn_dump = slam->get_mapper() && slam->get_mapper()->get_last_pass_trace().ran; // [reference-port] patch 0011
            for (const auto& kf : all_keyfrms) {
                if (!kf) {
                    continue;
                }
                if (conn_dump) {
                    std::vector<std::pair<int, unsigned int>> raw_conn, raw_ordered;
                    kf->graph_node_->get_raw_state(raw_conn, raw_ordered);
                    conn_f << i << '\t' << kf->id_ << '\t';
                    for (size_t k = 0; k < raw_conn.size(); ++k) {
                        conn_f << (k ? "," : "") << raw_conn[k].first << ':' << raw_conn[k].second;
                    }
                    conn_f << '\t';
                    for (size_t k = 0; k < raw_ordered.size(); ++k) {
                        conn_f << (k ? "," : "") << raw_ordered[k].first << ':' << raw_ordered[k].second;
                    }
                    conn_f << '\n';
                }
                if (keyframe_meta_seen.insert(kf->id_).second) {
                    keyframe_meta_f << i << '\t' << kf->id_ << '\t' << fmt_double_hex(kf->timestamp_) << '\n';
                }
                const auto covis = kf->graph_node_->get_covisibilities();
                std::ostringstream covis_str;
                for (size_t k = 0; k < covis.size(); ++k) {
                    if (!covis[k]) {
                        continue;
                    }
                    covis_str << (k ? "," : "") << covis[k]->id_ << ':'
                              << kf->graph_node_->get_num_shared_landmarks(covis[k]);
                }
                const auto parent = kf->graph_node_->get_spanning_parent();
                const auto children = kf->graph_node_->get_spanning_children();
                std::ostringstream children_str;
                bool first_c = true;
                for (const auto& c : children) {
                    if (!c) {
                        continue;
                    }
                    children_str << (first_c ? "" : ",") << c->id_;
                    first_c = false;
                }
                keyframes_f << i << '\t' << kf->id_ << '\t'
                            << fmt_mat44_9g(kf->get_pose_cw()) << '\t' << fmt_mat44_hex(kf->get_pose_cw()) << '\t'
                            << (kf->will_be_erased() ? 1 : 0) << '\t'
                            << covis_str.str() << '\t'
                            << (parent ? (long long)parent->id_ : -1) << '\t'
                            << children_str.str() << '\n';
            }

            for (const auto& lm : all_lms) {
                if (!lm) {
                    continue;
                }
                const auto ref_kf = lm->get_ref_keyframe();
                std::ostringstream obs_str;
                bool first_o = true;
                for (const auto& obs : lm->get_observations()) {
                    auto kf = obs.first.lock();
                    if (!kf) {
                        continue;
                    }
                    obs_str << (first_o ? "" : ",") << kf->id_ << ':' << obs.second;
                    first_o = false;
                }
                landmarks_f << i << '\t' << lm->id_ << '\t'
                            << fmt_vec3_9g(lm->get_pos_in_world()) << '\t' << fmt_vec3_hex(lm->get_pos_in_world()) << '\t'
                            << (lm->has_representative_descriptor() ? fmt_descriptor_hex(lm->get_descriptor(), 0) : "") << '\t'
                            << fmt_vec3_9g(lm->get_obs_mean_normal()) << '\t' << fmt_vec3_hex(lm->get_obs_mean_normal()) << '\t'
                            << fmt_double(lm->get_min_valid_distance()) << '\t' << fmt_double(lm->get_max_valid_distance()) << '\t'
                            << lm->get_num_observed() << '\t' << lm->get_num_observable() << '\t'
                            << (ref_kf ? (long long)ref_kf->id_ : -1) << '\t'
                            << obs_str.str() << '\n';
            }
        }
    }

    stella_vslam::util::g_log_destroyed = false; // [reference-port] patch 0011
    if (loop_dump) { // patch 0013
        stella_vslam::util::g_loop_hook = nullptr;
        stella_vslam::util::g_loop_trace = nullptr;
        std::fclose(loop_events_fp);
        loop_snap_f.close();
        loop_kfobs_f.close();
        loop_destroyed_f.close();
    }
    before.close();
    after.close();
    if (!light) {
        kf_destroyed_f.close();
        conn_f.close();
        keypoints_f.close();
        descriptors_f.close();
        matches_f.close();
        frame_trace_f.close();
        local_map_f.close();
        kf_decision_f.close();
        rng_f.close();
        keyframes_f.close();
        landmarks_f.close();
        culled_landmarks_f.close();
        triangulation_f.close();
        fused_landmarks_f.close();
        local_ba_f.close();
        culled_keyframes_f.close();
        global_opt_step_f.close();
        erased_keyframes_f.close();
        erased_landmarks_f.close();
        track_pre_f.close();
        kf_insert_f.close();
        kf_insert_lm_f.close();
        keyframe_meta_f.close();
    }

    // Final TUM-format trajectory (tum_eval.py-compatible): pause_other_threads()
    // inside save_frame_trajectory() calls mapper_->async_pause(), which -- since
    // no mapping thread was ever started -- resolves immediately (is_terminated_
    // defaults true; see system::startup_single_threaded()).
    slam->save_frame_trajectory(out_dir + "/trajectory.tum", "TUM");

    // NOTE: intentionally do not call slam->shutdown() -- it calls
    // mapping_thread_->join() on a null unique_ptr<thread> since no thread was
    // ever started by startup_single_threaded(). The process exit (or ~system())
    // releases resources without joining anything.
    return 0;
}
