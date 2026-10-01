// stella_port reference tool, module 4b: pose_optimizer_g2o call dumper.
//
// Clean-room: written against stella_vslam's own public API (system.h,
// config.h, all BSD-2) and the BSD-2 example utility tum_rgbd_util.h/.cc,
// same as dump_stella_init.cc (module 3). Nothing here is derived from
// orb_port/ or ORB-SLAM2.
//
// Unlike dump_stella_init.cc/dump_frame_bow.cc, this tool's data source
// REQUIRES a small library patch: pose_optimizer_g2o::optimize() has no
// public API surface exposing what edges/observations it built internally,
// and every real call happens deep inside tracking_module/module::
// frame_tracker/module::relocalizer/module::loop_detector (private
// members, not reachable from outside to replay with a reconstructed frame
// state without duplicating a large amount of tracking-state machinery --
// which module 3's dump_stella_init.cc explicitly avoided doing for the
// initializer's much smaller state machine). So this leaf takes the other
// documented option: `stella_port/reference/patches/
// 0006-pose-optimizer-g2o-trace.patch` adds an opt-in (default-off) global
// trace buffer inside pose_optimizer_g2o.cc itself (see
// pose_optimizer_g2o.h's pose_optimizer_g2o_trace_* declarations) that
// records every real optimize() call's inputs (initial pose, camera
// intrinsics, every valid observation's keypoint/octave/inv_sigma_sq/
// landmark-world-position) and outputs (final pose, outlier flags,
// returned inlier count) -- exactly the values pose_optimizer_g2o.cc
// itself computes, not reimplemented or reconstructed. No algorithm code
// changed; the hunks are additive (see the patch). Everything else in this
// tool is a completely ordinary run of the same synchronous single-
// threaded driver loop stella_port/reference/driver/main.cc uses
// (feed_monocular_frame + synchronize_background_modules per frame), so
// pose_optimizer_g2o::optimize() fires from its real call sites: frame
// tracking (motion-model/bow/robust re-tracking, module::frame_tracker),
// relocalization (module::relocalizer), and loop-closure Sim3 verification
// (module::loop_detector) -- whichever actually occur on the sequence.
//
// NOT captured: per-iteration chi2/lambda/accept-reject trial detail
// inside g2o::OptimizationAlgorithmLevenberg's do-while loop. g2o's public
// SparseOptimizer/Solver API exposes no such hook, and patching g2o itself
// (a separate BSD library, not stella_vslam) was judged out of scope for
// this leaf -- see stella_port/HANDOVER.md module-4b note. Validation
// against check_sv_g2o_pose.c is therefore on FINAL outputs (pose,
// outlier flags, inlier count) given identical inputs, not per-iteration
// internals.
//
// Determinism: run twice, diff (tools/dump_stella_g2o.py does this).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/optimize/pose_optimizer_g2o.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace {

std::string fmt_double_hex(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}

std::string fmt_mat44row_hex(const double m[16]) {
    std::ostringstream oss;
    for (int i = 0; i < 16; ++i) {
        oss << fmt_double_hex(m[i]);
        if (i != 15) {
            oss << ',';
        }
    }
    return oss.str();
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "usage: dump_stella_g2o_pose <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir> <max_frames|-1>\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string config_path = argv[2];
    const std::string seq_dir = argv[3];
    const std::string out_dir = argv[4];
    const long max_frames = std::atol(argv[5]);

    stella_vslam::optimize::pose_optimizer_g2o_trace_clear();
    stella_vslam::optimize::pose_optimizer_g2o_trace_enable(true);

    auto cfg = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_path);
    slam->startup_single_threaded(true);

    tum_rgbd_sequence sequence(seq_dir);
    const auto frames = sequence.get_frames();
    const size_t n = (max_frames >= 0) ? std::min<size_t>(frames.size(), (size_t)max_frames) : frames.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& f = frames[i];
        cv::Mat img = cv::imread(f.rgb_img_path_, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "failed to read " << f.rgb_img_path_ << "\n";
            return 2;
        }
        slam->feed_monocular_frame(img, f.timestamp_);
        slam->synchronize_background_modules();
    }

    const auto& trace = stella_vslam::optimize::pose_optimizer_g2o_trace_get();

    std::ofstream calls_f(out_dir + "/calls.tsv");
    std::ofstream obs_f(out_dir + "/obs.tsv");
    std::ofstream outliers_f(out_dir + "/outliers.tsv");

    calls_f << "call_index\tcall_site\tnum_obs\tfx\tfy\tcx\tcy\t"
            << "num_trials_robust\tnum_trials\tnum_each_iter\t"
            << "initial_pose_hex\tfinal_pose_hex\tnum_valid_obs\n";
    obs_f << "call_index\tidx\tx\ty\toctave\tinv_sigma_sq\tpos_w_hex\n";
    outliers_f << "call_index\tidx\tis_outlier\n";

    for (const auto& t : trace) {
        calls_f << t.call_index << '\t' << t.call_site << '\t' << t.obs.size() << '\t'
                << fmt_double_hex(t.fx) << '\t' << fmt_double_hex(t.fy) << '\t'
                << fmt_double_hex(t.cx) << '\t' << fmt_double_hex(t.cy) << '\t'
                << t.num_trials_robust << '\t' << t.num_trials << '\t' << t.num_each_iter << '\t'
                << fmt_mat44row_hex(t.cam_pose_cw_initial) << '\t'
                << fmt_mat44row_hex(t.cam_pose_cw_final) << '\t'
                << t.num_valid_obs << '\n';

        for (const auto& o : t.obs) {
            std::ostringstream pw;
            pw << fmt_double_hex(o.pos_w[0]) << ',' << fmt_double_hex(o.pos_w[1]) << ',' << fmt_double_hex(o.pos_w[2]);
            obs_f << t.call_index << '\t' << o.idx << '\t'
                  << fmt_double_hex(o.x) << '\t' << fmt_double_hex(o.y) << '\t'
                  << o.octave << '\t' << fmt_double_hex((double)o.inv_sigma_sq) << '\t'
                  << pw.str() << '\n';
        }
        for (size_t idx = 0; idx < t.outlier_flags.size(); ++idx) {
            outliers_f << t.call_index << '\t' << idx << '\t' << (int)t.outlier_flags[idx] << '\n';
        }
    }

    std::cerr << "dump_stella_g2o_pose: " << trace.size() << " pose_optimizer_g2o::optimize() calls captured\n";
    return 0;
}
