// stella_port reference tool, module 4b part 2: bundle-adjustment call
// dumper (local_bundle_adjuster_g2o::optimize, global_bundle_adjuster::
// optimize_for_initialization, global_bundle_adjuster::optimize).
//
// Clean-room: written against stella_vslam's own public API (BSD-2) and the
// BSD-2 example utility tum_rgbd_util.h/.cc, exactly like
// dump_stella_g2o_pose.cc. Nothing here is derived from orb_port/ or
// ORB-SLAM2.
//
// Data source: stella_port/reference/patches/0007-ba-trace.patch adds an
// opt-in (default off) capture buffer (optimize/ba_trace.h) filled from
// INSIDE the real BA functions with the values they themselves computed:
// the map view they read, the g2o vertices/edges they built (insertion
// order), per-iteration chi2/lambda (via terminate_action, after its own
// bookkeeping), the levels of every edge at the start of each optimize()
// stage, the final vertex/edge state and the results applied back to the
// map. No algorithm value is changed. This tool then runs the ordinary
// synchronous single-threaded driver loop (same as
// stella_port/reference/driver/main.cc): every LOCAL BA call and the
// initial-map GLOBAL BA fire from their real call sites. After the last
// frame the tool additionally drives the REAL global_bundle_adjuster::
// optimize() (the loop-closure BA entry point; no loop closure occurs in
// fr1_xyz/fr1_desk) on the final map, keyframes in spanning-tree order
// (get_keyframes_from_root, as loop_bundle_adjuster does), twice
// (huber on/off); this is a real-library call on real map state, not a
// hand-built graph.
//
// Output: <out_dir>/ba_calls.bin, little-endian, format "BAC2":
//   u32 magic 0x32434142, u32 n_calls, then per call:
//   u32 kind(0 local,1 global init,2 global loop) u32 call_index
//   u32 curr_keyfrm_id u32 fixed_kf_id_threshold u32 num_first u32 num_second
//   f64 gain_threshold u32 use_huber u32 fix_markers u32 flag_given u32 flag_in
//   u32 use_additional u32 has_markers u32 ran_optimize u32 returned_ok
//   f64 fx fy cx cy ; u32 n_isq, f32 isq[n_isq]
//   u32 n_order_kf, u32 ids[] ; u32 n_order_lm, u32 ids[]
//   u32 n_mv_kf, per kf: u32 id u32 erased u32 root f64 pose[16] u32 has_slots
//        u32 n_slots u32 slots[] u32 n_kp {u32 idx f32 x f32 y i32 octave}[]
//   u32 n_mv_lm, per lm: u32 id u32 erased f64 pos[3] u32 n_obs {u32 kf u32 idx}[]
//   u32 n_vert, per: u32 vtx_id u32 is_lm u32 fixed u32 owner f64 init[7] f64 final[7]
//   u32 n_edge, per: u32 lm_vtx u32 kf_vtx u32 kf_id u32 lm_id u32 idx
//        f64 obs[2] f64 isq f64 delta u32 has_kernel f64 chi2 f64 err[2]
//        u32 depth_pos u32 level
//   u32 n_stage, per: u32 requested i32 returned u32 flag_after u32 stopped
//        u32 n_levels u8 levels[] pad-to-4 ; u32 n_iter {f64 chi2 f64 lambda i32 lev u32 flag}[]
//   u32 n_outlier {u32 kf u32 lm}[]
//   u32 n_applied {u32 kf f64 pose[16]}[]
//   u32 n_opt_lm u32 ids[]
//
// Determinism: run twice, diff (tools/dump_stella_g2o.py --ba does this).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/publish/map_publisher.h"
#include "stella_vslam/optimize/ba_trace.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

using namespace stella_vslam::optimize;

namespace {

struct Writer {
    std::ofstream f;
    explicit Writer(const std::string& path) : f(path, std::ios::binary) {}
    void u32(uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); }
    void i32(int32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); }
    void f32(float v) { f.write(reinterpret_cast<const char*>(&v), 4); }
    void f64(double v) { f.write(reinterpret_cast<const char*>(&v), 8); }
    void pad4(size_t n) {
        const char z[4] = {0, 0, 0, 0};
        const size_t rem = n % 4;
        if (rem) {
            f.write(z, 4 - rem);
        }
    }
};

void write_call(Writer& w, const ba_trace_call& c) {
    w.u32(c.kind);
    w.u32(c.call_index);
    w.u32(c.curr_keyfrm_id);
    w.u32(c.fixed_keyframe_id_threshold);
    w.u32(c.num_first_iter);
    w.u32(c.num_second_iter);
    w.f64(c.gain_threshold);
    w.u32(c.use_huber);
    w.u32(c.fix_markers);
    w.u32(c.flag_given);
    w.u32(c.flag_in);
    w.u32(c.use_additional_keyframes);
    w.u32(c.has_markers);
    w.u32(c.ran_optimize);
    w.u32(c.returned_ok);
    w.f64(c.fx);
    w.f64(c.fy);
    w.f64(c.cx);
    w.f64(c.cy);
    w.u32((uint32_t)c.inv_level_sigma_sq.size());
    for (float v : c.inv_level_sigma_sq) {
        w.f32(v);
    }
    w.u32((uint32_t)c.order_keyfrm_ids.size());
    for (auto v : c.order_keyfrm_ids) {
        w.u32(v);
    }
    w.u32((uint32_t)c.order_lm_ids.size());
    for (auto v : c.order_lm_ids) {
        w.u32(v);
    }
    w.u32((uint32_t)c.mv_kfs.size());
    for (const auto& id_kf : c.mv_kfs) {
        const auto& k = id_kf.second;
        w.u32(k.id);
        w.u32(k.erased);
        w.u32(k.spanning_root);
        for (int i = 0; i < 16; ++i) {
            w.f64(k.pose_cw[i]);
        }
        w.u32(k.has_lm_slots ? 1 : 0);
        w.u32((uint32_t)k.lm_slots.size());
        for (auto v : k.lm_slots) {
            w.u32(v);
        }
        w.u32((uint32_t)k.kps.size());
        for (const auto& ik : k.kps) {
            w.u32(ik.second.idx);
            w.f32(ik.second.x);
            w.f32(ik.second.y);
            w.i32(ik.second.octave);
        }
    }
    w.u32((uint32_t)c.mv_lms.size());
    for (const auto& id_lm : c.mv_lms) {
        const auto& l = id_lm.second;
        w.u32(l.id);
        w.u32(l.erased);
        for (int i = 0; i < 3; ++i) {
            w.f64(l.pos[i]);
        }
        w.u32((uint32_t)l.obs.size());
        for (const auto& o : l.obs) {
            w.u32(o.first);
            w.u32(o.second);
        }
    }
    w.u32((uint32_t)c.vertices.size());
    for (const auto& v : c.vertices) {
        w.u32(v.vtx_id);
        w.u32(v.is_landmark);
        w.u32(v.fixed);
        w.u32(v.owner_id);
        for (int i = 0; i < 7; ++i) {
            w.f64(v.est_init[i]);
        }
        for (int i = 0; i < 7; ++i) {
            w.f64(v.est_final[i]);
        }
    }
    w.u32((uint32_t)c.edges.size());
    for (const auto& e : c.edges) {
        w.u32(e.lm_vtx_id);
        w.u32(e.kf_vtx_id);
        w.u32(e.kf_id);
        w.u32(e.lm_id);
        w.u32(e.idx);
        w.f64(e.obs[0]);
        w.f64(e.obs[1]);
        w.f64(e.inv_sigma_sq);
        w.f64(e.huber_delta);
        w.u32(e.has_kernel);
        w.f64(e.chi2_final);
        w.f64(e.err_final[0]);
        w.f64(e.err_final[1]);
        w.u32(e.depth_positive_final);
        w.u32(e.level_final);
    }
    w.u32((uint32_t)c.stages.size());
    for (const auto& st : c.stages) {
        w.u32(st.requested_iters);
        w.i32(st.returned_iters);
        w.u32(st.flag_after);
        w.u32(st.stopped_by_terminate ? 1 : 0);
        w.u32((uint32_t)st.edge_level_at_start.size());
        w.f.write(reinterpret_cast<const char*>(st.edge_level_at_start.data()), (std::streamsize)st.edge_level_at_start.size());
        w.pad4(st.edge_level_at_start.size());
        w.u32((uint32_t)st.iters.size());
        for (const auto& it : st.iters) {
            w.f64(it.chi2);
            w.f64(it.lambda);
            w.i32(it.lev_iter);
            w.u32(it.flag);
        }
    }
    w.u32((uint32_t)c.outliers.size());
    for (const auto& o : c.outliers) {
        w.u32(o.first);
        w.u32(o.second);
    }
    w.u32((uint32_t)c.applied_poses.size());
    for (const auto& a : c.applied_poses) {
        w.u32(a.first);
        for (int i = 0; i < 16; ++i) {
            w.f64(a.second[i]);
        }
    }
    w.u32((uint32_t)c.optimized_lm_ids.size());
    for (auto v : c.optimized_lm_ids) {
        w.u32(v);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "usage: dump_stella_g2o_ba <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir> <max_frames|-1>\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string config_path = argv[2];
    const std::string seq_dir = argv[3];
    const std::string out_dir = argv[4];
    const long max_frames = std::atol(argv[5]);

    ba_trace_clear();
    ba_trace_enable(true);

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

    // Real loop-BA entry point on the final map (see header comment).
    {
        std::vector<std::shared_ptr<stella_vslam::data::keyframe>> all_keyfrms;
        slam->get_map_publisher()->get_keyframes(all_keyfrms);
        std::shared_ptr<stella_vslam::data::keyframe> root;
        for (const auto& kf : all_keyfrms) {
            if (kf && kf->graph_node_->is_spanning_root()) {
                root = kf;
                break;
            }
        }
        if (root) {
            const auto keyfrms = root->graph_node_->get_keyframes_from_root();
            for (int use_huber = 1; use_huber >= 0; --use_huber) {
                std::unordered_set<unsigned int> okf, olm, omk;
                stella_vslam::eigen_alloc_unord_map<unsigned int, stella_vslam::Vec3_t> lm_pos;
                stella_vslam::eigen_alloc_unord_map<unsigned int, stella_vslam::Mat44_t> kf_pose;
                stella_vslam::eigen_alloc_unord_map<unsigned int, std::array<stella_vslam::Vec3_t, 4>> mk_pos;
                const stella_vslam::optimize::global_bundle_adjuster gba(10, use_huber != 0, false);
                gba.optimize(keyfrms, okf, olm, omk, lm_pos, kf_pose, mk_pos, nullptr);
            }
        }
    }

    const auto& trace = ba_trace_get();
    int n_local = 0, n_init = 0, n_loop = 0;
    Writer w(out_dir + "/ba_calls.bin");
    w.u32(0x32434142u);
    w.u32((uint32_t)trace.size());
    for (const auto& c : trace) {
        write_call(w, c);
        (c.kind == 0 ? n_local : c.kind == 1 ? n_init : n_loop)++;
    }
    w.f.close();

    std::cerr << "dump_stella_g2o_ba: " << trace.size() << " BA calls captured (local " << n_local
              << ", global-init " << n_init << ", global-loop " << n_loop << ")\n";
    return 0;
}
