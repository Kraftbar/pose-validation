// stella_port reference tool: frame construction (grid + area/cell
// queries) and bag-of-words dumper, module 2 of the pure-C stella_vslam
// port.
//
// Clean-room: written against stella_vslam's own public API
// (system.h/config.h/data/frame.h/data/frame_observation.h, all BSD-2) and
// the BSD-2 example utility tum_rgbd_util.h/.cc (copied verbatim from
// stella_vslam_examples via stella_port/reference/driver/, same license,
// see LICENSE.tum_rgbd_util in this directory). Nothing here is derived
// from orb_port/ or ORB-SLAM2 -- only stella_vslam's own source
// (runs/stella_port/reference_build/src, external/candidates/stella_vslam)
// and FBoW's own source/README (external/candidates/stella_vslam/3rd/FBoW,
// MIT) were read.
//
// Drives the SAME synchronous, deterministic pipeline as
// stella_port/reference/driver/main.cc (system::startup_single_threaded()
// + feed_monocular_frame() + synchronize_background_modules(), same
// reference config/vocab) -- i.e. this builds each data::frame exactly the
// way stella's own monocular tracking does, with the real
// feature::orb_extractor + camera::perspective + fbow::Vocabulary. After
// each feed_monocular_frame() call, tracker->curr_frm_ already carries:
//   - frm_obs_.keypt_indices_in_cells_ / num_grid_cols_ / num_grid_rows_
//     (built by data::assign_keypoints_to_grid() inside system.cc's
//     extract_orb(), before tracking runs)
//   - bow_vec_ / bow_feat_vec_ (set by frame::compute_bow(), called from
//     tracking_module::track() at level storeLevel=4 --
//     data/bow_vocabulary_util.cc: bow_vocab->transform(descriptors, 4, ...))
// so this tool reads those fields straight off curr_frm_/get_keypoints_in_cell()
// rather than reconstructing a frame by hand.
//
// Dumps, per sequence, under <out_dir>:
//   grid_meta.tsv    frame_idx  num_grid_cols  num_grid_rows
//   grid.tsv         frame_idx  cell_x  cell_y  kp_indices (comma-separated,
//                    container order == push_back order == keypoint index
//                    order); only non-empty cells are written (empty is the
//                    default state and need not be checked row-by-row)
//   area_query.tsv   frame_idx  query_kp_idx  margin  min_level  max_level
//                    result_kp_indices (comma-separated, in the exact order
//                    get_keypoints_in_cell() returns them)
//   bow_vec.tsv      frame_idx  word_id  weight_9g  weight_hex
//                    (container order == std::map<uint32_t,_float> order,
//                    i.e. ascending word_id)
//   bow_feat.tsv     frame_idx  node_id  kp_indices (comma-separated, in
//                    push_back order); container order == ascending node_id
//   score.tsv        frame_a  frame_b  score_17g  score_hex (double)
//
// bow_vec.tsv/bow_feat.tsv/score.tsv cover EVERY frame, not just the ones
// stella's own tracking happens to call frame::compute_bow() on: stella
// only calls it when the motion model fails (see tracking_module.cc), but
// keyframes also compute BoW (mapping/relocalization/loop detection all
// exercise it), so this tool loads its own fbow::Vocabulary instance
// (independent of tracking_module's protected bow_vocab_ pointer, same
// vocab file) and calls Vocabulary::transform(frm_obs_.descriptors_, 4,
// ...) explicitly for every frame's curr_frm_, after feed_monocular_frame()
// has built it -- same descriptors, same level, same call
// frame::compute_bow() would have made
// (data/bow_vocabulary_util.cc: bow_vocab->transform(descriptors, 4, ...)).
// score.tsv then runs fbow::BoWVector::score() between each frame i and
// i+1 (consecutive) and i and i+50 (far apart), over these explicitly
// computed vectors, for every valid i.
//
// Query set (fixed, matches the module-2 task spec): every 25th keypoint
// index (0, 25, 50, ...) as (ref_x, ref_y) = undist_keypts_[q].pt, crossed
// with margin in {5, 15, 50} and (min_level, max_level) in
// {(-1,-1), (0,0), (0,3), (2,7)}.
//
// Determinism: this pipeline is the same single-threaded, DETERMINISTIC=ON
// build as stella_port/reference/driver -- run twice and diff to confirm
// (tools/dump_stella_frame_bow.py does this).

#include "tum_rgbd_util.h"

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/frame_observation.h"

#include <fbow/vocabulary.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string fmt_float(float v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return std::string(buf);
}

std::string fmt_float_hex(float v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", (double)v);
    return std::string(buf);
}

std::string fmt_double(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

std::string fmt_double_hex(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}

template <typename Vec>
std::string join_csv(const Vec& v) {
    std::ostringstream oss;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) {
            oss << ',';
        }
        oss << v[i];
    }
    return oss.str();
}

const int MARGINS[] = {5, 15, 50};
const int LEVEL_RANGES[][2] = {{-1, -1}, {0, 0}, {0, 3}, {2, 7}};

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "usage: dump_frame_bow <vocab.fbow> <config.yaml> <tum_seq_dir> <out_dir> <max_frames|-1>\n";
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

    // Own vocabulary instance (see header comment) -- same file, same
    // fbow::Vocabulary::transform() call frame::compute_bow() makes, but
    // invoked explicitly for every frame regardless of tracking_module's
    // internal call pattern.
    fbow::Vocabulary vocab;
    vocab.readFromFile(vocab_path);
    if (!vocab.isValid()) {
        std::cerr << "failed to load vocabulary: " << vocab_path << "\n";
        return 2;
    }

    tum_rgbd_sequence sequence(seq_dir);
    const auto frames = sequence.get_frames();

    std::ofstream grid_meta(out_dir + "/grid_meta.tsv");
    std::ofstream grid(out_dir + "/grid.tsv");
    std::ofstream area_query(out_dir + "/area_query.tsv");
    std::ofstream bow_vec_f(out_dir + "/bow_vec.tsv");
    std::ofstream bow_feat_f(out_dir + "/bow_feat.tsv");
    std::ofstream score_f(out_dir + "/score.tsv");

    grid_meta << "frame_idx\tnum_grid_cols\tnum_grid_rows\n";
    grid << "frame_idx\tcell_x\tcell_y\tkp_indices\n";
    area_query << "frame_idx\tquery_kp_idx\tmargin\tmin_level\tmax_level\tresult_kp_indices\n";
    bow_vec_f << "frame_idx\tword_id\tweight_9g\tweight_hex\n";
    bow_feat_f << "frame_idx\tnode_id\tkp_indices\n";
    score_f << "frame_a\tframe_b\tscore_17g\tscore_hex\n";

    const size_t n = (max_frames >= 0) ? std::min<size_t>(frames.size(), (size_t)max_frames) : frames.size();
    std::vector<fbow::BoWVector> bow_vecs(n); // indexed by frame_idx, filled below
    for (size_t i = 0; i < n; ++i) {
        const auto& f = frames[i];
        cv::Mat img = cv::imread(f.rgb_img_path_, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "failed to read " << f.rgb_img_path_ << "\n";
            return 2;
        }

        slam->feed_monocular_frame(img, f.timestamp_);
        slam->synchronize_background_modules();

        if (!tracker) {
            continue;
        }
        const auto& frm = tracker->curr_frm_;
        const auto& fo = frm.frm_obs_;

        // ---- grid ----
        grid_meta << i << '\t' << fo.num_grid_cols_ << '\t' << fo.num_grid_rows_ << '\n';
        for (unsigned int cx = 0; cx < fo.num_grid_cols_ && cx < fo.keypt_indices_in_cells_.size(); ++cx) {
            const auto& col = fo.keypt_indices_in_cells_[cx];
            for (unsigned int cy = 0; cy < fo.num_grid_rows_ && cy < col.size(); ++cy) {
                const auto& cell = col[cy];
                if (cell.empty()) {
                    continue;
                }
                grid << i << '\t' << cx << '\t' << cy << '\t' << join_csv(cell) << '\n';
            }
        }

        // ---- area/cell queries ----
        const auto& kpts = fo.undist_keypts_;
        for (size_t q = 0; q < kpts.size(); q += 25) {
            const float ref_x = kpts[q].pt.x;
            const float ref_y = kpts[q].pt.y;
            for (int margin : MARGINS) {
                for (const auto& lvl : LEVEL_RANGES) {
                    auto result = frm.get_keypoints_in_cell((float)ref_x, (float)ref_y, (float)margin, lvl[0], lvl[1]);
                    area_query << i << '\t' << q << '\t' << margin << '\t' << lvl[0] << '\t' << lvl[1] << '\t'
                               << join_csv(result) << '\n';
                }
            }
        }

        // ---- bow (explicit, every frame -- see header comment) ----
        fbow::BoWVector bow_vec;
        fbow::BoWFeatVector bow_feat_vec;
        vocab.transform(fo.descriptors_, 4, bow_vec, bow_feat_vec);
        bow_vecs[i] = bow_vec;

        for (const auto& e : bow_vec) {
            bow_vec_f << i << '\t' << e.first << '\t' << fmt_float((float)e.second) << '\t'
                      << fmt_float_hex((float)e.second) << '\n';
        }
        for (const auto& e : bow_feat_vec) {
            bow_feat_f << i << '\t' << e.first << '\t' << join_csv(e.second) << '\n';
        }
    }

    // ---- scores: (i, i+1) consecutive and (i, i+50) far-apart, every
    // valid i ----
    for (size_t i = 0; i < n; ++i) {
        if (i + 1 < n) {
            double s = fbow::BoWVector::score(bow_vecs[i], bow_vecs[i + 1]);
            score_f << i << '\t' << (i + 1) << '\t' << fmt_double(s) << '\t' << fmt_double_hex(s) << '\n';
        }
        if (i + 50 < n) {
            double s = fbow::BoWVector::score(bow_vecs[i], bow_vecs[i + 50]);
            score_f << i << '\t' << (i + 50) << '\t' << fmt_double(s) << '\t' << fmt_double_hex(s) << '\n';
        }
    }

    std::cerr << "dump_frame_bow: wrote " << n << " frames to " << out_dir << "\n";
    return 0;
}
