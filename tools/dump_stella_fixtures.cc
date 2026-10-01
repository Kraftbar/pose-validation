// tools/dump_stella_fixtures.cc
//
// Produces the exact grayscale input the stella_port reference driver feeds
// to feature::orb_extractor::extract() for each frame of a TUM sequence, as
// 8-bit PGMs the C port's harnesses can decode without any PNG/color-space
// code of their own.
//
// Clean-room: this file only reproduces two small, already-public pieces of
// stella_vslam's own driver-side code, both BSD-2 and already vendored in
// this repo:
//   - stella_port/reference/driver/tum_rgbd_util.{h,cc} frame ordering
//     (rgb.txt row order, header skipped, nearest-depth matching with the
//     same 0.1s default threshold -- reused verbatim here only to keep
//     fixture frame_idx aligned with runs/stella_port/reference_dumps
//     frame_idx; the depth match itself is irrelevant to the image we dump).
//   - runs/stella_port/reference_build/src/src/stella_vslam/util/image_converter.cc
//     convert_to_grayscale(): cv::cvtColor(img, img, cv::COLOR_RGB2GRAY) for
//     color_order "RGB" (the reference config's Camera.color_order), the
//     only branch exercised by the deterministic TUM RGB-D config.
// Linked against the exact OpenCV 4.6.0 build
// (PKG_CONFIG_PATH=/tmp/pose-opencv/pkgconfig, same as
// runs/stella_port/reference_build/driver_build) so cv::imread +
// cv::cvtColor byte-match the C++ reference driver's own image_pyramid_[0]
// input, not just "a" OpenCV's.

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct img_info {
    double timestamp;
    std::string path;
};

std::vector<img_info> read_index(const std::string& seq_dir, const std::string& list_file) {
    std::vector<img_info> out;
    std::ifstream ifs(list_file);
    if (!ifs) {
        throw std::runtime_error("cannot open " + list_file);
    }
    std::string s;
    // header: 3 lines
    std::getline(ifs, s);
    std::getline(ifs, s);
    std::getline(ifs, s);
    while (!ifs.eof()) {
        std::getline(ifs, s);
        if (s.empty()) {
            continue;
        }
        std::stringstream ss(s);
        double t;
        std::string name;
        ss >> t >> name;
        out.push_back({t, seq_dir + "/" + name});
    }
    return out;
}

// Same association as tum_rgbd_sequence::tum_rgbd_sequence (nearest depth
// frame per RGB frame, default 0.1s threshold) -- only used to reproduce the
// reference driver's frame_idx -> rgb image ordering; the depth path itself
// is discarded here since this tool only dumps grayscale RGB frames.
std::vector<std::string> ordered_rgb_paths(const std::string& seq_dir, double min_timediff_thr = 0.1) {
    const auto rgb = read_index(seq_dir, seq_dir + "/rgb.txt");
    const auto depth = read_index(seq_dir, seq_dir + "/depth.txt");
    std::vector<std::string> out;
    for (const auto& r : rgb) {
        double best_dt = std::abs(r.timestamp - depth.front().timestamp);
        for (const auto& d : depth) {
            double dt = std::abs(r.timestamp - d.timestamp);
            if (dt < best_dt) {
                best_dt = dt;
            }
        }
        if (best_dt > min_timediff_thr) {
            continue;
        }
        out.push_back(r.path);
    }
    return out;
}

bool write_pgm(const std::string& path, const cv::Mat& gray) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        return false;
    }
    std::fprintf(f, "P5\n%d %d\n255\n", gray.cols, gray.rows);
    for (int y = 0; y < gray.rows; ++y) {
        std::fwrite(gray.ptr<unsigned char>(y), 1, gray.cols, f);
    }
    std::fclose(f);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: dump_stella_fixtures <seq_dir> <out_dir> [max_frames]\n";
        return 1;
    }
    const std::string seq_dir = argv[1];
    const std::string out_dir = argv[2];
    const long max_frames = argc > 3 ? std::atol(argv[3]) : -1;

    const auto rgb_paths = ordered_rgb_paths(seq_dir);
    const size_t n = (max_frames >= 0) ? std::min<size_t>(rgb_paths.size(), (size_t)max_frames) : rgb_paths.size();

    for (size_t i = 0; i < n; ++i) {
        cv::Mat img = cv::imread(rgb_paths[i], cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            std::cerr << "failed to read " << rgb_paths[i] << "\n";
            return 2;
        }
        if (img.channels() == 3) {
            cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
        }
        else if (img.channels() == 4) {
            cv::cvtColor(img, img, cv::COLOR_RGBA2GRAY);
        }
        // channels() == 1: already gray, left as-is (matches
        // convert_to_grayscale(), which no-ops in that case).
        char name[64];
        std::snprintf(name, sizeof(name), "/%06zu.pgm", i);
        if (!write_pgm(out_dir + name, img)) {
            std::cerr << "failed to write " << out_dir + name << "\n";
            return 3;
        }
    }
    std::cout << n << " frames written to " << out_dir << "\n";
    return 0;
}
