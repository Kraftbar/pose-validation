// RD-VIO reference tooling: the undistorted image stream of the reference driver, as a gray pack.
// usage: rd_undistort_pack <sensor.yaml> <in cam0.gray> <out undistorted.gray>
// Same calls as runs/rdvio_port/*_reference_build/rdvio_gray_driver.cpp: initUndistortRectifyMap (radtan, or fisheye for
// "equidistant") with K as the new camera matrix, CV_32FC1 maps, remap INTER_LINEAR, cv::setNumThreads(1).
// Pack format (input and output): "OKGRAY1\0", u32 width, height, count, then count x {u64 stamp, width*height bytes}.
// The C system driver (rdvio_port/c/rdvio_c_euroc.c) reads the output until the remap is ported (module M7b).
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
int main(int argc, char **argv) {
    if (argc < 4) { std::fprintf(stderr, "usage: rd_undistort_pack sensor.yaml in.gray out.gray\n"); return 1; }
    cv::setNumThreads(1);
    YAML::Node y = YAML::LoadFile(argv[1]);
    auto K = y["cam0"]["intrinsics"].as<std::vector<double>>();
    auto D = y["cam0"]["distortion"].as<std::vector<double>>();
    auto res = y["cam0"]["resolution"].as<std::vector<int>>();
    cv::Mat Km = (cv::Mat_<double>(3, 3) << K[0], 0, K[2], 0, K[1], K[3], 0, 0, 1);
    cv::Mat Dm = (cv::Mat_<double>(1, 4) << D[0], D[1], D[2], D[3]);
    cv::Mat m1, m2;
    std::string dmodel = y["cam0"]["distortion_model"] ? y["cam0"]["distortion_model"].as<std::string>() : "radtan";
    if (dmodel == "equidistant")
        cv::fisheye::initUndistortRectifyMap(Km, Dm, cv::Mat::eye(3, 3, CV_64F), Km, cv::Size(res[0], res[1]), CV_32FC1, m1, m2);
    else
        cv::initUndistortRectifyMap(Km, Dm, cv::Mat(), Km, cv::Size(res[0], res[1]), CV_32FC1, m1, m2);
    FILE *in = std::fopen(argv[2], "rb"), *out = std::fopen(argv[3], "wb");
    char magic[8]; uint32_t gh[3];
    if (!in || !out || std::fread(magic, 1, 8, in) != 8 || std::memcmp(magic, "OKGRAY1\0", 8) || std::fread(gh, 4, 3, in) != 3 ||
        gh[0] != (uint32_t)res[0] || gh[1] != (uint32_t)res[1]) { std::fprintf(stderr, "bad input pack\n"); return 2; }
    std::fwrite(magic, 1, 8, out); std::fwrite(gh, 4, 3, out);
    for (uint32_t i = 0; i < gh[2]; ++i) {
        uint64_t stamp;
        cv::Mat raw(res[1], res[0], CV_8UC1), un;
        if (std::fread(&stamp, 8, 1, in) != 1 || std::fread(raw.data, 1, raw.total(), in) != raw.total()) { std::fprintf(stderr, "short pack\n"); return 3; }
        cv::remap(raw, un, m1, m2, cv::INTER_LINEAR);
        if (!un.isContinuous()) un = un.clone();
        std::fwrite(&stamp, 8, 1, out);
        std::fwrite(un.data, 1, un.total(), out);
    }
    std::fclose(in);
    return std::fclose(out) == 0 ? 0 : 4;
}
