// Image decoding for the BRISK hookup of check_ok_frontend: decodes every EuRoC PNG of one camera exactly like the
// reference's DatasetReader (cv::imread(filename, cv::IMREAD_GRAYSCALE), OpenCV 4.6 of external/vio/deps/opencv) and
// packs the 8-bit images into one file, so the C harness needs no PNG decoder.
//
//   okvis_png2gray <cam data dir with <timestamp_ns>.png> <out.gray>
//
// out.gray: "OKGRAY1\0", u32 width, u32 height, u32 n, then n x { u64 timestamp_ns, u8 pixels[width * height] (row-major) },
// sorted by timestamp. Build: g++ -O2 -std=c++17 okvis_png2gray.cc -I<ocv>/include/opencv4 -L<ocv>/lib
//   -lopencv_imgcodecs -lopencv_core -Wl,-rpath,<ocv>/lib
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 3) { std::fprintf(stderr, "okvis_png2gray <png dir> <out.gray>\n"); return 2; }
  cv::setNumThreads(1);
  std::vector<std::pair<uint64_t, std::string>> files;
  for (const auto& e : std::filesystem::directory_iterator(argv[1])) {
    if (e.path().extension() != ".png") continue;
    files.emplace_back(std::stoull(e.path().stem().string()), e.path().string());
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) { std::fprintf(stderr, "no PNG in %s\n", argv[1]); return 1; }
  FILE* f = std::fopen(argv[2], "wb");
  if (!f) return 1;
  uint32_t w = 0, h = 0, n = (uint32_t)files.size();
  std::fwrite("OKGRAY1", 8, 1, f);
  std::fwrite(&w, 4, 1, f); std::fwrite(&h, 4, 1, f); std::fwrite(&n, 4, 1, f);
  for (const auto& fl : files) {
    cv::Mat im = cv::imread(fl.second, cv::IMREAD_GRAYSCALE);
    if (im.empty() || im.type() != CV_8UC1 || !im.isContinuous()) { std::fprintf(stderr, "bad image %s\n", fl.second.c_str()); return 1; }
    if (w == 0) { w = (uint32_t)im.cols; h = (uint32_t)im.rows; }
    if ((uint32_t)im.cols != w || (uint32_t)im.rows != h) { std::fprintf(stderr, "size change at %s\n", fl.second.c_str()); return 1; }
    std::fwrite(&fl.first, 8, 1, f);
    std::fwrite(im.data, (size_t)w * h, 1, f);
  }
  std::fseek(f, 8, SEEK_SET);
  std::fwrite(&w, 4, 1, f); std::fwrite(&h, 4, 1, f);
  std::fclose(f);
  std::printf("%u images %ux%u -> %s\n", n, w, h, argv[2]);
  return 0;
}
