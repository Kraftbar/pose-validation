// basalt_port module M4 oracle: C port (bs_image.c / bs_fast.c) vs the real basalt-headers / basalt / OpenCV 4.6 code, tolerance 0.
//   bs_image_test sort   [seed N]        bs_fast_sort_response_desc vs std::sort (ties), and the heap-sort fallback vs std::__introsort_loop
//   bs_image_test fast   [seed N]        bs_fast9_16 vs cv::FAST (synthetic images, EuRoC cells), x, y, response, order
//   bs_image_test image  <seqdir> [step] PNG loader vs cv::imread(UNCHANGED) << 8, pyramids vs basalt::ManagedImagePyr (whole mipmap)
//   bs_image_test detect <seqdir> [step] bs_detect_keypoints vs basalt::detectKeypoints on real frames + synthetic images
//   bs_image_test replay <flow.bin> <seqdir>   C loader + C detector vs the M0 FLOW dump (image hash; new keypoints)
//   bs_image_test sens                   sensitivity: a deliberately wrong C variant must be caught
// Last stdout line: "<label>: <bad>/<total>"; exit 0 iff bad == 0.
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <basalt/image/image_pyr.h>
#include <basalt/utils/keypoints.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

extern "C" {
#include "bs_fast.h"
#include "bs_image.h"
}

static uint64_t g_total = 0, g_bad = 0;
static void chk(bool ok, const char* what, const std::string& ctx = "") {
  g_total++;
  if (!ok) {
    g_bad++;
    if (g_bad <= 20) fprintf(stderr, "  MISMATCH %s %s\n", what, ctx.c_str());
  }
}
static int finish(const char* label) {
  printf("%s: %llu/%llu\n", label, (unsigned long long)g_bad, (unsigned long long)g_total);
  return g_bad == 0 ? 0 : 1;
}

typedef std::mt19937_64 Rng;

// ------------------------------------------------------------------ images
static void make_image(Rng& r, int kind, int w, int h, std::vector<uint8_t>& im) {
  im.assign((size_t)w * h, 0);
  std::uniform_int_distribution<int> u8(0, 255);
  switch (kind) {
    case 0:  // white noise
      for (auto& p : im) p = (uint8_t)u8(r);
      break;
    case 1: {  // smooth gradient + noise of random amplitude
      int amp = 1 + (int)(r() % 60);
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) im[(size_t)y * w + x] = (uint8_t)std::min(255, std::max(0, 40 + x * 2 + y + (int)(r() % (2 * amp + 1)) - amp));
      break;
    }
    case 2: {  // random blocks, high contrast (many corners with equal scores)
      int bs = 2 + (int)(r() % 6);
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          uint64_t hsh = (uint64_t)(x / bs) * 0x9E3779B97F4A7C15ull ^ (uint64_t)(y / bs) * 0xC2B2AE3D27D4EB4Full ^ (r() & 0);
          im[(size_t)y * w + x] = (hsh >> 33) & 1 ? 230 : 25;
        }
      break;
    }
    case 3:  // few gray levels (ties everywhere)
      for (auto& p : im) p = (uint8_t)(100 + 15 * (int)(r() % 3));
      break;
    case 4:  // saturated extremes
      for (auto& p : im) p = (r() & 1) ? 255 : 0;
      break;
    case 5: {  // sparse isolated bright / dark dots on a flat field (isolated corners at threshold edges)
      int base = 20 + (int)(r() % 200);
      for (auto& p : im) p = (uint8_t)base;
      int n = (int)(r() % (w * h / 10 + 1));
      for (int i = 0; i < n; i++) im[r() % im.size()] = (uint8_t)(base + (int)(r() % 61) - 30 > 0 ? std::min(255, std::max(0, base + (int)(r() % 61) - 30)) : 0);
      break;
    }
    default: {  // low-pass noise (blurred) + step edges
      std::vector<int> t((size_t)w * h);
      for (auto& p : t) p = (int)(r() % 256);
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          int s = 0, c = 0;
          for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++) {
              int xx = x + dx, yy = y + dy;
              if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
              s += t[(size_t)yy * w + xx];
              c++;
            }
          im[(size_t)y * w + x] = (uint8_t)(s / c);
        }
    }
  }
}

// ------------------------------------------------------------------- sort
struct Tk { bs_fast_kp k; int id; };

static void test_sort(uint64_t seed) {
  Rng r(seed);
  for (int it = 0; it < 6000; it++) {
    size_t n = it < 40 ? (size_t)it : (it < 3000 ? (size_t)(r() % 40) : (size_t)(r() % 2600));
    int levels = 1 + (int)(r() % 4) * (int)(r() % 40);  // number of distinct response values: many ties
    std::vector<cv::KeyPoint> a(n);
    std::vector<bs_fast_kp> b(n);
    for (size_t i = 0; i < n; i++) {
      float resp = (float)(r() % (uint64_t)levels);
      a[i] = cv::KeyPoint((float)i, (float)(i * 7 % 13), 7.f, -1, resp);
      b[i] = {(float)i, (float)(i * 7 % 13), resp};
    }
    std::sort(a.begin(), a.end(), [](const cv::KeyPoint& x, const cv::KeyPoint& y) { return x.response > y.response; });
    bs_fast_sort_response_desc(b.data(), n);
    bool ok = true;
    for (size_t i = 0; i < n; i++) ok &= a[i].pt.x == b[i].x && a[i].response == b[i].response;
    chk(ok, "sort", "n=" + std::to_string(n) + " levels=" + std::to_string(levels));
  }
  // explicit depth limits: the heap-sort fallback (std::__introsort_loop is a libstdc++ internal, used only to reach that branch)
  for (int it = 0; it < 3000; it++) {
    size_t n = 17 + (size_t)(r() % 700);
    long depth = (long)(r() % 4);
    int levels = 2 + (int)(r() % 50);
    std::vector<cv::KeyPoint> a(n);
    std::vector<bs_fast_kp> b(n);
    for (size_t i = 0; i < n; i++) {
      float resp = (float)(r() % (uint64_t)levels);
      a[i] = cv::KeyPoint((float)i, 0.f, 7.f, -1, resp);
      b[i] = {(float)i, 0.f, resp};
    }
    auto cmp = [](const cv::KeyPoint& x, const cv::KeyPoint& y) { return x.response > y.response; };
    std::__introsort_loop(a.begin(), a.end(), depth, __gnu_cxx::__ops::__iter_comp_iter(cmp));
    std::__final_insertion_sort(a.begin(), a.end(), __gnu_cxx::__ops::__iter_comp_iter(cmp));
    bs_fast_sort_depth_test(b.data(), n, depth);
    bool ok = true;
    for (size_t i = 0; i < n; i++) ok &= a[i].pt.x == b[i].x && a[i].response == b[i].response;
    chk(ok, "sort_depth", "n=" + std::to_string(n) + " depth=" + std::to_string(depth));
  }
}

// ------------------------------------------------------------------- FAST
static bool fast_same(const cv::Mat& m, int threshold, bool nonmax, const char* ctx, uint64_t* nk = nullptr) {
  std::vector<cv::KeyPoint> kp;
  cv::FAST(m, kp, threshold, nonmax);
  std::vector<bs_fast_kp> out(m.rows * m.cols + 1);
  int n = bs_fast9_16(m.data, m.cols, m.rows, (int)m.step, threshold, nonmax, out.data(), (int)out.size());
  bool ok = n == (int)kp.size();
  if (ok)
    for (int i = 0; i < n; i++) ok &= kp[i].pt.x == out[i].x && kp[i].pt.y == out[i].y && memcmp(&kp[i].response, &out[i].response, 4) == 0;
  if (nk) *nk += kp.size();
  chk(ok, "fast", std::string(ctx) + " thr=" + std::to_string(threshold) + " nonmax=" + std::to_string(nonmax) + " n_cv=" + std::to_string(kp.size()) + " n_c=" + std::to_string(n));
  return ok;
}

static void test_fast(uint64_t seed) {
  Rng r(seed);
  uint64_t nk = 0;
  const int thr_set[] = {40, 20, 10, 5};
  for (int it = 0; it < 3000; it++) {
    int w = it < 1500 ? 50 : 7 + (int)(r() % 120);
    int h = it < 1500 ? 50 : 7 + (int)(r() % 100);
    if (it % 500 == 499) { w = 752; h = 480; }
    int kind = (int)(r() % 7);
    std::vector<uint8_t> im;
    make_image(r, kind, w, h, im);
    cv::Mat m(h, w, CV_8U, im.data());
    for (int t : thr_set) fast_same(m, t, true, ("kind" + std::to_string(kind) + " " + std::to_string(w) + "x" + std::to_string(h)).c_str(), &nk);
    int t = 1 + (int)(r() % 127);
    fast_same(m, t, true, "randthr", &nk);
    if (it % 4 == 0) fast_same(m, thr_set[r() % 4], false, "nonmax0", &nk);
  }
  fprintf(stderr, "  fast: %llu cv keypoints compared\n", (unsigned long long)nk);
  printf("  keypoints compared: %llu\n", (unsigned long long)nk);
}

// ------------------------------------------------------------- real frames
static std::vector<std::string> read_names(const std::string& seq) {
  std::vector<std::string> names;
  FILE* f = fopen((seq + "/mav0/cam0/data.csv").c_str(), "r");
  if (!f) { fprintf(stderr, "cannot open data.csv\n"); exit(2); }
  char line[512];
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#') continue;
    char* c = strchr(line, ',');
    if (!c) continue;
    std::string s(c + 1);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    names.push_back(s);
  }
  fclose(f);
  return names;
}

static void real_u16(const std::string& path, basalt::ManagedImage<uint16_t>& out, bool& ok) {
  cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
  ok = img.type() == CV_8UC1;
  if (!ok) return;
  out.Reinitialise(img.cols, img.rows);
  for (size_t i = 0; i < (size_t)img.cols * img.rows; i++) out.ptr[i] = (uint16_t)((int)img.ptr()[i] << 8);
}

static void test_image(const std::string& seq, int step) {
  auto names = read_names(seq);
  Rng r(5);
  for (size_t f = 0; f < names.size(); f += step)
    for (int cam = 0; cam < 2; cam++) {
      std::string p = seq + "/mav0/cam" + std::to_string(cam) + "/data/" + names[f];
      basalt::ManagedImage<uint16_t> ref;
      bool ok;
      real_u16(p, ref, ok);
      uint16_t* mine = nullptr;
      int w = 0, h = 0;
      int rc = bs_image_load_euroc(p.c_str(), &mine, &w, &h);
      chk(ok && rc == BS_IMG_OK && (size_t)w == ref.w && (size_t)h == ref.h && memcmp(mine, ref.ptr, ref.size() * 2) == 0, "load", p);
      if (rc != BS_IMG_OK) continue;
      for (int levels : {3, 1, 5}) {
        if (levels != 3 && (f / step) % 7) continue;
        basalt::ManagedImagePyr<uint16_t> pr;
        pr.setFromImage(ref, levels);
        bs_pyr cp = {};
        bs_pyr_set(&cp, mine, w, h, levels);
        const auto mm = pr.mipmap();
        bool same = (int)mm.w == cp.pitch && (int)mm.h == cp.h && memcmp(mm.ptr, cp.data, (size_t)cp.pitch * cp.h * 2) == 0;
        chk(same, "pyr", p + " levels=" + std::to_string(levels));
        bs_pyr_free(&cp);
      }
      free(mine);
    }
  // in-memory PNGs written by cv::imencode: 8-bit gray must equal imdecode(UNCHANGED) << 8, everything else must be refused (not silently converted)
  for (int it = 0; it < 300; it++) {
    int w = 1 + (int)(r() % 90), h = 1 + (int)(r() % 70);
    int kind = it % 4;  // 0: 8U1, 1: 16U1, 2: 8U3, 3: 8U4
    cv::Mat m(h, w, kind == 1 ? CV_16UC1 : kind == 0 ? CV_8UC1 : kind == 2 ? CV_8UC3 : CV_8UC4);
    for (size_t i = 0; i < m.total() * m.elemSize(); i++) m.data[i] = (uint8_t)((it % 8 < 4) ? r() : (r() % 3) * 100);
    std::vector<uint8_t> png;
    cv::imencode(".png", m, png, {cv::IMWRITE_PNG_COMPRESSION, (int)(r() % 10)});
    cv::Mat back = cv::imdecode(png, cv::IMREAD_UNCHANGED);
    uint16_t* o = nullptr;
    int ow, oh;
    int rc = bs_image_decode_euroc(png.data(), png.size(), &o, &ow, &oh);
    if (kind == 0) {
      bool ok = rc == BS_IMG_OK && back.type() == CV_8UC1 && ow == w && oh == h;
      for (int i = 0; ok && i < w * h; i++) ok &= o[i] == (uint16_t)((int)back.data[i] << 8);
      chk(ok, "decode_gray8", std::to_string(w) + "x" + std::to_string(h));
    } else chk(rc == BS_IMG_UNSUPPORTED && !o, "decode_refuse", "kind " + std::to_string(kind));
    free(o);
  }
  // synthetic sizes, including odd ones
  for (int it = 0; it < 600; it++) {
    int w = 8 + (int)(r() % 200), h = 8 + (int)(r() % 150), levels = 1 + (int)(r() % 4);
    if (it % 3 == 0) { w &= ~1; h &= ~1; }
    // subsample() reflects with abs(2c - 2), so every source level must be >= 3 pixels in both directions (else the real code reads out of bounds too)
    while (levels > 1 && (std::min(w, h) >> (levels - 1)) < 3) levels--;
    basalt::ManagedImage<uint16_t> im(w, h);
    int mode = (int)(r() % 3);
    for (size_t i = 0; i < im.size(); i++) im.ptr[i] = mode == 0 ? (uint16_t)r() : mode == 1 ? (uint16_t)((r() & 255) << 8) : (r() & 1 ? 65535 : 0);
    basalt::ManagedImagePyr<uint16_t> pr;
    pr.setFromImage(im, levels);
    bs_pyr cp = {};
    bs_pyr_set(&cp, im.ptr, w, h, levels);
    const auto mm = pr.mipmap();
    chk((int)mm.w == cp.pitch && (int)mm.h == cp.h && memcmp(mm.ptr, cp.data, (size_t)cp.pitch * cp.h * 2) == 0, "pyr_synth", std::to_string(w) + "x" + std::to_string(h) + " L" + std::to_string(levels));
    bs_pyr_free(&cp);
  }
}

static bool detect_same(const basalt::Image<const uint16_t>& lvl0, int pitch, int grid, int npc, const std::vector<Eigen::Vector2d>& cur, const std::string& ctx, uint64_t* ncorn) {
  basalt::KeypointsData kd;
  Eigen::aligned_vector<Eigen::Vector2d> pts(cur.begin(), cur.end());
  basalt::detectKeypoints(lvl0, kd, grid, npc, pts);
  std::vector<double> flat(cur.size() * 2 + 1);
  for (size_t i = 0; i < cur.size(); i++) { flat[2 * i] = cur[i][0]; flat[2 * i + 1] = cur[i][1]; }
  std::vector<double> out(2 * (size_t)(lvl0.w * lvl0.h) / 4 + 16);
  int n = bs_detect_keypoints(lvl0.ptr, pitch, (int)lvl0.w, (int)lvl0.h, grid, npc, flat.data(), (int)cur.size(), out.data(), (int)out.size() / 2);
  bool ok = n == (int)kd.corners.size();
  if (ok)
    for (int i = 0; i < n; i++) ok &= memcmp(&kd.corners[i][0], &out[2 * i], 8) == 0 && memcmp(&kd.corners[i][1], &out[2 * i + 1], 8) == 0;
  if (ncorn) *ncorn += kd.corners.size();
  chk(ok, "detect", ctx + " n_ref=" + std::to_string(kd.corners.size()) + " n_c=" + std::to_string(n));
  return ok;
}

static void test_detect(const std::string& seq, int step) {
  auto names = read_names(seq);
  Rng r(9);
  uint64_t ncorn = 0, nframes = 0;
  std::uniform_real_distribution<double> ud(0, 1);
  for (size_t f = 0; f < names.size(); f += step)
    for (int cam = 0; cam < 2; cam++) {
      std::string p = seq + "/mav0/cam" + std::to_string(cam) + "/data/" + names[f];
      basalt::ManagedImage<uint16_t> ref;
      bool ok;
      real_u16(p, ref, ok);
      if (!ok) { chk(false, "detect_load", p); continue; }
      basalt::ManagedImagePyr<uint16_t> pr;
      pr.setFromImage(ref, 3);
      const auto l0 = pr.lvl(0);
      int pitch = (int)pr.mipmap().w;
      // tracked-point sets: none, random fraction of the cells occupied, dense, plus float-valued positions near cell borders
      for (int variant = 0; variant < 5; variant++) {
        std::vector<Eigen::Vector2d> cur;
        int np = variant == 0 ? 0 : variant == 1 ? (int)(r() % 60) : variant == 2 ? (int)(r() % 200) : variant == 3 ? 100 + (int)(r() % 40) : (int)(r() % 25);
        for (int i = 0; i < np; i++) {
          double x = ud(r) * (ref.w + 20) - 10, y = ud(r) * (ref.h + 20) - 10;
          if (variant == 4) { x = (double)(int)(x / 50) * 50 + (r() % 3) - 1 + (float)ud(r) * 0.001f; y = (double)(int)(y / 50) * 50 + (r() % 3) - 1; }
          cur.emplace_back((double)(float)x, (double)(float)y);
        }
        detect_same(l0, pitch, 50, 1, cur, p + " v" + std::to_string(variant), &ncorn);
      }
      detect_same(l0, pitch, 50, 1, {}, p + " grid50", &ncorn);
      if ((f / step) % 5 == 0) {
        detect_same(l0, pitch, 32, 1, {}, p + " grid32", &ncorn);
        detect_same(l0, pitch, 64, 3, {}, p + " grid64 npc3", &ncorn);
        detect_same(l0, pitch, 50, 4, {}, p + " grid50 npc4", &ncorn);
        detect_same(l0, pitch, 50, 1, {{0.0, 0.0}, {751.5, 479.5}, {-3.0, 20.0}, {1e30, 5.0}, {std::nan(""), 1.0}}, p + " edgepts", &ncorn);
      }
      nframes++;
    }
  // synthetic images (other sizes, strides)
  for (int it = 0; it < 400; it++) {
    int w = 60 + (int)(r() % 400), h = 60 + (int)(r() % 300);
    int kind = (int)(r() % 7);
    std::vector<uint8_t> b;
    make_image(r, kind, w, h, b);
    basalt::ManagedImage<uint16_t> im(w, h);
    for (size_t i = 0; i < im.size(); i++) im.ptr[i] = (uint16_t)(b[i] << 8) | (uint16_t)(r() & 0xff);  // low bytes must be ignored
    basalt::ManagedImagePyr<uint16_t> pr;
    pr.setFromImage(im, 2);
    int grid = (it % 4 == 0) ? 32 : (it % 4 == 1) ? 40 : 50;
    int npc = 1 + (int)(r() % 3);
    std::vector<Eigen::Vector2d> cur;
    for (int i = 0, np = (int)(r() % 30); i < np; i++) cur.emplace_back((double)(float)(ud(r) * w), (double)(float)(ud(r) * h));
    detect_same(pr.lvl(0), (int)pr.mipmap().w, grid, npc, cur, "synth kind" + std::to_string(kind) + " " + std::to_string(w) + "x" + std::to_string(h), &ncorn);
  }
  printf("  real frames (both cams): %llu, corners compared: %llu\n", (unsigned long long)nframes, (unsigned long long)ncorn);
}

// ---------------------------------------------------------------- replay
struct Cur { const uint8_t* p; uint64_t n, i; bool err; };
template <class T> static T rd(Cur& c) { T v{}; if (c.i + sizeof(T) > c.n) { c.err = true; return v; } memcpy(&v, c.p + c.i, sizeof(T)); c.i += sizeof(T); return v; }

static uint64_t fnv_u16(const uint16_t* p, size_t n, uint64_t h) {
  const uint8_t* c = (const uint8_t*)p;
  for (size_t i = 0; i < n * 2; i++) { h ^= c[i]; h *= 1099511628211ull; }
  return h;
}

static void test_replay(const std::string& flow, const std::string& seq) {
  FILE* f = fopen(flow.c_str(), "rb");
  if (!f) { fprintf(stderr, "cannot open %s\n", flow.c_str()); exit(2); }
  std::vector<uint8_t> buf;
  { uint8_t tmp[1 << 16]; size_t k; while ((k = fread(tmp, 1, sizeof tmp, f)) > 0) buf.insert(buf.end(), tmp, tmp + k); }
  fclose(f);
  auto names = read_names(seq);
  std::map<int64_t, std::string> by_t;
  {
    FILE* g = fopen((seq + "/mav0/cam0/data.csv").c_str(), "r");
    char line[512];
    while (fgets(line, sizeof line, g)) { if (line[0] == '#') continue; by_t[strtoll(line, nullptr, 10)] = ""; }
    fclose(g);
    size_t i = 0;
    for (auto& kv : by_t) kv.second = names[i++];
  }
  Cur c{buf.data(), buf.size(), 0, false};
  uint64_t frames = 0, hash_ok = 0, new_total = 0, new_found = 0, subseq_ok = 0, sub_bad = 0, extra_cells = 0, exact_frames = 0;
  std::set<uint64_t> prev_ids;
  std::map<uint64_t, std::pair<float, float>> prev_pos;
  while (c.i < c.n && !c.err) {
    uint32_t tag = rd<uint32_t>(c);
    uint64_t len = rd<uint64_t>(c);
    if (c.err || len > c.n - c.i) break;
    Cur pl{c.p + c.i, len, 0, false};
    c.i += len;
    if (tag != 1) continue;
    int64_t t = rd<int64_t>(pl);
    rd<uint64_t>(pl);
    uint32_t ncam = rd<uint32_t>(pl);
    struct Im { uint32_t w, h; uint64_t hash; } ims[4];
    for (uint32_t k = 0; k < ncam; k++) { ims[k].w = rd<uint32_t>(pl); ims[k].h = rd<uint32_t>(pl); ims[k].hash = rd<uint64_t>(pl); }
    struct Ob { uint64_t id; float m[6]; };
    std::vector<Ob> obs[4];
    for (uint32_t k = 0; k < ncam; k++) {
      uint32_t n = rd<uint32_t>(pl);
      for (uint32_t j = 0; j < n; j++) { Ob o; o.id = rd<uint64_t>(pl); for (int q = 0; q < 6; q++) o.m[q] = rd<float>(pl); obs[k].push_back(o); }
    }
    if (pl.err) { chk(false, "replay_parse"); break; }
    frames++;
    // images: loader + hash for every camera
    uint16_t* im0 = nullptr;
    int w0 = 0, h0 = 0;
    bool all = true;
    for (uint32_t k = 0; k < ncam; k++) {
      uint16_t* im = nullptr;
      int w, h;
      std::string p = seq + "/mav0/cam" + std::to_string(k) + "/data/" + by_t[t];
      int rc = bs_image_load_euroc(p.c_str(), &im, &w, &h);
      bool ok = rc == BS_IMG_OK && (uint32_t)w == ims[k].w && (uint32_t)h == ims[k].h && fnv_u16(im, (size_t)w * h, 1469598103934665603ull) == ims[k].hash;
      all &= ok;
      if (k == 0 && ok) { im0 = im; w0 = w; h0 = h; } else free(im);
    }
    chk(all, "replay_hash", "t=" + std::to_string(t));
    hash_ok += all;
    // keypoints: new = cam0 ids absent from the previous frame's cam0 observations
    if (im0) {
      bs_pyr pyr = {};
      bs_pyr_set(&pyr, im0, w0, h0, 3);
      int lw, lh;
      const uint16_t* l0 = bs_pyr_lvl(&pyr, 0, &lw, &lh);
      std::vector<double> cur;
      std::vector<std::pair<float, float>> news;
      for (auto& o : obs[0]) {
        if (prev_ids.count(o.id)) { cur.push_back((double)o.m[2]); cur.push_back((double)o.m[5]); }
        else news.push_back({o.m[2], o.m[5]});
      }
      if (frames > 1) {
        std::vector<double> out(2 * (size_t)w0 * h0 / 4);
        int n = bs_detect_keypoints(l0, pyr.pitch, w0, h0, 50, 1, cur.data(), (int)cur.size() / 2, out.data(), (int)out.size() / 2);
        // the dump holds tracked points AFTER filterPoints, so cells whose tracked point was filtered later are free here, and new points
        // that filterPoints dropped are missing: the dumped new set must be an ordered subsequence of the C detections.
        size_t pos = 0;
        bool sub = n >= 0;
        for (auto& nw : news) {
          while (pos < (size_t)n && !((float)out[2 * pos] == nw.first && (float)out[2 * pos + 1] == nw.second)) pos++;
          if (pos == (size_t)n) { sub = false; break; }
          pos++;
        }
        new_total += news.size();
        if (sub) { new_found += news.size(); subseq_ok++; }
        else sub_bad++;
        if (n >= 0 && (size_t)n == news.size()) exact_frames++;
        if (n >= 0) extra_cells += (size_t)n - (sub ? news.size() : 0);
        chk(sub, "replay_new_subsequence", "t=" + std::to_string(t) + " dumped_new=" + std::to_string(news.size()) + " c_detected=" + std::to_string(n));
      }
      bs_pyr_free(&pyr);
      free(im0);
    }
    prev_ids.clear();
    for (auto& o : obs[0]) prev_ids.insert(o.id);
  }
  printf("  frames %llu, image hash equal %llu, frames with new keypoints a subsequence of the C detections %llu (bad %llu)\n", (unsigned long long)frames, (unsigned long long)hash_ok, (unsigned long long)subseq_ok, (unsigned long long)sub_bad);
  printf("  dumped new keypoints %llu all found in C order %llu; frames where the C detection count equals the dumped new count %llu; C detections not in the dump (filtered later / cells freed by filtering) %llu\n",
         (unsigned long long)new_total, (unsigned long long)new_found, (unsigned long long)exact_frames, (unsigned long long)extra_cells);
}

// ------------------------------------------------------------ sensitivity
static void test_sens() {
  // a wrong FAST (threshold - 1 in the C call) and a stable-sort variant must differ from the real ones on enough inputs
  Rng r(3);
  uint64_t diff_fast = 0, diff_sort = 0, n = 0;
  for (int it = 0; it < 400; it++) {
    std::vector<uint8_t> im;
    make_image(r, it % 7, 50, 50, im);
    cv::Mat m(50, 50, CV_8U, im.data());
    std::vector<cv::KeyPoint> kp;
    cv::FAST(m, kp, 10, true);
    std::vector<bs_fast_kp> out(2501);
    int k = bs_fast9_16(m.data, 50, 50, 50, 11, 1, out.data(), 2501);
    bool same = k == (int)kp.size();
    for (int i = 0; same && i < k; i++) same &= kp[i].pt.x == out[i].x && kp[i].pt.y == out[i].y && kp[i].response == out[i].response;
    diff_fast += !same;
    std::vector<bs_fast_kp> s(kp.size());
    for (size_t i = 0; i < kp.size(); i++) s[i] = {kp[i].pt.x, kp[i].pt.y, kp[i].response};
    std::vector<cv::KeyPoint> a = kp;
    std::sort(a.begin(), a.end(), [](const cv::KeyPoint& x, const cv::KeyPoint& y) { return x.response > y.response; });
    std::stable_sort(s.begin(), s.end(), [](const bs_fast_kp& x, const bs_fast_kp& y) { return x.response > y.response; });
    bool ss = true;
    for (size_t i = 0; i < a.size(); i++) ss &= a[i].pt.x == s[i].x && a[i].pt.y == s[i].y;
    diff_sort += !ss;
    n++;
  }
  printf("sensitivity: threshold off by one differs in %llu/%llu images; std::stable_sort tie order differs from std::sort in %llu/%llu cells\n",
         (unsigned long long)diff_fast, (unsigned long long)n, (unsigned long long)diff_sort, (unsigned long long)n);
  g_bad = (diff_fast > 0 && diff_sort > 0) ? 0 : 1;
  g_total = n;
  printf("bs_image sens: %llu/%llu\n", (unsigned long long)(diff_fast > 0 && diff_sort > 0 ? 0 : 1), (unsigned long long)n);
  exit(g_bad ? 1 : 0);
}

int main(int argc, char** argv) {
  cv::setNumThreads(0);
  std::string mode = argc > 1 ? argv[1] : "";
  if (mode == "sort") { test_sort(argc > 2 ? strtoull(argv[2], 0, 10) : 1); return finish("bs_image sort"); }
  if (mode == "fast") { test_fast(argc > 2 ? strtoull(argv[2], 0, 10) : 1); return finish("bs_image fast"); }
  if (mode == "image" && argc > 2) { test_image(argv[2], argc > 3 ? atoi(argv[3]) : 10); return finish("bs_image image"); }
  if (mode == "detect" && argc > 2) { test_detect(argv[2], argc > 3 ? atoi(argv[3]) : 10); return finish("bs_image detect"); }
  if (mode == "replay" && argc > 3) { test_replay(argv[2], argv[3]); return finish("bs_image replay"); }
  if (mode == "sens") test_sens();
  fprintf(stderr, "usage: see header\n");
  return 2;
}
