// basalt_port module M5 oracle: C port (bs_patch.c / bs_flow.c) vs the real basalt FrameToFrameOpticalFlow<float, Pattern51> / OpticalFlowPatch /
// Image::interp / Sophus SE2 / Eigen LDLT, tolerance 0 (memcmp of every float).
//   bs_flow_test eigen  [seed N]             Eigen sub-models: J^T J redux, 3x3 LDLT solve, Hinv*J^T, -HJ*res, epipolar product (random + degenerate)
//   bs_flow_test interp [seed N]             Image::interp / interpGrad / InBounds on real + synthetic images, borders included
//   bs_flow_test patch  <seqdir> [seed N]    OpticalFlowPatch::setFromImage / residual, SE2::exp, Transform *=, trackPointAtLevel, trackPoint
//   bs_flow_test calib  <calib.json> <config.json>   C json readers vs cereal Calibration / VioConfig; essential matrix
//   bs_flow_test flow   <seqdir> <start> <count> [calib config]   real FrameToFrameOpticalFlow in lockstep with the C flow on real frames
//   bs_flow_test synth  [seed N] [nseq]      same on synthetic stereo sequences (textures, black / saturated blocks, large disparities)
//   bs_flow_test replay <flow.bin> <seqdir> [calib config] [maxframes]  C flow over all frames vs the M0 FLOW dump (ids + 6 floats, bit exact)
// Last stdout line: "<label>: <bad>/<total>"; exit 0 iff bad == 0.
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <basalt/calibration/calibration.hpp>
#include <basalt/image/image_pyr.h>
#include <basalt/io/dataset_io.h>
#include <basalt/optical_flow/frame_to_frame_optical_flow.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/keypoints.h>
#include <basalt/utils/vio_config.h>
#include <tbb/global_control.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

extern "C" {
#include "bs_flow.h"
#include "bs_image.h"
#include "bs_patch.h"
}

using namespace basalt;
typedef OpticalFlowPatch<float, Pattern51<float>> PatchT;
typedef FrameToFrameOpticalFlow<float, Pattern51> FF;
typedef std::mt19937_64 Rng;

static uint64_t g_total = 0, g_bad = 0;
static std::map<std::string, std::pair<uint64_t, uint64_t>> g_cat;   // statement -> (bad, total)
static void chk(bool ok, const char* what, const std::string& ctx = "") {
  g_total++;
  g_cat[what].second++;
  if (!ok) {
    g_bad++;
    g_cat[what].first++;
    if (g_bad <= 25) fprintf(stderr, "  MISMATCH %s %s\n", what, ctx.c_str());
  }
}
static int finish(const char* label) {
  for (auto& kv : g_cat) printf("  %-24s %llu/%llu\n", kv.first.c_str(), (unsigned long long)kv.second.first, (unsigned long long)kv.second.second);
  printf("%s: %llu/%llu\n", label, (unsigned long long)g_bad, (unsigned long long)g_total);
  return g_bad == 0 ? 0 : 1;
}
static bool same(const float* a, const float* b, size_t n) { return memcmp(a, b, n * sizeof(float)) == 0; }
static double ur(Rng& r, double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(r); }
static int ui(Rng& r, int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(r); }

// ------------------------------------------------------------------ shared inputs
static std::string g_calib = "/home/nybo/github/pose-validation/external/vio/basalt_src/data/euroc_ds_calib.json";
static std::string g_config = "/home/nybo/github/pose-validation/external/vio/basalt_src/data/euroc_config.json";

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

static bool load_u16(const std::string& path, ManagedImage<uint16_t>& out) {
  cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (img.empty() || img.type() != CV_8UC1) return false;
  out.Reinitialise(img.cols, img.rows);
  for (int y = 0; y < img.rows; y++)
    for (int x = 0; x < img.cols; x++) out(x, y) = (uint16_t)(img.at<uint8_t>(y, x) << 8);
  return true;
}

static void load_calib(Calibration<double>& calib) {
  std::ifstream is(g_calib, std::ios::binary);
  if (!is.is_open()) { fprintf(stderr, "cannot open %s\n", g_calib.c_str()); exit(2); }
  cereal::JSONInputArchive ar(is);
  ar(calib);
}

static void c_calib_from(const Calibration<double>& calib, bs_flow_calib& cc) {
  cc.ncam = 2;
  for (int i = 0; i < 2; i++) {
    const auto p = calib.intrinsics[i].getParam();
    for (int k = 0; k < 6; k++) cc.intr[i][k] = p[k];
    const auto& T = calib.T_i_c[i];
    for (int k = 0; k < 3; k++) cc.T_i_c[i][k] = T.translation()[k];
    for (int k = 0; k < 4; k++) cc.T_i_c[i][3 + k] = T.so3().data()[k];
  }
}

static void c_config_from(const VioConfig& v, bs_flow_config& c) {
  c.pattern = v.optical_flow_pattern;
  c.levels = v.optical_flow_levels;
  c.max_iterations = v.optical_flow_max_iterations;
  c.grid_size = v.optical_flow_detection_grid_size;
  c.skip_frames = v.optical_flow_skip_frames;
  c.max_recovered_dist2 = v.optical_flow_max_recovered_dist2;
  c.epipolar_error = v.optical_flow_epipolar_error;
}

static bs_imgv view_of(const Image<const uint16_t>& im) { return bs_imgv{im.ptr, (int)(im.pitch / 2), (int)im.w, (int)im.h}; }

// ------------------------------------------------------------------ image pool (real + synthetic)
struct PoolImg {
  ManagedImage<uint16_t> im;
  ManagedImagePyr<uint16_t> pyr;
  std::string name;
};

static void synth_image(Rng& r, int kind, ManagedImage<uint16_t>& im) {
  const int w = 752, h = 480;
  im.Reinitialise(w, h);
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      uint16_t hi = 0;
      switch (kind) {
        case 0: hi = 0; break;                                         // black
        case 1: hi = 100; break;                                       // flat
        case 2: hi = 255; break;                                       // saturated
        case 3: hi = (uint8_t)(r() & 255); break;                      // noise
        case 4: hi = (uint8_t)((x + 2 * y) & 255); break;              // ramp
        case 5: hi = (((x / 6) + (y / 6)) & 1) ? 230 : 25; break;      // checker
        case 6: hi = (x < w / 2) ? 3 : 4; break;                       // nearly black with an edge (tiny sums)
        default: hi = (uint8_t)(128 + 100 * sin(x * 0.07) * cos(y * 0.05)); break;
      }
      im(x, y) = (uint16_t)((hi << 8) | (kind == 3 ? 0 : 0));
    }
}

static void build_pool(const std::string& seq, Rng& r, std::vector<PoolImg>& pool, int nreal) {
  auto names = read_names(seq);
  for (int k = 0; k < nreal; k++) {
    size_t f = (size_t)(k * (names.size() - 2) / std::max(1, nreal - 1));
    for (int d = 0; d < 2; d++) {                      // frame f and f+1 (consecutive, for tracking), cam0
      PoolImg p;
      std::string path = seq + "/mav0/cam" + std::to_string(k % 2) + "/data/" + names[f + d];
      if (!load_u16(path, p.im)) { fprintf(stderr, "cannot load %s\n", path.c_str()); exit(2); }
      p.pyr.setFromImage(p.im, 3);
      p.name = path;
      pool.push_back(std::move(p));
    }
  }
  for (int kind = 0; kind < 8; kind++) {
    PoolImg p;
    synth_image(r, kind, p.im);
    p.pyr.setFromImage(p.im, 3);
    p.name = "synth" + std::to_string(kind);
    pool.push_back(std::move(p));
  }
}

// ------------------------------------------------------------------ eigen sub-models
static void test_eigen(int seed) {
  Rng r(1000 + seed);
  uint64_t perm_cov[3][3] = {};
  const int N = 60000;
  // (1) J^T J (3x52 * 52x3, lazy coefficient product), Hinv = H.ldlt().solveInPlace(I), HJ = Hinv * J^T, inc = -HJ * res
  for (int it = 0; it < N; it++) {
    Eigen::Matrix<float, 52, 3> J;
    int kind = it % 8;
    double scale = pow(10.0, ur(r, -4, 3));
    for (int i = 0; i < 52; i++)
      for (int c = 0; c < 3; c++) {
        double v = ur(r, -1, 1) * scale;
        if (kind == 1 && c == 1) v = 0;                             // zero column (singular H)
        if (kind == 2) v = ur(r, -1, 1) * scale * (c == 0 ? 1 : 1e-3);   // badly scaled
        if (kind == 3 && c == 2) v = J(i, 0) * 0.5 + ur(r, -1e-4, 1e-4) * scale;   // nearly dependent
        if (kind == 4 && i % 3 != 0) v = 0;                         // sparse
        if (kind == 5 && c == 2) v = J(i, 1);                       // exactly dependent
        if (kind == 6) v = (double)(int)(v * 3);                    // small integers (ties in the pivot search)
        if (kind == 7) v = (i < 4 ? v : 0);                         // rank deficient
        J(i, c) = (float)v;
      }
    Eigen::Matrix3f H = J.transpose() * J;
    float Hc[9];
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) Hc[a + 3 * b] = bs_patch_dbg_dot52(&J(0, a), &J(0, b));
    chk(same(H.data(), Hc, 9), "JtJ", "kind " + std::to_string(kind));
    Eigen::Matrix3f Hinv;
    Hinv.setIdentity();
    Eigen::LDLT<Eigen::Matrix3f> ld = H.ldlt();
    ld.solveInPlace(Hinv);
    float Hic[9];
    bs_patch_dbg_ldlt3_inverse(H.data(), Hic);
    {
      bool eq = same(Hinv.data(), Hic, 9);
      // a NaN produced by the Eigen code and by the C code may differ in the payload only: count it separately below
      chk(eq, "ldlt3", "kind " + std::to_string(kind));
    }
    auto P = ld.transpositionsP().indices();
    perm_cov[P[0]][P[1]]++;
    Eigen::Matrix<float, 3, 52> HJ = Hinv * J.transpose();
    Eigen::Matrix<float, 3, 52> HJc;
    for (int i = 0; i < 52; i++)
      for (int a = 0; a < 3; a++)
        HJc(a, i) = Hic[a] * J(i, 0) + (Hic[a + 3] * J(i, 1) + Hic[a + 6] * J(i, 2));
    chk(same(HJ.data(), HJc.data(), 156), "HinvJt");
    Eigen::Matrix<float, 52, 1> res;
    for (int i = 0; i < 52; i++) res[i] = (float)(ur(r, -1, 1) * pow(10.0, ur(r, -3, 1)));
    Eigen::Vector3f inc = -HJ * res;
    float incc[3];
    bs_patch_dbg_inc(HJ.data(), res.data(), incc);
    chk(same(inc.data(), incc, 3), "inc");
  }
  printf("  LDLT transposition patterns (idx0,idx1): ");
  for (int a = 0; a < 3; a++)
    for (int b = 0; b < 3; b++)
      if (perm_cov[a][b]) printf("(%d,%d)=%llu ", a, b, (unsigned long long)perm_cov[a][b]);
  printf("\n");
  // (2) epipolar product p0^T E p1 (Vector4f^T * Matrix4f * Vector4f)
  for (int it = 0; it < 200000; it++) {
    Eigen::Matrix4f E;
    Eigen::Vector4f a, b;
    double sc = pow(10.0, ur(r, -3, 2));
    for (int i = 0; i < 16; i++) E(i % 4, i / 4) = (float)(ur(r, -1, 1) * sc * (it % 3 == 0 && i % 4 == 3 ? 0 : 1));
    for (int i = 0; i < 4; i++) { a[i] = (float)ur(r, -1, 1); b[i] = (float)ur(r, -1, 1); }
    a[3] = it % 2 ? 0.f : a[3];
    b[3] = it % 2 ? 0.f : b[3];
    float real = a.transpose() * E * b;
    float c = bs_flow_epipolar(E.data(), a.data(), b.data());
    chk(memcmp(&real, &c, 4) == 0, "epipolar");
  }
  // (3) Vector2f squaredNorm and the 2x2 * 2x2 product of trackPoint
  for (int it = 0; it < 100000; it++) {
    Eigen::Vector2f v((float)ur(r, -3, 3), (float)ur(r, -3, 3));
    float real = v.squaredNorm(), c = v[0] * v[0] + v[1] * v[1];
    chk(memcmp(&real, &c, 4) == 0, "sqnorm2");
    Eigen::Matrix2f A, B;
    for (int i = 0; i < 4; i++) { A.data()[i] = (float)ur(r, -2, 2); B.data()[i] = (float)ur(r, -2, 2); }
    Eigen::Matrix2f P = A * B;
    float Pc[4] = {A(0, 0) * B(0, 0) + A(0, 1) * B(1, 0), A(1, 0) * B(0, 0) + A(1, 1) * B(1, 0), A(0, 0) * B(0, 1) + A(0, 1) * B(1, 1),
                   A(1, 0) * B(0, 1) + A(1, 1) * B(1, 1)};
    chk(same(P.data(), Pc, 4), "mul22");
  }
}

// ------------------------------------------------------------------ interp
static void test_interp(int seed) {
  Rng r(2000 + seed);
  std::vector<PoolImg> pool;
  for (int kind = 0; kind < 8; kind++) {
    PoolImg p;
    synth_image(r, kind, p.im);
    if (kind == 3) for (size_t i = 0; i < p.im.size(); i++) p.im.ptr[i] = (uint16_t)(r() & 0xffff);   // full 16-bit noise
    p.pyr.setFromImage(p.im, 3);
    pool.push_back(std::move(p));
  }
  uint64_t ngrad = 0;
  for (int it = 0; it < 400000; it++) {
    PoolImg& p = pool[it % pool.size()];
    int l = ui(r, 0, 3);
    auto img = p.pyr.lvl(l);
    bs_imgv v = view_of(img);
    float x, y;
    int mode = ui(r, 0, 3);
    if (mode == 0) { x = (float)ur(r, -3, img.w + 3); y = (float)ur(r, -3, img.h + 3); }
    else if (mode == 1) { x = (float)ur(r, 1, img.w - 3); y = (float)ur(r, 1, img.h - 3); }
    else if (mode == 2) { x = (float)(ui(r, 0, img.w) + (ui(r, 0, 1) ? 0.0 : 0.999999)); y = (float)(ui(r, 0, img.h) + (ui(r, 0, 1) ? 0.0 : 0.5)); }
    else { x = (float)ur(r, 0, 3); y = (float)ur(r, img.h - 4, img.h + 1); }
    for (float border : {0.f, 1.f, 2.f}) {
      Eigen::Vector2f pe(x, y);
      chk(img.InBounds(pe, border) == (bs_img_inbounds(&v, x, y, border) != 0), "inbounds", "border " + std::to_string(border));
    }
    Eigen::Vector2f pe(x, y);
    if (img.InBounds(pe, 0.f)) {
      float a = img.interp<float>(pe), b = bs_img_interp(&v, x, y);
      chk(memcmp(&a, &b, 4) == 0, "interp");
    }
    if (img.InBounds(pe, 1.f)) {
      Eigen::Vector3f a = img.interpGrad<float>(pe);
      float b[3];
      bs_img_interp_grad(&v, x, y, b);
      chk(same(a.data(), b, 3), "interpGrad");
      ngrad++;
    }
  }
  printf("  interpGrad evaluations: %llu\n", (unsigned long long)ngrad);
}

// ------------------------------------------------------------------ calib / config
static int test_calib(const std::string& cpath, const std::string& vpath) {
  Calibration<double> calib;
  g_calib = cpath;
  load_calib(calib);
  VioConfig vc;
  vc.load(vpath);
  bs_flow_calib cc, cr;
  bs_flow_config kc, kr;
  memset(&cc, 0, sizeof cc); memset(&cr, 0, sizeof cr); memset(&kc, 0, sizeof kc); memset(&kr, 0, sizeof kr);
  c_calib_from(calib, cr);
  c_config_from(vc, kr);
  chk(bs_flow_calib_load(cpath.c_str(), &cc) == 0, "calib_load");
  chk(memcmp(&cc, &cr, sizeof cc) == 0, "calib_bits");
  for (int i = 0; i < 2; i++) {
    for (int k = 0; k < 6; k++) if (cc.intr[i][k] != cr.intr[i][k]) printf("  intr[%d][%d] C %.17g real %.17g\n", i, k, cc.intr[i][k], cr.intr[i][k]);
    for (int k = 0; k < 7; k++) if (cc.T_i_c[i][k] != cr.T_i_c[i][k]) printf("  T_i_c[%d][%d] C %.17g real %.17g\n", i, k, cc.T_i_c[i][k], cr.T_i_c[i][k]);
  }
  bs_flow_config_default(&kc);
  chk(bs_flow_config_load(vpath.c_str(), &kc) == 0, "config_load");
  chk(memcmp(&kc, &kr, sizeof kc) == 0, "config_fields");
  Eigen::Matrix4d Ed;
  Sophus::SE3d T_i_j = calib.T_i_c[0].inverse() * calib.T_i_c[1];
  computeEssential(T_i_j, Ed);
  Eigen::Matrix4f E = Ed.cast<float>();
  float Ec[16];
  bs_flow_essential(&cr, Ec);
  chk(same(E.data(), Ec, 16), "essential");
  printf("  intrinsics cam0 fx=%.17g cam1 fx=%.17g, dist2 %.9g epipolar %.9g\n", cc.intr[0][0], cc.intr[1][0], (double)kc.max_recovered_dist2, (double)kc.epipolar_error);
  return 0;
}

// ------------------------------------------------------------------ patch level
static void test_patch(const std::string& seq, int seed) {
  Rng r(3000 + seed);
  std::vector<PoolImg> pool;
  build_pool(seq, r, pool, 24);
  VioConfig vc;
  vc.load(g_config);
  Calibration<double> calib;
  load_calib(calib);
  FF ff(vc, calib);
  bs_flow_config kc;
  bs_flow_calib cc;
  c_config_from(vc, kc);
  c_calib_from(calib, cc);
  bs_flow* cf = bs_flow_new(&kc, &cc);
  float pat[2 * BS_PAT];
  bs_pattern_init(pat, 51);
  chk(same(PatchT::pattern2.data(), pat, 2 * BS_PAT), "pattern51");
  for (int p : {50, 52}) {
    float a[2 * BS_PAT];
    bs_pattern_init(a, p);
    const Eigen::Matrix<float, 2, 52> ref = (p == 52 ? Pattern52<float>::pattern2 : Pattern50<float>::pattern2);
    chk(same(ref.data(), a, 104), "pattern52_50", std::to_string(p));
  }
  uint64_t n_valid = 0, n_inval = 0, n_res_ok = 0, n_res_bad = 0, n_tl_ok = 0, n_tl_bad = 0, n_tp_ok = 0, n_tp_bad = 0, n_exp_small = 0;
  const int NP = 60000;
  for (int it = 0; it < NP; it++) {
    PoolImg& A = pool[(size_t)ui(r, 0, (int)pool.size() - 1)];
    PoolImg* Bp = &A;
    {   // second image: the next frame of the same camera for real pairs (even index -> +1), else any
      size_t ia = &A - pool.data();
      if (ia < 48 && ia % 2 == 0) Bp = &pool[ia + 1];
      else Bp = &pool[(size_t)ui(r, 0, (int)pool.size() - 1)];
    }
    PoolImg& B = *Bp;
    int l = ui(r, 0, 3);
    auto la = A.pyr.lvl(l), lb = B.pyr.lvl(l);
    bs_imgv va = view_of(la), vb = view_of(lb);
    float px, py;
    if (ui(r, 0, 9) == 0) { px = (float)ur(r, -3, la.w + 3); py = (float)ur(r, -3, la.h + 3); }
    else { px = (float)ur(r, 6, la.w - 7); py = (float)ur(r, 6, la.h - 7); }
    // ---- setFromImage
    PatchT rp(la, Eigen::Vector2f(px, py));
    bs_patch cp;
    float pos[2] = {px, py};
    bs_patch_set(&cp, &va, pat, pos);
    bool same_patch = rp.valid == (cp.valid != 0) && memcmp(&rp.mean, &cp.mean, 4) == 0 && same(rp.data.data(), cp.data, 52) &&
                      same(rp.H_se2_inv_J_se2_T.data(), cp.HJ, 156);
    chk(same_patch, "setFromImage", A.name + " l" + std::to_string(l) + " pos " + std::to_string(px) + "," + std::to_string(py));
    (rp.valid ? n_valid : n_inval)++;
    // ---- warp
    float sl = (float)pow(10.0, ur(r, -3, -0.5));
    Eigen::AffineCompact2f T;
    T.setIdentity();
    if (ui(r, 0, 3)) {
      T.linear() << 1.f + (float)ur(r, -sl, sl), (float)ur(r, -sl, sl), (float)ur(r, -sl, sl), 1.f + (float)ur(r, -sl, sl);
    }
    float tsc = (float)pow(10.0, ur(r, -2, 0.7));
    T.translation() << px + (float)ur(r, -tsc, tsc), py + (float)ur(r, -tsc, tsc);
    // ---- residual (transformed pattern as in trackPointAtLevel)
    {
      PatchT::Matrix2P tp = T.linear().matrix() * PatchT::pattern2;
      tp.colwise() += T.translation();
      PatchT::VectorP rres, cres;
      bool rb = rp.residual(lb, tp, rres);
      bool cb = bs_patch_residual(&cp, &vb, tp.data(), cres.data()) != 0;
      chk(rb == cb && same(rres.data(), cres.data(), 52), "residual");
      (rb ? n_res_ok : n_res_bad)++;
    }
    // ---- trackPointAtLevel
    if (rp.valid) {
      Eigen::AffineCompact2f tr = T;
      float trc[6];
      memcpy(trc, tr.matrix().data(), sizeof trc);
      bool rb = ff.trackPointAtLevel(lb, rp, tr);
      bool cb = bs_track_point_at_level(&vb, &cp, pat, vc.optical_flow_max_iterations, trc) != 0;
      chk(rb == cb && same(tr.matrix().data(), trc, 6), "trackPointAtLevel");
      (rb ? n_tl_ok : n_tl_bad)++;
    }
    // ---- SE2::exp + Transform *=
    if (it % 4 == 0) {
      Eigen::Vector3f inc;
      double regime = pow(10.0, ur(r, -9, 0.3));
      inc << (float)ur(r, -3, 3), (float)ur(r, -3, 3), (float)(ur(r, -1, 1) * regime);
      if (it % 8 == 0) inc[2] = (float)(ur(r, -1.2e-5, 1.2e-5));    // around the Taylor switch
      if (it % 16 == 0) inc[2] = it % 32 == 0 ? 0.f : -0.f;
      Eigen::Matrix3f M = Sophus::SE2<float>::exp(inc).matrix();
      float Mc[9];
      bs_se2_exp_matrix(inc.data(), Mc);
      chk(same(M.data(), Mc, 9), "SE2exp");
      if (std::fabs(inc[2]) < 1e-5f) n_exp_small++;
      Eigen::AffineCompact2f t2 = T;
      float t2c[6];
      memcpy(t2c, t2.matrix().data(), sizeof t2c);
      t2 *= M;
      bs_affine_mul_assign(t2c, Mc);
      chk(same(t2.matrix().data(), t2c, 6), "Transform*=");
    }
  }
  // ---- trackPoint (4 levels, both pyramids), on the real consecutive pairs
  for (int it = 0; it < 6000; it++) {
    size_t k = (size_t)ui(r, 0, 23) * 2;
    PoolImg &A = pool[k], &B = pool[k + 1];
    bs_pyr pa = {}, pb = {};
    bs_pyr_set(&pa, A.im.ptr, (int)A.im.w, (int)A.im.h, 3);
    bs_pyr_set(&pb, B.im.ptr, (int)B.im.w, (int)B.im.h, 3);
    Eigen::AffineCompact2f t1, t2, t2c_real;
    t1.setIdentity();
    if (ui(r, 0, 1)) t1.linear() << 1.f + (float)ur(r, -0.1, 0.1), (float)ur(r, -0.1, 0.1), (float)ur(r, -0.1, 0.1), 1.f + (float)ur(r, -0.1, 0.1);
    float x = ui(r, 0, 20) == 0 ? (float)ur(r, -5, 757) : (float)ur(r, 20, 732), y = ui(r, 0, 20) == 0 ? (float)ur(r, -5, 485) : (float)ur(r, 20, 460);
    t1.translation() << x, y;
    t2 = t1;
    bool rb = ff.trackPoint(A.pyr, B.pyr, t1, t2);
    float t2c[6];
    memcpy(t2c, t1.matrix().data(), sizeof t2c);
    bool cb = bs_flow_track_point(cf, &pa, &pb, t1.matrix().data(), t2c) != 0;
    chk(rb == cb && same(t2.matrix().data(), t2c, 6), "trackPoint");
    (rb ? n_tp_ok : n_tp_bad)++;
    bs_pyr_free(&pa);
    bs_pyr_free(&pb);
  }
  printf("  patches valid/invalid %llu/%llu, residual ok/bad %llu/%llu, trackPointAtLevel ok/bad %llu/%llu, trackPoint ok/bad %llu/%llu, SE2 exp small branch %llu\n",
         (unsigned long long)n_valid, (unsigned long long)n_inval, (unsigned long long)n_res_ok, (unsigned long long)n_res_bad, (unsigned long long)n_tl_ok,
         (unsigned long long)n_tl_bad, (unsigned long long)n_tp_ok, (unsigned long long)n_tp_bad, (unsigned long long)n_exp_small);
  bs_flow_free(cf);
  ff.input_queue.push(nullptr);
}

// ------------------------------------------------------------------ flow lockstep (real class vs C)
struct Lockstep {
  VioConfig vc;
  Calibration<double> calib;
  bs_flow_config kc;
  bs_flow_calib cc;
  FF* ff = nullptr;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> out;
  bs_flow* cf = nullptr;
  uint64_t frames = 0, kps = 0, bad_frames = 0;
  Lockstep() {
    vc.load(g_config);
    load_calib(calib);
    c_config_from(vc, kc);
    c_calib_from(calib, cc);
    ff = new FF(vc, calib);
    ff->output_queue = &out;
    cf = bs_flow_new(&kc, &cc);
  }
  ~Lockstep() {
    ff->input_queue.push(nullptr);
    delete ff;
    bs_flow_free(cf);
  }
  void step(int64_t t, const ManagedImage<uint16_t>& i0, const ManagedImage<uint16_t>& i1, const std::string& ctx) {
    OpticalFlowInput::Ptr d(new OpticalFlowInput);
    d->t_ns = t;
    d->img_data.resize(2);
    for (int c = 0; c < 2; c++) {
      d->img_data[c].img.reset(new ManagedImage<uint16_t>(i0.w, i0.h));
      const ManagedImage<uint16_t>& s = c ? i1 : i0;
      memcpy(d->img_data[c].img->ptr, s.ptr, s.size() * 2);
    }
    ff->input_queue.push(d);
    OpticalFlowResult::Ptr res;
    out.pop(res);
    const uint16_t* imgs[2] = {i0.ptr, i1.ptr};
    int rc = bs_flow_process(cf, t, imgs, (int)i0.w, (int)i0.h);
    bool ok = rc == 0;
    for (int c = 0; ok && c < 2; c++) {
      const bs_flow_obs* o = bs_flow_result(cf, c);
      const auto& m = res->observations[c];
      ok = (size_t)o->n == m.size();
      int k = 0;
      for (auto it = m.begin(); ok && it != m.end(); ++it, ++k) {
        ok = o->kp[k].id == it->first && same(o->kp[k].m, it->second.matrix().data(), 6);
        kps++;
      }
    }
    frames++;
    if (!ok) bad_frames++;
    chk(ok, "flow_frame", ctx + " t=" + std::to_string(t) + " rc=" + std::to_string(rc));
  }
};

static void test_flow(const std::string& seq, size_t start, size_t count) {
  auto names = read_names(seq);
  Lockstep L;
  for (size_t f = start; f < std::min(names.size(), start + count); f++) {
    ManagedImage<uint16_t> a, b;
    if (!load_u16(seq + "/mav0/cam0/data/" + names[f], a) || !load_u16(seq + "/mav0/cam1/data/" + names[f], b)) { fprintf(stderr, "load failed\n"); exit(2); }
    L.step(1000 + (int64_t)f, a, b, "frame " + std::to_string(f));
  }
  printf("  frames %llu, observations compared %llu, bad frames %llu, last id %llu\n", (unsigned long long)L.frames, (unsigned long long)L.kps,
         (unsigned long long)L.bad_frames, (unsigned long long)bs_flow_last_keypoint_id(L.cf));
}

// ------------------------------------------------------------------ synthetic stereo sequences
static void make_texture(Rng& r, int w, int h, std::vector<float>& tex) {
  tex.assign((size_t)w * h, 0.f);
  for (auto& v : tex) v = (float)ur(r, 0, 255);
  std::vector<float> t2(tex.size());
  int passes = ui(r, 1, 3);
  for (int p = 0; p < passes; p++) {
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float s = 0;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) s += tex[(size_t)std::min(h - 1, std::max(0, y + dy)) * w + std::min(w - 1, std::max(0, x + dx))];
        t2[(size_t)y * w + x] = s / 9.f;
      }
    tex.swap(t2);
  }
  float lo = 1e9, hi = -1e9;
  for (float v : tex) { lo = std::min(lo, v); hi = std::max(hi, v); }
  for (auto& v : tex) v = (v - lo) / (hi - lo) * 255.f;
}

static void sample(const std::vector<float>& tex, int w, int h, double sx, double sy, ManagedImage<uint16_t>& out, Rng& r, double noise, int mode) {
  out.Reinitialise(w, h);
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      double fx = x + sx, fy = y + sy;
      int ix = (int)std::floor(fx), iy = (int)std::floor(fy);
      double ax = fx - ix, ay = fy - iy;
      auto T = [&](int xx, int yy) { return (double)tex[(size_t)std::min(h - 1, std::max(0, yy)) * w + std::min(w - 1, std::max(0, xx))]; };
      double v = (1 - ax) * (1 - ay) * T(ix, iy) + ax * (1 - ay) * T(ix + 1, iy) + (1 - ax) * ay * T(ix, iy + 1) + ax * ay * T(ix + 1, iy + 1);
      v += ur(r, -noise, noise);
      if (mode == 1) v = v * 0.25;                       // dark
      if (mode == 2) v = std::min(255.0, v * 3.0);       // saturating
      out(x, y) = (uint16_t)(((int)std::min(255.0, std::max(0.0, std::round(v))) << 8));
    }
}

static void test_synth(int seed, int nseq) {
  Rng r(4000 + seed);
  const int w = 752, h = 480;
  for (int s = 0; s < nseq; s++) {
    Lockstep L;
    std::vector<float> tex;
    make_texture(r, w, h, tex);
    // black / white blocks painted into the texture (invalid patches, all-black patches)
    for (int b = 0; b < 12; b++) {
      int bx = ui(r, 0, w - 60), by = ui(r, 0, h - 60), bs = ui(r, 8, 60);
      float val = b % 3 == 0 ? 0.f : b % 3 == 1 ? 255.f : 128.f;
      for (int y = by; y < std::min(h, by + bs); y++)
        for (int x = bx; x < std::min(w, bx + bs); x++) tex[(size_t)y * w + x] = val;
    }
    double x = 0, y = 0, vx = ur(r, -2, 2), vy = ur(r, -2, 2);
    double disp = ur(r, 3, 30), dy = ur(r, -1.5, 1.5);
    int mode = s % 3;
    double noise = s % 2 ? 1.0 : 6.0;
    for (int f = 0; f < 25; f++) {
      ManagedImage<uint16_t> a, b;
      sample(tex, w, h, x, y, a, r, noise, mode);
      sample(tex, w, h, x + disp, y + dy, b, r, noise, mode);
      L.step(f, a, b, "synth seq " + std::to_string(s) + " frame " + std::to_string(f));
      x += vx + ur(r, -0.5, 0.5);
      y += vy + ur(r, -0.5, 0.5);
      if (f % 8 == 7) { vx = ur(r, -4, 4); vy = ur(r, -4, 4); }
    }
    printf("  seq %d (mode %d): frames %llu observations %llu bad %llu last id %llu\n", s, mode, (unsigned long long)L.frames, (unsigned long long)L.kps,
           (unsigned long long)L.bad_frames, (unsigned long long)bs_flow_last_keypoint_id(L.cf));
  }
}

// ------------------------------------------------------------------ replay against the M0 dump
struct Cur { const uint8_t* p; uint64_t n, i; bool err; };
template <class T> static T rd(Cur& c) { T v{}; if (c.i + sizeof(T) > c.n) { c.err = true; return v; } memcpy(&v, c.p + c.i, sizeof(T)); c.i += sizeof(T); return v; }

static void test_replay(const std::string& flow, const std::string& seq, size_t maxframes) {
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
  VioConfig vc;
  vc.load(g_config);
  bs_flow_config kc;
  bs_flow_calib cc;
  bs_flow_config_default(&kc);
  if (bs_flow_config_load(g_config.c_str(), &kc) || bs_flow_calib_load(g_calib.c_str(), &cc)) { fprintf(stderr, "json load failed\n"); exit(2); }
  bs_flow* cf = bs_flow_new(&kc, &cc);
  Cur c{buf.data(), buf.size(), 0, false};
  uint64_t frames = 0, frames_ok = 0, kps = 0, kps_bad = 0, count_bad = 0, id_bad = 0, new_total = 0, lost_total = 0, fc_bad = 0;
  uint64_t maxn = 0;
  while (c.i < c.n && !c.err && (maxframes == 0 || frames < maxframes)) {
    uint32_t tag = rd<uint32_t>(c);
    uint64_t len = rd<uint64_t>(c);
    if (c.err || len > c.n - c.i) break;
    Cur pl{c.p + c.i, len, 0, false};
    c.i += len;
    if (tag != 1) continue;
    int64_t t = rd<int64_t>(pl);
    uint64_t fcount = rd<uint64_t>(pl);
    uint32_t ncam = rd<uint32_t>(pl);
    for (uint32_t k = 0; k < ncam; k++) { rd<uint32_t>(pl); rd<uint32_t>(pl); rd<uint64_t>(pl); }
    struct Ob { uint64_t id; float m[6]; };
    std::vector<Ob> obs[4];
    for (uint32_t k = 0; k < ncam; k++) {
      uint32_t n = rd<uint32_t>(pl);
      for (uint32_t j = 0; j < n; j++) { Ob o; o.id = rd<uint64_t>(pl); float rm[6]; for (int q = 0; q < 6; q++) rm[q] = rd<float>(pl);
        o.m[0] = rm[0]; o.m[1] = rm[3]; o.m[2] = rm[1]; o.m[3] = rm[4]; o.m[4] = rm[2]; o.m[5] = rm[5];   // dump: row-major 2x3 -> column-major
        obs[k].push_back(o); }
    }
    if (pl.err || ncam != 2) { chk(false, "replay_parse"); break; }
    uint16_t* im[2] = {nullptr, nullptr};
    int w = 0, h = 0;
    for (int k = 0; k < 2; k++) {
      int ww, hh;
      std::string p = seq + "/mav0/cam" + std::to_string(k) + "/data/" + by_t[t];
      if (bs_image_load_euroc(p.c_str(), &im[k], &ww, &hh) != BS_IMG_OK) { fprintf(stderr, "load failed %s\n", p.c_str()); exit(2); }
      w = ww; h = hh;
    }
    const uint16_t* ci[2] = {im[0], im[1]};
    int rc = bs_flow_process(cf, t, ci, w, h);
    bool ok = rc == 0;
    chk(fcount == frames, "frame_counter");
    fc_bad += fcount != frames;
    std::set<uint64_t> prev0;
    for (int k = 0; ok && k < 2; k++) {
      const bs_flow_obs* o = bs_flow_result(cf, k);
      maxn = std::max<uint64_t>(maxn, o->n);
      if ((size_t)o->n != obs[k].size()) { ok = false; count_bad++; break; }
      for (int j = 0; j < o->n; j++) {
        kps++;
        bool e = o->kp[j].id == obs[k][j].id && same(o->kp[j].m, obs[k][j].m, 6);
        if (o->kp[j].id != obs[k][j].id) id_bad++;
        if (!e) { kps_bad++; ok = false; }
      }
    }
    chk(ok, "replay_frame", "frame " + std::to_string(frames) + " t=" + std::to_string(t) + " rc=" + std::to_string(rc));
    frames_ok += ok;
    for (int k = 0; k < 2; k++) free(im[k]);
    frames++;
    if (!ok && g_bad > 3) break;
  }
  (void)new_total; (void)lost_total;
  printf("  frames %llu, bit-exact frames %llu, observations compared %llu (mismatching %llu, id mismatches %llu), count mismatches %llu, frame-counter mismatches %llu, max keypoints per cam %llu, last id %llu\n",
         (unsigned long long)frames, (unsigned long long)frames_ok, (unsigned long long)kps, (unsigned long long)kps_bad, (unsigned long long)id_bad,
         (unsigned long long)count_bad, (unsigned long long)fc_bad, (unsigned long long)maxn, (unsigned long long)bs_flow_last_keypoint_id(cf));
  bs_flow_free(cf);
}

int main(int argc, char** argv) {
  tbb::global_control tbb_gc(tbb::global_control::max_allowed_parallelism, 1);
  cv::setNumThreads(0);
  std::string mode = argc > 1 ? argv[1] : "";
  auto seed_arg = [&](int idx) { return (argc > idx + 1 && std::string(argv[idx]) == "seed") ? atoi(argv[idx + 1]) : 1; };
  if (mode == "eigen") { test_eigen(seed_arg(2)); return finish("bs_flow eigen"); }
  if (mode == "interp") { test_interp(seed_arg(2)); return finish("bs_flow interp"); }
  if (mode == "patch" && argc > 2) { test_patch(argv[2], seed_arg(3)); return finish("bs_flow patch"); }
  if (mode == "calib" && argc > 3) { test_calib(argv[2], argv[3]); return finish("bs_flow calib"); }
  if (mode == "flow" && argc > 4) {
    if (argc > 6) { g_calib = argv[5]; g_config = argv[6]; }
    test_flow(argv[2], strtoul(argv[3], 0, 10), strtoul(argv[4], 0, 10));
    return finish("bs_flow flow");
  }
  if (mode == "synth") { test_synth(seed_arg(2), argc > 4 ? atoi(argv[4]) : 6); return finish("bs_flow synth"); }
  if (mode == "replay" && argc > 3) {
    if (argc > 5) { g_calib = argv[4]; g_config = argv[5]; }
    test_replay(argv[2], argv[3], argc > 6 ? strtoul(argv[6], 0, 10) : 0);
    return finish("bs_flow replay");
  }
  fprintf(stderr, "usage: bs_flow_test eigen|interp|patch <seq>|calib <calib> <config>|flow <seq> <start> <n>|synth|replay <flow.bin> <seq>\n");
  return 2;
}
