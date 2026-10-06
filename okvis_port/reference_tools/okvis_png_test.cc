// OK_PORT_TEST_C: ok_png.c
// OK_PORT_TEST_LIBS: imgcodecs zlib
// Tolerance-0 comparison of ok_png_decode_gray (okvis_port/c/ok_png.c) against the REAL cv::imdecode(buf, IMREAD_GRAYSCALE)
// of the reference's OpenCV 4.6 (the decoder behind cv::imread in DatasetReader):
//   real   : every PNG under $OK_PNG_DIRS (colon-separated; default: the EuRoC frames of runs/okvis_port/reference_brisk/frames)
//   opencv : random images encoded by cv::imencode (8/16-bit, 1/3/4 channels, compression 0-9, the five zlib strategies, bilevel)
//   crafted: PNGs written here with zlib: every colour type / bit depth, all five row filters (random per row), Adam7, IDAT split
//            into random chunks, random zlib levels, gAMA / sRGB / sBIT / tRNS / PLTE ancillary chunks, grey-equal RGB pixels
//   corrupt: truncated and bit-flipped versions of the crafted files: the C decoder must not crash (run under ASan to check
//            memory safety); where both decoders succeed their pixels are compared too, the rest is only counted.
// Last line: "png: <mismatches>/<compared>".
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zlib.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <random>
#include <string>
#include <vector>
extern "C" {
#include "../c/ok_png.h"
}

static long g_bad, g_tot, g_files[4], g_both_fail, g_c_only_fail, g_cv_only_fail, g_corrupt_cmp;

/* compare one encoded PNG; kind 0 real, 1 opencv, 2 crafted, 3 corrupt (failures of either side only counted) */
static void check(const std::vector<unsigned char>& png, int kind, const std::string& what) {
    cv::Mat ref;
    try { ref = cv::imdecode(png, cv::IMREAD_GRAYSCALE); } catch (const cv::Exception&) { ref = cv::Mat(); }
    unsigned char* out = nullptr;
    int w = 0, h = 0;
    const int err = ok_png_decode_gray(png.data(), png.size(), &out, &w, &h);
    g_files[kind]++;
    if (ref.empty() || err) {
        if (kind == 3) {
            if (ref.empty() && err) g_both_fail++; else if (err) g_c_only_fail++; else g_cv_only_fail++;
        } else {
            g_tot++; g_bad++;
            if (g_bad <= 10) std::printf("  %s: OpenCV %s, C error %d\n", what.c_str(), ref.empty() ? "failed" : "decoded", err);
        }
        std::free(out);
        return;
    }
    if (kind == 3) g_corrupt_cmp++;
    g_tot++;
    bool same = ref.type() == CV_8UC1 && ref.cols == w && ref.rows == h && ref.isContinuous() &&
                std::memcmp(ref.data, out, (size_t)w * h) == 0;
    if (!same) {
        g_bad++;
        if (g_bad <= 10) {
            long diff = 0; int first = -1;
            if (ref.cols == w && ref.rows == h)
                for (int i = 0; i < w * h; ++i) if (ref.data[i] != out[i]) { if (first < 0) first = i; diff++; }
            std::printf("  %s: %dx%d vs %dx%d, %ld pixels differ (first %d: cv %d C %d)\n", what.c_str(), ref.cols, ref.rows, w, h, diff,
                        first, first >= 0 ? ref.data[first] : -1, first >= 0 ? out[first] : -1);
        }
    }
    std::free(out);
}

/* ---- a small PNG writer (crafted cases) ---- */
static void put32(std::vector<unsigned char>& v, uint32_t x) { for (int i = 3; i >= 0; --i) v.push_back((unsigned char)(x >> (8 * i))); }
static void chunk(std::vector<unsigned char>& png, const char* tag, const std::vector<unsigned char>& data) {
    put32(png, (uint32_t)data.size());
    std::vector<unsigned char> td(tag, tag + 4);
    td.insert(td.end(), data.begin(), data.end());
    png.insert(png.end(), td.begin(), td.end());
    put32(png, (uint32_t)crc32(0, td.data(), (uInt)td.size()));
}
static int paeth(int a, int b, int c) {
    int p = a + b - c, pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
}
/* filter one image (or Adam7 sub-image) of rows x rowbytes raw scanlines into out, random filter per row */
static void filter_rows(const std::vector<unsigned char>& raw, size_t rows, size_t rowbytes, size_t bpp, std::mt19937& rng,
                        std::vector<unsigned char>& out) {
    for (size_t y = 0; y < rows; ++y) {
        const int f = (int)(rng() % 5);
        out.push_back((unsigned char)f);
        const unsigned char* cur = raw.data() + y * rowbytes;
        const unsigned char* prev = y ? raw.data() + (y - 1) * rowbytes : nullptr;
        for (size_t x = 0; x < rowbytes; ++x) {
            const int a = x >= bpp ? cur[x - bpp] : 0, b = prev ? prev[x] : 0, c = (prev && x >= bpp) ? prev[x - bpp] : 0;
            const int pred = f == 0 ? 0 : f == 1 ? a : f == 2 ? b : f == 3 ? (a + b) / 2 : paeth(a, b, c);
            out.push_back((unsigned char)(cur[x] - pred));
        }
    }
}
static std::vector<unsigned char> craft(std::mt19937& rng, std::string& what) {
    static const int types[5] = {0, 2, 3, 4, 6};
    const int type = types[rng() % 5];
    std::vector<int> depths;
    if (type == 0) depths = {1, 2, 4, 8, 16}; else if (type == 3) depths = {1, 2, 4, 8}; else depths = {8, 16};
    const int depth = depths[rng() % depths.size()];
    const int ch = type == 2 ? 3 : type == 4 ? 2 : type == 6 ? 4 : 1;
    const uint32_t W = 1 + rng() % 70, H = 1 + rng() % 70;
    const int interlace = (int)(rng() % 3 == 0);
    const int grey_rgb = (type == 2 || type == 6) && rng() % 3 == 0;   /* r == g == b everywhere */
    const int npal = type == 3 ? 1 + (int)(rng() % (1u << depth)) : 0;
    std::vector<unsigned char> png = {137, 80, 78, 71, 13, 10, 26, 10}, ihdr;
    put32(ihdr, W); put32(ihdr, H);
    ihdr.push_back((unsigned char)depth); ihdr.push_back((unsigned char)type); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back((unsigned char)interlace);
    chunk(png, "IHDR", ihdr);
    char desc[200];
    std::string anc;
    const int r = (int)(rng() % 6);
    if (r == 1) { std::vector<unsigned char> g; static const uint32_t gs[] = {45455, 100000, 220000, 50000, 31250, 95000, 105001};
                  const uint32_t gv = gs[rng() % 7]; put32(g, gv); chunk(png, "gAMA", g); anc += " gAMA" + std::to_string(gv); }
    if (r == 2) { chunk(png, "sRGB", {(unsigned char)(rng() % 4)}); anc += " sRGB"; }
    if (r == 3 && (type == 2 || type == 6)) {
        std::vector<unsigned char> s; for (int i = 0; i < ch; ++i) s.push_back((unsigned char)(1 + rng() % depth));
        chunk(png, "sBIT", s); anc += " sBIT";
        std::vector<unsigned char> g; put32(g, 45455); chunk(png, "gAMA", g); anc += " gAMA45455";
    }
    if (type == 3 || ((type == 2 || type == 6) && rng() % 4 == 0)) {
        const int n = type == 3 ? npal : 1 + (int)(rng() % 256);
        std::vector<unsigned char> pal;
        for (int i = 0; i < 3 * n; ++i) pal.push_back((unsigned char)rng());
        if (type == 3 && rng() % 3 == 0) for (int i = 0; i < n; ++i) pal[3 * i + 1] = pal[3 * i + 2] = pal[3 * i];
        chunk(png, "PLTE", pal); anc += " PLTE" + std::to_string(n);
    }
    if (r == 4 && type != 4 && type != 6) {
        std::vector<unsigned char> t;
        if (type == 3) for (int i = 0; i < npal; ++i) t.push_back((unsigned char)rng());
        else for (int i = 0; i < (type == 2 ? 3 : 1); ++i) { const unsigned v = (unsigned)(rng() % (1u << depth)); t.push_back((unsigned char)(v >> 8)); t.push_back((unsigned char)v); }
        chunk(png, "tRNS", t); anc += " tRNS";
    }
    /* raw samples, then the (Adam7) scanlines */
    const size_t bpp = (size_t)((ch * depth + 7) / 8);
    auto sample = [&](void) -> unsigned { return type == 3 ? (unsigned)(rng() % npal) : (unsigned)(rng() % (1u << depth)); };
    std::vector<unsigned> img((size_t)W * H * ch);
    for (size_t i = 0; i < (size_t)W * H; ++i) {
        for (int c = 0; c < ch; ++c) img[i * ch + c] = sample();
        if (grey_rgb) img[i * ch + 1] = img[i * ch + 2] = img[i * ch];
    }
    static const int pass[7][4] = {{0, 0, 8, 8}, {4, 0, 8, 8}, {0, 4, 4, 8}, {2, 0, 4, 4}, {0, 2, 2, 4}, {1, 0, 2, 2}, {0, 1, 1, 2}};
    std::vector<unsigned char> filtered;
    for (int p = 0; p < (interlace ? 7 : 1); ++p) {
        const uint32_t x0 = interlace ? pass[p][0] : 0, y0 = interlace ? pass[p][1] : 0, dx = interlace ? pass[p][2] : 1, dy = interlace ? pass[p][3] : 1;
        const size_t pw = W > x0 ? 1 + (W - 1 - x0) / dx : 0, ph = H > y0 ? 1 + (H - 1 - y0) / dy : 0;
        if (!pw || !ph) continue;
        const size_t rowbytes = (pw * ch * depth + 7) / 8;
        std::vector<unsigned char> raw(rowbytes * ph, 0);
        for (size_t y = 0; y < ph; ++y)
            for (size_t x = 0; x < pw; ++x)
                for (int c = 0; c < ch; ++c) {
                    const unsigned v = img[((y0 + y * dy) * W + (x0 + x * dx)) * ch + c];
                    const size_t bit = (x * ch + c) * depth;
                    unsigned char* row = raw.data() + y * rowbytes;
                    if (depth == 16) { row[bit / 8] = (unsigned char)(v >> 8); row[bit / 8 + 1] = (unsigned char)v; }
                    else if (depth == 8) row[bit / 8] = (unsigned char)v;
                    else row[bit / 8] |= (unsigned char)(v << (8 - depth - bit % 8));
                }
        filter_rows(raw, ph, rowbytes, bpp, rng, filtered);
    }
    uLongf zn = compressBound((uLong)filtered.size());
    std::vector<unsigned char> z(zn);
    compress2(z.data(), &zn, filtered.data(), (uLong)filtered.size(), (int)(rng() % 10));
    z.resize(zn);
    for (size_t off = 0; off < z.size();) {                      /* IDAT split into random pieces */
        const size_t n = std::min(z.size() - off, (size_t)(1 + rng() % (z.size() + 1)));
        chunk(png, "IDAT", std::vector<unsigned char>(z.begin() + (long)off, z.begin() + (long)(off + n)));
        off += n;
    }
    chunk(png, "IEND", {});
    std::snprintf(desc, sizeof desc, "crafted type %d depth %d %ux%u interlace %d%s%s", type, depth, W, H, interlace, grey_rgb ? " grey-rgb" : "", anc.c_str());
    what = desc;
    return png;
}

static void real_dir(const std::string& dir) {
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    while (dirent* e = readdir(d)) {
        const std::string n = e->d_name;
        if (n.size() < 4 || n.substr(n.size() - 4) != ".png") continue;
        FILE* f = std::fopen((dir + "/" + n).c_str(), "rb");
        if (!f) continue;
        std::vector<unsigned char> buf;
        unsigned char tmp[65536];
        size_t k;
        while ((k = std::fread(tmp, 1, sizeof tmp, f)) > 0) buf.insert(buf.end(), tmp, tmp + k);
        std::fclose(f);
        check(buf, 0, dir + "/" + n);
    }
    closedir(d);
}

int main() {
    cv::setNumThreads(1);
    std::mt19937 rng(20261006u);
    /* real files */
    const char* dirs = std::getenv("OK_PNG_DIRS");
    std::string ds = dirs ? dirs : "runs/okvis_port/reference_brisk/frames";
    for (size_t a = 0; a <= ds.size();) {
        const size_t b = ds.find(':', a);
        real_dir(ds.substr(a, b == std::string::npos ? std::string::npos : b - a));
        if (b == std::string::npos) break;
        a = b + 1;
    }
    /* OpenCV's own encoder */
    static const int cvtypes[6] = {CV_8UC1, CV_8UC3, CV_8UC4, CV_16UC1, CV_16UC3, CV_16UC4};
    for (int i = 0; i < 1500; ++i) {
        const int t = cvtypes[rng() % 6];
        const int W = i == 0 ? 752 : 1 + (int)(rng() % 90), H = i == 0 ? 480 : 1 + (int)(rng() % 90);
        cv::Mat m(H, W, t);
        cv::randu(m, cv::Scalar::all(0), cv::Scalar::all(CV_MAT_DEPTH(t) == CV_16U ? 65536 : 256));
        if (rng() % 4 == 0) cv::GaussianBlur(m, m, cv::Size(5, 5), 2.0);   /* smooth images compress differently */
        std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, (int)(rng() % 10), cv::IMWRITE_PNG_STRATEGY, (int)(rng() % 5)};
        if (t == CV_8UC1 && rng() % 5 == 0) { params.push_back(cv::IMWRITE_PNG_BILEVEL); params.push_back(1); }
        std::vector<unsigned char> buf;
        cv::imencode(".png", m, buf, params);
        check(buf, 1, "opencv type " + std::to_string(t) + " " + std::to_string(W) + "x" + std::to_string(H));
    }
    /* crafted, and corrupted versions of them */
    for (int i = 0; i < 9000; ++i) {
        std::string what;
        std::vector<unsigned char> png = craft(rng, what);
        check(png, 2, what);
        if (i % 3 == 0) {
            std::vector<unsigned char> c = png;
            const int mode = (int)(rng() % 3);
            if (mode == 0) c.resize(rng() % c.size());
            else if (mode == 1) for (int k = 0; k < 1 + (int)(rng() % 4); ++k) c[8 + rng() % (c.size() - 8)] ^= (unsigned char)(1u << (rng() % 8));
            else {                      /* damage the zlib stream / the header fields but keep every CRC valid */
                for (size_t pos = 8; pos + 12 <= c.size();) {
                    const uint32_t len = (uint32_t)c[pos] << 24 | (uint32_t)c[pos + 1] << 16 | (uint32_t)c[pos + 2] << 8 | c[pos + 3];
                    if (pos + 12 + len > c.size()) break;
                    const bool idat = !std::memcmp(&c[pos + 4], "IDAT", 4), ihdr = !std::memcmp(&c[pos + 4], "IHDR", 4);
                    if (len && ((idat && rng() % 2) || (ihdr && rng() % 8 == 0))) {
                        for (int k = 0; k < 1 + (int)(rng() % 3); ++k) c[pos + 8 + rng() % len] ^= (unsigned char)(1u << (rng() % 8));
                        const uint32_t cr = (uint32_t)crc32(0, &c[pos + 4], len + 4);
                        for (int b = 0; b < 4; ++b) c[pos + 8 + len + b] = (unsigned char)(cr >> (24 - 8 * b));
                    }
                    pos += 12 + len;
                }
            }
            check(c, 3, "corrupt " + what);
        }
    }
    std::printf("  files: real %ld, opencv-encoded %ld, crafted %ld, corrupt %ld (both fail %ld, only C fails %ld, only OpenCV fails %ld, both decode %ld)\n",
                g_files[0], g_files[1], g_files[2], g_files[3], g_both_fail, g_c_only_fail, g_cv_only_fail, g_corrupt_cmp);
    std::printf("png: %ld/%ld\n", g_bad, g_tot);
    return g_bad == 0 && g_tot > 0 ? 0 : 1;
}
