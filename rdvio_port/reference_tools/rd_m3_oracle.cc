// RD-VIO M3 oracle (reference tooling, never distributed): drives the REAL rdvio classes (UniformInteger / LotBox over std::default_random_engine,
// glibc rand(), find_essential_matrix / find_rotation_matrix / find_homography_matrix and the PARSAC variants, the geometric error functions,
// PoissonDiskFilter) with random inputs and writes records in the layouts of rdvio_port/c/rd_rand.h / rd_ransac.h / rd_poisson.h.
// usage: rd_m3_oracle <outdir> [seed] [count]
#include <rdvio/geometry/essential.h>
#include <rdvio/geometry/homography.h>
#include <rdvio/geometry/stereo.h>
#include <rdvio/util/parsac.h>
#include <rdvio/util/poisson_disk_filter.h>
#include <rdvio/util/random.h>
#include <csetjmp>
#include <csignal>
#include <random>
#include <cstdio>
#include <cstdlib>
#include <string>
using namespace rdvio;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static void W(FILE *f, const void *p, size_t n) { fwrite(p, 1, n, f); }
static void Wd(FILE *f, double v) { W(f, &v, 8); }
static void Wu(FILE *f, uint32_t v) { W(f, &v, 4); }
static void Wq(FILE *f, uint64_t v) { W(f, &v, 8); }

// The C++ PARSAC reads an empty vector (UB, null dereference) when no hypothesis was ever accepted; such calls are skipped
// (the record is not written, the persistent bin-confidence state is untouched because the crash precedes its update).
static sigjmp_buf g_jb;
static void on_segv(int) { siglongjmp(g_jb, 1); }
static int g_skipped = 0;

static FILE *open(const std::string &dir, const char *name) { return fopen((dir + "/" + name + ".bin").c_str(), "wb"); }

static void rng_tests(const std::string &dir, int cnt) {
    FILE *fu = open(dir, "rnguni"), *fl = open(dir, "rnglot"), *fg = open(dir, "rngglibc");
    // uniform_int_distribution<size_t> over default_random_engine: record = u32 seed, u32 n, n x {u64 a, u64 b, u64 result}
    for (int t = 0; t < cnt / 20; ++t) {
        uint32_t seed = (I(0, 4) == 0) ? (uint32_t)I(0, 5) : (uint32_t)rng();
        if (I(0, 20) == 0) seed = 2147483647u;
        UniformInteger<size_t> d; d.seed(seed);
        int n = I(1, 60);
        Wu(fu, seed); Wu(fu, n);
        for (int i = 0; i < n; ++i) {
            uint64_t a, b;
            int m = I(0, 9);
            if (m < 5) { a = I(0, 200); b = a + I(0, 300); }
            else if (m == 5) { a = 0; b = (uint64_t)2147483645ULL; }
            else if (m == 6) { a = 5; b = 5 + (uint64_t)2147483646ULL + I(0, 1000); }
            else if (m == 7) { a = 0; b = (uint64_t)rng() >> I(0, 40); }
            else if (m == 8) { a = I(0, 3); b = a; }
            else { a = 0; b = 2147483644ULL - I(0, 3); }
            uint64_t r = d.next(a, b);
            Wq(fu, a); Wq(fu, b); Wq(fu, r);
        }
    }
    // LotBox: u32 size, u32 seed, u32 nops, nops x {u32 op (0 draw, 1 refill_all), u64 result}
    for (int t = 0; t < cnt / 20; ++t) {
        uint32_t size = I(1, 120), seed = (I(0, 3) == 0) ? 0 : (uint32_t)rng();
        LotBox lb(size); lb.seed(seed);
        int n = I(1, 80);
        Wu(fl, size); Wu(fl, seed); Wu(fl, n);
        for (int i = 0; i < n; ++i) {
            if (I(0, 6) == 0) { lb.refill_all(); Wu(fl, 1); Wq(fl, 0); }
            else { uint64_t r = lb.draw_without_replacement(); Wu(fl, 0); Wq(fl, r); }
        }
    }
    // glibc: u32 seed, then 2000 outputs
    for (int t = 0; t < std::max(4, cnt / 500); ++t) {
        uint32_t seed = (t == 0) ? 0 : (t == 1) ? 1 : (uint32_t)rng();
        srand(seed);
        Wu(fg, seed);
        for (int i = 0; i < 2000; ++i) { int32_t v = rand(); W(fg, &v, 4); }
    }
    srand(0);
    fclose(fu); fclose(fl); fclose(fg);
}

static vector<3> rbearing(double s = 0.6) { return vector<3>(N() * s, N() * s, 1.0).normalized(); }

// ---- error functions ----
static void err_tests(const std::string &dir, int cnt) {
    FILE *fe = open(dir, "essgeo"), *fh = open(dir, "homgeo"), *fr = open(dir, "roterr");
    for (int t = 0; t < cnt; ++t) {
        matrix<3> M; for (int i = 0; i < 9; ++i) M.data()[i] = N();
        vector<2> a(N() * 0.5, N() * 0.5), b(N() * 0.5, N() * 0.5);
        double e = essential_geometric_error(M, a, b);
        W(fe, M.data(), 72); W(fe, a.data(), 16); W(fe, b.data(), 16); Wd(fe, e);
        double h = homography_geometric_error(M, a, b);
        W(fh, M.data(), 72); W(fh, a.data(), 16); W(fh, b.data(), 16); Wd(fh, h);
        Eigen::Quaterniond q(N(), N(), N(), N()); q.normalize();
        matrix<3> R = q.toRotationMatrix();
        vector<3> p = rbearing(), p2 = (R * p + vector<3>(N() * 1e-2, N() * 1e-2, N() * 1e-2)).normalized();
        double r = acos((R * p).dot(p2));
        W(fr, R.data(), 72); W(fr, p.data(), 24); W(fr, p2.data(), 24); Wd(fr, r);
    }
    fclose(fe); fclose(fh); fclose(fr);
}

// ---- find_* ----
static void put_find(FILE *f, size_t n, double thr, double conf, size_t maxit, int seed, const std::vector<vector<2>> &a, const std::vector<vector<2>> &b,
                     const matrix<3> &M, const std::vector<char> &mask) {
    Wu(f, (uint32_t)n); Wd(f, thr); Wd(f, conf); Wq(f, maxit); W(f, &seed, 4);
    for (auto &p : a) W(f, p.data(), 16);
    for (auto &p : b) W(f, p.data(), 16);
    W(f, M.data(), 72);
    Wu(f, (uint32_t)mask.size()); W(f, mask.data(), mask.size());
}

static void find_tests(const std::string &dir, int cnt) {
    FILE *fe = open(dir, "fess"), *fr = open(dir, "frot"), *fh = open(dir, "fhom"), *pe = open(dir, "pess"), *ph = open(dir, "phom");
    for (int t = 0; t < cnt; ++t) {
        size_t n = (I(0, 15) == 0) ? I(0, 8) : I(5, 90);
        double outlier = (I(0, 3) == 0) ? 0.0 : U(0, 0.6);
        double noise = (I(0, 3) == 0) ? 1e-4 : U(1e-4, 4e-3);
        Eigen::Quaterniond Rq(Eigen::AngleAxisd(U(0, 0.4), vector<3>(N(), N(), N()).normalized()));
        vector<3> tr = vector<3>(N(), N(), N()).normalized() * U(0.05, 1.0);
        double thr = (I(0, 3) == 0) ? 1.0 : U(1e-4, 4e-3) * 1.0;
        double conf = (I(0, 2) == 0) ? 0.99 : 0.999;
        size_t maxit = (I(0, 3) == 0) ? (size_t)I(1, 60) : 1000;
        int seed = (I(0, 2) == 0) ? 0 : I(0, 100000);
        std::vector<vector<2>> a(n), b(n);
        for (size_t i = 0; i < n; ++i) {
            vector<3> X(N() * 1.0, N() * 0.8, U(2, 12));
            vector<3> Y = Rq * X + tr;
            a[i] = X.hnormalized(); b[i] = Y.hnormalized();
            a[i] += vector<2>(N() * noise, N() * noise); b[i] += vector<2>(N() * noise, N() * noise);
            if (U(0, 1) < outlier) b[i] = vector<2>(U(-0.9, 0.9), U(-0.9, 0.9));
            for (int k = 0; k < 2; ++k) { a[i](k) = std::max(-0.95, std::min(0.95, a[i](k))); b[i](k) = std::max(-0.95, std::min(0.95, b[i](k))); }
        }
        if (I(0, 30) == 0 && n > 3) b[1](0) = b[0](0), b[1](1) = b[0](1);
        std::vector<char> mask;
        // essential
        matrix<3> E = find_essential_matrix(a, b, mask, thr, conf, maxit, seed);
        put_find(fe, n, thr, conf, maxit, seed, a, b, E, mask);
        // homography
        matrix<3> H = find_homography_matrix(a, b, mask, thr, conf, maxit, seed);
        put_find(fh, n, thr, conf, maxit, seed, a, b, H, mask);
        // parsac (persistent static state inside; the harness replays the sequence in order)
        signal(SIGSEGV, on_segv);
        if (!sigsetjmp(g_jb, 1)) {
            matrix<3> PE = find_essential_matrix_parsac(a, b, mask, thr, conf, maxit, seed);
            put_find(pe, n, thr, conf, maxit, seed, a, b, PE, mask);
        } else g_skipped++;
        if (!sigsetjmp(g_jb, 1)) {
            matrix<3> PH = find_homography_matrix_parsac(a, b, mask, thr, conf, maxit, seed);
            put_find(ph, n, thr, conf, maxit, seed, a, b, PH, mask);
        } else g_skipped++;
        signal(SIGSEGV, SIG_DFL);
        // rotation: bearings
        {
            std::vector<vector<3>> p(n), q(n);
            Eigen::Quaterniond Rr(Eigen::AngleAxisd(U(0, 0.3), vector<3>(N(), N(), N()).normalized()));
            for (size_t i = 0; i < n; ++i) {
                p[i] = rbearing(); q[i] = (Rr * p[i] + vector<3>(N() * noise, N() * noise, N() * noise)).normalized();
                if (U(0, 1) < outlier) q[i] = rbearing();
            }
            double rthr = U(0.05, 2.0) * M_PI / 180.0;
            matrix<3> R = find_rotation_matrix(p, q, mask, rthr, conf, maxit, seed);
            Wu(fr, (uint32_t)n); Wd(fr, rthr); Wd(fr, conf); Wq(fr, maxit); W(fr, &seed, 4);
            for (auto &x : p) W(fr, x.data(), 24);
            for (auto &x : q) W(fr, x.data(), 24);
            W(fr, R.data(), 72); Wu(fr, (uint32_t)mask.size()); W(fr, mask.data(), mask.size());
        }
    }
    fclose(fe); fclose(fr); fclose(fh); fclose(pe); fclose(ph);
}

// ---- Poisson disk filter: op stream ----
static void poisson_tests(const std::string &dir, int cnt) {
    FILE *f = open(dir, "pois");
    for (int t = 0; t < cnt / 10; ++t) {
        uint64_t id = t;
        double radius = (I(0, 3) == 0) ? U(1.0, 5.0) : U(8.0, 30.0);
        PoissonDiskFilter<2> filt(radius);
        Wu(f, 0); Wq(f, id); Wd(f, radius);
        int nops = I(1, 250);
        double span = (I(0, 2) == 0) ? 60.0 : 700.0;
        for (int i = 0; i < nops; ++i) {
            int op = I(0, 9);
            vector<2> p(U(-5, span), U(-5, span * 0.6));
            if (I(0, 8) == 0 && filt.get_points().size()) p = filt.get_points()[I(0, (int)filt.get_points().size() - 1)] + vector<2>(U(-1, 1), U(-1, 1));
            if (op < 2) { filt.preset_point(p); Wu(f, 1); Wq(f, id); W(f, p.data(), 16); }
            else if (op < 5) { bool r = filt.permit_point(p); Wu(f, 2); Wq(f, id); W(f, p.data(), 16); Wu(f, r); }
            else if (op < 8) { bool r = filt.insert_point(p); Wu(f, 3); Wq(f, id); W(f, p.data(), 16); Wu(f, r); }
            else if (op < 9) {
                std::vector<vector<2>> cand(I(0, 30)); for (auto &c : cand) c = vector<2>(U(-5, span), U(-5, span * 0.6));
                Wu(f, 4); Wq(f, id); Wu(f, (uint32_t)cand.size()); for (auto &c : cand) W(f, c.data(), 16);
                filt.insert_points(cand);
                Wu(f, (uint32_t)cand.size()); for (auto &c : cand) W(f, c.data(), 16);
            } else if (I(0, 3) == 0) { filt.clear(); Wu(f, 5); Wq(f, id); }
        }
        Wu(f, 6); Wq(f, id);
    }
    fclose(f);
}

int main(int argc, char **argv) {
    std::string dir = argv[1];
    rng.seed(argc > 2 ? atoll(argv[2]) : 1);
    int cnt = argc > 3 ? atoi(argv[3]) : 3000;
    rng_tests(dir, cnt);
    err_tests(dir, cnt);
    find_tests(dir, cnt / 10);
    poisson_tests(dir, cnt);
    fprintf(stderr, "skipped parsac calls (C++ crash on empty best-set): %d\n", g_skipped);
    return 0;
}
