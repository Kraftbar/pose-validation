/* SV_PORT_SOURCES: check_sv_extract.c sv_extract.c sv_fast.c sv_image.c sv_undistort.c
 * SPDX-License-Identifier: MIT
 *
 * Harness: for each frame of a TUM sequence, runs sv_orb_extract() on the
 * fixture PGM (see tools/dump_stella_fixtures.cc for how the PGM was
 * produced -- the exact grayscale image stella_vslam's own OpenCV 4.6.0
 * build feeds to orb_extractor::extract()), undistorts the resulting
 * keypoints with sv_undistort_point() (reference config's Camera
 * intrinsics/distortion, hardcoded below -- same config for every fr1_*
 * TUM sequence), and compares keypoints/descriptors 1:1, in order,
 * against runs/stella_port/reference_dumps/<seq>/{keypoints,descriptors}.tsv,
 * via those files' hex columns (bit-exact, no tolerance).
 *
 * response is compared against 0 for every keypoint on both sides: stella's
 * own camera::perspective::undistort_keypoints() (see
 * runs/stella_port/reference_build/src/src/stella_vslam/camera/perspective.cc)
 * default-constructs each undistorted cv::KeyPoint and only copies
 * angle/size/octave across -- response is never copied, so
 * frm_obs_.undist_keypts_ (what keypoints.tsv dumps) always has
 * response==0 regardless of what orb_extractor originally computed. This
 * harness reproduces that by simply not comparing sv_orb_extract's real
 * cornerScore response -- it treats "response" as the constant 0 that both
 * sides are known to produce at this stage of the pipeline.
 *
 * Usage: check_sv_extract <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * Prints "<seq_label>: <mismatches>/<total>\n"; exits 0 iff mismatches==0.
 */
#include "sv_extract.h"
#include "sv_undistort.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SV_CAP 20000

/* Reference config: stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml */
static const sv_camera_params CAM = {
    517.306408, 516.469215, 318.643040, 255.313989,
    0.262383, -0.953104, -0.005358, 0.002628, 1.163314
};
static const sv_orb_params ORB_PARAMS = {1.2f, 8, 20, 7, 800};

typedef struct ref_row {
    int frame_idx, kp_idx;
    float x, y, angle, response;
    int octave;
    unsigned char desc[32];
} ref_row;

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Reads one aligned row from both TSVs (they are guaranteed 1:1, same
 * frame_idx/kp_idx order -- see reference README "Dumps"). Returns 1 on
 * success, 0 at EOF. */
static int read_ref_row(FILE* kf, FILE* df, ref_row* out) {
    char kline[1024], dline[128];
    if (!fgets(kline, sizeof(kline), kf)) return 0;
    if (!fgets(dline, sizeof(dline), df)) return 0;

    /* keypoints.tsv: frame_idx kp_idx x_9g x_hex y_9g y_hex octave angle_9g angle_hex response_9g response_hex */
    char x_hex[64], y_hex[64], angle_hex[64], response_hex[64];
    double dummy;
    int n = sscanf(kline, "%d\t%d\t%lf\t%63s\t%lf\t%63s\t%d\t%lf\t%63s\t%lf\t%63s",
                   &out->frame_idx, &out->kp_idx, &dummy, x_hex, &dummy, y_hex,
                   &out->octave, &dummy, angle_hex, &dummy, response_hex);
    if (n != 11) {
        fprintf(stderr, "check_sv_extract: malformed keypoints.tsv row: %s", kline);
        return 0;
    }
    out->x = strtof(x_hex, NULL);
    out->y = strtof(y_hex, NULL);
    out->angle = strtof(angle_hex, NULL);
    out->response = strtof(response_hex, NULL);

    /* descriptors.tsv: frame_idx kp_idx descriptor_hex */
    int dframe, dkp;
    char dhex[80];
    n = sscanf(dline, "%d\t%d\t%79s", &dframe, &dkp, dhex);
    if (n != 3 || dframe != out->frame_idx || dkp != out->kp_idx || strlen(dhex) != 64) {
        fprintf(stderr, "check_sv_extract: descriptors.tsv desync (frame %d/%d vs %d/%d)\n",
                dframe, dkp, out->frame_idx, out->kp_idx);
        return 0;
    }
    int i;
    for (i = 0; i < 32; i++) {
        int hi = hex_nibble(dhex[i * 2]), lo = hex_nibble(dhex[i * 2 + 1]);
        out->desc[i] = (unsigned char)((hi << 4) | lo);
    }
    return 1;
}

static unsigned char* read_pgm(const char* path, int* w, int* h) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    char magic[3] = {0};
    if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P5") != 0) { fclose(f); return NULL; }
    int maxval;
    /* Skip whitespace/comments minimally; fixture writer never emits comments. */
    if (fscanf(f, "%d %d %d", w, h, &maxval) != 3) { fclose(f); return NULL; }
    fgetc(f); /* single whitespace byte before binary data */
    unsigned char* buf = (unsigned char*)malloc((size_t)(*w) * (size_t)(*h));
    size_t got = fread(buf, 1, (size_t)(*w) * (size_t)(*h), f);
    fclose(f);
    if (got != (size_t)(*w) * (size_t)(*h)) { free(buf); return NULL; }
    return buf;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_extract <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 2;
    }
    const char* seq_label = argv[1];
    const char* fixtures_dir = argv[2];
    const char* dump_dir = argv[3];
    long max_frames = argc > 4 ? atol(argv[4]) : -1;

    char kp_path[4096], desc_path[4096];
    snprintf(kp_path, sizeof(kp_path), "%s/keypoints.tsv", dump_dir);
    snprintf(desc_path, sizeof(desc_path), "%s/descriptors.tsv", dump_dir);
    FILE* kf = fopen(kp_path, "r");
    FILE* df = fopen(desc_path, "r");
    if (!kf || !df) {
        fprintf(stderr, "check_sv_extract: cannot open %s / %s\n", kp_path, desc_path);
        return 2;
    }
    char header[1024];
    if (!fgets(header, sizeof(header), kf) || !fgets(header, sizeof(header), df)) {
        fprintf(stderr, "check_sv_extract: empty tsv header\n");
        return 2;
    }

    sv_keypoint* kps = (sv_keypoint*)malloc(sizeof(sv_keypoint) * SV_CAP);
    unsigned char* descs = (unsigned char*)malloc(32 * SV_CAP);

    long long total = 0, mismatches = 0;
    ref_row pending;
    int have_pending = 0;
    int eof_ref = 0;

    long frame_idx;
    for (frame_idx = 0; max_frames < 0 || frame_idx < max_frames; frame_idx++) {
        char pgm_path[4096];
        snprintf(pgm_path, sizeof(pgm_path), "%s/%06ld.pgm", fixtures_dir, frame_idx);
        int w, h;
        unsigned char* gray = read_pgm(pgm_path, &w, &h);
        if (!gray) break; /* end of sequence */

        int n_got = sv_orb_extract(gray, w, h, &ORB_PARAMS, kps, descs, SV_CAP);
        free(gray);
        if (n_got < 0) {
            fprintf(stderr, "check_sv_extract: frame %ld exceeded cap %d\n", frame_idx, SV_CAP);
            n_got = SV_CAP;
        }

        int i;
        for (i = 0; i < n_got; i++) {
            float ux, uy;
            sv_undistort_point(&CAM, kps[i].x, kps[i].y, &ux, &uy);
            kps[i].x = ux;
            kps[i].y = uy;
        }

        /* Gather this frame's reference rows (aligned lookahead-by-1). */
        int ref_n = 0;
        ref_row* ref_rows = NULL;
        int ref_cap = 0;
        for (;;) {
            if (!have_pending && !eof_ref) {
                have_pending = read_ref_row(kf, df, &pending);
                if (!have_pending) eof_ref = 1;
            }
            if (!have_pending) break;
            if (pending.frame_idx != (int)frame_idx) break;
            if (ref_n == ref_cap) {
                ref_cap = ref_cap ? ref_cap * 2 : 64;
                ref_rows = (ref_row*)realloc(ref_rows, sizeof(ref_row) * (size_t)ref_cap);
            }
            ref_rows[ref_n++] = pending;
            have_pending = 0;
        }

        long long frame_total = ref_n > n_got ? ref_n : n_got;
        long long frame_mismatch = 0;
        if (ref_n != n_got) {
            /* Count mismatch desyncs the whole frame -- every slot counts
             * as a mismatch rather than guessing a partial alignment. */
            frame_mismatch = frame_total;
        }
        else {
            for (i = 0; i < n_got; i++) {
                const ref_row* r = &ref_rows[i];
                if (kps[i].x != r->x || kps[i].y != r->y || kps[i].octave != r->octave ||
                    kps[i].angle != r->angle || 0.f != r->response ||
                    memcmp(descs + (size_t)i * 32, r->desc, 32) != 0) {
                    frame_mismatch++;
                }
            }
        }
        total += frame_total;
        mismatches += frame_mismatch;
        free(ref_rows);

        if (!have_pending && eof_ref && frame_idx > 0) {
            /* No more reference rows at all remain for later frames;
             * still let the fixture loop end naturally on its own (missing
             * pgm) rather than guessing here. */
        }
    }

    fclose(kf);
    fclose(df);
    free(kps);
    free(descs);

    printf("%s: %lld/%lld\n", seq_label, mismatches, total);
    return mismatches == 0 ? 0 : 1;
}
