/* SV_PORT_SOURCES: check_sv_bow.c sv_bow.c
 * SPDX-License-Identifier: MIT
 *
 * Harness: loads the real ORB vocabulary (external/candidates/orb_vocab.fbow,
 * read here -- sv_bow.c itself is stdio-free, per the module-2 brief -- and
 * handed to sv_bow_load_memory() as a memory buffer), then for EVERY frame
 * of the sequence (the reference tool,
 * stella_port/reference_tools/dump_frame_bow.cc, calls
 * fbow::Vocabulary::transform() explicitly per frame now, independent of
 * whichever frames stella's own tracking happens to call
 * frame::compute_bow() on -- see that file's header comment) reads that
 * frame's descriptors from module 1's own validated
 * runs/stella_port/reference_dumps/<seq>/descriptors.tsv, runs
 * sv_bow_transform() at level 4 (stella's fixed level, see
 * data/bow_vocabulary_util.cc), and compares the resulting bow_vec /
 * bow_feat_vec, in container (ascending id) order, against
 * bow_vec.tsv/bow_feat.tsv.
 *
 * Also compares sv_bow_score() (fbow::BoWVector::score) against score.tsv,
 * which the reference tool computes for every (i, i+1) consecutive pair and
 * every (i, i+50) far-apart pair, over the same per-frame bow vectors --
 * compared as exact double bit patterns (parsed from score.tsv's %a hex
 * column, which round-trips a double exactly).
 *
 * Module-2 reference dumps live in a sibling directory to the module-1
 * dumps this harness is invoked with (see check_sv_frame.c's header
 * comment for why/how that path is derived).
 *
 * Usage: check_sv_bow <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (fixtures_dir unused, kept for check_stella_port.py's fixed CLI shape.)
 * Prints "<seq_label>: <mismatches>/<total>\n"; exits 0 iff mismatches==0.
 */
#include "sv_bow.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 65536
#define MAX_KPTS_PER_FRAME 20000

static char* derive_frame_bow_dir(const char* dump_dir) {
    const char* needle = "reference_dumps";
    const char* p = strstr(dump_dir, needle);
    if (!p) return NULL;
    size_t prefix_len = (size_t)(p - dump_dir);
    size_t needle_len = strlen(needle);
    const char* suffix = p + needle_len;
    const char* repl = "reference_frame_bow";
    size_t out_len = prefix_len + strlen(repl) + strlen(suffix);
    char* out = (char*)malloc(out_len + 1);
    memcpy(out, dump_dir, prefix_len);
    memcpy(out + prefix_len, repl, strlen(repl));
    memcpy(out + prefix_len + strlen(repl), suffix, strlen(suffix) + 1);
    return out;
}

static uint8_t* read_whole_file(const char* path, size_t* len_out) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    if (len < 0) { fclose(f); return NULL; }
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = (uint8_t*)malloc((size_t)len);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)len, f);
    fclose(f);
    if (got != (size_t)len) { free(buf); return NULL; }
    *len_out = (size_t)len;
    return buf;
}

/* --- reference bow_vec.tsv / bow_feat.tsv, grouped by frame ------------ */
typedef struct { uint32_t word_id; float weight; } ref_word;
typedef struct { uint32_t node_id; uint32_t* idx; uint32_t count; } ref_node;

typedef struct {
    int frame_idx;
    ref_word* words; uint32_t n_words;
    ref_node* nodes; uint32_t n_nodes;
} ref_frame;

static ref_frame* g_frames = NULL;
static int g_n_frames = 0, g_cap_frames = 0;

static ref_frame* find_or_add_frame(int frame_idx) {
    int i;
    for (i = 0; i < g_n_frames; i++) {
        if (g_frames[i].frame_idx == frame_idx) return &g_frames[i];
    }
    if (g_n_frames == g_cap_frames) {
        g_cap_frames = g_cap_frames ? g_cap_frames * 2 : 64;
        g_frames = (ref_frame*)realloc(g_frames, sizeof(ref_frame) * (size_t)g_cap_frames);
    }
    ref_frame* fr = &g_frames[g_n_frames++];
    memset(fr, 0, sizeof(*fr));
    fr->frame_idx = frame_idx;
    return fr;
}

static int cmp_frame_idx(const void* a, const void* b) {
    int x = ((const ref_frame*)a)->frame_idx;
    int y = ((const ref_frame*)b)->frame_idx;
    return (x > y) - (x < y);
}

static void load_bow_vec(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return;
    char line[1024];
    fgets(line, sizeof(line), f); /* header */
    int frame_idx; unsigned int word_id; double dummy; char hex[64];
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "%d\t%u\t%lf\t%63s", &frame_idx, &word_id, &dummy, hex) != 4) continue;
        ref_frame* fr = find_or_add_frame(frame_idx);
        fr->words = (ref_word*)realloc(fr->words, sizeof(ref_word) * (fr->n_words + 1));
        fr->words[fr->n_words].word_id = word_id;
        fr->words[fr->n_words].weight = strtof(hex, NULL);
        fr->n_words++;
    }
    fclose(f);
}

typedef struct { int frame_a, frame_b; double score; } ref_score;
static ref_score* g_scores = NULL;
static int g_n_scores = 0, g_cap_scores = 0;

static void load_scores(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return;
    char line[256];
    fgets(line, sizeof(line), f); /* header */
    int a, b; double dummy; char hex[64];
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "%d\t%d\t%lf\t%63s", &a, &b, &dummy, hex) != 4) continue;
        if (g_n_scores == g_cap_scores) {
            g_cap_scores = g_cap_scores ? g_cap_scores * 2 : 256;
            g_scores = (ref_score*)realloc(g_scores, sizeof(ref_score) * (size_t)g_cap_scores);
        }
        g_scores[g_n_scores].frame_a = a;
        g_scores[g_n_scores].frame_b = b;
        g_scores[g_n_scores].score = strtod(hex, NULL);
        g_n_scores++;
    }
    fclose(f);
}

static void load_bow_feat(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return;
    char line[MAX_LINE];
    fgets(line, sizeof(line), f); /* header */
    while (fgets(line, sizeof(line), f)) {
        int frame_idx; unsigned int node_id; char csv[MAX_LINE];
        csv[0] = '\0';
        int n = sscanf(line, "%d\t%u\t%65535[^\n]", &frame_idx, &node_id, csv);
        if (n < 2) continue;
        ref_frame* fr = find_or_add_frame(frame_idx);
        fr->nodes = (ref_node*)realloc(fr->nodes, sizeof(ref_node) * (fr->n_nodes + 1));
        ref_node* rn = &fr->nodes[fr->n_nodes++];
        rn->node_id = node_id;
        rn->idx = NULL;
        rn->count = 0;
        const char* p = csv;
        while (*p) {
            char* end;
            unsigned long v = strtoul(p, &end, 10);
            if (end == p) break;
            rn->idx = (uint32_t*)realloc(rn->idx, sizeof(uint32_t) * (rn->count + 1));
            rn->idx[rn->count++] = (uint32_t)v;
            p = end;
            if (*p == ',') p++;
        }
    }
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_bow <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 2;
    }
    const char* seq_label = argv[1];
    const char* dump_dir = argv[3];
    long max_frames = argc > 4 ? atol(argv[4]) : -1;
    (void)max_frames; /* frames to test are selected by the reference dump, not a raw frame cap */

    char* frame_bow_dir = derive_frame_bow_dir(dump_dir);
    if (!frame_bow_dir) {
        fprintf(stderr, "check_sv_bow: could not derive frame_bow dir from %s\n", dump_dir);
        return 2;
    }
    char bowvec_path[4096], bowfeat_path[4096], score_path[4096], desc_path[4096];
    snprintf(bowvec_path, sizeof(bowvec_path), "%s/bow_vec.tsv", frame_bow_dir);
    snprintf(bowfeat_path, sizeof(bowfeat_path), "%s/bow_feat.tsv", frame_bow_dir);
    snprintf(score_path, sizeof(score_path), "%s/score.tsv", frame_bow_dir);
    snprintf(desc_path, sizeof(desc_path), "%s/descriptors.tsv", dump_dir);
    free(frame_bow_dir);

    load_bow_vec(bowvec_path);
    load_bow_feat(bowfeat_path);
    load_scores(score_path);
    qsort(g_frames, (size_t)g_n_frames, sizeof(ref_frame), cmp_frame_idx);

    if (max_frames >= 0) {
        int i, kept = 0;
        for (i = 0; i < g_n_frames; i++) {
            if (g_frames[i].frame_idx < max_frames) g_frames[kept++] = g_frames[i];
        }
        g_n_frames = kept;
    }

    if (g_n_frames == 0) {
        printf("%s: 0/0\n", seq_label);
        return 0;
    }

    size_t vocab_len;
    uint8_t* vocab_buf = read_whole_file("external/candidates/orb_vocab.fbow", &vocab_len);
    if (!vocab_buf) {
        fprintf(stderr, "check_sv_bow: cannot read external/candidates/orb_vocab.fbow "
                        "(run from the repo root)\n");
        return 2;
    }
    sv_bow_vocab vocab;
    if (sv_bow_load_memory(vocab_buf, vocab_len, &vocab) != 0) {
        fprintf(stderr, "check_sv_bow: sv_bow_load_memory failed\n");
        return 2;
    }

    FILE* df = fopen(desc_path, "r");
    if (!df) {
        fprintf(stderr, "check_sv_bow: cannot open %s\n", desc_path);
        return 2;
    }
    char line[256];
    fgets(line, sizeof(line), df); /* header */

    uint8_t* descs = (uint8_t*)malloc(32 * MAX_KPTS_PER_FRAME);

    /* Keep every frame's computed BoW vector around (indexed by frame_idx)
     * so the score.tsv pass below can re-score without recomputing. */
    int max_frame_idx = g_n_frames ? g_frames[g_n_frames - 1].frame_idx : -1;
    int s;
    for (s = 0; s < g_n_scores; s++) {
        if (g_scores[s].frame_b > max_frame_idx) max_frame_idx = g_scores[s].frame_b;
    }
    sv_bow_vector* bv_store = (sv_bow_vector*)calloc((size_t)(max_frame_idx + 1), sizeof(sv_bow_vector));

    long long total = 0, mismatches = 0;
    int fi; /* index into g_frames (sorted ascending) */

    /* Streaming cursor over descriptors.tsv, persistent across target
     * frames: `have_row` caches one already-read-but-not-yet-consumed row
     * (the first row of the frame group *after* the one just finished). */
    int have_row = 0;   /* 1 = row_frame/row_kp/row_hex hold a pending row, -1 = EOF reached */
    int row_frame = -1, row_kp; char row_hex[80];
    int group_valid = 0; /* 1 once a full frame group has been read into descs/cur_n/cur_frame */
    int cur_frame = -1, cur_n = 0;

    for (fi = 0; fi < g_n_frames; fi++) {
        int target = g_frames[fi].frame_idx;

        /* Advance through descriptors.tsv, one whole frame group at a
         * time, until we land on `target` (frame groups are visited in
         * ascending frame_idx order, same as g_frames) or run out. */
        while (!(group_valid && cur_frame == target) && have_row != -1) {
            cur_n = 0;
            cur_frame = -1;
            for (;;) {
                if (!have_row) {
                    if (fscanf(df, "%d\t%d\t%79s\n", &row_frame, &row_kp, row_hex) != 3) { have_row = -1; break; }
                    have_row = 1;
                }
                if (cur_frame == -1) cur_frame = row_frame;
                if (row_frame != cur_frame) break; /* next frame's row is now pending */
                if (cur_n < MAX_KPTS_PER_FRAME) {
                    int b;
                    for (b = 0; b < 32; b++) {
                        unsigned int hi, lo;
                        hi = (row_hex[b * 2] <= '9') ? (unsigned)(row_hex[b * 2] - '0') : (unsigned)(row_hex[b * 2] - 'a' + 10);
                        lo = (row_hex[b * 2 + 1] <= '9') ? (unsigned)(row_hex[b * 2 + 1] - '0') : (unsigned)(row_hex[b * 2 + 1] - 'a' + 10);
                        descs[(size_t)cur_n * 32 + b] = (uint8_t)((hi << 4) | lo);
                    }
                    cur_n++;
                }
                have_row = 0;
            }
            group_valid = (cur_frame != -1);
        }

        if (group_valid && cur_frame == target) {
            group_valid = 0; /* consumed; force a fresh group read for the next target */
            sv_bow_vector bv;
            sv_bow_feat_vector bf;
            sv_bow_transform(&vocab, descs, (unsigned int)cur_n, 4, &bv, &bf);

            ref_frame* rf = &g_frames[fi];

            total++;
            int ok = 1;
            if (bv.count != rf->n_words) {
                ok = 0;
            }
            else {
                uint32_t w;
                for (w = 0; w < bv.count; w++) {
                    if (bv.words[w].word_id != rf->words[w].word_id ||
                        bv.words[w].weight != rf->words[w].weight) {
                        ok = 0;
                        break;
                    }
                }
            }
            total++;
            int feat_ok = 1;
            if (bf.count != rf->n_nodes) {
                feat_ok = 0;
            }
            else {
                uint32_t n;
                for (n = 0; n < bf.count; n++) {
                    if (bf.nodes[n].node_id != rf->nodes[n].node_id ||
                        bf.nodes[n].count != rf->nodes[n].count) {
                        feat_ok = 0;
                        break;
                    }
                    uint32_t k;
                    for (k = 0; k < bf.nodes[n].count; k++) {
                        if (bf.nodes[n].kp_indices[k] != rf->nodes[n].idx[k]) {
                            feat_ok = 0;
                            break;
                        }
                    }
                    if (!feat_ok) break;
                }
            }

            if (!ok) mismatches++;
            if (!feat_ok) mismatches++;

            /* Keep bv (do not free) -- reused by the score.tsv pass below. */
            bv_store[target] = bv;
            sv_bow_feat_vector_free(&bf);
        }
        else {
            /* target frame_idx never appeared in descriptors.tsv (should
             * not happen -- every frame stella tracks has descriptors) */
            total++;
            mismatches++;
        }
    }

    /* --- score.tsv: sv_bow_score() against every (i,i+1)/(i,i+50) pair,
     * compared as exact double bit patterns. --- */
    for (s = 0; s < g_n_scores; s++) {
        int a = g_scores[s].frame_a, b = g_scores[s].frame_b;
        total++;
        /* bv_store[] is zero-initialized (calloc), so a frame with no
         * words (never written to by the loop above -- either because it
         * had 0 descriptors or was skipped) is naturally an empty
         * sv_bow_vector, which sv_bow_score() handles like FBoW's own
         * empty-map case. */
        double computed = sv_bow_score(&bv_store[a], &bv_store[b]);
        if (computed != g_scores[s].score) {
            mismatches++;
        }
    }

    int fidx;
    for (fidx = 0; fidx <= max_frame_idx; fidx++) {
        sv_bow_vector_free(&bv_store[fidx]);
    }
    free(bv_store);
    free(descs);
    free(vocab_buf);
    fclose(df);

    printf("%s: %lld/%lld\n", seq_label, mismatches, total);
    return mismatches == 0 ? 0 : 1;
}
