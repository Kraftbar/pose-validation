/* SV_PORT_SOURCES: check_sv_rng.c sv_rng.c
 * SPDX-License-Identifier: MIT
 *
 * Standalone harness: compares sv_rng.c against
 * runs/stella_port/reference_init/rng_fixture.txt, produced by
 * stella_port/reference_tools/dump_rng.cc from the real std::mt19937 /
 * std::uniform_int_distribution / std::shuffle (GCC 13 libstdc++) that
 * stella_vslam's util::random_array.cc uses with use_fixed_seed=true.
 * The RNG fixture is not per-sequence, but this harness still follows
 * tools/check_stella_port.py's shared calling convention
 *   <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (so the shared runner's discover-and-run-every-check_*.c loop works
 * unmodified) -- fixtures_dir/dump_dir/max_frames are accepted and
 * ignored except to locate the fixture file, which lives at
 * <root>/runs/stella_port/reference_init/rng_fixture.txt where <root> is
 * recovered from fixtures_dir (".../runs/stella_port/fixtures/<seq>").
 * Prints one line "<seq_label>: <mismatches>/<total>" (raw engine draws +
 * create_random_array draws combined); exits 0 iff mismatches==0.
 */
#include "sv_rng.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fixture_path_from_fixtures_dir(const char* fixtures_dir, char* out, size_t out_cap) {
    const char* marker = "/fixtures/";
    const char* p = strstr(fixtures_dir, marker);
    if (p) {
        size_t root_len = (size_t)(p - fixtures_dir);
        snprintf(out, out_cap, "%.*s/reference_init/rng_fixture.txt", (int)root_len, fixtures_dir);
    }
    else {
        /* fallback: fixtures_dir itself is/under stella_port's runs root */
        snprintf(out, out_cap, "%s/../reference_init/rng_fixture.txt", fixtures_dir);
    }
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n", argv[0]);
        return 2;
    }
    const char* seq_label = argv[1];
    const char* fixtures_dir = argv[2];
    char fixture_path[4096];
    fixture_path_from_fixtures_dir(fixtures_dir, fixture_path, sizeof(fixture_path));

    FILE* f = fopen(fixture_path, "r");
    if (!f) {
        fprintf(stderr, "cannot open %s\n", fixture_path);
        return 2;
    }

    long raw_mismatches = 0, raw_total = 0;
    long arr_mismatches = 0, arr_total = 0;

    char tag[8];
    while (fscanf(f, "%7s", tag) == 1) {
        if (tag[0] == 'r' && tag[1] == 'a' && tag[2] == 'w') {
            int n;
            if (fscanf(f, "%d", &n) != 1) break;
            sv_mt19937 e;
            sv_mt19937_init_default(&e);
            for (int i = 0; i < n; ++i) {
                unsigned long expect;
                if (fscanf(f, "%lu", &expect) != 1) { fprintf(stderr, "raw: truncated fixture\n"); return 2; }
                uint32_t got = sv_mt19937_next(&e);
                raw_total++;
                if (got != (uint32_t)expect) {
                    raw_mismatches++;
                }
            }
        }
        else if (tag[0] == 'a' && tag[1] == 'r' && tag[2] == 'r') {
            unsigned int size, rand_max;
            int reps;
            if (fscanf(f, "%u %u %d", &size, &rand_max, &reps) != 3) break;
            sv_mt19937 e;
            sv_mt19937_init_default(&e);
            uint32_t* out = (uint32_t*)malloc(sizeof(uint32_t) * size);
            for (int r = 0; r < reps; ++r) {
                sv_create_random_array(size, 0U, rand_max, &e, out);
                for (unsigned int i = 0; i < size; ++i) {
                    unsigned long expect;
                    if (fscanf(f, "%lu", &expect) != 1) { fprintf(stderr, "arr: truncated fixture\n"); return 2; }
                    arr_total++;
                    if (out[i] != (uint32_t)expect) {
                        arr_mismatches++;
                    }
                }
            }
            free(out);
        }
        else {
            fprintf(stderr, "unknown tag %s\n", tag);
            return 2;
        }
    }
    fclose(f);

    long mismatches = raw_mismatches + arr_mismatches;
    long total = raw_total + arr_total;
    printf("%s: %ld/%ld\n", seq_label, mismatches, total);

    return mismatches == 0 ? 0 : 1;
}
