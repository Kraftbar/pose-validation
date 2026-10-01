/* SV_PORT_SOURCES: check_sv_bow_db.c sv_bow_db.c sv_bow.c
 * SPDX-License-Identifier: MIT
 *
 * Observer protocol in reference_bow_db/README.md; expected output comes
 * from the real installed stella library, not from a model of this C port. */
#include "sv_bow_db.h"
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { sv_bow_db_keyframe key; sv_bow_vector vector; } fixture_key;
static uint32_t bits(float f) { uint32_t u; memcpy(&u, &f, sizeof(u)); return u; }
static fixture_key *lookup(fixture_key **keys, size_t n, uint32_t id)
{
    for (size_t i = 0; i < n; ++i) if (keys[i]->key.id == id) return keys[i];
    return NULL;
}
static int replay(const char *path, FILE *out)
{
    FILE *in = fopen(path, "r");
    sv_bow_db *db = sv_bow_db_create();
    fixture_key **keys = NULL;
    size_t n_keys = 0, cap = 0, queries = 0;
    sv_bow_db_result result = {0};
    const sv_bow_db_keyframe **reject = NULL;
    int status = 2;
    char op;
    if (!in || !db) goto done;
    while (fscanf(in, " %c", &op) == 1) {
        uint32_t id, n;
        if (op == 'K') {
            if (fscanf(in, "%"SCNu32" %"SCNu32, &id, &n) != 2 || n > 1000000 || lookup(keys,n_keys,id)) goto done;
            if (n_keys == cap) {
                size_t next = cap ? cap*2 : 16;
                fixture_key **p = realloc(keys, next*sizeof(*p));
                if (!p) goto done;
                keys = p; cap = next;
            }
            fixture_key *k = calloc(1, sizeof(*k));
            if (!k) goto done;
            keys[n_keys++] = k;
            k->key.id = id; k->key.bow = &k->vector; k->vector.count = n;
            k->vector.words = n ? calloc(n, sizeof(sv_bow_word)) : NULL;
            if (n && !k->vector.words) goto done;
            for (uint32_t i = 0; i < n; ++i)
                if (fscanf(in, "%"SCNu32" %a", &k->vector.words[i].word_id, &k->vector.words[i].weight) != 2) goto done;
        }
        else if (op == 'A' || op == 'E') {
            if (fscanf(in, "%"SCNu32, &id) != 1) goto done;
            fixture_key *k = lookup(keys,n_keys,id);
            if (!k) goto done;
            if (op == 'A' ? sv_bow_db_add(db,&k->key) : sv_bow_db_erase(db,&k->key)) goto done;
        }
        else if (op == 'C') sv_bow_db_clear(db);
        else if (op == 'S') {
            size_t nw = sv_bow_db_num_words(db);
            fprintf(out, "S %zu\n", nw);
            for (size_t i = 0; i < nw; ++i) {
                sv_bow_db_word_view v;
                if (sv_bow_db_word_at(db,i,&v)) goto done;
                fprintf(out, "W %"PRIu32" %zu", v.word_id, v.count);
                for (size_t j = 0; j < v.count; ++j) fprintf(out, " %"PRIu32, v.keyframes[j]->id);
                fputc('\n',out);
            }
        }
        else if (op == 'Q') {
            uint32_t qid; float minimum, ratio;
            if (fscanf(in, "%"SCNu32" %"SCNu32" %a %a %"SCNu32, &qid, &id, &minimum, &ratio, &n) != 5 || n > 1000000) goto done;
            fixture_key *k = lookup(keys,n_keys,id);
            if (!k) goto done;
            reject = n ? calloc(n, sizeof(*reject)) : NULL;
            if (n && !reject) goto done;
            for (uint32_t i = 0; i < n; ++i) {
                uint32_t rid;
                if (fscanf(in, "%"SCNu32, &rid) != 1) goto done;
                fixture_key *r = lookup(keys,n_keys,rid);
                if (!r) goto done;
                reject[i] = &r->key;
            }
            if (sv_bow_db_query(db,&k->vector,minimum,ratio,reject,n,&result)) goto done;
            free(reject); reject = NULL;
            fprintf(out, "Q %"PRIu32" %"PRIu32" %"PRIu32" %08"PRIx32" %zu %zu\n",
                qid,result.max_common_words,result.min_common_words,bits(result.best_score),result.count,result.accepted_count);
            for (size_t i = 0; i < result.count; ++i) {
                sv_bow_db_match *m = &result.matches[i];
                fprintf(out,"R %"PRIu32" %"PRIu32" %d %08"PRIx32" %d\n",
                    m->keyframe->id,m->common_words,m->scored,bits(m->score),m->accepted);
            }
            ++queries;
        }
        else goto done;
    }
    if (queries && !ferror(in) && !ferror(out)) status = 0;
done:
    if (status) fprintf(stderr,"invalid/incomplete fixture: %s\n",path);
    if (in) fclose(in);
    free(reject); sv_bow_db_result_free(&result); sv_bow_db_destroy(db);
    for (size_t i = 0; i < n_keys; ++i) { sv_bow_vector_free(&keys[i]->vector); free(keys[i]); }
    free(keys); return status;
}

static int selftest(void)
{
    unsigned tests = 0, bad = 0;
#define CHECK(x) do { ++tests; if (!(x)) { ++bad; fprintf(stderr,"API check failed at line %d\n",__LINE__); } } while (0)
    sv_bow_db *db = sv_bow_db_create();
    if (!db) return 2;
    sv_bow_word words[] = {{9,1}}, unsorted[] = {{2,.5f},{1,.5f}};
    sv_bow_vector v = {words,1}, invalid = {unsorted,2};
    sv_bow_db_keyframe key = {12,&v}, alias = {12,&v}, wrong = {13,&invalid};
    sv_bow_db_result r = {0}; sv_bow_db_word_view view;
    CHECK(!sv_bow_db_query(db,&v,0,.8f,NULL,0,&r) && r.count == 0);
    CHECK(sv_bow_db_add(db,&wrong) == -1 && sv_bow_db_num_words(db) == 0);
    CHECK(!sv_bow_db_add(db,&key) && !sv_bow_db_add(db,&key));
    CHECK(!sv_bow_db_query(db,&v,0,0,NULL,0,&r) && r.count == 1 && r.matches[0].common_words == 2);
    CHECK(!sv_bow_db_erase(db,&alias));
    CHECK(!sv_bow_db_query(db,&v,0,0,NULL,0,&r) && r.matches[0].common_words == 1);
    const sv_bow_db_keyframe *reject[] = {&alias};
    CHECK(!sv_bow_db_query(db,&v,0,0,reject,1,&r) && r.count == 1);
    reject[0] = &key;
    CHECK(!sv_bow_db_query(db,&v,0,0,reject,1,&r) && r.count == 0);
    CHECK(!sv_bow_db_query(db,&v,1,0,NULL,0,&r) && r.accepted_count == 1);
    CHECK(sv_bow_db_query(db,&v,0,-1,NULL,0,&r) == -1 && r.accepted_count == 1);
    CHECK(sv_bow_db_query(db,&v,0,NAN,NULL,0,&r) == -1);
    CHECK(sv_bow_db_query(db,&v,INFINITY,0,NULL,0,&r) == -1);
    CHECK(sv_bow_db_query(db,&v,0,FLT_MAX,NULL,0,&r) == -1);
    CHECK(sv_bow_db_query(db,&invalid,0,0,NULL,0,&r) == -1);
    CHECK(sv_bow_db_query(db,&v,0,0,NULL,1,&r) == -1);
    CHECK(!sv_bow_db_query(db,&v,0,1,NULL,0,&r) && r.accepted_count == 0);
    CHECK(!sv_bow_db_erase(db,&key) && !sv_bow_db_erase(db,&key));
    CHECK(sv_bow_db_num_words(db) == 1 && !sv_bow_db_word_at(db,0,&view) && view.count == 0);
    CHECK(sv_bow_db_word_at(db,1,&view) == -1);
    sv_bow_db_clear(db);
    CHECK(sv_bow_db_num_words(db) == 0);
    sv_bow_db_result_free(&r); sv_bow_db_destroy(db);
    printf("api: %u/%u\n",bad,tests);
    return bad ? 1 : 0;
#undef CHECK
}

int main(int argc, char **argv)
{
    if (argc == 2 && !strcmp(argv[1],"--selftest")) return selftest();
    char commands[4096], expected[4096], label[256];
    FILE *actual = NULL;
    if (argc == 6 && !strcmp(argv[1],"--case")) {
        snprintf(label,sizeof(label),"%s",argv[2]);
        snprintf(commands,sizeof(commands),"%s",argv[3]);
        snprintf(expected,sizeof(expected),"%s",argv[4]);
        actual = fopen(argv[5],"w+");
    }
    else if (argc == 4 || argc == 5) {
        /* Existing shared-runner ABI. This leaf checks the full lifecycle
         * fixture even when a max-frames diagnostic argument is supplied. */
        char base[3072]; snprintf(base,sizeof(base),"%s",argv[3]);
        char *p = strrchr(base,'/'); if (!p) return 2; *p = 0;
        p = strrchr(base,'/'); if (!p) return 2; *p = 0;
        snprintf(label,sizeof(label),"%s",argv[1]);
        snprintf(commands,sizeof(commands),"%s/bow_db/fixtures/%s/commands.txt",base,label);
        snprintf(expected,sizeof(expected),"%s/bow_db/fixtures/%s/expected.tsv",base,label);
        actual = tmpfile();
    }
    else { fprintf(stderr,"usage: check --case label commands expected actual | label fixtures dump [max_frames] | --selftest\n"); return 2; }
    if (!actual) return 2;
    int status = replay(commands,actual);
    if (status) { fclose(actual); return status; }
    rewind(actual);
    FILE *want = fopen(expected,"r");
    if (!want) { fclose(actual); return 2; }
    char a[4096], b[4096]; size_t total = 0, bad = 0;
    for (;;) {
        char *pa = fgets(a,sizeof(a),actual), *pb = fgets(b,sizeof(b),want);
        if (!pa && !pb) break;
        ++total;
        if (!pa || !pb || strcmp(a,b)) {
            if (bad++ < 5) fprintf(stderr,"line %zu got: %swant: %s",total,pa?a:"<EOF>\n",pb?b:"<EOF>\n");
        }
    }
    int io = ferror(actual) || ferror(want);
    fclose(actual); fclose(want);
    printf("%s: %zu/%zu\n",label,bad,total);
    return io || !total ? 2 : bad ? 1 : 0;
}
