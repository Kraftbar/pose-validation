/* SPDX-License-Identifier: BSD-3-Clause */
/* OKVIS2 pure-C port, module 7d (part 1): DBoW2 vocabulary / database / query. See ok_dbow.h for the notices. */
#include "ok_dbow.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct rd { const unsigned char* p; size_t n, off; int bad; } rd;
static uint32_t ru32(rd* r) { uint32_t v = 0; if (r->off + 4 > r->n) { r->bad = 1; return 0; } memcpy(&v, r->p + r->off, 4); r->off += 4; return v; }
static double rf64(rd* r) { double v = 0; if (r->off + 8 > r->n) { r->bad = 1; return 0; } memcpy(&v, r->p + r->off, 8); r->off += 8; return v; }

int ok_dbow_voc_parse(ok_dbow_voc* v, const unsigned char* p, size_t n) {
    rd r; uint32_t i, c;
    memset(v, 0, sizeof *v);
    r.p = p; r.n = n; r.off = 0; r.bad = 0;
    v->k = (int)ru32(&r); v->L = (int)ru32(&r); v->weighting = (int)ru32(&r); v->scoring = (int)ru32(&r);
    v->nnodes = (int)ru32(&r); v->nwords = (int)ru32(&r);
    if (r.bad || v->nnodes <= 0 || v->nnodes > (1 << 24)) return 1;
    v->nodes = (ok_dbow_node*)calloc((size_t)v->nnodes, sizeof(ok_dbow_node));
    for (i = 0; i < (uint32_t)v->nnodes; ++i) {
        ok_dbow_node* nd = &v->nodes[i];
        uint32_t desclen;
        nd->id = ru32(&r); nd->parent = ru32(&r); nd->word_id = ru32(&r); nd->weight = rf64(&r);
        nd->nchildren = (int)ru32(&r);
        if (r.bad || nd->nchildren < 0 || nd->nchildren > v->nnodes) { ok_dbow_voc_free(v); return 1; }
        nd->children = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(nd->nchildren ? nd->nchildren : 1));
        for (c = 0; c < (uint32_t)nd->nchildren; ++c) nd->children[c] = ru32(&r);
        desclen = ru32(&r);
        if (r.bad || r.off + desclen > r.n || desclen > 48) { ok_dbow_voc_free(v); return 1; }
        memcpy(nd->desc, r.p + r.off, desclen); r.off += desclen;
    }
    if (r.bad) { ok_dbow_voc_free(v); return 1; }
    return 0;
}
int ok_dbow_voc_load(ok_dbow_voc* v, const char* path) {
    FILE* f = fopen(path, "rb");
    unsigned char* buf; long n; int rc;
    if (!f) return 1;
    fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
    buf = (unsigned char*)malloc((size_t)(n > 0 ? n : 1));
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(buf); return 1; }
    fclose(f);
    rc = ok_dbow_voc_parse(v, buf, (size_t)n);
    free(buf);
    return rc;
}
void ok_dbow_voc_free(ok_dbow_voc* v) {
    int i;
    if (!v->nodes) return;
    for (i = 0; i < v->nnodes; ++i) free(v->nodes[i].children);
    free(v->nodes);
    memset(v, 0, sizeof *v);
}

/* brisk::Hamming::PopcntofXORed(a, b, 3) over the 48 bytes, as double (FBrisk::distance) */
static double distance48(const unsigned char* a, const unsigned char* b) {
    int i, n = 0;
    for (i = 0; i < 48; ++i) { unsigned x = (unsigned)(a[i] ^ b[i]); while (x) { n += (int)(x & 1u); x >>= 1; } }
    return (double)n;
}

/* TemplatedVocabulary::transform(feature, word_id, weight): descend to the nearest child at every level (first minimum) */
static void transform_one(const ok_dbow_voc* v, const unsigned char* f, uint32_t* word_id, double* weight) {
    uint32_t final_id = 0;
    do {
        const ok_dbow_node* cur = &v->nodes[final_id];
        double best_d;
        int c;
        final_id = cur->children[0];
        best_d = distance48(f, v->nodes[final_id].desc);
        for (c = 1; c < cur->nchildren; ++c) {
            const uint32_t id = cur->children[c];
            const double d = distance48(f, v->nodes[id].desc);
            if (d < best_d) { best_d = d; final_id = id; }
        }
    } while (v->nodes[final_id].nchildren != 0);
    *word_id = v->nodes[final_id].word_id;
    *weight = v->nodes[final_id].weight;
}

void ok_dbow_transform(const ok_dbow_voc* v, const unsigned char* feat, int nfeat, ok_dbow_bow* out) {
    double* acc = (double*)calloc((size_t)(v->nwords ? v->nwords : 1), sizeof(double));
    unsigned char* present = (unsigned char*)calloc((size_t)(v->nwords ? v->nwords : 1), 1);
    int i, n = 0, j;
    double norm = 0.0;
    memset(out, 0, sizeof *out);
    for (i = 0; i < nfeat; ++i) {
        uint32_t id; double w;
        transform_one(v, feat + 48 * (size_t)i, &id, &w);
        if (w > 0) {                                         /* BowVector::addWeight: the first insertion stores w, then += */
            if (present[id]) acc[id] += w; else { acc[id] = w; present[id] = 1; ++n; }
        }
    }
    out->n = n;
    out->id = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(n ? n : 1));
    out->w = (double*)malloc(sizeof(double) * (size_t)(n ? n : 1));
    for (i = 0, j = 0; i < v->nwords; ++i) if (present[i]) { out->id[j] = (uint32_t)i; out->w[j] = acc[i]; ++j; }
    for (j = 0; j < n; ++j) norm += fabs(out->w[j]);          /* normalize(L1) */
    if (norm > 0.0) for (j = 0; j < n; ++j) out->w[j] /= norm;
    free(acc); free(present);
}
void ok_dbow_bow_free(ok_dbow_bow* b) { free(b->id); free(b->w); memset(b, 0, sizeof *b); }

void ok_dbow_db_init(ok_dbow_db* db, const ok_dbow_voc* v) {
    memset(db, 0, sizeof *db);
    db->voc = v;
    db->rows = (ok_dbow_row*)calloc((size_t)(v->nwords ? v->nwords : 1), sizeof(ok_dbow_row));
}
void ok_dbow_db_free(ok_dbow_db* db) {
    int i;
    for (i = 0; i < db->voc->nwords; ++i) { free(db->rows[i].entry); free(db->rows[i].weight); }
    free(db->rows); free(db->pose_ids);
    memset(db, 0, sizeof *db);
}
int ok_dbow_db_add(ok_dbow_db* db, const unsigned char* feat, int nfeat, uint64_t pose_id) {
    ok_dbow_bow v;
    int i;
    const int entry = db->nentries++;
    ok_dbow_transform(db->voc, feat, nfeat, &v);
    for (i = 0; i < v.n; ++i) {
        ok_dbow_row* row = &db->rows[v.id[i]];
        if (row->n == row->cap) {
            row->cap = row->cap ? 2 * row->cap : 8;
            row->entry = (uint32_t*)realloc(row->entry, sizeof(uint32_t) * (size_t)row->cap);
            row->weight = (double*)realloc(row->weight, sizeof(double) * (size_t)row->cap);
        }
        row->entry[row->n] = (uint32_t)entry; row->weight[row->n] = v.w[i]; ++row->n;
    }
    ok_dbow_bow_free(&v);
    if (db->npose == db->cappose) { db->cappose = db->cappose ? 2 * db->cappose : 64; db->pose_ids = (uint64_t*)realloc(db->pose_ids, sizeof(uint64_t) * (size_t)db->cappose); }
    db->pose_ids[db->npose++] = pose_id;
    return entry;
}

/* ---- std::sort(first, last) with operator< = Score < (introsort: median-of-3 + Hoare partition + insertion sort below 16) ---- */
static int lt(const ok_dbow_result* a, const ok_dbow_result* b) { return a->score < b->score; }
static void swap_r(ok_dbow_result* a, ok_dbow_result* b) { ok_dbow_result t = *a; *a = *b; *b = t; }
static void insertion_sort(ok_dbow_result* first, ok_dbow_result* last) {
    ok_dbow_result* i;
    if (first == last) return;
    for (i = first + 1; i != last; ++i) {
        if (lt(i, first)) {
            const ok_dbow_result val = *i;
            ok_dbow_result* q;
            for (q = i; q != first; --q) *q = *(q - 1);
            *first = val;
        } else {
            const ok_dbow_result val = *i;
            ok_dbow_result* cur = i;
            ok_dbow_result* nx = i - 1;
            while (lt(&val, nx)) { *cur = *nx; cur = nx; --nx; }
            *cur = val;
        }
    }
}
static void unguarded_insertion_sort(ok_dbow_result* first, ok_dbow_result* last) {
    ok_dbow_result* i;
    for (i = first; i != last; ++i) {
        const ok_dbow_result val = *i;
        ok_dbow_result* cur = i;
        ok_dbow_result* nx = i - 1;
        while (lt(&val, nx)) { *cur = *nx; cur = nx; --nx; }
        *cur = val;
    }
}
static void heap_sift(ok_dbow_result* a, long start, long end, ok_dbow_result v) {   /* max-heap on operator< */
    long top = start, child = start;
    while (child < (end - 1) / 2) {
        child = 2 * (child + 1);
        if (lt(&a[child], &a[child - 1])) --child;
        a[start] = a[child]; start = child;
    }
    if ((end & 1) == 0 && child == (end - 2) / 2) { child = 2 * (child + 1); a[start] = a[child - 1]; start = child - 1; }
    while (start > top) { const long parent = (start - 1) / 2; if (!lt(&a[parent], &v)) break; a[start] = a[parent]; start = parent; }
    a[start] = v;
}
static void heap_sort(ok_dbow_result* first, ok_dbow_result* last) {            /* make_heap + sort_heap (depth limit fallback, never reached in practice) */
    long n = (long)(last - first), i;
    if (n < 2) return;
    for (i = (n - 2) / 2; ; --i) { heap_sift(first, i, n, first[i]); if (i == 0) break; }
    while (last - first > 1) { ok_dbow_result v; --last; v = *last; *last = *first; heap_sift(first, 0, (long)(last - first), v); }
}
static void move_median_to_first(ok_dbow_result* result, ok_dbow_result* a, ok_dbow_result* b, ok_dbow_result* c) {
    if (lt(a, b)) {
        if (lt(b, c)) swap_r(result, b); else if (lt(a, c)) swap_r(result, c); else swap_r(result, a);
    } else if (lt(a, c)) swap_r(result, a);
    else if (lt(b, c)) swap_r(result, c);
    else swap_r(result, b);
}
static ok_dbow_result* unguarded_partition(ok_dbow_result* first, ok_dbow_result* last, ok_dbow_result* pivot) {
    for (;;) {
        while (lt(first, pivot)) ++first;
        --last;
        while (lt(pivot, last)) --last;
        if (!(first < last)) return first;
        swap_r(first, last);
        ++first;
    }
}
static void introsort_loop(ok_dbow_result* first, ok_dbow_result* last, long depth_limit) {
    while (last - first > 16) {
        ok_dbow_result* mid; ok_dbow_result* cut;
        if (depth_limit == 0) { heap_sort(first, last); return; }
        --depth_limit;
        mid = first + (last - first) / 2;
        move_median_to_first(first, first + 1, mid, last - 1);
        cut = unguarded_partition(first + 1, last, first);
        introsort_loop(cut, last, depth_limit);
        last = cut;
    }
}
static void std_sort_results(ok_dbow_result* first, ok_dbow_result* last) {
    long n = (long)(last - first), lg = 0;
    if (first == last) return;
    while (n > 1) { n >>= 1; ++lg; }
    introsort_loop(first, last, lg * 2);
    if (last - first > 16) { insertion_sort(first, first + 16); unguarded_insertion_sort(first + 16, last); }
    else insertion_sort(first, last);
}

void ok_dbow_sort_results(ok_dbow_result* r, int n) { std_sort_results(r, r + n); }

void ok_dbow_query(const ok_dbow_db* db, const ok_dbow_bow* q, ok_dbow_result** out, int* nout) {
    double* pairs = (double*)calloc((size_t)(db->nentries ? db->nentries : 1), sizeof(double));
    unsigned char* seen = (unsigned char*)calloc((size_t)(db->nentries ? db->nentries : 1), 1);
    ok_dbow_result* ret;
    int i, r, n = 0, j;
    for (i = 0; i < q->n; ++i) {
        const double qvalue = q->w[i];
        const ok_dbow_row* row = &db->rows[q->id[i]];
        for (r = 0; r < row->n; ++r) {
            const uint32_t entry = row->entry[r];
            const double dvalue = row->weight[r];
            const double value = fabs(qvalue - dvalue) - fabs(qvalue) - fabs(dvalue);
            if (seen[entry]) pairs[entry] += value; else { pairs[entry] = value; seen[entry] = 1; ++n; }
        }
    }
    ret = (ok_dbow_result*)malloc(sizeof(ok_dbow_result) * (size_t)(n ? n : 1));
    for (i = 0, j = 0; i < db->nentries; ++i) if (seen[i]) { ret[j].id = (uint32_t)i; ret[j].score = pairs[i]; ++j; }
    std_sort_results(ret, ret + n);
    for (j = 0; j < n; ++j) ret[j].score = -ret[j].score / 2.0;
    free(pairs); free(seen);
    *out = ret; *nout = n;
}

void ok_dbow_filtered(const ok_dbow_db* db, const ok_dbow_result* orig, int norig, uint64_t** state_ids, double** scores, int* nout) {
    /* dBoWResult: the same results sorted ascending by Id (a std::sort on distinct ids) */
    ok_dbow_result* byid = (ok_dbow_result*)malloc(sizeof(ok_dbow_result) * (size_t)(norig > 0 ? norig : 1));
    unsigned char* suppressed = (unsigned char*)calloc((size_t)(norig > 0 ? norig : 1), 1);
    unsigned char* used = (unsigned char*)calloc((size_t)(norig > 0 ? norig : 1), 1);
    int f, a, i, j, n = 0;
    const int nonmax_radius = 5;
    memcpy(byid, orig, sizeof(ok_dbow_result) * (size_t)norig);
    for (i = 1; i < norig; ++i) {                              /* ids are distinct: any correct sort gives the same order */
        const ok_dbow_result v = byid[i];
        for (j = i - 1; j >= 0 && byid[j].id > v.id; --j) byid[j + 1] = byid[j];
        byid[j + 1] = v;
    }
    for (f = 0; f < norig; ++f) {
        const double score = orig[f].score;
        const uint64_t id = orig[f].id;
        int is_max = 1, lo, hi;
        if (id >= (uint64_t)norig) continue;
        if (score < 0.4) break;
        if (suppressed[f]) continue;                            /* suppressedIds.count(f): the index in score order (as upstream) */
        lo = (int)id - nonmax_radius; if (lo < 0) lo = 0;
        hi = (int)id + nonmax_radius; if (hi > norig - 1) hi = norig - 1;
        for (a = lo; a <= hi; ++a) if (byid[a].score > score) is_max = 0;
        if (!is_max) continue;
        for (a = lo; a <= hi; ++a) suppressed[a] = 1;
        used[id] = 1;
    }
    *state_ids = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(norig > 0 ? norig : 1));
    *scores = (double*)malloc(sizeof(double) * (size_t)(norig > 0 ? norig : 1));
    for (i = 0; i < norig; ++i)                                  /* `for (size_t id : ids)`: dBoWResult.at(id), poseIds.at(id) */
        if (used[i]) { (*state_ids)[n] = db->pose_ids[i]; (*scores)[n] = byid[i].score; ++n; }
    *nout = n;
    free(byid); free(suppressed); free(used);
}
