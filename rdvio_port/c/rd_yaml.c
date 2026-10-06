/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port: the YAML subset of the RD-VIO configs (block mappings by indentation, `- ` sequences, flow `[...]` / `{...}`
 * spanning lines, `#` comments, the `%YAML` line). Original code of this project (the same reader as okvis_port/c/ok_config.c). */
#include "rd_yaml.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ YAML subset -> tree */
enum { Y_SCALAR, Y_SEQ, Y_MAP };
typedef struct yn {
    int kind;
    char* s;                    /* scalar text (trimmed) */
    int n, cap;
    char** keys;                /* map keys */
    struct yn** v;              /* seq items / map values */
} yn;

typedef struct ytext { char* t; size_t n, pos; } ytext;

static yn* yn_new(int kind) { yn* y = (yn*)calloc(1, sizeof *y); y->kind = kind; return y; }
static void yn_add(yn* y, char* key, yn* v) {
    if (y->n == y->cap) {
        y->cap = y->cap ? 2 * y->cap : 8;
        y->v = (yn**)realloc(y->v, sizeof(yn*) * (size_t)y->cap);
        y->keys = (char**)realloc(y->keys, sizeof(char*) * (size_t)y->cap);
    }
    y->keys[y->n] = key; y->v[y->n] = v; y->n++;
}
static void yn_free(yn* y) {
    int i;
    if (!y) return;
    for (i = 0; i < y->n; ++i) { free(y->keys[i]); yn_free(y->v[i]); }
    free(y->keys); free(y->v); free(y->s); free(y);
}
static char* trimdup(const char* a, const char* b) {
    char* s;
    while (a < b && isspace((unsigned char)*a)) a++;
    while (b > a && isspace((unsigned char)b[-1])) b--;
    s = (char*)malloc((size_t)(b - a) + 1);
    memcpy(s, a, (size_t)(b - a)); s[b - a] = 0;
    return s;
}
static yn* scalar(const char* a, const char* b) { yn* y = yn_new(Y_SCALAR); y->s = trimdup(a, b); return y; }

static int at_end(const ytext* x) { return x->pos >= x->n; }
static char cur(const ytext* x) { return at_end(x) ? 0 : x->t[x->pos]; }
static void skip_sp(ytext* x) { while (!at_end(x) && (cur(x) == ' ' || cur(x) == '\t' || cur(x) == '\r')) x->pos++; }
static void skip_ws(ytext* x) { while (!at_end(x) && isspace((unsigned char)cur(x))) x->pos++; }
static void to_eol(ytext* x) { while (!at_end(x) && cur(x) != '\n') x->pos++; if (!at_end(x)) x->pos++; }
/* the next line with content: returns its indentation (-1 at the end) and leaves pos at its start */
static int next_line(ytext* x) {
    for (;;) {
        size_t p = x->pos;
        int ind = 0;
        if (at_end(x)) return -1;
        while (p < x->n && (x->t[p] == ' ' || x->t[p] == '\t')) { p++; ind++; }
        if (p < x->n && x->t[p] != '\n' && x->t[p] != '\r') return ind;
        while (p < x->n && x->t[p] != '\n') p++;
        x->pos = p < x->n ? p + 1 : p;
    }
}

static yn* parse_flow(ytext* x) {
    skip_ws(x);
    if (cur(x) == '[') {
        yn* y = yn_new(Y_SEQ);
        x->pos++;
        for (;;) {
            skip_ws(x);
            if (at_end(x)) break;
            if (cur(x) == ']') { x->pos++; break; }
            yn_add(y, NULL, parse_flow(x));
            skip_ws(x);
            if (cur(x) == ',') x->pos++;
        }
        return y;
    }
    if (cur(x) == '{') {
        yn* y = yn_new(Y_MAP);
        x->pos++;
        for (;;) {
            size_t k0;
            skip_ws(x);
            if (at_end(x)) break;
            if (cur(x) == '}') { x->pos++; break; }
            k0 = x->pos;
            while (!at_end(x) && cur(x) != ':' && cur(x) != '}') x->pos++;
            if (cur(x) != ':') break;
            { char* key = trimdup(x->t + k0, x->t + x->pos); x->pos++; yn_add(y, key, parse_flow(x)); }
            skip_ws(x);
            if (cur(x) == ',') x->pos++;
        }
        return y;
    }
    {
        const size_t a = x->pos;
        while (!at_end(x) && cur(x) != ',' && cur(x) != ']' && cur(x) != '}' && cur(x) != '\n') x->pos++;
        return scalar(x->t + a, x->t + x->pos);
    }
}

/* value after "key:" or "- " on the current line */
static yn* parse_block(ytext* x, int ind);
static yn* parse_value(ytext* x, int ind) {
    skip_sp(x);
    if (at_end(x) || cur(x) == '\n') {
        int ci;
        to_eol(x);
        ci = next_line(x);
        if (ci > ind) {
            const char c0 = x->t[x->pos + (size_t)ci];
            if (c0 == '[' || c0 == '{') { yn* y; x->pos += (size_t)ci; y = parse_flow(x); to_eol(x); return y; }
            return parse_block(x, ci);
        }
        return scalar("", "");
    }
    if (cur(x) == '[' || cur(x) == '{') { yn* y = parse_flow(x); to_eol(x); return y; }
    { const size_t a = x->pos; while (!at_end(x) && cur(x) != '\n') x->pos++; { yn* y = scalar(x->t + a, x->t + x->pos); to_eol(x); return y; } }
}

static yn* parse_block(ytext* x, int ind) {
    yn* y = NULL;
    for (;;) {
        const int li = next_line(x);
        if (li != ind) break;
        x->pos += (size_t)li;
        if (cur(x) == '-' && (x->pos + 1 >= x->n || isspace((unsigned char)x->t[x->pos + 1]))) {
            if (!y) y = yn_new(Y_SEQ);
            if (y->kind != Y_SEQ) break;
            x->pos++;
            yn_add(y, NULL, parse_value(x, ind));
        } else {
            const size_t k0 = x->pos;
            if (!y) y = yn_new(Y_MAP);
            if (y->kind != Y_MAP) break;
            while (!at_end(x) && cur(x) != ':' && cur(x) != '\n') x->pos++;
            if (cur(x) != ':') { to_eol(x); continue; }
            { char* key = trimdup(x->t + k0, x->t + x->pos); x->pos++; yn_add(y, key, parse_value(x, ind)); }
        }
    }
    return y ? y : scalar("", "");
}

static yn* yparse_file(const char* path) {
    FILE* f = fopen(path, "rb");
    ytext x;
    size_t i;
    int line_start = 1;
    yn* root;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); x.n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    x.t = (char*)malloc(x.n + 1);
    if (fread(x.t, 1, x.n, f) != x.n) { fclose(f); free(x.t); return NULL; }
    fclose(f);
    x.t[x.n] = 0;
    /* blank out comments and the %YAML directive */
    for (i = 0; i < x.n; ++i) {
        const char c = x.t[i];
        if (c == '\n') { line_start = 1; continue; }
        if ((c == '#' && (line_start || isspace((unsigned char)x.t[i - 1]) || x.t[i - 1] == ',')) || (c == '%' && line_start)) {
            while (i < x.n && x.t[i] != '\n') x.t[i++] = ' ';
            line_start = 1;
            continue;
        }
        if (!isspace((unsigned char)c)) line_start = 0;
    }
    x.pos = 0;
    root = parse_block(&x, next_line(&x) < 0 ? 0 : next_line(&x));
    free(x.t);
    return root;
}

static const yn* yget(const yn* y, const char* key) {
    int i;
    if (!y || y->kind != Y_MAP) return NULL;
    for (i = 0; i < y->n; ++i) if (!strcmp(y->keys[i], key)) return y->v[i];
    return NULL;
}

/* ------------------------------------------------------------------ public API */
void* rd_yaml_load(const char* path) { return yparse_file(path); }
void rd_yaml_free(void* root) { yn_free((yn*)root); }
/* "a.b.c": nested mapping keys; NULL if absent */
const void* rd_yaml_find(const void* root, const char* dotted) {
    const yn* y = (const yn*)root;
    char key[128];
    const char* s = dotted;
    while (y && *s) {
        size_t n = strcspn(s, ".");
        if (n >= sizeof key) return NULL;
        memcpy(key, s, n); key[n] = 0;
        y = yget(y, key);
        s += n;
        if (*s == '.') s++;
    }
    return y;
}
int rd_yaml_seq_len(const void* node) { const yn* y = (const yn*)node; return y && y->kind == Y_SEQ ? y->n : -1; }
const void* rd_yaml_seq_at(const void* node, int i) { const yn* y = (const yn*)node; return y && y->kind == Y_SEQ && i >= 0 && i < y->n ? y->v[i] : NULL; }
const char* rd_yaml_scalar(const void* node) { const yn* y = (const yn*)node; return y && y->kind == Y_SCALAR ? y->s : NULL; }
