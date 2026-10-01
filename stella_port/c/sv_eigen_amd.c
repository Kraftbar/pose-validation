/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_eigen_amd.h (MPL-2.0, transliterated from Eigen's Amd.h/Ordering.h). */
#include "sv_eigen_amd.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int amd_flip(int i) { return -i - 2; }

/* clear w */
static int cs_wclear(int mark, int lemax, int* w, int n) {
    int k;
    if (mark < 2 || (mark + lemax < 0)) {
        for (k = 0; k < n; k++) {
            if (w[k] != 0) {
                w[k] = 1;
            }
        }
        mark = 2;
    }
    return mark; /* at this point, w[0..n-1] < mark holds */
}

/* depth-first search and postorder of a tree rooted at node j */
static int cs_tdfs(int j, int k, int* head, const int* next, int* post, int* stack) {
    int i, p, top = 0;
    stack[0] = j; /* place j on the stack */
    while (top >= 0) {
        p = stack[top]; /* p = top of stack */
        i = head[p];    /* i = youngest child of p */
        if (i == -1) {
            top--;         /* p has no unordered children left */
            post[k++] = p; /* node p is the kth postordered node */
        } else {
            head[p] = next[i]; /* remove i from children of p */
            stack[++top] = i;  /* start dfs on child node i */
        }
    }
    return k;
}

static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

/* Eigen's internal::minimum_degree_ordering: C is the complete symmetric
 * pattern (both triangles + diagonal), destroyed. Cp[n+1], Ci[t]. perm has
 * n+1 slots on entry (used as workspace `last`), first n are the result. */
static void minimum_degree_ordering(int n, int* Cp, int* Ci, int cnz, int* perm) {
    int d, dk, dext, lemax = 0, e, elenk, eln, i, j, k, k1, k2, k3, jlast, ln, dense, nzmax, mindeg = 0,
                                nvi, nvj, nvk, mark, wnvi, ok, nel = 0, p, p1, p2, p3, p4, pj, pk, pk1, pk2, pn, q, t, h;

    dense = imax(16, (int)(10 * sqrt((double)n))); /* find dense threshold */
    dense = imin(n - 2, dense);

    t = cnz + cnz / 5 + 2 * n; /* add elbow room to C */

    int* W = (int*)malloc(sizeof(int) * (size_t)(8 * (n + 1)));
    int* len = W;
    int* nv = W + (n + 1);
    int* next = W + 2 * (n + 1);
    int* head = W + 3 * (n + 1);
    int* elen = W + 4 * (n + 1);
    int* degree = W + 5 * (n + 1);
    int* w = W + 6 * (n + 1);
    int* hhead = W + 7 * (n + 1);
    int* last = perm; /* use P as workspace for last */

    /* --- Initialize quotient graph -------------------------------------- */
    for (k = 0; k < n; k++) {
        len[k] = Cp[k + 1] - Cp[k];
    }
    len[n] = 0;
    nzmax = t;

    for (i = 0; i <= n; i++) {
        head[i] = -1; /* degree list i is empty */
        last[i] = -1;
        next[i] = -1;
        hhead[i] = -1;      /* hash list i is empty */
        nv[i] = 1;          /* node i is just one node */
        w[i] = 1;           /* node i is alive */
        elen[i] = 0;        /* Ek of node i is empty */
        degree[i] = len[i]; /* degree of node i */
    }
    mark = cs_wclear(0, 0, w, n); /* clear w */

    /* --- Initialize degree lists ---------------------------------------- */
    for (i = 0; i < n; i++) {
        int has_diag = 0;
        for (p = Cp[i]; p < Cp[i + 1]; ++p) {
            if (Ci[p] == i) {
                has_diag = 1;
                break;
            }
        }

        d = degree[i];
        if (d == 1 && has_diag) { /* node i is empty */
            elen[i] = -2;         /* element i is dead */
            nel++;
            Cp[i] = -1; /* i is a root of assembly tree */
            w[i] = 0;
        } else if (d > dense || !has_diag) { /* node i is dense or has no structural diagonal element */
            nv[i] = 0;                       /* absorb i into element n */
            elen[i] = -1;                    /* node i is dead */
            nel++;
            Cp[i] = amd_flip(n);
            nv[n]++;
        } else {
            if (head[d] != -1) {
                last[head[d]] = i;
            }
            next[i] = head[d]; /* put node i in degree list d */
            head[d] = i;
        }
    }

    elen[n] = -2; /* n is a dead element */
    Cp[n] = -1;   /* n is a root of assembly tree */
    w[n] = 0;     /* n is a dead element */

    while (nel < n) { /* while (selecting pivots) do */
        /* --- Select node of minimum approximate degree ------------------ */
        for (k = -1; mindeg < n && (k = head[mindeg]) == -1; mindeg++) {
        }
        if (next[k] != -1) {
            last[next[k]] = -1;
        }
        head[mindeg] = next[k]; /* remove k from degree list */
        elenk = elen[k];        /* elenk = |Ek| */
        nvk = nv[k];            /* # of nodes k represents */
        nel += nvk;             /* nv[k] nodes of A eliminated */

        /* --- Garbage collection ----------------------------------------- */
        if (elenk > 0 && cnz + mindeg >= nzmax) {
            for (j = 0; j < n; j++) {
                if ((p = Cp[j]) >= 0) { /* j is a live node or element */
                    Cp[j] = Ci[p];      /* save first entry of object */
                    Ci[p] = amd_flip(j); /* first entry is now amd_flip(j) */
                }
            }
            for (q = 0, p = 0; p < cnz;) { /* scan all of memory */
                if ((j = amd_flip(Ci[p++])) >= 0) { /* found object j */
                    Ci[q] = Cp[j]; /* restore first entry of object */
                    Cp[j] = q++;   /* new pointer to object j */
                    for (k3 = 0; k3 < len[j] - 1; k3++) {
                        Ci[q++] = Ci[p++];
                    }
                }
            }
            cnz = q; /* Ci[cnz...nzmax-1] now free */
        }

        /* --- Construct new element -------------------------------------- */
        dk = 0;
        nv[k] = -nvk; /* flag k as in Lk */
        p = Cp[k];
        pk1 = (elenk == 0) ? p : cnz; /* do in place if elen[k] == 0 */
        pk2 = pk1;
        for (k1 = 1; k1 <= elenk + 1; k1++) {
            if (k1 > elenk) {
                e = k;             /* search the nodes in k */
                pj = p;            /* list of nodes starts at Ci[pj]*/
                ln = len[k] - elenk; /* length of list of nodes in k */
            } else {
                e = Ci[p++]; /* search the nodes in e */
                pj = Cp[e];
                ln = len[e]; /* length of list of nodes in e */
            }
            for (k2 = 1; k2 <= ln; k2++) {
                i = Ci[pj++];
                if ((nvi = nv[i]) <= 0) {
                    continue; /* node i dead, or seen */
                }
                dk += nvi;      /* degree[Lk] += size of node i */
                nv[i] = -nvi;   /* negate nv[i] to denote i in Lk*/
                Ci[pk2++] = i;  /* place i in Lk */
                if (next[i] != -1) {
                    last[next[i]] = last[i];
                }
                if (last[i] != -1) { /* remove i from degree list */
                    next[last[i]] = next[i];
                } else {
                    head[degree[i]] = next[i];
                }
            }
            if (e != k) {
                Cp[e] = amd_flip(k); /* absorb e into k */
                w[e] = 0;            /* e is now a dead element */
            }
        }
        if (elenk != 0) {
            cnz = pk2; /* Ci[cnz...nzmax] is free */
        }
        degree[k] = dk;  /* external degree of k - |Lk\i| */
        Cp[k] = pk1;     /* element k is in Ci[pk1..pk2-1] */
        len[k] = pk2 - pk1;
        elen[k] = -2; /* k is now an element */

        /* --- Find set differences --------------------------------------- */
        mark = cs_wclear(mark, lemax, w, n); /* clear w if necessary */
        for (pk = pk1; pk < pk2; pk++) {      /* scan 1: find |Le\Lk| */
            i = Ci[pk];
            if ((eln = elen[i]) <= 0) {
                continue; /* skip if elen[i] empty */
            }
            nvi = -nv[i]; /* nv[i] was negated */
            wnvi = mark - nvi;
            for (p = Cp[i]; p <= Cp[i] + eln - 1; p++) { /* scan Ei */
                e = Ci[p];
                if (w[e] >= mark) {
                    w[e] -= nvi; /* decrement |Le\Lk| */
                } else if (w[e] != 0) { /* ensure e is a live element */
                    w[e] = degree[e] + wnvi; /* 1st time e seen in scan 1 */
                }
            }
        }

        /* --- Degree update ---------------------------------------------- */
        for (pk = pk1; pk < pk2; pk++) { /* scan2: degree update */
            i = Ci[pk];                  /* consider node i in Lk */
            p1 = Cp[i];
            p2 = p1 + elen[i] - 1;
            pn = p1;
            for (h = 0, d = 0, p = p1; p <= p2; p++) { /* scan Ei */
                e = Ci[p];
                if (w[e] != 0) { /* e is an unabsorbed element */
                    dext = w[e] - mark; /* dext = |Le\Lk| */
                    if (dext > 0) {
                        d += dext;   /* sum up the set differences */
                        Ci[pn++] = e; /* keep e in Ei */
                        h += e;       /* compute the hash of node i */
                    } else {
                        Cp[e] = amd_flip(k); /* aggressive absorb. e->k */
                        w[e] = 0;            /* e is a dead element */
                    }
                }
            }
            elen[i] = pn - p1 + 1; /* elen[i] = |Ei| */
            p3 = pn;
            p4 = p1 + len[i];
            for (p = p2 + 1; p < p4; p++) { /* prune edges in Ai */
                j = Ci[p];
                if ((nvj = nv[j]) <= 0) {
                    continue; /* node j dead or in Lk */
                }
                d += nvj;    /* degree(i) += |j| */
                Ci[pn++] = j; /* place j in node list of i */
                h += j;       /* compute hash for node i */
            }
            if (d == 0) { /* check for mass elimination */
                Cp[i] = amd_flip(k); /* absorb i into k */
                nvi = -nv[i];
                dk -= nvi; /* |Lk| -= |i| */
                nvk += nvi; /* |k| += nv[i] */
                nel += nvi;
                nv[i] = 0;
                elen[i] = -1; /* node i is dead */
            } else {
                degree[i] = imin(degree[i], d); /* update degree(i) */
                Ci[pn] = Ci[p3];                /* move first node to end */
                Ci[p3] = Ci[p1];                /* move 1st el. to end of Ei */
                Ci[p1] = k;                     /* add k as 1st element in of Ei */
                len[i] = pn - p1 + 1;           /* new len of adj. list of node i */
                h %= n;                         /* finalize hash of i */
                next[i] = hhead[h];             /* place i in hash bucket */
                hhead[h] = i;
                last[i] = h; /* save hash of i in last[i] */
            }
        } /* scan2 is done */
        degree[k] = dk; /* finalize |Lk| */
        lemax = imax(lemax, dk);
        mark = cs_wclear(mark + lemax, lemax, w, n); /* clear w */

        /* --- Supernode detection ---------------------------------------- */
        for (pk = pk1; pk < pk2; pk++) {
            i = Ci[pk];
            if (nv[i] >= 0) {
                continue; /* skip if i is dead */
            }
            h = last[i]; /* scan hash bucket of node i */
            i = hhead[h];
            hhead[h] = -1; /* hash bucket will be empty */
            for (; i != -1 && next[i] != -1; i = next[i], mark++) {
                ln = len[i];
                eln = elen[i];
                for (p = Cp[i] + 1; p <= Cp[i] + ln - 1; p++) {
                    w[Ci[p]] = mark;
                }
                jlast = i;
                for (j = next[i]; j != -1;) { /* compare i with all j */
                    ok = (len[j] == ln) && (elen[j] == eln);
                    for (p = Cp[j] + 1; ok && p <= Cp[j] + ln - 1; p++) {
                        if (w[Ci[p]] != mark) {
                            ok = 0; /* compare i and j*/
                        }
                    }
                    if (ok) { /* i and j are identical */
                        Cp[j] = amd_flip(i); /* absorb j into i */
                        nv[i] += nv[j];
                        nv[j] = 0;
                        elen[j] = -1; /* node j is dead */
                        j = next[j];  /* delete j from hash bucket */
                        next[jlast] = j;
                    } else {
                        jlast = j; /* j and i are different */
                        j = next[j];
                    }
                }
            }
        }

        /* --- Finalize new element---------------------------------------- */
        for (p = pk1, pk = pk1; pk < pk2; pk++) { /* finalize Lk */
            i = Ci[pk];
            if ((nvi = -nv[i]) <= 0) {
                continue; /* skip if i is dead */
            }
            nv[i] = nvi;              /* restore nv[i] */
            d = degree[i] + dk - nvi; /* compute external degree(i) */
            d = imin(d, n - nel - nvi);
            if (head[d] != -1) {
                last[head[d]] = i;
            }
            next[i] = head[d]; /* put i back in degree list */
            last[i] = -1;
            head[d] = i;
            mindeg = imin(mindeg, d); /* find new minimum degree */
            degree[i] = d;
            Ci[p++] = i; /* place i in Lk */
        }
        nv[k] = nvk; /* # nodes absorbed into k */
        if ((len[k] = p - pk1) == 0) { /* length of adj list of element k*/
            Cp[k] = -1;                /* k is a root of the tree */
            w[k] = 0;                  /* k is now a dead element */
        }
        if (elenk != 0) {
            cnz = p; /* free unused space in Lk */
        }
    }

    /* --- Postordering ------------------------------------------------------ */
    for (i = 0; i < n; i++) {
        Cp[i] = amd_flip(Cp[i]); /* fix assembly tree */
    }
    for (j = 0; j <= n; j++) {
        head[j] = -1;
    }
    for (j = n; j >= 0; j--) { /* place unordered nodes in lists */
        if (nv[j] > 0) {
            continue; /* skip if j is an element */
        }
        next[j] = head[Cp[j]]; /* place j in list of its parent */
        head[Cp[j]] = j;
    }
    for (e = n; e >= 0; e--) { /* place elements in lists */
        if (nv[e] <= 0) {
            continue; /* skip unless e is an element */
        }
        if (Cp[e] != -1) {
            next[e] = head[Cp[e]]; /* place e in list of its parent */
            head[Cp[e]] = e;
        }
    }
    for (k = 0, i = 0; i <= n; i++) { /* postorder the assembly tree */
        if (Cp[i] == -1) {
            k = cs_tdfs(i, k, head, next, perm, w);
        }
    }
    free(W);
}

static int cmp_int(const void* a, const void* b) {
    int x = *(const int*)a, y = *(const int*)b;
    return (x > y) - (x < y);
}

int sv_amd_order(int n, const int* Ap, const int* Ai, int* perm) {
    int i, j, p;
    if (n <= 0) {
        return 0;
    }
    const int nnz = Ap[n];

    /* Sorted copy of A (column-major, sorted rows) and of A^T. */
    int* As_p = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    int* As_i = (int*)malloc(sizeof(int) * (size_t)(nnz > 0 ? nnz : 1));
    memcpy(As_p, Ap, sizeof(int) * (size_t)(n + 1));
    memcpy(As_i, Ai, sizeof(int) * (size_t)nnz);
    for (j = 0; j < n; ++j) {
        qsort(As_i + As_p[j], (size_t)(As_p[j + 1] - As_p[j]), sizeof(int), cmp_int);
    }
    int* T_p = (int*)calloc((size_t)(n + 2), sizeof(int));
    int* T_i = (int*)malloc(sizeof(int) * (size_t)(nnz > 0 ? nnz : 1));
    for (p = 0; p < nnz; ++p) {
        T_p[As_i[p] + 1]++;
    }
    for (i = 0; i < n; ++i) {
        T_p[i + 1] += T_p[i];
    }
    {
        int* fill = (int*)malloc(sizeof(int) * (size_t)(n + 1));
        memcpy(fill, T_p, sizeof(int) * (size_t)(n + 1));
        for (j = 0; j < n; ++j) {
            for (p = As_p[j]; p < As_p[j + 1]; ++p) {
                T_i[fill[As_i[p]]++] = j; /* row j of column As_i[p] of A^T ascending in j */
            }
        }
        free(fill);
    }

    /* symm = A^T + A pattern (union of sorted lists), t = cnz + cnz/5 + 2n slots. */
    int cnz = 0;
    for (j = 0; j < n; ++j) {
        int a = As_p[j], b = T_p[j];
        const int ae = As_p[j + 1], be = T_p[j + 1];
        while (a < ae || b < be) {
            if (a < ae && b < be) {
                if (As_i[a] == T_i[b]) {
                    ++a;
                    ++b;
                } else if (As_i[a] < T_i[b]) {
                    ++a;
                } else {
                    ++b;
                }
            } else if (a < ae) {
                ++a;
            } else {
                ++b;
            }
            ++cnz;
        }
    }
    const int t = cnz + cnz / 5 + 2 * n;
    int* Cp = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    int* Ci = (int*)malloc(sizeof(int) * (size_t)(t > 0 ? t : 1));
    int q = 0;
    for (j = 0; j < n; ++j) {
        int a = As_p[j], b = T_p[j];
        const int ae = As_p[j + 1], be = T_p[j + 1];
        Cp[j] = q;
        while (a < ae || b < be) {
            if (a < ae && b < be) {
                if (As_i[a] == T_i[b]) {
                    Ci[q++] = As_i[a];
                    ++a;
                    ++b;
                } else if (As_i[a] < T_i[b]) {
                    Ci[q++] = As_i[a++];
                } else {
                    Ci[q++] = T_i[b++];
                }
            } else if (a < ae) {
                Ci[q++] = As_i[a++];
            } else {
                Ci[q++] = T_i[b++];
            }
        }
    }
    Cp[n] = q;

    int* perm_ws = (int*)malloc(sizeof(int) * (size_t)(n + 1));
    minimum_degree_ordering(n, Cp, Ci, cnz, perm_ws);
    memcpy(perm, perm_ws, sizeof(int) * (size_t)n);

    free(perm_ws);
    free(Ci);
    free(Cp);
    free(T_i);
    free(T_p);
    free(As_i);
    free(As_p);
    return 0;
}

void sv_pose_optimizer_amd_order(int n, int perm[]) {
    int i;
    for (i = 0; i < n; ++i) {
        perm[i] = i;
    }
}
