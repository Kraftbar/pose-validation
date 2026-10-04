/* SPDX-License-Identifier: BSD-3-Clause */
/* OKVIS2 pure-C port, module 2a: Time / Duration. See ok_time.h for notices. */
#include "ok_time.h"

#include <math.h>

#define OK_INT_MIN (-2147483647 - 1)
#define OK_INT_MAX 2147483647

int ok_normalize_sec_nsec_u64(uint64_t* sec, uint64_t* nsec) {
    const uint64_t nsec_part = *nsec % 1000000000UL;
    const uint64_t sec_part = *nsec / 1000000000UL;
    if (sec_part > 4294967295U) return -1; /* UINT_MAX */
    *sec += sec_part;
    *nsec = nsec_part;
    return 0;
}

int ok_normalize_sec_nsec_u32(uint32_t* sec, uint32_t* nsec) {
    uint64_t sec64 = *sec, nsec64 = *nsec;
    if (ok_normalize_sec_nsec_u64(&sec64, &nsec64)) return -1;
    *sec = (uint32_t)sec64;
    *nsec = (uint32_t)nsec64;
    return 0;
}

int ok_normalize_sec_nsec_unsigned_i64(int64_t* sec, int64_t* nsec) {
    int64_t nsec_part = *nsec, sec_part = *sec;
    while (nsec_part >= 1000000000L) { nsec_part -= 1000000000L; ++sec_part; }
    while (nsec_part < 0) { nsec_part += 1000000000L; --sec_part; }
    if (sec_part < 0 || sec_part > OK_INT_MAX) return -1;
    *sec = sec_part;
    *nsec = nsec_part;
    return 0;
}

int ok_normalize_sec_nsec_signed_i64(int64_t* sec, int64_t* nsec) {
    int64_t nsec_part = *nsec, sec_part = *sec;
    while (nsec_part > 1000000000L) { nsec_part -= 1000000000L; ++sec_part; } /* sic: '>' not '>=' */
    while (nsec_part < 0) { nsec_part += 1000000000L; --sec_part; }
    if (sec_part < OK_INT_MIN || sec_part > OK_INT_MAX) return -1;
    *sec = sec_part;
    *nsec = nsec_part;
    return 0;
}

int ok_normalize_sec_nsec_signed_i32(int32_t* sec, int32_t* nsec) {
    int64_t sec64 = *sec, nsec64 = *nsec;
    if (ok_normalize_sec_nsec_signed_i64(&sec64, &nsec64)) return -1;
    *sec = (int32_t)sec64;
    *nsec = (int32_t)nsec64;
    return 0;
}

/* ------------------------------------------------ Time ------------------------------------------------ */

ok_time ok_time_make(uint32_t sec, uint32_t nsec) {
    ok_time t;
    t.sec = sec; t.nsec = nsec;
    ok_normalize_sec_nsec_u32(&t.sec, &t.nsec);
    return t;
}

ok_time ok_time_from_sec(double t) {
    ok_time r;
    r.sec = (uint32_t)floor(t);
    r.nsec = (uint32_t)round((t - (double)r.sec) * 1e9);
    return r;
}

double ok_time_to_sec(ok_time t) { return (double)t.sec + 1e-9 * (double)t.nsec; }

ok_time ok_time_from_nsec(uint64_t t) {
    ok_time r;
    r.sec = (uint32_t)(int32_t)(t / 1000000000);   /* int32_t(t / 1000000000) */
    r.nsec = (uint32_t)(int32_t)(t % 1000000000);
    ok_normalize_sec_nsec_u32(&r.sec, &r.nsec);
    return r;
}

uint64_t ok_time_to_nsec(ok_time t) { return (uint64_t)t.sec * 1000000000ull + (uint64_t)t.nsec; }
int ok_time_is_zero(ok_time t) { return t.sec == 0 && t.nsec == 0; }

ok_duration ok_time_sub(ok_time a, ok_time b) {
    return ok_duration_make((int32_t)a.sec - (int32_t)b.sec, (int32_t)a.nsec - (int32_t)b.nsec);
}

int ok_time_add(ok_time a, ok_duration d, ok_time* out) {
    int64_t sec_sum = (int64_t)a.sec + (int64_t)d.sec;
    int64_t nsec_sum = (int64_t)a.nsec + (int64_t)d.nsec;
    if (ok_normalize_sec_nsec_unsigned_i64(&sec_sum, &nsec_sum)) return -1;
    *out = ok_time_make((uint32_t)sec_sum, (uint32_t)nsec_sum); /* T(uint32, uint32): ctor normalises (a no-op here) */
    return 0;
}

int ok_time_sub_duration(ok_time a, ok_duration d, ok_time* out) {
    return ok_time_add(a, ok_duration_neg(d), out);
}

int ok_time_lt(ok_time a, ok_time b) { return a.sec < b.sec || (a.sec == b.sec && a.nsec < b.nsec); }
int ok_time_gt(ok_time a, ok_time b) { return a.sec > b.sec || (a.sec == b.sec && a.nsec > b.nsec); }
int ok_time_le(ok_time a, ok_time b) { return a.sec < b.sec || (a.sec == b.sec && a.nsec <= b.nsec); }
int ok_time_ge(ok_time a, ok_time b) { return a.sec > b.sec || (a.sec == b.sec && a.nsec >= b.nsec); }
int ok_time_eq(ok_time a, ok_time b) { return a.sec == b.sec && a.nsec == b.nsec; }

double ok_time_diff_sec(ok_time a, ok_time b) { return ok_duration_to_sec(ok_time_sub(a, b)); }

/* ---------------------------------------------- Duration ---------------------------------------------- */

ok_duration ok_duration_make(int32_t sec, int32_t nsec) {
    ok_duration d;
    d.sec = sec; d.nsec = nsec;
    ok_normalize_sec_nsec_signed_i32(&d.sec, &d.nsec);
    return d;
}

ok_duration ok_duration_from_sec(double t) {
    ok_duration d;
    if (t >= 0.0) d.sec = (int32_t)floor(t);
    else d.sec = (int32_t)floor(t) + 1;               /* HAVE_TRUNC undefined */
    d.nsec = (int32_t)((t - (double)d.sec) * 1000000000);
    return d;
}

double ok_duration_to_sec(ok_duration d) { return (double)d.sec + 1e-9 * (double)d.nsec; }

ok_duration ok_duration_from_nsec(int64_t t) {
    ok_duration d;
    d.sec = (int32_t)(t / 1000000000);
    d.nsec = (int32_t)(t % 1000000000);
    ok_normalize_sec_nsec_signed_i32(&d.sec, &d.nsec);
    return d;
}

int64_t ok_duration_to_nsec(ok_duration d) { return (int64_t)d.sec * 1000000000ll + (int64_t)d.nsec; }

ok_duration ok_duration_add(ok_duration a, ok_duration b) { return ok_duration_make(a.sec + b.sec, a.nsec + b.nsec); }
ok_duration ok_duration_sub(ok_duration a, ok_duration b) { return ok_duration_make(a.sec - b.sec, a.nsec - b.nsec); }
ok_duration ok_duration_neg(ok_duration a) { return ok_duration_make(-a.sec, -a.nsec); }
ok_duration ok_duration_mul(ok_duration a, double scale) { return ok_duration_from_sec(ok_duration_to_sec(a) * scale); }
int ok_duration_lt(ok_duration a, ok_duration b) { return a.sec < b.sec || (a.sec == b.sec && a.nsec < b.nsec); }
int ok_duration_gt(ok_duration a, ok_duration b) { return a.sec > b.sec || (a.sec == b.sec && a.nsec > b.nsec); }
int ok_duration_le(ok_duration a, ok_duration b) { return a.sec < b.sec || (a.sec == b.sec && a.nsec <= b.nsec); }
int ok_duration_ge(ok_duration a, ok_duration b) { return a.sec > b.sec || (a.sec == b.sec && a.nsec >= b.nsec); }
int ok_duration_eq(ok_duration a, ok_duration b) { return a.sec == b.sec && a.nsec == b.nsec; }
int ok_duration_is_zero(ok_duration d) { return d.sec == 0 && d.nsec == 0; }
