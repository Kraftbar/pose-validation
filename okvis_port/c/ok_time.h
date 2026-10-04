/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * OKVIS2 pure-C port, module 2a: okvis::Time / okvis::Duration (okvis_time).
 *
 * Derived from OKVIS2 (okvis_time/include/okvis/{Time,Duration}.hpp, implementation, src):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The semantics derive from ROS time
 *   (Copyright (c) 2008, Willow Garage, Inc., BSD). Redistribution requires retaining these notices; the names
 *   of ETH Zurich, Imperial College London, TUM and Willow Garage may not be used to endorse derived products.
 *
 * C99, <stdint.h> <math.h> only. No clock access (Time::now / sleep are not ported: the port never reads wall
 * time). The C++ code throws std::runtime_error when a value leaves the 32-bit range; here the corresponding
 * functions return -1 (and leave the result untouched).
 *
 * Quirks of the original that are kept bit-for-bit: Time::fromSec does not renormalise (nsec may become 1e9),
 * Duration::fromSec for a negative whole number gives sec = floor(d)+1 and a negative nsec, Duration
 * normalisation carries only when nsec > 1e9 (strictly).
 */
#ifndef OK_TIME_H
#define OK_TIME_H

#include <stdint.h>

typedef struct ok_time { uint32_t sec, nsec; } ok_time;           /* okvis::Time */
typedef struct ok_duration { int32_t sec, nsec; } ok_duration;    /* okvis::Duration */

/* ---- normalisation helpers (Time.cpp / Duration.cpp); return 0 or -1 on 32-bit overflow ("throws") ---- */
int ok_normalize_sec_nsec_u64(uint64_t* sec, uint64_t* nsec);
int ok_normalize_sec_nsec_u32(uint32_t* sec, uint32_t* nsec);
int ok_normalize_sec_nsec_unsigned_i64(int64_t* sec, int64_t* nsec);
int ok_normalize_sec_nsec_signed_i64(int64_t* sec, int64_t* nsec);
int ok_normalize_sec_nsec_signed_i32(int32_t* sec, int32_t* nsec);

/* ---- Time ---- */
ok_time ok_time_make(uint32_t sec, uint32_t nsec);                /* Time(sec, nsec): normalised */
ok_time ok_time_from_sec(double t);                               /* Time(double) / fromSec */
double ok_time_to_sec(ok_time t);
ok_time ok_time_from_nsec(uint64_t t);                            /* fromNSec */
uint64_t ok_time_to_nsec(ok_time t);
int ok_time_is_zero(ok_time t);
ok_duration ok_time_sub(ok_time a, ok_time b);                    /* a - b (Duration), carry in the ctor */
int ok_time_add(ok_time a, ok_duration d, ok_time* out);          /* a + d, -1 if out of range */
int ok_time_sub_duration(ok_time a, ok_duration d, ok_time* out); /* a - d = a + (-d) */
int ok_time_lt(ok_time a, ok_time b);
int ok_time_gt(ok_time a, ok_time b);
int ok_time_le(ok_time a, ok_time b);
int ok_time_ge(ok_time a, ok_time b);
int ok_time_eq(ok_time a, ok_time b);
/* (a - b).toSec() as used throughout ImuError */
double ok_time_diff_sec(ok_time a, ok_time b);

/* ---- Duration ---- */
ok_duration ok_duration_make(int32_t sec, int32_t nsec);          /* Duration(sec, nsec): signed-normalised */
ok_duration ok_duration_from_sec(double t);                       /* Duration(double) / fromSec */
double ok_duration_to_sec(ok_duration d);
ok_duration ok_duration_from_nsec(int64_t t);
int64_t ok_duration_to_nsec(ok_duration d);
ok_duration ok_duration_add(ok_duration a, ok_duration b);
ok_duration ok_duration_sub(ok_duration a, ok_duration b);
ok_duration ok_duration_neg(ok_duration a);
ok_duration ok_duration_mul(ok_duration a, double scale);         /* Duration(toSec()*scale) */
int ok_duration_lt(ok_duration a, ok_duration b);
int ok_duration_gt(ok_duration a, ok_duration b);
int ok_duration_le(ok_duration a, ok_duration b);
int ok_duration_ge(ok_duration a, ok_duration b);
int ok_duration_eq(ok_duration a, ok_duration b);
int ok_duration_is_zero(ok_duration d);

#endif
