/* SPDX-License-Identifier: MIT
 * Standalone C99 PNG -> OpenCV 4.6 IMREAD_GRAYSCALE-compatible pixels.
 * See ../reference_png/README.md for limits, metadata semantics and validation.
 */
#ifndef OK_PNG_H
#define OK_PNG_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Success is zero. Caller owns *out and must free() it. On every error valid
 * output arguments are reset to NULL/0/0. Output is tightly packed, row-major.
 * Thread-safe; no globals, external libraries or caller-provided allocator.
 */
enum { OK_PNG_OK = 0, OK_PNG_INVALID = 1, OK_PNG_NOMEM = 2,
       OK_PNG_LIMIT = 3, OK_PNG_IO = 4 };
int ok_png_decode_gray(const unsigned char *buf, size_t n,
                       unsigned char **out, int *w, int *h);
int ok_png_read_gray(const char *path, unsigned char **out, int *w, int *h);
#ifdef __cplusplus
}
#endif
#endif
