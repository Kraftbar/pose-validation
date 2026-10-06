# ok_png: PNG decoding for the pure-C OKVIS2 app

`okvis_port/c/ok_png.{h,c}` (MIT, see `LICENSE`) decodes a PNG to 8-bit grayscale with the same pixels as
`cv::imread(path, cv::IMREAD_GRAYSCALE)` of the reference's OpenCV 4.6 (`external/vio/deps/opencv`, libpng behind it). The DatasetReader of
the OKVIS2 reference uses exactly that call, so `okvis_c_euroc` can read a EuRoC sequence directly, without the `.gray` packs.

Written from RFC 1950/1951 and the PNG specification; no zlib or libpng code was copied. Codex wrote the decoder (2026-10-05). Claude
wrote the test and the app hookup (2026-10-06).

## Interface

```c
int ok_png_decode_gray(const unsigned char* buf, size_t n, unsigned char** out, int* w, int* h);   /* 0 = OK_PNG_OK */
int ok_png_read_gray(const char* path, unsigned char** out, int* w, int* h);
```

The output is tightly packed and row-major, and the caller frees it. Errors are `OK_PNG_INVALID` / `NOMEM` / `LIMIT` / `IO`; on an error
all outputs are reset. The decoder is C99, libc only, and keeps no global state.

Coverage:
- every colour type (gray, RGB, palette, gray+alpha, RGBA) and bit depth (1/2/4/8/16), Adam7 interlacing, all five row filters;
- inflate with stored, fixed and dynamic Huffman blocks;
- chunk CRCs and the zlib Adler-32 are checked;
- libpng's RGB-to-gray integer weights and its gamma tables from gAMA / sRGB / sBIT, as OpenCV's gray path uses them;
- the eXIf orientation, which OpenCV applies after decoding.

Size limits: `OK_PNG_MAX_BYTES` 256 MB and `OK_PNG_MAX_PIXELS` 64 M, both compile-time.

## Validation

`okvis_port/reference_tools/okvis_png_test.cc` compares the decoder with the real `cv::imdecode(buf, IMREAD_GRAYSCALE)`, tolerance 0.
It runs in `python3 tools/check_okvis_port.py ... --eigen-tests` (test libs `imgcodecs zlib`).

| set | files | mismatches |
|---|---:|---:|
| real EuRoC PNGs (`runs/okvis_port/reference_brisk/frames`: MH_01_easy and V1_02_medium cam0; `OK_PNG_DIRS` adds more) | 14 | 0 |
| `cv::imencode` output: 8/16-bit, 1/3/4 channels, compression 0–9, all five zlib strategies, bilevel, random and smooth content | 1,500 | 0 |
| crafted with zlib: every colour type / bit depth, random row filter per row, Adam7, IDAT split into random chunks, zlib levels 0–9, gAMA / sRGB / sBIT / tRNS / PLTE, grey-equal RGB | 9,000 | 0 |
| corrupt: truncated, bit-flipped, or the zlib stream / IHDR damaged with valid CRCs | 3,000 | no crash |

Of the corrupt files, 2,817 fail in both decoders and 159 decode with identical pixels in both. 24 are rejected only by `ok_png`: libpng
returns a partial image there, `ok_png` an error. The whole test is clean under `-fsanitize=address,undefined`.

End to end, `okvis_c_euroc` on MH_01_easy mono reading the original PNGs (`mav0/cam0/data.csv` + `data/`) writes the same `final.csv` /
`causal.csv` sha256 as with the gray packs and as the reference (see `okvis_port/HANDOVER.md`).
