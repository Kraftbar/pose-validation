# RD-VIO M7 image leaf — 2026-10-06

Dependency-free C99 specialization of the OpenCV image functions called by
RD-VIO (`Jianxff/rd_vio` at `099f5e886ebb9d33ccf0e4b17af7c48c57da69b4`).
The oracle is the installed **OpenCV 4.6.0** in `external/vio/deps/opencv`.
All reported comparisons are bytewise, with tolerance zero. Nothing committed.

## Scope and exact specialization

- Continuous 8-bit grayscale input, already undistorted by the caller.
- CLAHE: clip limit **6.0**, **8×8** tiles, `BORDER_REFLECT_101` extension.
  If only one dimension is divisible by eight, OpenCV still adds eight pixels
  on that axis when extending the other axis. The port preserves this rule.
- Optical-flow pyramid: **21×21** window, maximum level **3** (up to four
  image/derivative pairs), derivatives enabled, image `BORDER_REFLECT_101`,
  derivative `BORDER_CONSTANT`. Every image has **21 pixels of padding on all
  four sides**, including the derivative image. Small inputs stop early under
  OpenCV's `(next_width <= 21 || next_height <= 21)` rule.
- LK: grayscale, the prebuilt image/Scharr pyramid, **30 iterations**, epsilon
  **0.01** (squared in double), minimum eigenvalue **1e-4**, and
  **`OPTFLOW_USE_INITIAL_FLOW` in both directions**. RD-VIO uses this flag even
  when it initializes next points with the current points.
- GFTT: quality **0.001**, minimum distance **20**, block size **3**, Sobel
  aperture **3**, Harris **k=0.04**, no mask. EuRoC requests **200** keypoints.
  The API accepts the limit; zero means unlimited, as in OpenCV. Harris is the
  actual RD-VIO setting; the min-eigen alternative is also implemented/tested.
- `cv::norm(Point2f)` and RD-VIO's image-side status checks: 20-pixel border,
  displacement above integer `rows/4`, reverse status, reverse distance >0.5.

RD-VIO performs a **second response-only `std::sort`** after GFTT. OpenCV GFTT
itself breaks response ties by decreasing image address. The second sort has
no such tiebreaker, so its equal-score permutation must also be reproduced.
Both stages, including complete `KeyPoint` fields, are compared. The scalar
median-partition/introsort model follows the accepted OKVIS leaf method; no
GPL/libstdc++ source was read or copied. The GFTT distance grid is implemented;
RD-VIO's subsequent Poisson filtering remains in the existing M3 module.

## C interface

See [`../c/rd_cv.h`](../c/rd_cv.h). Implementations:
[`rd_cv.c`](../c/rd_cv.c), [`rd_cv_lk.c`](../c/rd_cv_lk.c),
[`rd_cv_gftt.c`](../c/rd_cv_gftt.c).

1. `rd_cv_clahe(src,w,h,dst)` accepts separate or identical input/output buffers.
2. `rd_cv_build_pyramid` fills a **zero-initialized** `rd_cv_pyramid`;
   `rd_cv_free_pyramid` releases it. Each level owns complete padded storage;
   the logical image starts at `(21,21)`. Derivatives are interleaved `int16_t`
   dx/dy; `step` is in pixels. Free before rebuilding.
3. `rd_cv_gftt` allocates its ordered keypoint list; caller frees it.
   `rd_cv_sort_keypoints` applies RD-VIO's second sort in place.
   `rd_cv_corner_response` optionally emits Sobel/covariance observations;
   their memory is valid only during the callback.
4. `rd_cv_lk` takes current points and in/out initial flow. It writes status
   and optional err; call again with the reversed pyramids and current points
   as reverse initial flow. `rd_cv_track_status` combines both passes' checks.

Functions return 1 on success, 0 for invalid inputs/allocation failure.
Supported dimensions are 1..16384; LK input coordinates must be finite and
within ±1e6. No general ROI/stride API, configurable CLAHE, multichannel image,
other LK flags/window/criteria, or arbitrary corner parameters are claimed.
The port models **default dispatch on this reference machine**, regardless
of the machine on which the C code runs. It does not query CPU features.

Only `<stdint.h>`, `<stdlib.h>`, `<string.h>`, `<math.h>` and `<float.h>` are
used by the port. No SIMD intrinsics, OpenCV, Eigen or C++ dependency. Compile
as C99 with libm, `-ffp-contract=off -fno-fast-math`; use IEEE binary32/binary64,
32-bit int and round-to-nearest/ties-to-even. Explicit `fmaf` calls model the
reference's fused operations despite disabling implicit contraction.
Fixture encoding is the pinned little-endian x86-64 native ABI.

## Reproduction

From the repository root, using at most four threads:

```sh
# Build native oracle + C harness, generate the complete 70-case set, replay it.
python3 -B rdvio_port/reference_cv/run.py
# Reuse the saved gzip fixtures through the main port runner.
python3 -B tools/check_rdvio_port.py --modules m7
python3 -B tools/check_rdvio_port.py --modules m7 --sanitize
# --oracle regenerates M7's complete image fixture set instead of reusing it.
python3 -B tools/check_rdvio_port.py --modules m7 --oracle
# Reproduce CPU feature reports, symbol/disassembly evidence, disabled-CPU test.
python3 -B rdvio_port/reference_cv/dispatch.py
```

`build.py` compiles the native dump tools with `-O2 -DNDEBUG
-ffp-contract=off -fno-fast-math`, links the installed OpenCV libraries, and
records source/binary hashes in `provenance_{normal,san}.json`. The **libraries
have their own original Release build flags** (`-O3`, SSE3 baseline, dispatched
ISA kernels), recorded in `dispatch_default.txt`; oracle flags do not rebuild
or change those libraries. Native tools call `cv::setNumThreads(1)`.

The input stream is opened read-only:
`external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray` (`OKGRAY1\0`, three
u32 dimensions/count, then u64 timestamp + grayscale bytes per frame).
Undistorted fixtures use the same cam0 calibration, double K/D,
`initUndistortRectifyMap(...,CV_32FC1)` and `remap(...,INTER_LINEAR)` as the
reference driver. No replacement undistortion is implemented in C.

Real pairs use bases **0,100,700,1800,3000**, each with gaps **1..5** after
undistortion; five additional raw pairs use base 0. Initial points come from
GFTT, with controlled prediction offsets and extra border/outside/subpixel
points. Synthetic inputs are black, white, ramp, repeated checkerboard and
seeded xorshift noise, with a 37×9-pixel translated second image. Dimensions:
752×480, 640×480, 753×479, 751×480, 640×481, 31×27, 1×1, 43×43. Some initial
flows deliberately predict large motion outside the image.

Each fixture stores inputs followed by named length-delimited outputs:
CLAHE for both images, all padded pyramid images and derivatives, Sobel dx/dy,
covariance, Harris and min-eigen responses, both GFTT lists, RD-VIO's second
sort, both LK point/status/err arrays, norms and combined status. Expected
outputs never replace C intermediates. The C harness independently generates
the GFTT points used for real tracking. It rejects truncated/surplus fixtures.

**Failed-point LK err requires care:** OpenCV leaves some entries untouched.
The leaf oracle and checker initialize caller-owned err buffers with the same
quiet-NaN bit pattern `0x7fc12345` before each call, so all err bytes can be
compared without pretending that uninitialized storage has a defined value.
This does not change LK computation. The actual RD-VIO stream is observe-only
and excludes err bytes for status=0; every point and status is still compared.

Images/dumps stay under ignored `runs/rdvio_port/reference_cv/`. Fixtures are
gzip-compressed; replay decompresses one case in memory. The generator checks
free space before creating its <1 GB uncompressed batch. Never commit the
EuRoC images or dumps (non-commercial evaluation data).

## Dispatch finding

The machine reports SSE/SSE2/SSE3 baseline, AVX/AVX2/FMA3 available, AVX512F
unavailable. `nm` and the full LK disassembly show **one baseline
`LKTrackerInvoker`**, with SSE XMM arithmetic, and no `opt_AVX2` LK kernel.
Its `CV_SIMD128 && !CV_NEON` branch is compiled in; disabling CPU features does
not turn it into the source's scalar fallback. Scharr likewise has a baseline
SIMD implementation. The C model preserves the four accumulation lanes:

- The first 16 pixels of each 21-pixel row contribute to SIMD lanes, and the
  remaining five to separate scalar sums. Tensor lanes reduce as
  `(q0+q2)+(q1+q3)` before addition to the scalar tail.
- LK residual dot products add paired integer products at x and x+4 **before**
  float conversion. Two four-lane sums then combine x/y components. A scalar
  left-fold over all 441 pixels is not equivalent.
- Sobel selects **`opt_AVX2`**, including fused `v_muladd` in full vector groups
  and **unfused scalar tails**. The row boundary is a multiple of 32 pixels.
- Harris uses `calcHarrisLine_AVX`: `k*(a+c)^2`, with a differently grouped SSE
  remainder and a double-k scalar tail. Min-eigen uses float square roots.
- `boxFilter<float>` accumulates in **double**, not float. Initializing the
  vertical sum from +0 also matters for signed-zero bytes.

The old setting disabled AVX2/FMA3/AVX, but **`AVX512_*`, `SSE4_2`, `SSE4_1`
are unrecognized names**. `dispatch_original_env.txt` records the warnings
and still-enabled SSE4 support. Correct names are `AVX512-SKX`, `SSE4.2`,
`SSE4.1`; disabling SSE3 has limited effect because it is baseline code.

Corrected CPU-disable comparison, two cases (odd noise and undistorted EuRoC):

| Output | Bytes compared | Changed bytes |
|---|---:|---:|
| CLAHE, both images | 1,443,294 | 0 |
| Padded pyramid images and derivatives | 11,672,250 | 0 |
| LK points/status/err, both passes | 10,816 | 0 |
| Sobel dx | 2,886,588 | 117,724 |
| Sobel dy | 2,886,588 | 129,060 |
| Harris response | 2,886,588 | 419,111 |

GFTT response fields changed; its point coordinates/order in these two cases
did not. Thus an unchanged trajectory hash does **not** demonstrate that all
OpenCV image buffers are CPU-feature independent. The port targets the
original/default optimized path, not the disabled-CPU diagnostic path.

## Results

Normal and ASan/UBSan builds independently pass the same complete fixture set:

| Inputs | Cases | Compared output bytes | Mismatches |
|---|---:|---:|---:|
| Synthetic | 40 | 394,162,049 | 0 |
| Raw MH_01_easy pairs | 5 | 83,438,220 | 0 |
| Undistorted MH_01_easy pairs | 25 | 417,191,100 | 0 |
| **Total, per build** | **70** | **894,791,369** | **0** |

2,290 output records per build. Both builds pass 18 API/ownership checks and
reject three corrupted/truncated/surplus fixtures. LeakSanitizer is disabled
(`ASAN_OPTIONS=detect_leaks=0`); no leak-check claim. Results and fixture hashes:
`validation_{normal,san}.json`, `fixtures_manifest.json`, `check_{normal,san}.log`
and `.err`, `api_*.log`, `negative_*.log` in the artifact directory.

## Full RD-VIO observation and replay

Patch [`0008-m7-image-stream.patch`](../reference/patches/0008-m7-image-stream.patch)
only observes `OpenCvImage` preprocessing, detection and tracking. It emits
actual input images/points and outputs through `RDVIO_CV_STREAM`. No numerical
operation or error-buffer initialization is changed. The C stream harness
recomputes each operation as it arrives, allowing every frame to be checked
without retaining tens of GB of image dumps.

```sh
# Uses tools/build_rdvio_reference.py in an isolated, owned M7 build tree.
python3 -B rdvio_port/reference_cv/build_reference.py
# Uses tools/run_rdvio_reference.py with the isolated binary and a new tag.
python3 -B rdvio_port/reference_cv/run_reference.py --tag m7_stream
# Optional sanitizer check of the stream reader and all three event types.
python3 -B rdvio_port/reference_cv/run_reference.py \
  --sanitize --tag m7_stream_san_smoke --max-seconds 3
```

The build wrapper redirects only the builder's paths into
`runs/rdvio_port/m7_reference_build/`, reuses the existing reference Ceres
installation, and uses four build jobs. Its generated driver replaces PNG
reads with the supplied decoded gray stream, checks every timestamp against
cam0/data.csv, preserves the original OpenCV remap, and explicitly sets one
OpenCV thread. Neither the shared reference build nor the original driver is
modified. `make_patch.py` regenerates patch 0008 from the upstream source and
`stream_dump.hpp`.

MH_01_easy completed: **3,682 driver frames, 3,633 poses**, with every image
operation actually executed by that run replayed. The last frame remains
queued awaiting the next IMU sample in the canonical driver, hence 3,681
preprocessing/detection calls and 3,680 tracking calls.

| Actual operation | Calls | Compared output bytes | Mismatches |
|---|---:|---:|---:|
| CLAHE + complete padded pyramid | 3,681 | 12,067,584,264 | 0 |
| GFTT + RD-VIO response sort | 3,681 | 41,082,272 | 0 |
| Forward/reverse LK + final image status | 3,680 | 42,030,111 | 0 |
| **Total** | **11,042** | **12,150,696,647** | **0** |

17,772 bytes of failed-track err storage are excluded from this stream total;
points and statuses for failed tracks are included. This is a full normal-build
stream replay, not a claim that the whole RD-VIO C port is integrated.
The actual stream is consumed online and not retained; the reproducible leaf
fixtures are retained separately.

The trajectory SHA-256 is identical to canonical `run1` and `m5`:

```
f0d60a3e03c1243d750da735a5f5da1ec232237a6d84c4e9a35f15747377a0fe
```

Evidence: `runs/rdvio_port/m7_stream/{check.log,check.err,MH_01_easy/run.json}`,
`reference_cv/validation_stream.json` under `runs/rdvio_port/`.
The system Python lacks numpy, so the runner's optional ATE scoring helper
reported an import error; no new ATE result is claimed. Trajectory hashing and
all byte comparisons completed successfully.

The stream harness also passes ASan/UBSan over the first three seconds:
59 preprocessing calls, 59 detection calls, 58 tracking calls;
**194,591,160 bytes, zero mismatches**, excluding 1,588 unspecified err bytes.
Its short trajectory equals the normal smoke run byte for byte. This is
additional stream-reader sanitizer coverage; the complete 70-case leaf set
is independently sanitizer-clean as reported above.

Retained M7 artifacts are about **0.46 GiB**, including compressed fixtures
and the isolated reference build. More than 18 GB remained free.

## Limits and integration responsibilities

This leaf does not port `imread`, `remap`, EPnP/`solvePnP`, Rodrigues, Ceres
bicubic interpolation, or the full RD-VIO image object. FAST/ORB factories in
`OpenCvImage` are unused by its per-frame path. M3 supplies Poisson filtering;
M6/M8 must wire ownership, the persistent detector limit, double-to-float point
conversion, feature prediction and final keypoint updates around this API.
No masks, alternative configurations, other CPUs' native dispatch paths,
other OpenCV/C++ standard-library versions, or whole C-port trajectory are
claimed. The second sort is tied to the tested libstdc++ ordering.

## Licenses

OpenCV-derived implementation: Apache-2.0 with the retained BSD-3-Clause
source notices, in [`LICENSE-OpenCV-source`](LICENSE-OpenCV-source) and
[`LICENSE-OpenCV-Apache-2.0`](LICENSE-OpenCV-Apache-2.0). New harnesses, scripts
and observation glue: MIT, [`LICENSE-MIT`](LICENSE-MIT). RD-VIO itself is
Apache-2.0. No GPL source was read/copied, and no files in `okvis_port/` were
modified.
