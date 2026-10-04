# BRISK frontend leaf — completed 2026-10-01

Pure C99 port of the **BRISK path actually selected by OKVIS2's EuRoC
frontend**, validated against the pinned native library. No commits or
benchmark registration. Source ownership stays within `reference_brisk/`,
`c/ok_brisk*` and `c/check_ok_brisk*`; artifacts stay in ignored
`runs/okvis_port/reference_brisk/`.

## Upstream and scope

- OKVIS2: `a2ea00688cd10988aae7bd52ab7935ce9a657ec0`.
- Its BRISK submodule: `1ef8b42a5c2fdd0e0c976f3ab7b179806381f570`
  (CMake version 2.0.7).
- `Frontend::initialiseBriskFeatureDetectors()` constructs
  `ScaleSpaceFeatureDetector<HarrisScoreCalculator>`, **not**
  `BriskFeatureDetector` / AGAST. The AGAST implementation is consequently
  not part of this leaf.
- `config/euroc.yaml`: uniformity radius **38**, absolute Harris threshold
  **150**, **0 octaves**, maximum **700** keypoints. The frontend constructor's
  fallback values are 40 / 200 / 0 / 450; these are overridden by this config.
- Descriptor: BRISK v2, rotation invariant, scale invariance disabled,
  pattern scale 1; **66 sample points, 384 short pairs, 856 long pairs,
  48 descriptor bytes**. The fixed descriptor scale is still quantized with
  upstream's formula; it is not the detector's 12-pixel keypoint size.
- Camera-aware extraction receives the ray-direction map, 2x3 image-Jacobian
  map, horizontal focal length and gravity direction in camera coordinates.
  The C API accepts these same inputs. It also implements upstream's
  image-gradient orientation fallback near the gravity axis or invalid rays.

Supported specialization: continuous 8-bit grayscale images, fresh detection,
zero octaves, no detector mask, default v2 descriptor settings above. No
legacy BRISK v1, arbitrary pattern file, 16-bit image mode, multioctave
pyramid, scale-invariant descriptor or supplied-keypoint detector mode is
claimed. Detector radius/threshold/limit are API parameters; the reported
reference comparison uses the EuRoC settings.

This is a frontend component comparison, not an OKVIS2 trajectory/ATE result.
Camera-map construction and the transformation of gravity into camera
coordinates remain the camera/system caller's responsibility.

## C interface and integration

See [`../c/ok_brisk.h`](../c/ok_brisk.h).

1. Create an `ok_brisk_context` once. It owns the used scale's complete
   1024-angle pattern table (about 0.8 MB) and orientation pairs.
2. Call `ok_brisk_detect(image, width, height, 38, 150, 700, ...)`.
   The function allocates the ordered keypoint array; caller frees it.
3. Call `ok_brisk_describe` with caller-owned camera maps and gravity direction.
   It removes border keypoints in place, updates their angles, and allocates
   `count * 48` descriptor bytes. Caller frees descriptors and keypoints.
4. Destroy the context when finished. A NULL trace disables diagnostics;
   callbacks receive ephemeral array views, never transfer ownership.

Compile the three `.c` implementation files with C99 and libm only:

```sh
gcc -std=c99 -O2 -ffp-contract=off -fno-fast-math \
  okvis_port/c/check_ok_brisk.c \
  okvis_port/c/ok_brisk_detector.c \
  okvis_port/c/ok_brisk_descriptor.c \
  okvis_port/c/ok_brisk_camera.c -lm -o /tmp/check_ok_brisk
```

No SIMD intrinsics, OpenCV, Eigen, C++ runtime or nonstandard library is used
by the port. Standard C allocation and libm are used. Bit comparisons target
IEEE binary32, 32-bit `int`, little-endian native fixtures and the pinned
x86-64 reference/toolchain. Floating contraction and fast-math must stay off.

## Reference and reproduction

`build.py` copies only the BRISK subtree to the isolated runs folder. It
applies the generated [`trace.patch`](trace.patch) there, builds the real
library with SSE enabled, and compiles `dump_main.cpp` against the existing
OpenCV 4.6 installation. The patches only observe existing computations;
there are no numerical guards, reordered operations or replacement algorithms.
The native camera maps come directly from OKVIS2's real
`PinholeCamera<RadialTangentialDistortion>::initialiseCameraAwarenessMaps()`.
No full OKVIS2 rebuild is needed and its shared checkout/build is untouched.

```sh
python3 okvis_port/reference_brisk/build.py
python3 okvis_port/reference_brisk/fetch_frames.py \
  --seq MH_01_easy --indices 0,1,2,3,10,25,50,100
python3 okvis_port/reference_brisk/fetch_frames.py \
  --seq V1_02_medium --indices 0,1,10,25,50,100
python3 okvis_port/reference_brisk/run.py
# Recheck C against the existing complete, deterministic fixture set:
python3 okvis_port/reference_brisk/run.py --reuse
```

The fetcher reads the ETH archive URLs from `tools/vio_harness/fetch_seq.py`.
It extracts only selected PNGs. Stored outer members allow nested range
access; deflated outer members are streamed up to the requested cam0 frame
window, handling inner ZIP data descriptors without storing an archive.
Indices in streaming mode are local-entry order; each manifest records the
original timestamp filename. ETH may return HTTP 429; retries are bounded.
**EuRoC is non-commercial evaluation data: never commit images or dumps.**
All generated files, including synthetic images and negative fixtures, remain
under `runs/`. The complete retained leaf artifacts use about **0.77 GB**, below
its 1.5 GB allocation. No existing datasets or other workers' outputs were
removed. The fetcher does not download an entire sequence to disk.

The reference uses one OpenCV thread and explicitly enabled native optimized
paths. Compiler: GCC/G++ 13.3.0. Final reference flags: `-O2 -DNDEBUG
-ffp-contract=off -fno-fast-math -mssse3`; C port: `-O2 -ffp-contract=off
-fno-fast-math`. `provenance.json` records pins, source hashes, trace-patch hash,
effective CMake flags, OpenCV core/config hashes and frontend parameters.

## Observations and fixture contract

Each case stores dimensions, camera/mode, gravity direction, focal length and
the input grayscale image, followed by ordered named byte records. Maps are
separate per-camera files. `pattern.bin` is context initialization, independent
of any image. The C harness never feeds native intermediate outputs back into
its computation; expected buffers are used only by the trace comparator.

Compared exactly, without tolerance:

- Used scale index, border size, full rotated pattern table, short/long pairs.
- Entire integer Harris score image; row-major 2D maxima; sorted candidates;
  uniformity-selected candidates; subpixel keypoints including every field.
- Descriptor border filtering, complete integral image, each 2x2 camera warp,
  smoothing scale and gravity-direction branch decision.
- All 66 pre-orientation intensities when fallback is used; all 66 final
  sampling intensities; final angles, keypoints and every descriptor byte.

The `cv::KeyPoint` record has five floats followed by octave/class-id int32s.
Trace framing is a zero-padded 32-byte name, uint32 byte count, then payload.
The checker rejects truncated, surplus and mismatching fixture data.

## Results

14 real cam0 images: eight MH_01_easy and six V1_02_medium. Five synthetic
images: black, white, ramp, repeated checkerboard and deterministic noise.
Modes are ordinary orientation, camera-aware controlled gravity and a
camera-axis direction that exercises the gradient fallback.

| Calibration / inputs | Cases | Exact bytes compared | Trace records | Mismatches |
|---|---:|---:|---:|---:|
| cam0: real + synthetic, 3 modes | 57 | 186,574,452 | 63,962 | 0 |
| cam1: same real cam0 image inputs, 2 camera-aware modes | 28 | 91,578,824 | 29,263 | 0 |
| Total | **85** | **278,153,276** | **93,225** | **0** |

41,495 final keypoints/descriptors are compared. Camera-1 calibration is
applied to the same cam0 images as a controlled leaf input, not presented as
real cam1 recordings. Gravity directions are controlled poses, not states
captured from an end-to-end estimator.

All 89 binary/map outputs match across two fresh native processes per mode.
Normal and ASan/UBSan builds pass the full 85-case set. Both builds also pass
21 API boundary/ownership checks and reject three negative fixture variants.
LeakSanitizer is disabled under sandbox ptrace; no leak-check claim is made.
Commands, per-case logs, checksums and summary: `validation.json`,
`check_cam{0,1}_{normal,san}.log`, `api_*.log`, `negative/` in the runs folder.

## Exactness details worth preserving

- SSE Scharr products use signed high-half multiplication before smoothing.
  The C port implements the corresponding signed shifts explicitly.
- Uniformity uses float LUT weights, ceil, and **saturating unsigned byte
  addition**, not an ordinary occupancy boolean or wrapping byte addition.
- Equal-score candidate order matters. The independent C comparison sort
  reproduces the pinned native median-partition/insertion ordering, checked
  against real candidate arrays and repeated checkerboard ties. No libstdc++
  source was read/copied. This is not a promise of matching every C++ vendor's
  unspecified equal-key ordering.
- Preserve upstream's subpixel edge-case assignments (including its unusual
  `delta_y = delta_x1` / `delta_x2` branches); do not "correct" them silently.
- Unqualified global `log`, `sqrt` and `atan2` in this BRISK build select
  **double** arithmetic even for float arguments. Using `logf` initially
  changed the pattern by 1–2 ULP and propagated to sample intensities.
- Camera warp multiplication accumulates float inputs in double. The
  OpenCV `cv::eigen` path here uses its float Jacobi implementation, only the
  upper triangle of the nonsymmetric warp, and its own scaled hypot routine.
  It does not call Eigen (`HAVE_EIGEN` is off in the pinned OpenCV config).

## Licenses

BRISK-derived C code/pattern data: BSD-3-Clause, notices in `LICENSE-BRISK`.
The small OpenCV camera arithmetic adaptation retains BSD-3/Apache-2.0 terms
and notices in `LICENSE-OpenCV-source` and `LICENSE-OpenCV-Apache-2.0`.
Project-authored harnesses/scripts: MIT, `LICENSE-MIT`.
No GPL source or other port's implementation was read or reused. Native
OpenCV/Eigen dependencies are reference-only; the C modules use standard C
and libm. No Eigen algorithm was transplanted into this leaf.
