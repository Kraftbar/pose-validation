# RD-VIO M7b — undistortion and EPnP (2026-10-06, Codex)

Dependency-free C99 specializations of the **installed OpenCV 4.6.0** in
`external/vio/deps/opencv`, for the reference driver and RD-VIO's
`solve_pnp_4pt` / `solve_pnp_6pt`. All comparisons below count differing
**bytes**, including float representations, signed zeros, and degenerate
outputs. Tolerance is zero. No changes to the system or PARSAC sources.

## Interface and specialization

`c/rd_cv_undist.{h,c}` exports:

```c
int rd_cv_undistort_maps(const double K[4], const double D[4], int equidistant,
                        int w, int h, float *m1, float *m2);
int rd_cv_remap_linear(const uint8_t *src, int w, int h,
                       const float *m1, const float *m2, uint8_t *dst);
```

K is `(fx, fy, cx, cy)`, with zero skew; rectification is identity, the output
camera matrix equals K, and D has four radtan or equidistant coefficients.
Maps are contiguous CV_32FC1 equivalents. Images are contiguous grayscale,
with equal source/destination dimensions, INTER_LINEAR, and BORDER_CONSTANT
zero. Dimensions are 1..32766. Functions return 1 on success, 0 on invalid
arguments/allocation failure. Map buffers must be distinct from images and
each other. Exact image in-place remapping is supported; partial overlaps
are outside the interface. Map construction requires finite K/D and nonzero
focal lengths. EuRoC calibration is taken directly from `euroc_sensor.yaml`.

`c/rd_cv_pnp.{h,c}` and `c/rd_cv_pnp_math.{h,c}` export the requested callback:

```c
void rd_cv_pnp6(void *ctx, const double X[6][3], const double x[6][2], double T[16]);
void rd_cv_pnp4(void *ctx, const double X[4][3], const double x[4][2], double T[16]);
```

`ctx` is unused. Inputs round to Point3f/Point2f, K is CV_32F identity, and
EPnP uses no distortion or initial guess. Double rvec/tvec round to float,
Rodrigues runs on the float rvec, and the float pose is promoted to double.
**T is column-major**, matching Eigen's matrix<4> storage. The diagnostic
`rd_cv_pnp` entry also exposes the double rvec/tvec and optional intermediate
trace callbacks. Only 4 and 6 correspondences are accepted. There is no
degeneracy rejection added to OpenCV's behavior.

The port includes OpenCV's Jacobi SVD, singular-value ordering, deterministic
RNG and null-space completion, SVD inverse/solve, EPnP's QR and Gauss-Newton
steps, and both Rodrigues conversions. All workspace is local to each call;
there is no OpenCV/Eigen/BLAS dependency. Compile the C with:

```sh
cc -std=c99 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math ... -lm
```

Only the permitted standard C headers are used by the port. Test tools use
stdio/C++/Python. Source adaptations retain notices in `LICENSE-M7b-OpenCV`
and the existing `LICENSE-OpenCV-Apache-2.0`. `transcribe_epnp.py` reproduces
the structural EPnP translation from the local Apache-licensed OpenCV source.

## Dispatch findings

The machine reports `SSE SSE2 SSE3 *SSE4.1 *SSE4.2 *FP16 *AVX *AVX2
*AVX512-SKX?`; AVX2/FMA3 support is true and AVX512 support is false.
`dispatch_m7b.py` preserves build information, feature checks, symbol lists,
disassembly, native comparisons, and GDB breakpoint/backtrace evidence.

* Radtan maps execute **opt_AVX2**, with two groups of four double lanes per
  eight pixels. The port retains that row progression and lane offsets,
  explicit fused multiply-add operations, and the compiled scalar row/tail
  contractions. The 3x3 inverse uses cofactors; simplifying it to `1/fx`
  changes rounding. Fisheye uses the scalar atan/polynomial path; its
  Matx33 inverse also uses cofactors even with DECOMP_SVD requested.
* Remap executes baseline **RemapVec_8u / FixedPtCast<...,15>**. Disassembly
  shows SSE signed-short multiply/add pairs and integer shifts. Float maps
  quantize to 1/32-pixel bins, ties to even, with signed-short coordinate
  saturation. The four integer weights sum to 32768; the zero-fraction entry
  is `(32767,0,0,1)`. Products, horizontal pairs, vertical addition, +16384,
  and shift by 15 are exact in int32, so the scalar sum preserves every byte.
  NaN/overflow coordinate conversion reproduces the INT_MIN sentinel.
* EPnP executes baseline **MulTransposedR<double,double>** and OpenCV's
  **JacobiSVDImpl_<double>**, not a dispatched AVX2 SVD or external LAPACK.
  Dot products/norms retain their left-to-right order; Givens updates retain
  the SSE two-double ordering and its distinct scalar-tail expression.

With `OPENCV_CPU_DISABLE=AVX2,FMA3,AVX,AVX512-SKX,SSE4.2,SSE4.1,SSSE3,POPCNT`,
GDB confirms radtan switches to cpu_baseline. Across the 104 cases, map1 has
5,790 differing bytes and map2 8,406; remapped image bytes remain identical.
The zero-distortion, nontrivial-intrinsics cases expose these differences;
the initial distorted-calibration cases alone did not. Fisheye and all
60,336,000 EPnP comparison bytes remain identical with dispatch disabled.
The C target remains the reference's **default** dispatch.

## Validation results

Both normal and ASan+UBSan leaf runs give this table; counts are per run,
not doubled for the sanitizer replay. Eight damaged/truncated/surplus/empty
fixtures are rejected in each mode. Sanitizers report no errors, with leak
detection enabled for the leaf harnesses.

| Piece / inputs | Cases | Bytes compared | Mismatches |
|---|---:|---:|---:|
| Maps + remap, real and synthetic | 104 | 172,281,705 | 0 |
| Full MH_01 undistorted pack | 3,682 frames | 1,329,084,196 | 0 |
| EPnP + Rodrigues, synthetic + intermediates | 18,000 | 60,336,000 | 0 |
| EPnP, every captured MH_01 call | 127,057 | 16,263,296 | 0 |
| C system variant (b), native EPnP callback | 3,633 poses | 584,917 | 0 |

The system row is a normal build. The leaf total is **1,577,965,197 bytes**
per mode. Pack comparison includes the 20-byte header and every timestamp;
only one raw/expected frame is buffered at a time, with no new image pack.

Synthetic maps cover five calibrations (including zero distortion with
nontrivial K), both models, 752x480, 641x479, 7x5, 1x1, noise and checkerboard.
Eight real frames span MH_01, for both branches. Eight extra arbitrary-map
cases cover widths around SIMD boundaries, half-bin ties, negative/outside
coordinates, short saturation, infinities and NaNs. Full-pack comparison
uses all 3,682 real frames against `reference_tools/rd_undistort_pack.cc`'s
existing `runs/rdvio_port/system/MH_01_easy.undist.gray`.

Synthetic EPnP uses 1,000 cases for each of nine conditions for each point
count: ordinary, planar, almost planar, behind-camera, depths scaled by
1e-6/1e6, collinear, coincident, and noisy projections. Expected rvec/tvec
and float Rodrigues are from the installed library. A namespaced copy of
OpenCV EPnP adds intermediate observations; its outputs must also pass
memcmp against installed solvePnP before a fixture is accepted.

Patch `0010-m7b-pnp-dump.patch` observes every 4/6-point call in a separate
reference tree/run. MH_01 invokes the 6-point form 127,057 times and the
4-point form zero times; 4-point coverage is synthetic. Inputs are dumped
before the float rounding and poses after RD-VIO's float-to-double assembly.
The observe-only reference run and C system with `rd_cv_pnp6` both have:

```text
f0d60a3e03c1243d750da735a5f5da1ec232237a6d84c4e9a35f15747377a0fe
```

`check_m7b_system.py` builds the existing system variant (b) by renaming its
callback symbol at compile time and linking these C files instead of the
OpenCV shim. It passes no logged PARSAC masks and links only libm. System
sources are untouched. The run uses the existing undistorted pack, whose
entire contents have independently matched the C undistortion/remap.

## Reproduction

From the repository root, with the existing raw and undistorted gray packs:

```sh
# Generate fresh native fixtures and run all leaf comparisons.
python3 -B tools/check_rdvio_port.py --modules m7b --oracle
# Replay and sanitize.
python3 -B tools/check_rdvio_port.py --modules m7b
python3 -B tools/check_rdvio_port.py --modules m7b --sanitize
# Native dispatch evidence, then the full C system variant (b).
python3 -B rdvio_port/reference_cv/dispatch_m7b.py
python3 -B rdvio_port/reference_cv/check_m7b_system.py
```

The leaf runner requires the complete real-call dump. To recreate it:

```sh
python3 -B rdvio_port/reference_cv/make_pnp_patch.py
python3 -B rdvio_port/reference_cv/build_reference.py --root m7b_reference_build
mkdir -p runs/rdvio_port/m7b_reference_run
RDVIO_CV_PNP_DUMP="$PWD/runs/rdvio_port/m7b_reference_run/pnp_real.bin" \
RDVIO_CV_GRAY="$PWD/external/vio/data/okvis_brisk_tmp/MH_01_easy/gray/cam0.gray" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=1 \
python3 -B tools/run_rdvio_reference.py MH_01_easy --tag m7b_reference_run \
  --stock-binary "$PWD/runs/rdvio_port/m7b_reference_build/build/rdvio_ref_driver"
```

The reference build uses at most four jobs; run it separately from other
heavy work. The current run's optional ATE scoring could not import numpy;
the driver exited successfully and the canonical trajectory hash matched.
No ATE claim is made here.

Artifacts live under `runs/rdvio_port/reference_cv_m7b/`, including
`validation_{normal,san}.json`, fixture hashes, logs, dispatch evidence,
source/library provenance and `system/validation_normal.json`. The separate
reference tree/run use the tags above. New artifacts total about 320 MiB;
no EuRoC image/dump is written to a tracked source directory.

## Limits / remaining integration

The requested specializations have no outstanding fixture mismatch. This is
not a general replacement for calib3d: no arbitrary K/R/new camera matrix,
skew, other distortion lengths, color/strided images, interpolation/border
modes, other PnP point counts/methods, initial guesses, or Rodrigues Jacobians.
Nonfinite EPnP input and overflowing calibration arithmetic are not covered.
The bitwise target is this x86-64 reference build, default rounding mode and
host libm; other toolchains/architectures require their own validation.
Native integration into the normal system driver remains with its owner,
as requested. No `rd_sys*`, `rd_imu_parsac*`, `rd_map*`, or driver source was
edited, and nothing was committed.

## Files created / changed

* Port: `c/rd_cv_undist.{h,c}`, `c/rd_cv_pnp.{h,c}`,
  `c/rd_cv_pnp_math.{h,c}`.
* Harnesses: `c/check_rd_cv_undist.c`, `c/check_rd_cv_pnp.c`.
* Native tools: `reference_cv/dump_undist.cpp`, `dump_pnp.cpp`,
  `prepare_pnp_trace.py`, `m7b_io.hpp`.
* Build/replay/evidence: `reference_cv/build_m7b.py`, `run_m7b.py`,
  `check_m7b_system.py`, `dispatch_m7b.py`, `m7b_io.h`, `transcribe_epnp.py`.
* Observation patch: `reference_cv/make_pnp_patch.py`,
  `reference/patches/0010-m7b-pnp-dump.patch`.
* Licence/docs: `reference_cv/LICENSE-M7b-OpenCV`, this file, a link in
  `reference_cv/README.md`, and the new top entry in `HANDOVER.md`.
* Runner integration only: `tools/check_rdvio_port.py`.
