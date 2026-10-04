# RD-VIO license audit (phase 0 of the pure-C RD-VIO port)

Date: 2026-10-04. Scope: everything compiled into the mono + IMU path of the RD-VIO reference
(`external/vio3/rd_vio`, `Jianxff/rd_vio`, HEAD `099f5e886ebb9d33ccf0e4b17af7c48c57da69b4`, last commit 2024-04-14), plus the
dependencies of the reference build. Method as in `docs/okvis2_license_audit.md`: read the LICENSE/COPYING files, grep every
per-file copyright/licence header class, `ldd` of the built binary, the CMake caches. Not legal advice.

## Verdict

**No blocker.** Everything that ends up in the algorithm path is Apache-2.0 (RD-VIO itself, XRSLAM-derived, OpenCV 4.x), BSD-3 (Ceres),
MPL-2.0 (Eigen 3.4), or MIT (spdlog + its bundled fmt, yaml-cpp). No GPL/LGPL/non-commercial code is compiled in. Three things to act on:

1. **The RD-VIO `LICENSE` is the unfilled Apache-2.0 template** (`Copyright [yyyy] [name of copyright owner]`, no holder named, no
   per-file headers: `grep -ri copyright src/` finds nothing). The repo README says it is "separated and modified from xrslam"
   (`openxrlab/xrslam`, `LICENSE`: Apache-2.0, "Copyright 2022 XRSLAM Authors"). Derived C files must therefore carry:
   Apache-2.0 + "Copyright 2022 XRSLAM Authors; modifications (rd_vio) Jianxff 2023-2024", and ship `rdvio_port/LICENSES/rdvio-Apache-2.0.txt`
   plus a NOTICE entry. Apache-2.0 section 4(b) (state changes) is met by the C files' headers ("translated to C99").
2. **Eigen 3.3.7 (the stock XRSLAM/RD-VIO Ceres 1.14 build) contains LGPL-2.1 files** on the SPARSE_SCHUR path:
   `Eigen/src/SparseCholesky/SimplicialCholesky_impl.h` (adapted from T. Davis' LDL, LGPL text in the header) and
   `Eigen/src/OrderingMethods/Amd.h` (adapted from CSparse, LGPL text; both guarded by `NonMPL2.h`). In Eigen 3.4.0 both files
   carry the MPL-2.0 header and Davis' relicensing grant ("has executed a license with Google LLC to permit distribution of this code and
   derivative works as part of Eigen under the Mozilla Public License v. 2.0"). The port therefore models **Eigen 3.4.0 only**
   (as `okvis_port` does, `ok_amd.c` / `ok_sparse.c` are reusable), and the deterministic reference is built against Eigen 3.4.0 +
   Ceres 2.2.0, never the 3.3.7 tree. The stock binary's Eigen 3.3.7 LGPL parts are irrelevant for distribution because the reference
   is never distributed, but they are the reason not to port from the 3.3.7 sources.
3. **Do not ship EuRoC / phone data or dumps** (non-commercial, as for the other ports). The `configs/euroc_sensor.yaml`
   calibration copied into `rdvio_port/reference/configs/` is a test fixture only.

Provenance caveat (cannot be closed by reading licences): RD-VIO's files have no headers, so a GPL origin of individual algorithms
(the initializer is a VINS-style visual-inertial alignment; the sliding-window design is OKVIS/VINS-like) cannot be excluded from the
files alone. The repository claims Apache-2.0, XRSLAM (SenseTime-affiliated OpenXRLab) published it under Apache-2.0, and the
algorithms involved are textbook (Wahba rotation, gravity/scale linear alignment, Schur-complement marginalisation). Nothing here was
checked against GPL sources (clean-room rule). Same residual risk as for any Apache-2.0 repository without per-file headers.

## RD-VIO itself (`src/`, 8,725 lines of hpp+cpp excluding spdlog, of which the examples are ~1.6 k)

| Path | License | Notes |
|---|---|---|
| `src/rdvio`, `rdvio_estimation`, `rdvio_extra`, `rdvio_geometry`, `rdvio_map`, `rdvio_util` | Apache-2.0 (repo `LICENSE`, no per-file headers, no holder named) | XRSLAM-derived (see verdict 1). The pose-graph / global-localizer / HTTP parts of XRSLAM (`httplib.h` MIT, `json.h` MIT, `base64.h`) were removed by the split and are **not** in rd_vio. |
| `examples/` (`dataset.hpp`, `pviz.hpp`, `test_*.cpp`) | Apache-2.0 | Pangolin viewer (MIT) + EuRoC reader; not built (`add_subdirectory(examples)` disabled by reference patch 0001), not ported. |
| `configs/*.yaml` | Apache-2.0 | `euroc_sensor.yaml` / `advio_sensor.yaml` hold dataset calibration numbers (facts about the sensors). `setting.yaml` has a `backend:` section naming `Vocabulary/ORBvoc.txt` (ORB-SLAM vocabulary) -- **no code in rd_vio reads it** (`grep -i backend src` is empty) and the file is not shipped. Do not copy those keys into the port's config. |
| `3rd/spdlog` | MIT (spdlog, (c) 2016 Gabi Melman) + bundled fmt (MIT-style, (c) 2012-2016 Victor Zverovich) | Used only by `rdvio_util/src/debug.cpp` for log output. Not ported (plain stderr). |

## Dependencies compiled / linked into the reference

| Dependency | Version (stock build -> port reference) | License | Used for | Port action |
|---|---|---|---|---|
| Ceres Solver | 1.14.0 (xrslam superbuild, EigenSparse, no SuiteSparse/CXSparse, LAPACK on) -> **2.2.0** (okvis2 submodule `85331393`, pristine) | BSD-3-Clause ((c) 2015 / 2023 Google); 2.2.0 `LICENSE` also holds the Apache-2.0 text for Abseil-derived headers (`fixed_array.h`, `memory.h`) | SPARSE_SCHUR + traditional Dogleg, Cauchy loss, manifold (quaternion), `SizedCostFunction` factors, **`ceres::BiCubicInterpolator`/`Grid2D`** in `OpenCvImage::evaluate` (image sampling; check whether the call is live before porting) | Port algorithms with BSD-3 notices (`okvis_port` already transliterated the Dogleg/Schur/LDLT core from 2.2.0; reuse). |
| Eigen | 3.3.7 (stock) -> **3.4.0** (`external/vio/deps`) | MPL-2.0; the only LGPL-2.1 files (3.3.7: `SimplicialCholesky_impl.h`, `Amd.h`, `IncompleteLUT.h`) are MPL-relicensed in 3.4.0 except `IncompleteLUT.h` (not on this path) | all linear algebra, `JacobiSVD`, `EigenSolver`, `SelfAdjointEigenSolver`, `LLT`, inverse | Port Eigen 3.4.0 evaluation-order models (MPL-2.0 files, as `ok_eigen.c`, `ok_dense.c`). |
| OpenCV | 4.6.0 (`external/vio/deps/opencv`) | Apache-2.0 (since 4.5.0); bundled 3rdparty (zlib, libpng, libjpeg-turbo, ...) permissive. Not built with `OPENCV_ENABLE_NONFREE`; no contrib modules used | `GFTTDetector`, `calcOpticalFlowPyrLK`, `buildOpticalFlowPyramid`, `createCLAHE`, `solvePnP(EPNP)`, `Rodrigues`, `undistort`, `remap`/`initUndistortRectifyMap` (driver), `imread`, `cv::norm` | Re-implement the call subset bit-exactly from the (Apache-2.0) OpenCV sources; keep the OpenCV notice on derived files (see PLAN). |
| yaml-cpp | 0.7.x (MSCEqF `_deps`) | MIT ((c) 2008-2015 Jesse Beder) | `YamlConfig` loader | Not ported (own minimal config reader). |
| glog / gflags | 0.6.0 / 2.2.2 | BSD-3-Clause | needed by Ceres 2.2 (stock Ceres 1.14 used miniglog) | none |
| OpenBLAS / LAPACK | 0.3.26 | BSD-3-Clause | linked via Ceres `LAPACK=ON`; not on the numeric path (`dense_linear_algebra_library_type` = EIGEN default) | none |
| libunwind, liblzma, libgfortran, libstdc++, libgcc | system | MIT / 0BSD / GPL-3 with GCC Runtime Library Exception | runtime of the reference binary | irrelevant for the C port; the reference is never distributed |
| Pangolin | not built | MIT | example viewer only | none |

## What the port must carry / replace

1. Apache-2.0 text + NOTICE for all files translated from RD-VIO (verdict 1); BSD-3 notice (Google Inc. + contributors, endorsement clause)
   for Ceres-derived files; MPL-2.0 headers for Eigen-derived evaluation-order models; Apache-2.0 notice (OpenCV Foundation / Intel et al.)
   for OpenCV-derived kernels (PLAN section 4).
2. No learned components are involved in RD-VIO (GFTT + KLT + ORB-free), so the "no learned methods" rule is met. (RD-VIO's "dynamic
   environment" robustness is IMU-PARSAC, a geometric RANSAC variant.)
3. The `parsac`/`imu_parsac` code is original to RD-VIO (paper: RD-VIO, Li et al., ISMAR 2023); no third-party notice found.
4. Everything above is based on the licence texts in the checkouts, not on web lookups.

## Provenance of this audit

`external/vio3/rd_vio/{LICENSE,README.md,3rd/spdlog/LICENSE,3rd/spdlog/include/spdlog/fmt/bundled/LICENSE.rst}`;
`grep -rn -i "copyright|license|adapted|based on" src` (no hits); `external/vio2/xrslam/{LICENSE,...}`; Eigen 3.3.7
`COPYING.README` + header blocks of `Amd.h` and `SimplicialCholesky_impl.h` (`external/vio2/xrslam/build/_deps/depends-eigen-src`) vs Eigen 3.4.0
(`external/vio/deps/root/usr/include/eigen3`); Ceres 1.14 `CMakeCache.txt` / `config.h` (`CERES_USE_EIGEN_SPARSE`, `CERES_NO_SUITESPARSE`,
`CERES_NO_CXSPARSE`) and Ceres 2.2.0 `LICENSE`; `ldd` of `runs/rdvio_port/reference_build/build/rdvio_ref_driver`.
