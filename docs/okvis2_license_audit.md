# OKVIS2 license audit (phase 0 of the pure-C OKVIS2 port)

Date: 2026-10-01. Scope: everything linked into the mono/stereo VIO + loop-closure path of
`okvis_app_synchronous` as built in `external/vio/okvis2` (pinned upstream commit
`a2ea00688cd10988aae7bd52ab7935ce9a657ec0`, submodule shas in the table below), plus the
runtime dependencies resolved from `external/vio/deps`. Method: read the LICENSE/COPYING files and
every per-file copyright header class (grep, unique-line count), `ldd` of the built binary, the
CMake cache (which optional solvers are on), and Eigen's own `COPYING.README`.

Build configuration that matters for licensing (from `build/CMakeCache.txt`): `SUITESPARSE=OFF`,
`CXSPARSE=OFF`, `EIGENSPARSE=ON`, `USE_NN=OFF`, `BUILD_ROS2=OFF`, `HAVE_LIBREALSENSE=OFF`,
`BUILD_TESTS=OFF`. So no SuiteSparse/CHOLMOD (GPL/LGPL), no libtorch, no ROS, no RealSense SDK.

## Verdict

**No blocker licenses.** Everything in the mono/stereo VIO + loop-closure path is BSD-3-Clause,
Apache-2.0, MPL-2.0, BSD-style (DBoW2, with one unusual clause), MIT, or Boost. No GPL, no LGPL,
no non-commercial software terms were found in linked code. The only non-commercial item is
**data** (EuRoC), not code. Items needing action are listed under "What the port must replace /
watch".

## Table: source dirs of OKVIS2 itself

| Path | LOC (hpp+cpp, no tests) | License | Notes |
|---|---|---|---|
| `okvis_ceres`, `okvis_common`, `okvis_cv`, `okvis_frontend`, `okvis_kinematics`, `okvis_multisensor_processing`, `okvis_util`, `okvis_apps` | see PLAN.md | BSD-3-Clause, (c) 2015 ASL ETH Zurich, (c) 2020 SRL Imperial College London, (c) 2024 SRL TU Munich | Notice must be retained in derived files. Clause 3: names of ASL/ETH, SRL/Imperial, TUM and contributors may not be used to endorse derivatives. README asks (not a license term) to cite the OKVIS2 paper. |
| `okvis_time`, `okvis_timing` | 1,935 | BSD-3-Clause, partly derived from ROS `rostime` ("Willow Garage Inc.", BSD) | Time/Duration semantics (uint32 sec/nsec, normalisation) come from ROS time; both notices stay on any file derived from them. |
| `okvis_frontend/include/opengv/*`, `src/Frame*Adapter.cpp`, `LoopclosureNoncentralAbsoluteAdapter.cpp` | n/a | BSD-3-Clause (OKVIS2 notice), written against OpenGV | OpenGV adapters/sac problems for RANSAC. |
| `okvis_ros2/` | n/a | BSD-3-Clause (OKVIS2) | Not built (`BUILD_ROS2=OFF`); out of scope. |
| `cnn/demo.py`, `resources/fast-scnn.pt` | n/a | script BSD-3; **weights: no license statement in repo** | `USE_NN=OFF`, keypoint classification is not part of the port. Do not ship the weights. |
| `resources/small_voc.yml.gz` | 64 KB | covered by repo BSD-3, **provenance undocumented** | See "DBoW2 vocabulary" below. |
| `resources/meshes/` | n/a | repo BSD-3 | Only used by the ROS/visualiser. Out of scope. |
| `config/*.yaml` | n/a | repo BSD-3 | `euroc.yaml` carries the EuRoC calibration numbers (facts about the dataset's sensors; do not ship dataset-derived config in a product). |

## Table: submodules (`external/`) actually compiled into the binary

| Component | Pinned commit | License | In path? | Notes |
|---|---|---|---|---|
| `brisk` (BRISK + AGAST detector/descriptor, Leutenegger) | `1ef8b42a` | BSD-3-Clause, (c) 2011/2013 ASL ETH, 2020 Imperial, 2024 TUM | yes (detector, descriptor, Hamming) | Codex owns the port of this leaf. Contains AGAST (c) 2010 Elmar Mair, BSD-3 (15 files, `brisk/agast/`), and `include/sse2neon/SSE2NEON.h` (MIT; ARM only, not used on x86). |
| `ceres-solver` | `85331393` (v2.2.0) | BSD-3-Clause (Google); `LICENSE` also carries the Apache-2.0 text for Abseil-derived files (`include/ceres/internal/fixed_array.h`, `memory.h`, "Copyright The Abseil Authors") and an MIT notice for libmv-derived code in `examples/` (not built) | yes (all nonlinear least squares) | Configured `EIGENSPARSE=ON`, no SuiteSparse/CXSparse, and Ceres' own CMake adds `-DEIGEN_MPL2_ONLY`. miniglog is not used (`MINIGLOG=OFF`, real glog). |
| `DBoW2` (Galvez-Lopez) | `3924753d` | **BSD-style with an extra notification clause** (see below) | yes (loop-closure place recognition, `FBrisk` descriptor class in `okvis_frontend`) | Clause 3: "The original author of the work must be notified of any redistribution of source code or in binary form." |
| `opengv` (Kneip) | `91f4b19c` | BSD-3-Clause, (c) 2013 Laurent Kneip, ANU (147 files) | yes (P3P/GP3P absolute pose RANSAC, 2D-2D relative pose, 5-pt) | `python/pybind11` (BSD-3) is not built. OpenGV's optional nonlinear refinement pulls Eigen's *unsupported* `NonLinearOptimization` (MINPACK license, permissive) if instantiated. |
| `googletest` | `58d77fa8` | BSD-3-Clause | no (`BUILD_TESTS=OFF`) | |
| `external/patches/{DBoW2,opengv}/CMakeLists.txt` | n/a | BSD-3 (OKVIS2) | build only | |

## Table: dependencies resolved from `external/vio/deps` (`ldd` of the built binary)

| Dependency | Version | License | Needed by | Port action |
|---|---|---|---|---|
| Eigen | 3.4.0 | MPL-2.0 (headers). `COPYING.README`: some files are BSD or LGPL; in 3.4 the only LGPL file is `IterativeLinearSolvers/IncompleteLUT.h`, which is not on this path (Ceres defines `EIGEN_MPL2_ONLY`). `OrderingMethods/Amd.h` and `SparseCholesky/SimplicialCholesky_impl.h` are CSparse/LDL (T. Davis) code *relicensed to MPL-2.0 by explicit grant to Google*; they are what Ceres' `SPARSE_NORMAL_CHOLESKY` + `EIGEN_SPARSE` uses. | everywhere (Ceres, OKVIS, OpenGV) | Port must reproduce Eigen's numerics (bit-exact) -> files derived from Eigen carry MPL-2.0 (stella_port convention, `SPDX: MPL-2.0` + copyright lines); Eigen-derived files are also where the Amd/LDL numerics come from (stella_port already has `sv_eigen_amd.c`, `sv_eigen_llt.c`, MPL-2.0). |
| OpenCV | 4.6.0 | Apache-2.0 (since 4.5.0); bundled 3rdparty (zlib, libpng, libjpeg, ...) are permissive | `cv::Mat` containers, `imread`/`imwrite`, `cvtColor`, `FileStorage` (loads the DBoW2 vocabulary from YAML), drawing for debug views, features2d interfaces for BRISK | Port: none of this is needed (own image buffer, own vocabulary format). `libopencv_highgui` is a link-only stub in our build (no GUI). |
| glog | 0.6.0 | BSD-3-Clause | logging, `OKVIS_ASSERT` plumbing | replace by plain asserts/stderr |
| gflags | 2.2.2 | BSD-3-Clause | glog flags | none |
| Boost | 1.83.0 (`filesystem` linked; headers elsewhere) | Boost Software License 1.0 | dataset directory listing in apps/readers | none |
| OpenBLAS / LAPACK | 0.3.x (apt) | BSD-3-Clause | linked because `find_package(LAPACK REQUIRED)`; Ceres' default dense algebra is Eigen (`dense_linear_algebra_library_type` EIGEN), so no numerical use on this path | none |
| libunwind | 8.x | MIT | glog stack traces | none |
| libgfortran, libgcc, libstdc++ | GCC 13 | GPL-3.0 **with GCC Runtime Library Exception** | runtime of the reference binary | irrelevant for the C port; the reference build is never distributed |
| liblzma | 5.x | 0BSD / public domain | transitive | none |

## DBoW2 vocabulary (`resources/small_voc.yml.gz`)

* Header of the file: `k: 9, L: 3, scoringType: 0 (L1), weightingType: 0 (TF-IDF)`, 48-byte (384-bit)
  BRISK descriptors per node, 64 KB gzip, stored as an OpenCV `FileStorage` YAML.
* The OKVIS2 repo contains **no statement of who trained it, on which images, or under what
  license** beyond the repo-wide BSD-3. The only commit touching it in the clone is unrelated.
* Consequence for the port (same as stella's vocabulary rule): do **not** ship this file in
  a product. Train an own BRISK-descriptor vocabulary from permissively licensed imagery
  (the stella port already has an own ORB vocabulary pipeline, `stella_port/vocab/own_orb_v1.fbow`,
  reusable for the training method) and keep the DBoW2 *scoring* semantics (L1 norm, TF-IDF, k=9,
  L=3) for compatibility; use `small_voc.yml.gz` only as a reference-run input.
* DBoW2 clause 3 ("original author must be notified of any redistribution of source code or in binary
  form") is unusual. If the C loop-closure module is a translation of DBoW2 code, notify
  Dorian Galvez-Lopez on redistribution and keep the DBoW2 notice. If it is implemented from the
  paper (Galvez-Lopez & Tardos, T-RO 2012) plus black-box behavioural checks, no DBoW2 notice/clause
  applies. Recommended: the second route (also FBoW, MIT, is already vendored for stella).

## What the port must replace / watch

1. **Ceres solver -> own sparse LM / Schur.** Ceres is permissively licensed (BSD-3 + Abseil Apache-2.0
   bits), so porting its algorithms is allowed with notices, but the port will not link it
   (library-free C99). Required behaviour (details in `okvis_port/PLAN.md`): DENSE_SCHUR + DOGLEG for
   the realtime graph, SPARSE_NORMAL_CHOLESKY (Eigen simplicial LDLT with AMD) + DOGLEG for the full
   graph, Cauchy loss, manifold plus `ceres::Problem` bookkeeping. Eigen-derived numerics (Amd.h,
   SimplicialCholesky) are MPL-2.0 files.
2. **DBoW2 vocabulary** -> own trained vocabulary (above), own file format; no OpenCV `FileStorage`.
3. **OpenCV** is replaced by a plain image struct plus PGM/PNG-free fixtures (harness-side); OpenCV
   (Apache-2.0) is allowed as an offline dump tool only.
4. **BRISK** (BSD-3, ETH/Imperial/TUM) + AGAST (BSD-3, Mair) -> Codex-owned leaf; both notices needed on
   derived files. AGAST's notice is a separate copyright holder.
5. **OpenGV** (BSD-3, Kneip) -> reimplement GP3P/absolute-pose and 2D-2D (5-pt, rotation-only) RANSAC;
   keep the ANU/Kneip notice if derived. The stella port already has a 5-point leaf (Codex).
6. **EuRoC data** is "In Copyright - Non-Commercial Use Permitted": never commit images, IMU
   CSVs, ground truth, or dumps derived from them (all stay under gitignored `runs/`). Calibration
   numbers copied from EuRoC into `okvis_mono_euroc*.yaml` are test fixtures only.
7. Do not ship `fast-scnn.pt` (no license statement) or any `USE_NN` code.
8. Attribution to carry in the repo for OKVIS2-derived files: BSD-3 notice naming ASL/ETH Zurich,
   SRL/Imperial College London and SRL/TU Munich (copy of upstream `LICENSE`), plus ROS/Willow Garage
   for Time/Duration-derived code, plus Eigen MPL-2.0 file headers for Eigen-derived numerics.

## Provenance of this audit

Grep of unique license-line classes over `okvis_*`, `external/{brisk,DBoW2,opengv,ceres-solver}`;
`external/ceres-solver/LICENSE` (BSD-3 then Apache-2.0 sections), `external/DBoW2/LICENSE.txt`,
`external/brisk/LICENSE`, `external/opengv/License.txt`, `external/eigen/COPYING.README`, Debian
`copyright` files of the `deps/root` packages, `ldd build/okvis_app_synchronous`. Not legal advice.
