# RD-VIO pure-C port: architecture map, determinism, module order

Upstream: `external/vio3/rd_vio` (`Jianxff/rd_vio` @ `099f5e886ebb9d33ccf0e4b17af7c48c57da69b4`, 2024-04-14), a plain-CMake split of XRSLAM
(`openxrlab/xrslam`). Mono + IMU only (phones: Outdoor-1 4.77 m with XRSLAM's `iphone_slam.yaml`-style settings, indoor 0.8-0.9 m; EuRoC
0.12-0.19 m). Licences: `docs/rdvio_license_audit.md` (no blocker; Apache-2.0 with an unfilled LICENSE template; use Eigen 3.4 not 3.3.7).
Method as `stella_port/` and `okvis_port/` (read `okvis_port/HANDOVER.md`): pristine copy + numbered patches, deterministic single-thread
reference, instrumentation-only dump patches, C99 module + `check_rd_*` harness at tolerance 0, Eigen 3.4.0 evaluation-order models.
LOC = hpp+cpp lines of the pristine copy (no spdlog, no examples).

## 1. Pipeline (what `Handler` runs with `-DTHREADING=OFF`, synchronous)

```
Handler::track_gyroscope / track_accelerometer  -> pairs gyro+accel into ImuData (linear interpolation of the slower stream), queues
Handler::track_camera(image)      -> Frame (image, IMU slice since the previous frame, imu/camera extrinsics), FeatureTracker::track_frame
 FeatureTracker::run (front thread, here inline)
   state INITIALIZING: Frame::track_keypoints  (CLAHE'd image, KLT forward+backward check against the previous frame, detect new GFTT/Harris
        keypoints with Poisson-disk spacing), keep up to max_init_frames frames, Initializer::initialize when enough parallax
   state TRACKING: mirror the sliding-window map's latest keyframe state, KLT with IMU-predicted start points (predict_keypoints), new keypoints,
        hand the Frame to the Frontend (`FeatureTracker::solve_pnp` is only a one-frame Ceres pose refinement)
 Frontend::run (back thread, here inline): SlidingWindowTracker::track per frame
   localize_newframe   (Ceres: frame pose+motion vs IMU prior + reprojection priors of mapped tracks)
   manage_keyframe     (keyframe vs sub-frame decision; rotation-only "R" frames handled via FT_NO_TRANSLATION lifting/merging)
   [keyframe] track_landmark (triangulate new tracks) -> refine_window (Ceres: window states, inverse-depth landmarks, marginalisation
        prior, IMU factors, reprojection factors) -> slide_window (Schur marginalisation of the oldest frame)
   [sub-frame] refine_subwindow (small Ceres problem, first frame fixed)
   [parsac_flag] judge_track_status / update_track_status: IMU-PARSAC dynamic-feature rejection
Initializer: keyframes -> essential/homography/PnP SfM (init_sfm) -> init_imu (gyro-bias least squares, gravity/scale/velocity linear
        solve, refinement, apply)
```
Output = body (IMU) pose after `get_latest_state` (with IMU propagation to the image time); no loop closure, no relocalisation.

## 2. Modules in dependency order

Eigen cols: which Eigen features the module needs. "bit-exact OpenCV" marks calls that must be reproduced from OpenCV's own sources.

| # | Module | Upstream | LOC | Eigen / Ceres / OpenCV used | C port size (est.) | Reuse |
|---|---|---|---|---|---|---|
| M1 | **IMU pre-integration + residual/Jacobians + quaternion manifold + Lie helpers (DONE, bit-exact)** | `estimation/preintegrator.{h,cpp}`, `ceres/preintegration_factor.h`, `ceres/quaternion_parameterization.h`, `geometry/lie_algebra.{h,cpp}` | 114+201+29+94 | Quaternion/AngleAxis, 3x3 lazy products, 9x9 and 15x15 GEMM, `Matrix15::inverse()` (PartialPivLU, unblocked) + `LLT<15>`, GEMV, `SizedCostFunction<15,...>` | ~900 (rd_imu 330, rd_lie 170, rd_eigen 100 + harness) | `ok_eigen` (quat product/normalize, 3x3 rules, `ok_gemm`), `ok_dense` (`ok_gebp`, `ok_llt_lower`, `ok_gemv_col`); new: PartialPivLU unblocked + `triangular_solve_matrix<OnTheLeft>` |
| M2 (DONE, bit-exact; pnp.h deferred to M8) | Camera / reprojection residuals, rotation prior, S2 tangent basis, stereo/essential/homography/Wahba geometry | `ceres/reprojection_factor.h`, `rotation_factor.h`, `geometry/{stereo,essential,homography,wahba,pnp}` | 125+69+186+95+299+158+29+206 | `JacobiSVD<3x3>`, `EigenSolver<10x10>` (5-pt Grobner action matrix), `dproj_dp`, `hnormalized`, OpenCV `solvePnP(EPNP)` + `Rodrigues` (bit-exact, see 4) | ~1,400 | stella `sv_eigen_svd` (JacobiSVD 3x3), `sv_eigen_eigensolver` (10x10 real EigenSolver), `rd_lie` |
| M3 (DONE, bit-exact; IMU_PARSAC deferred to M8) | Utilities: RANSAC / PARSAC / IMU-PARSAC, RNG, Poisson-disk filter, tags | `util/{ransac,parsac,imu_parsac,random,poisson_disk_filter,tag}.h`, `extra/poisson_disk_filter.h` | 103+379+413+172+115+93 | libstdc++ `default_random_engine` (= `minstd_rand0`) + `uniform_int/real_distribution` semantics, glibc `rand()/srand(0)`, `std::hash<int>` in `unordered_map` (order not iterated) | ~900 | stella `check_sv_rng` (mt19937 + libstdc++ `uniform_int_distribution`; here `minstd_rand0` is simpler: x = 16807 x mod 2147483647, `generate_canonical` for the real distribution) |
| M4 | Solver: Ceres 2.2 SPARSE_SCHUR + traditional DOGLEG, Cauchy(1.0) loss, quaternion manifold, `Problem` bookkeeping | `estimation/solver.{h,cpp}`, `ceres/marginalization_factor.h` (dynamic-size cost function) | 218+72 (+Ceres ~2.4 k C) | Ceres 2.2.0 (reference build), EigenSparse `SimplicialLDLT` + AMD, Schur eliminator on mixed block sizes (E-block = 1-D inverse depth, F-blocks = 4/3/3/3/3 per frame) | ~1,500 new + reuse | `ok_solve`/`ok_blas`/`ok_dense`/`ok_sparse`/`ok_amd`/`ok_problem` (Dogleg, Jacobi scaling, Cauchy corrector, Problem order, LDLT+AMD). **New**: SPARSE_SCHUR path (`SparseSchurComplementSolver`: `BlockRandomAccessSparseMatrix` reduced system -> compressed row -> Eigen simplicial LDLT with AMD), `ceres::Manifold` Plus/PlusJacobian only |
| M5 | Marginalisation factor | `ceres/marginalization_factor.h` (478), `estimation/marginalization_factor.h` | 521 | dynamic `Matrix`, `std::map<Track*,...,compare>` ordering by track id, `SelfAdjointEigenSolver<Dynamic>` (threshold 1e-8, `sqrt_inv_cov = sqrt(lambda) V^T`), `inverse()` of 3x3/1x1 blocks, dynamic GEMM/GEMV | ~800 | `ok_selfadjoint_eig(n<=32)` (n up to 15*12 = 180: **needs the dynamic-size path**: blocked tridiagonalisation, column-major symv, `ok_dense` GEMV/GEBP); `ok_dense` |
| M6 | Map: Frame, Track, Map (keypoints, tags, first-observation bookkeeping, triangulation, landmark point) | `map/*` | 194+84+103+83+125+63 | `Track::triangulate` (DLT via `JacobiSVD`? see source), linear solves | ~700 | `rd_lie`, M2 |
| M7 | Front end A: image processing (bit-exact OpenCV) | `extra/opencv_image.{h,cpp}`, `map/frame.cpp` (detect/track keypoints) | 210+59+194 | `createCLAHE(6.0, 8x8).apply`, `buildOpticalFlowPyramid(winSize 21, 3 levels, withDerivatives)`, `calcOpticalFlowPyrLK` (forward + reverse, `OPTFLOW_USE_INITIAL_FLOW`, 30 iters, eps 0.01), `GFTTDetector(max_pts, 1e-3, minDist 20, block 3, Harris)` (= `cornerHarris`/`cornerMinEigenVal` + `goodFeaturesToTrack` sort + min-distance grid), `cv::norm` | ~2,000 | none (stella has ORB/FAST only) |
| M8 | Front end B: feature tracker (keyframe mirroring, IMU-predicted KLT start points, Ceres pose refinement) | `rdvio/feature_tracker.{h,cpp}`, `frontend.{h,cpp}` | 274+47+97+40 | `solvePnP` EPNP (bit-exact), M3 RANSAC | ~700 | M2, M3 |
| M9 | Initializer | `rdvio/initializer.{h,cpp}` | 555+42 | `JacobiSVD<3x3>` (gyro-bias normal equations), linear gravity/scale solve (dynamic `Matrix` + `colPivHouseholderQr`/`ldlt`? to be confirmed), PnP, triangulation, Ceres BA | ~900 | M2, M4, M6 |
| M10 | Sliding window tracker, IMU-PARSAC status logic | `rdvio/sliding_window_tracker.{h,cpp}` | 784+56 | fixed small linear algebra, M3 PARSAC, M4 solves | ~1,000 | M1-M6 |
| M11 | Handler / Config / YAML | `handler.cpp`, `config.cpp`, `types.h`, `extra/yaml_config` | 227+207+185+501 | none (own minimal key=value config, IMU stream pairing) | ~600 | - |

Total estimate: ~11 k C99 lines (OKVIS2 port: ~12 k with M7 BRISK/DBoW2 excluded). Largest risks, in order: (1) OpenCV LK / Harris / CLAHE bit-exactness
(runtime CPU dispatch: the reference must be pinned with `OPENCV_CPU_DISABLE`, see section 3); (2) dynamic-size `SelfAdjointEigenSolver` for the
marginalisation (up to 180x180); (3) the SPARSE_SCHUR numerics in Ceres 2.2.0 (not covered by `ok_solve`, which does DENSE_SCHUR and
SPARSE_NORMAL_CHOLESKY).

## 3. Deterministic reference (done)

`tools/build_rdvio_reference.py`: `git archive HEAD` of `external/vio3/rd_vio` (pristine, no local build fixes) -> `runs/rdvio_port/reference_build/src`,
patches `rdvio_port/reference/patches/`, then CMake `-DTHREADING=OFF`, `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`, against **Ceres 2.2.0
(pristine okvis2 submodule, built by `reference_build/build_ceres.sh`, EIGENSPARSE on, SuiteSparse/CXSparse off) + Eigen 3.4.0 + OpenCV 4.6.0**.

| patch | content |
|---|---|
| `0001-release-build-no-fastmath.patch` | stock CMake forces `Debug`, `-Og -ffast-math -msse3 -mtune=native` (fast-math: non-reproducible and not portable to a bit-exact C port) -> Release `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`; examples (Pangolin viewer) not built; static library; `#include <optional>` (gcc 13) |
| `0002-ceres22-manifold.patch` | Ceres 1.14 `LocalParameterization` -> Ceres 2.2 `Manifold` (Plus, PlusJacobian = [I;0], Minus/MinusJacobian only for completeness), `manifold_ownership` |
| `0003-preintegration-dump.patch` | M1 instrumentation: `RDVIO_PORT_DUMP_DIR`, `RDVIO_PORT_DUMP_EVERY="integ=1,pred=1,pie=20,plus=50"`: `PreIntegrator::integrate` (inputs, outputs), `predict`, `CeresPreIntegrationErrorFactor::Evaluate` (+ prior flag), `QuaternionParameterization::Plus`; no numerical effect (the dumped run's trajectory is byte-identical to the undumped run) |

Patches 0004 (M2 factor / geometry dumps) and 0005 (M3 RANSAC / PARSAC / Poisson op-stream dumps) are instrumentation only, like 0003 (see `reference/README.md`).

Driver `rdvio_port/reference/driver/rdvio_ref_driver.cpp` (undistorts with OpenCV `initUndistortRectifyMap`+`remap` -- the library never applies the
distortion --, `%.17g` output). Configs `rdvio_port/reference/configs/{euroc_sensor,setting}.yaml` = upstream `configs/` (IMU-PARSAC on, window 12).
Runner `tools/run_rdvio_reference.py` (score with `external/gnss/venv/bin/python`: needs numpy).

Result (EuRoC MH_01_easy, 3682 frames, ~2 min/run): trajectories **byte-identical** across 4 reference runs (plain x2, `MALLOC_PERTURB_=165`,
`setarch -R`), sha256 `f0d60a3e03c1...`. Stock binary (Ceres 1.14 + Eigen 3.3.7, fast-math, `-Og`) is also self-deterministic (2 runs identical):
ATE SE3 0.1656 m, Sim3 0.1639 m; reference (Ceres 2.2 + Eigen 3.4, no fast-math) ATE SE3 0.1514 m, Sim3 0.1491 m, scale 1.006, 3633/3682 poses.
Determinism holes found: **none in the synchronous build.** Things that looked suspicious and are fine: `RandomBase` seeds from `std::random_device`
but every RANSAC/PARSAC call re-seeds explicitly (`config->random()` = 648, default 0); `Sampler`/`ImuParsac` use glibc `srand(0)`/`rand()` (global state,
deterministic single-threaded; **a port must reproduce glibc's TYPE_3 additive-feedback generator for `rand()`**); `unordered_set<Track*>` / `unordered_map<Frame*,...>`
are only probed, never iterated; Ceres `Problem` ordering follows `AddParameterBlock` order, not pointer order. Real hole outside the algorithm:
the stock `THREADING=ON` build (two sleeping worker threads, `max_solver_time`) is timing dependent and was not used.
OpenCV runtime CPU dispatch (AVX2/FMA kernels in LK/Harris/CLAHE could differ from an SSE2 C port): a run with `OPENCV_CPU_DISABLE=AVX2,FMA3,AVX,AVX512_*,SSE4_2,SSE4_1,SSSE3,POPCNT,SSE3` gives the **same trajectory hash**
(so the OpenCV paths used here are CPU-feature independent, or the variable did not take effect: not verified with `cv::getCPUFeaturesLine`; re-check before porting M7).

## 4. OpenCV parts that must be ported bit-exact (OpenCV 4.6.0 is Apache-2.0; port from its sources, keep its notice)

| call | OpenCV source | notes |
|---|---|---|
| `createCLAHE(clip 6.0, 8x8).apply(image, image)` | imgproc/src/clahe.cpp | integer histogram / LUT, bilinear interpolation of 8-bit LUTs (integer + float weights); fully deterministic, no SIMD float |
| `buildOpticalFlowPyramid(img, pyr, 21x21, maxLevel 3, withDerivatives=true, BORDER_REFLECT_101, BORDER_CONSTANT)` | video/src/lkpyramid.cpp | `pyrDown` (5-tap binomial, integer) + `calcScharrDeriv` (int16) with 2 px border padding |
| `calcOpticalFlowPyrLK` x2 (forward, reverse), `OPTFLOW_USE_INITIAL_FLOW`, criteria 30 / 0.01 | video/src/lkpyramid.cpp (`LKTrackerInvoker`) | the float accumulation order of the 21x21 window sums is what to match (SIMD lanes: reproduce the 4-lane/8-lane partial sums of the baseline path) |
| `GFTTDetector(N, 1e-3, 20, 3, useHarris)` -> `goodFeaturesToTrack` | imgproc/src/featureselect.cpp + `cornerHarris` (corner.cpp, `Sobel`/`boxFilter` float) | candidate threshold = quality * max response, sort by response (`std::sort`, **implementation-defined order for ties**: reproduce libstdc++ introsort), min-distance grid |
| `solvePnP(..., SOLVEPNP_EPNP)` (4 and 6 points), `Rodrigues` | calib3d/src/epnp.cpp, solvepnp.cpp, calibration.cpp | EPnP uses `cv::SVD` / Jacobi eigen (own code, not Eigen). **Only reached through `find_pnp_matrix_parsac_imu` in `SlidingWindowTracker::judge_track_status`, i.e. only with `parsac_flag: true`** (EuRoC `setting.yaml`; the XRSLAM iPhone setting has it false, so the phone configuration does not need it) |
| `cv::norm(Point2f)`, `cv::imread` (PNG) | core / imgcodecs | `imread` and `remap` (undistortion) are harness/driver-side: the port's input is the undistorted 8-bit image (dump it as a PGM fixture) |

## 5. Ceres usage (what M4 reproduces)

* Problem per call of `Solver::solve`: `Problem::Options` with DO_NOT_TAKE_OWNERSHIP; quaternion blocks (4) with the quaternion manifold, p/v/bg/ba (3) plain,
  inverse depth (1); fixed blocks via `SetParameterBlockConstant` (`FT_FIX_POSE`, `FT_FIX_MOTION`).
* Options: `linear_solver_type = SPARSE_SCHUR`, `trust_region_strategy_type = DOGLEG` (default TRADITIONAL_DOGLEG), `max_num_iterations = 30` (setting
  `solver.iteration_limit`), `max_solver_time_in_seconds = 1e6`, `num_threads = 1`, `update_state_every_iteration = true`; all tolerances Ceres defaults.
* Loss: `CauchyLoss(1.0)` on reprojection and reprojection-prior and rotation-prior residuals (after `sqrt_inv_cov` whitening, i.e. 1 sigma), none on IMU and marginalisation.
* Cost functions: `SizedCostFunction` for the factors (analytic Jacobians written wrt the **ambient** quaternion with a zero 4th column and `PlusJacobian = [I; 0]`,
  see M1), `CeresMarginalizationFactor : CostFunction` with a runtime residual/parameter-block layout.
* Block structure for the Schur elimination: inverse depths (1) are the E blocks; with `Problem` order frames first then tracks, Ceres' `ComputeStableSchurOrdering`
  / maximal-independent-set ordering puts them first (to be measured with a snapshot dump, as okvis_port patch 0008).

## 6. Why it scores what it scores (implementation facts the paper does not stress; from source reading, to be confirmed by experiments)

* **Reprojection error is measured on the tangent plane of the observed bearing** (`local_tangent` = S2 basis + the bearing itself, `hnormalized`), not on the image
  plane: well conditioned for wide FOV and independent of intrinsics; the keypoint noise (default 0.5 px / focal) is applied through `sqrt_inv_cov`, so the robust loss
  `Cauchy(1.0)` is a **1-sigma** knee. (Hard-coded, "TODO make configurable".)
* **IMU factor is re-integrated for every window solve** with the current bias estimate of the first frame (`keyframe_preintegration.integrate(..., frame_i->motion.bg, ba, true, true)`),
  and keeps the first-order bias Jacobians as well; covariance via continuous noise * `1/dt` discretisation, `dt` floored at 1e-7. The residual whitening is the exact
  `LLT` of the **inverse** of the propagated 15x15 covariance (not a diagonal approximation); bias random walk adds `cov * dt`.
* **Rotation-only sub-frames** (`FT_NO_TRANSLATION`, "R" frames): while the camera only rotates, frames are not promoted to keyframes (no baseline for triangulation); they
  are merged in blocks of three and refined in a tiny problem with the keyframe fixed and **rotation-prior factors** for untriangulated tracks. This is what lets the
  system survive the static/rotating starts that break the equivariant filters.
* **Landmarks are anchored in their first keyframe** (inverse depth, first-observation frame as reference) and only tracks whose first frame is a keyframe get window factors;
  untriangulated or invalid tracks are trashed after each refine (`rpe >= 3 px`, depth outside (1e-3, 50) m).
* **Marginalisation keeps linearisation points** (`pose_linearization_point`, `motion_linearization_point`: first-estimate-style prior) and uses an eigen-decomposition
  with a hard 1e-8 eigenvalue cut for the square-root information of the prior.
* **Image side**: CLAHE (clip 6, 8x8 tiles) on every frame before KLT -- the single biggest lever for the dark/low-contrast phone and EuRoC frames; KLT is checked both
  forward and backward (0.5 px), displacement above rows/4 rejected, 20 px border rejected; Harris-scored GFTT with a Poisson-disk minimum distance
  (`feature_tracker.min_keypoint_distance`: 10 px EuRoC `setting.yaml`, 25 px XRSLAM iPhone settings), max 200 keypoints.
* **Settings that matter** (per the earlier phone study): window 12 + sub-frame 5 + `force_keyframe_landmarks` 50 (`setting.yaml`) vs window 10 / 3 / 35 (iPhone settings
  gave 4.77 m on Outdoor-1 where the EuRoC-style settings collapsed); IMU noise densities taken from the sensor yaml as continuous covariances.
* **No timing logic in the algorithm**: `solver_time_limit` is 1e6 s, so results do not depend on machine speed (unlike OKVIS2's realtime pacing).
* Gravity is a fixed world-z constant 9.80665 m/s^2 (no gravity-direction state after initialisation): the world frame is gravity aligned by the initializer.
