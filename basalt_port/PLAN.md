# Basalt pure-C port: survey, licences, module plan

Groundwork + module M0 done (2026-10-07): reference build, determinism evidence and the dump patch are in `basalt_port/reference/README.md`;
the M0 answers are section 4, the dump layouts section 5. No port code yet beyond the dump reader `basalt_port/c/check_bs_dump.c`.
Method as for `okvis_port/PLAN.md` and `rdvio_port/PLAN.md`: deterministic single-threaded reference, observe-only dump
patches, C replay harnesses at tolerance 0.

## 1. Source and licences

Source: official repo `https://gitlab.com/VladyslavUsenko/basalt.git`, master `0f3b2b52` (2026-03-22), version string 0.1.7,
fetched shallow with submodules (the only submodule is `thirdparty/vcpkg`; this master no longer vendors basalt-headers, Sophus,
Pangolin, opengv as submodules but pulls them through vcpkg overlay ports, whose pinned refs are the checkouts below,
`external/vio/basalt_src/_deps`). Disk: basalt_src 137 MB, runs/basalt_port 60 MB; `/` has 25 GB free.

| component | version / ref | licence | on the VIO path? | port can copy? |
|---|---|---|---|---|
| Basalt (VladyslavUsenko/basalt) | master 0f3b2b52 | BSD-3-Clause (LICENSE, (c) 2019 Usenko, Demmel) | yes | yes (keep notice) |
| basalt-headers (camera, IMU preintegration, spline, image, Sophus utils) | aa441ba3 | BSD-3-Clause | yes | yes |
| Sophus | 1.24.6 (d0b7315a) | MIT | yes (SO3/SE3/SE2) | yes |
| Eigen | 3.4.0 used here; vcpkg baseline 5.0.1 | MPL-2.0 | yes (all dense algebra) | MPL-2.0 files stay MPL-2.0 (as for stella/okvis ports) |
| oneTBB | 2021.11 used; baseline 2022.3 | Apache-2.0 | yes (queues, parallel_for/reduce) | not ported (serial) |
| OpenCV | 4.6.0 used; baseline opencv4 | Apache-2.0 (4.5+) | `cv::imread`, `cv::FAST` only | reimplement FAST, PNG decode (`okvis_port/reference_png/ok_png.c` exists) |
| cereal | 1.3.2 (ebef1e92) | BSD-3-Clause | config/calib JSON only | no (own parser) |
| magic_enum | 0.9.7 (e046b69a) | MIT | enum <-> string in vio_config only | no |
| nlohmann-json | 3.11.3 (3.12.0 baseline) | MIT | time_utils stats only | no |
| fmt | 9.1.0 (12.1.0 baseline) | MIT | logging only | no |
| opengv | 91f4b19c | BSD-3-Clause style (Kneip, ANU) | **not** on the VIO path (mapper only) | existing `okvis_port/c/ok_opengv.c` if the mapper is ever wanted |
| ethz_apriltag2 (`thirdparty/apriltag`) | in-tree | BSD-3 style (ETH ASL, Skybotix) | calibration only | no |
| Pangolin | 0.9.4 (baseline) | MIT | GUI only, not built | no |
| CLI11 | 2.6.2 baseline | BSD-3-Clause | executables only | no |
| Boost, lz4, bzip2, ros_comm / roscpp_core (rosbag) | baseline | BSL-1.0 / BSD-2 / bzip2 / BSD | rosbag input only, patched out | no |
| RealSense, vcpkg | - | Apache-2.0, MIT | T265 live executables only | no |

Everything on the VIO path is permissive (BSD-3 / MIT / MPL-2.0 / Apache-2.0): no GPL, unlike ORB-SLAM3/stella-adjacent code.
Reading and porting Basalt itself is unconstrained; keep the BSD-3 notices in `basalt_port/LICENSES` and `NOTICE` as the other ports do (not created yet).

## 2. What the EuRoC stereo VIO run actually executes

`euroc_config.json`: optical flow `frame_to_frame`, pattern 51, 3 pyramid levels, 5 iterations, grid 50; `vio_linearization_type
ABS_QR`, `vio_sqrt_marg true`, `vio_use_lm true`, `vio_max_states 3`, `vio_max_kfs 7`, `vio_max_iterations 7`, jacobian scaling off.
Camera model `ds` (double sphere) for both cameras, 752x480 8-bit PNG widened to uint16 (`<< 8`). **Float** (`Scalar = float`)
estimator by default (`--use-double 0`); the frontend is always `float`; the IMU stream, calibration load and state output are
double. Per frame: image -> `FrameToFrameOpticalFlow` thread (pyramid, track previous patches left cam, track left->right,
epipolar filter, detect new FAST corners in empty grid cells, `OpticalFlowResult`) -> `SqrtKeypointVioEstimator` thread
(IMU preintegration between frames, `measure`: add state, keypoint/landmark bookkeeping and triangulation, `optimize` = LM loop with
QR landmark elimination, dense `LDLT` of the reduced H, back-substitution, `marginalize` = sqrt-form QR marginalisation prior).

### Module table (upstream LOC; "path" = executed with the config above)

| area | files | LOC | path? | notes |
|---|---|---|---|---|
| Frontend flow | `optical_flow/frame_to_frame_optical_flow.h` 408, `patch.h` 226, `patterns.h` 171, `optical_flow.h/.cpp` 219 | 1,024 | yes | float SE2 inverse-compositional patch tracking over a 3-level pyramid; `SE2::exp`, 3x3 solve (`.ldlt`/inverse of small fixed matrices), epipolar check via `calib.unproject` + essential matrix. `patch_optical_flow.h` 413 and `multiscale_...` 471: other types, not default |
| Keypoint detection | `utils/keypoints.cpp` `detectKeypoints` (~90 of 429), `keypoints.h` | ~120 | yes | per-50x50-cell `cv::FAST` (thresholds 40,20,10,5), sort by response (`std::sort`, not stable: tie order is libstdc++ introsort-specific), bounds check. Rest of the file (ORB-like angles/descriptors, matching, opengv RANSAC) is mapper-only |
| Images | `basalt-headers image/image.h` 958, `image_pyr.h` 200, `dataset_io_euroc.h` 316, `dataset_io.h` | ~700 used | yes | pyramid `subsample` (integer 2x2 average, uint16), bilinear `interp` + gradient in float, EuRoC CSV + PNG loader |
| Camera / calibration | `camera/double_sphere_camera.hpp` 448, `calibration.hpp` 194, `calib_bias.hpp` 219, `stereographic_param.hpp` 155 | ~1,000 | yes | `project`/`unproject` + Jacobians in float and double; stereographic landmark direction parametrisation; other camera models only if another dataset is wanted (pinhole 324, kb4 531, ...) |
| Lie / utils | Sophus 1.24.6 (SO3/SE3/SE2 exp, log, mult, Jr), `sophus_utils.hpp` 538, `eigen_utils`, `hash.h` | ~1,500 used | yes | closed forms in `Scalar`; `sin`/`cos`/`sqrt`/`atan2` libm calls (match glibc as in the other ports) |
| IMU | `imu/preintegration.h` 373, `imu/imu_types.h` 283, `utils/imu_types.h` 389 | ~1,000 | yes | `IntegratedImuMeasurement<Scalar>`: integration with bias-Jacobians (exact scheme to be read in M3), `predictState`, `residual` + Jacobians, `popFromImuDataQueue` interpolation |
| Estimator | `sqrt_keypoint_vio.cpp` 1,460 + `.h` 261, `sqrt_ba_base.cpp` 270 + `.h` 124, `ba_base.cpp` 604 + `.h` 167, `landmark_database.cpp` 247 + `.h` 156, `vio_estimator.*` 318, `utils/ba_utils.h` 146, `common_types.h` 319, `vio_config.*` 333 | ~4,400 | yes | state/frame/keyframe sets (`std::map`/`aligned_map`, ordered), `measure`, `addVisionData`, keyframe + marginalisation policy (`marginalize`: pick frames/kfs to drop by `vio_kf_marg_feature_ratio`, overlap score), `optimize` LM (lambda update, step check, `vio_use_lm`), `computeError`, `computeProjections`, nullspace/Jacobian scaling (off), marg-prior bookkeeping |
| Linearisation (ABS_QR) | `linearization_abs_qr.cpp` 711 + `.hpp` 141, `landmark_block_abs_dynamic.hpp` 585, `landmark_block.cpp/.hpp` 203, `imu_block.hpp` 235, `block_diagonal.hpp` 89, `linearization_base.*` 182, `accumulator.h` 281 | ~2,400 | yes | per-landmark Householder QR (`makeHouseholder`/`applyHouseholderOnTheLeft` in `landmark_block_abs_dynamic.hpp:467`), Huber weights, absolute-pose (not relative) frame representation, Schur-free "sqrt" reduced system, `get_dense_H_b` (dense `H = J^T J` accumulate), back-substitution; 11 `tbb::parallel_*` sites (serial in the reference, see below) |
| Marginalisation | `marg_helper.cpp` 361 + `.h` 68 | 429 | yes | `marginalizeHelperSqrtToSqrt` (hand-written column-pivot-free Householder QR on dynamic float matrix, `rank_threshold = sqrt(eps)`), `SqToSq` / `SqToSqrt` use `.ldlt`, `.jacobiSvd`, `.completeOrthogonalDecomposition`, `.colPivHouseholderQr`, `.inverse` (with the euroc config only `SqrtToSqrt` runs: 3678 of 3678 marginalisations, M0 dump, section 4a) |
| Not on the path | `sqrt_keypoint_vo.cpp` 1,340 + `.h` 251 (`--use-imu 0`; diverges on EuRoC per `docs/vio_candidates_20261001.md`), `sc_ba_base` 808 + 452, `linearization_{abs,rel}_sc` 863 + 285 (other `LinearizationType`s), `nfr_mapper` 758 (mapper), `calibration/*`, splines, `marg_data_io` 226, GUI/devices | ~8,000 | no | skip; `SelfAdjointEigenSolver` appears only in `sqrt_ba_base.cpp:247` / `sc_ba_base.cpp` debug checks (vio_debug) |

Executed-path total ~10.5 k upstream lines (including the header-only basalt-headers and Sophus parts used), (smaller than the earlier ports' sources, which also needed Ceres/OpenCV); estimate ~5-6 k lines of C. Basalt has no loop closure / relocalisation on this path (VIO only),
no Ceres, no external optimiser: its own LM + QR + dense LDLT.

### Floating-point and numerics (the actual porting risk)

* **Float throughout** the estimator and frontend (Scalar float, only calibration/IMU input/state output double). Eigen's
  fixed-size float kernels (2/3/4/6/9/15-sized products, SSE2 packet paths without `-march`) and dynamic float GEMM/GEBP,
  GEMV, triangular solves and `LDLT` (unblocked, diagonal pivoting) must be modelled bit-exactly. The earlier ports measured
  Eigen **3.4.0 double** product orders (`okvis_port/c/ok_dense.c`, `eigen_product_modes_test.cc`); the float packet size is 4
  instead of 2, so the models need a float variant. This is the largest new piece of work.
* Dense decompositions: `LDLT` of the reduced H (dynamic, about 90x90 (7 keyframes x 6 + 3 states x 15 pose/vel/bias dof, before marginalisation-prior rows) at max_kfs 7 + max_states 3), Householder QR (own code in
  `marg_helper` and Eigen `makeHouseholder`/`applyHouseholderOnTheLeft` in the landmark block), 3x3 / 2x2 fixed inverses and
  solves (patch tracking, triangulation). No SVD / EVD on the default path (verify with the dump).
* TBB: reference runs with parallelism 1; measured (see reference README) that `parallel_reduce` is then a sequential in-order
  accumulation with no split/join, `parallel_for` in index order. The port writes serial loops with the same accumulation order.
  The estimator state containers are ordered (`std::map`); the frontend's `tbb::concurrent_unordered_map` is copied into an ordered map
  (no hazard); the estimator's libstdc++ `std::unordered_*` iteration order does feed float summation order (section 4b).
* `std::sort` on FAST responses (ties), `std::unordered_*` in `landmark_database.cpp` / `measure` (iteration order matters, model it: section 4b),
  `std::mt19937` only in the unused opengv path. ODR hazard in `compute_sqrt_cov_inv` (float vs double `sqrt`): section 4d.
* `cv::FAST` (OpenCV 4.6: FAST_9_16 with non-max suppression and the `cv::KeyPoint` response = threshold-search score) and OpenCV's
  PNG decode are external numerics: need a C reimplementation validated against OpenCV at tolerance 0 (random + real images),
  like BRISK in the OKVIS port.
* `libm`: `sinf/cosf/sqrtf/atan2f` vs double variants inside Sophus templates; test against glibc 2.39 as the other ports did.

## 3. Proposed modules (our style: one validated step each, replay harness at tolerance 0)

Effort is a rough estimate in agent-days of focused work, calibrated on the earlier ports (RD-VIO was done in about seven module steps).

| step | scope | validation | C files (proposed) | effort |
|---|---|---|---|---|
| M0 (done 2026-10-07) | reference harness: observe-only dump patch `0003-port-dump-instrumentation` (flow, IMU preintegration, per-LM-step digests, marginalisation, executed-path counters), C99 reader `check_bs_dump.c`, runner `tools/check_basalt_port.py`; iteration-order audit; path audit | trajectory sha256 `a7d3e7a3...` with the dump off and on (verified); reader parses 41,887 records of the full MH_01 dump with 0 layout violations | `basalt_port/c/check_bs_dump.c`, `reference/patches/0003` | 1.5 (done) |
| M1 | numeric substrate: Eigen float evaluation-order models (fixed small products, dynamic GEMM/GEMV/triangular/LDLT/Householder), Sophus SO3/SE3/SE2, libm audit | random tests against the real Eigen/Sophus (float + double) | `bs_eigen.{h,c}`, `bs_lie.{h,c}` | 4 |
| M2 | calibration + double-sphere camera (project/unproject + Jacobians, float and double), stereographic param, config/calib JSON reader | dump replay of every projection in a run + random tests vs real class | `bs_cam.{h,c}`, `bs_config.c` | 1.5 |
| M3 | IMU: `IntegratedImuMeasurement` (integration, bias Jacobians, `predictState`, `residual`), `imu_block` Jacobians | replay of every preintegration + residual record | `bs_imu.{h,c}` | 1.5 |
| M4 | images: EuRoC loader, PNG decode (reuse `ok_png`), `subsample` pyramid, bilinear interp/gradient, cv::FAST + response, `detectKeypoints` incl. `std::sort` tie order | FAST/pyramid vs OpenCV on all MH_01 images; `detectKeypoints` replay | `bs_image.{h,c}`, `bs_fast.c` | 3 |
| M5 | optical-flow frontend: patch (pattern 51, float), `trackPoint(AtLevel)`, `trackPoints`, `addPoints`, `filterPoints`, epipolar filter, track-id bookkeeping, `OpticalFlowResult` | replay each frame's `OpticalFlowResult` (observations, ids, positions) bitwise | `bs_flow.{h,c}` | 3 |
| M6 | landmark database (incl. a model of libstdc++ `unordered_map`/`unordered_set` node order for `kpts`, `observations`, `unconnected_obs0`, checked first against `lm_order_hash`/`host_order_hash`) + ABS_QR linearisation: landmark block (Householder), pose/IMU blocks, Huber, `get_dense_H_b`, back-substitute, `computeError`/`computeProjections`, serial accumulation order | replay per-iteration `H,b`, error, inc bitwise | `bs_lin.{h,c}`, `bs_ldb.{h,c}` | 4 |
| M7 | LM `optimize` + dense LDLT solve (float Eigen model from M1), step acceptance, lambda update | per-iteration state/cost replay | `bs_opt.c` | 2 |
| M8 | marginalisation (`marginalizeHelperSqrtToSqrt`, prior bookkeeping, frame/keyframe selection policy in `marginalize`/`measure`) | replay marg prior + kf set per frame | `bs_marg.{h,c}`, `bs_vio.{h,c}` | 3 |
| M9 | system app: threadless driver (frontend then estimator per frame, same FIFO semantics), TUM output; byte-identical trajectory on all 11 EuRoC sequences + profile | full-trajectory hash equality vs reference (the end criterion used for RD-VIO) | `basalt_app.c` | 2 |
| optional | double-estimator (`--use-double 1`), VO mode, other camera models, mapper/opengv | only if wanted | | 3+ |

Total ~25 agent-days on this estimate; M1 (float Eigen) and M4 (FAST/PNG vs OpenCV) carry the uncertainty. Basalt's absolute
runtime in the reference is 61-62 s for MH_01 single-threaded -O2 (RTF 0.34), so replay validation of all 11 sequences is
affordable.

## 4. M0 answers (evidence: dump `runs/basalt_port/dumps/all`, counters below are per full MH_01_easy run, 3682 frames)

### 4a. Which linearisation / marginalisation path runs

Code reading (`sqrt_keypoint_vio.cpp`, `linearization_base.cpp`, `marg_helper.cpp`) and the executed-path counters of the instrumented run agree:

| fact | code | counter (MH_01) |
|---|---|---|
| estimator Scalar | `--use-double 0` default -> `SqrtKeypointVioEstimator<float>` | est_ctor_float 1, est_ctor_double 0 |
| linearisation | `vio_linearization_type ABS_QR`; `LinearizationBase::create` | lin_create_abs_qr 7356 (3678 `optimize` + 3678 `marginalize`), abs_sc 0, rel_sc 0 |
| marginalisation helper | `is_lin_sqrt = isLinearizationSqrt(ABS_QR) = true` and `marg_data.is_sqrt` (vio_sqrt_marg) -> `get_dense_Q2Jp_Q2r` + `MargHelper::marginalizeHelperSqrtToSqrt` | marg_helper_sqrt_to_sqrt 3678, marg_helper_sq_to_sqrt 0, marg_helper_sq_to_sq 0 |
| LM loop | `vio_use_lm`; dense `Eigen::LDLT<Ref<MatX>>` of `H + diag(max(lambda*diag, 1e-6))` | optimize_run 3678 (first 4 frames skipped: needs > 4 states), lm_steps 19803 (16953 accepted, 2850 rejected, 0 invalid), ldlt_solves 19803, ldlt_retry 0, get_dense_h_b 19803 |
| outer linearise | `linearizeProblem` + `performQR` | 21105 each (17427 in `optimize`, 3678 in `marginalize`) |
| debug paths | `vio_debug`/`vio_extended_logging` false: `logMargNullspace`, `checkMargEigenvalues` (the only SelfAdjointEigenSolver users) | log_marg_nullspace 0 |
| sizes | 7 kf poses x 6 + 3 states x 15 | max reduced H dim 87, max marginalisation Q2Jp 1760 x 72 |
| bookkeeping | | new_kf 462, landmarks_added 25041, kf drops 455, imu_integrate 36810 (3681 frame pairs) |

Port consequence: of `marg_helper.cpp` only `marginalizeHelperSqrtToSqrt` (~80 lines: column-permute, hand Householder with `rank_threshold = sqrt(eps)`) is needed; `SqToSq` / `SqToSqrt` (`completeOrthogonalDecomposition`, `.ldlt` square root), the double instantiation, ABS_SC / REL_SC, and the nullspace checks are dead on this path. Eigen kernels needed in float: dynamic LDLT (87 x 87 solve), `makeHouseholderInPlace` / `applyHouseholderOnTheLeft` (marg helper and landmark block), fixed 9 x 9 LDLT (`compute_sqrt_cov_inv`), small fixed products.

### 4b. Unordered containers and iteration order

`tbb::concurrent_unordered_map` is **not** a numerical hazard: the only one on the stereo EuRoC path is `result` in `FrameToFrameOpticalFlow::trackPoints`, and it is copied into an ordered `Eigen::aligned_map` (`transform_map_2.insert(result.begin(), result.end())`), so its order never matters. `Corners`, `Matches`, `vis_map`, `timestamp_to_id`, `patch_optical_flow` / `multiscale` maps are mapper / GUI / other-flow-type code, not executed. The hazards are the **libstdc++ `std::unordered_*` of the estimator**:

| container | where | iteration feeds numerics? |
|---|---|---|
| `std::unordered_set<int> unconnected_obs0` | `measure` | **yes**: loop order = order in which new landmarks are created (triangulated, `lmdb.addLandmark`) |
| `LandmarkDatabase::kpts` (`unordered_map<size_t, Keypoint>`, hash = identity) | `LinearizationAbsQR` ctor builds `landmark_ids` by iterating it; blocks, QR row layout of Q2Jp and every float accumulation (`error`, `H`, `b`, `l_diff`) follow this order | **yes** |
| `LandmarkDatabase::observations` (`unordered_map<TimeCamId, map<...>>`, hash = `hash_combine(hash_combine(0, frame_id), cam_id)`, 64-bit golden-ratio variant) | `computeError`: `host_frames` order = float summation order of the cost | **yes** (the cost decides LM accept/reject) |
| same map | `LinearizationAbsQR` ctor (`relative_pose_lin`, `host_to_idx_`, `relative_pose_per_host`) | no (keyed lookups; the by-host vector is unused in ABS_QR) |
| `unordered_set<size_t> lost_landmaks` | `count()` in the linearisation; iterated only to `removeLandmark` (erase keeps the order of the others) | no |
| `host_to_landmark_block`, `relative_pose_lin`, `outliers_concurrent` (null here), `abs_lin_data` (`optimize_single_frame_pose`, not called), `ExecutionStats` maps | | no |

Test (`reference/experiments/exp_unordered_order.patch`, env `BASALT_EXP_ORDER`, 10 full MH_01 runs, same binary): reversing / sorting the `unconnected_obs0` loop, another bucket count for `unconnected_obs0` (`reserve(4099)`), sorting / reversing `landmark_ids`, another bucket count for `kpts` (`reserve(16384)`), sorting / reversing the `computeError` host frames:
**every** perturbation changes the TUM bytes (first differing state 4 = first optimisation, or 48 for the host-frame order), the control mask 0 reproduces `a7d3e7a3...`; but the maximum position deviation is 1.6 - 3.0 mm and ATE is unchanged (Sim3 0.0337 in 9 runs, 0.0338 in one; SE3 0.0740 in all).
Conclusion: for a **byte-identical** port the C code must reproduce the libstdc++ node order (single forward list, new node at the head of an empty bucket or after the bucket's predecessor, rehash re-inserts in list order with the `_Prime_rehash_policy` prime table, erase keeps the order of the others) for `kpts`, `observations` and `unconnected_obs0`; for an ATE-level port the order is irrelevant. The dump gives the check: every `OPT_BEGIN` record carries `lm_order_hash` (ids in `kpts` order) and `host_order_hash` (observations order), and with `BASALT_PORT_DUMP_FULL=1` the full id lists, so the model can be validated bitwise before the numerics (module M6).

### 4c. Sim3 0.034 m vs SE3 0.074 m: algorithm or pipeline?

Same ATE script (`basalt_port/reference_tools/ate_eval.py`, `benchmark.ate_rmse` / `umeyama_alignment`, nearest GT within 5 ms, 3638 matched states), same data, same `euroc_config.json` / `euroc_ds_calib.json` (byte-identical to the release's copies):

| run | Sim3 | scale | SE3 | 4-DoF (yaw + t) |
|---|---|---|---|---|
| reference build (this port target, -O2 strict, 1 thread) | 0.0337 | 1.0155 | 0.0740 | 0.0770 |
| same sources, `-O3 -march=native` (FMA/AVX2), 1 thread (`runs/basalt_port/exp2_build`) | 0.0328 | 1.0154 | 0.0732 | 0.0763 |
| prebuilt release `basalt_vio` 0.1.7, 4 threads, 3 runs (bytes differ, ATE equal to 4 digits) | 0.0206 | 1.0148 | 0.0662 | 0.0684 |
| prebuilt, `--num-threads 1` | 0.0206 | 1.0148 | 0.0662 | 0.0684 |

(the prebuilt's own `rms_ate` is 0.0662 = our SE3 number; `docs/vio_candidates_20261001.md` 0.066 / 0.021 is reproduced.)

* The SE3-vs-Sim3 gap is **algorithmic (scale)**, not a pipeline artefact: all four configurations give scale 1.0148 - 1.0155 (1.5 % of an 80.5 m path) and Sim3 is 2.2x / 3.2x better than SE3; aligning with only yaw + translation (the gravity-aligned world frame, 4 DoF) is no better than full SE3 (0.077 vs 0.074), so roll/pitch/gravity is not the issue.
* The 0.034 vs 0.021 difference between builds is a **float-noise bifurcation**, not systematic: all builds are identical to < 5 mm until frame ~1380 (t = 69 s); after that the prebuilt and the `-march=native` build leave our reference trajectory (max deviation 6.5 cm and 4.8 cm, mostly at the end), while 9 different container-order perturbations of our build stay within 3 mm. Single-thread vs 4-thread prebuilt differ by 1.3 mm. So the port's target (this reference) is a legitimate member of a spread of 0.021 - 0.034 (Sim3) / 0.066 - 0.074 (SE3) over float-level build variants; accuracy claims for Basalt on MH_01 should quote the spread, and a byte-exact C port will land exactly on 0.0337 / 0.0740.

### 4d. New finding: an ODR hazard that changes the trajectory (build-order-dependent numerics)

`IntegratedImuMeasurement<float>::compute_sqrt_cov_inv` (basalt-headers `preintegration.h`) calls an unqualified `sqrt(ldlt.vectorD()[i])`. In the linearisation translation units it resolves to float `sqrtss` + `divss`; in a unit with different includes (`sqrt_keypoint_vio.cpp`) it resolves to `::sqrt(double)` (`cvtss2sd; sqrtsd; divsd; cvtsd2ss`, correctly rounded double result, which is not always the same float). The inline function is emitted weak in each unit and the linker keeps one. The first draft of the dump patch called `get_sqrt_cov_inv()` from `sqrt_keypoint_vio.cpp`, made that unit emit the double version, the linker picked it, and the trajectory changed (sha `68d99d40...`, first deviation at the first optimisation, up to 1.5 mm) **with the dump switched off**. The reference uses the float copy (`1.0f / sqrtf(d)`), the C port must too; `tools/build_basalt_reference.py` now fails the build if the linked driver does not contain the `sqrtss` copy, and patch 0003 deliberately does not dump `sqrt_cov_inv` (M3 verifies it through the `ITER_STEP` digests instead). Any future instrumentation must not instantiate Eigen/basalt templates in a new translation unit without re-checking the trajectory hash.

## 5. Dump records (patch 0003; reader `basalt_port/c/check_bs_dump.c`, runner `python3 tools/check_basalt_port.py`)

Env: `BASALT_PORT_DUMP_DIR=<dir>` (enables; writes `flow|imu|iter|marg|summary.bin`), `BASALT_PORT_DUMP_EVERY="flow=N,imu=N,iter=N,marg=N"` (sample the first and every N-th frame / `optimize()` call / marginalisation; an `optimize()` call is dumped completely or not at all), `BASALT_PORT_DUMP_FULL=1` (iter + marg records carry the dense arrays). Framing as in okvis_port / rdvio_port: `u32 tag, u64 payload bytes, payload`, little endian; each file starts with a `HDR` record (tag 100: u32 version 1, u32 sizeof(Scalar) = 4, u32 every x4, u32 full). S = float32, matrices column major. Hashes are FNV-1a 64 over the raw float bytes (state digest: all frame poses and states, current + linearisation point + delta + lin flag).

| tag | file | one per | payload |
|---|---|---|---|
| 1 FLOW | flow.bin | frame | `i64 t_ns, u64 frame_counter, u32 ncam, ncam x {u32 w, u32 h, u64 hash(uint16 pixels)}, ncam x {u32 n, n x {u64 id, f32 m[2x3 row-major]}}` = the `OpticalFlowResult` given to the estimator (ids ascending, full affine incl. linear part) |
| 2 IMU_PREINT | imu.bin | frame pair | `i64 start, end; S bg_lin[3], ba_lin[3], accel_cov[3], gyro_cov[3]; u32 ns; ns x {i64 t, S a[3], S g[3]}` (calibrated samples exactly as passed to `integrate`, tail sample with frame time), then `i64 dt_ns; S dq[4] (xyzw), dp[3], dv[3], cov[81], d_state_d_ba[27], d_state_d_bg[27]` |
| 5 IMU_PREDICT | imu.bin | frame pair | `i64 t0, t1; S g[3]; S state0[16], state1[16]` (quat xyzw, p, v, bg, ba) = `predictState` input / output |
| 7 OPT_BEGIN | iter.bin | optimize() | counts (poses, states, landmarks, observations, host frames, aom size, imu blocks), lambda, `state_hash, lm_order_hash, host_order_hash, lm_value_hash`, marg prior rows/cols + H/b hashes, [full: landmark ids in `kpts` order, hosts in `observations` order] |
| 3 ITER_STEP | iter.bin | LM inner step (accepted or not) | `it, j, flags, lambda before/after, error_total, after_vi_err, after_marg_err, l_diff, f_diff, relative_decrease, step_norminf, Hdim, hash(H), hash(b), hash(inc), state_pre, state_post (after inc, before undo), lm_value_post`, [full: `H[n*n], b[n], inc[n]`] |
| 6 OPT_END | iter.bin | optimize() | `it, it_rejected, converged, terminated, lambda, state_hash` |
| 4 MARG | marg.bin | marginalisation | path (0 SqrtToSqrt), aom layout, `idx_to_keep`, `idx_to_marg`, kfs_to_marg, kf_ids_all, prior in (rows, cols, hashes), Q2Jp / Q2r (rows, cols, hashes), H_new / b_new / final b hashes, [full: prior H,b, Q2Jp, Q2r, H_new, b_new, final b] |
| 9 SUMMARY | summary.bin | process | `u32 n, n x {u32 len, name, u64 value}`: the executed-path counters of 4a |

Sizes (MH_01): `all` (every record, digests only) 54 MB; `full` (flow=50, imu=50, iter=40, marg=15 + dense arrays) 34 MB. Overhead (sequential runs): wall 61 s off, 64 s `all`, 61 s `full`. Trajectory with the dump off / `all` / `full` is `a7d3e7a3...` in all three.
Replay inputs per module: M2 camera = FLOW; M3 IMU = IMU_PREINT/PREDICT; M5 frontend = FLOW (images hashed; the C frontend reads the PNGs); M6 landmark order / linearisation = OPT_BEGIN + ITER_STEP hashes (full: H,b); M7 LM = ITER_STEP scalars; M8 marginalisation = MARG (full: Q2Jp -> H_new).
