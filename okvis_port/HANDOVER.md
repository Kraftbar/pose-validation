# OKVIS2 pure-C port — handover

## Performance (Codex) 2026-10-06 — DONE, all gates pass

Artifacts and isolated builds: `runs/okvis_port/perf/codex/` (no commits).
`perf` is unavailable (`perf_event_paranoid=4`); the baseline `-O2 -g -pg`
600-frame mono profile is `profile/gprof.txt`. Self samples: matching 64.59%,
map matching 7.82%, vocabulary transform 2.50%; the Hamming byte-at-a-time
bit loops dominate the first and third. No floating-point expression or
accumulation order was changed.

Changes:
- `ok_hamming.h`: exact, alignment-safe 64-bit integer population counts,
  shared by `ok_frontend.c` and `ok_dbow.c`. No target-specific compiler flags.
- `ok_system.c`, `ok_vigraph.{c,h}`: disable replay-event accumulation for a
  system-owned backend; replay recording remains enabled by default. Complete
  graph teardown, including parameter blocks retained in an ownership vector.
  Removed blocks keep their addresses until `ok_vg_free`, so the replay
  harness pointer bijection and live two-pose references remain valid.

600 mono frames, single-threaded, `-O2 -g -ffp-contract=off -fno-fast-math`:

| Version | Wall seconds | Max RSS KiB | Incremental speedup |
|---|---:|---:|---:|
| Baseline | 122.96 | 269780 | — |
| Hamming only | 40.38 | 269856 | 3.05x |
| Hamming + event queue / teardown | 40.54 | 101644 | 1.00x (timing noise); 62.3% less RSS |

Both CSV files compare byte-for-byte across all three versions. The Hamming
kernel also passes 65,536 byte-pair patterns and 100,000 unaligned random
comparisons with the original loop (`check_hamming.c` in the artifact dir).

Full standalone results (3,681 processed frames each, single-threaded):

| Mode | Before wall s | After wall s | Speedup | Before RSS KiB | After RSS KiB |
|---|---:|---:|---:|---:|---:|
| Mono | 626.01 | 302.50 | 2.07x | 1242272 | 243724 |
| Stereo | 1672.07 | 629.92 | 2.65x | 2415376 | 389568 |

All four canonical final/causal SHA256 hashes pass for both before and after
(`results.json`, `baseline_*/sums.txt`, `after_*/sums.txt`). The native mono
system replay passes 155,335,661 comparisons, zero mismatches (`quick_system.log`).
ASan/UBSan with leak detection passes 400 frames (exit 0, no leaks); its only
diagnostic is the known `ok_vsolve.c:27` zero-length memcpy with a null source.
The full runner `tools/check_okvis_port.py --tag m8,s8 --native-solve 1 --eigen-tests --data
external/vio/data/okvis_brisk_tmp` (re-run by Claude after Codex hit its usage limit;
`runs/okvis_port/perf/claude_full_suite.log`) passes every row with 0 mismatches, exit 0:
- all 24 m8 / s8 harnesses, including check_ok_system: 155,335,661 (mono) and 300,332,809
  (stereo) comparisons;
- the 14 Eigen / okvis oracle tests.
Claude reviewed the diff: the SWAR popcount, the event queue switched off only for the
system-owned backend, and the teardown. Blocks are kept until ok_vg_free, so memory grows
with the blocks ever created; RSS still falls 80-84 % against the old event queue.

Rebuild with `python3 runs/okvis_port/perf/codex/build.py memory`; the build
adds `ok_png.c` only because the current app links its PNG fallback (all runs
here use the gray packs). PNG and BRISK sources are untouched.


Goal: clean-room, library-free C99 port of OKVIS2 (BSD-3, `external/vio/okvis2`),
bit-exact against a deterministic single-threaded reference build, same method
as `stella_port/` (read `stella_port/HANDOVER.md` for method, conventions and
exactness pitfalls; Eigen-derived code goes in MPL-2.0 files).
Mono+IMU is the primary target (phones), stereo+IMU secondary (drones).
GNSS (loosely coupled position factor) comes after the VIO port; keep the
estimator's factor interface open for it.

Clean room: never read GPL sources (`orb_port/`, ORB-SLAM2/3, OpenVINS,
VINS, GVINS). Allowed: OKVIS2 (BSD-3), Ceres (BSD-3), BRISK (BSD-3),
DBoW2/DLib (BSD-style, check attribution clause), Eigen (MPL-2.0),
OpenCV (BSD-3/Apache-2), glog/gflags (BSD-3).

## Reserved for Codex (2026-10-01): BRISK leaf

Codex owns the BRISK keypoint detector + descriptor as used by OKVIS2
(`brisk` library in the OKVIS2 tree: AGAST/FAST score, scale-space detection,
orientation, descriptor bits). Reserved paths: `okvis_port/c/ok_brisk*.{h,c}`,
`okvis_port/c/check_ok_brisk*.c`, `okvis_port/reference_brisk/**`,
`runs/okvis_port/reference_brisk/**`. Claude does not touch these.

## Reserved for Codex (2026-10-06): performance

Codex speeds up the C port without changing an output bit: it owns `okvis_port/c/*.{c,h}` (except `ok_brisk*` and `ok_png*`), with results in
`runs/okvis_port/perf/`. Claude does not edit `okvis_port/c` while it runs.

## Status

### 2026-10-06: PNG input: `ok_png` (written by Codex, tested and hooked up by Claude); `okvis_c_euroc` reads EuRoC PNGs directly

- `okvis_port/c/ok_png.{h,c}` (MIT, `reference_png/LICENSE`): PNG -> 8-bit gray with the pixels of `cv::imread(path, IMREAD_GRAYSCALE)` of the
  reference's OpenCV 4.6. Scope and validation are in `okvis_port/reference_png/README.md`.
- Test: `reference_tools/okvis_png_test.cc`, run by `--eigen-tests`; the runner gained the test libs `imgcodecs` and `zlib`. Against the real
  `cv::imdecode`, 0 mismatches on 10,514 files: 14 real EuRoC frames, 1,500 `cv::imencode` outputs, 9,000 crafted PNGs (every colour type, bit
  depth, filter, Adam7, gAMA / sRGB / sBIT / tRNS / PLTE). 3,000 corrupt files never crash. The test is clean under ASan/UBSan.
- `okvis_c_euroc` uses the `gray/cam<i>.gray` packs if present, otherwise `mav0/cam<i>/data.csv` + `data/<file>`, as DatasetReader does.
  MH_01 mono from the original PNGs: `final.csv` `dfe3b58e...` and `causal.csv` `cc29a746...`, the reference hashes. 642 s, against 624 s with packs.

### 2026-10-05 (Claude): BRISK hooked up (M7a, Codex's leaf) and module 8 (the system driver) DONE: the whole C pipeline runs MH_01 mono and stereo from the images + imu0/data.csv + the YAML config and writes trajectories byte-identical to the reference

Deliverables
- C (C99): `ok_system.{h,c}` (BSD-3 + MPL: ThreadedSlam init / processFrame / optimisePublishMarginalise / the stopThreading schedule of patches 0001 + 0006 run sequentially: first-frame IMU drop rule, IMU deque pop / prune (`imuTemporalOverlap` 0.02), detection pose = `lastOptimisedState_` (stored AFTER the detection of the previous frame, i.e. the newest state before the previous addStates) IMU-propagated to the frame, `initPose` while it is unset (first TWO frames), Frontend::detectAndDescribe with Codex's `ok_brisk*` (extraction direction `T_WC.inverse().C() * (0,0,-1)` as float, focal `float(fu)`), the `< 15 keypoints` drop, addStates, data association, setKeyframe, optimiseRealtimeGraph, synchronise when a loop closure is available, publish (causal row), applyStrategy, optimiseFullGraph; TrajectoryOutput / writeFinalCsvTrajectory writers incl. the anyState reconstruction `pose(kf) * T_Sk_S` (cacheless) and `C_WSk * v_Sk`); the driver reaches the backend only through an `ok_sys_be` table (direct calls by default) and the frontend through `ok_fe_est`, so a harness can compare every call. `ok_config.{h,c}` (BSD-3: the YAML subset of the OKVIS2 configs, ViParametersReader semantics: T_SC = `Transformation(Matrix4d)` then `Transformation(r, q.normalized())`, booleans as parseEntry, strtod / strtol). `okvis_c_euroc.c` (the app: `okvis_c_euroc <config.yaml> <sequence dir> <vocabulary.bin> <out dir> [max frames]`; DatasetReader order: before each image t the IMU up to the first measurement later than t + Duration(0.021), values through `strtof` like `std::stof`, measurements older than start - 1 s dropped). `ok_cam.c`: `ok_cam_awareness_maps` (PinholeCamera::initialiseCameraAwarenessMaps: normalised back-projected rays + 2x3 projection Jacobians as float). `ok_vigraph.{h,c}`: `ok_vg_anystate_count / _at`.
- Tools: `okvis_port/reference_tools/okvis_png2gray.cc` (cv::imread IMREAD_GRAYSCALE with the reference's OpenCV 4.6 -> one `.gray` pack per camera: "OKGRAY1", u32 w, h, n, n x {u64 ts, pixels}; the port has no PNG decoder), `tools/okvis_port_images.py <seq>` (fetch cams + imu0 into `external/vio/data/okvis_brisk_tmp/<seq>`, decode, delete the PNGs: 2.7 GB for MH_01 stereo), runner `tools/check_okvis_port.py --data ROOT` (check_ok_frontend runs BRISK natively, check_ok_system runs end to end; without --data check_ok_system is skipped).
- Harnesses: `check_ok_frontend.c` with `OK_BRISK_IMAGES` (BRISK on the images replaces the logged keypoints / descriptors after comparing them; refactored into `fe_setup` / `fe_report`, includable with `OK_FRONTEND_AS_LIB`), `check_ok_vslam.c` (addStates hook, ADDIMU config captured), new `check_ok_system.c` (the C system from the dataset; the log only CHECKS: every ThreadedSlam backend call (addImu, addCamera, addStates with the IMU deque + keypoints, setKeyframe, optimiseRealtimeGraph, synchronise, applyStrategy, optimiseFullGraph) tag + argument bytes + results, the descriptors of record 161, everything check_ok_frontend compares, the whole log consumed, then `causal.csv` and `final.csv` of the reference run row by row).

Result (tolerance 0, bitwise; native solves)

| check | m8 (mono) | s8 (stereo) |
|---|---|---|
| awareness maps vs the real `initialiseCameraAwarenessMaps` (Codex's native dump) | cam0 1,082,880 ray + 2,165,760 Jacobian floats, 0 | cam1 same counts, 0 |
| check_ok_frontend with native BRISK: keypoints (x y size) / descriptor bytes | 4,884,705 / 78,100,065, 0 | 9,759,951 / 156,048,786, 0 (whole harness 310,055,801, 0) |
| check_ok_system (C system end to end, 3,681 frames, 1 dropped at startup, 36,817 IMU measurements) | 155,335,661 values, 0; causal.csv + final.csv 7,364 rows identical | 300,332,809 values, 0; 7,364 rows identical |
| `okvis_c_euroc` standalone (no harness, no log): sha256 of the outputs | final `dfe3b58e...`, causal `cc29a746...` = reference | final `673fa08f...`, causal `04965fdc...` = reference |

The standalone stereo run reproduces the reference although it cannot substitute the 40 degenerate GP3P samples (UB upstream, see the M7c/d entry): the C code skips such a sample and the run still ends in the same trajectory (as two builds of the reference with different UB outcomes did). Runtime: mono 624 s / stereo 1,723 s single-threaded (reference with dumps 278 / 589 s), max RSS 1.2 / 2.4 GB. ASan/UBSan: the app over the first 400 mono frames (incl. loop closures) is clean apart from the known `memcpy(dst, NULL, 0)` of `ok_vsolve.c`; LeakSanitizer reports teardown leaks: `ok_vg_free` never frees parameter blocks (landmark / state blocks, removed or not; pointer identity of the replay bijection), about 1 MB per 400 frames.

Facts found
- Bug caught by the stereo check only: the camera-model blob buffer of the driver was 64 bytes for an 80-byte header; camera 1's blob overwrote d[2], d[3] of camera 0 (mono unaffected, ASan blind: inside one struct). check_ok_system flagged ADDSTATES byte 996 of the first frame; the standalone run had diverged from row 0 (0.20 m causal / 0.05 m final).
- The camera awareness Jacobian of the 309 / 715 border pixels whose ray projects outside the image is UNINITIALISED upstream (`cv::Mat` never written there); the reference's fresh 8.7 MB allocation is mmap'd zero pages, so zeros reproduce it.
- `lastOptimisedState_` lags one frame more than the name suggests: the detection of frame k uses the state of frame k-2 after its full processing, propagated over the deque of addStates(k-1) plus the measurements popped for frame k.

Not ported / next
- PNG decoding (the gray packs stand in for cv::imread), IMU-less operation, enforce_realtime, CNN, depth cameras, do_final_ba, online extrinsics, multi-session: rejected by `ok_sys_new` / the config reader.
- Other sequences (only MH_01 has reference dumps), runtime (2-3x the reference), memory (blocks never freed).

### 2026-10-05 (Claude): modules 7c (OpenGV: GP3P, Stewenius, rotation-only, Ransac with mt19937) and 7d (place recognition: DBoW2, verifyRecognisedPlace, quickSolver) DONE, bit-exact on mono and stereo; the frontend has no log-answered hook left (except the documented UB runs)

Deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h>` + `<float.h>`): `okvis_port/c/ok_opengv.{h,c}` (BSD-3 + MPL: `Ransac::computeModel` with `SampleConsensusProblem` sampling (shuffled-index draws from `std::mt19937(12345)` + `std::uniform_int_distribution<int>(0, INT_MAX)` = Lemire downscaling on 32-bit words, bound by `std::bind` so every run starts a fresh generator), `AbsolutePoseSacProblem` (GP3P: sample of 4, fourth-point disambiguation) + OKVIS2's `FrameAbsolutePoseSacProblem` scores (sigma-angle weighted), `CentralRelativePoseSacProblem` (Stewenius, sample of 8: 5 for the solver, 8 for the 4 x 10 candidate disambiguation) + `FrameRelativePoseSacProblem`, `RotationOnlySacProblem` (`twopt_rotationOnly`, `arun`) + `FrameRotationOnlySacProblem`, `triangulate2`, `cayley2rot`, the glue of `gp3p_main` / `fivept_stewenius_main`), `ok_opengv_gp3p_gen.c` (mechanical transcription of the OpenGV Groebner GP3P solver, 134 functions, generated by `tools/gen_okvis_gp3p.py`), `ok_opengv_stew_gen.c` (`composeA`, `tools/gen_okvis_stewenius.py`), `ok_eigen_eigsolver{8,10}.c` + `.inc` + `ok_eigen_eigsolver.h` (EigenSolver<Matrix<double,N,N>> compute + eigenvectors(), size-templated copy of stella's 10x10 solver), `ok_eigen_cx.c` (std::complex<double> `/` = libgcc's range-guarded Smith/Baudin-Smith algorithm, `*`, `sqrt` via the csqrt identity), `ok_eigen_{svd,qr,fullpivlu}.{c,h}` (stella's MPL JacobiSVD / ColPivHouseholderQR / FullPivLU adaptations, identifiers renamed; the 5x9 wide SVD of Stewenius is the `N < 9` branch), `ok_dbow.{h,c}` (DBoW2 vocabulary-tree transform, L1 database, `queryL1`, libstdc++ introsort, `getFilteredDBoWResult`), `ok_place.{h,c}` + `ok_place_dist.c` (`verifyRecognisedPlace` refinement: the `quickSolver` Problem built for the module-4 solver, information matrix, extra-outlier count; the Eigen float distinctiveness statistic), `ok_frontend.{h,c}` (native adapters of `FrameNoncentralAbsoluteAdapter` / `FrameRelativeAdapter` / `LoopclosureNoncentralAbsoluteAdapter`, `runRansac3d2d` / `runRansac2d2d` on them, `verifyRecognisedPlace`, the loop-closure block incl. the DBoW database; `ok_fe_est` lost the `ransac` hook and gained observers + `attempt_loop_closure` / `add_loop_closure_frame`; `place_recognition` stays only as the fallback for tags without place.bin), `ok_solve.{h,c}` (new `strategy_lm`: `LevenbergMarquardtStrategy` after Ceres 2.2.0, and `cauchy_a`: `CauchyLoss(a)`; the defaults keep the graph solver unchanged), `ok_vslam.{h,c}` (the keypoint landmarks `Frame::landmarks_` stored by `convertToPoseGraphMst`: `T_SW * landmark` + initialised flag; `is_pose_graph_frame` / `is_place_recognition_frame` / `is_loop_closure_frame` / `is_recent_loop_closure_frame`).
- Tools: `tools/convert_okvis_vocabulary.py` (OKVIS2's `small_voc.yml.gz` -> the binary payload `ok_dbow_voc_load` reads, written into the gitignored `runs/okvis_port/vocabulary/`; `--check place.bin` proves it IDENTICAL to the vocabulary the reference loaded), `tools/gen_okvis_gp3p.py`, `tools/gen_okvis_stewenius.py`, runner flags `--ransac-dump` / `--place-dump`.
- Reference: patches `0013-ransac-input-log.patch` (`OKVIS_PORT_RANSAC_DIR`: `ransac.bin`, the adapter data of every OpenGV run + its result, kind 3 included) and `0014-place-recognition-log.patch` (`OKVIS_PORT_PLACE_DIR`: `place.bin`, the vocabulary as loaded, every database add, every query with bag of words / unsorted results / retained ids, a stage record of every `verifyRecognisedPlace`). Series 0001-0014 applied to the pristine copy reproduces `reference_build/src` exactly (`diff -r`, only the empty `fast-scnn.pt` placeholder differs). m8 / s8 were EXTENDED, not regenerated: two lean runs per sequence (`--ransac-dump --place-dump`, 4 / 8 min, a few MB) whose `ransac.bin` / `place.bin` were copied into `m8/dumps` / `s8/dumps`; their trajectories are byte-identical to the canonical ones (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`) for BOTH instrumented builds (0013, 0014). The 2.6 GB of images (own dirs `external/vio/data/okvis_tmp`, `MH_01_easy_okvis`, `MH_01_easy_okvis_s`) were deleted at the end.
- Tests: `reference_tools/okvis_opengv_test.cc` (links the REAL libopengv.a of the reference build + the UNMODIFIED OKVIS2 `Frame*SacProblem` headers through the shadow adapters in `reference_tools/shadow/`, new test directive `OK_PORT_TEST_LIBS: opengv`) and `okvis_place_test.cc`; `check_ok_frontend.c` runs the whole thing on the dumps.

Result (tolerance 0, bitwise)
| oracle test, real OpenGV / Eigen 3.4.0 / libstdc++ | mismatches / compared |
|---|---|
| `std::mt19937(12345)` + `uniform_int_distribution<int>(0, INT_MAX)` bound with `std::bind` (3.0 M draws) + raw `mt19937` (0.1 M) | 0 / 3,100,000 |
| `std::complex<double>` `/`, `*`, `sqrt` incl. exponents 1e-300..1e300 and zero imaginary parts | 0 / 11,824,650 |
| `EigenSolver<Matrix<double,8,8>>` / `<10,10>` eigenvalues + `eigenvectors()` (random, companion-like, Hessenberg, scaled) | 0 / 100,000 each |
| `gp3p_main` (4 random geometries incl. near-degenerate tilt) | 0 / 20,000 |
| `FrameAbsolutePoseSacProblem<GP3P>`: `computeModelCoefficients` / `getSelectedDistancesToModel` / `Ransac::computeModel` (iterations, inliers, model bits) | 0 / 12,000, 0 / 12,000, 0 / 3,000 |
| `triangulate2` (through the Stewenius distances), rotation-only model / distances / Ransac, `fivept_stewenius` real parts of the 10 essentials, Stewenius model / distances / Ransac | 0 / 3,000; 0 / 12,000, 0 / 12,000, 0 / 3,000; 0 / 4,500; 0 / 4,500, 0 / 4,500, 0 / 1,500 |
| `okvis_place_test`: the Eigen float distinctiveness expression (rows 1-200), libstdc++ `std::sort` vs the introsort of `ok_dbow.c` (30,000 tie-heavy vectors up to 900 entries) | 0 / 6,000, 0 / 30,000 |

Harness result (`python3 tools/check_okvis_port.py --tag m8,s8 --native-solve 1 --eigen-tests`, every row PASS, exit 0)

| check_ok_frontend (module 7b-7d, with `ransac.bin` + `place.bin`) | m8 (mono) | s8 (stereo) |
|---|---|---|
| frames run through the C frontend / backend calls made by it vs the logged entry records (tag + argument bytes, 0 mismatches) | 3,681 / 1,784,673 | 3,681 / 3,736,956 |
| keyframe decision vs logged `setKeyframe` | 3,681 | 3,681 |
| native RANSAC runs of the frontend: adapter data + iterations + inliers + model bits vs `ransac.bin` | 199 runs, 117,371 values, 0 | 377 runs, 252,913 values, 0 |
| every logged run re-run natively on the LOGGED adapter data (kinds 0 / 1 / 2 / 3) | 12 / 3 / 3 / 181 runs, 8,503 values, 0 | 4 / 2 / 2 / 369 runs, 16,210 values, 0 (40 kind-3 runs differ through a degenerate sample, UB upstream, not counted) |
| database adds (entry id, frame, features, bag of words) | 134 adds, 73,464 values, 0 | 109 adds, 85,462 values, 0 |
| queries (bag of words, all unsorted results with their bits, retained ids and scores) | 3,648 queries, 2,690,738 values, 0 | 3,645 queries, 3,435,779 values, 0 |
| `verifyRecognisedPlace` stage records (exit code, matches, landmarks, counts, avg bits, RANSAC model, refined pose, Ceres iterations / termination / costs, H, outliers) | 643 calls, 538,455 values, 0 | 5,355 calls, 5,620,463 values, 0 (40 with a replaced RANSAC result, all equal) |
| `attemptLoopClosure` / `addLoopClosureFrame` made by the C frontend (args vs the logged entry records, results) | 48, 0 | 112, 0 |
| everything check_ok_vslam compares (graph calls, results, problem events, optimise states / blocks / IMU, program order, native solves) | 77,198,655 total with 11,108 native solves, 0 | 144,247,064 total with 11,114 native solves, 0 |

Full runner table, all PASS: check_ok_cam m8 1,043,775 / s8 2,220,128; check_ok_err m8 30,919,758 / s8 80,622,441; check_ok_frontend m8 77,198,655 / s8 144,247,064; check_ok_graph m8 821,516 / s8 1,226,098; check_ok_imu m8 4,228,502 / s8 4,002,758; check_ok_kin m8 458,615 / s8 1,009,374; check_ok_param m8 5,540,618 / s8 9,903,313; check_ok_problem m8 183,343 / s8 354,760; check_ok_solve m8 17,330,940 / s8 34,563,464; check_ok_vigraph m8 57,492,422 / s8 101,559,917; check_ok_vslam m8 70,197,097 / s8 127,315,136 (native solves); eigen_eig / eval_shapes / gemm / product_modes, okvis_err (1,122,774,544) / frontend (2,000,000) / kin_cam (56,049,278) / opengv (15,216,650) / place (36,000) / solve_dense (6,355,849) / solve_sparse (233,342) / twopose (98,290,979) / vslam (1,064,000) tests PASS, 0 mismatches. ASan / UBSan: the frontend harness over the first 800,000 backend records of m8 (database, 3 verifications incl. the first loop closure, native solves) is clean apart from the pre-existing benign `memcpy(dst, NULL, 0)` in `ok_vsolve.c`.

Sensitivity (bugs the checks caught while the code was written, not deliberate mutations): the stale `11 - iu` literal of the 8x8 EigenSolver (all 100,000 random cases failed), the `-block * col` association (6,149 of 12,000 distance vectors), the assignment-vs-construction rule of `U * W * V^T` (4,067 of 4,500 Stewenius models), the 40 stereo kind-3 runs with a degenerate GP3P sample (the first results no C port can reproduce), H without the row-major quirk (mismatch at the first accepted loop closure, frame 274), `T_Sold_Snew` re-normalised (1 ulp in 16 values at 4 accepted closures, first at frame 915) and a wrong redux order of `stdev.sum()` (`avg` 21 ulp off in every accepted candidate); each was invisible to the next-looser check (e.g. the RANSAC inlier sets agreed all along).

Facts found (cheap, Eigen / libstdc++ / Ceres)
- Association rules for small Eigen expressions, measured against the real classes (each is a pass/fail oracle, see the tests): `rotation_t R = U * W * V.transpose()` (CONSTRUCTION, dynamic `MatrixXd` factors) keeps the Matrix3d rule (rows 0-1 left fold, row 2 `a0 b0 + (a1 b1 + a2 b2)`), the same expression as an ASSIGNMENT `R = U * W * V.transpose()` is evaluated through a dynamic temporary and every row is `a0 b0 + (a1 b1 + a2 b2)`; `inverse.col(3) = -inverse.block<3,3>(0,0) * model.col(3)` = `ok_m3_mulv` on the negated block (rows 0-1 packet, row 2 scalar); `Matrix<double,3,4> * Vector4d` into a Vector3d: rows 0-1 left fold of 4, row 2 `(p0 + p1) + (p2 + p3)`; `A^T * v` all rows left fold; `v^T * w` and `error.transpose() * error` left fold.
- `Eigen::EigenSolver` generalises from stella's 10x10 to 8x8 only if every constant that is really `size` (the `11 - iu` column count of the Francis step's `applyHouseholderOnTheLeft`, the `40 * size` iteration limit, the balanced squared-norm tree) is parameterised: one stale literal wrote past `T` into `U` and changed only the eigenvectors, not the eigenvalues.
- `JacobiSVD<MatrixXd>` of a 3x3 and of a 5x9 matrix (wide: QR of the adjoint, `householderQ().evalTo()` for the full V) are covered by stella's `ok_eigen_jacobisvd_3x3` / `_Nx9_v` unchanged; `FullPivLU::inverse()` = `solve(Identity)`, the 10x10 products are `ok_gemm`; `EE * SOLS` (real 9x4 times complex 4x10) is a plain left fold per component; `pow(std::complex, 2)` is `z * z`.
- `verifyRecognisedPlace` quirk: `EvaluateWithMinimalJacobians` writes the 2x6 block ROW-major into `Eigen::Matrix<double,2,6>::data()` (column-major), so `H += jacobianMinimal^T * jacobianMinimal` accumulates a scrambled matrix: `M(r, c) = buf[r + 2c]`; replicating it is what makes H exact. `pose->estimate()` / `estimator.extrinsics()` are `TransformationCacheless` copies of the raw parameters (no quaternion re-normalisation); `T_Sold_Snew` = those 7 numbers.
- The `quickSolver` is `ceres::Solve` with the DEFAULT options except threads / iterations: LEVENBERG_MARQUARDT (new in `ok_solve.c`: `diagonal = clamp(squared column norms)`, `D = sqrt(diagonal / radius)`, step negated, radius / `max(1/3, 1 - (2q-1)^3)` on acceptance, `/ decrease_factor` (doubling) on rejection or invalid step) with SPARSE_NORMAL_CHOLESKY (EIGEN_SPARSE) and `CauchyLoss(3)`; one free 6-dof block, constant extrinsics and landmarks.
- `getFilteredDBoWResult` indexes `dBoWResult[a]` and `poseIds.at(id)` by ENTRY index and tests `suppressedIds.count(f)` with the position in SCORE order: ported as written (the three indices coincide only when every database entry shares a word with the query).
- Database entries are added only inside the loop-closure block's entry condition (`!isLoopClosing && !isLoopClosureAvailable && !needsFullGraphOptimisation && isInitialized`), so keyframes created while a loop closure runs never reach the database.
- Stereo loop-closure correspondences contain the SAME 3D point twice (one landmark seen by both cameras): a GP3P sample with a duplicated point makes the Groebner matrix singular, `EigenSolver` returns `NumericalIssue` and upstream `gp3p_main` then reads UNINITIALISED `EigenSolver` storage (`m_eivec`; stale stack, the build layout decides): 40 of the 369 stereo kind-3 runs (0 of 181 mono) meet such a sample, 7 of the 369 results differ between two builds of the SAME source whose trajectories are byte-identical. The C code counts the failure (`ok_og_result.degenerate`) and skips the sample; the harness re-runs every logged run natively, compares all of them and, for exactly the runs with `degenerate > 0` whose result differs, continues with the logged result (`40` substitutions; `place.bin` comes from the build `sp` while `problem.bin` / `ransac.bin` come from the build `s8` / `sr`; all 40 stage records still agree).
- Why it scores (cheap): the verification funnel is the whole loop-closure filter: mono 643 `verifyRecognisedPlace` calls -> 462 die at descriptor matching (< 10 matches or < 8 landmarks), 128 at the GP3P RANSAC (< 10 inliers or ratio < 0.7), 4 at the descriptor-distinctiveness statistic, 1 at the refinement, 48 accepted (31 closures after the drift heuristic); stereo 5,355 calls -> 4,982 / 4 / 228 / 29 / 0 / 112 accepted (35 closures). The database query is cheap (729 words, k = 9, L = 3, L1) and 3,648 / 3,645 queries return a handful of candidates above `p_dbow = 0.4` after non-maximum suppression (radius 5); only the OLDEST candidate that passes is used. RANSAC is otherwise rare (matchToMap 12 / 4 runs, relative pose 3 + 3 / 2 + 2 runs): the IMU window carries the pose.

Not yet exact / next
- Undefined behaviour upstream (above): runs with a degenerate GP3P sample cannot be reproduced bit for bit in any C port; they are isolated and reported by the harness.
- DBoW2 clause 3 (the original author must be notified of any redistribution of source or binary): `ok_dbow.*` is a source-level port; nothing has been redistributed, NOTICE / `LICENSES/dbow2-LICENSE.txt` / `docs/okvis2_license_audit.md` record it. The vocabulary stays a reference-run input (provenance undocumented): a product needs its own.
- Not exercised / not ported: the multi-session `componentDBows_` branch, RadialTangential8 in `verifyRecognisedPlace`, the CNN sky / person filters, `FrameRelativeAdapter` with a failed back-projection of frame B (upstream leaves the vector uninitialised).
- Next: BRISK (Codex's `ok_brisk*`) feeds `ok_fe_add_frame`; then the system driver (ThreadedSlam::processFrame glue: IMU propagation, `addStates`, `setKeyframe`, publish) and the vocabulary loader call (`ok_fe_set_vocabulary`).


### 2026-10-05 (Claude): STEP 0 dump consolidation (tags m8 / s8) + module 7b (frontend data association) DONE, bit-exact on mono and stereo with the OpenGV RANSAC and the place recognition answered from the log

Dump inventory (`runs/okvis_port/reference_runs/MH_01_easy/`), replaces everything below that mentions m2 / m2cov / s1 / m3 / m3cov / s3 / m6 / s6 / m7 / s7 (all deleted 2026-10-05 after m8 / s8 passed every row):
- `run1`, `run2` (canonical M1 dumps + trajectories), `cov`, `kcount`, `scount`, `s2` (trajectory / call-count runs): kept.
- `m8` (mono, 2.1 GB, 278 s) and `s8` (stereo, 3.8 GB, 589 s): ONE run each with `python3 tools/run_okvis_reference.py MH_01_easy --tag m8 --consolidated` (stereo: `--tag s8 --config okvis_port/reference/configs/okvis_stereo_euroc_deterministic.yaml --data-dir <symlink dir>`). `--consolidated` = `--dump --solve-dump --graph-dump` with `CONSOLIDATED` in the runner: `--dump-every prop=4,preint=4,append=20,eval=1000 --kin-every all=3000 --err-every all=1000,reproj=3000,llt=20,pplus=20,pplusj=40,pminusj=2000,hplus=300,hplusj=400,pose=10,sab=10,relpose=1,ctor=1 --solve-every 100 --solve-full-every 3 --solve-sparse-full-every 4 --graph-tpeval-every 200 --graph-lm-every 8 --graph-lm-sub 16 --graph-problem-full-every 200`. They carry imu / kin / cam / err / param / solve (sampled snapshots; every full-graph solve) / graph (sampled) / problem (EVERY Problem call, the whole mutation log, the backend entry records, the frontend descriptors and RANSAC records of patch 0012). Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`).
- (2026-10-05, later: m8 / s8 each gained `ransac.bin` and `place.bin`, see the entry above.)
- Disk: `reference_runs/MH_01_easy` was 22 GB with m8 / s8 next to the 16.1 GB of old tags (m6 3.2, s6 4.9, s7 2.5, m3cov 1.5, s3 1.5, m7 1.3, m3 0.6, m2cov 0.4, ...; free space on the shared 220 GB disk 7.3 GB), 6.1 GB after the deletion (m8 2.1 + s8 3.8 + run1 / run2 / cov / kcount / scount / s2 0.2; free 26 GB); the 2.6 GB of images (own dirs `external/vio/data/MH_01_easy_okvis` and the stereo symlink dir `MH_01_easy_okvis_s`) were deleted too. The sparser sampling lowers the compared-record count per kind but keeps every kind and code path.

Module 7b deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h>`): `okvis_port/c/ok_frontend.{h,c}` (BSD-3 + MPL: `Frontend::dataAssociationAndInitialization` without the place-recognition internals: `matchToMap` incl. the per-segment matcher `matchToMapByThread` / `...Unitialised` (run sequentially in the reference's order), `matchMotionStereo` (+ `runRansac2d2d` glue), `matchStereo`, `removeOutliers`, `doWeNeedANewKeyframe`, `runRansac3d2d` glue, the correspondence lists of `FrameNoncentralAbsoluteAdapter` / `FrameRelativeAdapter`, descriptor / back-projection store, hamming matching), `ok_triangulate.c` (`triangulateFast`), accessors added to `ok_vslam.{h,c}` (`ok_vsb_frame`, `ok_vsb_num_frames`, `ok_vsb_camera`, `ok_vsb_is_in_imu_window`, per-camera `T_SC` in the frame view) and `ok_vigraph.{h,c}` (`ok_vg_extrinsics_values`). The frontend writes to the estimator only through the `ok_fe_est` table (one entry per ViSlamBackend method it calls) plus two hooks: `ransac` (OpenGV runs, M7c) and `place_recognition` (M7d).
- Reference: patch `0012-frontend-input-log.patch` (instrumentation only): record 161 = the 48-byte BRISK descriptors of the multiframe handed to `addStates` (right after its entry record), record 162 = one per OpenGV `Ransac::computeModel` run of the frontend (kind 0 GP3P matchToMap / 1 rotation-only / 2 Stewenius / 3 GP3P of `verifyRecognisedPlace`, correspondences, iterations, inliers, model). Series 0001-0012 applied to the pristine copy reproduces `reference_build/src` exactly (`diff -r`, only googletest and the empty `fast-scnn.pt` differ).
- Harness `check_ok_frontend.c` (runner glob + skip rule for tags without record 161; `--native-solve N`): replays problem.bin like `check_ok_vslam`, but after every `addStates` the C frontend runs on the logged keypoints (ADDSTATES), the logged descriptors and the C backend state; every backend call it makes is compared (tag + argument bytes) with the next logged entry record, then executed (the graph calls it causes are compared as in check_ok_vslam). The keyframe decision is compared with the logged `setKeyframe`. Records 162 (kinds 0-2: correspondence count must equal the C adapter list) and the logged `attemptLoopClosure` / `addLoopClosureFrame` calls answer the two not-yet-ported blocks. `check_ok_vslam.c` got a look-ahead queue and skips records 161 / 162. Test `reference_tools/okvis_frontend_test.cc`: `ok_fe_triangulate_fast` vs the real `okvis::triangulation::triangulateFast` on 2,000,000 random ray pairs (valid / invalid / parallel / divergent / tiny baseline): 0 mismatches.

Result (tolerance 0, bitwise; `python3 tools/check_okvis_port.py --tag m8,s8 --native-solve 1 --eigen-tests`, every row PASS, exit 0)

| check_ok_frontend | m8 (mono) | s8 (stereo) |
|---|---|---|
| frames run through the C frontend | 3,681 | 3,681 |
| backend calls made by the C frontend, tag + argument bytes vs the logged entry records (addLandmark, addObservation, removeObservation, setLandmark, mergeLandmark(s), setPose, optimiseRealtimeGraph, cleanUnobservedLandmarks, MultiFrame::setLandmarkId) | 1,784,594, 0 mismatches | 3,736,809, 0 |
| keyframe decision vs logged setKeyframe | 3,681, 0 | 3,681, 0 |
| RANSAC runs answered from the log (C correspondence count + kind compared) | 18 (36 values), 0 | 8 (16), 0 |
| loop-closure attempts replayed from the log (place recognition oracle) | 48 | 112 |
| everything check_ok_vslam compares (graph calls, results, problem events, optimise states / blocks / IMU, program order, native solves) | 73,758,942 (73,770,050 with native solves), 0 | 134,781,449 (134,792,563), 0 |

Full runner table, all PASS: check_ok_cam m8 1,043,775 / s8 2,220,128; check_ok_err m8 30,919,758 / s8 80,622,441; check_ok_frontend m8 73,770,050 / s8 134,792,563 (with 11,108 / 11,114 native solves); check_ok_graph m8 821,516 / s8 1,226,098; check_ok_imu m8 4,228,502 / s8 4,002,758; check_ok_kin m8 458,615 / s8 1,009,374; check_ok_param m8 5,540,618 / s8 9,903,313; check_ok_problem m8 183,343 / s8 354,760; check_ok_solve m8 17,330,940 / s8 34,563,464; check_ok_vigraph m8 57,492,422 / s8 101,559,917; check_ok_vslam m8 70,197,097 / s8 127,315,136 (native solves); eigen_eig / eval_shapes / gemm / product_modes, okvis_err (1,122,774,544) / frontend (2,000,000) / kin_cam (56,049,278) / solve_dense / solve_sparse / twopose (98,290,979) / vslam tests PASS, 0 mismatches.

Sensitivity (first 6,000 backend records of m8): reprojection gate `0.06` -> `0.02` crashes the replay (diverges), the 3d-landmark test `cos(10/f)` -> `cos(12/f)` gives 34,331 mismatches; two tiny constant perturbations (`score` factor 0.5 -> 0.5000000001, motion-stereo ray cosine 0.5 -> 0.5000001) and the doWeNeedANewKeyframe radius factor 0.09 -> 0.1 are NOT seen in that window (they only change rare decisions; the full run is what pins them).

Facts found
- `matchToMap` works on ONE `getLandmarks` snapshot (points, quality, observation SETS) taken before the camera loop: for camera 1 the landmark observations that camera 0 of the same frame has just added are NOT in the set (re-reading the live graph per camera changed the matches of stereo frame 3: first mismatch of the stereo run).
- The record of a failed `verifyRecognisedPlace` (RANSAC kind 3, no attemptLoopClosure) must not be handed to a later matchToMap RANSAC: the oracle drops kind 3.
- RANSAC is rarely used on MH_01 (mono / stereo): GP3P 3d2d 12 / 4 runs, rotation-only and Stewenius 3 + 3 / 2 + 2 runs (2d2d only until `isInitialized_`); `verifyRecognisedPlace` ran GP3P 181 / 369 times, 48 / 112 candidates reached `attemptLoopClosure`.
- `trackingQuality` is log output only (the C frontend returns 1 and never sets `trackingLost_`), exactly like the backend port.
- Eigen rules confirmed on this module (random tests + replay): `Vector3d::dot`, `a.transpose() * b` (1x3 * 3x1) = left fold; `(C * v).normalized()` = `ok_m3_mulv` then `x / sqrt(L(x.x))`; `2x2.computeInverseWithCheck(1e-12)` closed form; `Vector4d::norm()` = `sqrt((p0+p2)+(p1+p3))`; `std::max(a, b)` = `(a < b) ? b : a`; `uint32_t distances` compared with the `double` matching threshold behaves like double 60.0.
- Why it scores (cheap findings): the front end is a plain map matcher with a very small search (3 stored descriptors per landmark, 6-7 pixel gates scaled by f), keyframes by the 0.60 field-of-view IoU of disk masks at 1/10 resolution, no RANSAC in the common case (the IMU window carries the pose) and stereo triangulation by midpoint with a 2.6-sigma consistency check; the strength is the backend (module 6), not the frontend.

Not yet exact / next
- [DONE later the same day, see the entry above] M7c: OpenGV GP3P, Stewenius 5-point, rotation-only, the `Ransac` loop with the libstdc++ `mt19937`(12345) sampler: the 18 + 8 logged runs of kinds 0-2 (plus 181 + 369 of kind 3 for the place-recognition GP3P; records 162, inputs derivable from the C adapters) are the validation set; `ok_fe_est.ransac` is the hook to replace.
- [DONE later the same day, see the entry above] M7d: `quickSolver` (RadialTangential8 not needed), `verifyRecognisedPlace`, `getFilteredDBoWResult`, DBoW2 + vocabulary (provenance undocumented); needs a dump of the DBoW query results (patch 0013) and the stock vocabulary.
- BRISK detector / descriptor (Codex) feeds `ok_fe_add_frame`; then the system driver (ThreadedSlam::processFrame glue: IMU propagation, `addStates`, `setKeyframe`, publish).

### 2026-10-04 (Claude): module 6 (ViSlamBackend: strategy, IMU-frame / keyframe / loop-closure bookkeeping, loop closure, synchronisation) DONE, bit-exact on mono and stereo; native solve on the C graph closes the loop

Deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h>`): `okvis_port/c/ok_vslam.{h,c}` (BSD-3 + MPL; the backend state: multiframe keypoints / landmark ids / cleared-image flags, `auxiliaryStates_` (loopId, isPoseGraphFrame, recentLoopClosureFrames, closedLoop), the `imuFrames_` / `keyFrames_` / `loopClosureFrames_` / `currentLoopClosureFrames_` sets, `touchedStates_` / `touchedLandmarks_` / `eliminateStates_` / the add-states backlog (what the realtime graph did while a loop closure was running), `fullGraphRelativePoseConstraints_`, `lastFreeze_`; `addStates`, `setKeyframe`, landmark and observation calls with their full-graph mirroring, `eliminateImuFrames`, `applyStrategy` (lost check skipped: `trackingQuality` only logs; keyframe -> pose-graph conversion with `convertToPoseGraphMst` + full-graph replication, freezing, loop-closure frame conversion, `expandKeyframe`), `optimiseRealtimeGraph` (initial fixation, onlyNewestState freezing, copy to the full graph, `syncFrom`), `optimiseFullGraph` (100x relative-pose pre-pass with `numIter/3` and `function_tolerance` 1e-3, then 1e-6), `attemptLoopClosure` (drift / uncertainty heuristic, rigid re-alignment), `addLoopClosureFrame`, `synchroniseRealtimeAndFullGraph`, `cleanUnobservedLandmarks`, `mergeLandmark(s)`), `ok_vsb_geom.c` (BSD-3 + MPL: sorted id sets, `overlapFraction` with OpenCV 4.6's filled `cv::circle` rasteriser, Eigen `AngleAxis` / `angularDistance` / `stableNorm`), `ok_vsolve.c` (`ok_vg_solve_native`: ViGraph::optimise on the C graph), `ok_vigraph.{h,c}` extended with read accessors (state / landmark / observation / link views, `cleanUnobservedLandmarks` with the removed observations, the Cauchy flag of converted observations, eliminate hash, solver options).
- The backend reports every graph call it makes through `ok_vsb_hooks.trace` (patch-0010 record layout, pointer fields zeroed) and runs every graph optimise through `ok_vsb_hooks.solve` (logged solver result, or `ok_vsb_solve_native`).
- Reference: patch `0011-backend-call-log.patch`: records with tags 128.. in `problem.bin`: the ENTRY of every `ViSlamBackend` method the frontend / ThreadedSlam call (with the multiframe keypoints, camera models, extrinsics and initial landmark ids in `addStates`), its results at exit (tag | 0x100) and every `MultiFrame::setLandmarkId` (160; includes the writes the backend itself makes). These are the INPUTS of the backend; the graph records of patch 0010 that follow are what the C backend must regenerate. Series 0001-0011 applied to the pristine copy reproduces `reference_build/src` exactly (`diff -r`, only the empty `resources/fast-scnn.pt` placeholder differs).
- Harness `check_ok_vslam.c` (runner glob + skip rule for tags without backend records; `--native-solve N`), tests `reference_tools/okvis_vslam_test.cc`.
- Runs (`runs/okvis_port/reference_runs/MH_01_easy/`): `m7` (mono, 1.3 GB, 313 s) and `s7` (stereo, 2.5 GB, 638 s): `--graph-dump` WITHOUT `--solve-dump` (problem.bin only; graph.bin deleted, m6/s6 keep it). Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`). m6/s6 are KEPT: they alone carry solve.bin (the M4 / PROBLEM-snapshot rows of check_ok_solve / check_ok_vigraph). Images (2.6 GB) and the stereo symlink dir deleted afterwards.

Result (tolerance 0, bitwise; `python3 tools/check_okvis_port.py --tag m7,s7 [--native-solve 1]`): the C backend, driven only by the backend entry records, regenerates the whole mutation-record stream, and the C graph (not the log) carries the state between calls.

| check | m7 (mono) | s7 (stereo) |
|---|---|---|
| backend entry records replayed (of them MultiFrame::setLandmarkId) | 1,799,461 (847,509) | 3,751,753 (1,794,312) |
| graph calls made by the C backend, each compared with the next logged record (tag, graph, argument bytes, result bytes; pointer fields masked) | 5,473,199 records, 5,530,315 argument blobs, 5,367,252 result blobs, 0 mismatches | 11,267,381 / 11,323,646 / 11,097,967, 0 |
| backend results (affected / updated state sets, loop-closure verdict + skip flag, landmark sets, cleaned count) | 18,518, 0 | 18,588, 0 |
| Problem calls of each call vs problem.bin (kind, block / residual-block identity, loss, order) | 10,667,804, 0 | 21,711,319, 0 |
| optimise(): options, states, parameter blocks (value hash, flags, quality / classification), IMU terms | 33,324 + 7.38 M + 33.7 M + 1.83 M, 0 | 33,342 + 6.01 M + 64.0 M + 1.49 M, 0 |
| program order at every Solve() | 608,735, 0 | 1,286,900, 0 |
| NATIVE solve (`OK_NATIVE_SOLVE=1`): every graph optimise() solved on the C graph, result bytes (changed blocks, IMU re-integration state, termination, iterations) vs the log | 11,108 of 11,108 solves, 0 | 11,114 of 11,114 (incl. the sparse loop-closure solves), 0 |
| total | 70,600,469 / 70,611,577 with native solves, 0 mismatches | 128,225,521 / 128,236,635 with native solves, 0 |

Full runner table (`python3 tools/check_okvis_port.py --tag m2,m2cov,s1,m3,m3cov,s3,m6,s6,m7,s7 --native-solve 1 --eigen-tests`, all PASS, exit 0; compared values): check_ok_cam m2 1,952,756 / m2cov 5,167,676 / s1 3,599,828 / m3 463,401 / m3cov 463,401 / s3 639,859; check_ok_err m3 97,704,786 / m3cov 308,341,057 / s3 252,692,033; check_ok_graph m6 2,574,366 / s6 3,622,011; check_ok_imu m2 3,456,254 / m2cov 1,461,926 / s1 1,370,998 / m3 17,309,162 / m3cov 17,309,162 / s3 16,415,106; check_ok_kin m2 4,470,640 / m2cov 12,617,279 / s1 4,546,686 / m3 68,932 / m3cov 68,932 / s3 151,573; check_ok_param m3 14,429,982 / m3cov 83,811,901 / s3 83,877,750; check_ok_problem m6 597,921 / s6 1,276,340 / m7 597,921 / s7 1,276,340; check_ok_solve m6 46,808,362 / s6 69,914,360; check_ok_vigraph m6 65,992,302 / s6 119,990,222 / m7 54,580,286 / s7 95,120,447 (m7/s7 without the solve.bin snapshot rows); check_ok_vslam m7 70,611,577 / s7 128,236,635 (native solves); eigen_eig / eval_shapes / gemm / product_modes, okvis_err / kin_cam / solve_dense / solve_sparse / twopose / vslam tests PASS, 0 mismatches.

Sensitivity: three deliberate mutations of `ok_vslam.c` (keypoint radius x1.4 in the overlap; a 1-ulp error in the loop-closure position ramp; `<` -> `<=` in the least-covisible keyframe choice) give 13.3 M / 348 / 153 k mismatches. ASan/UBSan clean (first 700 k backend records with every 20th solve native, and the whole m7 replay). `okvis_vslam_test.cc` vs the real libraries: filled `cv::circle` 60,000 cases (centres far outside the image, radii 0-14), `overlapFraction` transcription (cv::Mat + cv::circle + bitwise ops + std::set_intersection) 4,000 random multiframes (1-2 cameras, cleared images), `AngleAxisd(q)` / `Quaterniond(AngleAxisd)` / `angularDistance` 200,000 quaternions incl. the `stableNorm` branch: 0 mismatches.

Which paths the sequences exercise (backend calls per run, mono / stereo): 3,681 / 3,681 `addStates`, 11,046 / 11,044 `optimiseRealtimeGraph` (two onlyNewest solves + the 10-iteration window solve per frame), 48 / 112 `attemptLoopClosure` (31 / 35 accepted, the rest rejected by the drift or the 3-sigma heuristic), 31 / 35 `addLoopClosureFrame` + full-graph solves + `synchronise`, 31 (mono) / 35 (stereo) `mergeLandmarks` calls and 270 single `mergeLandmark` calls on stereo, 415 / 663 MST conversions, 212 / 438 `convertToObservations` (frontier expansion and loop-closure frames), 3,681 `applyStrategy` and `cleanUnobservedLandmarks`, 29,323 / 68,227 `removeObservation`, backlog / touched-set replay in `synchronise` after every loop closure. NOT exercised and therefore not verified: `doFinalBa` (`do_final_ba` is off in the apps; the C version is a sketch that lacks `redoPropagationAlways`, `softConstrainExtrinsics`, the second pass), `ViSlamBackend::clear` / `ViGraphEstimator::clear` (returns failure in C), online extrinsics (`do_extrinsics`), `setLandmarkClassification` replay (the classification network is off), depth cameras, RadialTangential8, `addStatesFromOther`, `trackingQuality` (log output only: skipped), `drawOverheadImage` / `saveMap` / `writeFinalCsvTrajectory`, the second pass `eliminateStates_` loop of `synchronise` (dead code upstream: it iterates the map it has just cleared).

Facts found
- The backend never needs the graph's log values: every graph mutation is a deterministic function of the backend inputs, and the covisibility-based decisions (which keyframe leaves, which MST edges) reproduce exactly because the covisibility cache, the MST (module 5d) and the Eigen arithmetic of `attemptLoopClosure` / `synchronise` are exact. The per-keypoint landmark id is frontend state with three writers (frontend matching, `removeObservation` / `cleanUnobservedLandmarks` -> 0, `mergeLandmark(s)` -> into-id) and decides `overlapFraction`; hence patch 0011 logs every `MultiFrame::setLandmarkId` (the merge writes appear BEFORE the MERGELM record: the C++ writes them inside the mutation scope).
- `overlapFraction`: IoU of the disks (radius `int(min(rows,cols)/10 * kptradius_)`, `kptradius_ = 0.09 * detection_threshold / 36`) of the matched keypoints vs all detected keypoints at 1/10 resolution; `min` of the two frames; a frame whose images were cleared (`clearAllImages()` after its pose-graph conversion) contributes no landmarks, so the overlap with it is 0.0, and the `>=` comparisons make the LATER of equal-overlap frames win (a 0.0 overlap still beats nothing). Disk centres use `cvRound` of the float product `pt * 0.1` (round-half-even).
- A filled `cv::circle` is the midpoint circle `Circle()` of drawing.cpp (not `EllipseEx`): the `inside` test uses the radius, off-image circles are clipped per scanline.
- `AngleAxisd(q)` takes `q.vec().norm()` (left fold of the squares) and only falls to `stableNorm` below epsilon; `Quaterniond(AngleAxisd)` is `cos(ha), sin(ha) * axis`; the `Transformation(r, q)` constructors normalise q again (also for `T_WS_set`). The `stableNorm` model assumes the first element is 16-byte aligned (one kernel call), the case of a stack `Quaterniond`.
- `optimiseRealtimeGraph` copies the realtime states / landmarks to the full graph only while no loop closure is running or pending (`isLoopClosing_ || isLoopClosureAvailable_`); otherwise every change is queued (`touchedStates_`, `touchedLandmarks_`, `eliminateStates_`, the backlog of new states) and replayed by `synchroniseRealtimeAndFullGraph` (landmarks and observations of touched landmarks removed and re-added from the realtime graph, links of touched states re-created from clones).
- `ok_vg_solve_native` needs nothing but the graph's Problem bookkeeping: parameter and residual blocks in program order (`ok_problem_program`), constant flags from the Problem, manifold from the block size (7 pose, 4 homogeneous point, 9 speed-and-bias), the terms copied by value (IMU terms deep-copied; the solver's re-integration state is written back into the graph's term), `function_tolerance` / `linear_solver_type` as the backend set them. The solve updates the graph's blocks in place, exactly like Ceres' user-state write-back; no log value is used.

Next: M7b (frontend matching, triangulation, keyframe decision, RANSAC glue; M7c OpenGV subset; BRISK is Codex's). Its interface to the backend is exactly the set of backend entry records of patch 0011 (addStates with the keypoints and descriptors-independent multiframe data, addLandmark / addObservation / removeObservation / setObservationInformation / mergeLandmarks / attemptLoopClosure / addLoopClosureFrame / optimise calls, MultiFrame::setLandmarkId): a frontend port can be validated by regenerating that record stream from the images and IMU (given BRISK features), the backend then consumes it unchanged.

### 2026-10-04 (Claude): module 5 part 2 (M5d: ViGraph / ViGraphEstimator state and every graph mutation) DONE, bit-exact on mono and stereo

Deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h>`): `okvis_port/c/ok_vigraph.{h,c}` (BSD-3 + MPL): the graph state (states with pose / speed-and-bias / extrinsics blocks, IMU links shared by neighbouring states, priors, per-state and per-landmark observation maps, the global observation hash, pose-graph / const / relative links, anyState, cached covisibilities, MST scratch) and every mutation: `addStatesInitialise` (gravity alignment via `ok_imu_init_pose`, priors), `addStatesPropagate` (M1 propagation + `ImuError`), landmarks (add / remove / set / initialised / quality / classification), `addObservation` (information `I * 64/size^2`) / `addExternalObservation` / `removeObservation` / `removeAllObservations`, `computeCovisibilities`, relative pose constraints, `updateLandmarks`, `cleanUnobservedLandmarks`, `eliminateStateByImuMerge` (`ImuError::append`, swap-with-last Problem removals, `anyState_` arithmetic), `mergeLandmark`, freeze / unfreeze of poses and speed-and-biases (incl. `removeInCeres`), `convertToPoseGraphMst` (+ `buildMst`: edges `(-covisibility, (u, v))` sorted like `std::sort`, Kruskal with the rank / path-compression disjoint sets, longest-term edge, keep-or-remove decision per frame, re-conversion of an existing link, `ok_twopose` add / compute), `convertToObservations`, `addExternalTwoPoseLink` (clone as `...Const`), `removeTwoPoseConstLink(s)`, `removeSpeedAndBiasPrior`, and the direct accesses of `ViSlamBackend` (initial fixation, constant / variable landmarks, observation information, state copies, `ImuError::syncFrom`). Every Problem call goes through `ok_problem` and is also appended to an event queue. Serialisers reproduce the PROBLEM payloads of patch 0008 (`ok_vg_imu_snapshot`, `ok_vg_reproj_payload`, ...). `ok_twopose.h` got `hp_live_init` (the live landmark block's initialisation flag read by `convertToReprojectionErrors`).
- Harness `check_ok_vigraph.c` (+ runner glob / skip rule; `check_ok_problem` now skips the new records).
- Reference: patch `0010-vigraph-mutation-log.patch` (new header `OkvisPortVigraphLog.hpp`, hooks in `ViGraph.{hpp,cpp}`, `ViGraphEstimator.cpp`, `ViSlamBackend.cpp`, a read-only `ImuError::portRedoState`). Records (tag >= 32, layouts in `ok_vigraph.h`) go into `problem.bin` right AFTER the Problem calls of the outermost mutation (RAII scope). `optimise()` logs the graph before the solve (every state, the digest of every parameter block incl. landmark quality / classification, the digest of every IMU term) and after it what the solver changed (values, IMU re-integration state). Series 0001-0010 applied to the pristine copy reproduces `reference_build/src` exactly (`diff -r`).
- Runs (`runs/okvis_port/reference_runs/MH_01_easy/`): `m6` (mono 3.2 GB, 334 s) and `s6` (stereo 4.9 GB, 560 s), `--solve-dump --solve-every 20 --solve-full-every 4 [--solve-sparse-full-every 4 for s6] --graph-dump`. Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`).

Result (tolerance 0, bitwise; `python3 tools/check_okvis_port.py --tag m6,s6`): the replay executes every logged mutation on the C graph (5,378,362 / 11,109,083 records, 11,108 / 11,114 optimise calls) and compares

| check | m6 (mono) | s6 (stereo) |
|---|---|---|
| Problem calls of each mutation vs `problem.bin` (kind, identity of every block / residual block through a bijection, loss, order) | 10,667,804, 0 mismatches | 21,711,319, 0 |
| mutation results (new ids, new poses / speed-and-biases, anyState, covisibilities, MST edges, created / removed terms, converted observations, cleaned landmarks) | 402,425, 0 | 635,850, 0 |
| graph before every optimise: states, keyframe flags, per-state counts, digest (flags, value hash, landmark quality / classification) of every parameter block, digest of every IMU term | 7.38 M + 33.7 M + 1.83 M, 0 | 6.01 M + 64.0 M + 1.49 M, 0 |
| PROBLEM snapshot (sampled, 615 / 623 solves): every parameter value, constant flag, manifold kind, residual block with type, loss, parameter blocks and the bytes of its cost-function payload (reprojection, IMU incl. measurements, pose / speed-and-bias / relative-pose priors, TwoPose, TwoPose const) | 3.0 M + 8.4 M, 0 | 4.9 M + 20.0 M, 0 |
| program order at every `Solve()` (hash, full order every 50th) | 608,735, 0 | 1,286,900, 0 |
| hash of all parameter blocks after the solve (solver output applied to the C graph) vs the END record | 22,216, 0 | 22,228, 0 |
| `check_ok_vigraph` total | 65,992,302, 0 | 119,990,222, 0 |

Also 0 mismatches under ASan/UBSan on the first 600 k mutations of m6.

Full runner table (`python3 tools/check_okvis_port.py --tag m2,m2cov,s1,m3,m3cov,s3,m6,s6 --eigen-tests`, all PASS, exit 0): check_ok_cam m2 1,952,756 / m2cov 5,167,676 / s1 3,599,828 / m3 463,401 / m3cov 463,401 / s3 639,859; check_ok_err m3 97,704,786 / m3cov 308,341,057 / s3 252,692,033; check_ok_graph m6 2,574,366 / s6 3,622,011; check_ok_imu m2 3,456,254 / m2cov 1,461,926 / s1 1,370,998 / m3 17,309,162 / m3cov 17,309,162 / s3 16,415,106; check_ok_kin m2 4,470,640 / m2cov 12,617,279 / s1 4,546,686 / m3 68,932 / m3cov 68,932 / s3 151,573; check_ok_param m3 14,429,982 / m3cov 83,811,901 / s3 83,877,750; check_ok_problem m6 597,921 / s6 1,276,340; check_ok_solve m6 46,808,362 / s6 69,914,360; check_ok_vigraph m6 65,992,302 / s6 119,990,222; eigen_eig / eval_shapes / gemm / product_modes, okvis_err / kin_cam / solve_dense / solve_sparse / twopose tests PASS, 0 mismatches. (m6 compares fewer level-2 sparse vectors than the old m5: its `--solve-sparse-full-every` was the default 10 instead of 4; the same records are replayed.)

[SUPERSEDED 2026-10-05 by the m8 / s8 inventory above; the tags named here were deleted] Dump sets kept (`reference_runs/MH_01_easy/`) and what each covers: `run1`/`run2` (M1 canonical IMU dumps + trajectories), `m2`, `m2cov`, `s1` (M1 + M2 kinematics / cameras, mono sampled / denser / stereo), `m3`, `m3cov`, `s3` (M1-M3 incl. error terms / manifolds), `m6`, `s6` (M4 solve.bin + M5 graph.bin + problem.bin + the M5d mutation log: supersede m4, s4 (solve.bin only) and m5, s5 (same three files without the mutation log), all deleted 2026-10-04), `cov`, `kcount`, `scount`, `s2` (trajectory / call-count runs, no dumps). Images re-fetched for the runs and deleted afterwards (`fetch_seq_stream.py MH_01_easy cam0,cam1,imu0`, 2.6 GB; the stereo run uses a symlink dir `MH_01_easy_s`).

Facts found
- The `ImuError` constructor resizes `dPdsigma_` to four zero matrices, so a snapshot always lists four (also before the first redo); long terms (>= 50 measurements, i.e. after merges) are NOT re-integrated by `Evaluate` (only `redo_` stays set), so a solve changes them only through the redo counter / reference biases: after a solve the replay re-integrates only when the redo counter moved.
- The full graph's IMU terms are kept in step with the realtime ones by `syncFrom` after each realtime solve (79 k calls on mono) and back from the full graph after a loop closure; the pose-graph link of the full graph is the `...Const` clone made at conversion time.
- `ProblemImpl*` (the Problem log) and `ceres::Problem*` (the graph's `problem_`) differ: the replay claims the Problem constructed last.
- Never exercised by EuRoC MH_01 (mono or stereo) and therefore NOT ported / verified: `addStatesFromOther`, `freezeExtrinsicsUntil` / `unfreezeExtrinsicsFrom`, `obtainPoseGraphMst`, `addOneSidedDepthError`, online extrinsics (`do_extrinsics`), `softConstrainExtrinsics` / `setExtrinsicsVariable` (logged, not replayed), `PseudoImuError` (no IMU), `ViGraphEstimator::clear` (logged, not replayed), `setLandmarkClassification` and `setObservationInformation` (never called with the shipped configs, replay implemented but untested), `RadialTangential8`.
- Where the port is "exact by construction rather than by replay": `std::sort` of the landmark qualities in `convertToPoseGraphMst` only feeds a `std::set`, so it is skipped; `removeParameterBlock` dependents are never present (the C code would drop them in ascending program order).

Next: M6 `ViSlamBackend` (strategy: `applyStrategy`, `eliminateImuFrames`, keyframe / loop-closure frame sets, `optimiseRealtimeGraph` / `optimiseFullGraph` driving the graph, `synchroniseRealtimeAndFullGraph`, `doFinalBa`) replayed against the same mutation log (its calls into the graph are exactly the logged records, so the backend's decisions can be validated by regenerating the record stream), then the frontend (BRISK is Codex's), RANSAC, place recognition and the full-system replay. A native solve on the C graph (build an `ok_sv_problem` from the graph and run `ok_sv_solve` instead of applying the logged solver output) is the remaining step to close the loop without the log's post-solve values.

### 2026-10-03 (Claude): module 5 part 1 (TwoPose* terms, updateLandmarks, PseudoInverse, ceres::Problem bookkeeping) DONE, bit-exact on mono and stereo; graph bookkeeping (M5d) remains

Deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h>`): `okvis_port/c/ok_twopose.{h,c}` (BSD-3 + MPL: `TwoPoseGraphError::addObservation` bookkeeping, `TwoPoseStandardGraphError::compute` (per-landmark Gauss-Newton blocks with the Cauchy corrector, `PseudoInverse::symmSqrt` marginalisation, the 6x6 eigendecomposition into `J_` / `DeltaX_`), `convertToReprojectionErrors`, `EvaluateWithMinimalJacobians` of `TwoPoseStandardGraphError` / `TwoPoseStandardGraphErrorConst` and of `TwoPoseExtrinsicsGraphError` / `TwoPoseExtrinsicsGraphErrorConst` (all Jacobian-pointer combinations), `okvis::PseudoInverse::symm / symmSqrt / symmSqrtU`),
  `ok_graph.{h,c}` (BSD-3 + MPL: the per-landmark body of `ViGraph::updateLandmarks`, `Vector4d::norm`, the readers of the module-5 dump payloads shared by the harnesses), `ok_problem.{h,c}` (BSD-3, Ceres-derived: the `ProblemImpl` bookkeeping that defines the PROGRAM ORDER: append on add, swap-with-last removal of parameter and residual blocks, dependent-residual removal, constant/variable flags, manifolds; pointer-keyed, so the reference log replays verbatim),
  `ok_eigen.c`: `ok_selfadjoint_eig(n, ...)` generalises the 15x15 model to any n <= 32 (Eigen's closed-form 3x3 tridiagonalisation, the Householder path, the QL iteration, hcoeffs alignment parity as a parameter; `ok_selfadjoint_eig15` is a wrapper), `ok_err.c` exports its small-product models (`ok_lazy_a / ok_lazy_tree_rm / ok_lazy_c`, new `ok_lazy_tree`, `ok_red_tree / ok_red_vec`).
  `ok_solve.c` evaluates the types 7-10 natively from the new PROBLEM payload (hook `on_term` reports the raw outputs); harnesses `check_ok_graph.c` (graph.bin: compute / convert / Evaluate / updateLandmarks replay), `check_ok_problem.c` (problem.bin: every Problem of the run rebuilt mutation by mutation, program order compared at every `Solve()`), `check_ok_solve.c` (now verifies every native TwoPose evaluation against the ORACLE record: kind "twopose"; dumps without the payload still replay).
- Reference: patch `0009-graph-twopose-problem-dump.patch` (`OKVIS_PORT_GRAPH_DUMP_DIR`: `graph.bin` with `G_TP_COMPUTE` (every compute: inputs with the full observation set and parameter snapshots, outputs), `G_TP_CONVERT`, sampled `G_TP_EVAL` (`OKVIS_PORT_GRAPH_TPEVAL_EVERY`, 50) and `G_LM_UPDATE` (`OKVIS_PORT_GRAPH_LM_EVERY` 2 calls x `OKVIS_PORT_GRAPH_LM_SUB` 8 landmarks); `problem.bin` from the Ceres copy's `ProblemImpl` (every Add/Remove/SetConstant/SetVariable/SetManifold, construction/destruction, program-order hash at every `Solve()`, the full order every `OKVIS_PORT_GRAPH_PROBLEM_FULL_EVERY` = 50th); `portDump` payloads of the four TwoPose* classes in the PROBLEM record of patch 0008). Runner options `--graph-dump --graph-tpeval-every --graph-lm-every --graph-lm-sub --graph-problem-full-every`. Series 0001-0009 applied to the pristine copy reproduces `reference_build/src` exactly.
- Tests against the real classes (`--eigen-tests`): `reference_tools/okvis_twopose_test.cc` (random scenarios with 1-2 cameras, 3-40 landmarks, outliers, Cauchy / no loss, duplications, moved reference pose, unnormalised quaternions: `TwoPoseStandardGraphError` addObservation + compute + Evaluate (every pointer combination) + Const clone + convertToReprojectionErrors, `TwoPoseExtrinsicsGraphError(Const)` Evaluate on the real compute, `PseudoInverse` 3x3/6x6, `Vector4d::norm`; 0 mismatches), `eigen_eig_test.cc` now covers n = 3, 6, 9, 15 fixed and 12 / 18 dynamic (0 mismatches).
- Runs (`runs/okvis_port/reference_runs/MH_01_easy/`): `m5` (mono, 2.5 GB: solve.bin 1.9 GB with the same sampling as m4 + graph.bin 178 MB + problem.bin 367 MB, 673 s) and `s5` (stereo, 3.3 GB, 980 s). Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`): the dumps have no effect. Images and the stereo symlink dir deleted afterwards (re-fetch: `fetch_seq_stream.py MH_01_easy cam0,cam1,imu0`, 2.6 GB). m4/s4 kept (same solve.bin sampling; m5/s5 supersede them).

Result (tolerance 0, bitwise, `python3 tools/check_okvis_port.py --tag m5,s5 --eigen-tests`)

| harness | m5 (mono) | s5 (stereo) |
|---|---|---|
| `check_ok_graph`: TwoPoseStandardGraphError::compute (H00_, b0_, J_, DeltaX_, linearisation point, landmarks in S0, marginalised flags) | 503 records, 273,884 values, 0 mismatches | 739 records, 513,748 values, 0 |
| `check_ok_graph`: convertToReprojectionErrors (landmarks back to world) | 270 records, 57,804 values, 0 | 545 records, 125,708 values, 0 |
| `check_ok_graph`: TwoPose{Standard,Const} Evaluate (sampled 1/50 of 3.19 M / 3.04 M calls: residuals, Jacobians, minimal Jacobians, untouched buffers) | 63,862 records, 1,650,670 values, 0 | 60,711 records, 1,736,763 values, 0 |
| `check_ok_graph`: updateLandmarks (quality, initialisation, reset point; 1/2 calls x 1/8 landmarks) | 98,668 landmarks, 592,008 values, 0 | 207,632 landmarks, 1,245,792 values, 0 |
| `check_ok_problem`: program order at every Solve() (51 / 114 Problems; 1.79 M / 3.85 M AddResidualBlock, 1.79 M / 3.84 M removals, 103 k / 120 k parameter blocks, 6.8 M / 13.7 M constant/variable toggles) | 11,157 solves (224 with the full order), 597,921 values, 0 | 11,226 solves (225 full), 1,276,340 values, 0 |
| `check_ok_solve` (M4 replay, now with the native TwoPose terms) | 615 solves, 52,615,971 values (9,038,682 of them the 252,993 TwoPose evaluations), 0 | 623 solves, 69,914,360 values (9,881,394 / 248,704 TwoPose), 0 |

Facts found
- The pipeline never removes a parameter block that still has residual blocks (`RemoveParameterBlock` dependents removed implicitly: 0 on both sequences), so the iteration order of Ceres' pointer-keyed dependent set never influences the program order on MH_01 (the port removes dependents in ascending program order and reports the count; the log would record the real order).
- `problem.bin` also holds the frontend's pose-refinement Problems (`Frontend` `quickSolver`, 49 / 112 Problems with one Solve each): replayed exactly too; `ViGraph` has 2 Problems per run.
- Both extrinsics blocks of a state share the state's id (all `PoseParameterBlock`s of `addStatesInitialise` get `id.value()`); `TwoPoseGraphError` indexes its extrinsics infos by slot `offset + cameraIndex`, never by id.
- `TwoPoseStandardGraphError::compute` is called 503 / 739 times (one per MST edge created by `convertToPoseGraphMst`), `convertToReprojectionErrors` 270 / 545 times; the terms are evaluated 3.19 M / 3.04 M times (165 terms per late window solve). `updateLandmarks` runs once per frame plus once per full-graph optimisation (3712 / 3716 calls).

New Eigen 3.4.0 evaluation-order rules (measured bit-exact by `okvis_twopose_test.cc` and the dump replay)
- Assignment vs construction of a small lazy product decides packets vs trees: `X = A * B` (operator=, `Product` assumes aliasing) evaluates into a temporary of the PRODUCT's preferred storage order (`traits<Product>::Flags`: row-major only if one operand has NoPreferredStorageOrderBit and the other is row-major, otherwise column-major), then copies: with a column-major lhs the temporary is filled with packets along its columns (left folds, case A) whatever the destination's order (`Jmin(RowMajor) = J_.block<6,6>(0,0) * Jerr`); with a row-major lhs the product is row-major (`EvalToRowMajor`) against a column-major temporary, so every coefficient is scalar: the redux tree (this is the M3 rule B, `J = Jmin * J_lift` into a `Map<RowMajor>`). A CONSTRUCTION `const Matrix<6,6,RowMajor> Jmin = J_ * JerrRef` has no temporary: the column-major product goes straight into the row-major object, storage orders disagree, every coefficient is the tree (case D, `ok_lazy_tree`). A product into a block of a row-major matrix (`JerrRef.block<3,3>(0,3) = C * crossMx(d)`) is the aliasing temporary (stella 3x3 rules) plus a copy.
- Dynamic sizes turn trees into left folds: the coefficient redux of a dynamic-size inner expression (`J_.block(0, j, n, 6)` with runtime n, or a product whose depth became Dynamic because `V * VectorXd.asDiagonal()` was evaluated into a `Matrix<3, Dynamic>` temporary in `PseudoInverse::symm`) is `redux_impl<DefaultTraversal, NoUnrolling>`, a plain left fold for every coefficient; dynamic-rows column-major temporaries (`MatrixXd(n, 6)`, the `n x 7` temporary of the extrinsics `J = Jmin * J_lift`, heap-aligned, even n) are slice-vectorised with every row in a packet (left folds).
- `DeltaX_ = -M * (M.transpose() * b0_)`: the inner product with the `Transpose` lhs is case C (vectorised redux of contiguous operands, D = 6 lanes `(p0+(p2+p4)) + (p1+(p3+p5))`, `ok_red_vec`), evaluated first; the negation stays inside the outer lazy product (`(-m)*t`, left folds). `M = W * V_inv_sqrt`, `mH += M * M.transpose()`, `mb += M * (V_inv_sqrt.transpose() * b1)` are case A (the 3x3 transposed product is `ok_m3_mulv_lhsT`); `+=` / `-=` of a lazy product add its coefficient to the destination; depth-2 products (`J^T J`, `J^T r` of the 2-row reprojection Jacobians) are order-free. Diagonal products (`D.asDiagonal() * V^T`, `V * D.asDiagonal()`) are coefficient scalings; `(ev > tol).select(f(ev), 0)` is elementwise; `std::max(a, b)` is `(a < b) ? b : a` (matters for +-0); `1.0e-8 * double(cols) * max` and `epsilon * cols * max` associate left to right.
- `SelfAdjointEigenSolver<Matrix3d>` uses the closed-form `tridiagonalization_inplace_selector<_,3,false>` (no Householder, Q = I or `[1 0 0; 0 m01 m02; 0 m02 -m01]`), `<Matrix<6,6>>` the Householder path whose symv never reaches the paired (alignment-peeled) loop (size <= 8), `<MatrixXd>` n = 12 / 18 the paired loop with heap-aligned hcoeffs: one generalised model (`ok_selfadjoint_eig`) reproduces all of them; the Givens `applyOnTheRight` is alignment-independent in value (packet `c*y - s*x` == scalar `-s*x + c*y`).
- `Vector4d::norm()` is `sqrt((p0+p2) + (p1+p3))`. A column-major `Matrix<double,2,3>` whose buffer the reprojection error filled through a row-major map (`J1_minimal` in updateLandmarks) is read back scrambled: `J1_minimal.transpose() * J1_minimal` uses the logical `(i, a) = data[i + 2a]`; `pos_Ci = pos_Ci / pos_Ci[3]` divides all four entries by the original w.
- The Cauchy corrector inside compute() is OKVIS' own transcription of Ceres' corrector (`mJ = sqrt_rho1 * (mJ - alpha_sq_norm * residual * (residual^T mJ))`: `t_j = r0*J0j + r1*J1j`, `o_ij = (alpha*r_i)*t_j`), not the Ceres `Corrector` arrangement of M4; `residual.norm() > 3.0` is tested on the unscaled residual, `rank < 3 && minDist < 2.99` skips the landmark.

Not ported and why
- `TwoPoseExtrinsicsGraphError::compute` (online extrinsics calibration: `do_extrinsics` is false in every shipped config; its Evaluate is ported and tested on the real compute), `TwoPoseGraphError::strength` (visualisation only; `PseudoInverse::symm` that it uses is ported), `obtainPoseGraphMst` / `Component` (multi-session load/save), `addOneSidedDepthError` (depth cameras), the `weight != 1.0` branch of `addObservation` (never passed).
- Module 5 part 2 (M5d, DONE 2026-10-04, see above; original plan): the `ViGraph` / `ViGraphEstimator` state and the graph mutations themselves: `addStatesInitialise` (gravity alignment: `acos(ez . e_acc)`, `oplus(-increment)`), `addStatesPropagate` (M1 propagation), `addStatesFromOther`, `addLandmark` / `removeLandmark` / `setLandmark`, `addObservation` / `removeObservation` / `addExternalObservation`, priors, `addRelativePoseConstraint`, `computeCovisibilities`, `eliminateStateByImuMerge` (M1 `append`; `anyState_` arithmetic `T_Sk_S = T_WSk^-1 * T_WS`, `v_Sk = C^T v`), `freeze* / unfreeze*`, `convertToPoseGraphMst` (`buildMst`: Kruskal over `-covisibility` weights with `std::sort` of `(w, (u, v))`, edge selection, keep/remove decisions, information halving `setInformation(information())`), `convertToObservations`, `addExternalTwoPoseLink` / `removeTwoPoseConstLink(s)`, `mergeLandmark`, `cleanUnobservedLandmarks`, `removeSpeedAndBiasPrior`. Validation plan: a ViGraph-level mutation log (patch 0010: every public mutation with its arguments, tagged by graph), replayed through the C graph; the Problem calls it emits must equal `problem.bin` and the parameter blocks at every `optimise()` must equal the PROBLEM snapshot, which closes the chain graph state -> program order -> solve (M4) -> updated state. Then M6 (`ViSlamBackend`).

Gotchas for the next modules
- `ok_twopose_add_observation` snapshots the parameter values it is given (the C++ `ParameterBlockInfo` copies at first sight); `compute()` reads the LIVE reference pose through `pose_live[0]` (set it when the block moved in between, as the harness does from the record). A term that was `convertToReprojectionErrors`'d keeps `isComputed_` (upstream never resets it); fresh terms are created instead.
- `check_ok_graph` takes the landmark-info vector index / sparse offset from the record (they depend on the global `addObservation` order, which the per-landmark grouping of the record loses; they are bookkeeping only).
- `problem.bin` records residual-block ids / parameter pointers per `ProblemImpl`; a replay must key its Problems by the `this` pointer (`P_NEW` / `P_DELETE`, pointers are reused by the frontend's short-lived Problems).

### 2026-10-03 (Claude): module 4 (the Ceres solver numerics) DONE, bit-exact on mono and stereo

Deliverables
- C (C99, `<stdint.h> <math.h> <stdlib.h> <string.h> <limits.h>`): `okvis_port/c/ok_solve.{h,c}` + `ok_solve_linear.c` + `ok_solve_internal.h` (BSD-3, Ceres/OKVIS-derived: `Problem` reduction (`RemoveFixedBlocks`, fixed cost), `ComputeStableSchurOrdering` + `LexicographicallyOrderResidualBlocks`,
  `ReorderProgramForSparseCholesky` (block Hessian pattern -> AMD), `BlockJacobianWriter` structure, `ProgramEvaluator` / `ResidualBlock::Evaluate` with the OKVIS manifolds (`PlusJacobian` products) and `CauchyLoss` + `Corrector`, Jacobi scaling, `TrustRegionMinimizer`,
  `TrustRegionStepEvaluator`, `DoglegStrategy` (traditional), `SchurEliminator<Dynamic,Dynamic,Dynamic>` + `DenseSchurComplementSolver` + `EigenDenseCholesky`, `SparseNormalCholeskySolver` + `InnerProductComputer` + `EigenSparseCholesky`), `ok_blas.{h,c}` (BSD-3: Ceres `small_blas` naive kernels),
  `ok_dense.{h,c}` (MPL-2.0: Eigen dynamic-size redux, GEMV (both storage orders + the GemvProduct row-vector fallback), GEBP (SSE2, mr = nr = 4, pk = 8), `triangular_solve_matrix<OnTheRight>`, triangular vector solves, symmetric rank update with `tribb_kernel`, `LLT<..., Lower>` unblocked/blocked, blocking heuristic, `InvertPSDMatrix<Dynamic>`),
  `ok_sparse.{h,c}` (MPL-2.0: `SimplicialLDLT<..., Upper, NaturalOrdering>` analyze/factorize/solve) and `ok_amd.c` (MPL-2.0: `AMDOrdering<int>`, the stella_port transliteration copied with a renamed entry point).
  Harness `check_ok_solve.c` (replays every snapshotted `Solve()` from `solve.bin`; `OK_DEBUG=1` prints the first mismatches), runner `tools/check_okvis_port.py` (`check_ok_solve*`, solver-only tags, Ceres internal include dir for the tests).
- Reference: patch `0008-solver-dump.patch` (hooks header in the Ceres copy + 6 Ceres files + ViGraph/error-term headers; `OKVIS_PORT_SOLVE_DUMP_DIR`, `_EVERY`, `_FULL_EVERY`, `_SPARSE_FULL_EVERY`), runner options `--solve-dump --solve-every --solve-full-every --solve-sparse-full-every` in `tools/run_okvis_reference.py`.
  The patch series 0001-0008 applied to a pristine copy reproduces `runs/okvis_port/reference_build/src` exactly (verified with `diff -r`).
- Tests against the real libraries (runner `--eigen-tests`): `reference_tools/okvis_solve_dense_test.cc` (6.36 M values vs real Eigen 3.4.0 statements and the header-only Ceres kernels `small_blas.h` / `invert_psd_matrix.h`: redux, colwise squaredNorm with destination peeling, GEMV, LLT factor + solve for n = 1..300 incl. failing pivots, `InvertPSDMatrix<Dynamic>` n = 1..12, `MatrixMatrixMultiply` & co. with every kOperation),
  `reference_tools/okvis_solve_sparse_test.cc` (233 k values: `SimplicialLDLT` factors/solutions on random block-sparse SPD systems in Ceres' lower-block CRS layout, `AMDOrdering` permutations). 0 mismatches.
- Runs (`runs/okvis_port/reference_runs/MH_01_easy/`): `m4` (mono, 2.0 GB, 302 s) and `s4` (stereo, 2.3 GB, 961 s), every 20th realtime solve + every full-graph solve snapshotted (level 1), every 4th of those with full vectors (level 2). Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`): the dump has no effect. Images deleted afterwards.

Result (tolerance 0, bitwise, `python3 tools/check_okvis_port.py --tag m4,s4 --eigen-tests`)

| tag | Solve() calls (snapshotted) | compared values | mismatches |
|---|---|---|---|
| m4 (mono) | 11108 (615: 553 DENSE_SCHUR + 62 SPARSE_NORMAL_CHOLESKY) | 45,095,247 (reduced/reordered program 0.66 M, 3607 iterations 9.0 M, Dogleg 3.8 M, Gauss-Newton 1.6 M, reduced dense systems 21.1 M, sparse systems 7.4 M, 252,993 oracle evaluations 1.5 M, END 5.5 k) | 0 |
| s4 (stereo) | 11114 (623: 553 + 70) | 61,525,190 (3678 iterations, 2514 dense systems up to n = 243, 526 sparse systems up to n = 4077 / nnz 104 k, 248,704 oracle evaluations) | 0 |

"Compared" means every scalar of every iteration summary (cost, cost change, gradient norms, step norm, relative decrease, radius, validity/acceptance flags, model cost change, candidate cost), the hashes of x / candidate x / delta / trust-region step / gradient / Jacobi scaling / residuals at every iteration (the full vectors at level 2), every Dogleg call (radius, mu, alpha, step norm, gradient and GN norms, the vectors), every Gauss-Newton solve (mu loop, lm diagonal, solution), the Schur structure, the reduced dense system (lhs, rhs, solution) and the sparse system (J^T J CRS values and pattern, rhs, x) of every linear solve, the parameters handed to the not-ported terms, and the END record (termination type, iteration counts, initial/final cost, hash of all parameter blocks after the solve). Also 0 mismatches under ASan/UBSan and at `-O0` on the whole m4 replay. The two tiny 2-block solves per frame (pose + speed/bias, Schur size 9) are covered as well as the sliding-window solves (350-770 e-blocks, 14-33 f-blocks, Schur size 120-300) and the loop-closure pose-graph solves.

Ceres configuration found (SOLVE record of patch 0008, identical for every call; `ViGraph` ctor + `ViSlamBackend::optimise{Realtime,Full}Graph`)
- `minimizer_type = TRUST_REGION`, `trust_region_strategy_type = DOGLEG`, `dogleg_type = TRADITIONAL_DOGLEG`, `num_threads = 1`, `jacobi_scaling = true`, `use_nonmonotonic_steps = false`, `max_num_consecutive_invalid_steps = 5`, `use_inner_iterations = false`, no callbacks (`enforce_realtime` off), no bounds, no user `linear_solver_ordering`, `dynamic_sparsity = false`, `use_explicit_schur_complement = false`, `max_num_refinement_iterations = 0`, no mixed precision.
- Tolerances: `function_tolerance 1e-6` (1e-3 for the loop-closure pre-pass), `gradient_tolerance 1e-10`, `parameter_tolerance 1e-8`, `initial_trust_region_radius 1e4`, `max 1e16`, `min 1e-32`, `min_relative_decrease 1e-3`, `min_lm_diagonal 1e-6`, `max_lm_diagonal 1e32`; Dogleg constants `mu` in [1e-8, 1], increase factor 10, thresholds 0.25 / 0.75.
- Realtime graph: `linear_solver_type = DENSE_SCHUR`, `max_num_iterations = 10`, `dense_linear_algebra_library_type = EIGEN`: `SchurEliminatorBase::Create` falls to `SchurEliminator<Dynamic,Dynamic,Dynamic>` (the e-set holds landmarks AND speed-and-bias blocks, row sizes 2/15/6/9) with `InvertPSDMatrix<Dynamic>` = `selfadjointView<Upper>().llt().solve(Identity)`; `EigenDenseCholesky` = `LLT<Ref<MatrixXd>, Lower>` in place on the row-major `BlockRandomAccessDenseMatrix` buffer. Loss: `CauchyLoss(1.0)` on reprojection errors only. Manifolds: `PoseManifold` (7/6), `HomogeneousPointManifold` (4/3), speed-and-bias Euclidean. Extrinsics constant (removed by `RemoveFixedBlocks`).
- Full graph: `SPARSE_NORMAL_CHOLESKY`, `sparse_linear_algebra_library_type = EIGEN_SPARSE`, `linear_solver_ordering_type = AMD`: Ceres reorders the parameter BLOCKS with `Eigen::AMDOrdering` on the block Hessian pattern, then `SimplicialLDLT<SparseMatrix<double>, Upper, NaturalOrdering<int>>` on the scalar `J^T J` (`InnerProductComputer`, LOWER_TRIANGULAR block CRS with the `D` rows appended); `max_num_iterations = 15` (5 for the pre-pass with `RelativePoseError` x100).
- Program order matters and is NOT what `Problem::GetParameterBlocks` returns (that is the pointer-sorted `ParameterMap`): the PROGRAM record of patch 0008 records it; M5/M6 must reproduce `Problem`'s bookkeeping (`AddParameterBlock` order, swap-with-last on removal) to regenerate it. The residual-block order from `GetResidualBlocks` is the program order.
- Not reproduced here (other modules): `TwoPoseGraphError` / `TwoPoseGraphErrorConst` evaluations (replayed from ORACLE records at the time; ported in M5 part 1, 165 terms per window solve late in the sequence), the frontend `quickSolver` (`ReprojectionError<RadialTangentialDistortion8>`, default Ceres options, M7d), `ViGraph::updateLandmarks` after the full-graph solve (ported in M5 part 1).

New Eigen 3.4.0 / Ceres 2.2.0 evaluation-order rules (all measured bit-exact, `okvis_solve_dense_test.cc` / `okvis_solve_sparse_test.cc` and the dump replay)
- Dynamic-size `squaredNorm` / `norm` / `dot` / `(a-b).norm()` are reductions over expressions WITHOUT DirectAccess (`cwiseAbs2`, `binaryExpr`, differences), so `first_default_aligned` returns 0 and the address of the data never matters: two 2-lane accumulators over groups of four (lanes (0,1),(2,3)), `res0 += res1`, a trailing pair, `lane0 + lane1`, then the odd tail; sizes < 2 are a plain left fold (`ok_dyn_*`). Ceres' `Dot()` adds `0 + (0 + d)` (one block, one thread).
- `colwise().squaredNorm()` is `member_sum` over `cwiseAbs2` and is vectorised only because Ceres' maps are row-major: `VectorRef(x + pos, cols) += ...` is a LinearVectorized assignment peeled by the DESTINATION pointer (`x` is an aligned `Vector`, so column j is a packet column iff `j >= pos & 1` and within the aligned range); packet columns reduce the rows with `packetwise_redux_impl`'s tree `p0 + ((p1+p2)+(p3+p4)) + ... + tail`, peeled columns with the left fold (`ok_colwise_sqnorm_add`).
- Ceres `small_blas` naive kernels (every dynamic-size `MatrixMatrixMultiply` / `MatrixTransposeMatrixMultiply` / `MatrixVectorMultiply` / `MatrixTransposeVectorMultiply`): columns in groups (odd last column, then a pair, then fours), each accumulator a left fold from 0.0 (`tmp = 0.0; tmp += a*b`), store `+=` / `-=` / `=`. With CUSTOM_BLAS=ON the Eigen path is only taken when ALL four sizes are static, which the dynamic eliminator never does.
- `GemvProduct::scaleAndAddTo` (any `y += A*x` expression) falls back to `dst += alpha * row.dot(x)` when the lhs has ONE row at runtime: a vectorised redux for a row-major map's row, a left fold (`p0 + p1 + ...`) for a column-major map's row (dynamic inner stride, no packets). The triangular vector solvers call `general_matrix_vector_product::run` directly (no fallback). Column-major kernel: per row one chain over the columns from 0 then `y + alpha*acc`; row-major kernel: two lanes over column pairs, `lane0 + lane1`, scalar tail, `y += alpha*cc`.
- GEBP (SSE2, no FMA): rows `[0, 4*(rows/4))` one chain per entry; the next `2*((rows%4)/2)` rows use, for column groups of four, the even-k chain C and odd-k chain D over the first `8*(depth/8)` k's then `C + D` then the remainder (single chain for leftover columns); the last odd row one chain; store `acc*alpha + R`. The blocking heuristic (L1 32 KB / L2 512 KB / L3 16 MB on the reference machine, single thread) only chunks rows at multiples of 4 for this module's sizes, which never changes a summation (`ok_blocking_sizes` reproduces it anyway).
- `LLT<Ref<MatrixXd>, Lower>` (Ceres dense Schur) and `LLT<RowMajor, Upper>` (= the same algorithm on the transposed view, `InvertPSDMatrix<Dynamic>`): n < 32 unblocked (`x = a_kk - leftfold(squares of the strided row)`, `A21 -= A20 * A10^T` through the GEMV expression incl. the one-row fallback, `A21 /= sqrt(x)`); n >= 32 blocked with `blockSize = clamp((n/8/16)*16, 8, 128)`: unblocked diagonal block, `triangular_solve_matrix<OnTheRight, Upper, RowMajor tri>` (panels of 4: `b = leftfold(l*r)`, `other = (other - b) * (1/tri_ii)` in the RowMajor-tri branch, `other *= 1/tri_jj` then `r -= a*b` column updates in the OnTheRight kernel, GEBP updates with alpha = -1), `rankUpdate(A21, -1)` = `general_matrix_matrix_triangular_product` (GEBP on the off-diagonal part, 4x4 diagonal blocks through a zeroed buffer `0 + (-acc)` then `res += buffer`).
- LLT vector solve: L-solve panels of 8 (`x_i /= L_ii` only if `x_i != 0`, `x_j -= x_i * L_ji`, then the column-major GEMV kernel with alpha = -1 for the rows below); U-solve on the adjoint from the bottom (row-major GEMV kernel on the already-solved tail, then inside the panel `x_i -= vectorised dot of the row tail`, `x_i /= U_ii` if non-zero).
- `InvertPSDMatrix<Dynamic>(true, m)`: `m.selfadjointView<Upper>()` copied to both triangles, LLT Upper (transposed Lower), `solve(Identity)` = two `triangular_solve_matrix<OnTheRight>` passes (Upper with the triangular matrix in RowMajor storage, then Lower with it in ColMajor storage) on the row-major identity viewed column-major, panels of 4 with GEBP updates. The e-block inverses are 3x3, 6x6 and 9x9.
- Back substitution `y_block = inverse_ete * y_block` is a RowMajor GEMV into a temporary (`0 + 1*cc`, two lanes + tail).
- `SimplicialLDLT::factorize_preordered<true>`: `y[i] += a` (from 0), `d = y[k]*1.0 + 0.0`, `l_ki = yi / D_i`, `y[Li[p]] -= Lx[p]*yi` (undivided yi), `d -= l_ki*yi`, failure iff `d == 0`; solve: unit-lower column solve (skipping zero rhs entries), `x_i = (1/d_i) * x_i`, unit-upper row solve `tmp -= L*x`. `AMDOrdering` on the block pattern as in stella (symmetrised pattern, `minimum_degree_ordering`); Ceres uses `perm[k]` as "old block at new position k".
- `LexicographicallyOrderResidualBlocks` fills every e-block bucket from its end while scanning forwards: the residual blocks of one landmark end up in REVERSED program order, which fixes the accumulation order of `E^T E`, `E^T F`, `E^T b` and the Schur complement. `ComputeStableSchurOrdering`: `stable_sort` by degree (distinct neighbours) with ties in program order, greedy independent set, then the rest in sorted order; speed-and-bias blocks (degree <= 5) usually sort before landmarks and become e-blocks.
- `ChunkOuterProduct` iterates the chunk's f-blocks in ascending block id (`std::map`), the buffer offsets are in first-encounter order; `UpdateRhs`/`BackSubstitute` use `sj = b_row; sj -= E * v` (naive MV, op -1) etc. exactly as transcribed in `ok_solve_linear.c`.
- The Jacobi scaling is computed at iteration 0 only and applied to every later Jacobian; `delta = step * scaling` elementwise; `candidate_cost` evaluation uses the scratch residual buffer (no numerical difference since the per-block `squaredNorm` is address-independent, see above).

Not ported and why
- `SchurEliminator<2,3,6>` and the other static specialisations (never selected: the detected structure is fully dynamic), `SchurEliminatorForOneFBlock`, SPARSE_SCHUR / ITERATIVE_SCHUR / CGNR, LAPACK / CUDA / SuiteSparse / Accelerate paths, inner iterations, line search, bounds, `SUBSPACE_DOGLEG`, Levenberg-Marquardt, gradient checking, `IterationCallback` (`CeresIterationCallback` only with `enforce_realtime`), `dynamic_sparsity`, mixed precision / iterative refinement; `InvertPSDMatrix` with `assume_full_rank = false` (JacobiSVD; Ceres passes `kFullRankETE = true`).
- `Solver::Summary` bookkeeping beyond what the END record checks (timing, messages), `Problem` mutation API (M5/M6: the port's `ok_sv_problem` is a snapshot; the add/remove bookkeeping that defines the program order is theirs).

Gotchas for the next modules
- Keep the program order of parameter blocks (not the pointer order) in M5/M6; the dump's PROGRAM record is the ground truth for the replay.
- `ImuError::Evaluate` may re-integrate during a solve (state changes even for rejected steps); the replay carries the `ok_imu_error` state through the solve exactly like Ceres does, so a graph-level replay must keep one `ok_imu_error` per IMU link across solves (the M1 state is part of the estimator state).
- The trust-region iteration count observed: mono 2-3 iterations for the tiny solves, 10 (11 ITER records) for most window solves, 15 (16) for the loop-closure solves; `MaxSolverIterationsReached` dominates, `FunctionToleranceReached` ends a quarter of the solves (CONVERGENCE 2680 of 11108).

### 2026-10-03 (Claude): module 3 (manifolds, parameter blocks, error terms) DONE

Deliverables
- C: `okvis_port/c/ok_param.{h,c}` (PoseManifold / HomogeneousPointManifold plus, plusJacobian, minus, minusJacobian; speed-and-bias plus/minus/identity Jacobians; block <-> estimate conversions),
  `ok_err.{h,c}` (ReprojectionError over radtan / equidistant / none, PoseError, SpeedAndBiasError, RelativePoseError, HomogeneousPointError; every constructor and `setInformation` = Eigen `LLT` model for n = 2,3,6,9;
  residual + full + minimal Jacobians for every `jacobians` / `jacobiansMinimal` pointer combination, including which buffers stay unwritten). IMU error was already M1.
  Harnesses `check_ok_param.c`, `check_ok_err.c` (dump replay, also check that unwritten Jacobian buffers stay untouched; `OK_DEBUG=1` prints the first mismatching entries).
- Tests against real code: `reference_tools/okvis_err_test.cc` (1.12 G compared values vs the REAL okvis_ceres classes + Ceres headers/libceres.a for the reference side only: all constructors, failing LLT, HomogeneousPointError and NoDistortion which the
  pipeline never uses, every pointer combination, edge values; verified sensitive by a mutation of one product order), `reference_tools/eigen_product_modes_test.cc` (executable record of the Eigen product orders below).
  Runner: `tools/check_okvis_port.py` now builds tests with `// OK_PORT_TEST_LIBS: ceres glog`, discovers `check_ok_param*`/`check_ok_err*`, skips them for tags recorded before patch 0007 (m2, m2cov, s1), `--err-every` in `tools/run_okvis_reference.py`.
- Reference: patch `0007-error-terms-manifolds-dump.patch` (`OKVIS_PORT_ERR_DUMP_DIR`, `OKVIS_PORT_ERR_EVERY`).
- Runs (MH_01_easy, `runs/okvis_port/reference_runs/MH_01_easy/`): `m3` (mono, sampled, 612 MB), `m3cov` (mono, denser, 1.5 GB), `s3` (stereo, 1.5 GB). Trajectories byte-identical to the canonical runs (mono `dfe3b58e...`/`cc29a746...`, stereo `673fa08f...`/`04965fdc...`): the dump has no effect.
  cam0/cam1 images deleted afterwards (re-fetch: `fetch_seq_stream.py MH_01_easy cam0,cam1,imu0`).
- Call counts, whole run mono / stereo: reproj Evaluate 148 M / 393 M, PoseError 16.8 k / 17.5 k, SpeedAndBiasError 16.8 k / 17.5 k, RelativePoseError 231 / 239, pose plus 1.5 M, plusJacobian 1.8 M, minusJacobian (from the error terms) 63 M,
  homogeneous plus 30 M, plusJacobian 35 M, setInformation/LLT 1.9 M. Never called: HomogeneousPointError, PoseManifold::minus, HomogeneousPointManifold::minus/minusJacobian (random test only).

Result (tolerance 0, bitwise): m3 97.7 M / m3cov 308 M / s3 253 M compared values in check_ok_err (reproj 89 M / 297 M / 237 M), check_ok_param 14.4 M / 83.8 M / 83.9 M, 0 mismatches; m2, m2cov, s1 and all M1/M2 rows still PASS; clean at -O0/-O3 and under ASan/UBSan.

New Eigen 3.4.0 evaluation-order rules (measured, all in `eigen_product_modes_test.cc`)
- Small lazy products (all dims <= 8, `p_k = a_ik*b_kj`): (A) lhs column-major: rows [0, 2*(R/2)) left fold ((p0+p1)+p2)+..., an odd last row is a scalar coefficient with the halving tree (D=3: p0+(p1+p2); D=4: (p0+p1)+(p2+p3); D=6: (p0+(p1+p2))+(p3+(p4+p5))),
  independent of the rhs storage and of a Map/Block/local destination, also for column-vector results (6x6*6 is all left fold, 3x3*3 and 3x4*4x4 have the tree last row).
  (B) lhs AND rhs row-major (J_minimal * J_lift into `Map<RowMajor>`): NO packets, every coefficient is the halving tree; same for a small row-major local with odd column count. (A plain 2x7 row-major local would get packet columns, not used.)
  (C) lhs row-major, rhs column-major (`Map<RowMajor> J1 * S`): every coefficient is a vectorised redux of contiguous operands: lanes (p0+p2),(p1+p3) added horizontally, i.e. D=4: (p0+p2)+(p1+p3).
- `-A*B` keeps the negation inside the lazy product: terms are (-a_ik)*b_kj (use a negated copy of A); `Identity()*2.0` / `Identity()*1.0/var` keep +0 off-diagonals; `setIdentity(); *= -1.0` makes the zeros -0.0.
- A nested `A*B*C` evaluates `A*B` into a temp first (left folds for the 2-row cases); `(S*M).eval()` of a column-major S with a row-major M is case (A).
- Large dimension (9): `9x9 * 9` is the GEMV column kernel: per row a left fold starting from 0.0 then `res = 0 + 1*c`; `-S * Identity9` is a GEBP product with alpha = -1: `res = 0 + (-1 * acc)` (use `ok_gemm` then `0 + (-1*out)`).
- `LLT<Matrix<double,n,n>>` with n < 32 is the unblocked algorithm: `x = a_kk - squaredNorm(row k left of the diagonal)` with a LEFT FOLD of squares (strided block, no packets), `sqrt`, `A21 -= A20*A10^T` is a GEMV (per row fold from 0, then `a + (-1)*c`),
  `A21 /= x` true division per element; on failure (`x <= 0`) the partially factorised matrix (lower part) is what `matrixL()` returns; `matrixL().transpose()` to dense gives L^T with +0 below the diagonal.
- Manifolds: `PoseManifold::minusJacobian`'s `Jq_pinv(3x4)*oplus(q_inv)` is case (A) with R=3 (rows 0-1 left fold, row 2 tree); `plusJacobian`'s `oplus(q)*S(4x3)` is case (A) R=4 (left fold, zeros included).

Not ported and why
- `DepthError` (depth cameras; never constructed), `covariance_` / `covariance()` (6x6 `inverse()`, never read), `jacobiansCorrect` (numeric-diff debug), RadialTangential8 (as in M2), `ParameterBlock` bookkeeping (ids, fixed flags, timestamps: no arithmetic), `PoseManifold::verifyJacobianNumDiff`,
  `TwoPoseGraphError` / `TwoPoseExtrinsicsGraphError` (M5, they contain their own linear algebra).
- ReprojectionError with an invalid projection (|z| < 1e-12) leaves kp uninitialised in C++; the C code zero-fills, dumps/tests exclude the case.

Next: M4 solver (Ceres DENSE_SCHUR / SPARSE_NORMAL_CHOLESKY numerics: trust-region/dogleg, Jacobi scaling, Schur elimination, Eigen LLT/AMD/SimplicialLDLT sub-kernels) with a per-Solve problem snapshot dump (PLAN section 5, row 4); the LLT model here is the 9x9-and-smaller unblocked path only.

### 2026-10-02 (Claude): module 2 (kinematics, time, cameras) DONE; stereo reference made deterministic

Deliverables
- C: `okvis_port/c/ok_time.{h,c}` (Time/Duration incl. ROS quirks; the three time helpers formerly in `ok_imu.c` moved here, `ok_time` typedef now in `ok_time.h`),
  `ok_kin.{h,c}` (Transformation cached + cacheless, sinc/deltaQ/rightJacobian/plus/oplus/crossMx, Eigen quaternion-from-matrix),
  `ok_cam.{h,c}` (PinholeCamera radtan / equidistant / none: every non-batch project/projectHomogeneous/backProject variant incl. intrinsics Jacobians and
  external-parameter forms; `NCameraSystem::computeOverlaps`). Not ported: RadialTangential8, masks, undistort/awareness maps, batch variants.
  Harnesses `check_ok_kin.c`, `check_ok_cam.c` (dump replay), `okvis_port/reference_tools/okvis_kin_cam_test.cc` (random + edge-value test against the REAL okvis_time /
  okvis_kinematics / okvis_cv classes, 56 M compared values incl. +-0, singular points, both cache modes, all variants the pipeline never calls, overlaps for radtan and equidistant pairs).
- Reference: patch `0005-kinematics-camera-dump.patch` (`OKVIS_PORT_KIN_DUMP_DIR`, `OKVIS_PORT_KIN_EVERY`, per-kind call counts at exit), patch `0006-drain-publication-queue.patch`
  (see below), stereo config `configs/okvis_stereo_euroc_deterministic.yaml`, runner options `--kin-every`, `--data-dir`; `tools/check_okvis_port.py --tag a,b,c` takes several tags.
- Runs (MH_01_easy; tags under `runs/okvis_port/reference_runs/MH_01_easy/`): `m2` (mono, sampled), `m2cov` (mono, 3x denser), `s1` (stereo, sampled), `s2` (stereo, no dumps), `kcount`/`scount` (call counts).
  Images deleted (cam0+cam1 = 2.6 GB while running); symlink dataset dirs removed.

Determinism
- Mono with dumps: `final.csv` `dfe3b58e...` and `causal.csv` `cc29a746...` identical to the canonical run (instrumentation has no effect).
- Stereo: `s1` (dumps on) and `s2` (plain) byte-identical, `final.csv` `673fa08f...`, `causal.csv` `04965fdc...`; ATE SE3 0.0193 / Sim3 0.0145 (stock stereo 0.030 / 0.019; `docs/vio_candidates_20261001.md`).
- New determinism hole found (patch 0006): `ThreadedSlam::stopThreading` shuts the publication queue down without draining it, so under load the publishing thread drops the last states
  of `causal.csv` (kcount run: 3679 of 3682 rows; `final.csv` unaffected). Reference now waits for the queue to empty.

Which calls the pipeline really makes (call counts, mono / stereo, whole MH_01 run)
- ctor(r,q) 43.7 M / 83.2 M, inverse 10.1 M / 20.0 M, T*T 21.6 M / 51.3 M, T*v3 6.5 M / 6.6 M, T*v4 9.9 M / 26.9 M, cacheless->cached convert 5.4 M / 25.3 M, sinc/deltaQ 3.9 M / 3.8 M,
  rightJacobian 0.39 M, set(r,q) 11 k, ctor(Matrix4d) 62 / 119, oplus(delta) 4, T() 29 / 34, cacheless C() 3.6 k.
  Never called: set(Matrix4d), setCoeffs, oplusJacobian, liftJacobian, T3x4 (the PoseManifold code of M3 uses its own).
- project(3) 82 M / 210 M, project with Jacobian 69 M / 194 M (both via projectHomogeneous, always without intrinsics Jacobian), backProject 2.0 M / 5.4 M, overlap maps 1 / 2 computations.
  Never called: ...WithExternalParameters, backProject with Jacobian, homogeneous back-projection (replayed only through the random test).
- Dump counts replayed (tolerance 0, bitwise): m2 4.47 M kin + 1.95 M cam values, m2cov 12.6 M + 5.2 M, s1 4.5 M + 3.6 M, all 0 mismatches; every overlap mask (360,960 / 1,804,815 pixels) exact.

New Eigen 3.4.0 evaluation-order rules (all verified bit-exact vs real Eigen by `okvis_kin_cam_test`)
- `Matrix3d::trace()` / `diagonal().sum()` = d0 + (d1 + d2) (unrolled redux tree). `Quaternion = Matrix3` is Shoemake as in Quaternion.h with that trace.
- `Matrix4d * Matrix<4,3>` (small, lazy product, every row packetised): each entry a plain left fold of the 4 products, zeros included (sign of zero matters).
- 2-term products (2x2*2x2, 3x2*2x2, E^T*E, dot of 2-vectors) are order-insensitive; 2x2 inverse = `invdet=1/(a00*a11 - a10*a01)`, entries `a11*invdet, -a10*invdet, -a01*invdet, a00*invdet`.
- `T*v + r` with a Matrix3d*Vector3d product: the product is evaluated first (stella rule), then summed. `Transformation::inverse`: `-(C^T r)` with the all-left `A^T*v` rule, identical for cacheless (`toRotationMatrix().transpose()*r`).
- `sinc(h)*0.5*dAlpha` = `(sinc*0.5)*dAlpha`; `rightJacobian` update `ret += (a*X + b*X2)` elementwise; dot of two 3-vectors `(a0b0+a1b1)+a2b2`.
- Time: `fromSec` does not renormalise (nsec can become 1e9); Duration `fromSec` of a negative whole number gives `floor(d)+1` and a negative nsec; signed normalisation carries only for nsec > 1e9.
- Gotcha: non-const `Transformation::coeffs()` throws for the cached class; always go through a const reference.

Next: M3 (done, see above).

### 2026-10-01 (Claude): phase 0 + module 1 (IMU propagation / preintegration) DONE

Deliverables
- License audit: `docs/okvis2_license_audit.md` (no blocker licenses; must-replace list: Ceres, DBoW2 vocabulary, OpenCV, OpenGV, BRISK/AGAST notices).
- Architecture map, module order, Ceres usage, threading/nondeterminism table: `okvis_port/PLAN.md`.
- Deterministic single-threaded reference: `tools/build_okvis_reference.py` (pinned upstream copy + `okvis_port/reference/patches/000{1..4}*.patch`,
  `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`, provenance in `runs/okvis_port/reference_build/provenance.json`), config
  `okvis_port/reference/configs/okvis_mono_euroc_deterministic.yaml`, driver `tools/run_okvis_reference.py`, data fetch
  `tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,imu0` (streams the nested zip, no temp zip). See `okvis_port/reference/README.md`.
- Module 1: `okvis_port/c/ok_imu.{h,c}` (propagation, ImuError ctor/redoPreintegration/append/Evaluate with residual + Jacobians, initPose),
  `okvis_port/c/ok_eigen.{h,c}` (Eigen 3.4.0 evaluation-order models, MPL-2.0), harness `okvis_port/c/check_ok_imu.c`, runner
  `tools/check_okvis_port.py` (`--eigen-tests` also runs `okvis_port/reference_tools/eigen_*_test.cc` against real Eigen).
  C99, only `<stdint.h> <math.h> <stdlib.h> <string.h>`; licence headers per `okvis_port/NOTICE`.

Determinism of the reference (EuRoC MH_01_easy, mono+IMU, loop closure on, 31 loop closures, 3682 frames, ~5 min/run)
- Canonical result: `final.csv` sha256 `dfe3b58e6a33f487...`, `causal.csv` sha256 `cc29a746ea6feb00...`; ATE (SE3, MH_01 GT) 0.169 m
  (the stock non-deterministic run in docs/vio_candidates scored 0.114 m; mono loop-closure timing makes that spread normal).
- Byte-identical trajectories AND byte-identical IMU dumps (all five `imu_*.bin`) across two runs; identical trajectories across 20 further
  runs executed 10 at a time on a loaded machine (550-590 s wall each instead of 310 s), including `setarch -R` (no ASLR),
  `MALLOC_PERTURB_` and `MALLOC_ARENA_MAX=1` variants.
- Real nondeterminism found and fixed (patch 0002): `Frontend::matchToMap` allocates its descriptor pool uninitialised
  (`cv::Mat(3*N,48,CV_8UC1)`) and a landmark with no usable observation still offers descriptor row 0 (heap garbage) to the matcher.
  With idle CPUs the garbage happened to be harmless (5 of 5 runs identical), under load 11 of 37 runs diverged (one single run, then 5/8, 1/8, 2/10, 2/10 in successive parallel batches) (and always from a
  matcher-input hash difference at one frame, found with the patch-0004 traces: same keypoints/descriptors, different landmark-set hash).
- Other fixes (patch 0001): background loop-closure optimisation is joined inside the frame that launches it (the stock app imports its result
  whenever the thread happens to finish), blocking publication queue, matching-thread segments run sequentially (the segment count stays 4:
  `reprErr` averaging makes it semantic), RANSAC RNG seed 12345 instead of `time(0)+clock()`.
- Not covered: other sequences (stereo+IMU reference added 2026-10-02), `enforce_realtime`. Assumption baked into the reference: the background
  graph optimisation always finishes within one frame.

Module 1 result (`python3 tools/check_okvis_port.py --eigen-tests`, tolerance 0, bitwise `memcmp`, dumps from the reference run)

| record kind | records | compared values | mismatches |
|---|---|---|---|
| `ImuError::propagation` (no cov/jac) + a second call with covariance + Jacobian | 11,039 (all) | 5,342,876 | 0 |
| `redoPreintegration` (state, dPdsigma, P_delta, eigen-decomposition based sqrt information, information) | 4,687 (all) | 7,761,672 | 0 |
| `append` (IMU-state merge) | 1,417 (every 5th); 7,082 (all) in a separate coverage run | 2,346,552; 11,727,792 | 0 |
| `Evaluate` (residual + 4 Jacobians) | 11,728 (every 200th of 2.35 M); 93,817 (every 25th) in the coverage run | 1,858,048; 14,960,752 | 0 |
| `initPose` | 2 | 14 | 0 |

Also 0/0 on a second trajectory (a pre-fix loaded run), clean under ASan/UBSan, identical at `-O0`/`-O2`/`-O3` (with `-ffp-contract=off`).
Eigen cross-check programs (random cases vs real Eigen 3.4.0, same flags): SelfAdjointEigenSolver<15x15> 2,000 matrices bit-exact (eigenvalues + vectors),
10 GEMM statement shapes 0 mismatches (20,000 cases), 6 evaluate shapes 0 mismatches.
Eigen versions: the deps tree (Debian 3.4.0-4) and `external/eigen` (tag 3.4.0) differ only in `DenseBase.h`, `GenericPacketMath.h` (ctor `= default`),
`Macros.h`, `SparseSelfAdjointView.h`; `arch/SSE`, products, Jacobi, Householder, Eigenvalues are identical, so stella's rules apply unchanged.

Eigen evaluation-order facts measured/derived for this module (all in `ok_eigen.c`, `okvis_port/reference_tools/`)
- Reuses stella's 3x3 rules (rows 0-1 left fold, row 2 `a0+(a1+a2)`; `A^T*v` all left; quaternion product/normalise/inverse/toRotationMatrix).
- 15x15 products take the GEBP kernel (`rows+depth+cols >= 20`), SSE2 without FMA => `mr=4, nr=4, pk=8`: rows `[0, 4*(rows/4))` single accumulation chain
  over k; the next `2*((rows%4)/2)` rows use the 1-packet kernel (4-column panels: even/odd k chains C/D, `C+D` after the peeled `k < 8*(depth/8)`, then the
  remainder k added to C; leftover columns: one chain); the last odd row is scalar (one chain). Result written as `0 + 1*acc`.
- `dst = P*Q*P^T` (assignment) evaluates the inner product plainly and the outer product on the transposed problem; a *constructed* `M15 e = A*B*C^T`
  does not (assignment vs construction differ!). `info = S^T*S` is plain. `Map<RowMajor> = A*Block` is the plain column-major result.
- `Matrix15 * Vector15` (column-major GEMV): left fold per row. `Block<3,6>*Vector6`: rows 0-1 left fold, row 2 `(a0+(a1+a2))+(a3+(a4+a5))`.
  `Jm(15x6) * J_lift(6x7 RowMajor)` into a RowMajor map: aliasing temp is column-major 15x7, SliceVectorized, alignment peeling alternates per column
  (15 odd): one end row of every column is a scalar coefficient (pairwise tree), the rest packets (left fold). The 840-byte temp has no static alignment,
  Eigen peels by the runtime address; it was 16-byte aligned in the reference binary for all 11,728 sampled records (`g_eval_temp_parity` in `ok_imu.c`).
- `SelfAdjointEigenSolver<Matrix<double,15,15>>`: tridiagonalisation with the symv kernel (column pairs, packet loop, **peeling by the parity of the destination
  address `hCoeffs+i`**; the member vector is 16-byte aligned), rank-2 update, `dot` via the 2x2-lane `redux`, in-place Householder Q (RowMajor GEMV 2-lane sums +
  odd tail), implicit QL with Wilkinson shift, `numext::hypot` = `p*sqrt(1+(min/p)^2)` (not libm `hypot`), Givens, plane rotations, first-minimum selection sort.
- `Transformation(r, q)` normalises `q` again, so `Quaterniond(...).normalized()` passed into it is normalised twice (changes bits in ~5% of calls).
- `-Matrix::Identity()` has `-0.0` off-diagonals; `Transformation::set` normalises once.

Gotchas for the next modules
- Always replicate the statement form (construct vs assign, expression templates, temporaries) and test it against real Eigen with the shape test programs before trusting a model.
- The dump patch must never read uninitialised members (`information_`, `squareRootInformation_` before the first re-integration): dumps stay byte-identical only if it does not.
- Eval records are sampled (every 200th call of 2.35 M); the ImuError pre-state in them is compact (no measurement deque unless the call re-integrates).

Next (see PLAN.md section 5): M2 kinematics/camera models, M3 error terms, M4 solver (Ceres numerics), M5/M6 graph + strategy, frontend/RANSAC/place recognition, system replay.
`runs/okvis_port/` holds the pristine copy, build, runs and the canonical dumps (`reference_runs/MH_01_easy/run1/dumps`, sha256.txt; EuRoC-derived, never commit); the PNGs were deleted after the runs (to re-run the reference: `python3 tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,imu0`, 1.3 GB, then `tools/run_okvis_reference.py`). Disk used by this work at rest: ~425 MB under `runs/okvis_port` (excl. Codex's `reference_brisk`) + 5 MB data; transient peak was ~3 GB (fetched images 1.3 GB, dumps, 10 concurrent trace runs of ~150 MB `events.txt` each).

## Why it scores what it scores (implementation tricks and settings the paper does not mention)

Collected while porting; each item is something that visibly shapes the estimate.
- Two solvers, two graphs: a realtime window graph (DENSE_SCHUR, 10 Dogleg iterations, Cauchy(1) on reprojection errors only, Jacobi scaling) and a full graph (SPARSE_NORMAL_CHOLESKY with AMD block ordering, `function_tolerance` 1e-3 pre-pass with 1/3 of the iterations when loop-closure relative-pose constraints exist, then 1e-6). Every frame also runs a tiny `onlyNewestState` solve (2 iterations; all landmarks constant, every state but the newest frozen) before the 10-iteration window solve.
- Marginalisation is not Schur-complement marginalisation: states leaving the window are first removed from the visual problem by `convertToPoseGraphMst` (observations of the frames to leave are folded into landmark-eliminated relative-pose terms along a maximum-covisibility spanning tree, Gauss-Newton linearised once at conversion; frames with more than one MST edge keep their observations as well, re-set through the information LLT, so landmarks shared by several edges are counted twice on purpose; the longest-term edge is added whenever it has >= 2 co-observations). The terms are always given the Cauchy loss, even for observations that had none.
- Non-keyframe IMU frames are removed by merging IMU error terms (`ImuError::append`: continue the preintegration, no marginalisation), which keeps the window short without losing inertial information; merged terms of >= 50 measurements are never re-integrated again, so they keep their linearisation biases (a stale linearisation that costs accuracy when the bias moves, but saves time).
- Landmarks are re-classified at every full solve (`updateLandmarks`): quality from the smallest eigenvalue of the reprojection Hessian, landmarks behind a camera are re-seeded along their best ray, and an observation with reprojection error > 2.5 does not count.
- The speed-and-bias prior (sigma 0.1 on speed, bias sigmas from the config) sits on the first state of both graphs; the first pose gets a prior with information 1e8 on position, 1e2 on yaw and none on roll / pitch (gravity alignment fixes them), and an extra position fixation is added/removed around each solve until the system is initialised.
- The full graph copies every realtime result (poses, speed-and-biases, landmarks, IMU term states via `syncFrom`) instead of re-solving them, and the realtime graph takes the full graph's result back after a loop closure; the loop-closure work happens in the frame that launches it in this deterministic reference (in the stock app it races the next frames).
- Keyframe selection is a covisibility game, not a time window: the keyframe that leaves the window is the one with the least co-observations with the current frame / the current keyframe (the oldest keyframe is spared while it still shares >= 2 landmarks), at most 3 conversions per frame; IMU frames beyond `num_imu_frames` that are not keyframes are merged into IMU terms and their `anyState_` (pose / velocity in the frame of the most-overlapped keyframe, overlap = IoU of keypoint disks at 1/10 resolution, ties to the later frame) is what lets the full graph catch up later. A frame whose images were cleared after its conversion has overlap 0.
- The realtime graph freezes (constant, not removed) poses and speed-and-biases older than the 12th state before the oldest kept keyframe AND older than 2 s; frozen blocks keep their residuals, so they stay in the Problem and in the Schur ordering.
- Loop closure is a two-stage correction: `attemptLoopClosure` rigidly re-aligns the trajectory since the matched frame IN THE REALTIME GRAPH (rotation correction spread uniformly over the "loop steps" = distinct `loopId`s, translation weighted by travelled distance), accepted only if the relative position error stays within `drift%/100 + 2% scale + 8%/sqrt(steps)` of the travelled distance, the orientation within `0.0004 + 0.004/sqrt(steps)` rad per step and (for uncertain matches, sigma > 0.1 m) 3 sigma stays below that budget; 17 of the 48 mono attempts and 77 of the 112 stereo attempts were rejected this way. The measured relative pose then enters the full graph as a 100x-information relative-pose term for a short 1/3-iteration, `function_tolerance` 1e-3 pre-pass (removed afterwards) before the real full-graph solve.
- While the full graph optimises (in this deterministic reference: inside the frame that launched it) and until the result is imported, the realtime graph's changes are only QUEUED for the full graph; the import (`synchronise`) re-bases queued states with the pose change (`T_Wnew_Wold`), re-creates the touched landmarks' observations and the touched states' pose-graph links from the realtime graph and copies the full graph's states / landmarks back, so a loop closure always lands one frame later than the frame that detected it.

