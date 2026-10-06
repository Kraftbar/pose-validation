# OKVIS2 pure-C port: architecture map, determinism plan, module order

Pinned upstream: `external/vio/okvis2` @ `a2ea00688cd10988aae7bd52ab7935ce9a657ec0` (submodules: brisk `1ef8b42a`,
ceres-solver `85331393` = 2.2.0, DBoW2 `3924753d`, opengv `91f4b19c`). Target: mono+IMU (primary, phones), stereo+IMU
(secondary). Licenses: `docs/okvis2_license_audit.md` (no blockers). Method: same as `stella_port/` (read
`stella_port/HANDOVER.md`): deterministic single-threaded reference with trace dumps, C99 module + `check_*` harness at
tolerance 0, Eigen evaluation-order models measured against real Eigen (3.4.0 here, same version as stella's).
LOC below = lines of hpp+cpp (no tests), counted on the pristine copy.

## 1. Pipeline (what `okvis_app_synchronous` runs)

```
DatasetReader thread --> ThreadedSlam::addImuMeasurement / addImages (blocking queues)
main thread: ThreadedSlam::processFrame()  (one call per camera frame)
  1. wait for IMU covering [frame t - overlap, frame t + overlap]; first frame: ImuError::initPose (gravity alignment)
  2. IMU-propagate last optimised state to the frame time  (ImuError::propagation, no cov/jac)  -> pose guess
  3. Frontend::detectAndDescribe  (BRISK, per camera)
  4. join previous optimisation thread; copy last optimised state
  5. estimator.addStates (ViSlamBackend: new IMU link, ImuError, propagated pose/speed/bias)
  6. Frontend::dataAssociationAndInitialization:
        matchToMap (BRISK Hamming + reprojection gate, num_matching_threads segments)
        matchMotionStereo (2D-2D matches to older keyframe, triangulate, relative-pose/rotation-only RANSAC) 
        [stereo: matchStereo + triangulation]
        doWeNeedANewKeyframe (field-of-view overlap)
        place recognition: DBoW2 query (getFilteredDBoWResult) -> verifyRecognisedPlace (descriptor matching, GP3P RANSAC,
        pose refinement + information) -> estimator.attemptLoopClosure / addLoopClosureFrame -> matchToMap against the
        loop-closure landmarks
  7. thread A: optimisePublishMarginalise = optimiseRealtimeGraph (Ceres), import loop-closure result if available
        (synchroniseRealtimeAndFullGraph), publish state, applyStrategy (IMU-frame merge, keyframe -> pose-graph frame)
  8. thread B (when needsFullGraphOptimisation): ViSlamBackend::optimiseFullGraph (pose-graph/loop-closure Ceres solve)
end: stopThreading -> final trajectory csv (writeFinalCsvTrajectory)
```
There is **no marginalisation prior** (no Schur-complement `MarginalizationError` as in OKVIS1). Old states leave the
window in two ways: (a) non-keyframe frames beyond `num_imu_frames` are merged into their neighbours by
`ViGraphEstimator::eliminateStateByImuMerge` (which calls `ImuError::append`, i.e. longer preintegration), and (b)
keyframes beyond `num_keyframes` are converted to pose-graph frames by `convertToPoseGraphMst`: their landmark
observations are Schur-eliminated into `TwoPoseGraphError` relative-pose terms along a minimum spanning tree
(`buildMst`), plus `RelativePoseError` for loop-closure constraints. Two graphs exist: `realtimeGraph_` (sliding
window) and `fullGraph_` (everything, used for loop-closure/pose-graph optimisation), synchronised by
`synchroniseRealtimeAndFullGraph`.

## 2. Modules, files, LOC

| # | Module | Upstream files | LOC | Notes |
|---|---|---|---|---|
| M1 | IMU propagation / preintegration (**DONE**) | `okvis_ceres/src/ImuError.cpp`, `include/okvis/ceres/ImuError.hpp`, `ode/ode.hpp`, `okvis/PseudoInverse.hpp` (+ `okvis_kinematics` rotation helpers) | 1,984 | C: `okvis_port/c/ok_imu.{h,c}`, `ok_eigen.{h,c}`. Static `propagation`, `ImuError` ctor/`redoPreintegration`/`append`/`Evaluate` (residual + Jacobians), SelfAdjointEigenSolver<15x15>. `PseudoImuError` (IMU disabled) not ported. |
| M2 | Kinematics, time, camera models (**DONE**) | `okvis_kinematics` (942), `okvis_time` (1,295), `okvis_cv/cameras/*` (4,357: PinholeCamera + RadialTangential / Equidistant / NoDistortion, projection + Jacobians, back-projection), `NCameraSystem::computeOverlaps` | ~8,100 upstream, ~1,700 C | C: `ok_time.{h,c}` (Time/Duration), `ok_kin.{h,c}` (Transformation cached/cacheless, sinc/deltaQ/rightJacobian/plus/oplus/crossMx, Eigen quaternion-from-matrix), `ok_cam.{h,c}` (pinhole + radtan/equidistant/none, all project/backProject variants, NCameraSystem overlaps). Not ported: RadialTangential8 (shipped configs use radialtangential or equidistant), image masks, undistort/awareness maps (OpenCV remap, unused by the estimator), batch variants, `Frame`/`MultiFrame` containers (come with the frontend, M7b). Validated by dump replay (patch 0005) and by random tests against the real OKVIS2 classes. |
| M3 | Parameter blocks, manifolds, error terms (**DONE**) | `okvis_ceres`: `ReprojectionError{,Base}` (596), `PoseError`, `SpeedAndBiasError`, `RelativePoseError`, `HomogeneousPointError`, `DepthError` (opt.), `PoseLocalParameterization`, `HomogeneousPointLocalParameterization`, `*ParameterBlock*`, `ErrorInterface` | ~5,700 upstream, ~900 C | C: `ok_param.{h,c}` (PoseManifold / HomogeneousPointManifold plus/minus/Jacobians, speed-and-bias block, block <-> estimate conversions), `ok_err.{h,c}` (ReprojectionError over radtan / equidistant / none, PoseError, SpeedAndBiasError, RelativePoseError, HomogeneousPointError; all constructors, `setInformation` = Eigen `LLT` model, residual + full + minimal Jacobians for every `jacobians` / `jacobiansMinimal` pointer combination). The ImuError residual/Jacobians were already M1. Not ported: `DepthError` (never constructed by the pipeline), `covariance_` (`information.inverse()`, never read), `jacobiansCorrect`, RadialTangential8, `TwoPose*GraphError` (M5). Validated by dump replay (patch 0007) + a random test against the real classes (`okvis_err_test.cc`) + an executable record of the Eigen product orders (`eigen_product_modes_test.cc`). |
| M4 | Nonlinear solver replacing Ceres (**DONE**, bit-exact) | Ceres 2.2.0 `internal/ceres/{solver, program, reorder_program, parameter_block_ordering, graph_algorithms, block_jacobian_writer, program_evaluator, residual_block, corrector, loss_function, trust_region_minimizer, trust_region_step_evaluator, dogleg_strategy, schur_complement_solver, schur_eliminator_impl, detect_structure, dense_cholesky, invert_psd_matrix, sparse_normal_cholesky_solver, inner_product_computer, block_sparse_matrix, eigensparse, small_blas}` + Eigen `OrderingMethods/Amd.h`, `SparseCholesky/SimplicialCholesky*`, `Cholesky/LLT`, GEMV/GEBP/triangular/rank-update kernels | ~2,400 C | C: `ok_solve.{h,c}` + `ok_solve_linear.c` (reduction, Schur / AMD reordering, evaluator with manifolds and Cauchy loss, Jacobi scaling, trust-region minimizer, traditional Dogleg, SchurEliminator<Dynamic,Dynamic,Dynamic>, dense Schur + Eigen LLT, J^T J + SimplicialLDLT), `ok_blas.{h,c}` (Ceres naive small_blas), `ok_dense.{h,c}` (Eigen dynamic-size kernels, MPL), `ok_sparse.{h,c}` + `ok_amd.c` (MPL). Harness `check_ok_solve.c` replays every snapshotted Solve() (patch 0008) through the M1-M3 terms; TwoPose* terms (M5) are answered from recorded outputs. See section 3 and HANDOVER. |
| M5 | ViGraph + ViGraphEstimator (**part 1 DONE**, bit-exact: TwoPose* terms, updateLandmarks, PseudoInverse, ceres::Problem bookkeeping; part 2 = graph state + mutations pending) | `ViGraph.cpp/.hpp` (1,662), `ViGraphEstimator.cpp/.hpp` (1,527), `Component` (607), `TwoPoseGraphError` + `TwoPoseExtrinsicsGraphError` (2,345), `PseudoInverse.hpp`, Ceres `problem_impl.cc` bookkeeping | 3,800 + 2,345 | C: `ok_twopose.{h,c}` (addObservation bookkeeping, `TwoPoseStandardGraphError::compute` with the Cauchy corrector and the `symmSqrt` marginalisation, `convertToReprojectionErrors`, Evaluate of all four TwoPose* classes, `PseudoInverse`), `ok_graph.{h,c}` (`updateLandmarks` per landmark, dump readers), `ok_problem.{h,c}` (program order: append / swap-with-last removal / dependents / constant flags / manifolds), `ok_eigen.c` `ok_selfadjoint_eig(n)` for n = 3 (closed form), 6, 12, 18. Validated by the patch-0009 dumps (`graph.bin`, `problem.bin`: every compute, every Solve()'s program order, sampled Evaluate / updateLandmarks) on m5 / s5 and by `okvis_twopose_test.cc` against the real classes; `check_ok_solve` evaluates the TwoPose terms natively now. Pending (M5d): the state / landmark / observation maps and every graph mutation (`addStates*`, `eliminateStateByImuMerge`, `convertToPoseGraphMst` / `buildMst` (Kruskal), `convertToObservations`, freeze / unfreeze, `mergeLandmark`, `cleanUnobservedLandmarks`, ...), to be validated with a ViGraph-level mutation log (patch 0010) replayed against `problem.bin` and the PROBLEM snapshots. See HANDOVER. |
| M6 | ViSlamBackend (estimator policy) (**DONE**, bit-exact on mono and stereo, native solve closes the loop) | `ViSlamBackend.cpp/.hpp` (3,184) | ~1,900 C | C: `ok_vslam.{h,c}` (frames / IMU frames / keyframes / loop-closure frames, `eliminateImuFrames`, `applyStrategy` (pose-graph conversion, freezing, loop-closure frame conversion, frontier expansion), `optimiseRealtimeGraph` / `optimiseFullGraph`, `attemptLoopClosure`, `addLoopClosureFrame`, `synchroniseRealtimeAndFullGraph`, `cleanUnobservedLandmarks`, `mergeLandmark(s)`, the touched-set bookkeeping while a loop closure runs), `ok_vsb_geom.c` (overlap of two multiframes with OpenCV's filled `cv::circle`, Eigen AngleAxis / angularDistance), `ok_vsolve.c` (`ok_vg_solve_native`: ViGraph::optimise on the C graph). Validated by regenerating the whole mutation-record stream from the backend entry records (patch 0011) with `check_ok_vslam.c`, optionally with `OK_NATIVE_SOLVE=1` (every graph optimise solved natively). `doFinalBa`, `clear`, extrinsics freezing are not exercised by EuRoC and not verified. |
| M7a | Frontend: detection/description (**DONE**: Codex's BRISK leaf, hooked up 2026-10-05, bit-exact on every frame of MH_01 mono + stereo) | `Frontend::detectAndDescribe`, `initialiseBriskFeatureDetectors`; BRISK (Harris scale-space + descriptor) | Codex (BRISK leaf) | `okvis_port/c/ok_brisk*` (Codex), camera awareness maps `ok_cam_awareness_maps`, called from `ok_system.c`. |
| M7b | Frontend: matching, triangulation, keyframe decision (**DONE 2026-10-05**; RANSAC (M7c) and place recognition (M7d) are native since the same day; see HANDOVER) | `Frontend.cpp` (2,600) + `Frontend.hpp` (550), `stereo_triangulation`, `FrameNoncentralAbsoluteAdapter`, `FrameRelativeAdapter` | ~3,600 | `matchToMap(ByThread)`, `matchMotionStereo`, `matchStereo`, `removeOutliers`, `runRansac*`, `doWeNeedANewKeyframe` |
| M7c | OpenGV subset (**DONE 2026-10-05**, bit-exact vs the real OpenGV classes and on every logged run of mono + stereo; see HANDOVER) | `opengv` absolute-pose GP3P, relative-pose Stewenius 5-pt / rotation-only, `Ransac`, `SampleConsensusProblem` (+ okvis adapters / sac problems 1,965) | ~6 k used, ~4.6 k C (3.5 k + 0.4 k generated) | C: `ok_opengv.{h,c}`, `ok_opengv_gp3p_gen.c`, `ok_opengv_stew_gen.c` (generated), `ok_eigen_eigsolver{8,10}`, `ok_eigen_cx.c`, copies of stella's JacobiSVD / QR / FullPivLU. mt19937(12345) + `uniform_int_distribution<int>` reproduced exactly. UB found: GP3P samples with a duplicated 3D point read uninitialised EigenSolver storage upstream (stereo loop closure), isolated and reported by the harness. |
| M7d | Place recognition (**DONE 2026-10-05**, bit-exact on mono and stereo with the reference vocabulary; see HANDOVER) | `Frontend::getFilteredDBoWResult`, `verifyRecognisedPlace`, DBoW2 (`TemplatedVocabulary`, `TemplatedDatabase`, BRISK `FBrisk` = Hamming distance) 4,788 | ~1.2 k C | C: `ok_dbow.{h,c}` (transform, L1 database, introsort order, filtering), `ok_place.{h,c}` + `ok_place_dist.c` (quickSolver through the module-4 solver with the new Levenberg-Marquardt strategy and `CauchyLoss(3)`, H, distinctiveness), the loop-closure block in `ok_frontend.c`. The vocabulary is a reference-run input (`tools/convert_okvis_vocabulary.py`); a product needs an own BRISK vocabulary (k=9, L=3, L1, TF-IDF). DBoW2 clause 3 applies (source-level port): notify the author before any redistribution. |
| M8 | System / driver (**DONE 2026-10-05**: MH_01 mono + stereo trajectories byte-identical from images + IMU csv, see HANDOVER) | `ThreadedSlam` (1,445), `DatasetReader` (~500), `TrajectoryOutput`, `ViParametersReader` (487) | ~0.9 k C | `ok_system.{h,c}`, `ok_config.{h,c}`, `okvis_c_euroc.c`; harness `check_ok_system.c`. |

Threading/IO infrastructure (queues 338, util 617, timing 611, Realsense/ROS) is not ported.

## 3. Ceres usage (what M4 reproduces; measured on MH_01 with patch 0008, see HANDOVER for the full table)

Corrections to the original plan found by the dump: (a) `Problem::GetParameterBlocks` returns the pointer-sorted
`ParameterMap` (`std::map<double*, ...>`), NOT the program order; the stable Schur ordering and the AMD block pattern
depend on the program order (`AddParameterBlock` order modulo swap-with-last removals), which the port tracks with
`ok_problem` (M5 part 1; patch 0009 logs every Problem mutation and the order at every Solve: reproduced exactly on
m5 / s5, and the pipeline never removes a parameter block that still has residual blocks, so the pointer-keyed
dependent set of `RemoveParameterBlock` never matters there). (b) `ViGraph::optimise` runs 3 x per frame, not once (11108 Solve() calls on
MH_01 mono: 11046 DENSE_SCHUR, 62 SPARSE_NORMAL_CHOLESKY = 31 loop closures x (pre-pass + main)). (c) Speed-and-bias
blocks have lower Hessian degree than most landmarks, so the greedy independent set makes them e-blocks too:
the detected structure is <Dynamic, Dynamic, Dynamic> and Ceres uses the generic `SchurEliminator` with its naive
`small_blas` kernels and `InvertPSDMatrix<Dynamic>` (LLT solve on 3x3 / 6x6 / 9x9 blocks); the `<2,3,6>` static
specialisation is never instantiated. (d) `EigenDenseCholesky` is `LLT<Ref<MatrixXd>, Lower>` in place on Ceres'
row-major buffer (blocked path for the 120-300 sized reduced systems, block size 8/16). (e) With EIGEN_SPARSE Ceres
applies `AMDOrdering` to the BLOCK Hessian and runs `SimplicialLDLT<..., Upper, NaturalOrdering>` on the scalar
`J^T J` (lower-triangular block CRS mapped column-major); no SuiteSparse/CHOLMOD anywhere in the reference.
(f) The frontend's place-recognition pose refinement (`Frontend.cpp` `quickSolver`, `ReprojectionError<...>` of the camera's
distortion, `CauchyLoss(3)`, default Ceres options = LEVENBERG_MARQUARDT + SPARSE_NORMAL_CHOLESKY) is a separate Ceres user: done in
M7d (`ok_place.c`, `strategy_lm` of `ok_solve.c`).


* Problem: `ceres::Problem` with `enable_fast_removal = true`, all ownerships `DO_NOT_TAKE_OWNERSHIP`; parameter blocks:
  pose (7, `PoseManifold`: x = [r, q_xyzw], 6-dof local with `deltaQ`), speed+bias (9, Euclidean), extrinsics (7,
  `PoseManifold`, constant unless online calibration), landmarks `HomogeneousPointParameterBlock` (4, 3-dof local
  manifold). Residual blocks are added/removed dynamically (`SetParameterBlockConstant/Variable` for freezing).
* Cost functions (`ErrorInterface`): `ReprojectionError` (2-dim, with `CauchyLoss(1.0)` loss function owned by ViGraph),
  `ImuError` (15-dim, params pose/speedbias/pose/speedbias), `PoseError` / `SpeedAndBiasError` (priors),
  `RelativePoseError` (6-dim), `TwoPoseGraphError`/`TwoPoseExtrinsicsGraphError` (6-dim, 2 poses [+ extrinsics]),
  `HomogeneousPointError`, `DepthError`.
* Options (ViGraph ctor + call sites): `trust_region_strategy_type = DOGLEG` (TRADITIONAL_DOGLEG, defaults),
  realtime graph: `linear_solver_type = DENSE_SCHUR` (set in `ViSlamBackend::optimiseRealtimeGraph`,
  `max_num_iterations = estimator.realtime_max_iterations` (10), `num_threads = realtime_num_threads`
  (OpenMP-threaded evaluator), `enforce_realtime`: iteration callback `CeresIterationCallback` with a **wall-clock** time
  limit (off in the reference config); full graph: `SPARSE_NORMAL_CHOLESKY` (ctor default), `max_num_iterations = 15`
  after a `numIter/3`-iteration pre-pass with `function_tolerance = 1e-3` that adds `RelativePoseError` constraints
  (info x100), then `function_tolerance = 1e-6`. Everything else default: `initial_trust_region_radius 1e4`,
  `min_relative_decrease 1e-3`, `gradient_tolerance 1e-10`, `parameter_tolerance 1e-8`, `jacobi_scaling = true`,
  `min/max_lm_diagonal 1e-6/1e32`, dense algebra = EIGEN, sparse = EIGEN_SPARSE (no SuiteSparse).
* Ordering: no user `linear_solver_ordering`; for DENSE_SCHUR Ceres calls `ComputeStableSchurOrdering` (stable sort of
  parameter blocks by Hessian-graph degree, greedy independent set = landmarks as e-blocks; ties by program order, which
  is the order of `AddParameterBlock` modulo removals). For SPARSE_NORMAL_CHOLESKY: `J^T J` with Eigen
  `SimplicialLDLT<..., AMDOrdering<int>>` (Eigen `Amd.h` + `SimplicialCholesky_impl.h`, MPL-2.0 relicensed; `stella_port/c/sv_eigen_amd.c`
  and `sv_eigen_llt.c` are the starting points).
* Landmark handling around the solve: `ViGraph::updateLandmarks` (re-initialise landmarks behind cameras), quality
  heuristics; `ViGraph::optimise` returns the Ceres summary only for logging.

## 4. Threading model and nondeterminism (and the reference's answer)

Threads in the stock app: dataset reader; main (`processFrame`); realtime optimisation thread (joined at the start of the
next `processFrame`); full-graph (loop closure) thread, **not** joined until the next loop closure or shutdown;
publishing thread (callback writes the causal trajectory); optional visualisation thread; per-camera detection threads
(`parallelise_detection`); `num_matching_threads` matching threads; Ceres OpenMP threads (`realtime_num_threads`,
`full_graph_num_threads`); CNN threads (off).

| source | effect | reference build fix (patch / config) |
|---|---|---|
| full-graph optimisation thread runs concurrently with later frames and the realtime thread; `isLoopClosing_/isLoopClosureAvailable_/needsFullGraphOptimisation_` are plain bools read/written across threads; the frame at which the loop-closure result is imported depends on wall-clock | different trajectories run to run | 0001: `optimisationThread_.join()` right after launch, then the full-graph thread is launched **and joined in the same `processFrame`**. The reference therefore assumes the background optimisation always finishes within one frame (the stock app on a fast machine). |
| publication queue (`PushNonBlockingDroppingIfFull`, size 3) | causal-trajectory rows can be dropped | 0001: blocking push |
| OpenGV RANSAC seeded with `time(0)+clock()` (`SampleConsensusProblem(bool randomSeed = true)`) | RANSAC models differ | 0001: always `rng_alg_.seed(12345u)` (the `randomSeed=false` branch of upstream) |
| `num_matching_threads` partitions keypoints; `reprErr` is averaged over segments, so the count is **semantic** | results depend on the value, not on scheduling | keep 4; 0001 runs the segments sequentially on the calling thread (same partition) |
| Ceres OpenMP evaluator: per-thread partial sums change the floating point summation order | different numerics | config: `realtime_num_threads = full_graph_num_threads = 1`; `OMP_NUM_THREADS=1` |
| `parallelise_detection` (stereo) | none (separate cameras, disjoint outputs) | config: false |
| `enforce_realtime` + `Time::now()` budgets (`CeresIterationCallback`) | iteration count depends on speed | off in the config (upstream default false) |
| pointer-keyed containers (`ceres::Problem` fast-removal `unordered_set<ResidualBlock*>`) | order could depend on addresses | all OKVIS `std::map/std::set` are keyed by ids (`StateId`, `LandmarkId`, `uint64_t`), pointers only as values; Ceres evaluation order follows program (vector) order. Verified empirically (HANDOVER Status): identical outputs across runs, different binaries, ASLR on/off, `MALLOC_PERTURB_`, and 10 concurrent loaded runs. |
| **uninitialised descriptor pool in `Frontend::matchToMap`**: `descriptorPool[im] = cv::Mat(3*N, 48, CV_8UC1)` is never initialised, yet a landmark whose observations are all filtered out keeps `descriptors(Rect(0,0,48,o+1))` with `o = 0`, i.e. one row of heap garbage that is matched against the keypoints | run-to-run (timing/heap-layout dependent) divergence of the matches, seen in 11 of 37 loaded runs (one single run, then 5/8, 1/8, 2/10, 2/10 in successive parallel batches), never when the machine was idle | 0002: `descriptorPool[im].setTo(0)` (found by patch 0004 traces: identical keypoints/descriptors, different landmark-set hash at the same frame) |
| uninitialised `ImuError::squareRootInformation_` / `information_` before the first re-integration | only visible in dumps | 0003 writes zeros for `redoCounter_ == 0` |
| `std::unordered_*` in Eigen/Ceres internals iterated for numerics | none seen | `ComputeStableSchurOrdering` is used by Ceres 2.2 for DENSE_SCHUR |

Deterministic reference recipe: `python3 tools/build_okvis_reference.py` (copy `external/vio/okvis2` -> `runs/okvis_port/reference_build/src`,
apply `okvis_port/reference/patches/*.patch` (0001 schedule, 0002 zero-init descriptor pool, 0003 ImuError dumps, 0004 solver/graph trace, 0005 kinematics/camera dumps, 0006 drain publication queue, 0007 error-term/manifold dumps), build `okvis_app_synchronous` with `-O2 -DNDEBUG -ffp-contract=off
-fno-fast-math`, record `provenance.json`), run with `okvis_port/reference/configs/okvis_mono_euroc_deterministic.yaml`
via `tools/run_okvis_reference.py <seq> --tag run1 [--dump]`.

## 5. Proposed module order and validation strategy

| order | module | validation (all tolerance 0, harness `okvis_port/c/check_ok_*.c`, runner `tools/check_okvis_port.py`) |
|---|---|---|
| 1 | M1 IMU (done) | replay of every `propagation` / `redoPreintegration` / sampled `append` / sampled `Evaluate` record of the reference run + C++ shape tests against real Eigen |
| 2 | M2 kinematics + cameras (done) | dump replay of every Transformation / projection / back-projection call class (sampled) + NCameraSystem overlap maps (all), plus random-case tests against the real OKVIS2 headers (`okvis_port/reference_tools/okvis_kin_cam_test.cc`) |
| 3 | M3 error terms (+ manifolds) (done) | dump residual/Jacobian calls per term type (sampled, patch 0007) and replay (`check_ok_err.c`, `check_ok_param.c`); random-case tests against the real okvis_ceres classes |
| 4 | M4 solver (done) | patch 0008 snapshot per `Solve` + per-iteration records; `check_ok_solve.c` replays every snapshotted solve and compares every iteration, Dogleg/GN internal, reduced system and the final parameters bitwise (mono m4 + stereo s4, 0 mismatches); Eigen/Ceres kernels measured by `okvis_solve_dense_test.cc` / `okvis_solve_sparse_test.cc` against the real libraries. |
| 5 | M5 graph ops + M6 strategy (both done; M6: patch 0011 backend entry records replayed by `check_ok_vslam.c`) | part 1 (done): patch 0009 `graph.bin` (every `TwoPoseStandardGraphError::compute` with its observation set, `convertToReprojectionErrors`, sampled Evaluate of all TwoPose* classes, sampled `updateLandmarks` landmarks) replayed by `check_ok_graph.c`; `problem.bin` (every `ceres::Problem` mutation, program order at every Solve) replayed by `check_ok_problem.c`; `check_ok_solve.c` evaluates the TwoPose terms natively and verifies them against the ORACLE records. Part 2 (next): a ViGraph-level mutation log (patch 0010) replayed through the C graph, checked against `problem.bin` (emitted Problem calls) and the PROBLEM snapshots (parameter values at every optimise), then M6. |
| 6 | M7b/c matching, triangulation, RANSAC | per-frame dumps of match lists, triangulated points, RANSAC inliers (RNG fixed) |
| 7 | M7a BRISK (Codex) and M7d place recognition | descriptor/keypoint equality per frame; DBoW query result lists (own vocabulary will change results: validate with the stock vocabulary first, then retrain and track ATE) |
| 8 | M8 system | frame-by-frame replay of states (`final.csv`, `causal.csv`) against the reference; ATE parity |

Open questions to settle early: (1) how much of Ceres' `TrustRegionMinimizer`/`DoglegStrategy` numerics (iteration-level
double sums, Jacobi scaling, `ComputeEvaluator` threading) needs literal reproduction versus a faithful reimplementation
with identical iteration traces; (2) own BRISK vocabulary vs stock (`small_voc.yml.gz`); (3) GNSS factor interface: a
loosely coupled position factor fits `ViGraph` as another residual block type (see M3 interface), nothing in M1 depends on it.
