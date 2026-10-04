# OKVIS2 pure-C port — handover

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

## Status

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
- Module 5 part 2 (**M5d, next**): the `ViGraph` / `ViGraphEstimator` state and the graph mutations themselves: `addStatesInitialise` (gravity alignment: `acos(ez . e_acc)`, `oplus(-increment)`), `addStatesPropagate` (M1 propagation), `addStatesFromOther`, `addLandmark` / `removeLandmark` / `setLandmark`, `addObservation` / `removeObservation` / `addExternalObservation`, priors, `addRelativePoseConstraint`, `computeCovisibilities`, `eliminateStateByImuMerge` (M1 `append`; `anyState_` arithmetic `T_Sk_S = T_WSk^-1 * T_WS`, `v_Sk = C^T v`), `freeze* / unfreeze*`, `convertToPoseGraphMst` (`buildMst`: Kruskal over `-covisibility` weights with `std::sort` of `(w, (u, v))`, edge selection, keep/remove decisions, information halving `setInformation(information())`), `convertToObservations`, `addExternalTwoPoseLink` / `removeTwoPoseConstLink(s)`, `mergeLandmark`, `cleanUnobservedLandmarks`, `removeSpeedAndBiasPrior`. Validation plan: a ViGraph-level mutation log (patch 0010: every public mutation with its arguments, tagged by graph), replayed through the C graph; the Problem calls it emits must equal `problem.bin` and the parameter blocks at every `optimise()` must equal the PROBLEM snapshot, which closes the chain graph state -> program order -> solve (M4) -> updated state. Then M6 (`ViSlamBackend`).

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
