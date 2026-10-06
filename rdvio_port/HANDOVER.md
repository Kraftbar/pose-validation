# rdvio_port handover

Goal: dependency-free C99 port of RD-VIO (Jianxff/rd_vio, Apache-2.0), bit-exact against a deterministic reference, as references and
reusable pieces. Method and rules: `okvis_port/HANDOVER.md`, `stella_port/HANDOVER.md`. Plan, module table, OpenCV list, "why it scores what it scores":
`rdvio_port/PLAN.md`. Licences: `docs/rdvio_license_audit.md`.

## M7 (Codex) — image leaf done, bit-exact (2026-10-06)

`c/rd_cv.{h,c}`, `rd_cv_lk.c`, `rd_cv_gftt.c`: dependency-free C99 CLAHE (6,
8×8), padded optical-flow pyramid/Scharr, both initial-flow LK passes, GFTT
Harris/min-eigen, RD-VIO's second response sort, Point2f norm and image-side
status checks. Details, API, limits, licensing and commands:
[`reference_cv/README.md`](reference_cv/README.md).

| Validation | Cases / calls | Compared bytes | Mismatches |
|---|---:|---:|---:|
| Real + synthetic fixtures, normal | 70 | 894,791,369 | 0 |
| Same fixtures, ASan/UBSan | 70 | 894,791,369 | 0 |
| Full MH_01 actual image operations | 11,042 | 12,150,696,647 | 0 |

18 API checks and 3 negative fixtures pass in both leaf builds. Full stream:
3,681 preprocess + 3,681 detect + 3,680 forward/reverse track calls, covering
all image operations the 3,682-frame driver run executes (last frame stays
queued). Observe-only patch `reference/patches/0008-m7-image-stream.patch`,
isolated build `runs/rdvio_port/m7_reference_build/`, tag `m7_stream`.
Trajectory SHA-256 equals canonical exactly:
`f0d60a3e03c1243d750da735a5f5da1ec232237a6d84c4e9a35f15747377a0fe`.
The stream excludes 17,772 unspecified failed-point err bytes; fixtures use
seeded err buffers and compare every byte. Stream ASan/UBSan smoke: 176 calls,
194,591,160 bytes, 0 mismatches. No leak-check or new ATE claim.

Verified independently by Claude (2026-10-06):
- `python3 -B tools/check_rdvio_port.py --modules m7`: 70 cases, 894,791,369 bytes, 0 mismatches.
- A fresh full stream run, `rdvio_port/reference_cv/run_reference.py --tag m7_stream_claude`, reproduced the table:
  - kind 1: 3,681 calls, 12,067,584,264 bytes, 0 mismatches
  - kind 2: 3,681 calls, 41,082,272 bytes, 0 mismatches
  - kind 3: 3,680 calls, 42,030,111 bytes, 0 mismatches
  - trajectory sha256: `f0d60a3e03c1...`
- The excluded err bytes are harmless: `rdvio_extra/src/opencv_image.cpp` never reads `cverr` / `reverse_err`; only the status and the points are used.
- Caveat: the port reproduces the kernels OpenCV dispatches on THIS x86 CPU (SSE LK, AVX2/FMA Sobel via explicit `fmaf`, AVX Harris). Another CPU or an
  ARM build of OpenCV may dispatch differently, so the bit-exactness is with respect to this reference machine.
- Next: M6 (map layer), then M8-M11, which wire `rd_cv_*` into a C system as okvis_port module 8 did.

Dispatch settled: LK is compiled-in SSE/four-lane accumulation, no AVX2 LK
kernel. Sobel uses AVX2/FMA with unfused scalar tails; Harris uses AVX with
SSE/scalar remainders; box sums are double. The old CPU-disable string's
underscored SSE names and wildcard were invalid, though AVX/AVX2/FMA were
disabled. Correctly disabling them changes corner response bytes while LK
stays exact on the diagnostic pairs. Port targets default reference dispatch.

Recheck: `python3 -B tools/check_rdvio_port.py --modules m7 [--sanitize]`;
regenerate fixtures with `--oracle`. Compressed artifacts (~0.46 GiB including
isolated build) remain under owned `runs/` folders. No commits, shared build
changes, or OKVIS edits. M6/M8 still need to wire this API, M3 Poisson filtering,
persistent detector limits and the feature object's ownership. Driver-side
imread/remap and M8 EPnP/Rodrigues remain outside this leaf.

## State (2026-10-04)

* Phase 0 done: licence audit, PLAN.
* Reference done: `tools/build_rdvio_reference.py` (patches 0001-0006 + the Ceres copy `reference/ceres_patches/0006`), `tools/run_rdvio_reference.py`. MH_01_easy: 4 reference runs byte-identical
  (sha256 `f0d60a3e03c1...`), instrumented runs identical to the plain run, ATE SE3 0.1514 / Sim3 0.1491 m (stock 0.1656 / 0.1639).
* **M1 done, bit-exact**: `c/rd_imu.{h,c}` (PreIntegrator reset/increment/integrate/compute_sqrt_inv_cov/predict, `CeresPreIntegrationErrorFactor::Evaluate`
  incl. the prior-factor wrapper, `QuaternionParameterization::Plus`), `c/rd_lie.{h,c}` (hat, expmap, logmap, right_jacobian, S2 bases, `q * v`, 3x3 inverse),
  `c/rd_eigen.{h,c}` (`Matrix<15>::inverse()` = PartialPivLU unblocked + `triangular_solve_matrix<OnTheLeft>`, `LLT(M).matrixL().transpose()`), harness
  `c/check_rd_imu.c`, runner `tools/check_rdvio_port.py`, oracle `reference_tools/rd_imu_oracle.cc` (real classes, random inputs).

## M1 result (`python3 tools/check_rdvio_port.py --tag m1 --oracle`, tolerance 0, bitwise memcmp; also clean under ASan/UBSan)

| source | record kind | records | values compared | mismatches |
|---|---|---|---|---|
| MH_01 dump m1 | `integrate` (all calls: inputs, delta, cov, sqrt_inv_cov, 5 bias Jacobians) | 27,886 | 14,110,316 | 0 |
| MH_01 dump m1 | `predict` (all) | 10,896 | 174,336 | 0 |
| MH_01 dump m1 | `Evaluate` (every 100th) / `Plus` (every 500th) | 3,546 / 747 | 275,565 / 2,988 | 0 |
| MH_01 dump m1b | `Evaluate` (every 4th; masks: no-jac 72.6 k, all-10 9.7 k, prior-style 992 4.1 k, 1020 6) / `Plus` (every 40th) | 88,645 / 9,334 | 6,997,125 / 37,336 | 0 |
| oracle seeds 1,2,3,5 (3 k-20 k each) | `increment` (all jac/cov flag combos, dt 0..0.02, tiny dt, zero rate), `integrate`, `compute_sqrt_inv_cov`, `Evaluate` (random masks, prior), `Plus` | ~45 k-190 k | ~2.4 M-12 M per seed | 0 |

Harness sensitivity was checked by mutating the 3x3 determinant sum order (2,521 mismatches).

## Eigen 3.4.0 facts used / confirmed by M1 (all in rd_*.c, nothing new needed beyond okvis_port's rules except the first two)

* `Matrix<double,15,15>::inverse()` = `partialPivLu().inverse()`: `unblocked_lu` (size <= 16, `col /= pivot` true division, rank-1 update `a_ij -= l_i * u_j`,
  first-maximum pivot), then `P*I`, `triangular_solve_matrix<OnTheLeft>` UnitLower and Upper with SmallPanelWidth 4, `a = 1/diag` and `x *= a` for the Upper solve, GEBP
  updates with alpha -1 (static `gemm_blocking_space`: kc = mc = 15). No other model was needed for the 15x15 case.
* `A*C*A^T` with 9x9 operands is plain GEMM twice (no transposed-problem trick when the destination is a coefficientwise sum of two products), as
  `B*W*B^T` (9x6, 6x6).
* `Map<RowMajor 15xN> = S15 * Map<RowMajor 15xN>`: aliasing temporary, column-major GEMM (rows 15, cols 3 or 4, depth 15).
* `q * v` (Quaternion times Vector3) is `_transformVector`; `q.matrix()` = toRotationMatrix; `AngleAxis` <-> `Quaternion` formulas of Eigen 3.4 (`stableNormalized`).
* Not covered: `logmap` for a quaternion with 0 < |vec| < DBL_EPSILON (Eigen switches to `stableNorm`; the C code uses the plain norm; not hit in 14 M compared values).

## M2 (factors + geometry) and M3 (RNG / RANSAC / PARSAC / Poisson disk): done, bit-exact (2026-10-04)

Files (all `rdvio_port/c/`): `rd_factor.{h,c}` (reprojection / reprojection-prior / rotation-prior `Evaluate`, all Jacobian masks), `rd_geom.{h,c}`
(Wahba 2pt, essential 5pt incl. the Grobner action matrix + EigenSolver<10x10>, homography 4pt, `decompose_essential/homography`, 2-view and n-view
`triangulate_point`, `Track::triangulate / triangulation_angle / get_ / set_landmark_point`, `Frame::get_pose`), `rd_svd.{h,c}` + `rd_qr.{h,c}`
(JacobiSVD V/sigma for fixed 5x9, 8x9 and dynamic Nx4; copy/generalisation of the stella kernels), `rd_rand.{h,c}` (minstd_rand0, libstdc++
`uniform_int_distribution`, LotBox, glibc `srand/rand`), `rd_ransac.{h,c}` (RANSAC for E / R / H, PARSAC for E / H with the persistent 20x20 bin
confidences, error functions), `rd_poisson.{h,c}`; harnesses `check_rd_m2.c`, `check_rd_m3.c`; oracles `reference_tools/rd_m2_oracle.cc`,
`rd_m3_oracle.cc`; patches 0004 / 0005. Reused by path (read-only, not copied): `okvis_port/c/ok_eigen.c ok_dense.c`, `stella_port/c/sv_eigen_svd.c
sv_eigen_qr.c sv_eigen_eigensolver.c` (the stella eigensolver uses `<complex.h>`; fine for a reference port, replace for a strict-header build).

Results (`python3 tools/check_rdvio_port.py --modules m2,m3 --oracle --seeds 1,2,3 --count 20000`, tolerance 0, bitwise; ASan/UBSan clean on M2 oracle data):
see the table printed by the runner; per-row numbers are in the final report of this session and reproducible with the command. Dump runs on MH_01 (patches
0004 / 0005 active) keep the canonical trajectory sha256 `f0d60a3e03c1...` (ATE SE3 0.1514 / Sim3 0.1491 m).

Not covered / not exact yet: `pnp.h` + `IMU_PARSAC` (need OpenCV EPnP, module M8); rotation prior and `triangulation_angle` are never called in the MH_01
configuration (oracle-only validated, no real-data rows); PARSAC when no hypothesis is ever accepted (C++ dereferences an empty vector: crash/UB, port
returns zeros), PARSAC with second-set points outside (-1,1) (C++ out-of-bounds bin index, port returns -1), RANSAC with no model (C++ returns an
uninitialised matrix, mask empty, port zeros); EigenSolver non-convergence (port returns 0 solutions). `Tagged<>` is a bitset of enum flags, trivial in C,
done with the map layer (M6). Sub-second behaviour is deterministic only because the whole run is single threaded.

## New Eigen 3.4.0 rules measured here (on top of okvis_port/HANDOVER.md "Eigen" bullets and stella_port/HANDOVER.md "Eigen 3x3 double evaluation-order rules")
* 2-row lazy products with a column-major lhs (`2x2*2x3`, `2x3*3x3`, `2x3*3x1`): packets cover both rows, every entry is the LEFT fold
  `((a0*b0 + a1*b1) + a2*b2)` (`pmadd` without FMA = `a*b + acc`); a rhs `Transpose` makes no difference. The 2x3 / 2x4 destinations are statically 16-byte aligned,
  so there is no runtime peeling (this is why all reprojection-factor products are consistent). `(-A) * B` equals `-(A*B)` bitwise.
* Nested coefficient-based products are evaluated into a temporary only when `EvalBeforeNestingBit`-flagged (ordinary `Small,Small,Small` products are);
  only the depth-1 `LazyCoeffBasedProductMode` products (outer products) stay lazy.
* **Product storage order matters**: `U * Identity * V.transpose()` is a row-major `PlainObject` (lhs `NoPreferredStorageOrderBit`, rhs row-major) assigned from a column-major
  coefficient product, so vectorisation is off and EVERY entry is the halving tree `a0 + (a1 + a2)` (decompose_homography pure-rotation branch).
* `A * x.homogeneous()` is evaluated as `A.leftCols(n) * x; dst += A.col(n)` (the last column is ADDED, not multiplied): for the 9x3 * 3 part, packet rows (left fold) and one
  scalar last row (tree), then `+ col`. `x.homogeneous()` as an inner-product operand (`p2.homogeneous().transpose() * v`) reduces as the left fold `((a+b)+c)`.
* Tall `ColPivHouseholderQR<Matrix<double,Dynamic,4>>` (and fixed 9x5): `essential.adjoint() * bottom` is `<1,Small,Large>` = coefficient-based (dynamic `redux_dot`
  with the 4-way stride grouping) because `Size==Dynamic && MaxSize>=8` makes a product dimension "Large" and the Block inherits the parent's MAX columns (4 or 5 < 8);
  with MaxCols >= 8 (Nx9, fixed 9x8) it is the row-major GEMV kernel. Rule implemented as `dot_mode = (max columns < 8)` in `rd_qr.c`.
* 3x3 determinant is the row-0 expansion `(m00*(m11 m22 - m12 m21) - m01*(m10 m22 - m12 m20)) + m02*(m10 m21 - m11 m20)`. Polynomial-matrix products (custom scalar) use the
  unrolled halving tree for the inner sum.
* libstdc++ (GCC 13) `uniform_int_distribution<size_t>` over `minstd_rand0` (range 2147483645): fallback path `ret = x - 1; reject while ret >= past; ret /= scaling`; upscaling recursion for ranges beyond the engine range.
* glibc `rand()` = TYPE_3: r[0]=seed (0->1), r[i]=16807*r[i-1] mod 2^31-1 (Schrage) for i<31, r[31..33]=r[i-31], r[i]=r[i-31]+r[i-3] (uint32) discarding 310, output r>>1.

## Why it scores what it scores (additions from M2 / M3 source reading and measurements)
* Visual residual and Jacobians never touch the image plane: `r = hnormalized(T^T R_cs^T (...))` on the tangent plane of the observed bearing, whitened by `sqrt_inv_cov` (2x2, EuRoC: noise/focal).
* PARSAC (used only for the dynamic-feature rejection when `parsac_flag: true`) samples BIN INDICES as if they were point indices (`sample_index` from `draw_by_weight` indexes the data
  directly), with glibc `rand()` re-seeded by `srand(0)` per call, so the "prior-weighted spatial sampling" is effectively a deterministic walk over the first points; the score favours
  inliers spread over many image bins (covariance of the confidence-weighted bin centres), which is the real mechanism that keeps spatially concentrated dynamic features out.
* (source reading, not measured) The essential-matrix RANSAC in `Frame::track_keypoints` runs with threshold 1.0 on normalised-plane points (squared Sampson-like error <= 7.68): almost every KLT match is an inlier, so the
  frame-to-frame epipolar check is far weaker than the nominal 1 px; the separate rotation RANSAC (angle threshold) decides the `FT_NO_TRANSLATION` flag.

## [historical, M7 DONE 2026-10-06] Next module: M7 (OpenCV CLAHE / LK / GFTT, the largest bit-exactness risk), then M6 (map layer) / M8-M11; the whole sliding-window refinement (IMU + visual + marginalisation factors, Ceres solve, `marginalize`) is native and exact since M5, see the M5 section at the end.

## M4 (the Ceres solver: SPARSE_SCHUR + Dogleg) DONE, bit-exact on every dumped Solve() (2026-10-05)

Deliverables (`rdvio_port/`)
* C: `c/rd_solve.{h,c}` (problem reduction `RemoveFixedBlocks`, `ComputeStableSchurOrdering`, **new** `ReorderSchurComplementColumnsUsingEigen` (block AMD of `F^T F - F^T E E^T F`),
  `LexicographicallyOrderResidualBlocks`, `BlockJacobianWriter` structure, `ProgramEvaluator` with the quaternion manifold + `CauchyLoss`/`Corrector`, Jacobi scaling, `TrustRegionMinimizer`, step evaluator,
  traditional Dogleg, the user-state updates of `update_state_every_iteration`, `SetSummaryFinalCost`), `c/rd_solve_linear.c` (Schur elimination copied from `okvis_port/c/ok_solve_linear.c`,
  **new** `BlockRandomAccessSparseMatrix` reduced system, `ToCompressedRowSparseMatrixTranspose`, `SimplicialLDLT`), `c/rd_static.{h,c}` (the all-static kernels of `SchurEliminator<2,3,3>` / `<2,3,Dynamic>`),
  `c/rd_solve_internal.h`; harness `c/check_rd_solve.c`. Reuse BY PATH (read-only, notices stay with the originals): `okvis_port/c/ok_blas.c` (Ceres small_blas), `ok_dense.c`, `ok_sparse.c` (SimplicialLDLT),
  `ok_amd.c`, `ok_eigen.c`. The solver files themselves are MODIFIED COPIES of `okvis_port/c/ok_solve*.{c,h}` (term types, manifold, SPARSE_SCHUR, user state) with the okvis notices kept (see NOTICE).
* Reference: `reference/patches/0006-m4-solver-dump.patch` (rd_vio side: options, Problem snapshot with factor payloads, sampling; `RDVIO_PORT_SOLVE_DUMP_DIR`, `_EVERY`, `_FULL_EVERY`) and
  `reference/ceres_patches/0006-ceres-solver-dump.patch` (hooks inside a COPY of Ceres 2.2.0: adapted from okvis 0008 + an R_SPARSE record for the reduced Schur system). `tools/build_rdvio_reference.py --ceres`
  builds the patched copy into `reference_build/ceres-install-m4` (the pristine `ceres-install` is kept); runner `tools/run_rdvio_reference.py --solve-dump [--solve-every N --solve-full-every M]`.
  The dumped run keeps the canonical trajectory sha256 `f0d60a3e03c1...` (verified for the dump runs m4c and m23).
* Oracle: `reference_tools/rd_m4_oracle.cc` (real Ceres on RANDOM RD-VIO-shaped problems, synthetic smooth cost functions replayed through ORACLE records: 1-24 landmarks, 1-7 frames, pose-only problems,
  constant / unreferenced blocks, dense marginalisation-like factors), `reference_tools/rd_m4_static_test.cc` (the static kernels against the real Ceres `small_blas.h` / `invert_psd_matrix.h`).

Ceres configuration found (SOLVE record, identical for all 1455 sampled solves; `rdvio::Solver::solve`)
* `minimizer_type = TRUST_REGION`, `linear_solver_type = SPARSE_SCHUR`, `trust_region_strategy_type = DOGLEG`, `dogleg_type = TRADITIONAL_DOGLEG`, `num_threads = 1`, `max_num_iterations = 30` (setting `iteration_limit`),
  `max_solver_time_in_seconds = 1e6`, `update_state_every_iteration = true`, no callbacks, `jacobi_scaling = true`, tolerances at the Ceres defaults (function 1e-6, gradient 1e-10, parameter 1e-8,
  initial radius 1e4, min_relative_decrease 1e-3, mu in [1e-8, 1], `max_num_consecutive_invalid_steps = 5`), no user `linear_solver_ordering`, `use_explicit_schur_complement = false`.
* **Library defaults that matter and are not written in the RD-VIO source**: `sparse_linear_algebra_library_type = EIGEN_SPARSE` (the only compiled-in choice; default would be SUITE_SPARSE) and
  `linear_solver_ordering_type = AMD`. With EIGEN_SPARSE + SPARSE_SCHUR Ceres (1) lets `ComputeStableSchurOrdering` pick the e blocks (maximal independent set, ties in program order after a stable sort by degree),
  (2) re-orders the f (Schur complement) blocks with `Eigen::AMDOrdering` on the BLOCK pattern `F^T F - F^T E E^T F` (explicit zeros of the Eigen difference are kept), (3) factorises the reduced system with
  `SimplicialLDLT<Upper, NaturalOrdering>` (`AreJacobianColumnsOrdered` => `ordering_type = NATURAL`, no second AMD), the reduced matrix being the transposed (LOWER block) CRS of a `BlockRandomAccessSparseMatrix`
  whose cells are the f-block pairs (i <= j) in `std::set` order, diagonal cells FULL and read by the upper triangle only.
* Manifolds: quaternion blocks have the `QuaternionParameterization` (Plus = `(q * expmap(dq)).normalized()`, PlusJacobian = `[I; 0]` 4x3); p v bg ba and the inverse depth are Euclidean; blocks fixed through
  `SetParameterBlockConstant` are removed by the reduction (their residual blocks with no free block become `fixed_cost`).
* Problem shapes seen (MH_01_easy, every 5th Solve of 7.3 k): localize_newframe (727 sampled solves: 5 blocks, 224 residual blocks, 1 e block (the rotation), mean 2.5 ITER records, never capped),
  small refinements (606: 16 blocks), sliding-window refinements (120: 305 blocks = ~60 frame blocks + ~242 landmarks as e blocks, 1596 residual blocks, 30 iterations in 52% of them), 2 pose-only initializer solves
  (rows of 2, e = rotation, f = position: the STATIC `SchurEliminator<2,3,3>`).

Result (`python3 tools/check_rdvio_port.py --modules m4 --tag4 m4c --oracle --seeds 1,2,3 --count4 300`, tolerance 0, bitwise; also clean under ASan/UBSan): see the runner table in the session report;
m4c = 1455 solves, 2,112,234 values (reduced program 593,602, ITER 802,094 incl. full level-2 vectors, DOGLEG 171,654, GN 49,446, SCHUR 11,640, SPARSE reduced system 225,135, ORACLE 245,568 evaluations values, END 13,095);
oracle seeds 1-3 x 300 random problems: 6.52 M / 6.90 M / 6.78 M values; six more seeds x 400 problems (ad hoc): 8.7-9.9 M values each; all 0 mismatches.

New rules measured here (Ceres 2.2.0 / Eigen 3.4.0; on top of okvis_port/HANDOVER.md "M4")
* `update_state_every_iteration` is a state machine the replay MUST model: after every `FinalizeIteration` (iteration 0 included) the USER state of every reduced block := the best `parameters_`; cost functions that
  read Frame / Track memory outside their arguments see that user state, not the candidate: `CeresPreIntegrationErrorFactor` reads `bg_i_0`/`ba_i_0` from frame i's USER bg/ba (so `dbg` is non-zero when a step was
  accepted and the Jacobian is evaluated at the new point, before the callback runs), `CeresReprojectionPriorFactor` / `CeresRotationPriorFactor` read the reference pose and inverse depth, the prior factor reads frame i.
  The snapshot stores such memory as `live` entries (pointer + value); the port resolves them to the parameter's user state when the pointer is a problem block.
* `Solver::Summary::final_cost` is `min(initial_cost, cost of every recorded iteration)`, NOT `minimum_cost + fixed_cost`: a rejected step's candidate cost can be below the cost of the kept parameters.
  A problem whose blocks are all constant returns CONVERGENCE with `num_successful_steps = num_unsuccessful_steps = -1`, `num_eliminate_blocks = -1` in the (still written) reduced-program record.
* `SchurEliminatorBase::Create`: row 2 + e 3 + f 3 -> `<2,3,3>`, row 2 + e 3 + other f -> `<2,3,Dynamic>`, row 2 + any other e -> `<2,Dynamic,Dynamic>`; everything else (RD-VIO's windows have rows 2 / 15 / 165 and
  e blocks of 1 AND 3) is the fully dynamic eliminator. `small_blas.h` takes the Eigen path ONLY for the matrix-MATRIX kernels with all four dimensions static; the matrix-VECTOR kernels are always the naive loops.
  All-static kernels (rows-major Maps, `block.noalias() op= ...`): `MTM<2,3,2,3,+1>` = `C + (a0 b0 + a1 b1)`; `MTM<3,3,3,3,op>` = halving tree `a0 b0 + (a1 b1 + a2 b2)`; `MMM<3,3,3,3,op>` with row-major lhs and rhs:
  destination columns 0,1 are one packet (left fold over k), column 2 the scalar tail (halving tree). `InvertPSDMatrix<3>` (kSize < 5, assume_full_rank) is `m.inverse()` = closed-form cofactor inverse of the FULL
  row-major matrix with the determinant `(cofactors .* m.col(0)).sum()` on a strided column = halving tree `c0 m00 + (c1 m10 + c2 m20)` (the dynamic `InvertPSDMatrix<Dynamic>` is the LLT solve of okvis M4);
  `y_block = InvertPSDMatrix<3>(..) * y_block` (row-major Matrix3d temporary times a Map<Vector3d>) is a LEFT fold.
* `ReorderSchurComplementColumnsUsingEigen` pattern: the e blocks keep their order; f blocks are permuted by `AMDOrdering(block_schur_complement)` with `parameter_blocks[ne + i] = old[ne + perm[i]]`
  (same `ok_amd_order` as okvis, input = full symmetric block pattern incl. the diagonal).
* Dogleg on these problems (MH_01 sample): every one of the 3732 full Gauss-Newton computations was inside the 1e4 initial radius, none failed in the LDLT (no mu increase), so the interpolated dogleg branches
  are never taken by the full steps; the trust-region logic acts through REJECTED steps only: window solves take on average 1.4 successful steps in 30 iterations, 95% of their steps are rejected
  (the radius halves, the same Gauss-Newton direction is re-scaled), 52% of the window solves end at the iteration cap; localize_newframe / pose-only solves never reject (1 step, converge).
  So the window refinement is effectively one Gauss-Newton step plus a long tail of rejected shrinking steps (cheap: no new Jacobian), and `iteration_limit` only changes how long the tail runs.

Not covered / not exact yet
* (superseded by M5) the marginalisation factor was replayed through ORACLE records; it is native now. `SchurEliminator<2,3,4|6|9>` and the other static specialisations cannot occur with these block sizes (tangent 3 / 1) and
  are not implemented; `<2,3,Dynamic>` and `<2,3,3>` are covered by the oracle runs; SPARSE_NORMAL_CHOLESKY / DENSE_SCHUR are not part of this port (okvis_port has them).
* Only MH_01_easy was dumped (the phone / other-EuRoC problem mixes are not sampled); sampling is every 5th Solve (level 1) with every 4th of those at level 2 (all vectors).
* Solves that fail in Eigen's LDLT (d == 0: mu increase path) and `step_is_valid == false` paths occur in the random oracle problems (exact) but never in MH_01.

## Gotchas
* `RDVIO_PORT_SOLVE_DUMP_DIR` dumps are ~190 B/value; `--solve-every 5` is 278 MB on MH_01 (a few percent of that is the marginalisation ORACLE jacobians). The patched Ceres copy needs ~140 MB of build tree.
* The M2 / M3 dump rows need a dump run with the channels of patches 0004 / 0005 (`--dump --dump-every "integ=0,pred=0,pie=0,plus=0"` = ~230 MB; the m2 / m3 dumps of the previous session were deleted).
* Score with `external/gnss/venv/bin/python` (numpy); the system python has none.
* Do not use the stock `external/vio3/rd_vio/build` for comparisons of bits: it is fast-math, Ceres 1.14, Eigen 3.3.7.
* `run_rdvio_reference.py --dump` needs the build to contain patch 0003 (it does by default).
* Dump sizes: `integ=1` is ~170 MB per MH_01 run; `pie=4` ~290 MB. Delete `runs/rdvio_port/m1*/…/dump` when done (regenerable in 2 min).

## M5 (marginalisation prior: `marginalize` + `Evaluate`) DONE, bit-exact; the whole window solve is native (2026-10-05)

Deliverables (`rdvio_port/`)
* C: `c/rd_marg.{h,c}` (`rd_marg_init` = `MarginalizationFactor::MarginalizationFactor`, `rd_marg_eval` = `CeresMarginalizationFactor::Evaluate` (residual + row-major Jacobians against the kept
  linearisation points, `S r + infovec`), `rd_marg_marginalize` = `marginalize(index)`: information matrix from the old prior / victim IMU factor / victim-landmark reprojection factors, landmark Schur
  complements in ascending track id, victim-frame Schur complement (`Matrix15::inverse()`), `SelfAdjointEigenSolver`, 1e-8 cut, `sqrt_inv_cov`, `infovec`, new linearisation points),
  `c/rd_seig.{h,c}` (Eigen `SelfAdjointEigenSolver<MatrixXd>` model for n <= 300: generalised copy of okvis `ok_selfadjoint_eig` with heap storage), harness `c/check_rd_m5.c`.
  `rd_solve.{h,c}` / `check_rd_solve.c` now evaluate the marginalisation factor natively (`RD_SV_T_MAR` = 6 with a payload; the synthetic factors of the M4 oracle are `RD_SV_T_SYN` = 7 and keep the ORACLE replay);
  the ORACLE records of the real factor are only compared with the native outputs (row `marg`). Reused by path: `okvis_port/c/ok_dense.c` (`ok_gebp`, `ok_gemv_row`, `ok_blocking_sizes`), `rd_eigen.c`, `rd_imu.c`, `rd_factor.c`.
* Reference: `reference/patches/0007-m5-marginalization-dump.patch` (channel `marg`, `RDVIO_PORT_DUMP_EVERY=marg=n`, `RDVIO_PORT_MARG_FULL_EVERY=n` for the stage matrices; marginalisation factor payload in the 0006 solve snapshot),
  oracles `reference_tools/rd_m5_oracle.cc` (real `Map` / `Frame` / `Track` / `PreIntegrator` / factors: random windows of 2-13 frames with tracks, chains of `Map::marginalize_frame(0)` + new frames, random `Evaluate` calls with random Jacobian masks) and
  `reference_tools/rd_world.h` (shared window builder; `rd_m4_oracle.cc` now also puts REAL marginalisation factors into half of its dense marginalisation-like factors). Runner: `tools/check_rdvio_port.py --modules m5`
  (`--tag5`, `--chains5`, `--steps5`); the m4 row now defaults to the dump tag `m5` (the old `m4c` dump has no marginalisation payload and is not replayable).
* Dump recipe (MH_01, own images under `external/vio/data/rdvio_m5/MH_01_easy`, deleted afterwards): solve + marg dump `run_rdvio_reference.py MH_01_easy --tag m5 --data-dir ... --dump --dump-every "integ=0,pred=0,pie=0,plus=0,rpe=0,rot=0,wahba=0,ess5=0,hom4=0,decess=0,dechom=0,tri2=0,trin=0,trk=0,tang=0,glp=0,slp=0,fess=0,frot=0,fhom=0,pess=0,phom=0,pois=0,marg=3" --solve-dump --solve-every 5 --solve-full-every 4`
  with `RDVIO_PORT_MARG_FULL_EVERY=3` (90 s, 222 MB marg.bin + 310 MB solve.bin); the M2 / M3 dump (`--tag m23 --dump-every "integ=0,pred=0,pie=0,plus=0"`, 93 s, 1.2 GB incl. the complete marg.bin = all 617 marginalisations of MH_01) is what the m2 / m3 / m5 rows of the table below used (deleted afterwards like the images and the oracle dirs; `runs/rdvio_port/m5` (0.5 GB: marg.bin every 3rd call + solve.bin) is kept and is the default dump of the m4 / m5 rows; m1 / m4c are older dumps of earlier sessions, untouched).
  All dumped runs keep the canonical trajectory sha256 `f0d60a3e03c1...` (ATE SE3 0.1514 m, Sim3 0.1491 m).

Result (`python3 tools/check_rdvio_port.py --modules m1,m2,m3,m4,m5 --oracle --seeds 1,2,3 --tag m1 --tag2 m23 --tag3 m23 --tag4 m5 --tag5 m23 --count 20000 --count4 300 --chains5 30 --steps5 4`, tolerance 0, bitwise; exit 0):

| row | source | records | values compared | mismatches |
|---|---|---|---|---|
| m1 | MH_01 dump m1 (integ 27,886 / pred 10,896 / pie 3,546 / plus 747) | 43,075 | 14,563,205 | 0 |
| m1 | oracle seeds 1,2,3 x 20000 | | 12.04 M / 12.05 M / 12.09 M | 0 |
| m2 | MH_01 dump m23 (rpe 459,072, slp 397,921, glp 70,745, trk / trin 34,472, tri2 400, ...) | | 3,523,610 | 0 |
| m2 | oracle seeds 1,2,3 x 20000 | | 2.98 M / 2.98 M / 2.97 M | 0 |
| m3 | MH_01 dump m23 (fess 3,681, frot 3,680, pess 440, pois 7,361, fhom 1) | | 5,073,925 | 0 |
| m3 | oracle seeds 1,2,3 x 20000 | | 1.41 M / 1.43 M / 1.40 M | 0 |
| m4 | MH_01 solve dump m5 (1455 solves, all factors native): reduced 593,602 / iter 802,094 / dogleg 171,654 / gn 49,446 / schur 11,640 / sparse 225,135 / end 13,095 / **marg 10,676,508 (3,847 Evaluate calls)** | 1,455 solves | 12,543,174 | 0 |
| m4 | oracle seeds 1,2,3 x 300 random problems (half of the dense marginalisation-like factors are REAL marginalisation factors: marg 507 / 509 / 724 Evaluate calls, 0.74 M / 0.82 M / 1.09 M values) | 899 solves | 7.57 M / 6.21 M / 8.10 M | 0 |
| m5 | MH_01 dump m23, marg.bin: every marginalisation (priors 12 x 15 = 180 (588) and 13 x 15 = 195 (29)), 168,291 victim landmarks / 1,151,899 observations; outputs: lin 118,928, sqrt_inv_cov 20,153,925, infovec 111,495; stage matrices on 155 records: infomat 5,055,750, infovec 27,990, eigenvalues 27,990, eigenvectors 5,055,750 | 617 | 30,551,828 | 0 |
| m5 | MH_01 dump m5 (every 3rd call) | 206 | 11,331,816 | 0 |
| m5 | oracle seeds 1,2,3 (30 chains x 4 steps, windows of 2-13 frames incl. direct `marginalize(index > 0)` calls, priors 15 ... 180): marg 152 / 144 / 151 records, 6.94 M / 8.53 M / 7.63 M values; random Evaluate calls (random Jacobian masks) 450 x 3 records, 2.74 M / 3.67 M / 2.99 M values | | 9.67 M / 12.20 M / 10.62 M | 0 |

### The dynamic eigensolver was NOT a blocker: new Eigen 3.4.0 facts (Ceres/Eigen flags as before)
* `SelfAdjointEigenSolver<MatrixXd>` for n = 15 ... 195 is exactly the okvis model with larger scratch: `tridiagonalization_inplace` is NOT blocked at any size (column-by-column symv + rank-2 update; the symv peeling follows the parity of the
  heap-aligned `m_hcoeffs`, 0), and `mat = HouseholderSequence(mat, hCoeffs).setLength(n-1).setShift(1)` takes the `is_same_dense` in-place branch for every n, so the blocked application (BlockSize 48) is never used. The QL iteration, `hypot`, Givens
  and the first-minimum selection sort are unchanged; the sort only runs on `Success` (after 30 n iterations without convergence Eigen returns the UNSORTED result and does not throw; never hit in real data or the oracle).
* Dynamic GEMM (`block -= (A*inv)*B`, `J^T J`, `S * dr`): `general_matrix_matrix_product` with `computeProductBlockingSizes` (`ok_blocking_sizes`, kc 1): for depth <= 195 it never changes a sum (chunk boundaries fall on multiples of 4; removing the blocking in the
  port gives 0 mismatches), but the loops are kept. A nested product `(A*inv)*B` is evaluated first into a zeroed column-major temporary (`0 + 1*acc`), then the outer GEMM runs with alpha = -1 on the destination (`R + (-1)*acc`); `block += A^T B` with a dynamic depth
  accumulates in place. `Map<RowMajor> = S * Map<RowMajor>` evaluates into a column-major temporary (rows N, cols 3 / 4, depth N) and copies.
* Column-major GEMV (`general_matrix_vector_product<ColMajor>`): `block_cols = 16` for `cols >= 128` (and lhsStride*8 < 32000): one accumulator per row per 16-column block, `y = acc*alpha + y` after every block (matters for the 135 ... 195 column whitening `S * r`).
* `diag.asDiagonal() * V.transpose() * b` (lazy diagonal product as the lhs of a GEMV, `infovec`): `gemv_dense_selector<OnTheRight,RowMajor,false>` = `dest(i) += alpha * (lhs.row(i).cwiseProduct(b^T)).sum()` with the LinearVectorized dynamic redux (two two-lane accumulators, the same as `squaredNorm` / `dot`) over the
  terms `(d_i V(k,i)) * b_k`; a left fold or a single two-lane accumulator both fail (measured, 4.5-5.0 k of 5.9 k mismatches against real Eigen). `diag * V^T` itself is the elementwise `d_i * V(j,i)` (zero eigenvalues give `+-0`).
* `S * x + b` with a dynamic S: the product evaluates into a temporary (GEMV, `0 + acc`), then the elementwise sum. Small products of the visual factors (`3x2 * 2x3`, `3x2 * 2x1`, `1x2 * 2x3`, the 1x2 . 2x1 inner product): the left fold `(a0 b0) + (a1 b1)` for every entry; the landmark
  elimination `block -= (h_i^T * inv) * h_j` is an OUTER product `dst.col(j) -= h_j[j] * (h_i^T * inv)` (the scalar is applied to the lhs first, never extracted), and `(h_i^T * inv) * vec` is the left-associated scalar chain `((h_i inv) vec)`.
  The `unordered_map` iteration order of a landmark's frame entries does not matter (every (i, j) block and every segment is touched once per landmark); the landmark order (`std::map<Track*, ..., compare>`) is ascending track id.
* The real map holds 13 or 14 frames at `marginalize` time (sliding_window_size 12, `while (frame_num > 12)`), so the priors are 12 x 15 = 180 (588 of 617 calls) or 13 x 15 = 195 (29): `RD_EIG_MAX` = 300.

Not covered / not exact yet
* Eigensolver `NoConvergence` (Eigen returns the unsorted result; the port does the same but it was never exercised), `inv_infomat` not finite (landmark skipped, `isfinite`), observations of the victim's landmarks in frames that are not in the map (skipped; the dump never contains them). `marginalize(index > 0)` is never called by the real code (`Map::marginalize_frame` asserts index == 0); the port is generic in the index and the M5 oracle exercises it through direct factor calls.
* The map layer that decides WHICH tracks / observations reach `marginalize` (valid, anchored in a keyframe victim, factor existence) is C++ in the dump patch only; M6 has to port that selection.

Why it scores what it scores (M5, from source reading + measurements): see PLAN.md section 6, "Marginalisation keeps linearisation points".
