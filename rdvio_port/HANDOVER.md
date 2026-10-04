# rdvio_port handover

Goal: dependency-free C99 port of RD-VIO (Jianxff/rd_vio, Apache-2.0), bit-exact against a deterministic reference, as references and
reusable pieces. Method and rules: `okvis_port/HANDOVER.md`, `stella_port/HANDOVER.md`. Plan, module table, OpenCV list, "why it scores what it scores":
`rdvio_port/PLAN.md`. Licences: `docs/rdvio_license_audit.md`. Nothing committed.

## State (2026-10-04)

* Phase 0 done: licence audit, PLAN.
* Reference done: `tools/build_rdvio_reference.py` (patches 0001-0005), `tools/run_rdvio_reference.py`. MH_01_easy: 4 reference runs byte-identical
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

## Next module: M4 (SPARSE_SCHUR solver; needs a Ceres snapshot dump like okvis patch 0008) or M7 (OpenCV CLAHE / LK / GFTT, largest bit-exactness risk). M4 first, it unlocks the optimisation loop.

## Gotchas
* Score with `external/gnss/venv/bin/python` (numpy); the system python has none.
* Do not use the stock `external/vio3/rd_vio/build` for comparisons of bits: it is fast-math, Ceres 1.14, Eigen 3.3.7.
* `run_rdvio_reference.py --dump` needs the build to contain patch 0003 (it does by default).
* Dump sizes: `integ=1` is ~170 MB per MH_01 run; `pie=4` ~290 MB. Delete `runs/rdvio_port/m1*/…/dump` when done (regenerable in 2 min).
