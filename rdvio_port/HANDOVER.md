# rdvio_port handover

Goal: dependency-free C99 port of RD-VIO (Jianxff/rd_vio, Apache-2.0), bit-exact against a deterministic reference, as references and
reusable pieces. Method and rules: `okvis_port/HANDOVER.md`, `stella_port/HANDOVER.md`. Plan, module table, OpenCV list, "why it scores what it scores":
`rdvio_port/PLAN.md`. Licences: `docs/rdvio_license_audit.md`. Nothing committed.

## State (2026-10-04)

* Phase 0 done: licence audit, PLAN.
* Reference done: `tools/build_rdvio_reference.py` (patches 0001-0003), `tools/run_rdvio_reference.py`. MH_01_easy: 4 reference runs byte-identical
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

## Next module: M2 (reprojection/rotation factors + geometry) or M4 (SPARSE_SCHUR solver); suggestion M2 first
(small, same style: `CeresReprojectionErrorFactor`/`PriorFactor`/`RotationPriorFactor` Evaluate with dump patch 0004; then stereo/essential/homography/Wahba
from stella's `sv_eigen_svd`/`sv_eigen_eigensolver`). M4 needs a Ceres SPARSE_SCHUR snapshot dump like okvis patch 0008 and a block-structure measurement.

## Gotchas
* Score with `external/gnss/venv/bin/python` (numpy); the system python has none.
* Do not use the stock `external/vio3/rd_vio/build` for comparisons of bits: it is fast-math, Ceres 1.14, Eigen 3.3.7.
* `run_rdvio_reference.py --dump` needs the build to contain patch 0003 (it does by default).
* Dump sizes: `integ=1` is ~170 MB per MH_01 run; `pie=4` ~290 MB. Delete `runs/rdvio_port/m1*/…/dump` when done (regenerable in 2 min).
