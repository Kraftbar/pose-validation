# OKVIS2-X GNSS error terms + PoseManifold4d (ok_gps), 2026-10-07

Files: `okvis_port/c/ok_gps.{h,c}` (C99, BSD-3 + MPL-2.0 notice; uses ok_imu / ok_kin / ok_param / ok_err / ok_eigen and
`ok_gps_inverse3` of ok_gps_init.c), oracle `okvis_port/reference_tools/okvis_gps_test.cc`, build / run scripts and logs in
`runs/okvis2x_port/gps_leaf/` (`build_oracle.sh <plain|asan|mutN>`, `run.sh`, `run_all.sh`, `check_identical.py`, `oracle_seed{1..4}.txt`,
`sens_mut*.txt`, `asan_seed7.txt`, `identical.txt`). The Codex draft (385 + 54 lines) was rewritten: wrong full-Jacobian order,
`+0.0` left folds, duplicated IMU step code, 4-DoF plus-Jacobian read as row-major, refusal of valid segments.

API: `ok_gps_sync_{init,set_information,evaluate}`, `ok_gps_async_{init,init_sigma,free,set_information,evaluate,apply_preint}`
(struct fields = measurement(), information(), covariance(), error(), tk/tg = imu.t0/t1; the two X statics `useImuCovariance` /
`redoPropagationAlways` are per-object fields `use_imu_covariance` (1) / `redo_always` (0)), `ok_pose4_{plus,minus,plus_jacobian,minus_jacobian,right_multiply}`.
Jacobians are row-major 3x7 / 3x9 / 3x6; a minimal Jacobian is written only if its full Jacobian pointer is non-NULL (as upstream).

## ImuError: X vs OKVIS2
`ImuError.{cpp,hpp}` of X and of external/vio/okvis2 are byte-identical after the licence banner (same for Transformation.hpp). The GNSS
class has its own `redoPreintegration`: the loop body is the ImuError one (it uses `kinematics::sinc`, ImuError `ode::sinc`: same code), with
(a) the covariance recursion guarded by `useImuCovariance` and (b) no `symmSqrtU` / information tail. Neither touches the integrals, the
sub-Jacobians or P_delta, so ok_gps calls the validated `ok_imu_redo_preintegration` (the extra tail work is ignored). Upstream dereferences
`(it + 1)` of the last measurement (past the end); the value is only used when the coverage condition is violated, so the C uses zeros there.

## Result (tolerance 0, memcmp; seeds 1-4 x 20,000 scenarios, 142 MB compared per seed, 0 mismatches in every section)
See the table in `oracle_seed1.txt` (and 2-4). Per seed: manifold4 sizes / plus / minus / plusJacobian / minusJacobian /
RightMultiplyByPlusJacobian (rows 1..40) 20,000 cases each, static functions and the `ceres::Manifold` virtuals; sync ctor 20,000, residual and the four
Jacobians 40,000 evaluations (Evaluate and EvaluateWithMinimalJacobians, NULL-pointer patterns, sentinel-checked buffers); async ctor 20,000,
~49,850 evaluations (call sequences of 1-4 on one object: re-preintegration, stale linearisation with n >= 50, n = 49..51, both statics),
residual / error() / six Jacobians, applyPreInt before (~9,970) and after (~12,300) evaluation.
Inputs: random + EuRoC-like + gimbal / +-pi yaw / tiny / unnormalised / axis-aligned quaternions, T_GW translations to 1e6, IMU segments 0-1 s at
100 Hz / 200 Hz / 1 kHz with jitter and saturation, tg on / between stamps / 1 ns off / == tk, lever arm 0 and != 0, diagonal and full SPD information plus a few non-PD.
The X objects in the oracle are machine-code identical, function by function (49 + 106), to the objects of the deterministic reference build
(`check_identical.py`; requires compiling with the reference's own include set: the Ceres headers of X's bundled ceres-solver, not external/vio/deps/ceres, change the code generation).

Sensitivity (5,000 cases, seed 11, each variant `OK_GPS_MUTATE=n` of ok_gps.c must fail): row-0 residual as a tree 1,338; outer lift product with packets (left fold) 21,952;
Jprop block product swapped 13,885; position expression re-associated 3,985; right-multiply as a tree 990; cov product as a tree 6,719; error `(m-gv)-p` 11,031; redo
rule `n < 51` 638. NOT detected: `P = T Pd T^T` replaced by a plain left fold (0): the IMU term is absorbed by the GNSS covariance / structure of T, so that association is not
observable in the outputs (the GEBP model of ok_imu.c is kept).

## Eigen 3.4.0 facts used
Known: `ok_lazy_a` (rows 0-1 left fold, odd last row halving tree) for fixed-size lazy products <= 8 (3x3*3x6, 3x6*6x6, 3x6*6x9, 3x6*6x3, 3x6*6x7 columns), `ok_m3_mul/mulv`, `(-A)*B == -(A*B)`, LLT n=3, Matrix3d::inverse (ok_gps_inverse3), Quaternion / Transformation / oplusJacobian / minusJacobian of ok_kin / ok_param, 15x15 `P = T*Pd*T^T` model of ok_imu.c.
NEW (measured here):
- `J = sqrtI * J_minimal * J_lift` assigned into a row-major `Map<3x7>`: the inner product is the usual column-major temporary, but the outer product (column-major lhs, row-major dst) is evaluated scalar: redux tree `(p0+(p1+p2))+(p3+(p4+p5))` in ALL rows (`ok_lazy_tree`), not the packet rule. The minimal Jacobian `sqrtI * J_minimal` keeps the packet rule. (Probably also true of the same statement shape elsewhere with a column-major lhs.)
- `PoseManifold4d::minusJacobian`: `(Jq_pinv*Qplus).bottomRightCorner<1,4>()` equals row 5 of the PoseManifold 6x7 lift (tree row); `plusJacobian` = columns 0-2 and 5 of `oplusJacobian`.
- `ceres::Manifold::RightMultiplyByPlusJacobian` default (dynamic row-major Eigen): rows <= 8 every entry a plain left fold starting at the first product (no `0.0 +`), rows >= 9 GEBP on the transposed problem = `ok_gemm(4, rows, 7, PlusJacobian, A, out)`.
- Unrolled small products do not depend on temp alignment (no runtime peeling), so no parity parameter is needed for these shapes.

## Open items
- tools/check_okvis_port.py compiles `okvis_*_test.cc` with the OKVIS2 (not X) include set; the oracle therefore prints SKIP there. Build / run with `runs/okvis2x_port/gps_leaf/run_all.sh`
  (or add an include-set directive for X to the runner).
- ok_gps.c depends on ok_gps_init.c (inverse3) and on ok_imu_redo_preintegration (computes the unused symmSqrtU tail: ~one 15x15 eigen-solve per re-preintegration).
- Not covered: tg < tk, empty deque, measurement sets violating front <= tk <= tg <= back (upstream UB / asserts).
- Integration (ViGraph/ViSlamBackend) still has to supply `gpsParameters.r_SA`, the deque copy per factor and the Ceres loss / manifold wiring.
