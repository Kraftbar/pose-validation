# OKVIS2-X GNSS init helpers (ok_gps_init), 2026-10-06

Files: `okvis_port/c/ok_gps_init.{h,c}` (C99; BSD-3 + MPL-2.0 notice; libm + `ok_eigen` / `ok_kin` only),
oracle `okvis_port/reference_tools/okvis_gps_init_test.cc` (picked up automatically by `tools/check_okvis_port.py --eigen-tests`
through its `OK_PORT_TEST_C` header line; no change to the runner), logs in `runs/okvis2x_port/gps_init/`.

API (flat arrays: points `[x y z]*n`, covariances 3x3 column-major):
- `ok_gps_umeyama` = `umeyamaTransform` (n < 3 -> identity, return 1), result an `ok_tf` (C = block verbatim, q from the matrix).
- `ok_gps_estimate_rigid_ransac` = `estimateRigidRansac` (mt19937(42) + `ok_uniform_int`, optional inlier index output).
- `ok_gps_yaw_hessian`, `ok_gps_init_core` = the numeric core of `checkForGpsInit` AFTER the points were gathered
  (`robust` selects RANSAC(20, 20, 4.0, 0.7) + `T_GW.set(...)` vs. Umeyama; returns 1 on ratio < 0.25; else the yaw sigma in
  degrees; the Ceres refinement and the `yawErrorThreshold` comparison stay with the caller).
- Building blocks exposed: `ok_mt19937_*`, `ok_uniform_int`, `ok_gps_inverse3`, `ok_gps_inverse4`.

Result (tolerance 0, memcmp, seeds 1-4 x 20,000 scenarios, each scenario: umeyama + 3 RANSAC parameter sets + both core modes;
ASan/UBSan seed 7 clean): 0 mismatches in every section, 169.7 MB compared; see `oracle_seeds1-4.txt`. Sensitivity (wrong
reference variants must mismatch): seed 43, `rng() % n`, `*= 1/n` centroids, pivoted-LU inverses, even/odd-chain H, and
removing the duplicate-index rejection in the C port (checked by hand) all fail.

Eigen 3.4.0 / libstdc++ (GCC 13.3) facts used
- `mt19937` = standard recurrence; `uniform_int_distribution<int>(a, b)` for a 32-bit URNG with range < 2^32 - 1 is Lemire's
  nearly-divisionless method (`product = uint64(g()) * range; low = uint32(product); if (low < range) { threshold = -range % range;
  redraw while low < threshold }; result = product >> 32`), NOT the older modulo/rejection scheme (rd_rand.c models the other,
  fallback branch used for minstd).
- `Matrix3d H = MatrixXd(3,n) * MatrixXd(n,3)^T`: `product_type_selector<Small,Small,Large>` = GemmProduct; n + 6 < 20 takes the
  coefficient-based lazy product, otherwise GEBP. For 3 x 3 results both are one left fold from k = 0 per entry (GEBP: cols < 4 means
  no even/odd chains), so `ok_gemm` and a plain loop agree bitwise.
- `Matrix3d::inverse()`: `det = (c0*m00 + c1*m10) + c2*m20` (cwiseProduct(col0).sum() over a Block: Packet2d pair then tail),
  NOT the tree `a + (b + c)` that rd_inverse3_t uses for its transposed operand; entries `cofactor * invdet`.
- `Matrix4d::inverse()` = the SSE2 Packet2d kernel already modelled by rd_m4_inverse (reused as is).
- `Matrix3d * Vector3d (+ vector)` and `(a - b).norm()` follow ok_m3_mulv / ok_v3_norm unchanged; `v /= size_t` is a true division.
- The yaw Hessian `Ei^T * cov^-1 * Ei` is association-insensitive: Ei has one nonzero per column except column 3 (two), so any
  order of the 3-term sums gives the same bits (verified: swapped trees mismatch 0); only the inverse and the accumulation order matter.
- `RigidResult()` (too few points) is value-initialised (zeros); a `best` that never improves leaves R, t UNINITIALISED in C++
  (the C port returns zeros; the caller rejects ratio < 0.25 so it is never read; the oracle skips that comparison).

Open items: the Ceres 4-DoF refinement (`Align4DoF_Ceres`), the gathering of propagated points (`applyPreInt`), and the
`T_GW`-dependent consumers are other leaves. The oracle uses the okvis2 `Transformation` headers (identical to X's apart from the
license banner).
