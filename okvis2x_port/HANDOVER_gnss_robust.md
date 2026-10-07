# OKVIS2-X GNSS stage (b): `robust_gps_init: true` in the C app, 2026-10-07

Result: with `OKVIS_PORT_OKVIS2X=1` and `robust_gps_init: true` the C app `okvis_c_euroc` reproduces the OKVIS2-X deterministic reference
byte for byte (`causal.csv` whole file, `final.csv` columns 1-17, `global_final.csv`) on three scenarios, and stages (a) / off / default
mode are unchanged (numbers below). Run it: `python3 tools/okvis2x_check_gnss.py --stage-b` (or `tools/check_okvis_port.py --gnss-e2e`).

## 1. Why the robust init never fired on MH_01 (X reference `gps_b_mono1`)
`estimateRigidRansac(gps, world, 20, 20, 4.0, 0.7)` returns `RigidResult()` (value-initialised, `inlier_ratio = 0`) when
`gpsPoints.size() < 2 * n_points = 40`, and `checkForGpsInit` then rejects (`inlier_ratio < 0.25`, a DLOG). The points come from
`gpsStates_` of the REALTIME graph, which only holds the states still in the sliding window (the others are eliminated and erased from
`gpsStates_`): with 5 Hz fixes that is at most 27 points (2..27 over the whole run, 759 `checkForGpsInit` calls), never 40. The yaw gate
was never reached. Evidence: the C app (the C port of the same code, instrumented with the `CI` log line of `OKVIS_PORT_GNSS_LOG`) run on
`clean_gps_a` data with `robust_gps_init: true` reproduces `gps_b_mono1` byte for byte (causal `1e97d2eb4eae`, global `3e495bd69d64`, final
cols 1-17) and logs `RANSACREJECT` with npts <= 27 for every attempt. (No instrumented copy of the X build was needed: the C code is the same
algorithm, validated byte for byte; the DLOGs of X stay compiled out.)
Consequences for data design: the window must hold >= 40 fixes (>= ~14 Hz for the ~2.8 s window; generate 20 Hz), and the yaw sigma over the
window (not the whole history) must drop below 1 degree (needs a moving platform and sigma_h of 1-2 cm).

## 2. New GNSS datasets (tools/okvis2x_make_euroc_gps.py, X reference runs, each run twice, identical hashes)
Data roots `runs/okvis2x_port/data_r{1,2}/MH_01_easy/mav0` (symlinks to the sequence + own `gps0/data.csv`), config
`okvis2x_port/reference/configs/okvis2x_mono_euroc_gps_robusttrue_deterministic.yaml` (unchanged).
```
python3 tools/okvis2x_make_euroc_gps.py runs/okvis2x_port/data/MH_01_easy --out runs/okvis2x_port/data_r2/MH_01_easy/mav0 --rate 20 --noise 0.01,0.02 --blackout 60,30 --seed 5   # r2
python3 tools/okvis2x_make_euroc_gps.py ...                                                      --out .../data_r1/...               --rate 20 --noise 0.02,0.04 --blackout 70,12 --seed 4   # r1
python3 tools/okvis2x_run_reference.py MH_01_easy --tag gps_b_r2_1 --config <robusttrue yaml> --data-root runs/okvis2x_port/data_r2    # and _2
```
| scenario | fixes | X run (2 runs identical) | what happens |
|---|---|---|---|
| r2: 20 Hz, 1/2 cm, seed 5, 30 s blackout at 60 s | 3039 | `gps_b_r2_{1,2}`: causal `078b9fdd7c66`, final `1613128333cc`, global `4facea3e64a4` (wall 244 s) | Idle -> Initialising -> Initialised (state 392, yaw sigma 0.51 deg, 3 `Align4DoF_Ceres` calls, 2 iterations each), dropout (state 1026), ReInitialising (second branch of `needsGpsReInit` "reinit-fixed" exercised, 20+ times), full re-alignment (X log: "GPS fully re-Initialised", states 1026..3253). Global trajectory vs GT antenna in the GT frame (no alignment): RMS 2.04 cm, max 9.9 cm |
| r1: 20 Hz, 2/4 cm, seed 4, 12 s blackout at 70 s | 3399 | `gps_b_r1_{1,2}`: causal `8811174e282b`, final `b0f47380264e`, global `5d1f47c3c982` (wall 242 s) | yaw sigma 1.01-1.02 deg for the first 900 states (RANSAC passes, the 1 degree gate fails), init at state 900; global RMS 2.23 cm |
| MH_01 5 Hz (existing) | 760 | `gps_b_mono1`: causal `1e97d2eb4eae`, global `3e495bd69d64` | never initialises (section 1) |

## 3. What was ported (all new code oracle-validated at tolerance 0, then end to end)
| upstream | C | validation |
|---|---|---|
| `FourDoFResidual` as `AutoDiffCostFunction<FourDoFResidual,3,7>` executes it (Jet<double,7>: `Quaternion<Jet>::_transformVector` = `uv = vec x v; uv += uv; v + w*uv + vec x uv`, Jet `+ - *` as jet.h, no FMA; double path for cost-only evaluations) | `ok_align4.{h,c}`: `ok_align4_residual` | oracle `okvis_port/reference_tools/okvis_align4_test.cc` section 1: 0 / 486,000 (3 seeds x 6000 cases x 27 values) |
| `ceres::internal::EigenDenseQR` = `HouseholderQR<MatrixXd>::solve` (unblocked reflectors, redux of the cwiseAbs2 / cwiseProduct expressions with alignedStart = 0, dot path for one remaining column, RowMajor GEMV kernel otherwise, `Q^T c` by H_0..H_{r-1} on the vector, upper `solveInPlace`) | `ok_align4.c`: `ok_eigen_hqr_solve` | section 2: 0 / 37,230 (rows 4..700, cols 1..8, special values) |
| `DENSE_QR` linear solver (`DenseQRSolver`: `[J; diag(D)]`, `[b; 0]`), `DenseSparseMatrix` (row-major) `SquaredColumnNorm` (naive row loop) / `RightMultiplyAndAccumulate` (row-major GEMV) for ONE parameter block, LM strategy (already in `ok_solve.c`), new residual type `OK_SV_T_ALIGN4` | `ok_solve.{h,c}`, `ok_solve_linear.c` (documented edits; Dogleg / DENSE_SCHUR / SPARSE_NORMAL_CHOLESKY paths untouched) | section 3: the whole `ceres::Solve` of `Align4DoF_Ceres` (PoseManifold4d block, N Cauchy(3) blocks, 100 iterations) on 9000 random problems (N 2..220, outliers, yaw-only and general quaternions): termination, every IterationSummary (cost, cost_change, gradient_max_norm, gradient_norm, step_norm, relative_decrease, trust_region_radius, step flag) and the final T_GW: 0 / 27,027 + 0 / 167,024 |
| `Align4DoF_Ceres` driver, call in `checkForGpsInit` after the yaw gate (`T_GW = T_GW_refined`) | `ok_align4dof_ceres`, `ok_vggps.c` (`check_for_gps_init`; log line `CS` = Ceres brief report, `CR` = result) | the three reports of the X r2 log (iterations 2; costs 1.316419e-02 -> 1.254018e-02, 1.309765e-02 -> 1.239447e-02, 1.281425e-02 -> 1.228209e-02) are reproduced digit for digit |
| `ViGraph::checkValidGpsMeasurements` (:1128) - NOT inside `checkForGpsInit`: `ViSlamBackend::addGpsMeasurementsOnAllGraphs` calls it only for `robustGpsInit`, the accepted fixes come back in REVERSE order | `ok_vggps.c` `ok_vgps_check_valid_measurements`, `ok_vslam.c` `ok_vsb_add_gps_measurements` | end to end (this was the cause of the first r2 divergence at causal row 660: in `Initialised` every fix is tested against the current estimate, |error| > 3 sigma per axis rejects it) |
| `needsPosGpsAlignment` returns false with robust init (already in stage a), window of the last 100 states (already), RANSAC (already, `ok_gps_init.c`) | - | - |
`ok_sys_new` no longer rejects robust configs (still rejects non-cartesian data types).
Sensitivity (`runs/okvis2x_port/align4/build_oracle.sh mutN`, `-DOK_ALIGN4_MUTATE=N`): FMA in the Jet product (855 mismatches), `v + (w*uv + cross)` (8189), single-accumulator redux (8837),
GEMV instead of dot for one remaining column (499), Q^T in reverse order (15947), `tau*(e*t)` instead of `(tau*e)*t` (3684) are all detected; two mutants survive and are not observable
in any output (`0.0 + 1.0 * dot` for signed zeros, skipping the `rhs[i] != 0` test of the triangular solver).
Oracle build/run by hand: `runs/okvis2x_port/align4/build_oracle.sh [plain|asan|mutN]` then `bin/okvis_align4_test_<variant> [cases [seeds]]` (links the X reference's Ceres); under
`tools/check_okvis_port.py --eigen-tests` it is picked up through its `OK_PORT_TEST_C` line (Ceres + Eigen only; the X PoseManifold4d is copied verbatim into the test).

## 4. Numbers (C app vs X reference, MH_01 mono, 3,6xx frames, 1 thread, C wall 5-5.5 min)
Final code, `tools/okvis2x_check_gnss.py --cases ...` (all PASS = causal.csv byte for byte, final.csv cols 1-17, global_final.csv byte for byte; 4 in parallel, so C wall 5.0-5.6 min):
| case | mode | causal | global |
|---|---|---|---|
| `gps_b_r2_1` (data_r2) | robust true | `078b9fdd7c66` | `4facea3e64a4` |
| `gps_b_r1_1` (data_r1) | robust true | `8811174e282b` | `5d1f47c3c982` |
| `gps_b_mono1` (5 Hz) | robust true | `1e97d2eb4eae` | `3e495bd69d64` |
| `clean_gps_a` / `gnss_v2` / `gnss_v3` | robust false (stage a) | `deabab007e3d` / `86d2e74bbac7` / `bf4039465fda` | `0f711351270e` / `49e7c43e0729` / `8ed3f97f6d44` |
| `clean_off` | GNSS off, OKVIS2X=1 | `144d71718396` | - |
Default mode (OKVIS2 canonical, final `dfe3b58e6a33`, causal `cc29a746ea6f`): `check_ok_system` m8 and s8 end to end PASS inside the full suite
`python3 tools/check_okvis_port.py --tag m8,s8 --native-solve 1 --eigen-tests --data external/vio/data/okvis_brisk_tmp --gnss-e2e` (all 16 harness rows 0 mismatches, e.g. check_ok_system m8
0 / 155,335,661, s8 0 / 300,332,809; every eigen/okvis oracle PASS incl. okvis_align4_test 0 / 717,281). Caveat: that run aborted at the oracle link step of okvis_align4_test
(libceres of the deps needs LAPACK; fixed in `tools/check_okvis_port.py` by linking OpenBLAS), so the harness rows come from that run (log `runs/okvis2x_port/gnss_int/logs/suite_b.log`)
and the oracle list from a second invocation with `--harness check_ok_problem --eigen-tests` (`suite_c.log`); the GNSS e2e cases above were run separately, in parallel
(`--stage-b` in `--gnss-e2e` runs them sequentially, ~35 min).

## 5. Not covered / next
- geodetic / geodetic-leica data in `checkValidGpsMeasurements` (`LocalCartesian::Forward`), stereo + GNSS, `doFinalBa` GNSS unfreeze, multi-session: unchanged from stage (a).
- The rejection branch `RANSACREJECT` of the C code is exercised (gps_b_mono1: all 759 attempts); the X-side DLOG text is still not visible in X logs (compiled out).
- The 3-sigma rejection branch of `checkValidGpsMeasurements` is exercised: 52 rejections (`CV` log lines) in the first 1100 frames of r2 alone (first one at state 659, where the first draft without the check diverged). A scenario with gross outliers (the generator has no outlier option) and the `Off/Idle/Initialising` sigma rejection (`reject-inaccurate`, needs fixes with sigma_h > 6 m) are not exercised.
