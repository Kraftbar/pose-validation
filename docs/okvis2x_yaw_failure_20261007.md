# OKVIS2-X mono + GNSS on INSANE o1: the 48 m geo error is a missing GNSS initialisation, not a yaw-gate failure (2026-10-07)

Scope: `docs/drone_benchmark_20261002.md` section 3.3 / section 4 (bullet "o1 OKVIS2-X geo error 48 m with SE3 error 0.5 m"). Reproduced with the deterministic OKVIS2-X
reference and the bit-exact C port (`okvis_port/c`, `OKVIS_PORT_OKVIS2X=1`), diagnosed with the C GNSS event log, fixed with opt-in switches.
All numbers below are from runs in `runs/okvis2x_port/drone/` (not committed; EuRoC-/INSANE-derived).

## 1. Setup

- Data: `runs/okvis2x_port/drone/{o1,m14}/` regenerated with `tools/drone_harness/insane_to_euroc.py` (o1 `--max-s 100`, m14 full 181 s; the PNGs had been deleted from
  `external/drone/ins_*`). Regenerated `imu0/data.csv` and `gps0/data.csv` of o1 are byte-identical to `external/drone/ins_o1/mav0`.
- GNSS conversion (already done by that script, no new conversion needed): the PX4 receiver fixes `px4_gps.csv` (WGS84 lat, lon, h) -> ECEF (a = 6378137 m,
  f = 1/298.257223563) -> local ENU rotation at the dataset reference `ref_lla` from the INSANE README (o1: 46.606867, 14.279121, 484.017; m14: 30.599929, 34.867308, 526.594);
  `gps0/data.csv` = `t_ns, x=E, y=N, z=U [m], hErr1=hErr2=sqrt(var_xy), vErr=sqrt(var_z)`. The same file and `data_type: cartesian` go to the X reference and to the C app, so
  the failure is not an artefact of a conversion (it reproduces: section 2).
- Deterministic configs: `okvis2x_port/reference/configs/okvis2x_mono_drone_{o1,m14}_deterministic.yaml` = `external/drone/ins_*_cfg/okvis2x_mono_gnss.yaml` with
  `parallelise_detection: false`, `realtime_num_threads: 1`, `full_graph_num_threads: 1` and `do_final_ba: false` (the C port does not cover the final-BA GNSS unfreeze, see
  `okvis2x_port/HANDOVER_gnss_robust.md` section 5). `gps_parameters`: `robust_gps_init: true`, `yaw_error_threshold: 1.0`, `r_SA: 0 0 0`.
- Tools: `tools/okvis2x_drone.py` (run-c / run-x / cmp / score / run-euroc / run-default), `tools/okvis2x_drone_diag.py` (event-log tables). Scoring = the drone benchmark's
  `tools/gnss_harness/gnss_eval.py` (`score(...)`, `benchmark.umeyama_alignment`), RTK reference `gt_enu.tum`, lever `r_RTK`; SE3 = rigid-aligned ATE of `final.csv`,
  geo = RMS of `global_final.csv` against the reference WITHOUT alignment. Python: `external/gnss/venv/bin/python`.

## 2. Reproduction (C == X byte for byte)

| seq | X reference run | C app | causal | final cols 1-17 | global_final |
|---|---|---|---|---|---|
| o1 | `reference_runs/o1/x_o1` | `drone/out/o1/c_o1` | identical (`8019f570b278`) | identical | identical (`11de5c1b4a31`) |
| m14 | `reference_runs/m14/x_m14` | `drone/out/m14/c_m14` | identical (`d44dd50fa590`) | identical | identical (`a0520ef203d6`) |

| (unmodified) | SE3 [m] | Sim3 | geo RMS [m] | geo median | geo max |
|---|---|---|---|---|---|
| o1 deterministic X = C | 0.724 | 0.200 | **47.52** | 0.61 | 130.0 |
| o1 stock X (drone doc, threaded, final BA) | 0.47 / 0.68 | 0.31 | 47.9 / 47.7 | 0.60 | 130.5 |
| m14 deterministic X = C | 1.343 | 1.151 | 4.50 | 3.92 | 7.64 |
| m14 stock X (drone doc) | 1.39 | | 3.98 | | |

The failure reproduces (geo 47.5 vs 47.9 for the stock run; the SE3 difference 0.72 vs 0.47 is final BA on/off and threading). m14 works.

## 3. Diagnosis (C GNSS event log, `OKVIS_PORT_GNSS_LOG`; `OKVIS_PORT_FIX_DIAG=1` adds `CD` lines: window point spread and the yaw sigma the gate would compute)

**The stationary phase is not what fails. The GNSS-VIO state machine never initialises T_GW on o1 at all.** Evidence (unmodified C, `drone/out/o1/c_o1/gnss.log`):

1. State machine: `ST 0 0->1` (Idle) at the first fix and nothing after it: no `Initialising`, no `Initialised`, no `TG ... set`, no freeze, no alignment event (`AL`/`PA`/`RI`) in the
   whole 100 s. T_GW stays the identity, so `global_final.csv` is the VIO frame (rows are the VIO positions, `p_GA_G == p_WS_W` for `r_SA = 0`; the stock log has only
   "First measurements added" and, from the final BA, "Unfreezing GPS Extrinsics").
2. Why: `checkForGpsInit` runs on the fixes of the states still in the REALTIME sliding window (`gpsStates_`; entries are erased when a state is eliminated). In the robust mode
   `estimateRigidRansac(..., 20 iterations, n_points = 20, 4.0 m, 0.7)` returns "no result" for fewer than `2 * n_points = 40` points and `checkForGpsInit` rejects
   (`inlier_ratio 0 < 0.25`) before the yaw gate is ever evaluated. Counts on o1: 481 checks = 263 `EARLY` (fewer than 2 states with a fix in the window, all of t < 53 s: while
   the drone sits still the window carries one GNSS state; this is what the `n=1` lines show, I did not trace the elimination code) + 218 `RANSACREJECT`; the largest window ever
   seen was **28 points** (never 40) for 495 fixes at 4.95 Hz. On m14 (also 5 Hz fixes) the window reaches 69 points, so RANSAC runs there; that is the only difference between the
   two sequences (m14: first RANSAC success at t = 52 s, `Initialising` 71.0 s, `Initialised` 83.2 s after 95.6 m of flight, yaw sigma 0.944 deg with 69 points).
   This is the same mechanism as `HANDOVER_gnss_robust.md` section 1 (MH_01 5 Hz, `gps_b_mono1`: all 759 attempts rejected, `global` is 5.94 m off the GT frame).
3. The yaw/observability gate is not the culprit and is computed correctly for degenerate geometry. Observe-only `CD` lines (Umeyama over all window points, no RANSAC
   requirement, same Hessian = sum Ei^T Cov^-1 Ei over (dx,dy,dz,yaw), `sqrt(P(3,3))` in degrees; threshold 5 deg for Idle -> Initialising (hard-coded) and 1 deg for
   Initialising -> Initialised, `yaw_error_threshold`):

   | t [s] | window pts | rms spread VIO pts [m] | path [m] | yaw sigma if RANSAC were not required [deg] |
   |---|---|---|---|---|
   | 53.2 (first check with 3 pts) | 3 | 0.002 | 0.00 | 69996 |
   | stationary, t < 54 (5 checks with >= 3 pts) | <= 7 | <= 0.043 | | >= 1780 |
   | 60.2 | 20 | 1.06 | 5.4 | 43.3 |
   | 70.2 | 22 | 4.42 | 20.6 | 10.4 |
   | 74.6 (first < 5 deg) | 25 | 9.2 | | < 5 |
   | 80.2 | 26 | 12.3 | 51 | 3.6 |
   | 90.2 | 27 | 15.1 | 76 | 3.0 |
   | 99 (end) | 28 | 20.0 | 95 | 2.26 |

   So the stationary start gives a huge yaw sigma (the information on yaw is sum |p_xy - mean|^2 / sigma^2 with sigma^2 = 12.6 m^2 from the PX4 hErr 3.5 m; at 0.04 m spread it is
   ~0), exactly as it should; it would be rejected. The 5 deg gate is first met at t = 74.6 s (38.2 m travelled) and the 1 deg gate is NEVER met with only the window points
   (2.26 deg at the end). Even without the RANSAC floor the strict 1 deg gate would keep o1 in `Initialising` (see the fixes).
4. Consequence in numbers: the VIO world frame is rotated 120.8 deg (tilt 0.8 deg) against ENU (Umeyama of the final VIO trajectory onto the RTK reference). With T_GW = identity
   the global error at horizontal distance d from the origin is 2 d sin(60.4 deg) = 1.74 d: the stationary start (d ~ 0) gives the 0.6 m "pinned near the origin" (geo error per
   time bin [0,55) s 0.60 m), then the flight (72.7 m from the origin at the end) gives [55,60) 3.35, [60,70) 14.0, [70,80) 51.5, [80,90) 77.6, [90,100) 111 m, RMS 47.5 m, max 130 m.
   The "yaw pinned then rotates away" of the drone doc is the identity frame against a moving drone, not a re-estimated T_GW.

## 4. Fixes (opt-in, `OKVIS_PORT_FIX_<NAME>=1`, default off = bit-exact; marked "OUR modification" in `okvis_port/c/ok_gps_init.{c,h}`, `ok_vggps.c`)

| switch | what | note |
|---|---|---|
| `FIX_DIAG` | observe-only `CD` log lines (window spread, yaw sigma without the RANSAC floor) | byte-identical outputs (checked: same hashes with it on) |
| `FIX_RANSAC_SMALL` | in the first Idle -> Initialising check of the robust init only, use `n_points = n/2` (6 <= n < 40) instead of rejecting | `Initialising -> Initialised` (1 deg gate, Align4DoF, freeze) and the re-initialisation keep the 40-point rule |
| `FIX_HISTORY` (+ `FIX_HISTORY_N`, default 200) | keep every point seen by a check (keyed by the fix time; world position = what the state had at its last check) and run RANSAC / Hessian / Align4DoF on the last N points instead of the window | initial init only |

Not needed: a "minimum travelled distance" or a corrected yaw-sigma gate (item 3: the gate already rejects the stationary phase and is the only thing protecting the 1 deg init).

### 4.1 INSANE (deterministic C == X where unmodified; geo = no alignment; outputs `drone/out/{o1,m14}/c_*`)

| variant | o1 SE3 | o1 geo RMS (median / max) | o1 GNSS state | m14 SE3 | m14 geo RMS (median / max) |
|---|---|---|---|---|---|
| unmodified (X = C) | 0.724 | **47.52** (0.61 / 130.0) | Idle for 100 s | 1.343 | 4.50 (3.92 / 7.64) |
| `RANSAC_SMALL` | **0.206** | **8.57** (8.80 / 9.02) | Initialising at 74.6 s (38 m, 25 pts, yaw sigma < 5 deg); never Initialised (min 1.79 deg) | 1.343 (byte-identical to unmodified) | 4.50 (identical) |
| `HISTORY` (N = 200) | 0.445 | 8.35 (8.50 / 8.76) | Initialising 70.8 s (83 pts), Initialised 86.6 s (161 pts, 0.998 deg, 80 m) | 1.517 | 6.52 (4.16 / 13.8) **worse** |
| `HISTORY` N = 100 | 0.400 | 8.46 (8.64 / 8.88) | Initialising only | 1.423 | 5.41 (3.76 / 10.9) worse |
| reference numbers (drone doc) | loose smoother 1.47 / 10.45; PX4 fixes alone 2.20 / 10.7 | | | loose 1.46 / 4.15 | PX4 alone 1.37 / 4.08 |

o1: both fixes land at the receiver bias floor (the PX4 fixes are 10.7 m off in absolute terms; the loose smoother 10.45; `RANSAC_SMALL` 8.57 and a better SE3 than the unmodified
run and than the loose smoother). Geo error per time bin with `RANSAC_SMALL`: 8.8 / 8.9 / 8.5 / 8.0 / 8.1 / 8.4 m (flat; T_GW applies to the whole trajectory once estimated).
m14 `HISTORY` regression mechanism: the earlier, history-based T_GW (init 74.2 s vs 83.2 s, yaw 147.6 vs 149.7 deg) makes 55 fixes fail the 3-sigma validity test of
`checkValidGpsMeasurements` (errors 6-8 m against 3 sigma = 5.5 m for hErr 1.83 m), then a "dropout" (`RE 0 dropout=1219`, 89 s) and a re-initialisation that never completes (re-init keeps the
40-point rule); N = 100 gives 8 rejections and a smaller regression. Rejected, kept only as an opt-in experiment.

### 4.2 EuRoC GNSS cases (MH_01 mono + simulated GNSS; global_final vs GT antenna without alignment, RMS [m]; `tools/okvis2x_eval.py --antenna`)

| case | unmodified | `RANSAC_SMALL` | `HISTORY` |
|---|---|---|---|
| `gps_b_mono1` (5 Hz, window < 40: never initialises, same as o1) | 5.9396 | **0.0274** | 0.0652 |
| `gps_b_r1_1` (20 Hz, 2/4 cm) | 0.0223 | 0.0208 | 0.0253 |
| `gps_b_r2_1` (20 Hz, 1/2 cm, 30 s blackout, re-init) | 0.0204 | 0.0248 | 0.0451 |
| `clean_gps_a` / `gnss_v2` / `gnss_v3` (robust_gps_init false) | identical hashes (`deabab007e3d` / `0f711351270e` for clean_gps_a, with both switches on) | | |

`RANSAC_SMALL` is the one to use: o1 47.5 -> 8.6 m, mono1 5.94 -> 0.027 m, m14 and clean_gps_a byte-identical, r1 -0.001 m, r2 +0.004 m (all within the repo's 0.01 m neutral band).
Earlier revisions of the same switch, recorded because they regress the 20 Hz cases (hashes in `drone/logs`): reduced RANSAC in every check including Initialising -> Initialised and
re-init (r1 0.0171, r2 **0.0493**: a T_GW frozen from a 12-point fit); allowed after 40 checks with 20..39 points (r2 0.0282); allowed after 3 s stuck below 40 points (r2 0.0716, because the
20 Hz window also sits at 20..39 points for seconds); minimum 20 points (r2 0.0355). Restricting to the first Idle -> Initialising check lets the upstream 40-point rule do the final
init / freeze where the window allows it.

## 5. Regression gates (final code, all switches off)

- `tools/okvis2x_check_gnss.py` stage-b set with the FINAL code, all switches off (three invocations in parallel instead of one sequential `--stage-b`, same cases): `clean_gps_a:gps` PASS,
  `clean_off:off` PASS, `gps_b_mono1:gpsb` PASS (`1e97d2eb4eae` / `3e495bd69d64`), `gps_b_r2_1:gpsb:data_r2` PASS (`078b9fdd7c66` / `4facea3e64a4`), `gps_b_r1_1:gpsb:data_r1` PASS
  (`8811174e282b` / `5d1f47c3c982`); logs `drone/logs/final_gate{A,B}.log`. (The same set had also PASSed with an earlier revision of the switches in the tree.)
- Default-mode OKVIS2 canonical MH_01 (`okvis_c_euroc`, no `OKVIS_PORT_OKVIS2X`, `okvis_mono_euroc_deterministic.yaml`, final code): final `dfe3b58e6a33`, causal `cc29a746ea6f` (matches).
- The unmodified runs with logging (`run-euroc ... off`) reproduce the reference hashes: mono1 `1e97d2eb4eae`/`3e495bd69d64`, r2 `078b9fdd7c66`/`4facea3e64a4`, r1 `8811174e282b`/`5d1f47c3c982`.

## 6. Caveats

- `do_final_ba: false` (C port limitation); the stock run had final BA on. The unmodified geo error is the same (47.5 vs 47.9).
- One sequence per failure (o1), single deterministic run each; the 8.57 m is a receiver-bias floor, not an accuracy claim about the system. Nothing here touches the SLAM benchmark
  (`benchmark_native.py`): no files of the pose_slam implementations changed.
- The cause of the small window on o1 (<= 28 points) versus m14 (69) is a front-end property (keyframe / state elimination); I measured it, I did not trace it.
- With `FIX_RANSAC_SMALL` o1 stays in `Initialising` for the rest of the run (the gate never gets below 1 deg with <= 29 window points, minimum 1.79 deg), so T_GW is never frozen; the GNSS factors keep refining it.
