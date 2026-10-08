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

## 7. Validation on more GNSS data: unmodified vs `FIX_RANSAC_SMALL` vs the loose smoother (2026-10-08)

Setup: C port (`okvis_port/c`, `OKVIS_PORT_OKVIS2X=1`), single-threaded deterministic configs (`okvis2x_port/reference/configs/okvis2x_mono_{gvins,phone}_deterministic.yaml` = the shipped
configs of the earlier GNSS study with `parallelise_detection: false`, 1 realtime / 1 full-graph thread, `do_final_ba: false`; `robust_gps_init: true`, `yaw_error_threshold: 1.0`).
Driver `tools/okvis2x_val.py` (run / score / loose / cmp; python `external/gnss/venv/bin/python`), scoring = `tools/gnss_harness/gnss_eval.py` as in the drone benchmark: SE3 = rigid-aligned ATE of
`final.csv`, geo = RMS / median / max of `global_final.csv` against the reference with NO alignment, scored at the antenna (lever of the config). "loose" = `tools/gnss_loose_fusion.py` (batch) on the
GNSS-free C-port VIO trajectory (`final.csv` of a run without `gps_parameters`) with the same fixes. Data and outputs: `runs/okvis2x_port/val/` (not committed; raw downloads never stored,
images deleted afterwards; the converted `m14` PNGs of section 1 were also deleted for disk, `o1` kept). Fetch: GVINS-Dataset by HTTP range from Hugging Face straight into PNG
(`gvins_bag_to_euroc.py` now accepts a URL), Mobile-GVIO from Zenodo (`mobilegvio_to_euroc.py --png`).

Sequences actually obtained (GNSS input variants are simulated from the receiver's RTK epochs with `tools/gnss_harness/make_gps.py`: `rtk10` = RTK fixes as is, 10 Hz; `rtk5` = RTK at 5 Hz; `sim5` /
`sim1` = SPP/phone-grade AR(1) noise 1.5 m horizontal / 3 m vertical (tau 30 s) at 5 / 1 Hz; the RTK rows are optimistic because RTK is also the scoring reference):

- `gv_complex`: GVINS-Dataset `complex_environment`, first 300 s (6000 frames, 20 Hz, handheld walk, moving from the first frame, RTK float/no-carrier in the last 60 s).
- `gv_sports`: GVINS-Dataset `sports_field`, first 240 s (4800 frames, walking 1.1-1.4 m/s from the start, 327 m, RTK fixed for 100% of epochs).
- `o1cut`: INSANE `o1` cut at t = 50 s (988 frames, 5 s of stationary ground then take-off and flight; real PX4 receiver 5 Hz; same files/reference as section 1). This is the "short stationary start" case.
- `ph_o1`: Mobile-GVIO Outdoor-1 (Honor phone, iPhone fixes 1 Hz), first 85 s only (the Zenodo stream broke at 85 s: `ValueError: buffer is smaller than requested size`). The dataset GT starts at t = 287 s, so
  the only global reference in this window is the iPhone fixes themselves: "geo" is the error against those fixes (floor = their own error), not against truth.
- NOT obtained: further INSANE sequences. `cns-data.aau.at` stopped answering (TCP connect timeouts for the whole session after the first probe, which showed only `outdoor_1` among the outdoor_* names
  and `mars_1,2,3,4,7,8,9,14` with nav-cam zips of 1.7-5.6 GB). So there is no new INSANE sequence beyond `o1`/`m14` and the `o1cut` re-cut; stationary-start evidence is `o1`, `o1cut`, `m14`.

### 7.1 Results (geo = no alignment to the reference; "med / max" for geo)

| seq / GNSS input | VIO only SE3 | unmodified SE3 \| geo RMS (med / max) | `RANSAC_SMALL` SE3 \| geo RMS (med / max) | loose smoother SE3 \| geo | GNSS state: Initialising / Initialised [s] unmod -> fix |
|---|---|---|---|---|---|
| o1 (100 s, PX4 5 Hz) [sec. 4.1] | 0.724 | 0.724 \| 47.52 (0.61 / 130.0) | 0.206 \| 8.57 (8.80 / 9.02) | 1.47 \| 10.45 (drone doc) | never -> 74.6 / never |
| m14 (181 s, PX4 5 Hz) [sec. 4.1] | | 1.343 \| 4.50 (3.92 / 7.64) | 1.343 \| 4.50 (byte-identical) | 1.46 \| 4.15 (drone doc) | 71.0 / 83.2 both |
| o1cut (50 -> 100 s, PX4 5 Hz) | 0.418 | 0.418 \| **64.49** (50.7 / 130.7) | 0.400 \| **8.32** (8.32 / 8.97) | 0.827 \| 11.28 | never -> yes / never |
| gv_complex rtk10 | 4.332 | 0.199 \| 0.207 (0.04 / 0.8) | 0.235 \| 0.238 (0.04 / 1.2) | 0.062 \| 0.079 | 62.9 / 63.0 -> 10.0 / 62.9 |
| gv_complex rtk5 | | 1.201 \| 1.515 (0.06 / 6.6) | 0.448 \| 0.472 (0.06 / 1.8) | 0.099 \| 0.101 | 116.1 / 116.3 -> 14.0 / 116.1 |
| gv_complex sim5 | | 1.364 \| 5.830 (5.67 / 6.7) | 1.565 \| 5.660 (5.60 / 8.0) | 1.324 \| 3.304 | 116.1 / 116.3 -> 17.4 / 113.1 |
| gv_complex sim1 (1 Hz) | | 4.332 \| **302.58** (233.1 / 561.1) | 4.230 \| **7.18** (5.65 / 19.1) | 1.741 \| 4.736 | never -> 81.0 / never |
| gv_sports rtk10 | 5.957 | 0.871 \| 0.890 (0.05 / 6.2) | 0.341 \| 0.358 (0.03 / 1.7) | 0.069 \| 0.075 | 122.4 / 122.5 -> 13.6 / 112.7 |
| gv_sports rtk5 | | 1.193 \| 1.235 (0.11 / 6.0) | 0.429 \| 0.434 (0.05 / 1.9) | 0.083 \| 0.096 | 227.9 / 228.1 -> 38.6 / 224.3 |
| gv_sports sim5 | | 1.693 \| 2.958 (2.54 / 6.6) | 1.695 \| 2.845 (2.48 / 6.3) | 1.013 \| 3.422 | 227.9 / 228.1 -> 38.6 / 227.9 |
| gv_sports sim1 (1 Hz) | | 5.944 \| **134.90** (141.2 / 175.2) | 5.727 \| **8.89** (9.09 / 12.6) | 1.477 \| 4.905 | never -> 227.1 / never |
| ph_o1 85 s, iPhone fixes as given (hErr 14.25 m) | 3.818 | 3.818 \| 57.77 (41.3 / 112.0) | identical, same hashes | | GNSS never used: every fix `CV reject-inaccurate` (cov > 6 m sigma gate in `checkValidGpsMeasurements`); the fix cannot act |
| ph_o1 what-if: same fixes, reported sigma set to 5 m / 8 m | 3.818 | 3.818 \| 57.77 | 3.966 \| **5.31** (3.87 / 10.1) | 2.484 \| 3.863 | never -> yes / never |

VIO-only SE3 = the same C port without `gps_parameters`. The unmodified `sim1` and `ph_o1` runs have the causal trajectory byte-identical to the VIO-only run: with no GNSS state ever
initialised, GNSS has no effect at all. Where unmodified never initialises (1 Hz, 5 Hz PX4 short windows) the geo error is the identity-frame error; the fix removes it (302.6 -> 7.2, 134.9 -> 8.9,
64.5 -> 8.3, 47.5 -> 8.6, 57.8 -> 5.3 m) down to the noise level of the fixes (about the same floor as the loose smoother, 3.3-11.3 m). With the fix off, higher rates also initialise late
(window of 40 points: first Initialising 63-228 s on the 5-10 Hz GVINS data versus 10-39 s with the fix, Initialised at the same time or a few seconds earlier) and the 10 Hz cases are 2-3x
better (sports rtk10 0.890 -> 0.358, rtk5 1.235 -> 0.434, complex rtk5 1.515 -> 0.472). The loose smoother is better than OKVIS2-X on RTK-grade data (0.075-0.10 vs 0.24-1.2 m: it uses the full VIO trajectory
as a batch, OKVIS2-X is causal in the estimate it freezes) and comparable on noisy data.

### 7.2 Where the fix is not better, and the noise floor

- `gv_complex rtk10`: 0.207 -> 0.238 geo (+0.031), SE3 0.199 -> 0.235. Per time (RMS): t < 240 s 0.192 vs 0.176 (fix better), t >= 240 s (the RTK-float region) 0.277 vs 0.452; t < 60 s 0.246 vs 0.048.
  Noise floor of the experiment: dropping the first three fixes (`rtk10p`, which cannot matter physically) changes the unmodified result 0.207 -> 0.265 geo (SE3 0.199 -> 0.257) and the fix result
  0.238 -> 0.408 (SE3 0.235 -> 0.385; t < 60 s 0.048 -> 0.794 because the preliminary 6-point T_GW before the 40-point `Initialised` differs). So the +0.03 is inside the sensitivity of the unmodified
  run to a 0.3 s change of the input (+/-0.06), but the fix is the more sensitive one on this sequence (0.24 vs 0.41 for the two inputs; mean +0.09 m on RTK-grade 10 Hz data). `rtk5p` unmodified = byte-identical to `rtk5` unmodified (causal hash
  `a2e2d38d9cfd`), so no extra sample there.
- `gv_complex sim5`: SE3 1.364 -> 1.565 (+0.20), geo 5.83 -> 5.66; `gv_sports sim5`: SE3 1.693 -> 1.695, geo 2.96 -> 2.85. The geo error here is the 30 s-correlated simulated bias (floor), the SE3 difference is again
  the sensitivity of the later part of the run.
- Everything else moves in the direction of the fix: 7 of 8 GVINS pairs better in geo, 1 worse by 0.03 (inside the noise); m14 unchanged (byte-identical); EuRoC cases of section 4.2 neutral.

### 7.3 Upstream transfer (C == X) spot-check

`gv_complex sim1` (one of the failing, never-initialising cases), unmodified C vs the deterministic OKVIS2-X reference built in `runs/okvis2x_port/reference_build` (`tools/okvis2x_run_reference.py gvsim1 --tag x_sim1`,
data root `runs/okvis2x_port/val/xroot`, the same deterministic gvins config, X wall 531 s): `tools/okvis2x_val.py cmp gv_complex sim1 unmod gvsim1/x_sim1` -> PASS, causal.csv byte-identical (`00ccc166654c`), final.csv cols 1-17
identical, global_final.csv byte-identical (`8580344e3212`). So the failure (identity T_GW, 302.6 m geo) is upstream behaviour on this data, not a port artefact. Together with o1 and m14 (section 2) that is three real sequences with C == X.

### 7.4 Verdict: ADOPT `FIX_RANSAC_SMALL` as the recommended opt-in (keep default off to preserve bit-exactness); no refinement variant needed

- Evidence for: five independent cases of "GNSS never initialises" (o1, o1cut, gv_complex sim1, gv_sports sim1, ph_o1 what-if, plus EuRoC `gps_b_mono1`) go from 47-302 m geo error to 5-9 m (noise level of the fixes), and 5-10 Hz data initialises
  5-200 s earlier; the changed code path is exactly the one that fails (the first Idle -> Initialising check with 6 <= n < 40 window points), later `Initialising -> Initialised` still uses the 40-point rule.
- Evidence against / limits: one RTK-grade sequence (complex rtk10) is +0.03 to +0.17 m worse (inside the +/-0.06 m input-sensitivity of the unmodified code; fix variance larger), the 6-point preliminary T_GW can be poor for tens of seconds
  (0.79 m RMS in the first 60 s of `rtk10p_fix`); with <= 1 Hz data the state never reaches `Initialised` (stays `Initialising`, T_GW never frozen, the GNSS factors keep refining it) - same as o1.
- No regression above the repo's 0.01 m neutral band was found that is attributable to the fix rather than to chaotic divergence; the one place that looks like a real cost (early preliminary T_GW) was not worth a refined
  variant: `FIX_HISTORY` already showed (section 4.1) that using more points earlier regresses m14, and tightening the minimum n would re-introduce the exclusions the fix removes (earlier 20..39-point variants regressed `gps_b_r2_1`, section 4.2).
- Caveats: phone data is not a test of the fix with the stock gate: the iPhone's reported 14 m sigma is rejected by `reject-inaccurate` (> 6 m) before any init logic; only with the sigma lowered to 5 m does the fix matter (what-if row,
  reference = the fixes, 85 s). Phone OKVIS2-X mono VIO itself is poor (scale collapse in the earlier study). No stationary-start INSANE sequence beyond o1/o1cut/m14 could be added because the INSANE server was unreachable; no new
  INSANE outdoor sequence (only `outdoor_1` exists under outdoor_*). `do_final_ba: false` throughout. Single deterministic runs, simulated GNSS noise for 3 of the 5 GVINS inputs.
- Files added: `tools/okvis2x_val.py`, configs `okvis2x_mono_{gvins,phone}[_nogps]_deterministic.yaml`, `okvis2x_mono_drone_o1_nogps_deterministic.yaml`; `gvins_bag_to_euroc.py` (URL streaming) and `mobilegvio_to_euroc.py` (`--png`) extended.
