# Outdoor-1 map-unit blow-up and the Indoor-2 live-vs-final gap (phone pipeline, 2026-10-08)

Scope: our own `stella_vio` + `pp_live` (never the bit-exact ports). All fixes are opt-in; every default is byte-identical (checked, section 6). GT is only used for diagnosis and scoring. No commits.
Context: `phone_pipeline/README.md`, `docs/gnss_vio_benchmark_20261001.md` sections 13-17, `docs/STATUS.md` open decision 4.
New tools (all in `phone_pipeline/diag/`): `o1_run.py` (one lock-step run over a JPEG fetch dir, any sequence, `--skip`, `--live`), `o1_batch.py` (grid of configs x starts, N workers), `o1_score.py` / `o1_table.py` (paired tables),
`sim3util.py` (GT metres per map unit from 12 s Sim3 windows), `binned.py` (time-binned ATE), `ema_table.py` (evidence table of section 2), `advio_stream.py` (ADVIO straight from Zenodo into RAM, no zip on disk).
Code: `stella_vio/c/sv_run.c` (`--diag-log`, `rf_speed`, `rf_speed_win`, `kf_min_interval`, `kf_enough_lms`), `sv_system.{c,h}` (same switches, debug fields, `SV_BRIDGE_DEBUG=1` prints the bridge prior).
Raw runs: `runs/phone_pipeline/<seq>/blowup/<config>_s<start>/` (live.tum, pp.auto, log.txt, diag.log); images of Outdoor-1 kept in `runs/phone_pipeline/_fetch_o1/outdoor1` (1.4 GB, fetch layout of `tools/gnss_harness`), Indoor-2 in `_fetch_i2`.

## 0. Answer

1. The "blow-up at 200-260 s" is a **4.5x change of the map unit at 217-228 s** (GT metres per map unit 2.1-2.4 before, 0.45-0.55 from 228 s on, canonical `live_full`, start 0). It is not a servo or fusion effect (servo is off in that run) and it is **not "not a tracking reset"** as section 17 said: the canonical run has 26 R-frames (frames 3360-3389) and one R-frame **bridge** at frame 3390 (226.4 s; frame k is at k x 66.8 ms).
2. Mechanism (evidence in section 2): the user pauses at 209-216 s in an open area and restarts. Keyframes are inserted 3-5x faster (1-1.5/s -> 5.5/s), tracked landmarks drop 135 -> 90, the nearest-structure depth (10th percentile) grows from 2 to 35 map units, and the tracked pose becomes translation-noise dominated (mean frame-to-frame step 0.080 map units = 1.2 u/s while the 2 s chord speed is 0.1-0.4 u/s). Tracking is lost at 224.4 s, the R-frame chain bridges a new map part at 226.4 s. The bridge takes its **scale from `ema_speed`, an EMA of the per-frame path length |dc|/dt**; the jitter inflates it 0.6 -> 2.16 u/s (3.5x) while the camera really progresses at 0.3-0.5 u/s. The new part is therefore placed with a 4.4x too large unit (sigma 7.77), and the 4 s re-calibration (`rf_calib`) re-fits to the same polluted number (`vold` 2.161, f 1.29).
3. Fix: opt-in `--set rf_speed=1` (`rf_speed_win` 12): the pre-gap speed is the median of 2 s **chord** speeds over the last 12 s, direction from the latest chord. Outdoor-1 canonical run: bridge prior 1.70 -> 0.38 u/s, sigma 7.77 -> 1.74, f 1.29 -> 1.008; local scale after the event 3.0-3.6 m/unit (before 2.1-2.4) instead of 0.45-0.5; raw live-map Sim3 ATE 34.8 -> 9.9 m. Over 9 start perturbations: map Sim3 ATE 15.7 -> 10.8 m, worst local scale step (max/min of 12 s windows) 4.0 -> 2.8, causal AUTO ATE 4.72 -> 4.86 m (6 of 9 runs identical, 2 worse, 1 better: noise level, the fix-dominated output does not improve). With the gait servo on, the blow-ups that made the servo unusable outdoors mostly disappear (worst step 8.4 -> 2.4, map ATE 19.2 -> 7.6, AUTO 6.32 -> 5.32 m) but the servo is still not neutral against servo-off (5.32 vs 4.72, 6 of 9 worse): **policy unchanged, servo stays off with fixes.**
4. Not solved: one of the three large steps (start 300) happens at the same pause **without any bridge or tracking loss** (36.7 -> 9.3 m/unit, 4x); `rf_speed` cannot touch it. The pause itself (stationary + far scene) is the underlying trigger; keyframe throttling did not remove it (section 5).
5. Indoor-2 (live 0.61-0.67 with servo vs final 0.31): 89 % of the excess squared error sits in the last 20 s (82-102 s) around the loop closure at 88.3 s (map jump 2.6 m, drifted pre-loop map), 5 % in the start-up bin. Two fusion-side remedies (loop flagged as GAP, as NEW_FRAME) were worse and are rejected (section 7).

## 1. Reproduction and the first moment the scale goes wrong

Images (not on disk before): `external/gnss/venv/bin/python tools/gnss_harness/mobilegvio_to_euroc.py Outdoor-1 <dir> 1e9 --every 2` (5898 frames, 1.4 GB, about 60 min over the Zenodo link).
Run: `external/gnss/venv/bin/python phone_pipeline/diag/o1_run.py base --live` (= `run.py live outdoor1 --variants full`). **`live.tum`, `trajectory.tum` are `cmp`-identical to the canonical `live_full/` / `sv_full/`** (also with the diagnostics code added), AUTO 4.312 m as in section 15.

Map unit vs GT (GT metres per map unit: Sim3 scale of 12 s windows, step 6 s; rms of the fit 0.02-0.14 m before, so the windows are clean; start 0, live = final here):

| window [s] | GT chord m | map chord u | m/unit base | m/unit `rf_speed=1` | servo `sv05` | servo + `rf_speed=1` |
|---|---|---|---|---|---|---|
| 168-180 | 16.6 | 7.3 | 2.26 | 2.26 | 3.09 | 3.09 |
| 192-204 | 16.1 | 6.9 | 2.32 | 2.32 | 2.97 | 2.97 |
| 204-216 | 7.2 (pause) | 3.5 | 2.07 | 2.07 | 2.87 | 2.87 |
| 210-222 | 8.7 | 2.95 | 2.52 | 2.52 | 3.59 | 3.59 |
| 216-228 | 8.8 | 6.4 | **1.44** | 2.57 | 2.41 | 3.01 |
| 222-234 | 13.2 | 19.0 | **0.63** | 3.28 | 2.49 | 4.01 |
| 228-240 | 10.0 | 19.7 | **0.50** | 3.18 | 2.21 | 3.58 |
| 252-264 | 16.3 | 35.1 | **0.45** | 3.31 | 2.75 | 3.27 |
| 264-276 | 16.7 | 33.7 | **0.51** | 3.11 | 2.95 | 3.11 |

First moment (1 s resolution of the same run, local m/unit): 2.2-2.6 up to 208 s; the user slows at 209 (GT speed 0.08-0.27 m/s at 210-212, gait detector reports nothing between 210 and 219: no gait epochs in `pp.speed`); the map follows (0.19 u/s at 211); from **217.5 s** the map speed per real metre jumps (m/unit 1.7, 1.06, 0.71 at 217/218/219), at 224 s tracking drops to 20 landmarks and is lost at 224.4 s, R-frames 224.4-226.4 s, bridge at 226.4 s, m/unit 0.6-0.7 from 226 s (0.5 after the 4 s re-fit at 230 s).

State of stella_vio around the event (`runs/phone_pipeline/outdoor1/blowup/b3800d/diag.log`, 4 s bins):

| t [s] | keyframes/s | mean tracked lms | notes |
|---|---|---|---|
| 176-208 | 1.0-2.8 (mean 1.6) | 126-152 | walking |
| 208-212 | 2.5 | 135 | pause starts |
| 212-216 | 3.25 | 117 | |
| 216-220 | **5.5** | **92** | restart; 10th-percentile landmark depth of the new keyframes 2.8 -> 25-35 u (median 15-20 -> 55-60 u, far limit about 77) |
| 220-224 | **5.0** | **90** | |
| 224-228 | 1.5 | 101 | lost 224.4, 26 R-frames, bridge 226.4 |

Other state: 0 lost-state resets, 0 re-inits, 0 merges, 0 loops (`log.txt`: `maps=1 reinits=0 merges=0 rframes=26 rbridges=1`, 728 keyframes, landmarks 17.9k -> 18.2k at the event). Gravity: the per-map up vector is stable (live.tum columns 15-17), not involved. Servo: off in this run. Gyro prior: on (`gyro=1`), rotation fine (R-frame rotation inliers 226-558, median residual 0.11-0.15 deg). GNSS fusion's view: the fusion gets a new segment flag (GAP|LOOSE) at 226.4 s and re-fits its scale from the gait speed and the fixes, which is why the fused output survives; AUTO ATE per 30 s bin: 210-240 s **5.44 m (base) vs 2.27 m (`rf_speed=1`)**, 240-270 s 2.57 vs 1.70, whole run 4.31 vs 4.14.

## 2. Mechanism with numbers

(a) Why the map moves too little in 209-224 s and then jumps. After the pause the view is a far field: median landmark depth 55-60 map units = about 45 m with the pre-pause 2.2 m/u, 10th percentile 25-35 u. A translation of 1 m changes the image of such points by about 2 %, so PnP does not constrain the translation magnitude. Measured in the pose stream (frames 216-224 s, 124 tracked frames): mean frame-to-frame centre step 0.080 u (1.2 u/s) with 33 frames above 0.1 u (>1.5 u/s) and a maximum of 0.47 u; the 2-4 s chord speeds in the same stretch are 0.1-0.4 u/s (pre-pause 0.6 u/s, expected 0.46-0.5 for a 1.1 m/s walk at 2.2-2.4 m/u). The returned poses themselves jitter (the step statistics are computed on the pose returned by `sv_system_feed`, not on a BA-corrected pose).

(b) What the bridge reads. `sv_system.c` keeps `ema_speed = 0.97 ema_speed + 0.03 |dc|/dt` (path length per frame) and `ema_vel` (vector EMA, 0.1 weight). Per 2 s bin from the `--diag-log` (`phone_pipeline/diag/ema_table.py`, units map units/s):

| t [s] | `ema_speed` (path EMA, used) | `|ema_vel|` | chord speed of the bin | path speed of the bin | max step/frame [u] (frames > 0.1) |
|---|---|---|---|---|---|
| 200 | 0.617 | 0.631 | 0.607 | 0.629 | 0.080 (0) |
| 208 | 0.552 | 0.441 | 0.475 | 0.511 | 0.084 (0) |
| 212 | 0.308 | 0.236 | 0.161 | 0.240 | 0.028 (0) |
| 216 | 0.507 | 0.475 | 0.474 | 0.574 | 0.063 (0) |
| 218 | 0.940 | 0.627 | 0.385 | 1.162 | 0.203 (8) |
| 220 | 1.125 | 0.019 | 0.117 | 1.259 | 0.202 (9) |
| 222 | 1.490 | 0.500 | 0.406 | 1.596 | 0.473 (12) |
| 224 | **2.161** | 1.699 | 0.243 | 3.314 | 0.581 (8) |

At the bridge (`SV_BRIDGE_DEBUG=1`): `vnorm 1.6988 ema_speed 2.1610 base_new 0.0731 speed_prior 7.7661 sigma 7.7661`; the 4 s re-fit prints `f 1.290 vold 2.1610 vnew 1.6751`. The new part is near-field (median depth 6 u, 174 tracked landmarks), so it tracks cleanly at 1.675 u/s (2.16 after the re-fit); real 1.1 m/s over 2.16 u/s = 0.51 m/unit, i.e. 4.4x the 2.3 m/unit before (2.16 / 0.5 expected = 4.3x). The unit inherited from the polluted number is the whole blow-up. Section 17 observed "the map recovers within 60 s without the servo" for other starts; there the polluted pre-gap speed was smaller (no bridge in 6 of 9 starts, see below).

(c) The same pause without a bridge (start 300, `d300/diag.log`): keyframes 5.75/s at 212-216 s, tracked 96; path speed 0.036 -> 0.146-0.46 u/s (4-13x) at 214-218 s, chord 0.13-0.17 u/s afterwards against 0.035 before: a genuine 4-5x unit change (36.7 -> 9.3 m/unit) with 1 R-frame and no bridge. So the pause makes the map lose its scale by itself, and the bridge pollution makes it worse and permanent. Not instrumented further (hypothesis: the scale of the newly built local map after the pause is set by far landmarks only).

## 3. The fix: `rf_speed=1` (opt-in, default off)

`stella_vio/c/sv_system.c rspeed_update`: ring of the last 512 tracked centres; chord velocity over the newest sample at least 1.5 s back (about 2 s); `ema_speed` = median of those chord speeds of the last `rf_speed_win` = 12 s; `ema_vel` = direction of the latest chord, length = the median speed. Used exactly where the old EMAs were: R-frame extrapolation, bridge prior, bridge re-fit. `scale_section` also scales the ring (servo / calibration consistency). Without the switch the code path is the old one.

Same canonical run with `--set rf_speed=1`: `vnorm 0.3806 ema_speed 0.3806 speed_prior 1.7399 sigma 1.7399`, re-fit `f 1.008 vold 0.3806 vnew 0.3775` (no correction needed); m/unit after the event 2.6-3.6 (table above), `late/early` median-scale ratio 1.25 against 0.20 (base).

## 4. Outdoor-1, all start perturbations (pp_live, `full`, causal AUTO ATE SE3 [m] / raw live map Sim3 ATE [m] / worst step)

Starts as in section 16 (`--skip` 0..900 frames; 90 duplicates 75). `base` is the canonical configuration (re-run: start 0 reproduces `live_full`); `sv05` = `servo=0.5 servo_clip=0.2 servo_win=6 servo_dmin=2`. Worst step = max/min of the 12 s Sim3 scale from 24 s on (1 = constant). Numbers: `runs/phone_pipeline/outdoor1/blowup/table_cache.json`, `phone_pipeline/diag/o1_table.py base rf1 sv05 sv05rf1`.

| start | base | `rf_speed=1` | servo | servo + `rf_speed=1` |
|---|---|---|---|---|
| 0 | 4.31 / 34.8 / 7.6 | 4.14 / 9.9 / 1.8 | 4.38 / 3.3 / 1.7 | 5.13 / 6.9 / 1.8 |
| 15 | 4.18 / 8.5 / 2.3 | = base (no bridge) | 5.53 / 14.8 / 3.5 | 4.14 / 4.0 / 2.0 |
| 30 | 4.53 / 11.0 / 2.4 | = base | 5.48 / 16.9 / 3.2 | 4.91 / 5.1 / 2.0 |
| 45 | 8.11 / 32.5 / 8.3 | 9.12 / 10.8 / 2.6 | 11.89 / 40.8 / 29.8 | 11.12 / 19.5 / 4.7 |
| 60 | 4.32 / 5.4 / 2.0 | 4.75 / 7.4 / 2.3 | 9.78 / 33.3 / 20.4 | 4.24 / 3.9 / 1.7 |
| 75 | 3.78 / 6.7 / 2.2 | = base | 5.94 / 31.6 / 7.7 | 5.45 / 6.1 / 2.0 |
| 300 | 5.50 / 29.0 / 7.0 | = base (no bridge) | 5.15 / 16.3 / 3.7 | 4.58 / 16.1 / 3.5 |
| 600 | 3.67 / 7.1 / 2.3 | = base | 4.21 / 12.2 / 2.9 | 3.90 / 4.0 / 1.8 |
| 900 | 4.10 / 6.5 / 2.3 | = base | 4.53 / 3.7 / 2.4 | 4.40 / 3.0 / 2.1 |
| **mean** | **4.72 / 15.7 / 4.0** | 4.86 / 10.8 / 2.8 | 6.32 / 19.2 / 8.4 | 5.32 / 7.6 / 2.4 |
| median | 4.31 / 8.5 | 4.18 / 8.5 | 5.48 / 16.3 | 4.58 / 5.1 |

Paired against base (AUTO, 0.02 m threshold): `rf_speed=1` better in 1, worse in 2, equal in 6 of 9 (mean +0.14 m); servo worse in 8 of 9 (mean +1.60 m); servo + `rf_speed=1` better in 3, worse in 6 (+0.59 m). Reading: (i) `rf_speed=1` only changes runs that have a bridge (3 of 9: starts 0, 45, 60) and removes the map-unit step in the two where it was large (7.6 -> 1.8, 8.3 -> 2.6); the AUTO output does not move beyond noise because it is dominated by the fixes (and the fusion already re-fits scale at the segment boundary). (ii) The servo-induced blow-ups (starts 45, 60, 75: steps 29.8 / 20.4 / 7.7) are gone with `rf_speed=1` (4.7 / 1.7 / 2.0); servo map ATE mean 19.2 -> 7.6 and AUTO 6.32 -> 5.32. (iii) The servo still costs about 0.6 m against servo-off outdoors, mostly from starts where nothing blows up (start 0: 5.13 vs 4.31, start 75: 5.45 vs 3.78), which is the reference-poisoning problem of section 17 (false walking at the start of the run), not this one. (iv) Start 300 keeps its 7.0x step (pause without bridge).

## 5. Rejected trials (numbers; also in `docs/rejected_trials.md`)

* Keyframe throttling against the pause flood, start 0 / 45 / 300, AUTO / map ATE / worst step (base 4.31 / 8.11 / 5.50, steps 7.6 / 8.3 / 7.0): `kf_min_interval=0.5` 9.32 / 13.54 / 4.69, steps 3.5 / **406** / 11.1; `kf_enough_lms=60` 6.17 / 10.57 / 4.82, steps 3.5 / 6.7 / 5.2. The unit step is not removed (start 45 explodes with the interval cap), mean AUTO of the three starts worse (9.18 and 7.19 vs 5.97 for base); switches kept (opt-in, default off) as study tools only.

## 6. No regression on the other phone sequences, and default identity

| check | result |
|---|---|
| fr1_xyz exact port (`reinit_sec=0 init_max_level=0 init_confirm=1`) vs `stella_port` | 787 poses, `cmp` identical (rebuilt binary with all new code) |
| Outdoor-1 base: `live.tum`, `trajectory.tum` vs canonical `live_full/`, `sv_full/` | `cmp` identical |
| Indoor-2 base `live.tum` vs canonical `live_full/` | identical; ADVIO-15 base `live.tum` vs canonical: identical |
| Indoor-1, Indoor-2: `rf_speed=1` vs base, 6 starts each | `live.tum` identical for all (no R-frames, ema state unused): AUTO 1.20 / 1.06 mean for both |
| Outdoor-2 | no R-frames in `full` (`rframes=0`), so `rf_speed` is unreachable; not re-run (images not fetchable, 450 s over 0.3 MB/s) |
| ADVIO-20 (`full`: 66 R-frames, 1 bridge, no scale re-fit event), start 0 | base 11.820 / `rf_speed=1` 11.824 (map Sim3 10.247 vs 10.245) |
| ADVIO-15 (bridge + re-fit), 6 starts (0,10,20,30,40,50) | base 0.85 0.84 0.85 0.79 0.97 1.03 (mean 0.89) -> `rf_speed=1` 0.76 0.77 0.81 0.79 0.70 0.76 (mean **0.77**; 5 better, 0 worse); map ATE 0.7 both |

Servo (GNSS-free policy) with `rf_speed=1` on the GNSS-free sequences: ADVIO-15 servo 0.84, servo + `rf_speed=1` 0.85 (base 0.89; against base 3 better, 1 worse of 6); Indoor sequences are identical by construction (servo 0.79 / 0.61 as in section 16, reproduced: Indoor-1 1.20 -> 0.79, Indoor-2 1.06 -> 0.61).

Before/after in the metrics of the earlier studies (live causal ATE SE3, mean over starts): Indoor-1 1.20 -> 1.20, Indoor-2 1.06 -> 1.06, ADVIO-15 0.89 -> 0.77, ADVIO-20 11.82 -> 11.82 (1 start), Outdoor-1 4.72 -> 4.86 (map ATE 15.7 -> 10.8), Outdoor-2 not affected by construction. Because `rf_speed=1` is not ATE-neutral on ADVIO-15 (-0.12 m, 5 of 6) and Outdoor-1 map consistency improves while its fused ATE does not, it is **not promoted to default**; it stays opt-in (`--set rf_speed=1`), and the AGENTS.md rule about a full `benchmark_native.py --all_gt` does not apply (no change in the `simple_slam_*` benchmark code).

## 7. Indoor-2: live vs final (item 4)

Same method. Map unit vs GT (8 s windows, `live_full`): 9.3 m/unit at 6-14 s -> 8.3 (24-32 s) -> 7.6 (36-44 s) -> 7.1 (54-80 s) -> 5.6 (78-86 s) -> **4.5 (90-98 s)**; the final trajectory stays at 8.7-9.8 m/unit for the whole run (loop closure at frame 1382 = 88.3 s with Sim3 and BA). The unit drifts about 1.3 %/s, no event, no tracking loss, no R-frames (`rframes=0`): plain monocular scale drift in a corridor, removed only by the loop.

Time-binned error (SE3 over the whole scored run, `phone_pipeline/diag/binned.py`; scored from +12 s; final = `fuse_full_gait/causal.live`, live = `live_full/pp.auto`, servo = `live_servo/pp.auto`):

| bin [s] | final | live | live + servo |
|---|---|---|---|
| whole run | 0.309 | 1.104 | 0.667 |
| 12-22 | 0.367 | 0.871 | 0.547 |
| 22-32 | 0.194 | 0.544 | 0.345 |
| 32-42 | 0.254 | 0.295 | 0.333 |
| 42-52 | 0.188 | 0.229 | 0.228 |
| 52-62 | 0.322 | 0.324 | 0.361 |
| 62-72 | 0.220 | 0.223 | 0.245 |
| 72-82 | 0.265 | 0.262 | 0.290 |
| 82-92 | 0.578 | 1.112 | 0.956 |
| 92-102 | 0.144 | 3.092 | 1.593 |

Between 32 and 82 s live equals final (+-0.03 m). The servo's excess over final (mean square 0.479 vs 0.094 m^2 over the nine bins) is **89 % in 82-102 s** (drifted pre-loop map, the loop jump of 2.6 m = 0.314 map units in one frame at 8.4 m/unit, and the fusion's scale/yaw alignment estimated in the pre-loop frame) and 5 % in the 12-22 s start-up bin; the servo already removes the drift part (map unit 7.0-9.4, max/min 1.3 instead of 2.1). Fusion-side remedies on the post-loop frame, 6 starts, causal AUTO mean (servo 0.61): loop flagged `GF_ODOM_GAP` (`--pp-jump 1`) **0.67**, loop flagged `GF_ODOM_NEW_FRAME` (re-aligned as a new odometry frame) **0.70**, GAP worse in 6 of 6 starts, NEW_FRAME in 5 of 6; rejected (the NEW_FRAME switch was removed again). What would still be needed is a map-correction message from the mapper to the fusion (apply the Sim3 of the loop to the fusion state), not tried.

## 8. Open points

* The pause trigger itself (start 300: 4x step without bridge). Untested ideas: refuse to insert / trust new landmarks while the camera is stationary (needs a stationarity signal, e.g. the IMU accelerometer variance, which the gait stage already has), or gate the scale of a new keyframe against the gait/IMU speed at the restart.
* Servo outdoors: needs the section-17 reference problem solved on top of `rf_speed=1` (clean reference: 2.6 vs 3.58 m/unit).
* Outdoor-2 and the live ADVIO-20 multi-start runs with `rf_speed=1` (Outdoor-2: no R-frames; ADVIO-20 only start 0 run).
* The section-17 sentence "not a tracking reset" in `docs/gnss_vio_benchmark_20261001.md` is wrong for the canonical run (R-frames + bridge); left unedited, this document supersedes it.
