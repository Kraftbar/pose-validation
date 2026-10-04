# Research and strategy roadmap, 2026-10-03

Scope: what to do next for the permissive, dependency-free C99 pose stack (phones: mono camera + IMU + phone GNSS;
drones: camera(s) + IMU + GNSS), and why. Inputs: the project docs listed at the end, a literature/repo sweep
(2023-2026), and one tiny experiment (appendix A). Nothing here is committed or promoted; licences were read from the
repositories' own LICENSE files where stated as "(checked)".

## 0. Where we stand (from the docs, not re-verified)

| component | state | evidence |
|---|---|---|
| `stella_port` | bit-exact C port of stella_vslam mono, 2.2 cm mean ATE on TUM, 56/56 harness rows | README, `stella_port/HANDOVER.md` |
| `stella_vio` | fork: re-init into new maps, wider init matcher, init confirm (multi-start 92 -> 132/140); gyro prior / gravity / dead-reckoning opt-in, none promoted | `stella_vio/RESULTS.md` |
| `gnss_fusion` | C 4-DoF+scale smoother, robust preset; causal within 1.0-1.3x of GNSS-alone on 14/17 phone+drone cases; drone yaw failure fixed (o1d 8.88 -> 1.43) | `docs/gnss_vio_benchmark_20261001.md` s.10-11 |
| `okvis_port` | M1-M4 bit-exact (IMU, kinematics/cameras, error terms, the Ceres solver); M5 part 1 bit-exact (TwoPose pose-graph terms, updateLandmarks, ceres::Problem program order); M5 part 2 (graph state / mutations) next | `okvis_port/HANDOVER.md`, `PLAN.md` |
| phone finding | every mono-inertial system (OKVIS2-X, ORB-SLAM3-MI, OpenVINS, VINS-Fusion, XRSLAM) loses metric scale on long walks; cause = errors-in-variables bias: scale-observable signal 15-30 mm / 4 s vs visual position error 17-64 mm | `docs/phone_scale_diagnosis_20261003.md` |
| phone best | XRSLAM (Apache-2.0): full coverage, metric on 3 short/indoor seqs (0.84 / 0.98 / 1.55 m) and Outdoor-1 (6.44 m), scale collapse on Outdoor-2 / ADVIO-20 | s.10 |
| drone best | OKVIS2 (BSD-3) never loses tracking (0.53 m SE3 on a 20 m/s FPV flight); own loose fusion reproduces OKVIS2-X on m14 (1.46 vs 1.39 m) | `docs/drone_benchmark_20261002.md` |

The single most important number: on phones the raw 1 Hz fixes (5.7 / 14.7 / 12.0 m) beat every full-coverage
camera result, and fusion only reaches "no worse than GNSS alone" because the odometry has no reliable scale.

## 1. State of the art relevant to our blockers

### 1.1 Monocular VIO scale on pedestrians / smartphones

| approach | what it would buy us | licence | maturity | effort | verdict |
|---|---|---|---|---|---|
| **Gait (PDR) speed/step-length prior** as a factor on the scale state ([Sensors 2019, adaptive step length for handheld mono VO](https://doi.org/10.3390/s19040953): 1.6-7.5 % of distance; [PS-VINS 2024](https://ieeexplore.ieee.org/document/10400404/): step velocity + length models inside a smartphone VI-SLAM back end, no code) | metric scale indoors and outdoors from the accelerometer alone, independent of visual quality; our own check (appendix A): distance ratio 0.97-1.07 on the four Mobile-GVIO sequences with one calibration, 0.74-0.85 cross-user (ADVIO-20) | own code (model is textbook: Weinberg / cadence-linear) | mature in the PDR literature; not in any permissive VIO | small (C: ~150 LOC factor + step detector) | **do first** |
| GNSS **Doppler velocity** (Android `GnssMeasurement.PseudorangeRateMetersPerSecond`) as a metric velocity factor | per-epoch metric speed at 1-2 dm/s on phones in motion ([Android multi-GNSS Doppler evaluation](https://d-nb.info/1256353183/34); [velocity-aided smartphone positioning](https://www.sciencedirect.com/science/article/abs/pii/S0273117719300122)), i.e. 10-15 % scale per epoch at walking speed, better averaged; works for running / cycling / vehicles where gait models fail; `gnss_fusion` already has the velocity factor (simulated test: 2.04 -> 1.98 m) | own code; RTKLIB (BSD-2-Clause + clauses, checked) as reference for ephemeris/SPP | mature | medium (SPP + Doppler velocity solver ~1.5-2 k LOC C) | **do, after data** (no phone dataset we hold has raw Android GNSS; iOS exposes no raw data) |
| Learned inertial odometry: [TLIO](https://github.com/CathIAS/TLIO) (code BSD, checked; headset data), [RNIN-VIO](https://github.com/zju3dv/rnin-vio) (code Apache-2.0, checked; smartphone data, weights via Drive, dataset terms unstated), [AirIO](https://github.com/Air-IO/Air-IO) (BSD-3, checked; drones, EuRoC/Blackbird models), [EqNIO](https://github.com/RoyinaJayanth/EqNIO) (ICLR 2025, licence not found, dataset CC-BY-NC), [RoNIN](https://github.com/Sachini/ronin) (GPL-3, out), surveys [arXiv 2303.03757](https://arxiv.org/html/2303.03757v3) | 3-D displacement + covariance per 1 s window from IMU only; RNIN-VIO exists precisely to hold scale in low-excitation phone VIO | code permissive, **weights/training data are the risk** (NC or unstated) | research code, low activity (RNIN 3 commits) | eval 1 day; C99 inference of a 1-D ResNet ~2-3 days; own training data = large | evaluate offline, do not depend on it |
| Zero-velocity updates | only useful when the device actually stops (phones in hand rarely do); already implicit in gait model (cadence 0 -> speed 0) | own | - | tiny | fold into the gait factor |
| Learned-depth-aided VI init ([Merrill et al. 2023/2025](https://udel.edu/~nmerrill/pubs/Merrill2023RSS.pdf)), feed-forward 3D model init ([arXiv 2605.17327](https://arxiv.org/html/2605.17327v1)), line/VP init ([arXiv 2609.21186](https://arxiv.org/html/2609.21186)), [XR-VIO](https://arxiv.org/abs/2502.01297) (4-frame gyro-coupled init, no code found) | faster, less seed-sensitive initialisation under low parallax | networks: not dependency-free; VP/line and gyro-coupled rotation ideas are implementable | papers | medium | take the *idea* (gyro-tight rotation, then linear translation/scale) into `stella_vio`'s initialiser later; no network |
| Rolling-shutter VIO ([Ctrl-VIO](https://github.com/APRIL-ZJU/Ctrl-VIO), continuous-time) | models RS distortion | not checked (APRIL-ZJU repos are typically GPL) | research | large | **skip**: our diagnosis measured RS/exposure stamping as <= 0.025 scale effect |

### 1.2 Fast-rotation tracking loss and recovery

| approach | what it buys | licence | effort |
|---|---|---|---|
| [RD-VIO](https://arxiv.org/html/2310.15072v3) (in XRSLAM, Apache-2.0 checked): pure-rotation detection by a third RANSAC (rotation-only model), "R-frames" kept as subframes with deferred triangulation and a ZUPT-like position regulariser, IMU-PARSAC two-stage matching | keeps tracking *through* a rotation burst instead of going Lost; ADVIO completeness 94-99 % vs VINS-Fusion 59-82 % | idea is free to re-implement; code Apache-2.0 | medium (rotation-only tracking mode in `stella_vio`: 2-4 agent-days) |
| multi-map merge (ORB-SLAM3 Atlas is GPL; stella has loop detector + Sim3 already) | after a re-init, the old map is re-used when revisited (today "keep newest, log old") | own code on BSD parts | medium (2-3 agent-days) |
| gyro-predicted search windows (done in `stella_vio` `gyro=1`) | large local gain on Outdoor-1 (first 60 s 2.09 -> 0.35 m) but did not prevent the complex burst loss: the camera turns into *unmapped* space | - | done, opt-in |

Conclusion: the complex-environment burst is not fixable by prediction alone; survival needs rotation-only frames
(no triangulation, bearing constraints) until parallax returns, which is what RD-VIO does.

### 1.3 GNSS-VIO on phones: tight vs loose

- Only permissive GNSS-VIO: [OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X) (BSD-3, checked; T-RO 2025 [paper](https://arxiv.org/abs/2510.04612)); it fuses **position fixes** (cartesian/geodetic), not raw observations, and needs Ceres, PCL, supereight2, GeographicLib. Nothing new and permissive appeared for raw-measurement tight coupling (GVINS, IC-GVINS, GICI-LIB: GPL; R2-GVIO: AGPL; InGVIO: no licence).
- Smartphone raw tight coupling exists as research: GVINS modified for 1 Hz Samsung A51 raw measurements ([Appl. Sci. 2025](https://doi.org/10.3390/app152312796)) cuts 3-D RMS by 65-84 % **when differential corrections are applied** relative to SPP; i.e. the gain is mostly from corrections (DGNSS/RTK via NTRIP), not from coupling. FGO GNSS/PDR fusion on phones ([arXiv 2212.14264](https://arxiv.org/pdf/2212.14264), [Sensors 2025 PDR+GNSS FGO](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12845897/)) is the closest published analogue of our loose smoother + gait prior.
- Android specifics: raw measurements via `GnssMeasurement` ([Android docs](https://developer.android.com/develop/sensors-and-location/sensors/gnss)); duty cycling must be disabled for carrier phase; Doppler is available on all raw-capable phones. Datasets with raw phone GNSS + IMU + GT exist only for cars and without a camera ([GSDC 2023-24](https://www.kaggle.com/competitions/smartphone-decimeter-2023/data), [Android raw GNSS datasets](https://www.researchgate.net/publication/346469092_Android_Raw_GNSS_Measurement_Datasets_for_Precise_Positioning)).
- Verdict: **semi-tight**. Keep the pose-graph smoother (positions, velocity, gait), add an own SPP + Doppler-velocity solver that turns raw Android observations into fixes + velocities + honest covariances (and consumes NTRIP corrections when available). Full pseudorange/carrier factors inside the graph only pay off in urban canyons with < 4 usable satellites; defer.

### 1.4 Permissive VIO/SLAM systems not yet benchmarked

| system | licence | what it is | why / why not |
|---|---|---|---|
| [MSCEqF](https://github.com/aau-cns/MSCEqF) | Apache-2.0 (checked) | multi-state-constraint equivariant filter, mono VIO, standalone C++ (Eigen, Lie++, OpenCV), static init + ZUPT | cheap to benchmark (2-4 h); filter-based, possibly a lighter port target than OKVIS2 for phones; accuracy on phones unknown |
| [sqrtVINS](https://github.com/rpng/sqrtVINS) (2025) | **LGPL-3** (checked) | square-root filter VINS, 2x faster, float32, 100 ms init | reference/binary only; a port would be an LGPL derivative |
| [OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X) | BSD-3 | already run (GNSS mode) | continue as drone reference |
| DROID-SLAM (BSD-3), MAC-VO (Apache-2.0), [Monado/Basalt forks] | permissive | learned / GPU | not dependency-free; skip |
| [AB-VINS](https://arxiv.org/pdf/2406.05969) | unknown | "simple" VINS (RPNG) | check code availability when needed; RPNG code tends to be GPL/LGPL |

Nothing lighter than XRSLAM with a permissive licence and phone evidence has appeared; XRSLAM's core is Apache-2.0
(the per-method extras in its README are optional modules we do not use) and it depends on Ceres 1.14 + Eigen.

### 1.5 Datasets

| dataset | licence | what it adds | status |
|---|---|---|---|
| [LaMAria](https://github.com/cvg/lamaria) (ICCV 2025, [paper](https://arxiv.org/pdf/2509.26639)) | data CC BY 4.0, code MIT (checked) | 70 km / 22 h of **pedestrian** egocentric walks (Aria glasses: cameras + IMU; Aria has a GNSS receiver, check whether the GPS stream is in the VRS) with **cm-accurate** survey GT; the right place to measure long-walk scale drift with a trustworthy reference (Mobile-GVIO GT is a LiDAR rig with an estimated clock offset) | fetch 2-3 sequences |
| [Monado SLAM datasets](https://huggingface.co/datasets/collabora/monado-slam-datasets) | CC BY 4.0 | VR headset VI, room scale | low relevance |
| [VIO-GNSS dataset](https://zenodo.org/records/8276054) | CC BY 4.0 | OAK-D stereo + u-blox F9P **raw** GNSS, RTK float/fix, 35 min, no independent GT | useful for the SPP/Doppler solver, not for accuracy claims |
| GVINS `complex_environment` (held locally) | research use | u-blox raw observations incl. Doppler (`gnss_comm/GnssObsMsg.dopp`), RTK GT | **first test bed for the Doppler-velocity factor**, no download needed |
| GSDC 2023-24 | Kaggle terms (check) | raw phone GNSS + IMU + GT, cars | SPP solver validation only |
| [MARS-LVIG](https://mars.hku.hk/dataset.html) | CC BY-NC-SA 4.0 | 21 drone sequences, 80-130 m AGL, raw F9P GNSS, RTK GT | Drive throttling stopped us; retry with the authors' folder or a mirror; NC |
| [AgriLiRa4D](https://arxiv.org/html/2512.01753v1), [UAVScenes](https://arxiv.org/pdf/2507.22412) (2025) | check | newer UAV multi-sensor sets | candidates for a 2nd/3rd drone benchmark round |
| own phone recordings | ours | the only way to get **camera + IMU + raw Android GNSS** with GT: [VIRec](https://github.com/A3DV/VIRec) (MIT; camera + IMU + GPS on one clock) plus Google GnssLogger (raw `GnssMeasurement`) on the same phone, GT from a carried u-blox ZED-F9P RTK rover (NTRIP) outdoors | needed for items 3-4 |

## 2. Strategic options

1. **Finish the OKVIS2 port (M4-M8)**. Benefit: the only permissive, dependency-free VI-SLAM with loop closure
   and proven drone behaviour (never lost tracking on 3 drone sequences). Cost: PLAN estimates M4 4-6 k LOC
   (hardest), M5 3.8 k, M6 3.2 k, M7b-d ~15 k upstream, M8 3 k; M1-M3 (~16 k upstream -> 3.3 k C) took about three
   agent-days with several agents, so M4-M8 bit-exact is 15-25 agent-days. Its BRISK front end is the wrong one for
   phones (s.8.3: thousands of RANSAC failures, scale collapse under every setting), so for phones only the
   estimator (M4-M6) is reusable, behind the already bit-exact stella ORB front end.
2. **Port XRSLAM/RD-VIO (Apache-2.0)** for phones. Benefit: best phone robustness measured (initialises on all 6
   sequences, no tracking loss, 0.84-1.55 m indoors), R-frame handling of pure rotation. Cost: a second full port
   (~10 k LOC core + Ceres sliding-window BA with marginalisation) and it still collapses scale on long walks, which
   no port fixes. Keep as the reference and the design source for item 6; do not port now.
3. **Build the phone path on `stella_vio` + loose fusion with external scale** (gait, GNSS Doppler, GNSS Sim3):
   the evidence (diagnosis: scale is an information problem; XRSLAM collapses too) says this is where the metric
   gain is, it is cheap, deterministic, and all pieces exist (stella ORB, multi-map re-init, gravity, `gnss_fusion`
   scale state and velocity factor). Local accuracy stays visual (stella Sim3 0.25-0.8 m indoors; XRSLAM is ~2x better
   locally, which a tight back end could recover later).
4. **Android raw GNSS**: the largest untapped sensor; blocked by data, not by code.
5. **Learned IO**: promising for GNSS-denied scale on drones (AirIO, BSD-3) and phones (RNIN-VIO, Apache-2.0)
   but weights/training data licences are unresolved; evaluate, do not build on.

Recommendation: 3 first (phones), 1 continued in parallel with a reduced-exactness M4 (drones), 4 as soon as own data
exists, 2 and 5 as evaluations only.

## 3. Ranked roadmap

| # | item | expected gain (evidence) | effort | risk | licence | depends on | first step |
|---|---|---|---|---|---|---|---|
| 1 | **Gait speed prior for scale** in `gnss_fusion` (+ per-map scale for mono): step detector on the accelerometer, cadence-linear / Weinberg step length, factor `s_i * |dL_i/dt| - v_gait` with sigma ~10 %, per-user L0 re-estimated from GNSS when available | metric trajectories indoors (today none) and no scale collapse outdoors: appendix A gives distance ratio 0.97-1.07 on all four Mobile-GVIO sequences with one calibration, 0.74-0.85 cross-user; vs VIO scale 0.00-0.5 on the long walks | 8-16 agent-h | low (gait model fails for running/escalators/standing: cadence gate + GNSS fallback) | own, MIT | none | add `gait.txt` (t, cadence, v_est) producer in `tools/` from appendix A, then `gf_add_speed()` + study `gf_studies.py gait` on o1/o2_stella, a20_orb3mono, indoor1/2 |
| 2 | **End-to-end C phone pipeline**: `stella_vio` (multi-map, gravity=1) -> per-map Sim3 state in `gnss_fusion` (+ item 1, + fixes) -> metric output; evaluate on the 6 phone sequences vs XRSLAM and GNSS-alone with the s.9/10 scorer | one deterministic permissive pipeline with full coverage; target: beat GNSS-alone causal on Outdoor-1/2, ADVIO-20 (5.7 / 14.7 / 12.0 m) and <= 1.5 m indoors | 16-30 agent-h | medium (map hand-over, scale per map, latency) | BSD-2/MPL/MIT | 1 | write `tools/phone_pipeline.py` that runs `sv_run` + `gf_run` and scores; freeze configs; one table |
| 3 | **Own phone dataset** with camera + IMU + raw Android GNSS + RTK GT (VIRec MIT + GnssLogger; u-blox F9P rover with NTRIP as GT; 3-4 walks of 5-10 min, 1 run, 1 indoor-outdoor) | unlocks items 4 and honest absolute metres (Mobile-GVIO GT has an estimated clock offset, iPhone fixes come from a second device) | 1-2 human-days + 8 agent-h (converters: VIRec -> EuRoC, GnssLogger -> RINEX/own csv, time alignment, GT) | medium (hardware, duty cycling, phone model) | own data | none | pick a raw-GNSS-capable Android phone, record a 2 min test with both apps, verify one clock (`elapsedRealtimeNanos`) |
| 4 | **Raw-GNSS C module**: broadcast-ephemeris satellite pos/vel, Klobuchar/Saastamoinen, weighted SPP + Doppler velocity + receiver clock drift, chi2 outlier rejection; emits fixes + velocities + covariances for `gnss_fusion` (semi-tight) | Doppler velocity at 1-2 dm/s = per-epoch scale observability for walking, running, vehicles; SPP 2-5 m with honest sigmas instead of the OS fix | 3-5 agent-days | medium (constellation details; validate against RTKLIB `rnx2rtkp` on GSDC) | own C99; RTKLIB BSD-2 as reference only | develop on GVINS raw bag (held), evaluate on 3 | extract `/ublox_driver/range_meas` + ephemeris from the GVINS bag to text; SPP solver vs the dataset's RTK GT |
| 5 | **OKVIS2 port M4 scope decision + M5/M6** (estimator) with the front end pluggable: for M4 target *iteration-trace parity within tolerance* (cost, step norm, radius per iteration) instead of bit-exactness of Ceres' dogleg/Schur; validate by ATE parity on MH_01 (deterministic ref 0.0193 SE3) | permissive drone VIO core; later the same estimator behind the stella ORB front end on phones | M4 faithful 3-5 agent-days (bit-exact 8-12), M5+M6 6-10 days | high (Ceres numerics, graph bookkeeping) | BSD-3 + MPL-2.0 | M4 in progress (other agent) | agree with the M4 owner on the tolerance criterion and the per-Solve snapshot format before M5 starts |
| 6 | **Rotation-only tracking mode + map merge in `stella_vio`** (RD-VIO R-frames: rotation-only RANSAC, bearing constraints, deferred triangulation; merge old map on relocalisation via existing BoW + Sim3) | survive the complex_environment 3.7 rad/s burst (today: Lost at 35 s, 2 maps) and reuse old maps; ORB-SLAM3 survives it with 90 % coverage via multi-map | 3-5 agent-days | medium | own code on BSD-2 parts; idea from Apache-2.0 RD-VIO (not copied) | none | prototype: tag frames whose matches fit a pure rotation (theta_max < 1 deg), hold pose from gyro, skip triangulation; measure Lost frames on complex |
| 7 | **Cheap strategic benchmarks** (decide, not build): (a) XRSLAM on the 3 drone sequences (builds exist, re-fetch images), (b) MSCEqF on EuRoC + 2 phone sequences, (c) `stella_vio` + item 1 vs XRSLAM on 2 LaMAria sequences (cm GT, 1+ km walks) | tells whether one permissive VIO could serve both platforms, and gives a trustworthy long-walk scale number | 6-12 agent-h | low | Apache-2.0 refs; CC BY 4.0 data | none | `run_xrslam.sh`-style driver for the INSANE EuRoC folders; LaMAria VRS -> ASL export with their tools |
| 8 | **Learned-IO evaluation** (offline): RNIN-VIO (Apache-2.0 code, phone weights) and TLIO (BSD code) displacement predictions on our six phone IMU streams vs GT scale; AirIO (BSD-3) on the drone IMU streams for GNSS-gap bridging | if the per-window displacement scale is within 10 % where gait fails (running, stairs), a 1-D ResNet in C99 (~2-3 days) is a drop-in factor for `gnss_fusion` | 1 agent-day eval | licence of weights/training data; distribution shift (TLIO = headset) | code BSD/Apache; weights unresolved | none | run the RNIN-VIO network on `external/gnss/rob/*/imu0` and score displacement ratio per 20 s bin like appendix A |

Deprioritised, with reasons: rolling-shutter modelling (measured effect <= 0.025 in scale); a full tightly coupled
pseudorange/carrier back end (gain comes from corrections, needs own data first); porting XRSLAM now (second
port, same scale failure); learned depth / feed-forward 3D for init (not dependency-free); retrying map-anchored
scale tricks inside the VIO (rejected in both `pure_c_plus` and the stella_vio init studies).

## 4. Key insights

1. **Phone scale is an information problem, not an estimator problem.** The diagnosis (S 15-30 mm per 4 s vs N 17-64 mm)
   predicts the measured collapse for every VIO; a gait model reads a different observable (step cadence/amplitude)
   and gives 2-7 % distance scale in the literature and 3-7 % in our own check on the same phone, 15-25 % cross-user
   without calibration. Combined with GNSS when outdoors (per-user step length re-estimated from fixes) this is the
   cheapest path to a metric, full-coverage phone trajectory, indoors included.
2. **Android raw GNSS is the biggest untapped sensor** (Doppler velocity 1-2 dm/s per epoch); iOS does not expose it
   and none of our datasets has it. Data collection, not code, is the blocker; semi-tight (own SPP/velocity solver
   feeding the existing smoother) captures most of the value without a Ceres-class back end.
3. **The permissive landscape has not changed**: no new permissive mono VIO with phone evidence beyond XRSLAM; sqrtVINS
   is LGPL-3; learned-IO code is BSD/Apache but weights/training data are NC or unstated. OKVIS2 remains the right
   drone core; its BRISK front end is the wrong phone front end, so keep the estimator pluggable and plan M4 for
   trace-level parity rather than bit-exact Ceres.
4. **Fast rotation needs rotation-only frames, not better prediction** (RD-VIO's R-frames); recovery needs map
   merging, which stella's loop machinery can provide.
5. **Validate long-walk scale on cm-accurate pedestrian GT** (LaMAria, CC BY 4.0) before claiming any phone number;
   our current absolute phone metres rest on a LiDAR-rig GT with an estimated clock offset.

## Appendix A: gait-speed check (2026-10-03, numpy only, ~5 s CPU)

Question: does a step-cadence pedestrian model from the phone accelerometer give metric walking distance within
~10 % on the sequences where VIO scale collapses? Data: `external/gnss/rob/<seq>/imu0/data.csv` + `gt.tum` via
`tools/phone_diag/common.py` (same clock offsets as the diagnosis). Method: |f| minus 1 s moving mean, 0.12 s
smoothing, peak picking (>= 0.3 s apart, > 0.35 std); per 20 s bin with GT mean speed > 0.5 m/s: GT path length
(0.5 s smoothed) vs (a) `n_steps * L0`, (b) Weinberg `K * sum (amax-amin)^(1/4)`, (c) cadence-linear
`v = a + b * cadence`; L0, K, (a, b) fitted on **outdoor1 only** and applied unchanged.

```
calibrated on outdoor1: L0=0.705 m/step  K=0.464  v=a+b*cad: a=-0.874 b=1.193  (bins 19)
seq        bins  gt m/s  cad Hz | fixedL0 med [IQR] tot | weinberg med [IQR] tot | cadlin med [IQR] tot
outdoor1     19    1.26    1.79 | 0.99 [0.97,1.00] 1.00 | 0.99 [0.98,1.01] 1.00 | 1.00 [0.98,1.02] 1.00
outdoor2     22    1.26    1.75 | 0.98 [0.93,1.01] 0.97 | 0.99 [0.98,1.00] 0.99 | 0.95 [0.93,0.97] 0.96
indoor1       5    1.22    1.78 | 1.02 [1.02,1.03] 1.03 | 0.96 [0.96,0.98] 0.97 | 1.02 [1.00,1.04] 1.02
indoor2       4    0.98    1.60 | 1.16 [1.13,1.18] 1.15 | 1.08 [1.06,1.09] 1.07 | 1.06 [1.04,1.07] 1.05
advio15       1    0.53    1.60 | 2.14 [2.14,2.14] 2.14 | 1.37 [1.37,1.37] 1.37 | 1.96 [1.96,1.96] 1.96
advio20      15    1.53    1.83 | 0.83 [0.79,0.89] 0.83 | 0.74 [0.68,0.80] 0.74 | 0.86 [0.81,0.89] 0.85
```

Reading: same phone/user (Mobile-GVIO) 0.96-1.07 total distance with any of the three models; a different user and
phone walking faster (ADVIO-20, 1.53 m/s) is 15-26 % under with the outdoor1 calibration (a per-user L0 from a few
GNSS epochs fixes that); ADVIO-15 is a 0.5 m/s office shuffle (one bin, not a walking regime: gate on cadence and
speed). Compare the VIO scale on the same long walks: XRSLAM / OKVIS2-X 0.00-0.5. Limits: 20 s bins, GT path
length includes GT jitter (upper bound on the truth), six sequences, two users; no running or stairs.

Script (kept out of the tree; re-create under `tools/phone_diag/gait_speed.py` when item 1 starts):

```python
#!/usr/bin/env python3
import sys, numpy as np
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/phone_diag')
from common import load, gt_window, gt_continuous_mask
def movavg(x, n):
    n = max(1, int(n)); c = np.cumsum(np.insert(x, 0, 0.0)); y = (c[n:] - c[:-n]) / n
    pad = len(x) - len(y); return np.concatenate([np.full(pad // 2, y[0]), y, np.full(pad - pad // 2, y[-1])])
def steps(d):
    ti, f = d['ti'], d['f']; fs = 1.0 / np.median(np.diff(ti))
    a = np.linalg.norm(f, axis=1); a = a - movavg(a, fs * 1.0); a = movavg(a, fs * 0.12); thr = 0.35 * np.std(a)
    pk = np.where((a[1:-1] > a[:-2]) & (a[1:-1] >= a[2:]) & (a[1:-1] > thr))[0] + 1
    keep = []; last = -1e9
    for p in pk:
        if ti[p] - last >= 0.3: keep.append(p); last = ti[p]
    keep = np.array(keep); amp = np.array([np.ptp(a[max(0, p - int(fs * 0.3)):p + int(fs * 0.3)]) for p in keep])
    return ti[keep], amp
def gt_path(d, t0, t1):
    tg, p = d['tg'], d['p']; m = (tg >= t0) & (tg < t1) & gt_continuous_mask(tg)
    if m.sum() < 5: return np.nan
    q = p[m]; rate = 1.0 / np.median(np.diff(tg[m])); q = np.stack([movavg(q[:, k], rate * 0.5) for k in range(3)], 1)
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()) * ((t1 - t0) / (tg[m][-1] - tg[m][0]))
def bins(d, B=20.0):
    t0, t1 = gt_window(d); st, amp = steps(d); out = []
    for b0 in np.arange(t0, t1 - B, B):
        m = (st >= b0) & (st < b0 + B); n = int(m.sum()); L = gt_path(d, b0, b0 + B)
        if np.isnan(L) or L < 0.5 * B: continue
        out.append(dict(n=n, cad=n / B, w4=float(np.sum(amp[m] ** 0.25)) if n else 0.0, gt=L))
    return out
SEQS = ['outdoor1', 'outdoor2', 'indoor1', 'indoor2', 'advio15', 'advio20']
data = {s: bins(load(s)) for s in SEQS}; cal = data['outdoor1']
L0 = sum(b['gt'] for b in cal) / sum(b['n'] for b in cal); K = sum(b['gt'] for b in cal) / sum(b['w4'] for b in cal)
A = np.array([[1.0, b['cad']] for b in cal]); y = np.array([b['gt'] / 20.0 for b in cal]); ab = np.linalg.lstsq(A, y, rcond=None)[0]
print(f'calibrated on outdoor1: L0={L0:.3f} K={K:.3f} a={ab[0]:.3f} b={ab[1]:.3f} (bins {len(cal)})')
for s in SEQS:
    bs = data[s]; gt = np.array([b['gt'] for b in bs]); n = np.array([b['n'] for b in bs]); w4 = np.array([b['w4'] for b in bs]); cad = np.array([b['cad'] for b in bs])
    for name, e in (('fixedL0', n * L0), ('weinberg', K * w4), ('cadlin', (ab[0] + ab[1] * cad) * 20.0)):
        r = e / gt; print(f'{s:10s} {len(bs):3d} {name:9s} med {np.median(r):.2f} IQR [{np.percentile(r,25):.2f},{np.percentile(r,75):.2f}] tot {e.sum()/gt.sum():.2f}')
```

## Sources

Project docs: `README.md`, `AGENTS.md`, `docs/vio_candidates_20261001.md`, `docs/gnss_vio_benchmark_20261001.md`
(s.8-11), `docs/drone_benchmark_20261002.md`, `docs/phone_scale_diagnosis_20261003.md`, `stella_vio/{README,RESULTS,IMU_PLAN}.md`,
`okvis_port/{PLAN,HANDOVER}.md`, `gnss_fusion/README.md`, `stella_port/HANDOVER.md`.

External (licences read from the linked repositories where marked "checked" above):
[TLIO](https://github.com/CathIAS/TLIO), [RoNIN](https://github.com/Sachini/ronin), [RNIN-VIO](https://github.com/zju3dv/rnin-vio),
[AirIO](https://github.com/Air-IO/Air-IO) / [paper](https://arxiv.org/html/2501.15659v1), [EqNIO](https://github.com/RoyinaJayanth/EqNIO),
[XRSLAM](https://github.com/openxrlab/xrslam) / [RD-VIO](https://arxiv.org/html/2310.15072v3), [OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X) / [paper](https://arxiv.org/abs/2510.04612),
[MSCEqF](https://github.com/aau-cns/MSCEqF), [sqrtVINS](https://github.com/rpng/sqrtVINS) / [paper](https://arxiv.org/abs/2510.10346), [RTKLIB](https://github.com/tomojitakasu/RTKLIB),
[PS-VINS](https://ieeexplore.ieee.org/document/10400404/), [adaptive step length mono VO](https://doi.org/10.3390/s19040953),
[Android Doppler velocity evaluation](https://d-nb.info/1256353183/34), [velocity-aided smartphone positioning](https://www.sciencedirect.com/science/article/abs/pii/S0273117719300122),
[smartphone GNSS in GVINS](https://doi.org/10.3390/app152312796), [GNSS/PDR FGO](https://arxiv.org/pdf/2212.14264), [PDR+GNSS FGO walking dynamics](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12845897/),
[Android raw GNSS](https://developer.android.com/develop/sensors-and-location/sensors/gnss), [GSDC 2023-24](https://www.kaggle.com/competitions/smartphone-decimeter-2023/data),
[LaMAria](https://github.com/cvg/lamaria) / [paper](https://arxiv.org/pdf/2509.26639), [Monado SLAM datasets](https://huggingface.co/datasets/collabora/monado-slam-datasets),
[VIO-GNSS dataset](https://zenodo.org/records/8276054), [Mobile-GVIO](https://zenodo.org/records/20525157), [MARS-LVIG](https://mars.hku.hk/dataset.html),
[AgriLiRa4D](https://arxiv.org/html/2512.01753v1), [UAVScenes](https://arxiv.org/pdf/2507.22412), [VIRec](https://github.com/A3DV/VIRec),
[Merrill et al. depth-aided init](https://udel.edu/~nmerrill/pubs/Merrill2023RSS.pdf), [XR-VIO](https://arxiv.org/abs/2502.01297), [line/VP init](https://arxiv.org/html/2609.21186),
[Ctrl-VIO](https://github.com/APRIL-ZJU/Ctrl-VIO), [learned inertial positioning survey](https://arxiv.org/html/2303.03757v3), [AB-VINS](https://arxiv.org/pdf/2406.05969).
