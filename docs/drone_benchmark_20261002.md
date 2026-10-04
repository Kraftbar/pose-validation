# Outdoor drone benchmark: camera + IMU (+ GNSS), 2026-10-02

Benchmark only, nothing ported or committed. Scripts: `tools/drone_harness/` (fetchers, converters, `calib.py` config writer, `run_drone.sh`, `run_loose.sh`, `score.py`).
Results: `runs/drone_compare/table.md` and `runs/drone_compare/<seq>/<system>/{trajectory.tum,metrics.json,run.json,log.txt}`.
Builds/data: `external/drone/` (gitignored; images deleted at the end). Scoring reuses `benchmark.umeyama_alignment` via `tools/gnss_harness/gnss_eval.py`.

## 1. Candidate datasets (checked 2026-10-02)

| Dataset | Platform / sensors | GT | Licence | Size | Verdict |
|---|---|---|---|---|---|
| **INSANE** (Univ. Klagenfurt) | MAV; nav cam (mono 2056x1542 global shutter, down-looking), RealSense T265 fisheye stereo, 3 IMUs (PX4 200 Hz), PX4 consumer GNSS 5 Hz, dual RTK | dual-RTK 80 Hz 6DoF, cm-level; outdoor + Negev "Mars analog" flights | BSD-2 plus "no selling" condition (non-commercial, cite) | per-sequence zips: sensors 8-27 MB, nav cam 2.4-9 GB, stereo 2.2-4 GB; ranged GET works (server only about 0.5 MB/s per connection, so 32 parallel connections) | **used** (outdoor_1 head, mars_14) |
| **UZH-FPV** | FPV racing quad; Snapdragon stereo fisheye 640x480 30 Hz + IMU 500 Hz (also DAVIS) | Leica laser tracker, only for the flight part | CC BY-NC-SA 3.0 | 0.25-2.2 GB per Snapdragon sequence | **used** (outdoor_forward_5): fast (mean 8, peak 20 m/s), no GNSS |
| **MARS-LVIG** (HKU MaRS) | DJI M300; Hikvision 5 MP cam 10 Hz, Livox Avia + IMU, u-blox F9P raw GNSS, DJI RTK | RTK position only (3-DoF, 5 Hz) | CC BY-NC-SA 4.0 | 23 bags, 9.8-20+ GB each (HKairport_GNSS03: 9.76 GB, 9 m/s) on Google Drive | **partially acquired, not run** (section 4) |
| Zurich Urban MAV | MAV 5-15 m AGL, GoPro rolling shutter 1080p 30 Hz, on-board GPS, PX4 IMU (raw 10 Hz, fused 50 Hz) | photogrammetric camera poses (1 in 30 frames) | "no restriction" (incl. commercial) | 28 GB (range requests work) | **not run**: no camera-IMU extrinsics, low-rate IMU, rolling shutter; would need hand-eye calibration first |
| MUN-FRL | DJI M600 + Bell 412; mono 20 Hz, Xsens IMU 400 Hz, VLP-16, RTK/PPK GNSS | PPK | CC BY 4.0 | 27-90 GB per bag (Google Drive) | too big for the 12 GB budget |
| NTU VIRAL | M600 hexacopter; stereo, 2 lidars, IMU, UWB | Leica MS60 laser tracker (no GNSS, mostly indoor/campus) | CC BY-NC-SA 4.0 | 4-9.4 GB per sequence | no GNSS, not run |
| Blackbird | indoor mocap, stereo + IMU, up to 7 m/s | mocap | not stated on the pages I could read | about 4.8 TB total | indoor, skipped |
| ALTO | helicopter, down camera 20 Hz, GPS-INS | GPS-INS | not found | 150-260 km trajectories | place recognition focus, skipped |
| ctu-mrs coop_uav, "Low-altitude UAV VI dataset" | RTK GT, rosbags | | not checked | | not evaluated |

Drive problem (MARS-LVIG, MUN-FRL): anonymous range downloads from Google Drive return the "Quota exceeded" page for about 50-100% of requests after about 3 GB; the old per-file IDs printed on the MARS page are all dead (the live files are in the "Dataset_ROS_bags" folder).

## 2. Sequences used

| id | sequence | length | motion | GNSS | reference |
|---|---|---|---|---|---|
| `of5` | UZH-FPV outdoor_forward_5 Snapdragon (stereo + IMU) | 91 s of data (47 s on the ground), 17.6 s reference window | 141 m, mean 8, peak 20 m/s | none | Leica, IMU pose |
| `m14` | INSANE mars_14 (Negev, nav cam + PX4 IMU) | 181 s, 2658 frames at 15 Hz | 286 m, mean 1.6, peak 10.5 m/s, 3-19 m AGL | PX4 receiver (consumer, reported sigma 3-9 m) | dual-RTK midpoint at 7 Hz, ENU |
| `o1` | INSANE outdoor_1, first 100 s (Klagenfurt field) | 99 s, 1988 frames at 20 Hz | first 55 s stationary on the ground, then up to 7 m/s, up to 24 m height | PX4 receiver | dual-RTK midpoint |

Conversions: nav cam halved to 1028x771 grey, IMU = PX4 IMU, calibration = the dataset's Kalibr `nav_cam_radtan` (mars calibration for m14, klu1 for o1: the klu1/klu2 choice for outdoor_1 is an assumption). Time bases: the dataset's `px4_*.csv` and `ground_truth/*_revised.csv` agree (px4 GNSS vs GT lag search is flat around 0); no camera-IMU offset was estimated (none applied). FPV: Kalibr calibration as shipped in OpenVINS' `uzhfpv_outdoor` config (it is the dataset's result), camera time shift -8 ms applied.
Scoring: vehicle-centre RTK reference vs IMU pose with the 6 cm lever arm; GNSS fixes are converted to the same ENU frame (so "no-align" error is geo-referenced). The RTK-midpoint reference is fit-consistent with the 80 Hz GT to 5 cm (o1) and 25 cm (m14). IMU noise values are generic (not tuned per sequence). Single runs, one repeat where a result looked like an outlier. The machine had load average 30-40 from other agents, so RTF and wall times are pessimistic (OKVIS2 stereo on FPV took 10x real time; the isolated earlier EuRoC figure was 1.6x).

## 3. Results (ATE RMSE in m; SE3 = metric alignment, Sim3 = with scale; "geo" = no alignment, GNSS systems only)

### 3.1 `of5` UZH-FPV outdoor_forward_5, aggressive, no GNSS (141 m in 17.6 s)

| system | SE3 | Sim3 | scale | coverage | losses | RTF (loaded) |
|---|---|---|---|---|---|---|
| ORB-SLAM3 stereo-inertial (GPL, ref) | 0.87 | 0.86 | 1.00 | 92% | 136 failed tracks | 2.8 |
| ORB-SLAM3 mono-inertial (GPL, ref), run 1 / repeat | 16.8 / 0.86 | 2.74 / 0.79 | 2.39 / 1.01 | 91% / 64% | 167 / 328 | 2.2 / 1.1 |
| OKVIS2 mono | 0.53 | 0.41 | 0.99 | 100% | - | 9.5 |
| OKVIS2 stereo | 1.20 | 0.20 | 1.04 | 100% | - | 10.6 |
| Basalt stereo | 2.30 | 0.40 | 1.09 | 100% | 0 | 0.60 |
| stella_vslam mono (camera only) | 7.3 | 2.6 | 4.06 | 52% | lost, relocalised | 0.8 |

### 3.2 `m14` INSANE mars_14, PX4 GNSS + RTK reference (286 m)

| system | SE3 | Sim3 | scale | geo (no align) | coverage | RTF (loaded) |
|---|---|---|---|---|---|---|
| OKVIS2-X mono + GNSS | **1.39** | 1.33 | 1.01 | 3.98 | 100% | 3.6 |
| own loose fusion on OKVIS2 mono + GNSS | 1.46 | 1.44 | 1.01 | 4.15 | 100% | +1 s offline |
| PX4 GNSS fixes alone | 1.37 | 1.37 | 1.00 | 4.08 | - | - |
| OKVIS2 mono (VIO, LC + final BA) | 5.06 | 1.45 | 1.13 | - | 100% | 2.1 |
| OKVIS2-X mono, GNSS off | 6.35 | 5.76 | 0.94 | - | 100% | 3.1 |
| ORB-SLAM3 mono-inertial (GPL, ref) | crash (segfault in IMU init, 2 of 2 runs) | | | | 0% | |
| stella_vslam mono | 0.34 | 0.17 | 0.52 | - | 7% | 1.3 |

### 3.3 `o1` INSANE outdoor_1, first 100 s (stationary 55 s, then flight)

| system | SE3 | Sim3 | scale | geo (no align) | coverage |
|---|---|---|---|---|---|
| OKVIS2-X mono + GNSS, run 1 / repeat | 0.47 / 0.68 | 0.31 / 0.31 | 1.01 / 1.02 | **47.9 / 47.7** | 100% |
| OKVIS2-X mono, GNSS off | 0.97 | 0.18 | 1.04 | - | 100% |
| OKVIS2 mono | 2.55 | 1.58 | 1.09 | - | 100% |
| own loose fusion on OKVIS2 mono + GNSS | 1.47 | 1.46 | 1.00 | 10.45 | 100% |
| PX4 GNSS fixes alone | 2.20 | 1.39 | 0.94 | 10.7 | - |
| ORB-SLAM3 mono-inertial (GPL, ref) | 9.85 | 0.35 | 1.58 | - | 41% (20 failed tracks) |
| stella_vslam mono | 10.9 | 0.33 | 1.68 | - | 41% |

## 4. Failure diagnosis

- **MARS-LVIG**: acquired the first 2.7 GB of HKairport_GNSS03 (117 s) and converted it (own bag reader, intrinsics calibrated here from 9 chessboard views: fx 1445 px at 2448x2048, rms 0.28 px; extrinsics from the dataset's CAD yaml and the Livox manual). That slice is takeoff, climb to 88 m and hover (peak 4 m/s), so it says nothing about fast flight; the 9 m/s part lives later in the bag and Drive throttled the rest. No system was run on it. Camera timestamps need a separately measured offset to the Livox/GNSS clock (camera header = bag time + 0.13 s, IMU/GNSS header = bag time + 0.31 s, RTK header = bag time); `tools/drone_harness/mars_bag_to_euroc.py` and `drive_bag_slice.py` (index-based chunk slicing) are kept for a later retry.
- **o1 OKVIS2-X geo error 48 m with SE3 error 0.5 m (reproduced twice)**: during the 55 s stationary start the global frame is pinned near the origin (error 0.6 m); once the drone moves the global trajectory rotates away (11, 55, 71 m). The local trajectory is fine, so this is the GNSS-VIO yaw/global alignment failing when yaw is unobservable (no motion) before the first long baseline, with consumer-grade GNSS noise. The loose smoother avoided it (10.5 m, essentially the receiver bias) because it estimates yaw as a random walk and starts after motion.
- **GNSS error floor**: the raw PX4 fixes are 4.1 m (m14) and 10.7 m (o1) away from the RTK reference in absolute terms (receiver bias, 5 m vertical), but 1.4-2.2 m after SE3 alignment. No fusion removes the bias; geo errors of fused outputs sit at that floor (4.0-4.2 m, 10.5 m).
- **m14 camera-only**: stella tracks 7% of frames (down-looking camera over Negev desert, low texture, 3-19 m AGL, fast turns); ORB-SLAM3 mono-inertial segfaults in IMU initialisation on this sequence (2 of 2 runs, same crash class as on EuRoC V2_02 earlier).
- **o1 camera-only**: stella and ORB-SLAM3 initialise only after the drone starts moving (41% coverage); their SE3 error (10-11 m) is scale (1.6-1.7), Sim3 is 0.33-0.35.
- **ORB-SLAM3 mono-inertial on FPV is non-deterministic**: scale 2.39 (16.8 m) in run 1, 1.01 (0.86 m) in the repeat, with 167-328 failed local-map tracks in both; stereo-inertial also shows 136 failures. Trust it only as a reference upper bound.
- **FPV**: OKVIS2 survives the 20 m/s flight without any loss (the only system without lost-track events); Basalt is 4x worse in SE3 but 0.6 RTF under load. OKVIS2 mono (0.53) beat stereo (1.20) in SE3 on this one run, but stereo is better in Sim3 (0.20 vs 0.41), so the SE3 difference is scale drift of a single 17 s window, not a robust ranking.

## 5. Does GNSS help on drones?

Measured on two real consumer-GNSS flights (reported sigma 3-9 m, actual 1.4-2.2 m after alignment):

| | m14 SE3 | o1 SE3 | m14 geo | o1 geo |
|---|---|---|---|---|
| VIO only (OKVIS2-X, GNSS off) | 6.35 | 0.97 | not geo-referenced | |
| OKVIS2 mono VIO | 5.06 | 2.55 | | |
| OKVIS2-X + GNSS | 1.39 | 0.47 | 3.98 | 47.9 (yaw failure) |
| own loose fusion + GNSS | 1.46 | 1.47 | 4.15 | 10.45 |
| GNSS alone | 1.37 | 2.20 | 4.08 | 10.7 |

- GNSS cuts mono-inertial drift by about 4x on m14 (6.3 to 1.4 m) and about 2x on o1 (0.97 to 0.47), and it gives a geo-referenced frame. But on m14 the fused result (1.39) is no better than the raw GNSS fixes (1.37): at 286 m and 3-19 m height, this VIO's drift (about 2% of path) is comparable to consumer-GNSS noise, so GNSS dominates. The benefit is the bounded error over long flights and absolute frame, not centimetre accuracy.
- The loose smoother on top of plain OKVIS2 reproduces tightly-coupled OKVIS2-X on m14 (1.46 vs 1.39 SE3, 4.15 vs 3.98 geo) and does not suffer the o1 yaw failure; it is worse on o1 SE3 (1.47 vs 0.47) because the GNSS-only yaw/scale model is cruder when the first 55 s are stationary.
- Neither helps with the absolute bias of the receiver (4-11 m): that needs RTK/PPK corrections.
- FPV-class flight has no GNSS in the dataset, so no number there.

## 6. Which system for drones

On these three sequences, OKVIS2 (BSD-3) is the only system that never lost tracking and worked mono and stereo on all three; mono OKVIS2 gives 0.53 m SE3 on the 20 m/s FPV flight, 2.6-5.1 m (1.5 Sim3) on the INSANE flights without GNSS. OKVIS2-X adds the GNSS gain above but its global-frame yaw init is fragile after a stationary start (48 m). Basalt (BSD-3) is stereo only and an order of magnitude cheaper (RTF 0.6 loaded) but 2.3 m on FPV. ORB-SLAM3 (GPL) is the accuracy reference when it works (0.87 stereo-inertial on FPV) but crashed or restarted maps on 2 of 3 mono-inertial cases; stella (camera only) loses a down-looking drone camera over low texture (7-52% coverage) and has no metric scale.
Recommendation: OKVIS2/OKVIS2-X as the permissive drone core, with the loose GNSS smoother as the guard against the yaw-init failure.

Caveats: three sequences, single runs (a repeat only for outliers), conversions with generic IMU noise, short o1 slice (100 s), no MARS-LVIG numbers, FPV reference window of 17.6 s. INSANE licence (BSD-2 with a no-selling condition) and CC BY-NC-SA datasets: no data is committed.

## 7. Reproduce

```bash
source external/gnss/venv/bin/activate
python3 tools/drone_harness/insane_to_euroc.py mars_14 external/drone/ins_m14      # needs external/drone/insane/mars_14_sensors (unzipped *_sensors.zip) first
python3 tools/drone_harness/calib.py insane_mars external/drone/ins_m14 external/drone/ins_m14_cfg
tools/drone_harness/run_drone.sh okvis2x_mono_gnss $PWD/external/drone/ins_m14 $PWD/external/drone/ins_m14_cfg $PWD/runs/drone_compare/m14
tools/drone_harness/run_loose.sh $PWD/runs/drone_compare/m14 okvis2_mono $PWD/external/drone/ins_m14
python3 tools/drone_harness/score.py m14 external/drone/ins_m14
```
FPV: unzip `outdoor_forward_5_snapdragon_with_gt.zip` into `external/drone/fpv/of5`, `fpv_to_euroc.py`, `calib.py fpv`. ORB-SLAM3 runs through the existing `external/vio` build (GPL glue stays in `external/`; `tools/drone_harness/gpl_glue/` is empty because the driver is the stock EuRoC example).
