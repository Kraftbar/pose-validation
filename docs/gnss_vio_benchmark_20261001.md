# GNSS + visual-inertial odometry: what GNSS buys, measured (2026-10-01)

Research and benchmarking only, nothing ported or committed. Target product: drones + phones (mono camera + IMU + GNSS).
Follow-up to `docs/vio_candidates_20261001.md`. Artifacts: `runs/gnss_compare/` (`table.md`, `table.json`, per-run `trajectory.tum` + `metrics.json`,
`raw/` = run logs/configs), helpers `tools/gnss_harness/` (GPL-driving glue in `gpl_glue/`), own smoother `tools/gnss_loose_fusion.py`.
Builds/data live in `external/gnss/` (gitignored, 5.4 GB at the end; peak was about 13 GB, all raw bags and images deleted).

## 1. Candidates, licences, what ran

| System | Licence | Coupling | Ran? | Notes |
|---|---|---|---|---|
| **OKVIS2-X** (ethz-mrl) | **BSD-3** (supereight2 submodule MPL-2.0) | tight, GNSS position factors in the VI-SLAM graph, online GNSS-extrinsic init | **yes**, mono | builds without PCL: the only PCL use is two debug PLY dumps, replaced by a 20-line stub header (`tools/gnss_harness/pcl_stub/`, CMake edit in the external copy). Source built in about 15 min wall. TBB/GeographicLib/Boost came from the existing `external/vio/deps` |
| **GVINS** (HKUST) | GPL-3 | tight, raw pseudorange + Doppler | **yes** | needs ROS1: no-root robostack env (micromamba, `-c conda-forge -c robostack-staging`, noetic) in `external/gnss/mamba`; compile fixes: `-std=c++14`, OpenCV-4 legacy constants via a force-included compat header, `empy==3.3.4`. Bag is replayed with `rosbag play` in real time |
| **IC-GVINS** (WHU) | GPL-3 | INS-centric, tight with RTK-grade position fixes | **built, did not run to completion** | see section 4 |
| **GICI-LIB** (gici-open) | GPL-3 | tight/loose, SPP/RTK/PPP | **not run** | cloned and read only. Its dataset is on OneDrive/BaiduCloud (web-gated, 403 on the anonymous share API); the library eats its own raw formats (RINEX + `image-pack` + imu text, or `gici_ros` raw-GNSS messages). The GVINS bag only has `gnss_comm` messages, so a faithful run needs a bag->RINEX/gici conversion; the SRR (GNSS-solution + IMU + camera) path additionally needs a position+velocity solution stream. Estimated more than the 60 min box |
| **R2-GVIO** (JiangboSong251) | AGPL-3 (LICENSE file) | tight, VINS-Fusion based | **no code exists** | repo contains only README + LICENSE; its SYSU-Campus-GVI dataset is Baidu-only |
| InGVIO | no licence file | | not tried | unusable as a source (see previous note) |

Datasets (licences read from the repos/records on 2026-10-01):

| Dataset | Content | Licence | Access | Used |
|---|---|---|---|---|
| **GVINS-Dataset** (HKUST) `complex_environment` / `sports_field` / `urban_driving` | VI-Sensor (752x480 stereo, 20 Hz, ADIS16448 IMU 200 Hz), u-blox F9P raw GNSS 10 Hz + RTK solution as reference | **CC-BY-NC-SA-4.0** | Hugging Face (the old OneDrive link is dead); 17-36 GB bags, range requests work | **yes**, first 436 s of `complex_environment` (4.6 GB head fetched by HTTP range, nothing else downloaded) |
| **Mobile-GVIO** (SZU, Zenodo 20525157) | Honor phone mono camera 1280x720 30 Hz + IMU 100 Hz, iPhone 11 Pro Max GNSS fixes 1 Hz, GT from a rigid LiDAR-IMU rig (Fast-LIO2, in a local frame, not geo-referenced) | **CC-BY-4.0** | Zenodo, 5.8-13 GB zips (deflate), streamed and converted on the fly | **yes**, `Outdoor-1` (394 s, 501 m) |
| IC-GVINS dataset (KAIST urban38/39, own campus/building) | camera + IMU + RTK GNSS bags | none stated (code GPL-3) | BaiduCloud only | no |
| GICI-dataset | 12 sequences, camera + IMU + raw GNSS | none stated (repo GPL-3 file) | OneDrive/Baidu only | no |
| UrbanNav, KAIST Complex Urban | | | not attempted (GICI ships a `ros_urbannav` config; sizes tens of GB) | no |

The GVINS `urban_driving` (vehicle) sequence was not run: time went into getting two systems and a phone dataset to work. So there is no vehicle result;
`complex_environment` is a handheld/cart rig walking about 640 m in an urban campus with trees and buildings (RTK float/no-carrier for 20% of epochs,
worst 240-330 s).

## 2. Method

- GT: the receiver's own RTK fix (carrier-phase fixed for 81% of epochs, `h_acc` median 1.4 cm). Only GT epochs with `h_acc <= 0.1 m` are scored. Estimates are scored at the
  GNSS antenna (pose + R * r_SA, lever arm from OKVIS2-X's shipped GVINS config; GVINS has no lever arm so its body point is scored).
- Time base: sensor stamps are UTC; GNSS stamps are GPS time = UTC + 18 s plus a 26.2 ms (drifting to 27.4 ms) local-clock offset found in GVINS's own PPS sync (`gnss_result.csv`).
  Applied to GT and to the GNSS files.
- ATE: SE3 alignment via `benchmark.umeyama_alignment(with_scale=False)`; Sim3 scale in `metrics.json`. **No-align** = RMSE of the geo-referenced output (ENU frame of the GNSS) with no
  alignment at all, only reported for systems that emit a global frame (OKVIS2-X `*-global-final_trajectory.csv`, GVINS `fused_lla`).
- The GT (RTK) is also what the "RTK" GNSS variant feeds in, so the RTK rows are an optimistic upper bound (circular by construction). To avoid that, GNSS-grade variants were
  simulated from the RTK epochs: `sim` = AR(1) correlated noise (tau 30 s) with 1.5 m horizontal / 3 m vertical sigma plus white noise, 1 Hz (typical SPP / phone-grade),
  and 120 s blackouts (t = 100-220 s). GVINS uses its own raw pseudoranges (real SPP-grade input, independent of GT).
- GNSS off = same system, GNSS block removed (OKVIS2-X `gps_parameters` deleted; GVINS `gnss_enable: 0`). IC-GVINS has no GNSS-off mode (needs GNSS to initialise).
- All runs single, non-deterministic threading, loop closure off (as in the OKVIS2-X GVINS config), final (non-causal, BA'd) trajectory.

## 3. Results

### 3.1 `complex_environment` (436 s, 8715 frames, handheld rig, 16-thread Ryzen 7 3700X)

| system | GNSS input | ATE SE3 (m) | no-align / geo-ref error (m) | 100-220 s window | after (220-260 s) | float epochs 240-330 s | pose coverage | speed |
|---|---|---|---|---|---|---|---|---|
| OKVIS2-X mono | none (VIO) | 8.02 | n/a | 5.18 | 1.22 | 7.41 | 100% | RTF 1.41 (alone, includes final BA) |
| OKVIS2-X mono | RTK 10 Hz | **0.161** | **0.165** | 0.04 | 0.25 | 0.36 | 100% | RTF 1.48 |
| OKVIS2-X mono | RTK, blackout 100-220 s | 1.69 | 1.91 | 3.46 | 0.33 | 0.37 | 100% | n/m |
| OKVIS2-X mono | sim SPP-grade 1 Hz | 9.61 | 11.55 | 4.37 | 2.90 | 3.25 | 100% | n/m |
| OKVIS2-X mono | sim SPP-grade + blackout | 9.72 | 13.24 | 11.08 | 3.18 | 2.97 | 100% | n/m |
| GVINS (mono, left cam) | none (VIO) | 5.45 | n/a | 3.97 | 7.61 | 5.86 | 50% (10 Hz of 20 Hz) | real-time paced; 1.1 CPU-s per 4.4 s of data (about 0.25 core) |
| GVINS | raw pseudorange + Doppler 10 Hz | 1.27 | 3.46 (median 3.2) | 2.83 | 3.10 | 3.58 | 49%, span 98% | same |
| own loose fusion on OKVIS2 VIO (batch) | RTK 10 Hz | **0.079** | 0.087 | 0.07 | 0.11 | 0.15 | 100% | 2 s offline |
| own loose fusion (causal, 30 s window) | RTK | 0.092 | 0.098 | 0.08 | 0.16 | 0.18 | 100% | 8 s |
| own loose, batch / causal | RTK blackout | 0.257 / 0.650 | 0.27 / 0.73 | 0.47 / 1.32 | 0.11 / 0.20 | 0.15 / 0.18 | 100% | |
| own loose, batch / causal | sim SPP | 2.04 / 2.36 | 4.07 / 4.24 | 5.09 / 5.51 | 4.48 / 4.70 | 3.57 / 4.78 | 100% | |
| own loose, batch / causal | sim SPP + blackout | 1.89 / 2.39 | 4.28 / 4.48 | 5.54 / 5.86 | 4.82 / 5.46 | 3.67 / 5.06 | 100% | |

Full table: `runs/gnss_compare/table.md`. "n/m" = timings were taken with up to 3 other OKVIS jobs sharing the machine (wall 900-1270 s), so only the two clean runs are quoted.
GVINS publishes at 10 Hz by design (feature tracker halves the 20 Hz stream), so "coverage" is 50% of frames but 98% of the time span.
The GVINS VIO baseline here is the same code with the GNSS block off, it is the mono left-camera VIO (MEI model); OKVIS2-X mono uses the right camera (its shipped config).
Both are mono + low-grade IMU and drift 1-1.5% of path (5-8 m over 640 m).

### 3.2 `Mobile-GVIO Outdoor-1` (phone, 394 s, 15 fps subsample, real iPhone fixes, GT in a LiDAR frame so SE3 only)

| system | GNSS input | ATE SE3 (m) |
|---|---|---|
| OKVIS2-X mono (BRISK threshold 12, 1500 kp; default 34/1000 diverged to 54 km) | none | 46.9 |
| OKVIS2-X mono | iPhone GNSS 1 Hz (reported sigma 14 m) | **diverged (19.8 km)** |
| own loose fusion, batch / causal 30 s | iPhone GNSS | 33.2 / 42.1 |
| own loose fusion, GNSS sigma forced to 5 m (post-hoc) | iPhone GNSS | 27.0 / 40.2 |
| **iPhone GNSS fixes alone, SE3-aligned** | | **5.73** |

The phone scene is a low-texture running track; mono VIO is poor (about 9% of path) and there are about 4000 RANSAC failures in the log. The GNSS-only trajectory beats every
fused result. IC-GVINS (the one system that ships a phone-style NavSatFix interface) could not be run, so there is no second system on this data. The GNSS fix timestamp
is assumed to be on the camera clock (no cross-device sync information). The GT clock is 292.887 s off the sensor clock (cross-correlation of gyro norm vs GT angular speed, r = 0.75,
unique peak), applied via `estimate_offset.py`. Lever arm iPhone-to-camera unmodelled (r_SA = 0).

## 4. What failed, precisely

- **OKVIS2-X (earlier note: "needs PCL")**: not required. Only `ThreadedSlam.cpp`/`SubmappingInterface.cpp` include `pcl/io/ply_io.h` for debug dumps; stub + removing `find_package(PCL)` in
  `okvis_multisensor_processing/CMakeLists.txt` was enough. `USE_NN=OFF`, `-DCMAKE_MODULE_PATH=<deps>/share/cmake/geographiclib`. Submodules use ssh URLs, cloned by hand over https.
  Caveat found: with SPP-grade 1 Hz noisy GNSS the online GNSS-extrinsic initialisation only triggered at state 8080 (about 400 s in); with RTK it triggered at about 80 s. With
  the phone data (sigma 14 m) the global alignment blew up (19.8 km). Not tuned (`yaw_error_threshold`, `robust_gps_init` left as shipped).
- **IC-GVINS**: builds in the same conda env (abseil vendored, `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`). The GVINS bag was converted to its input convention (IMU re-expressed
  front-right-down, NavSatFix with real covariance at 1 Hz, UTC stamps, camera extrinsics rotated). It initialises correctly (GNSS heading 97.4 deg, pitch 0.29 deg), then segfaults in
  `MISC::getImuSeriesFromTo` called from `GVINS::addNewTimeNode` at the first keyframe ("Insert keyframe 0 ... with 0 new mappoints"), reproduced at replay rates 1.0 and 0.4, with and without
  CPU load, and under gdb. Likely cause: IMU stream/keyframe time bookkeeping with our data (the node also logs "Lost IMU data" at start because its IMU callback `try_lock` drops messages). About 60 min
  spent, stopped. Config and logs: `runs/gnss_compare/raw/ic_complex_on/`.
- **GICI-LIB**, **R2-GVIO**: see table above.
- Vehicle sequence and a second phone sequence not run (time). No temporal variability, all runs single.

## 5. Findings

1. **GNSS fusion is decisive when it is good, marginal when it is phone-grade.** OKVIS2-X mono: 8.0 m (VIO) -> 0.16 m with RTK-grade fixes, and the output is already geo-referenced
   (0.165 m without any alignment). With realistically noisy 1 Hz fixes (1.5/3 m) OKVIS2-X did not improve over VIO (9.6 m, geo-referenced error 11.5 m) because its global alignment initialised
   late; the bound is then about the noise level, not better.
2. **Raw-measurement tight coupling (GVINS) with real SPP-grade input**: 5.4 m -> 1.27 m SE3, 3.5 m absolute geo-referenced error (median 3.2 m, no alignment), at 0.25 core. This is the
   honest "drone with an ordinary GNSS chip" number and it is what a phone-side Android GnssMeasurement pipeline could reach. It is GPL, reference only.
3. **GNSS dropouts**: with RTK in the loop, a 120 s blackout makes OKVIS2-X drift to 3.5 m inside the window and snap back within the next 40 s (0.33 m). The simulated-SPP runs hide this because the noise floor dominates.
   Our offline smoother recovers to 0.47 m (batch) / 1.3 m (causal) inside the same window. The real float epochs (240-330 s) cost 0.15-0.36 m for RTK-fused systems and 3.6 m for GVINS (its raw-SPP floor).
4. **Our own 4-DoF + scale smoother (numpy, 200 lines, `tools/gnss_loose_fusion.py`) on top of OKVIS2 mono VIO matches or beats the tightly coupled OKVIS2-X on RTK-grade fixes**
   (0.079 m batch / 0.092 m causal vs 0.161 m) and is 2 s offline (8 s causal for 435 s). With SPP-grade 1 Hz noise it gives 2.0 m (SE3) but a 4.1 m global bias, because the simulated correlated noise
   (tau 30 s) cannot be averaged out in 436 s; this beats OKVIS2-X (9.6 m) but not GVINS's raw-measurement result in the absolute metric (3.5 m vs 4.1 m, close). Noise constants were set from physical reasoning, not tuned on these sequences. It needs
   a good VIO underneath: on the phone data (VIO itself 47 m) it only reached 27-33 m, and the unfused GNSS (5.7 m) is better.
5. **The phone result is a warning**: the VIO front end is the weak link, not the GNSS factor. OKVIS2 mono diverged with default feature thresholds on a plain track and was still 9% off at tuned
   thresholds. A GNSS-only 1 Hz track at 5.7 m RMSE would be preferable to a bad VIO fused with it. Fusion helps only with a VIO that is already locally accurate (about 1-2%).
6. Timing: OKVIS2-X mono runs 1.4-1.5x slower than real time on 16 threads in this synchronous, final-BA configuration (GNSS adds 5%). GVINS is real-time at about a quarter core.

## 6. Recommendation

**Drones (GNSS usually open sky, often RTK-capable boards)**: build a camera+IMU(+GNSS) estimator where GNSS enters as position (and later raw) factors in a permissive VIO. The cheapest proven path is
what OKVIS2-X does (BSD-3, 0.16 m with RTK, geo-referenced out of the box). As a first product, the offline/online loosely coupled smoother in `tools/gnss_loose_fusion.py` already reaches
0.08-0.09 m on RTK-grade fixes and is trivially portable (numpy, 200 lines, no solver dependency, causal variant 8 s per 435 s in Python). Add dropout handling (it already degrades gracefully: 1.3 m during a 2 min
blackout, causal) and a yaw/scale initialisation gate. Tight raw-measurement coupling (GVINS-like) is only worth the effort for SPP-only hardware.

**Phones**: iOS gives only CoreLocation fixes (no raw GNSS), Android gives raw pseudorange/Doppler (API 24+, no carrier-phase accumulation on most devices).
Plan for loose coupling of fix streams (1 Hz, claimed sigma of 5-15 m), because that is the only portable interface; use it as a global anchor and drift limiter, not for accuracy.
Spend the effort on the VIO front end first (phone cameras, rolling shutter, low-texture floors); a strong, drift-bounded VIO plus a robust (Huber, covariance-inflated) 1 Hz
position stream is what the smoother above can use. Raw Android GNSS (tight coupling) is a later step.

Phone facts (one line each):
- iOS: CoreLocation exposes position/velocity/accuracy only, no raw pseudorange/Doppler ([Apple forums](https://developer.apple.com/forums/thread/691886), "no API to get at the low level data").
- Android: `GnssMeasurement`/`GnssMeasurementsEvent` raw pseudorange-rate/Doppler since API 24 (Android 7.0), accumulated delta range in newer APIs ([Android docs](https://developer.android.com/develop/sensors-and-location/sensors/gnss), [GSA white paper](https://galileognss.eu/wp-content/uploads/2018/05/Using-GNSS-Raw-Measurements-on-Android-devices.pdf)).
- Public smartphone camera+IMU+GNSS datasets: **Mobile-GVIO** (Honor phone camera + IMU + iPhone fixes, CC-BY-4.0, used here, [Zenodo](https://zenodo.org/records/20525157)); ADVIO (iPhone/Android camera+IMU, indoor-biased, no raw GNSS, [paper](https://www.ecva.net/papers/eccv_2018/papers_ECCV/papers/Santiago_Cortes_ADVIO_An_Authentic_ECCV_2018_paper.pdf)); a smartphone raw-GNSS tightly-coupled VIO assessment exists ([Appl. Sci. 2025](https://doi.org/10.3390/app152312796)); the Google Smartphone Decimeter Challenge has raw Android GNSS but no camera.

## 7. Reproduce

`tools/gnss_harness/gvins_bag_to_euroc.py`, `make_gps.py`, `make_okvis2x_cfg.py`, `run_okvis2x.sh`, `gpl_glue/run_gvins.sh`, `score_all.py`.
Environment notes: `external/gnss/rosenv.sh` (micromamba robostack), `external/vio/env.sh` (no-root deps for OKVIS2-X). Dataset heads: HTTP range request to
`huggingface.co/datasets/Shawn202606/GVINS-Dataset/resolve/main/complex_environment.bag` (first 5 GB), `bag_head.py` handles the truncated bag.
Disk: peak about 13 GB (conda env 4.5 GB, bag head 5 GB, converted images 2.9 GB, bags 3.1 GB), end state 5.4 GB.

## 8. Phone/handheld robustness (2026-10-02)

Question: is OKVIS2's failure on real phone footage a BRISK/front-end problem, or do all systems fail? Benchmark only, nothing ported. Data re-fetched with the
existing streaming fetchers (Outdoor-1: 394 s, 5898 frames at 15 fps = every 2nd frame, 1280x720; complex_environment: first 436 s, 8721 frames at 20 Hz, 752x480, `/cam0` stream = the
"right" camera, OKVIS2-X's shipped calibration), scored and then deleted (peak new disk about 9 GB of images/fixtures, 0.6 GB left in `external/gnss/rob/out`).
Results: `runs/gnss_compare/robustness/` (`table.md` = every run incl. variants, `table.json`, `<run>/{metrics.json,traj.txt}`, `raw/` = run.json + log tails, `diag_*.json`,
`frontend_*.json`). Scripts: `tools/gnss_harness/{fetch_complex_images,make_layout,make_fixtures,make_robust_cfgs,robust_score,robust_fusion,robust_diag,frontend_probe,cam_imu_offset}.py`,
`tools/gnss_harness/gpl_glue/run_rob.sh` (one dispatcher for all systems), configs in `tools/gnss_harness/robust_cfg/`.

Scoring: `gnss_eval.score` on `benchmark.umeyama_alignment` (Sim3 and SE3 = `with_scale=False`), one alignment per run over all poses the system emitted (so a system that only keeps its
largest map is scored on that map; its "coverage" says how much that is). Coverage = emitted poses / camera frames. "losses / resets / maps" are parsed from each system's own log
(ORB-SLAM3: "Fail to track local map"/"set to lost" events, "Reseting active map", "New Map created"; stella: "tracking lost"; OKVIS: "TRACKING FAILURE"). RTF = wall / data duration with 2-4 jobs
sharing the 16-thread machine (not clean timings; OKVIS "track" = until "Finished!", before the final BA, which was killed for the later runs because the final-BA file did not change the
scored trajectory in the one run that completed: SE3 74.79 either way). Outdoor-1 GT = the LiDAR-rig GT in its local frame with the 292.887 s clock offset of section 3.2 (SE3 only is meaningful for IMU systems).
Calibration: Outdoor-1 = dataset `calib/orbslam3.yaml` (pinhole+radtan); complex = OKVIS2-X config. IMU noise: ORB-SLAM3 on Outdoor-1 both with the dataset's own yaml (600x350 resize, 4000 features, noise 1e-3/1e-2, "ds") and
with ours (full res, noise 1e-2/1e-1 as in the tuned OKVIS config); complex 4e-3/8e-2 (dataset imu.yaml).

### 8.1 Mobile-GVIO `Outdoor-1` (phone, 507 m walk/run, 394 s)

| system | sensors | ATE Sim3 (m) | ATE SE3 (m) | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|---|
| stella_vslam upstream mono (full 1280x720) | cam | 16.2 (4.3 on 176-394 s, 4.5 on 97-262 s) | - | 58.6 (arbitrary) | 99% (5827/5898), span 100% | 4 lost, all re-localised / 0 / 1 | 0.41 |
| stella C port (`sv_run`, full res, see 8.4) | cam | 37.5 (20.0 on 176-394 s, 5.4 on 97-262 s) | - | 121 | 100% | 0 / 0 / - | 0.74 |
| stella upstream / port at 640x360 | cam | 2.1 / 6.9 | - | - | **3% / 41%** | 1 lost, never recovered / 1968 lost frames | 0.11 / 0.20 |
| ORB-SLAM3 mono (GPL, ref) | cam | 2.8 | - | 17.9 | 43% (2516), one map, 97-262 s | 351 local-map failures / 3 / 8 maps | 0.84 |
| ORB-SLAM3 mono-inertial, dataset cfg (GPL, ref) | cam+IMU | 3.4 | **5.05** | 0.94 | 55% (3262), one map, 176-394 s | 121 / 121 / 62 maps (all in the first 180 s) | 0.55 |
| ORB-SLAM3 mono-inertial, our noise/full res | cam+IMU | 31.4 | 272 | 0.12 | 40% (238-394 s) | 128 / 108 / 61 | 1.07 |
| OpenVINS mono (GPL, ref), 3 configs | cam+IMU | never initialises | - | - | 0% | - | 0.1-0.24 |
| OKVIS2-X mono, default BRISK 34 / 1000 kp | cam+IMU | 57.5 | **32 712 (diverged)** | 0.001 | 100% | 215 TRACKING FAILURE, 3888 RANSAC FAIL / - | 1.43 |
| OKVIS2-X mono, tuned BRISK 12 / 1500 kp | cam+IMU | 59.8 | 74.8 | 0.45 | 100% | 0 TRACKING FAILURE, 1729 RANSAC FAIL | 2.2 track (7.4 with BA) |
| OKVIS2-X mono, tuned + IMU noise x10 | cam+IMU | 61.0 | 68.9 | 15.5 (stays put) | 100% | 2165 RANSAC FAIL | 2.0 track |

(The earlier note's 46.9 m for the tuned OKVIS is one run of a non-deterministic multi-threaded system; this one gave 74.8 m. Same conclusion.)

Fusion (own loose smoother, 4-DoF + scale, `tools/gnss_loose_fusion.py` via `robust_fusion.py`; SE3 ATE vs GT; real iPhone 1 Hz fixes, reported sigma 14 m unless stated). GNSS alone = 5.73 m over all 394 fixes;
because the good ORB-SLAM3 maps only cover part of the track, "GNSS alone on the same fixes" is given too (fixes within 1 s of an emitted pose):

| input | raw ATE (SE3) | fused batch | fused causal 30 s | Sim3 fit of the whole trajectory to the fixes (no smoother) | GNSS alone on the same fixes |
|---|---|---|---|---|---|
| ORB-SLAM3 mono-inertial (ds), 176-394 s | 5.05 | 5.44 (sigma forced 5 m: 6.75) | 7.21 | **3.38** | 4.47 (218 fixes) |
| ORB-SLAM3 mono + gravity from accelerometer, 97-262 s | (Sim3 2.81) | **2.72** (median scale 1.017) | - | 2.84 | 4.47 (167 fixes) |
| stella_vslam upstream mono + gravity, whole run | (Sim3 16.2) | 10.98 | - | 16.2 | 5.73 |
| stella C port mono + gravity, whole run | (Sim3 37.5) | 27.3 | - | 37.5 | 5.73 |
| OKVIS2-X tuned (earlier note: 33.2) | 74.8 | 36.7 | - | - | 5.73 |

Camera-only runs have arbitrary scale: for the smoother they were pre-normalised by one global Sim3 scale against the fixes, the smoother then re-estimates a scale random walk (it does, 4-DoF + scale; median 1.0-1.1), and the gravity direction
(which the smoother assumes) was taken from the mean accelerometer specific force (IMU used only for "up"; any phone has a gravity sensor). Result: a *good local trajectory* (ORB-SLAM3, 2.6-3.4 m over 3 min) fused with 1 Hz phone fixes
beats the fixes alone on the same epochs (2.7-3.4 vs 4.5 m), but that is only 40-55% of the sequence; every full-coverage fusion (stella, OKVIS) is worse than GNSS alone (5.73 m).

### 8.2 GVINS `complex_environment` (handheld rig, 436 s, gyro mean 0.54 rad/s, 1.1 rad/s p95, 3.7 rad/s peak at 34 s)

| system | sensors | ATE Sim3 (m) | ATE SE3 (m) | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|---|
| stella_vslam upstream mono | cam | 0.07 (12.5 s only) | - | 23.8 | **3%** (252), init at 22.4 s, lost at 35.0 s, never recovers | 1 / 0 / 1 | 0.31 |
| stella C port | cam | 0.11 (34 s) | - | 3.8 | **7%** (631), lost at 35 s for good (8022 lost frames) | 8022 lost / 0 / - | 0.37 |
| ORB-SLAM3 mono | cam | 1.54 | - | 5.65 | 90% (7828), 7-388 s | 64 failures / 1 / 3 maps | 0.37 |
| ORB-SLAM3 mono-inertial | cam+IMU | **crash** (SIGSEGV in `Tracking::PredictStateIMU`) after 82-363 resets in the first ~25 s; 4 attempts (default noise, noise x10 variant, repeat, accelerometer rescaled x1.075) | - | - | 0% | up to 363 / 180 / 94 | 0.33-0.40 |
| OpenVINS mono (GPL, ref) | cam+IMU | static init never fires (motion); dynamic init: 68.4 / diverges (270 km); gravity 9.13 (the accelerometer norm is 7% low, median 9.13 m/s^2): 22.2 | 320 | 0.32 | 99% | - | 0.2 |
| OKVIS2-X mono, default | cam+IMU | 3.95 | **10.9** (earlier run 8.0) | 0.94 | 100% | 0 / - / -, 19 RANSAC FAIL | 1.23 (1.10 track) |
| OKVIS2-X mono, BRISK 12 / 1500 kp | cam+IMU | 9.5 | 38.0 | 0.81 | 100% | 0 / -, 82 RANSAC FAIL | 2.3 |

### 8.3 Diagnosis

**Not a BRISK-only problem; the phone sequence defeats the mono IMU systems and exposes weak initialisation in every system.** What the data say:

1. **OKVIS2 fails on the phone at the front-end/association level, not in its IMU weighting.** On Outdoor-1 it logs 1729 (tuned) to 3888 (default) "RANSAC FAIL" and thousands of "large reprojection error" events spread over the whole run
   vs 19 / 82 on the GVINS rig where it works (3.95 m / 10.9 m SE3). Its estimated speed is right in the first 30 s (1.34 m/s, GT 1.3), then inflates to 3-4 m/s (scale 0.45 overall), freezes at 240-300 s and explodes (up to 15 m/s) at the end.
   Inflating the IMU noise 10x does not help (the trajectory then barely moves, 2165 RANSAC FAIL): vision-only association is not delivering. Default thresholds (34 / 1000 kp) diverge (SE3 32 km, 215 tracking failures); lowering the BRISK threshold (12 / 1500) stops
   the divergence but not the scale collapse; on the GVINS rig the same lowering makes it worse (10.9 -> 38 m), so the threshold is not the fix. (The BRISK implementation of OKVIS is not in the OpenCV 5.0 here, so BRISK vs ORB could not be probed in isolation; an OpenCV ORB probe
   on the same frames finds 393 (consecutive) / 312 (3 frames apart) RANSAC inliers per pair out of 1000 keypoints, inlier ratio 0.78 / 0.76, 0% of pairs under 30 inliers on Outdoor-1 (541 / 372, 0.84 / 0.78 on the rig): the images do support matching.)
2. **Vision-only front-ends do track Outdoor-1**: stella_vslam upstream at full resolution holds 99% of the frames (4 losses, all re-localised) and ORB-SLAM3 mono holds a 165 s map; on the same windows their Sim3 ATE is 2.7-4.5 m (ORB-SLAM3 mono 2.7 on 97-262 s, stella 4.5; ORB-SLAM3 MI 3.4, stella 4.3 on 176-394 s).
   Their weakness is initialisation and scale: stella's first 30 s are 40 m off (map initialised with 182 points from a low-parallax initial pair; scale 58 arbitrary and drifting), ORB-SLAM3 creates 62 maps in the first 176 s (IMU initialisation keeps failing, "IMU is not or recently initialized. Reseting active map") before one map survives to the end.
   ORB-SLAM3 mono-inertial is the only IMU system that reaches metric accuracy (5.05 m SE3, scale 0.94), and only with the dataset authors' tuned yaml (our noise/resolution: 272 m); it covers 55% of the time.
3. **Resolution matters more than the descriptor.** stella upstream at 640x360 initialises but is lost after 12 s (3% coverage), at 1280x720 it tracks 99%. The C port cannot take 1280x720 as shipped (see 8.4); at 640x360 it keeps 41%.
4. **The handheld rig fails the vision-only systems differently**: a 3.7 rad/s rotation burst at t = 34.0 s kills both stella variants one second later (lost at 35.0 / 35.2 s, upstream initialised at 22.4 s, the port at 0.5 s) for the rest of the run (no relocalisation target: the map is 80 keyframes); ORB-SLAM3 survives by starting new maps and merging (90%). ORB-SLAM3 mono-inertial crashes in its IMU-initialisation logic on this data in every try.
   OpenVINS' static initialiser needs a still platform; with dynamic init it diverges unless gravity magnitude is set to the measured 9.13 m/s^2 (the ADIS16448 accelerometer here is about 7% low), then 22 m.
5. **Time alignment is fine.** Cross-correlating the visual angular speed (scale-free, from stella / ORB-SLAM3 mono poses) with the gyro norm (`cam_imu_offset.py`): GVINS rig dt = 0.000 s (peak 0.93, 0.5 elsewhere); Outdoor-1 a flat peak over -0.06..+0.03 s, best -0.015 s at 15 fps (corr 0.5-0.6, the visual rates are noisy), the dataset yaml says +0.0018 s. So a >50 ms
   offset is excluded; with 15 fps images and regular stamps (frame intervals exactly 66.7 ms, IMU exactly 10.0 ms, so the stamps are clean but carry no jitter information) a 10-30 ms error cannot be excluded. The 292.887 s GT offset of section 3.2 is a separate, GT-side issue.
6. **Texture / blur / rolling shutter**: per 30 s bins (`diag_*.json`): Outdoor-1 has 2600-8800 FAST(20) corners per frame, gyro mean 0.32 rad/s (p95 0.57) and path speed 0.9-1.4 m/s: it is not dark, blurry or fast. The hardest stretch for ORB-SLAM3 (first 176 s, map resets) has the lowest sharpness (Laplacian variance 145-420, lowest 120-180 s) but not the lowest corner counts.
   The GVINS rig is the faster-moving one and gets dark at 240-330 s (mean grey 42-48, FAST(20) 800-1200) where ORB-SLAM3 mono and OKVIS still hold. Rolling shutter could not be isolated (no readout time in the data, no system models it); it is a plausible contributor on the phone, not proven.
   Honest summary: mono(-inertial) initialisation on a low-parallax, grass-and-track scene is the weak point; the reference-grade mono-inertial ORB-SLAM3 needs a tuned config and still spends 3 minutes failing to initialise.

### 8.4 Notes on the stella C port driver (`sv_run`, not modified)

- It rejects any frame size other than 640x480 (`w != p.cols`) and `--camera` only sets intrinsics: scratch copies in `external/gnss/rob/svbuild/` override `p.cols/p.rows` from the environment (a one-line `getenv` after `sv_system_params_default`).
- At 1280x720 `sv_extract.c` overruns its per-level candidate buffer (`to_distribute.cap = 8192`, heap overflow in `sv_distribute_keypoints`, found with ASan); the scratch build raises the cap to 262144 (unchanged behaviour where it never overflowed). Worth a real fix before any phone use.
- `sv_run` skips the first 3 lines of `rgb.txt`/`depth.txt` (TUM headers): without 3 header lines the first 3 frames are dropped and every pose gets a timestamp 3 frames late (found by the gyro cross-correlation: -0.15 s at 20 Hz); fixed in `make_layout.py`.
- The port is deterministic and tracks Outdoor-1 without a loss (upstream: 4 losses) but drifts more (37.5 vs 16.2 m Sim3); neither closes a loop there. RTF 0.37 (rig) / 0.74 (phone, full res) single-threaded.

### 8.5 Recommendation (front end for phones)

- Do **not** build the phone path on OKVIS2's BRISK front end as is: it diverges or collapses scale on the phone data under both threshold settings and IMU weightings, while an ORB front end (stella / ORB-SLAM3) keeps tracking and reaches 3-4 m locally.
- Use an **ORB-style front end at full camera resolution** (stella_vslam / our port, BSD) for tracking, with its own robust initialisation and relocalisation, and add the IMU as a **loosely coupled orientation/scale aid** first: gravity from the accelerometer already makes our 4-DoF + scale smoother usable on mono output (2.7 m fused vs 4.5 m GNSS alone on the 165 s ORB-SLAM3 stretch).
  A tightly coupled mono-inertial back end (ORB-SLAM3-style, 5 m SE3 with a tuned yaml) is the accuracy ceiling seen here, but it needs a much better initialiser than any system tested (62 maps in 3 min).
- Keep GNSS-only fixes as the safety net: no full-coverage camera-based result in this study (including all fusions) beat the 5.73 m of the raw phone fixes.
- Before porting: fix the port's keypoint-candidate cap for 720p, make the init tolerate planar low-parallax tracks, and evaluate on more phone sequences (single sequence, single run per system here; non-deterministic systems differ by tens of metres between runs).

Not done / limits: single run per system; no second phone sequence; ORB-SLAM3 mono-inertial on the GVINS rig crashed so no cam+IMU reference exists there; OpenVINS got only three cheap config tries per dataset (no tuning beyond FAST/num_pts/init window/gravity);
Outdoor-1 GT is in a local frame with an estimated clock offset, so absolute metres are indicative; camera-only scores use the camera pose against the rig GT (lever arm ignored).


## 9. More phone sequences (2026-10-02)

Question: is the Outdoor-1 failure pattern (section 8) specific to that sequence? Benchmark only, nothing ported, our stella C port (`stella_port/`, `stella_vio/`) deliberately not built or run. Single run per system (OKVIS2-X repeated once on the two sequences where it looked like an outlier; the final BA of OKVIS was killed after "Finished!" because it took 20-60 min on 300-450 s of data and does not change the scored trajectory, section 8). Scoring, log parsing and RTF as in section 8
(`phone_score.py` reuses `robust_score.py` / `gnss_eval.score` / `benchmark.umeyama_alignment`; "RTF" for OKVIS = tracking phase only, 2-4 jobs shared the 16 threads so RTFs are indicative). Results: `runs/gnss_compare/phone_more/` (`table.md` = every run incl. variants and fusion, `table.json`, `seq_stats.txt`, `gnss_alone.txt`, `<run>/{metrics.json,traj.txt}`, `raw/<run>/{run.json,log_tail.txt}`).

### 9.1 Data and licences

| dataset / sequence | licence | what was used | notes |
|---|---|---|---|
| Mobile-GVIO `Indoor-1`, `Indoor-2`, `Outdoor-2` (same Honor phone and calibration as Outdoor-1) | CC BY 4.0 (Zenodo 20525157) | whole Indoor-1 (119 s) and Indoor-2 (102 s), first 450 s of Outdoor-2 (of 19 GB); every 2nd frame = 15 fps, 1280x720, IMU 100 Hz | fixes: indoors the iPhone GNSS is useless (reported sigma about 67 m), so no GNSS fusion there, Outdoor-2 sigma 14 m at 1 Hz. GT = LiDAR-rig frame with its own clock offset (gyro-vs-GT-angular-speed cross-correlation, `estimate_offset.py`): Indoor-1 -282.52 s (r = 0.93), Indoor-2 -282.99 s (0.91), Outdoor-2 -293.93 s (0.60, unique peak; Outdoor-1 was -292.89 s, 0.75), so SE3 only for IMU systems |
| ADVIO `advio-15` (office, indoor, 52 s, 0.43 m/s) and `advio-20` (outdoor, 302 s, 1.65 m/s, 474 m) | **CC BY-NC 4.0** (Zenodo 1476931; non-commercial: fine for this internal benchmark, not for a product) | iPhone video 720x1280 portrait 60 fps -> every 2nd frame = 30 fps, gyro + accelerometer 100 Hz, ARKit/Tango/ARCore ignored, GT camera poses (IMU-integration based, 100 Hz, fix-point anchored), CoreLocation fixes (advio-20: 301 fixes, sigma 30 -> 5 m, advio-15: 38 indoor) | 0.07-0.26 GB per sequence zip. Calibration batches 13-17 / 20-23 from the ADVIO calibration README (iphone-03/04), Kalibr T_cam_imu, IMU noise from the same README. Accelerometer is stored in m/s^2 as specific force (+up at rest): confirmed by OKVIS2-X working on advio-15 (scale 1.04). Frame stamps are the real (jittered) platform stamps, GT stamps are 0.311-0.315 s ahead of the sensor clock (cross-correlation r = 1.000 on both) |
| not run | | other ADVIO sequences (mall, metro), Mobile-GVIO IO-1/2/3 (14-23 GB each), a third public set | time |

Disk: peak new about 7.5 GB (JPEG images of the five sequences, deleted after each sequence's runs; `advio20` was fetched twice because OpenVINS needed the images after a disk-full incident), 0.6 GB of run outputs left in `external/gnss/rob/out`, 14 MB of results in `runs/`, fixtures (imu / gt / gnss csv) 2-8 MB per sequence kept, images regenerable (below). Nothing committed.

### 9.2 Reproduce / reusable fixtures (for the stella-port agent)

```bash
G=external/gnss; PY=$G/venv/bin/python; T=tools/gnss_harness; R=$G/rob
# Mobile-GVIO (streams the zip over HTTP, ~25-40 min per 450 s; images 15 fps JPEG, ~0.3-0.5 MB/s output)
$PY $T/mobilegvio_to_euroc.py Indoor-1 $R/indoor1 1e9 --every 2        # Indoor-2 -> indoor2; Outdoor-2 $R/outdoor2 450 --every 2
# ADVIO (download advio-NN.zip from https://zenodo.org/records/1476931/files/advio-NN.zip?download=1)
$PY $T/advio_to_euroc.py advio-20.zip $R/advio20 --every 2             # 2.4 GB of JPEG for 302 s; advio-15 -> advio15 (0.23 GB)
for s in indoor1 indoor2 outdoor2 advio15 advio20; do $PY $T/make_layout.py $R/$s; $PY $T/mobile_make_gps.py $R/$s; done   # EuRoC mav0/, times.txt, TUM-style tum/{rgb,depth}.txt (sv_run), gps0 + gnss_enu.txt
$PY $T/make_phone_cfgs.py   # robust_cfg/<seq>/{stella,stella_lowfast,orb3_mono,orb3_mi,orb3_mi_ds,okvis_default}.yaml, ov/, port_camera.txt  (needs make_robust_cfgs.py run once for the outdoor1 templates)
# clock offsets of GT vs sensors are in $T/phone_offsets.json (estimate_offset.py), Mobile-GVIO GT must be shifted by it (GT.from_tum(path, dt=offset)); ADVIO stamps carry a +1.6e9 s base (see docstring)
for s in stella_up orb3_mono orb3_mi_ds okvis_default ov_mono; do $T/gpl_glue/run_rob.sh $s indoor1; done   # or gpl_glue/run_phone_seq.sh <seq>; OKVIS_NOBA=1 skips the final BA; SCFG=stella_lowfast selects the low-FAST stella config
$PY $T/phone_score.py; $PY $T/phone_fusion.py <seq> <src_run> <out_run> [--cam-only] [--fit]; $PY $T/phone_gnss_alone.py <seq> [runs]; $PY $T/phone_stats.py
```
Port notes for the stella-port agent: ADVIO frames are 720x1280 portrait (the port's `sv_run` rejects anything but 640x480 and `sv_extract.c` overflows at 720p/1280x720 per section 8.4); `port_camera.txt` and `tum/rgb.txt` are generated, nothing was run.

### 9.3 Results per sequence

Columns as in 8.1. Sim3 for camera-only systems is the only meaningful figure (arbitrary scale); the scale column is the Sim3 scale. Coverage = poses / camera frames (span = time covered). losses / resets / maps from each system's log (stella "tracking lost"; ORB-SLAM3 "Fail to track local map" / "Reseting active map" / "New Map created"; OKVIS "TRACKING FAILURE", with the RANSAC FAIL count in the text).

**Mobile-GVIO `Indoor-1`** (corridors, 119 s, 15 fps, 138 m at 1.3 m/s, gyro mean 0.26 rad/s; white walls, motion blur)

| system | ATE Sim3 | ATE SE3 | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|
| stella upstream mono, default | no initialisation | - | - | 0% | 0 / 0 / 0 | 0.33 |
| stella upstream mono, FAST 10/4 | 1.85 | - | 2.26 | 74% (30-119 s) | 0 / 0 / 1 | 0.35 |
| ORB-SLAM3 mono | 15.26 | - | 6.04 | 98% | 6 / 0 / 1 | 0.61 |
| ORB-SLAM3 mono-inertial (dataset cfg) | 0.51 (2 s) | 0.51 | 1.42 | **2%** (last 2 s) | 124 / 124 / 63 | 0.47 |
| OKVIS2-X mono default | 4.78 | **8.26** | 0.73 | 100% | 0 TRACKING FAILURE, 1352 RANSAC FAIL | 1.14 (1.50) |
| OpenVINS mono | no initialisation | - | - | 0% | - | 0.26 |

**Mobile-GVIO `Indoor-2`** (corridor/hall, 102 s, 92 m at 1.06 m/s)

| system | ATE Sim3 | ATE SE3 | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|
| stella upstream mono, default | 0.38 | - | 7.62 | 97% (span 98%) | 0 / 0 / 1 | 0.32 |
| stella upstream mono, FAST 10/4 | 0.25 | - | 9.67 | 99% | 0 / 0 / 1 | 0.40 |
| ORB-SLAM3 mono | 0.29 | - | 9.74 | 99% | 0 / 0 / 1 | 0.65 |
| ORB-SLAM3 mono-inertial (dataset cfg) | 0.20 (4 s) | 0.33 | 3.33 | **4%** (last 4 s) | 26 / 26 / 14 | 0.48 |
| OKVIS2-X mono default | 9.19 | **27.0** | 0.25 | 100% | 0 TF, 1343 RANSAC FAIL | 1.26 (1.90) |
| OpenVINS mono | no initialisation | - | - | 0% | - | 0.26 |

**Mobile-GVIO `Outdoor-2`** (first 450 s, 587 m at 1.3 m/s, gyro mean 0.37 rad/s; same kind of scene as Outdoor-1)

| system | ATE Sim3 | ATE SE3 | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|
| stella upstream mono | 50.5 (11-97 m per 30 s bin, growing) | - | 16.6 | 100% | 2 / 0 / 1 | 0.58 |
| ORB-SLAM3 mono | 0.30 (330-424 s only) | - | 20.8 | **22%** | 1790 / 43 / 54 | 0.98 |
| ORB-SLAM3 mono-inertial (dataset cfg) | 56.3 | **8961 (diverged)** | 0.005 | 61% (many short gaps) | 564 / 176 / 91 | 0.60 |
| OKVIS2-X mono default | 86.7 | **8943 (diverged)** | 0.005 | 100% | 111 TF, 2974 RANSAC FAIL | 1.99 (2.20) |
| OKVIS2-X mono default, repeat | 87.7 | **6409 (diverged)** | 0.007 | 100% | 109 TF | 2.07 |
| OpenVINS mono | no initialisation | - | - | 0% | - | 0.33 |

**ADVIO `advio-15`** (iPhone, office, 52 s, 30 fps, 21 m at 0.43 m/s, gyro mean 0.25 rad/s)

| system | ATE Sim3 | ATE SE3 | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|
| stella upstream mono | 0.81 | - | 2.56 | 80% (gap at 22-29 s) | 4 / 0 / 1 | 0.74 |
| ORB-SLAM3 mono | 0.89 | - | 3.12 | 98% (span 92%) | 92 / 0 / 2 | 1.03 |
| ORB-SLAM3 mono-inertial (dataset noise; and noise inflated to 1e-2 / 1e-1) | no map survives (both) | - | - | 0% | 51 / 48 / 27 (56 / 52 / 29) | 1.42 |
| OKVIS2-X mono default | 1.65 | **1.65** | 1.04 | 99% | 21 TF, 710 RANSAC FAIL | 2.18 (3.27) |
| OpenVINS mono | 1.59 (initialises at 21 s) | 850 (scale collapse) | 0.001 | 60% | - | 0.41 |

**ADVIO `advio-20`** (iPhone, outdoor urban, 302 s, 30 fps, 474 m at 1.65 m/s, gyro mean 0.39 / max 2.1 rad/s)

| system | ATE Sim3 | ATE SE3 | scale | coverage | losses / resets / maps | RTF |
|---|---|---|---|---|---|---|
| stella upstream mono | 53.7 (27-101 m per 30 s bin) | - | 11.3 | 99% | 1 / 0 / 1 | 0.98 |
| ORB-SLAM3 mono | 5.98 (1.0-10.8 m per bin) | - | 3.70 | 89% (2-267 s) | 278 / 1 / 5 | 0.96 |
| ORB-SLAM3 mono-inertial (dataset noise) | no map survives | - | - | 0% | 1033 / 975 / 517 | 1.44 |
| OKVIS2-X mono default | 55.8 | 58.8 | 1.94 | 100% | 404 TF, 1884 RANSAC FAIL | 3.86 (8.5 incl. BA) |
| OKVIS2-X mono default, repeat | 63.6 | **880 (diverged)** | 0.026 | 100% | 559 TF | 4.10 |
| OpenVINS mono | no initialisation | - | - | 0% | - | 0.73 |

### 9.4 Fusion with the phone GNSS fixes (`phone_fusion.py`, own smoother of section 8; sensitivities not tuned)

Only the two outdoor sequences have usable fixes (Outdoor-2: 450 fixes at 1 Hz, sigma 14 m; advio-20: 301 fixes, sigma 30 -> 5 m; indoors none). GNSS alone is scored on the same epochs as each run's poses. SE3 ATE vs GT unless noted.

| sequence | input | raw | fused batch smoother | Sim3 fit to fixes | GNSS alone (same fixes) |
|---|---|---|---|---|---|
| Outdoor-2 | ORB-SLAM3 mono + gravity (22%, 330-424 s) | (Sim3 0.30) | **2.13** | 2.06 | 7.08 (96 fixes) |
| Outdoor-2 | stella upstream mono + gravity (100%) | (Sim3 50.5) | 39.6 | 50.6 | **14.66** (450 fixes) |
| Outdoor-2 | ORB-SLAM3 mono-inertial (61%) | 8961 | 29.6 | - | 15.20 (300 fixes) |
| Outdoor-2 | OKVIS2-X mono (100%) | 8943 | 19.7 | - | **14.66** |
| advio-20 | ORB-SLAM3 mono + gravity (89%) | (Sim3 5.98, SE3 49.0) | 12.10 (Sim3 4.17) | 12.44 (Sim3 5.98) | 12.16 SE3 / 6.31 Sim3 (265 fixes) |
| advio-20 | stella upstream mono + gravity (99%) | (Sim3 53.7) | 42.8 | 54.0 | **12.00** SE3 / 6.21 Sim3 (301 fixes) |
| advio-20 | OKVIS2-X mono (100%) | 58.8 | 50.8 | - | 12.00 |

Same conclusion as Outdoor-1: the raw phone fixes beat every full-coverage fusion (Outdoor-2 14.7 m, advio-20 12.0 m SE3 / 6.2 Sim3); the only gain is on the short good ORB-SLAM3 mono stretch of Outdoor-2 (2.1 m vs 7.1 m), and on advio-20 the fusion of ORB-SLAM3 mono does not beat the fixes in SE3 (the iPhone fixes there carry a large scale/shape bias against the ADVIO GT: 12.0 SE3 vs 6.2 Sim3).

### 9.5 Does the Outdoor-1 pattern repeat? Per-system diagnosis

Outdoor-1 pattern (8.1): OKVIS2-X diverges or collapses scale; stella has full coverage but a large Sim3 error from scale/drift; ORB-SLAM3 mono is accurate on a short surviving map; ORB-SLAM3 mono-inertial survives only 55%; OpenVINS never initialises; raw GNSS beats all full-coverage fusion. **It repeats on the long outdoor walks (Outdoor-2, ADVIO-20) almost point for point, and indoors the IMU-based systems fail in the same way while the vision-only ones do fine.** The one counter-example is the short, slow ADVIO office sequence where OKVIS2-X is the best system (1.65 m SE3, correct scale).

- **OKVIS2-X (BRISK, IMU)**: Outdoor-2 diverges like Outdoor-1 (SE3 8943 / 6409 m in two runs, scale 0.005, 111 TRACKING FAILURE, 2974 RANSAC FAIL); advio-20 is erratic (58.8 m and 880 m in two runs, 404-559 tracking failures, 1884 RANSAC FAIL); indoors it tracks 100% but scale-collapses (0.73 / 0.25, SE3 8.3 / 27 m, 1.3k RANSAC FAIL on 100 s). Where it works (advio-15: 0.43 m/s, 52 s, 30 fps, only 21 TRACKING FAILURE) it is metric and best, so the failure is again front-end association over long, fast-ish, low-parallax or textureless stretches plus weak IMU observability at walking speed, not calibration (the advio-15 result also validates the ADVIO calibration / IMU convention). Non-deterministic: repeats differ by 1-3 orders of magnitude on the diverging runs.
- **stella_vslam upstream (mono)**: reliable coverage (97-100% on 4 of 5 sequences, 80% on advio-15), few losses (0-4), real time (RTF 0.3-1.0), but initialisation is fragile on corridors (Indoor-1: no initialisation at all with the default FAST thresholds 20/7, works after lowering to 10/4, from 30 s) and the scale drifts badly outdoors (Sim3 50.5 / 53.7 m on Outdoor-2 / ADVIO-20 over 450-474 m paths; 16 m on Outdoor-1). Indoors, with a map that does not loop, it is excellent locally (Sim3 0.25-0.38 m on Indoor-2, 0.8 m on advio-15).
- **ORB-SLAM3 mono (GPL ref)**: the most accurate where it holds a map (Outdoor-2 0.30 m, ADVIO-20 5.98 m over 89%, indoors 0.29 / 0.89 m) but outdoors it keeps losing tracking (Outdoor-2: 1790 failures, 54 maps, only the last 94 s survive; Outdoor-1 43% coverage; ADVIO-20 278 failures); Indoor-1 15 m (scale/drift over a long straight corridor).
- **ORB-SLAM3 mono-inertial (GPL ref)**: never a usable full run: 0-4% coverage indoors (IMU initialisation succeeds only in the last 2-4 s), 0% on both ADVIO sequences (48 / 975 resets, "Not enough motion for initializing", maps born with 130-190 points), 61% but diverged on Outdoor-2 (SE3 8961 m, 91 maps). The Outdoor-1 55% with a tuned yaml is the best this system did on phones.
- **OpenVINS (GPL ref)**: initialises on none of the 4 new Mobile/ADVIO-20 sequences ("not enough feats to compute disp: 0,34 < 15", only 33 valid features of 48 needed for the dynamic init, num_pts 200 / FAST 20 on a 1-config budget); advio-15 initialises at 21 s then loses scale (SE3 850 m). Same as Outdoor-1.

Cause ranking from these data (not isolated by experiment): (1) initialisation of mono and mono-inertial systems on low-parallax / textureless / planar scenes (corridors, track, pedestrian walking with 0.3-0.4 rad/s mean rotation), (2) scale observability and drift of mono over long walks without loops (stella 50 m, OKVIS scale collapse), (3) feature front-end robustness (white corridor walls; BRISK association; ORB with default FAST thresholds; not measured per frame here), (4) rolling shutter and motion blur remain plausible but unproven (no readout time in Mobile-GVIO, ADVIO lists line delay 0 for a rolling-shutter iPhone camera; none of the systems models it); (5) timestamps are not the issue: frame / IMU stamps are regular (Mobile: frame dt std 0.0-0.8 ms with one 75 ms gap at 15 fps, IMU 100.0 Hz, ADVIO: frame stamps real and exactly 33.3 ms apart after subsampling, IMU 99.9-100 Hz with 12 ms max gap; `seq_stats.txt`), the GT clock offsets are found to better than 0.01 s, and OKVIS runs correctly on advio-15 with the same stamping.

**Best system on phones overall**: no system is both metric and full-coverage on the long outdoor walks. Ranked by robustness across the 6 sequences incl. Outdoor-1: stella_vslam upstream (BSD, usable coverage everywhere, 0.25-0.8 m Sim3 indoors, 16-54 m outdoors from scale drift) and ORB-SLAM3 mono (best local accuracy, 22-98% coverage); the IMU-based systems (OKVIS2-X, ORB-SLAM3 mono-inertial, OpenVINS) are worse than the vision-only ones on 5 of 6 sequences. The recommendation of 8.5 stands: ORB-style front end at full resolution with robust initialisation, IMU only as gravity / scale aid, GNSS as safety net (the raw fixes beat every full-coverage result outdoors on all three phone sequences: 5.7 / 14.7 / 12.0 m).

Limits: one run per system (two for OKVIS on two sequences); OpenVINS and stella got one or two cheap config tries; no ORB-SLAM3 mono-inertial tuning on ADVIO (dataset noise and 1e-2 / 1e-1 noise tried on advio-15); Mobile-GVIO frame rate 15 fps (every 2nd frame), ADVIO 30 fps; Outdoor-2 truncated at 450 s; Mobile GT absolute metres indicative (LiDAR frame + estimated clock offset, camera pose vs rig GT, no lever arm); no cross-device (phone camera vs separate iPhone GNSS) sync information; RTFs measured with 2-4 concurrent jobs.
