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

## 10. Own C library for the loose fusion: `gnss_fusion/` (2026-10-02)

Own code, MIT, C99 (`<stdint.h> <math.h> <stdlib.h> <string.h> <limits.h>` only, own block-tridiagonal Gauss-Newton with 5x5 Cholesky blocks),
re-implementing `tools/gnss_loose_fusion.py` (section 2/3: 4-DoF + scale pose graph, Huber GNSS factors, 1 s nodes) and extending it. Folder layout, API and
model are in `gnss_fusion/README.md`; `gnss_fusion/tools/run_all.sh` re-runs everything below (outputs in the git-ignored `gnss_fusion/work/`).
Library: `gf_geo.{h,c}` (WGS-84 LLA/ECEF/ENU), `gf_fusion.{h,c}` (`gf_add_odom`, `gf_add_fix`, `gf_get_pose`, `gf_solve_batch`, `gf_query`),
driver `gf_run.c` (text in, TUM out, timing). No data or binaries committed; inputs are the saved trajectories / fixes of sections 3, 8, 9
(`external/gnss/...`, OKVIS2 VIO on GVINS complex_environment with the RTK and simulated-SPP fixes, real iPhone / ADVIO fixes, ORB-SLAM3 / stella / OKVIS2 phone runs).

### 10.1 Python vs C, same inputs

Same node grid, same initial alignment, same Gauss-Newton schedule (batch: 25 iterations on the whole graph; causal: 30 s window, 4 iterations per node, 10 settle
iterations on the first 11 nodes), same Huber IRLS. `gnss_fusion/tools/compare_py.py`, per odometry pose (positions; TUM text output rounds to 1 micrometre):

| case | mode | poses | max diff (mm) | rms (mm) | python s | C s (incl. file I/O) |
|---|---|---|---|---|---|---|
| complex_rtk | batch | 8714 | 0.0008 | 0.0005 | 2.2 | 0.15 |
| complex_rtk | causal | 8714 | 0.0131 | 0.0009 | 9.8 | 0.16 |
| complex_sim | batch | 8714 | 0.0008 | 0.0005 | 0.8 | 0.11 |
| complex_sim | causal | 8714 | 0.0008 | 0.0005 | 14.5 | 0.96 |
| complex_rtk_blk | batch | 8714 | 0.0009 | 0.0005 | 2.0 | 0.09 |
| complex_rtk_blk | causal | 8714 | 0.0139 | 0.0010 | 12.1 | 0.22 |
| complex_sim_blk | batch | 8714 | 0.0008 | 0.0005 | 0.9 | 0.06 |
| complex_sim_blk | causal | 8714 | 0.0008 | 0.0005 | 11.0 | 0.31 |
| o1_okvis | batch | 5898 | 0.0008 | 0.0005 | 2.3 | 0.05 |
| o1_okvis | causal | 5898 | 0.3141 | 0.0184 | 12.8 | 0.29 |
| o2_okvis | batch | 6736 | 0.0009 | 0.0005 | 3.2 | 0.06 |
| o2_okvis | causal | 6736 | 3843.5252 | 352.7449 | 12.5 | 0.42 |
| o1_orb3mono | batch | 2516 | 0.0008 | 0.0005 | 0.4 | 0.02 |
| o1_orb3mono | causal | 2516 | 0.0008 | 0.0005 | 4.5 | 0.10 |
| o2_orb3mono | batch | 1454 | 0.0008 | 0.0005 | 0.2 | 0.02 |
| o2_orb3mono | causal | 1454 | 0.0008 | 0.0005 | 2.4 | 0.04 |

Batch agrees to 0.001 mm (floating point noise plus the 6-decimal text output) on all 8 cases, including the camera-only ORB-SLAM3 cases (gravity from the
accelerometer through `gf_set_gravity`, scale from the fixes), node yaw to 0.005 microrad and scale to 5e-7. Causal agrees to 0.3 mm or better on 7 of 8 cases;
Outdoor-2 OKVIS2 (a diverged VIO, scale 0.005) is chaotic under 4-iteration windows and drifts apart after node 223 (max 3.8 m, rms 0.35 m), same behaviour class
as the python result itself (154 m ATE). Causal equivalence is obtained by feeding the fixes 0.5 s ahead (`--lookahead 0.5`): python associates a node with the
nearest fix up to half a node later, the real-time library attaches a fix as soon as it arrives and re-solves (one extra window solve), so it is not bit-identical to the
python replay in normal use. Geodetics (`tools/test_geo.py`, 404 random points): LLA->ENU 5e-7 m and ECEF 5e-5 m (print precision) vs the numpy conversion of the harness, ENU->LLA round trip 1e-7 m.
Differences by design: (i) python needs gaps bridged by pseudo-poses (`fill_gaps`), the library segments instead; (ii) monocular scale: python pre-normalises with one
global Sim3 scale, the library estimates a unit per odometry frame at the alignment (same numbers when there is one frame); (iii) python reads every fix of 1 s nodes only, the
library keeps the same one-fix-per-node nearest rule.

### 10.2 Results (ATE SE3 in m, `gnss_eval.score`; geo-referenced error where the output is in the ENU frame of the fixes)

Columns: raw odometry (SE3-aligned) | GNSS fixes alone (all fixes / only fixes within 1 s of an emitted pose, for the partial ORB-SLAM3 maps) | python smoother batch / causal 30 s | C smoother with python-equivalent
settings batch / causal 30 s | C smoother with `gf_config_robust()` (consistency test, 120 s drift test, chi2 gate 16.27 + 1 m floor, trimmed init, free monocular scale) batch / causal 30 s / live
output of `gf_get_pose` after init. Causal columns are scored from 30 s after the start (the alignment time) for both python and C. `complex_*` = GVINS complex_environment, OKVIS2 mono VIO,
`rtk` = receiver RTK fixes (also the GT, optimistic: GNSS alone 0.00), `sim` = simulated SPP-grade 1 Hz fixes (1.5 m / 3 m), `blk` = 100-220 s blackout; `o1/o2` = Outdoor-1/2 (iPhone fixes, reported sigma 14 m), `a15/a20` = ADVIO-15/20.
`okvis` = OKVIS2-X mono VIO, `orb3mono` = ORB-SLAM3 mono (partial maps with gaps, camera only: gravity from the accelerometer, scale from fixes), `stella` = stella_vslam upstream mono.

| sequence | raw odometry | GNSS alone (all / on covered epochs) | python batch / causal30 | C batch / causal30 | C robust batch / causal30 / live |
|---|---|---|---|---|---|
| complex_rtk | 8.02 | 0.00 / 0.00 | 0.08 / 0.09 | 0.08 / 0.09 | 0.08 / 0.09 / 0.10 |
| complex_sim | 8.02 | 4.18 / 4.18 | 2.04 / 2.16 | 2.04 / 2.16 | 2.04 / 2.16 / 2.15 |
| complex_rtk_blk | 8.02 | 0.00 / 0.00 | 0.26 / 0.67 | 0.26 / 0.67 | 0.26 / 0.67 / 0.67 |
| complex_sim_blk | 8.02 | 4.33 / 4.33 | 1.88 / 2.17 | 1.88 / 2.17 | 1.88 / 2.17 / 2.16 |
| o1_okvis | 46.86 | 5.73 / 5.73 | 33.22 / 43.18 | 33.22 / 43.18 | 6.20 / 8.20 / 8.22 |
| o2_okvis | 8943.46 | 14.66 / 14.66 | 19.66 / 153.94 | 19.66 / 154.07 | 14.41 / 16.62 / 16.63 |
| a15_okvis | 1.65 | 1.56 / 1.56 | 1.65 / 1.48 | 1.65 / 1.48 | 1.65 / 1.48 / 1.47 |
| a20_okvis | 58.84 | 12.00 / 12.00 | 50.82 / 56.62 | 50.82 / 56.62 | 11.93 / 12.13 / 12.12 |
| o1_orb3mono | 40.52 | 5.73 / 4.47 | 2.68 / 7.76 | 2.68 / 7.76 | 2.17 / 3.06 / 3.02 |
| o2_orb3mono | 30.99 | 14.66 / 7.08 | 2.59 / 1.14 | 2.59 / 1.14 | 2.53 / 2.58 / 2.52 |
| a15_orb3mono | 1.36 | 1.56 / 1.56 | 1.21 / 0.84 | 1.21 / 0.84 | 1.26 / 1.04 / 1.02 |
| a20_orb3mono | 48.97 | 12.00 / 12.16 | 12.08 / 13.88 | 12.08 / 13.88 | 12.12 / 12.57 / 12.52 |
| o1_stella | 68.90 | 5.73 / 5.73 | 10.70 / 37.24 | 10.70 / 37.24 | 6.22 / 15.19 / 15.19 |
| o2_stella | 95.04 | 14.66 / 14.66 | 38.95 / 61.24 | 38.95 / 61.24 | 13.78 / 38.29 / 38.34 |

| sequence (geo-referenced ENU only) | GNSS alone | python batch / causal30 | C batch / causal30 | C robust batch / causal30 |
|---|---|---|---|---|
| complex_rtk | 0.00 | 0.09 / 0.10 | 0.09 / 0.10 | 0.09 / 0.10 |
| complex_sim | 5.50 | 4.07 / 4.39 | 4.07 / 4.39 | 4.07 / 4.39 |
| complex_rtk_blk | 0.00 | 0.27 / 0.76 | 0.27 / 0.76 | 0.27 / 0.76 |
| complex_sim_blk | 5.35 | 4.28 / 4.64 | 4.28 / 4.64 | 4.28 / 4.64 |

Reading: (1) python and C columns are identical to the printed digits wherever the python is deterministic. (2) With the consistency test the fused result is never worse than
the raw iPhone fixes by more than 1-3 m when the VIO is bad (Outdoor-1 OKVIS2: 33.2 -> 6.2 m batch, GNSS alone 5.7; Outdoor-2: 19.7 -> 14.4, alone 14.7; ADVIO-20 50.8 -> 11.9, alone 12.0) and it
is unchanged when the odometry is good (complex_*, a15). Causal is 1-3 m behind batch on the bad-VIO sequences (8.2 vs 5.7, 16.6 vs 14.7): the distrust verdict needs a window of fixes, so the
newest seconds are fused with the unreliable odometry. (3) Monocular cam-only, full coverage (stella): batch 6.2 / 13.8 m is on par with GNSS alone (5.7 / 14.7), but causal is still 15 / 38 m (see 10.6).
(4) The partial ORB-SLAM3 maps (22-88 % coverage) stay far better than GNSS alone on the covered epochs, e.g. o1 3.1 vs 4.5 m causal, o2 2.6 vs 7.1 m.

### 10.3 (a) Robust loss, gating, "trust GNSS when the odometry is inconsistent"

Mechanism (`trust=1`): per node, in a +-15 s window, the odometry positions at the fixes are fitted with a trimmed 4-DoF + scale similarity. The odometry is distrusted for that node if
(metric odometry) the fitted scale leaves [1/1.5, 1.5] while the fixes move more than 3 times their own noise (second differences, MAD), or the fit residual exceeds 4 times the GNSS noise and 5 m, or the 120 s window residual exceeds 8 m. Distrusted
links lose their odometry factor (a weak position link of 1 m + 3 m/s * dt remains, yaw/scale random walks are relaxed), so the nodes there follow the fixes only (smoothed GNSS) and the output between nodes
is interpolated. Gate: a fix is dropped if its normalised residual `sum(r^2/(sigma^2+1 m^2)) > 16.27` (never inside a segment with < 5 fixes, never when > 50 % of the window would be dropped).
Two things this does not use: the reported fix sigma (the iPhone's 14 m says nothing; the second-difference noise of the fixes is 0.2-0.4 m, the error is a slow bias) and any ground truth. Thresholds were set from physical reasoning and the first looks at the
Outdoor-1 / Outdoor-2 windows, then checked on the other sequences; they were not tuned per sequence, but the sequences are few, so treat the margins as indicative.

Synthetic outliers on top of good odometry / real fixes (`gf_studies.py outliers`: 5 % spikes of 15 m + 5-12 reported sigmas in a random direction; two 20 s multipath bursts with a 25 m common offset, reported sigma unchanged; ATE SE3 m, GNSS alone in the second column; H = python Huber 2.5, N = no loss, C = Cauchy 2.385, Hg / Cg = Huber / Cauchy + gate + trimmed init, R = full robust preset):

| sequence | variant | GNSS alone | batch: H / N / C / Hg / Cg / R | causal: H / N / C / Hg / Cg / R |
|---|---|---|---|---|
| complex_sim | clean | 4.18 | 2.04 / 2.04 / 1.84 / 2.04 / 1.84 / 2.04 | 2.16 / 2.17 / 2.03 / 2.16 / 2.02 / 2.16 |
| complex_sim | spikes | 7.63 | 2.00 / 2.12 / 1.82 / 2.03 / 1.84 / 2.03 | 2.14 / 2.58 / 2.01 / 2.15 / 2.01 / 2.19 |
| complex_sim | burst | 8.40 | 2.84 / 4.92 / 1.84 / 1.96 / 1.76 / 2.87 | 6.97 / 7.35 / 6.03 / 5.28 / 3.45 / 7.99 |
| complex_sim | both | 10.60 | 2.85 / 4.93 / 1.83 / 1.94 / 1.73 / 2.86 | 7.05 / 7.60 / 6.09 / 5.46 / 3.95 / 7.90 |
| o1_orb3mono | clean | 5.73 | 2.68 / 2.68 / 2.68 / 2.70 / 2.70 / 2.17 | 7.76 / 7.76 / 7.99 / 7.76 / 7.99 / 3.06 |
| o1_orb3mono | spikes | 32.08 | 5.79 / 5.93 / 5.79 / 2.71 / 2.71 / 2.45 | 63.69 / 49.51 / 103.46 / 7.79 / 8.03 / 2.86 |
| o1_orb3mono | burst | 8.79 | 3.56 / 3.56 / 3.52 / 3.56 / 3.52 / 6.89 | 7.84 / 7.84 / 8.21 / 7.84 / 8.21 / 8.24 |
| o1_orb3mono | both | 33.49 | 2.88 / 2.91 / 2.91 / 3.81 / 3.76 / 6.95 | 76.38 / 71.53 / 136.45 / 74.63 / 139.42 / 9.20 |
| o1_okvis | clean | 5.73 | 33.22 / 32.37 / 35.54 / 36.85 / 37.86 / 6.20 | 43.18 / 41.69 / 45.21 / 43.21 / 45.23 / 8.20 |
| o1_okvis | spikes | 30.70 | 33.35 / 32.49 / 35.63 / 36.79 / 37.84 / 5.68 | 43.15 / 41.40 / 45.30 / 43.27 / 45.35 / 8.20 |
| o1_okvis | burst | 8.79 | 33.32 / 32.56 / 35.64 / 37.83 / 38.38 / 8.17 | 42.91 / 41.42 / 45.17 / 42.96 / 45.19 / 9.93 |
| o1_okvis | both | 29.61 | 33.31 / 32.30 / 35.72 / 37.92 / 38.47 / 7.79 | 41.70 / 41.00 / 44.47 / 43.16 / 45.38 / 19.60 |

Findings: Huber alone (python reference) already handles spikes. A burst of consistent bad fixes is only handled by the gate (complex_sim burst, batch: 2.84 -> 1.96 Huber+gate, 1.76 Cauchy+gate; causal 6.97 -> 5.28 / 3.45). The full preset
(consistency test on) loses that gain because a 20 s burst looks like inconsistent odometry to a 30 s window (batch 2.87, causal 7.99), a trade-off we did not resolve; a stricter AND-rule with the long window fixes some bursts but delays distrust on bad VIO (o1 OKVIS causal 8.2 -> 12.5), so it is
available (`trust_long_and`) but off. Cauchy without the consistency test is catastrophic when the odometry is junk (it ignores the fixes), so it is not in the preset. Trimmed initialisation matters for causal mode:
Outdoor-1 ORB-SLAM3 with spikes, causal, 63.7 m (python Huber) vs 7.8 m (gate + trimmed init).

### 10.4 (b) Odometry tracking loss (gaps, new maps)

`gf_add_odom` starts a new segment automatically after a gap > 2 s, or on `GF_ODOM_GAP` / `GF_ODOM_NEW_FRAME`; position is bridged by a weak link, a new frame is aligned from its own fixes once it spans 4 m (provisional state until then).
Test (`gf_studies.py loss`): complex_sim / complex_rtk with the odometry cut for 25 s and 20 s; afterwards it restarts either in the same frame with a 15-25 m position jump (relocalisation), or in an unrelated new frame (random yaw, +-60 m origin). The python smoother
gets the python harness' bridged gaps (constant-velocity pseudo-poses, one frame). ATE SE3 / geo-referenced error (m):

| case | python batch, gaps bridged | C batch | C causal 30 s | no loss at all (C batch / causal) |
|---|---|---|---|---|
| sim, same frame, jump | 2.39 / 4.24 | 2.09 / 4.20 | 2.26 / 4.04 | 2.04 / 2.16 (4.07 / 4.39) |
| sim, NEW frame | 5.19 / 6.36 | 2.04 / 4.17 | 2.17 / 3.87 | |
| rtk, same frame, jump | 0.10 / 0.11 | 0.08 / 0.09 | 0.09 / 0.10 | 0.08 / 0.09 |
| rtk, NEW frame | 0.19 / 0.20 | 0.08 / 0.09 | 0.10 / 0.10 | |

A break costs nothing measurable relative to no loss (the fixes re-align the new frame); an unflagged new frame is also survived in the robust preset (the consistency test sees it) but not with the python-equivalent settings (causal 47.9 m), so flag new maps.
First version of this had a bug found by the study: the chi2 gate rejected every fix after a position jump for 20 s (state not yet constrained); fixed by never gating inside segments with fewer than 5 fixes. Real partial maps (ORB-SLAM3 mono, raw trajectories with their gaps, no pseudo-poses): C robust vs python bridged, batch / causal:
o1 2.17 / 3.06 vs 2.68 / 7.76, o2 2.53 / 2.58 vs 2.59 / 1.14, a15 1.26 / 1.04 vs 1.21 / 0.84, a20 12.12 / 12.57 vs 12.08 / 13.88 (table above).

### 10.5 (c) GNSS blackout bridging

`gf_studies.py blackout`: complex RTK and simulated-SPP fixes removed from 100 s for 30 / 60 / 120 / 240 s; error vs GT in the ENU frame (no alignment), RMS per time-since-blackout bin; "live" causal output of `gf_get_pose`:

| RTK fixes, blackout | batch (smoother, fixes on both sides) | causal live: 0-30 s | 30-60 s | 60-120 s | 120-240 s |
|---|---|---|---|---|---|
| 30 s | 0.37 | 0.61 | | | |
| 60 s | 0.39 | 0.61 | 1.09 | | |
| 120 s | 0.43 (max 0.75) | 0.61 | 1.09 | 2.29 | |
| 240 s | 3.21 (max 5.18) | 0.61 | 1.09 | 2.29 | 10.45 (max 16.2) |

Dead-reckoning drift of OKVIS2 + the last alignment: about 0.6 m after 30 s, 1 m after 60 s, 2.3 m (60-120 s), then fast growth (yaw drift of the VIO): 10 m RMS in the 120-240 s bin. Before the blackout the error is 0.08-0.09 m. With
SPP-grade fixes the blackout is invisible (5.2-6.4 m up to 120 s, the fixes' own bias of 4-5 m dominates), 14.5 m at 120-240 s causal. The heuristic `sigma_h` (hypot(reported sigma / sqrt(n fixes), 0.02 * path since the last fix)) is calibrated on the RTK case
(median error / sigma 0.8-1.4, 71-93 % of epochs within 2 sigma) and optimistic for the SPP case (2.3-9: it does not know the fixes' own correlated bias). `GF_ST_NO_RECENT_FIX` flags dead reckoning after 10 s.

GNSS velocity (extension, no python counterpart): Doppler-like velocity (RTK-reference velocity + 0.1 m/s) on the simulated fixes: batch 2.04 -> 1.98, causal 2.16 -> 2.05 (SE3), small because the odometry is already good.

### 10.6 Timing (one thread, AMD Ryzen 7 3700X, gcc -O2)

Causal complex_sim (8714 odometry samples at 20 Hz, 436 nodes, 436 fixes), `gf_run --timing`, per call:

| call | python-equivalent settings | `gf_config_robust()` |
|---|---|---|
| `gf_add_odom` creating a node (1 per second): mean / p50 / p99 / max | 197 / 216 / 283 / 1721 microseconds (quiet machine; the max is the one-off alignment + 11-node replay at 30 s) | 469 / 437 / 672 / 3244 microseconds |
| `gf_add_odom`, other samples | 0.07 microseconds | 0.07 microseconds |
| `gf_add_fix` | 0.04 microseconds (queued; its cost is in the node call) | same |
| `gf_solve_batch`, 436 nodes | 2.2 ms | |

A second run during heavy load from other jobs on this 16-thread machine gave 327 / 227 / 3281 and 554 / 525 / 3534 microseconds (mean / p50 / p99), i.e. the medians hold, the tails are scheduler noise.

Per `gf_add_odom` that creates a node (1 per second at node_dt = 1 s) the cost is the window re-solve; all other odometry samples (20 Hz here) and fixes (queued until their node exists) cost
under 0.1 microsecond. At 1 node/s that is 0.02-0.05 % of one core; a 10x slower phone core would still spend about 5 ms per node. Memory in causal mode with `keep_history = 0`: window + 8 .. 4x that many nodes (about 0.4 KB each, 60 KB), no allocation per call once warm.

### 10.7 Open issues

- Monocular VO with an unstable scale in causal mode (stella full coverage): 15 / 38 m vs GNSS alone 5.7 / 14.7 m. The distrust verdict arrives only after 4-30 s of fixes, the spikes before it dominate the RMS; a monocular scale-jump test exists
(`trust_scale_k_mono`) but it hurts causal results so it is off. Batch is fine (6.2 / 13.8).
- Multipath bursts vs the consistency test interact (10.3); the gate works on its own.
- Causal output is only as good as the 30 s alignment: nothing is geo-referenced before `init_wait_s` (odometry passed through, status without `GF_ST_INIT`).
- Fixes during an odometry gap are dropped (no GNSS-only nodes), the segment is bridged and re-aligned when the odometry returns.
- No marginal covariance; `sigma_h` is a heuristic. Only one fix per 1 s node is used (the nearest), 10 Hz RTK is decimated like in the python reference.
- Tuned by reasoning and looked at on the same sequences it is evaluated on (14 sequence/VO combinations, 2 GNSS-simulation variants); the iPhone sequences are the only real GNSS; the RTK-fix variants use the GT receiver, so they are an upper bound.


## 10. More mono-inertial systems on the phone sequences (2026-10-02)

Question: does any other mono+IMU system give full coverage with metric scale on the phone data of sections 8-9? Benchmark only. XRSLAM (Apache-2.0) is the main subject; VINS-Fusion and DM-VIO are GPL reference only (executed, never copied); Kimera-VIO (BSD-2) was built and run on EuRoC only (note: `docs/vio_candidates_20261001.md` addendum). Same fixtures, GT (clock offsets of `phone_offsets.json`), scorer machinery and metrics as section 9 (`tools/vio_harness/more_phone_score.py` reuses `robust_score` / `gnss_eval.score` / `benchmark.umeyama_alignment`); images were re-fetched into `external/vio2/phone/` with `external/vio2/fetch_phone.sh` and deleted after the runs (peak about 5 GB). Results: `runs/gnss_compare/more_systems/` (`table.md`, `table.json`, `<run>/{metrics.json,traj.txt}`, `raw/<run>/{run.json,log_tail.txt}`).
Caveats: single run per cell; machine load average 30-60 (RTF column is wall/duration under that load, so only an upper bound; XRSLAM is single threaded, CPU-s per data-second: indoor1 about 1.1, advio20 about 3.9 incl. the reader's cv::undistort of 720x1280 frames); "poses / frames" for VINS-Fusion is low because it publishes about every second frame, use "covered span". XRSLAM default IMU noise = its `iphone12.yaml` continuous covariances; `[infl]` = OKVIS-tuned phone values (1e-2 / 1e-1 densities). Skipped on request (overloaded machine): VINS-Fusion on Outdoor-1 / Outdoor-2, Kimera on the phone sequences, DM-VIO (OOM at startup, see the addendum), repeats.

| sequence | system | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage (poses / frames) | losses | note |
|---|---|---|---|---|---|---|---|
| Indoor-1 (119 s) | XRSLAM | **0.84** | 0.73 | 1.02 | 92% | 0 | first pose at ~9 s; OKVIS2-X 8.26, stella Sim3 1.85 |
| Indoor-1 | VINS-Fusion | 802 | 14.4 | 0.01 | 49% (span 99%) | 0 | scale collapse |
| Indoor-2 (102 s) | XRSLAM | **0.98** | 0.56 | 1.07 | 82% | 0 | OKVIS2-X 27.0 |
| Indoor-2 | VINS-Fusion | no output | - | - | 0% | - | never initialised |
| ADVIO-15 (52 s) | XRSLAM default | 1451 (scale collapse) | 1.59 | 0.00 | 97% | 0 | |
| ADVIO-15 | XRSLAM [infl] | **1.55** | 1.51 | 1.71 | 97% | 0 | OKVIS2-X 1.65 |
| ADVIO-15 | VINS-Fusion | 614 | 1.62 | 0.00 | 49% (span 97%) | 0 | |
| ADVIO-15 | DM-VIO | no output | - | - | 0% | OOM at startup (19 GB) | |
| ADVIO-20 (302 s) | XRSLAM default / [infl] | 143 044 / 57 265 | 60.2 / 56.7 | 0.00 / 0.00 | 99% | 0 | scale collapse; GNSS alone 12.0 SE3 |
| ADVIO-20 | VINS-Fusion | 3610 | 8.5 | 0.01 | 18% (span 35%) | 0 | |
| Outdoor-1 (394 s) | XRSLAM default | **6.44** | 6.10 | 1.03 | 99% | 0 | GNSS alone 5.73; best full-coverage metric result of the study (OKVIS2-X 74.8, ORB-SLAM3-MI 5.05 at 55% coverage) |
| Outdoor-1 | XRSLAM [infl] | 35.2 | 27.5 | 1.53 | 99% | 0 | noise sensitivity: worse here, better on ADVIO-15 |
| Outdoor-2 (450 s) | XRSLAM default | 252 768 | 67.1 | 0.00 | 98% | 0 | scale collapse; GNSS alone 14.7 |

Failure diagnosis (XRSLAM, from the logs: 2 state changes per run, i.e. initialised once after 1.3-9 s, never reported TRACKING_FAIL): it keeps tracking the whole time with a visually plausible trajectory (Sim3 0.56-1.6 m indoors and on ADVIO-15, 6 m on Outdoor-1) but the metric scale is lost on the long outdoor walks (ADVIO-20, Outdoor-2: scale collapses to 0, drift 60-67 m Sim3) and on ADVIO-15 with default noise. The IMU observability at 0.4-1.6 m/s walking speed is weak, scale is then not held by the sliding-window VIO (no loop closure, no GNSS). Indoors with default noise it is metric (0.84 / 0.98 m SE3), far better than OKVIS2-X (8.3 / 27 m) and the ORB-SLAM3 mono-inertial (no map) of section 9; it initialises on all six sequences, which none of OpenVINS / ORB-SLAM3-MI / VINS-Fusion did. VINS-Fusion (shipped config, IMU noise 1e-2 / 1e-1, no loop closure) initialised on 4 of 5 but its scale collapsed on all of them.

GNSS fusion (own smoother, `tools/gnss_loose_fusion.py` via `tools/vio_harness/more_fusion.py`, a copy of `robust_fusion.py` with the run directory redirected; only Outdoor-1 qualifies: full coverage with metric scale, XRSLAM default, the one all-zero first pose removed):

| input | ATE SE3 (m) |
|---|---|
| GNSS fixes alone (394 fixes) | 5.73 |
| XRSLAM raw | 6.44 |
| XRSLAM + GNSS, batch smoother (median scale 1.009) | 5.76 |
| XRSLAM + GNSS, causal 30 s | 9.26 |
| XRSLAM, one global Sim3 fit to the fixes | 6.10 |

Fusion gives 5.76 m, i.e. no better than the raw fixes (5.73 m): the 1 Hz phone fixes with 14 m reported sigma add nothing to a VIO that is already at the GNSS accuracy level on this one sequence. Outdoor-2 (14.7 m GNSS) and ADVIO-20 (12.0 m) were not fused (scale collapsed, SE3 meaningless).

Answer: **no system is good on phones across the board.** XRSLAM is the best new result: full coverage, no tracking loss, metric on the three short/indoor sequences and on Outdoor-1 (0.84 / 0.98 / 1.55 / 6.44 m), but its scale collapses on ADVIO-20 and Outdoor-2, and the noise setting that fixes one sequence breaks another. It is therefore a candidate for the front end of a loosely coupled phone pipeline only if the scale is supplied externally (GNSS Sim3 fit; in the Outdoor-1 test it equals GNSS alone), not a drop-in. Licence: XRSLAM Apache-2.0 (its README notes that some optional methods carry additional licences; not used here), Kimera-VIO BSD-2; VINS-Fusion and DM-VIO GPL-3 (reference only); ADVIO data CC BY-NC 4.0, Mobile-GVIO CC BY 4.0.
Disk: builds and tools 2.0 GB in `external/vio2/` (images and datasets deleted; the shared `external/vio/data/MH_01_easy` holds 0.9 GB of re-fetched cam0 + imu0 that was left in place), results about 80 MB in `runs/`. Nothing committed.


## 11. gnss_fusion: fixes for the open issues of 10.7 (2026-10-03)

Own code, `gnss_fusion/` only. Baseline = the first `gf_config_robust()` (section 10), still available as `preset=robust1` in `gf_run`; "now" = `gf_config_robust()` with the features below. All new features are C-only and off in `gf_config_default()`, so the python-equivalent settings are unchanged: `compare_py.py` reproduces the section-10.1 table exactly (batch max 0.0008-0.0009 mm on 8 cases, causal 0.0131 / 0.0139 mm on rtk / rtk_blk, 0.31 mm o1_okvis, o2_okvis chaotic as before). Scores: `gnss_eval.score` (`benchmark.umeyama_alignment`), ATE SE3 in m; causal scored from 30 s. New cases in `gf_table.py`: `m14_okvis`, `o1d_okvis` (INSANE mars_14 / outdoor_1 first 100 s, OKVIS2 mono body poses of `runs/drone_compare/*/okvis2_mono/trajectory.tum` + PX4 GNSS in ENU, scored against the dual-RTK midpoint with its 6 cm lever), `o1_xrslam`, `o2_xrslam`, `a20_xrslam` (`runs/gnss_compare/more_systems/*_xrslam/traj.txt`, treated as metric VIO).

### 11.1 What changed

| issue | change (config key) | effect |
|---|---|---|
| 1 mono / drifting scale, causal | `trust_state_k=1.6`: distrust when the window similarity scale differs from the scale state by more than 1.6x (catches the stella collapse of Outdoor-2 where the VO shrinks ~4000x); `scale_min/max=0.25/4` (a scale state that flipped sign was seen with other thresholds); `trust_start_m=3`: the short consistency fit starts from the best contiguous sub-set instead of the all-points fit (the all-points fit is dragged by minority blocks) | o1_stella causal 15.19 -> 6.94 (GNSS alone 5.73), o2_stella 38.29 -> 17.66 (14.66) |
| 1b / 4 yaw locked by a poor first alignment | `grow_s=120, grow_ratio=20` (metric odometry): no frozen nodes at the start until the fixes span 20x their reported sigma or 120 s; the hard freeze pinned a yaw that was 24 deg off (xrslam o1) or arbitrary (stationary start) and corrected it only at 0.5 deg/sqrt(s) | o1_xrslam 10.62 -> 6.10, o1_okvis 8.20 -> 7.39, m14 1.93 -> 1.72, o1d 8.88 -> 1.43, a15_okvis 1.48 -> 1.37; complex_* unchanged (fix span / sigma = 25 > 20 at the alignment time) |
| 2 odometry gaps | `gnss_only_nodes=1`: while the odometry is silent for > `gap_s`, each fix makes a node without odometry link (weak position link, yaw / scale random walk continues); `gf_get_pose` returns it (`GF_ST_NO_ODOM`), `gf_node_pose(i)` for retained ones; `gf_run` writes them into `--out` / `--out-live` | output continues through tracking loss, see 11.3 |
| 3 burst vs trust | `trust_start_m` (above) makes the fit of a window that is mostly a common-offset burst use the burst block (a translation is free in the similarity fit) instead of calling the odometry inconsistent | partial, see 11.4 |

Tried and rejected (numbers on the o1/o2 stella, xrslam, orb3mono, complex_sim sets, causal): adaptive inflation of the odometry link sigma from the window residual (no effect, <0.1 m); looser boundary link to the frozen node (`freeze_sigma` 1-10 m: stella 15 -> 9 but complex_sim 2.16 -> 2.90, o2_orb3mono 2.58 -> 4.3); GNSS error-difference factors (consecutive fix residual difference with sigma 0.5-2 m, i.e. a bias random walk model: m14 1.93 -> 17.1 at 0.5, otherwise no gain); distrusted nodes weighted by the short-term GNSS noise instead of the reported sigma (o2 15.1-15.4 but o1_orb3mono batch 2.17 -> 2.9); chi2 gate before the trust test (batch o1_okvis clean 6.2 -> 10.3: the gate starves the test of fixes); longer trust windows (45-90 s: o1_okvis clean causal 8.2 -> 12.7-26); plain long windows `window_s=60..1000` (m14 / o1d / xrslam better, but complex_rtk_blk 0.67 -> 4.1 because the majority rule of the gate never lets the post-blackout fixes in, and o2_orb3mono 2.58 -> 5.0); `grow_s_mono` (growing window on monocular odometry: o2_orb3mono 2.58 -> 5.0, so it is metric-only); `trust_scale_k_mono` 1.5-2 (stella better, orb3mono 3.06 -> 4.1-4.4); `trust_rho_min` 2-3 (stella 15 -> 41 .. 14, chaotic).

### 11.2 Before / after (ATE SE3 m; batch / causal30 / causal live)

| sequence | GNSS alone | C robust v1 (baseline) | C robust now |
|---|---|---|---|
| complex_rtk | 0.00 | 0.08 / 0.09 / 0.10 | 0.08 / 0.09 / 0.10 |
| complex_sim | 4.18 | 2.04 / 2.16 / 2.15 | 2.04 / 2.16 / 2.15 |
| complex_rtk_blk | 0.00 | 0.26 / 0.67 / 0.67 | 0.26 / 0.67 / 0.67 |
| complex_sim_blk | 4.33 | 1.88 / 2.17 / 2.16 | 1.88 / 2.17 / 2.16 |
| o1_okvis | 5.73 | 6.20 / 8.20 / 8.22 | 6.14 / 7.39 / 7.40 |
| o2_okvis | 14.66 | 14.41 / 16.62 / 16.63 | 14.41 / 16.68 / 16.70 |
| a15_okvis | 1.56 | 1.65 / 1.48 / 1.47 | 1.65 / 1.37 / 1.36 |
| a20_okvis | 12.00 | 11.93 / 12.13 / 12.12 | 11.93 / 12.13 / 12.13 |
| o1_orb3mono (cov. 4.47) | 5.73 | 2.17 / 3.06 / 3.02 | 1.61 / 2.95 / 2.98 |
| o2_orb3mono (cov. 7.08) | 14.66 | 2.53 / 2.58 / 2.52 | 2.64 / 2.58 / 2.52 |
| a15_orb3mono | 1.56 | 1.26 / 1.04 / 1.02 | 1.25 / 0.98 / 0.96 |
| a20_orb3mono | 12.00 | 12.12 / 12.57 / 12.52 | 12.12 / 12.51 / 12.46 |
| o1_stella | 5.73 | 6.22 / 15.19 / 15.19 | 5.02 / 6.94 / 6.94 |
| o2_stella | 14.66 | 13.78 / 38.29 / 38.34 | 14.56 / 17.66 / 17.67 |
| m14_okvis (drone) | 1.37 | 1.46 / 1.93 / 1.93 | 1.46 / 1.72 / 1.72 |
| o1d_okvis (drone, 55 s stationary start) | 2.20 | 1.47 / 8.88 / 8.88 | 1.47 / 1.43 / 1.43 |
| o1_xrslam (phone, metric) | 5.73 | 5.18 / 10.62 / 10.62 | 5.17 / 6.10 / 6.11 |
| o2_xrslam (scale collapse) | 14.66 | 14.55 / 16.99 / 17.01 | 14.55 / 16.99 / 17.01 |
| a20_xrslam (scale collapse) | 12.00 | 11.99 / 12.14 / 12.15 | 11.99 / 12.14 / 12.15 |

Geo-referenced (un-aligned) error, where the output is in the ENU of the fixes: complex_* identical to v1 (e.g. complex_sim 4.07 / 4.39, sim_blk 4.28 / 4.64), m14 4.15 / 4.13 (GNSS 4.08), o1d 10.45 / 10.72 (GNSS 10.73). Causal against GNSS alone: all phone and drone cases are within 1.0-1.3x except o1_okvis (1.29x, 7.39 vs 5.73), o2_stella (1.20x) and m14 (1.26x of 1.37); the target "never worse than ~1.2x" is met on 14 of 17 sequence/odometry combinations that have GNSS-alone ATE above 1 m and missed slightly on those two, o1_stella 1.21x is at the line. Regressions on good odometry: complex_*, a15 none (<= 0.00 SE3 and geo); o2_orb3mono batch +0.11 (2.53 -> 2.64: the fixes inside the map gaps now take part in the batch solve; o1_orb3mono batch improves by 0.56 for the same reason), so one batch cell exceeds the 0.05 m budget; python-vs-C stays exact.

### 11.3 GNSS-only nodes in odometry gaps (`gf_studies.py gaps`)

Odometry cut for 25 / 60 / 120 s in complex_sim (same frame with a jump, or an unrelated new frame); error vs GT without alignment. Without the feature there is a single pose in the gap (the first sample after it); with it one pose per fix, equal to the GNSS error (sim fixes 4.8-5.7 m in the gap, fused 4.6-5.5; RTK fixes 0.07 m), and the 30 s after the gap and the whole-run error do not change (e.g. 60 s same frame causal: after 6.06 -> 6.04, whole 4.45 -> 4.45). The real partial maps: full time line (all fix epochs, GNSS-only poses included) o1_orb3mono 2.65 / 3.73 vs GNSS alone 5.73, o2_orb3mono 2.78 / 4.19 vs 14.66, a20_orb3mono 12.12 / 12.51 vs 12.00 (batch / causal30). The 120 s gap shows what is lost without odometry: 30 s after the gap 4.8 m causal (the VIO yaw is only re-established from the fixes).

### 11.4 Burst vs trust (`gf_studies.py outliers`, complex_sim, 20 s bursts with a 25 m offset; batch / causal)

gate alone (Huber + gate + trimmed init) 1.96 / 5.28; robust v1 2.87 / 7.99; now 2.70 / 6.88 (spikes + burst: 2.86 / 7.39 vs 2.86 / 7.90). o1_orb3mono burst: v1 6.89 / 8.24, now 6.74 / 5.45 (gate alone 3.56 / 7.84); both: now 6.78 / 9.15. o1_okvis burst now 8.11 / 8.53 (v1 8.17 / 9.93), spikes + burst causal 8.80 (v1 19.60). Using the robust-start fit in both windows gave complex_sim burst 1.96 / 5.51 and o1_orb3mono 3.51 / 5.01, but made o2_stella causal 17.9 -> 20.8 (the long-window rule then misses it), so it is used only for the short window. The gate and the trust test still do not agree fully: the majority rule of the gate disables it when a burst covers more than half of the 30 s window, which is the remaining causal gap to the gate-alone numbers.

### 11.5 Drones, long stationary start (`gf_studies.py drone`)

INSANE outdoor_1, first 55 s on the ground: the yaw of the odometry w.r.t. ENU is unobservable until the drone moves; the alignment at 30 s is arbitrary in yaw. Geo-referenced error of the causal live output over the run (from 30 s): baseline 33.9 m (45-54 m in the flight part, as bad as OKVIS2-X's 47.9 m), now 10.72 m, GNSS alone 11.32, batch 10.88 (this is PX4 receiver bias of ~10 m, OKVIS2-X has the same bias plus the 48 m yaw failure). The growing window lets the yaw be re-estimated from the first seconds of motion (bins 60-90 s: 9.5, 10.3, 12.4, 14.8 vs GNSS 10.7, 12.2, 13.4, 15.1). No explicit "yaw not yet observable" flag exists; the status has no bit for it. m14: geo 4.12 causal live vs GNSS 4.09.

### 11.6 Timing and remaining issues

Per `gf_add_odom` that creates a node (complex_sim, 436 nodes, machine under load from other jobs, so noisy): v1 mean 1.1 ms / p50 0.58 ms; now mean 1.0 ms / p50 0.74 ms; o1d (growing window up to 120 nodes at the start) mean 2.1 ms / p50 1.3 ms, p99 8 ms; steady state with the 30 s window is as in 10.6. Memory is bounded after the growth phase (nodes are trimmed again).
Remaining: o1_okvis causal 1.29x GNSS; the 3 m `trust_start_m` and the 20x `grow_ratio` were set from these 19 sequences (same data used for tuning and evaluation, 6 of the phone / drone combinations are ours); a burst covering most of a window is still partly accepted by the causal gate; monocular drifting-scale odometry stays worse than GNSS alone in causal mode by 1.2x (stella) since the information about the scale of a 30 s window is weak (fix noise of 14 m reported, bias random walk of ~0.85 m/sqrt(s) measured on the iPhone fixes); no marginalisation prior for the frozen boundary (an equivalent long window helped phones and drones but broke blackout recovery through the gate majority rule and o2_orb3mono); XRSLAM scale-collapsed runs (o2, a20) are only bridged by the GNSS (fused equals GNSS alone, 14.55 vs 14.66, 11.99 vs 12.00), the collapsed VO shape (Sim3 ATE 60-67 m) is not worth using.

## 12. gnss_fusion: gait (step cadence) speed prior as a factor (2026-10-03)

Roadmap item 1 (`docs/roadmap_research_20261003.md`, appendix A). Own code, `gnss_fusion/` only. New: `c/gf_gait.{h,c}` (streaming step detector / cadence / speed, C99, allowed headers only, 2 us per IMU sample, fixed 15 kB state), `gf_add_speed()` + config keys in `gf_fusion.{h,c}` (all off by default; `speed=1` in `gf_run`, `--speed file`), `c/gf_gait_run.c`, python prototype `tools/gait.py`, `tools/check_gait.py` (C vs python), `tools/gf_gait_study.py` (the study below) and `tools/gf_gait_report.py` (tables). Raw outputs: `gnss_fusion/work/gait_*.json`, `work/gait_study.md` (git-ignored).

### 12.1 What was built

* **Producer.** `an = |a|`, high-pass (EMA 0.8 s), double EMA smoothing (2 x 0.03 s), step = local maximum above `max(0.35 std, 0.3 m/s^2)` at least 0.3 s after the previous one. Epoch every 3 s over the last 6 s: cadence `(n-1)/(t_last-t_first)` (>= 4 steps, last step < 1.2 s old, 1.0..2.8 Hz), speed `v = k c cad^2`, 1-sigma `sqrt((0.2 v)^2 + 0.1^2)` (0.1 v after a per-user calibration). Not walking (shuffling, running, irregular) = state OTHER: no measurement is emitted. Still IMU (rms of the high-passed norm < 0.12 m/s^2, mean |gyro| < 0.12 rad/s over ~1.5 s, no step for 2 s) = STATIONARY: zero-velocity measurement. Gyro heading about the low-passed gravity is integrated alongside (calibration straightness test and the PDR baseline).
* **Model.** Generic population model: `c = 0.389`, i.e. 0.70 m step at 1.8 Hz (0.415 x 1.70 m anthropometry, fitted to nothing in this repo). The exponent 2 (step length grows with cadence) was picked by leave-one-sequence-out on the four Mobile-GVIO sequences against exponents 1, 1.5, 2.5 (cross-sequence distance-ratio spread 0.85-1.18 at exponent 1, 0.90-1.11 at 2), so the Mobile "generic" numbers carry a little optimism; ADVIO is out of sample. Appendix A's linear `a + b cad` fitted on Outdoor-1 alone had a = -1.6, b = 1.55 (unstable extrapolation) and was not used.
* **Calibration modes.** (a) *per-user*: `c` fitted on **other** sequences with a reference (c = sum v_GT / sum cad^2 over their walking epochs): each Mobile sequence is calibrated on the pooled other three (same phone and, as far as the dataset says, same person); ADVIO has no held-out walking sequence of its own (ADVIO-15 is a 0.4 m/s shuffle), so ADVIO gets the pooled Mobile `c` and is labelled **cross-user**, not per-user. (b) *online*: `gf_gait_gnss_fix()`: every 5 s, chord between the means of the fixes in the first and last 8 s of a 30 s window (variance bias removed) divided by the gait distance between the same two centres, only on straight stretches (gyro heading change <= 20 deg) with gait distance >= 15 m and fix sigma <= 20 m; `k` = (sum chords + 150 m prior) / (sum gait + 150 m prior), clamped 0.6..1.6. (c) *generic*: nothing. A fourth row "self (leak)" (c fitted on the scored sequence itself) appears only in the accuracy table as an upper bound.
* **Smoother factor** `gf_add_speed(g, t, v, sigma, window_s, flags)`: the measurement is the mean horizontal speed over `[t - window, t]`; it attaches to the node nearest to `t`. (1) *scale form*: `s_k * (horizontal odometry path over the window) / duration = v`, Huber (2 sigma) on the per-node scale state, along odometry links of the same frame, **including distrusted links** (they still measure the path length the scale multiplies; this is what lets the scale state catch up and un-distrust a collapsed VO, section 11.1 `trust_state_k`). (2) *position form* on bridged / GNSS-only / distrusted links: horizontal node displacement `/ dt = v` with sigma `sqrt(sigma^2 + 0.3^2)`. (3) *zero velocity*: STATIONARY marks every node of the window; displacement between consecutive marked nodes = 0 with sigma `0.05 m/s * dt` (floor 1 cm). Options: `speed_align=1` aligns an odometry frame without fixes (run has < 3 fixes) from the speed alone (scale = sum v T / sum odometry path, yaw 0, position continued; monocular frames get their unit rescaled), so Indoor sequences can be solved without any GNSS; `speed_scale_rw_rel=1` makes the scale random walk and prior relative above 1; `speed_scale_lim=1e5` replaces the 0.25..4 clamp while speed is on; `speed_scale_rw` (0 = unchanged) overrides the scale random walk. The study uses `preset=robust speed=1 speed_align=1 speed_scale_rw_rel=1`.
* **Experience while building it (all on these sequences, so treat the settings as tuned on the test data).** (i) stella Outdoor-1 causal without the gait has 356 of 393 nodes distrusted (the VO scale is far off the scale state, so the state-vs-fit test of section 11 never passes); with the scale factor skipped across distrusted links it still had 198 (the state could not move); with the factor acting through them, 16. (ii) Aligning later map frames from the speed alone when fixes exist locked a wrong yaw (Outdoor-2 ORB-SLAM3), so `speed_align` only acts while the run has had fewer than 3 fixes. (iii) A faster scale random walk helps OKVIS indoors (Indoor-2 batch 15.95 -> 6.78 at `speed_scale_rw=0.02`, Indoor-1 7.28 -> 6.18) but makes the diverged metric VIOs and Indoor-1 / Indoor-2 stella a little worse and the diverged XRSLAM scale ratios far worse (ADVIO-15 24 -> 154), so it stays off.

### 12.2 Gait accuracy (3 s epochs, 6 s window; walking = GT path speed > 0.5 m/s; GT speed = 0.5 s smoothed GT path length, an upper bound on the truth)

| sequence | calibration | GT-walking epochs | detected as WALK | median [IQR] | p10..p90 | distance ratio | rms rel. error | within 1 sigma | k (online) |
|---|---|---|---|---|---|---|---|---|---|
| outdoor1 | generic | 128 | 127 (1 OTHER) | 1.04 [1.02, 1.07] | 1.01..1.10 | 1.06 | 0.15 | 0.97 | 1.00 |
| outdoor1 | per-user (held-out) | 128 | 127 (1 OTHER) | 1.01 [1.00, 1.04] | 0.99..1.07 | 1.03 | 0.13 | 0.95 | 1.00 |
| outdoor1 | online GNSS | 128 | 127 (1 OTHER) | 0.95 [0.91, 0.96] | 0.89..1.01 | 0.95 | 0.15 | 0.96 | 0.88 |
| outdoor1 | self (leak) | 128 | 127 (1 OTHER) | 0.98 [0.97, 1.01] | 0.96..1.04 | 1.00 | 0.12 | 0.97 | 1.00 |
| outdoor2 | generic | 149 | 149 (0 OTHER) | 0.98 [0.97, 1.01] | 0.95..1.06 | 1.00 | 0.08 | 0.98 | 1.00 |
| outdoor2 | per-user (held-out) | 149 | 149 (0 OTHER) | 0.92 [0.90, 0.94] | 0.89..0.99 | 0.93 | 0.10 | 0.96 | 1.00 |
| outdoor2 | online GNSS | 149 | 149 (0 OTHER) | 0.95 [0.93, 0.98] | 0.89..1.01 | 0.96 | 0.10 | 0.98 | 1.01 |
| outdoor2 | self (leak) | 149 | 149 (0 OTHER) | 0.99 [0.97, 1.01] | 0.96..1.06 | 1.00 | 0.08 | 0.97 | 1.00 |
| indoor1 | generic | 37 | 37 (0 OTHER) | 1.09 [1.06, 1.11] | 1.04..1.14 | 1.10 | 0.15 | 0.95 | 1.00 |
| indoor1 | per-user (held-out) | 37 | 37 (0 OTHER) | 1.05 [1.03, 1.08] | 1.01..1.11 | 1.06 | 0.13 | 0.89 | 1.00 |
| indoor1 | self (leak) | 37 | 37 (0 OTHER) | 0.99 [0.97, 1.01] | 0.95..1.04 | 1.00 | 0.10 | 0.92 | 1.00 |
| indoor2 | generic | 29 | 29 (0 OTHER) | 1.09 [1.07, 1.13] | 1.03..1.17 | 1.10 | 0.13 | 0.97 | 1.00 |
| indoor2 | per-user (held-out) | 29 | 29 (0 OTHER) | 1.06 [1.04, 1.10] | 1.00..1.13 | 1.07 | 0.10 | 0.93 | 1.00 |
| indoor2 | self (leak) | 29 | 29 (0 OTHER) | 0.99 [0.97, 1.03] | 0.94..1.06 | 1.00 | 0.06 | 0.97 | 1.00 |
| advio15 | generic | 4 | 3 (1 OTHER) | 1.07 [0.94, 1.36] | 0.87..1.53 | 1.17 | 0.39 | 0.67 | 1.00 |
| advio15 | cross-user (Mobile cal) | 4 | 3 (1 OTHER) | 1.03 [0.91, 1.31] | 0.84..1.48 | 1.13 | 0.36 | 0.33 | 1.00 |
| advio20 | generic | 98 | 97 (1 OTHER) | 0.91 [0.87, 0.99] | 0.78..1.14 | 0.93 | 0.20 | 0.80 | 1.00 |
| advio20 | cross-user (Mobile cal) | 98 | 97 (1 OTHER) | 0.88 [0.84, 0.96] | 0.76..1.10 | 0.89 | 0.20 | 0.34 | 1.00 |
| advio20 | online GNSS | 98 | 97 (1 OTHER) | 0.91 [0.87, 0.99] | 0.78..1.15 | 0.93 | 0.20 | 0.77 | 0.99 |
| advio20 | self (leak) | 98 | 97 (1 OTHER) | 0.98 [0.94, 1.07] | 0.85..1.23 | 1.00 | 0.21 | 0.71 | 1.00 |

per-user constants c. Reading: Mobile-GVIO (same phone, same person, 394-450 s outdoor walks at 1.3 m/s, 100-120 s indoor at 1.0-1.3 m/s) is within 10 % in median and distance ratio with any calibration; the held-out per-user fit improves Outdoor-1 (1.04 -> 1.01) and the indoor sequences (1.09 -> 1.05 / 1.06) but not Outdoor-2 (0.98 -> 0.92: the other three sequences have a slightly shorter step), so per-user calibration from a short, different sequence is worth about 3-4 % in distance, not a step change. ADVIO-20 (other phone, other person, 1.5 m/s) is 7-9 % short with the generic model and 11-12 % with the Mobile constant; the self fit (leak) is 0.98 with `c = 0.420`, i.e. the person's step is 8 % longer than the population constant and 12 % longer than the Mobile person's. The online GNSS calibration did **not** help: its chords come out short (k = 0.88 on Outdoor-1: the iPhone fixes are smoothed and have 14 m sigma; an ad-hoc check that fed the GT as 1 Hz fixes gave a chord ratio of 0.945 on the same windows (not in the scripts); k = 1.01 / 0.99 on Outdoor-2 / ADVIO-20 where the chord ratio does not reveal the 9 % gait deficit of ADVIO-20) and made Outdoor-1 worse (0.95); no sequence has fixes that are good in the sense needed (the 394 Outdoor-1 fixes have sigma 14.25 m). The 6 s window gives the per-epoch scatter (rms relative error 0.08-0.15 on Mobile, 0.20 on ADVIO-20 where walking is less regular); the reported sigma covers the error in 89-98 % of Mobile epochs but only 33-34 % for the cross-user calibration (`gf_gait_set_model` switches the relative sigma to the per-user 0.1, which is overconfident for a different person; a cross-user constant should keep the generic 0.2, the study did not). ADVIO-15 is a 0.4 m/s office shuffle: 3 of its 4 GT-walking epochs are recognised, 1 is OTHER and the sequence has only 7 usable epochs (it is a failure regime of the prior, not a result). Stationary detection is untestable on the phones (no sequence stands still); on the drone IMU of INSANE outdoor_1 (55 s on the ground) 17 of 18 still epochs are flagged, 0 of 12 moving epochs are flagged still, 1 flight epoch is read as a walk (a drone is not a pedestrian).

### 12.3 With GNSS (ATE SE3 m, batch / causal30; baseline columns equal section 11)

| case | GNSS alone | robust (section 11) | + gait generic | + gait per-user (held-out) / cross-user | + gait online GNSS |
|---|---|---|---|---|---|
| Outdoor-1 stella | 5.73 | 5.02 / 6.94 | 4.95 / 6.27 | 5.09 / 7.27 | 5.64 / 9.36 |
| Outdoor-1 orb3mono | 5.73 | 1.61 / 2.95 | 1.34 / 2.16 | 1.82 / 2.35 | 2.91 / 3.33 |
| Outdoor-1 okvis | 5.73 | 6.14 / 7.39 | 6.07 / 7.14 | 6.04 / 7.11 | 6.16 / 7.31 |
| Outdoor-1 xrslam | 5.73 | 5.17 / 6.10 | 5.05 / 5.89 | 5.03 / 5.82 | 5.37 / 6.47 |
| Outdoor-2 stella | 14.66 | 14.56 / 17.66 | 14.53 / 16.85 | 14.53 / 17.06 | 14.53 / 17.43 |
| Outdoor-2 orb3mono | 14.66 | 2.64 / 2.58 | 0.81 / 5.38 | 2.96 / 5.48 | 1.47 / 5.47 |
| Outdoor-2 okvis | 14.66 | 14.41 / 16.68 | 14.39 / 16.62 | 14.39 / 16.80 | 14.39 / 16.73 |
| Outdoor-2 xrslam | 14.66 | 14.55 / 16.99 | 14.53 / 16.79 | 14.52 / 16.83 | 14.52 / 16.81 |
| ADVIO-20 stella | 12.00 | 11.92 / 12.06 | 11.76 / 12.15 | 11.78 / 12.29 | 11.72 / 12.10 |
| ADVIO-20 orb3mono | 12.00 | 12.12 / 12.51 | 12.02 / 12.32 | 12.21 / 12.39 | 11.99 / 12.30 |
| ADVIO-20 okvis | 12.00 | 11.93 / 12.13 | 11.92 / 12.12 | 11.93 / 12.12 | 11.92 / 12.12 |
| ADVIO-20 xrslam | 12.00 | 11.99 / 12.14 | 11.99 / 12.13 | 11.99 / 12.13 | 11.99 / 12.13 |

Against the section-11 robust preset (24 cells = 12 cases x batch / causal): the generic prior improves 15, is within 0.05 m on 7, and worsens 2 (mean change -0.13 m, ATE of the fused output is GNSS-limited here: 5.7 / 14.7 / 12.0 m GNSS alone). The worst is Outdoor-2 ORB-SLAM3 causal, 2.58 -> 5.38 (the 96 s map, 9 nodes get distrusted once the scale state follows the gait; batch 2.64 -> 0.81 is the best cell). Per-user (held-out): 7 better / 6 worse / 3 neutral, mean +0.11 m; online: 7 / 8 / 9, mean +0.25 m (Outdoor-1 stella causal +2.4 m through k = 0.88); cross-user on ADVIO-20: 2 / 2 / 4, mean 0.00. Causal fused / GNSS alone: robust preset max 1.29x (mean 0.99), + generic gait max 1.25x (mean 0.97). So the gait prior is a modest net gain with the real fixes and a generic constant; it does not change the ranking: the full-coverage inputs (stella, OKVIS2, XRSLAM) stay within +-25 % of GNSS alone and the partial ORB-SLAM3 maps still win only because they are scored on their covered epochs (section 11.2).

### 12.4 Without GNSS: what the gait does to the odometry scale

SE3 / Sim3 ATE (m) / scale ratio (estimated / true, from the Sim3 alignment) / path ratio (path length / GT path length), batch and causal30. The fixes are removed from the run (indoors they are useless anyway, sigma 20-65 m). Outdoor rows show the long-walk scale behaviour with no GNSS to hide behind; the Sim3 column is the shape quality the gait cannot change.

| case | raw odometry | + gait generic batch | + gait generic causal30 | + gait per-user / cross-user batch | + gait per-user / cross-user causal30 |
|---|---|---|---|---|---|
| Outdoor-1 stella | 68.90/16.22/0.02/0.03 | 9.23/6.58/0.91/1.01 | 9.38/6.32/0.90/0.96 | 9.77/6.53/0.90/1.00 | 10.01/6.22/0.89/0.95 |
| Outdoor-1 orb3mono | 40.52/2.81/0.06/0.06 | 1.44/0.78/0.97/0.98 | 1.14/0.74/0.98/1.00 | 2.27/0.75/0.95/0.96 | 1.97/0.70/0.95/0.97 |
| Outdoor-1 okvis | 46.86/46.51/0.89/0.80 | 45.88/45.37/0.87/0.76 | 47.90/46.94/0.81/0.80 | 45.17/44.55/0.86/0.73 | 47.36/46.33/0.81/0.80 |
| Outdoor-1 xrslam | 6.44/6.10/0.97/1.00 | 6.25/5.72/0.96/0.99 | 4.47/4.31/0.98/0.96 | 6.13/5.10/0.95/0.98 | 4.06/3.68/0.98/0.96 |
| Outdoor-2 stella | 95.04/50.52/0.06/0.04 | 57.61/50.34/0.67/0.45 | 59.79/51.22/0.64/0.42 | 59.56/50.15/0.63/0.42 | 61.86/51.17/0.59/0.39 |
| Outdoor-2 orb3mono | 30.99/0.30/0.05/0.05 | 2.64/0.25/0.92/0.92 | 2.03/0.23/0.91/0.92 | 4.57/0.31/0.86/0.86 | 3.38/0.24/0.85/0.86 |
| Outdoor-2 okvis | 8943/86.70/185/90.16 | 99.67/56.55/2.00/1.00 | 319/51.94/4.71/2.48 | 77.33/57.64/1.64/0.86 | 171/50.48/2.91/1.75 |
| Outdoor-2 xrslam | 252767/67.14/3440/1492 | 261/62.77/4.28/2.03 | 246/59.35/4.00/2.06 | 240/62.76/4.00/1.89 | 227/59.39/3.76/1.94 |
| Indoor-1 stella (indoor) | 10.75/1.85/0.44/0.42 | 0.67/0.31/1.03/1.02 | 0.61/0.37/1.03/1.02 | 0.28/0.28/1.00/0.99 | 0.35/0.34/0.99/0.99 |
| Indoor-1 orb3mono (indoor) | 17.60/15.26/0.17/0.16 | 13.09/12.97/1.13/1.05 | 14.30/14.30/1.00/1.80 | 12.38/12.34/1.07/1.08 | 13.96/13.96/0.99/1.77 |
| Indoor-1 okvis (indoor) | 8.26/4.78/1.38/1.13 | 7.28/4.69/1.31/1.08 | 5.85/1.85/1.29/1.20 | 6.28/4.57/1.24/1.02 | 4.84/1.99/1.23/1.16 |
| Indoor-1 xrslam (indoor) | 0.84/0.73/0.98/0.99 | 0.79/0.74/0.98/0.99 | 0.67/0.36/0.97/0.99 | 0.83/0.73/0.98/0.99 | 0.76/0.36/0.96/0.98 |
| Indoor-2 stella (indoor) | 11.21/0.25/0.10/0.10 | 0.38/0.25/1.02/1.04 | 0.39/0.24/1.02/1.04 | 0.27/0.25/0.99/1.01 | 0.27/0.25/0.99/1.01 |
| Indoor-2 orb3mono (indoor) | 11.22/0.29/0.10/0.10 | 0.46/0.28/1.03/1.04 | 0.44/0.19/1.03/1.03 | 0.29/0.29/1.00/1.01 | 0.19/0.19/1.00/1.00 |
| Indoor-2 okvis (indoor) | 27.00/9.19/3.99/3.17 | 15.95/8.57/2.48/2.17 | 23.24/7.92/3.23/3.27 | 10.77/7.76/1.76/1.69 | 19.32/7.43/2.75/2.93 |
| Indoor-2 xrslam (indoor) | 0.98/0.56/0.93/0.92 | 0.80/0.55/0.95/0.94 | 0.91/0.38/0.93/0.93 | 0.73/0.54/0.96/0.95 | 0.85/0.37/0.93/0.94 |
| ADVIO-15 stella (indoor) | 1.16/0.81/0.39/0.35 | 1.32/1.20/1.53/1.09 | 0.54/0.54/0.99/0.98 | 1.32/1.21/1.53/1.08 | 0.54/0.54/0.98/0.96 |
| ADVIO-15 orb3mono (indoor) | 1.36/0.89/0.32/0.28 | 1.11/1.04/1.28/1.11 | 0.54/0.54/1.01/1.02 | 1.11/1.05/1.26/1.09 | 0.54/0.54/0.99/1.00 |
| ADVIO-15 okvis (indoor) | 1.65/1.65/0.96/0.14 | - | - | - | - |
| ADVIO-15 xrslam (indoor) | 1451/1.59/2036/248 | 12.47/1.66/24.44/2.05 | 4.84/1.06/6.46/4.12 | 11.68/1.66/22.70/1.94 | 4.68/1.06/6.27/3.95 |
| ADVIO-20 stella | 65.60/53.66/0.09/0.06 | 16.47/8.65/0.79/0.81 | 23.42/20.12/0.81/0.68 | 16.46/6.99/0.78/0.80 | 18.23/11.30/0.78/0.73 |
| ADVIO-20 orb3mono | 48.97/5.98/0.27/0.27 | 11.97/3.39/0.83/0.85 | 11.78/3.03/0.82/0.83 | 13.91/3.45/0.80/0.82 | 13.51/3.14/0.80/0.80 |
| ADVIO-20 okvis | 58.84/55.83/0.52/0.17 | 58.90/55.75/0.51/0.17 | 56.14/51.76/0.49/0.18 | 58.99/55.66/0.49/0.17 | 56.16/51.69/0.48/0.18 |
| ADVIO-20 xrslam | 143044/60.20/4602/1014 | 1178/61.73/43.11/9.60 | 1111/55.44/30.73/10.07 | 1119/61.72/40.97/9.12 | 1052/55.41/29.13/9.54 |

* **Camera-only mono (stella, ORB-SLAM3) gets metric scale.** Indoor-1/2 SE3 10.8 / 11.2 / 11.2 m -> 0.67 / 0.38 / 0.46 m with the generic constant (distance ratio 1.02-1.04) and 0.28 / 0.27 / 0.29 with the held-out per-user constant (0.99-1.01): at that point the error is the Sim3 shape error (0.25-0.31 m). Causal30 is as good as batch (0.35 / 0.27 / 0.19 per-user). Outdoor-1 ORB-SLAM3 (partial map) 40.5 -> 1.4 m, Outdoor-2 ORB-SLAM3 31.0 -> 2.6 m, ADVIO-20 ORB-SLAM3 49.0 -> 12.0 m (scale 0.83 although the gait itself is 7-9 % short for this person; Sim3 3.4 m), Outdoor-1 stella 68.9 -> 9.2 m (scale 0.91, Sim3 6.6 m over 507 m of walk with a 0.02-scale VO).
* **Limits.** Indoor-1 ORB-SLAM3 (6 tracking losses, Sim3 15.3 m before and 13.0 m after) is shape-limited. Outdoor-2 stella (VO shrinking ~4000x over the walk) is only partly rescued (scale 0.06 -> 0.67): the scalar state with the section-11 random walk cannot follow a continuously decaying scale. Metric VIOs whose scale diverged (Outdoor-2 OKVIS2 185x, XRSLAM 3440x, ADVIO-20 XRSLAM 4602x, ADVIO-15 XRSLAM 2036x, Indoor-2 OKVIS2 4.0x) improve by up to three orders of magnitude in scale (batch 2.0x, 4.3x, 43x, 24x, 2.5x) but are not metric, and the long ones have Sim3 shapes of 52-62 m anyway. Well-behaved metric VIOs (Indoor XRSLAM 0.93-0.98, Outdoor-1 XRSLAM 0.97) are slightly better (Indoor-1 SE3 0.84 -> 0.79 batch / 0.67 causal, Indoor-2 0.98 -> 0.80 / 0.91). ADVIO-15 is outside the model (7 epochs; stella / ORB-SLAM3 batch scale 1.53 / 1.28, causal 0.99 / 1.01 on the 30-52 s tail; OKVIS2 cannot be aligned: `gf_solve_batch` returns -2).
* **Robustness check, 'did it hurt where scale was fine'.** Outdoor-1 OKVIS2 (scale 0.89, SE3 46.9 m raw because of yaw drift) 45.9 m: unchanged, the scale was never the problem there.

### 12.5 No odometry at all: gait speed + gyro-heading PDR (+ GNSS)

PDR = gait odometer integrated along the gyro heading (10 Hz poses, heading offset absorbed by the smoother's yaw), smoother settings `preset=robust odom_sp=0.3 odom_kp=0.1 yaw_rw_deg=2 scale_rw=0.02 scale_prior=0.3` (physically reasoned, not tuned); no magnetometer, no camera.

| sequence | calibration | PDR alone | PDR + GNSS | GNSS alone |
|---|---|---|---|---|
| outdoor1 | gen | 16.59/13.28/1.14/0.98 | 5.26 / 5.71 | 5.73 |
| outdoor1 | user | 15.51/13.28/1.12/0.95 | 5.28 / 5.95 | 5.73 |
| outdoor1 | onl | 13.46/13.31/1.03/0.88 | 5.43 / 6.95 | 5.73 |
| outdoor2 | gen | 10.50/10.43/1.01/0.92 | 13.83 / 17.26 | 14.66 |
| outdoor2 | user | 11.72/10.43/0.95/0.86 | 13.84 / 18.03 | 14.66 |
| outdoor2 | onl | 11.02/10.14/0.96/0.88 | 13.87 / 17.82 | 14.66 |
| indoor1 | gen | 1.59/1.15/1.06/1.00 | - | - |
| indoor1 | user | 1.26/1.15/1.03/0.97 | - | - |
| indoor2 | gen | 2.09/1.70/1.10/1.01 | - | - |
| indoor2 | user | 1.88/1.70/1.06/0.98 | - | - |
| advio15 | gen | 2.80/1.58/4.01/0.50 | - | - |
| advio15 | cross | 2.71/1.58/3.87/0.48 | - | - |
| advio20 | gen | 22.77/22.45/1.06/0.84 | 11.74 / 11.40 | 12.00 |
| advio20 | cross | 22.49/22.45/1.02/0.81 | 11.83 / 11.46 | 12.00 |
| advio20 | onl | 22.80/22.47/1.06/0.84 | 11.74 / 11.40 | 12.00 |

PDR alone gives 1.3-2.1 m SE3 on the short indoor walks (Sim3 1.2-1.7 m, scale 1.03-1.10), 3-6x worse than stella / ORB-SLAM3 + gait (0.3-0.7 m), and is poor outdoors on long walks (gyro heading drift: Sim3 10-13 m over 500-600 m, 22 m on ADVIO-20). PDR + GNSS (batch) is 5.26 / 13.83 / 11.74 m versus GNSS alone 5.73 / 14.66 / 12.00 ; the camera-based fusions with gait reach 4.95 / 14.39 / 11.76 (best generic-gait batch of stella / OKVIS2 / XRSLAM), so PDR + GNSS is within 0.5 m of them (and lowest on Outdoor-2); causal30 5.71 / 17.26 / 11.40 (Outdoor-2 1.18x GNSS alone). With phone fixes of 5-14 m sigma, speed and heading alone buy about as much as any of the cameras on the full-coverage runs.

### 12.6 Zero-velocity factor

INSANE outdoor_1 (OKVIS2 odometry + PX4 GNSS, 55 s on the ground): neutral with the full odometry (batch 1.47 -> 1.47, causal 1.43 -> 1.41; the odometry is already still). With the odometry cut for t = 12..50 s (GNSS-only nodes, only the factor models motion): batch scatter of the poses about their mean 0.84 -> 0.02 m, rms error 9.48 -> 9.09 m (the 9 m PX4 receiver bias is not removable), causal 0.91 -> 0.89 (the causal output of a GNSS-only node is the estimate at the time it was made; the factor reaches only the nodes created after the measurement, which arrives with a 3 s granularity). Not evaluated on a phone (none of the six sequences stands still).

### 12.7 Python vs C, regression, timing

* `tools/check_gait.py`: C `gf_gait` vs `tools/gait.py` on all 6 IMU streams in three modes (generic, user constant, online with the fixes): identical states and step counts in all 18 runs, max |difference| of speed, cadence, k, heading, odometer 5e-10 (the print resolution of `gf_gait_run`).
* Baseline discipline: `gf_table.py` re-run with the new binary: all 608 numbers (19 cases x modes, SE3 and un-aligned) and all `gf_run` summary lines identical to the section-11 `table.json`; `compare_py.py` (8 cases) reproduces section 10.1 exactly (batch max 0.0008-0.0009 mm, causal 0.0131 / 0.0139 mm rtk / rtk_blk, 0.31 mm o1_okvis, o2_okvis chaotic as before); `test_geo.py` PASS. Gait off changes nothing: no node field is touched without `speed=1` and the speed measurements are ignored.
* Timing (one thread, this machine, loaded by other jobs): `gf_gait_push` + estimate 2.1 us per IMU sample including file parsing (39 400 samples in 83 ms); smoother `gf_add_odom` that creates a node, Outdoor-1 stella causal (393 nodes): 323 us mean / 301 p50 / 640 p99 -> 655 / 727 / 1019 us with the speed factors (a second window solve per speed epoch); `add_fix` unchanged; the whole fusion study (24 cases, up to 8 variants each, batch + causal) takes about 30 s on 12 workers.

### 12.8 Open issues

* Only one person per dataset and two phones: the 'per-user' result is one user with one held-out recipe; cross-user is one person. Step length changes with fatigue, shoes, carrying; running, stairs, escalators and holding styles other than hand-held are not covered (no data); cadence outside 1.0-2.8 Hz is dropped (ADVIO-15).
* The generic constant (and exponent) is a population guess; the online GNSS calibration is not useful with 14 m / 5-30 m phone fixes (Doppler velocity from raw Android GNSS, roadmap item 4, would give per-epoch speed and calibrate k directly).
* The gait covers scale only; yaw drift still limits long-walk shape (Outdoor-1 / -2 stella 6.3-6.6 / 50 m Sim3). Scale that decays continuously (stella Outdoor-2) or whole-map collapses (XRSLAM / OKVIS) need a faster scale state (`speed_scale_rw`) that costs accuracy elsewhere; not solved.
* Outdoor-2 ORB-SLAM3 causal regression (2.58 -> 5.38) and Outdoor-1 stella causal staying above GNSS alone (6.27 vs 5.73) are open; online GNSS calibration is not recommended with these fixes; the zero-velocity factor in causal mode is limited by the 3 s measurement cadence.
* Settings (scale clamp, relative random walk, `speed_align` gate) were chosen while looking at these sequences, the exponent 2 on the four Mobile sequences; ADVIO-20 and the drone are the only out-of-sample checks.


## 13. End-to-end phone pipeline `phone_pipeline/` (roadmap item 2, 2026-10-03)

Question: can the pieces (stella_vio multi-map ORB SLAM with gravity / gyro / R-frames / merge, `gf_gait` speed prior, `gf_run` robust fusion with the phone fixes) be chained into one deterministic, permissive, all-C pipeline that
gives a continuous metric trajectory on the six phone sequences, and how does it compare with GNSS alone, XRSLAM, RD-VIO and OKVIS2? Own code (`phone_pipeline/`, MIT; README there has the run commands), nothing from GPL sources read. Images were
re-fetched for this study with the section-9.2 scripts (Mobile-GVIO Indoor-1/2, Outdoor-1, first 450 s of Outdoor-2: 15 fps 1280x720; ADVIO-15/20: every 2nd frame, 30 fps 720x1280) and deleted after each sequence
(peak scratch + run disk below 8 GB, about 150 MB of results left in `runs/phone_pipeline/`: `tables.md`, `<seq>/scores.json`, `<seq>/{sv_*,fuse_*}`).

### 13.1 Pipeline

`sv_run` (images + IMU, `--set gravity=1 rframe=1 merge=1 gyro=1`, extrinsic / time offset / gyro bias of the existing fits) writes per-frame poses with map id, R-frame flag and segment id and the gravity-aligned trajectory of every map. The adaptor
(`run.py make_odom`) turns it into a `gf_run` odometry stream: a new map id is `GF_ODOM_NEW_FRAME` (own scale / yaw / origin: every monocular map gets its own unit, aligned from its own fixes, or from the gait speed where there are none), a new segment of the same map
(a part bridged through a rotation-only stretch, its scale only a speed prior) is `GF_ODOM_GAP | GF_ODOM_LOOSE`, an R-frame sample (extrapolated position) is `GF_ODOM_LOOSE` (link sigma x5), tracking gaps > 2 s are detected by `gf_run` itself and
filled with GNSS-only nodes where fixes exist. `gf_gait_run` makes the walking speed measurements from the same phone IMU (3 s epochs, 6 s window); `gf_run` uses the robust preset + `speed=1 speed_align=1 speed_scale_rw_rel=1 speed_align_metric=1`, the fixes (outdoor
sequences only) and runs twice: batch (`batch.out`) and causal (`causal.live` = pose returned by `gf_get_pose` after every sample, only samples with the INIT bit; the first 30 s with fixes / 12 s without fixes are the alignment phase and not scored, as in sections 11/12).
Scoring: `gnss_eval.score` (`benchmark.umeyama_alignment`) on the output resampled to the camera-frame times (bridged / GNSS-only stretches count; linear interpolation between neighbouring poses <= 2.5 s apart; coverage = frames with a pose / frames). SE3 = rigid alignment, Sim3 = with scale, scale = estimated / true from the Sim3 fit.
The phone GT frames are LiDAR / dataset frames, not ENU, so there is no geo error against the GT; the geo-referencing check is the rms distance of the output to the fixes themselves.

### 13.2 Results (one run each, deterministic: re-running fusion gives byte-identical outputs; stella_vio is deterministic by construction)

### Headline: full pipeline (stella_vio gyro+R-frames+merge, gait, GNSS where usable), ATE SE3 [m], batch / causal

| sequence | full pipeline batch / causal | Sim3 batch / causal | scale ratio (est/true) batch | coverage batch / causal | rms distance to the fixes (geo-referencing check) batch / causal | GNSS alone | XRSLAM | RD-VIO (xrsetting) | OKVIS2-X | best earlier fusion, full-coverage odometry (batch / causal30) |
|---|---|---|---|---|---|---|---|---|---|---|
| Indoor-1 | **1.16 / 1.02** | 1.15 / 0.94 | 0.99 | 100% / 100% (90% of all frames) | n/a (no fixes) | n/a | 0.84 | 0.83 | 8.26 | 0.28 (i1_stella) / 0.35 (i1_stella) |
| Indoor-2 | **0.31 / 0.31** | 0.30 / 0.31 | 0.99 | 100% / 100% (88% of all frames) | n/a (no fixes) | n/a | 0.98 | 1.06 | 27.00 | 0.27 (i2_stella) / 0.27 (i2_stella) |
| Outdoor-1 | **5.50 / 7.10** | 5.50 / 6.71 | 1.00 | 100% / 100% (92% of all frames) | 2.50 / 9.63 | 5.73 | 6.44 | 4.77 | 32712 | 4.95 (o1_stella) / 5.82 (o1_xrslam) |
| Outdoor-2 | **13.28 / 15.10** | 13.12 / 9.55 | 0.98 | 100% / 100% (93% of all frames) | 5.44 / 17.81 | 14.66 | 252768 | 254661 | 8943 | 14.39 (o2_okvis) / 16.62 (o2_okvis) |
| ADVIO-15 | **0.90 / 0.66** | 0.83 / 0.65 | 1.24 | 93% / 85% (66% of all frames) | n/a (no fixes) | n/a | 1451 | 907.64 | 1.65 | 1.32 (a15_stella) / 0.54 (a15_stella) |
| ADVIO-20 | **11.76 / 12.32** | 4.97 / 5.27 | 0.84 | 100% / 100% (90% of all frames) | 2.89 / 4.80 | 12.00 | 143044 | 32225 | 58.84 | 11.72 (a20_stella) / 12.06 (a20_stella) |

GNSS alone = the raw fixes scored the same way (SE3); XRSLAM / RD-VIO (`xrsetting`, tuned for these sequences) / OKVIS2-X = the raw trajectories of sections 9, 10 and `runs/gnss_compare/more_systems*` (metric VIOs, SE3; XRSLAM/RD-VIO scale-collapse on Outdoor-2 and ADVIO, OKVIS2-X diverges on Outdoor-1/2); the last column is the best of the earlier
gnss_fusion tables (section 12.3 / 12.4 incl. gait variants, all odometry systems except the partial ORB-SLAM3 maps, batch / causal30 scored at the odometry sample times, i.e. a different odometry; for Indoor-1 that is stella upstream with the low-FAST config, which covered only 74 % of the sequence (30-119 s), so 0.28 is not like for like).

### Ablation (ATE SE3 [m] batch / causal; first row Sim3 per map because its scale is arbitrary)

| configuration | Indoor-1 | Indoor-2 | Outdoor-1 | Outdoor-2 | ADVIO-15 | ADVIO-20 |
|---|---|---|---|---|---|---|
| stella_vio default, camera only (Sim3 per map, batch only) | 0.88 (100%) | 0.23 (99%) | 45.74 (100%) | 30.84 (99%) | 0.61 (85%) | 39.43 (98%) |
| + gait speed prior (no GNSS) | 0.83 / 0.68 | 0.31 / 0.33 | 7.35 / 6.02 | 37.50 / 25.18 | 1.58 / 0.87 | 14.85 / 18.03 |
| + GNSS fixes (no gait) | n/a (no usable fixes) | n/a (no usable fixes) | 5.53 / 6.85 | 13.73 / 18.54 | n/a (no usable fixes) | 11.86 / 12.08 |
| + gait + GNSS | 0.83 / 0.68 | 0.31 / 0.33 | 5.67 / 7.88 | 13.66 / 26.13 | 1.58 / 0.87 | 11.78 / 12.24 |
| + R-frames + merge, + gait + GNSS | 0.83 / 0.68 | 0.31 / 0.33 | 5.50 / 8.79 | 14.06 / 18.81 | 0.89 / 0.69 | 11.73 / 12.22 |
| full = + gyro prior (R-frames, merge, gait, GNSS) | 1.16 / 1.02 | 0.31 / 0.31 | 5.50 / 7.10 | 13.28 / 15.10 | 0.90 / 0.66 | 11.76 / 12.32 |
| full with the dataset-calibration extrinsic instead of the sequence-fitted one (sensitivity) | 0.64 / 0.62 | 0.28 / 0.27 | 8.13 / 12.26 | 13.32 / 15.03 | 0.85 / 0.67 | 11.76 / 12.32 |
| full, gait only (no GNSS; metric, not geo-referenced) | 1.16 / 1.02 | 0.31 / 0.31 | 6.37 / 5.16 | 13.93 / 13.68 | 0.90 / 0.66 | 11.83 / 11.76 |
| full, GNSS only (no gait) | n/a | n/a | 5.54 / 6.98 | 13.38 / 14.28 | n/a | 11.88 / 12.51 |

Row 1 is camera-only (arbitrary scale: Sim3 with one similarity per map, coverage in brackets). Rows 2-7 are `gf_run` outputs. "full, gait only" and "full, GNSS only" switch one input off. The whole matrix (every variant x mode, Sim3, scale, coverage, tracked-only, distance to the fixes) is in `runs/phone_pipeline/tables.md`.

### Timing (CPU seconds, shared machine; sv_run = whole stella_vio run incl. ORB extraction at full resolution, single thread)

| sequence | frames | sv_run default CPU s (ms/frame) | sv_run full CPU s (ms/frame) | gait us/IMU sample | gf_run batch s | gf_run causal s (us/frame) |
|---|---|---|---|---|---|---|
| Indoor-1 | 1779 | 72 (40) | 73 (41) | 23.5 | 0.01 | 0.02 (11) |
| Indoor-2 | 1535 | 56 (37) | 57 (37) | 4.7 | 0.01 | 0.02 (11) |
| Outdoor-1 | 5898 | 362 (61) | 363 (62) | 1.4 | 0.04 | 0.24 (41) |
| Outdoor-2 | 6737 | 449 (67) | 457 (68) | 1.4 | 0.04 | 0.31 (46) |
| ADVIO-15 | 1553 | 58 (37) | 56 (36) | 4.7 | 0.01 | 0.02 (11) |
| ADVIO-20 | 9076 | 615 (68) | 619 (68) | 1.4 | 0.05 | 0.22 (24) |

### gf_run call latencies (full pipeline, causal run, one thread; us)

| sequence | add_odom without node (mean) | add_odom creating a node (mean / p99 / max) | add_fix (mean / p99) |
|---|---|---|---|
| Indoor-1 | 0.06 | 44.14 / 86.68 / 131.15 | - |
| Indoor-2 | 0.10 | 46.33 / 106.62 / 112.01 | - |
| Outdoor-1 | 0.06 | 519.14 / 849.32 / 2726.79 | 0.05 / 0.13 |
| Outdoor-2 | 0.06 | 247.32 / 359.32 / 4695.89 | 257.10 / 391.21 |
| ADVIO-15 | 0.09 | 52.37 / 161.20 / 169.00 | - |
| ADVIO-20 | 0.06 | 520.18 / 865.58 / 3051.68 | 1.07 / 0.14 |

Per frame CPU of the whole pipeline is dominated by stella_vio at full resolution (36-68 ms per frame single thread on this loaded 16-thread machine: about real time for the 15 fps Mobile-GVIO streams, 2x too slow for 30 fps ADVIO); gait about 1.4 us per IMU sample unloaded (the 4-24 us entries are
machine noise: the table takes the last run); `gf_run` < 0.6 ms per node creation (mean) and < 5 ms worst case, batch solve < 0.1 s for the whole sequence. Machine shared with other jobs, CPU time not wall.

### stella_vio events per sequence and variant (from sv_run logs): lost frames / resets / maps / R-frames / bridges / loops accepted (the final trajectory used as odometry contains the SLAM back-end corrections of these loops)

| sequence | default | rm | full | fullcal |
|---|---|---|---|---|
| Indoor-1 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 |
| Indoor-2 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 | 0 / 0 / 1 / 0 / 0 / 1 |
| Outdoor-1 | 3 / 0 / 1 / 0 / 0 / 0 | 0 / 0 / 1 / 66 / 1 / 0 | 0 / 0 / 1 / 26 / 1 / 0 | 0 / 0 / 1 / 0 / 0 / 0 |
| Outdoor-2 | 43 / 2 / 2 / 0 / 0 / 0 | 1 / 1 / 1 / 12 / 0 / 0 | 1 / 1 / 1 / 0 / 0 / 0 | 1 / 1 / 1 / 0 / 0 / 0 |
| ADVIO-15 | 103 / 1 / 2 / 0 / 0 / 0 | 0 / 0 / 1 / 87 / 1 / 1 | 0 / 0 / 1 / 42 / 1 / 1 | 0 / 0 / 1 / 43 / 1 / 1 |
| ADVIO-20 | 62 / 1 / 2 / 0 / 0 / 0 | 0 / 0 / 1 / 65 / 1 / 0 | 0 / 0 / 1 / 66 / 1 / 0 | 0 / 0 / 1 / 66 / 1 / 0 |

Loops accepted: only the Indoor sequences (1 each, `full`) and ADVIO-15 (1). The odometry handed to the fusion is the *final* stella_vio trajectory (including those back-end corrections and merges), so the causal columns are causal in the fusion, not in the SLAM back end; a per-frame live
pose stream was not written (needs the map labels at the time of the frame), and its difference is not measured; outdoors, where no loop was accepted, only the local BA / keyframe corrections of the SLAM differ.

### 13.3 Reading

* **Against XRSLAM / RD-VIO (SE3, metric)**: ahead on 4 of 6 sequences. Indoor-2 0.31 vs 0.98 / 1.06 (3x), ADVIO-15 0.90 / 0.66 vs ~1000 (both collapse to scale 0.000-0.001; OKVIS2-X 1.65), Outdoor-2 and ADVIO-20 (13.3 / 15.1 and 11.8 / 12.3 m vs XRSLAM 2.5e5 / 1.4e5 and RD-VIO 2.5e5 / 3.2e4 m: their metric scale collapses on the long walks, the
  gait-anchored stella_vio map does not) and equal to XRSLAM on Outdoor-1 (5.50 / 7.10 vs 6.44; RD-VIO with its sequence-tuned `xrsetting` 4.77 is better). Behind on Indoor-1 with the `full` config (1.16 / 1.02 vs 0.84 XRSLAM, 0.83 RD-VIO); with the default stella_vio it is level (0.83 / 0.68 batch / causal) and with the dataset-calibration extrinsic better (`fullcal` 0.64 / 0.62).
  Local accuracy (Sim3, Indoor-2 0.30, Indoor-1 0.8-1.15) is the monocular stella Sim3 quality; the metric part (scale 0.99 on both Indoor sequences) comes entirely from the gait speed.
* **Against GNSS alone (5.73 / 14.66 / 12.00 m)**: batch is marginally better on all three outdoor sequences (5.50, 13.28, 11.76: -4 %, -9 %, -2 %); causal live is **not** (7.10, 15.10, 12.32: +24 %, +3 %, +3 %). The target "beat GNSS-alone causal on Outdoor-1/2, ADVIO-20" is **not met** with fixes in the loop.
  The same pipeline **without** the fixes (gait scale only, "full, gait only"), which is metric but not geo-referenced, is below GNSS alone in causal mode on all three (5.16, 13.68, 11.76; batch 6.37, 13.93, 11.83) and Sim3 1.92 m on Outdoor-2 (a 587 m walk; its SE3 13.9 m is the 14 % gait scale deficit of that user: scale 0.86), 4.4 m on ADVIO-20.
  With 14 m fixes (bias random walk 0.85 m/sqrt(s)) the fusion cannot beat what the gait + visual shape already gives; it adds the geo-reference (distance to the fixes 2.5-5.4 m batch) and a safety net, and in causal mode the fixes cost 1.9 m on Outdoor-1 (7.10 with, 5.16 without).
* **What each part buys** (ablation): the gait prior is what makes the camera-only maps metric (Indoor-1/2 scale 0.99, SE3 0.83 / 0.31 from Sim3 0.88 / 0.23; Outdoor-1 45.7 m Sim3 -> 7.4 m SE3 without fixes). On Outdoor-2 and ADVIO-20 the default stella_vio splits into two maps (Lost 43 / 62 frames, 2 and 2 maps) and the gait+default result is useless
  (37.5 m, 14.9 m) because the second map's unit is not recovered; R-frames + merge + gyro keep one map (0 / 1 Lost frames, 1 map) and bring the gait-only result to 13.9 / 11.8 m. R-frames + merge alone are not enough on Outdoor-2 (the gait-only run lands on scale 0.77, 24.9 m), the gyro prior closes it. On Outdoor-1 the full config is 5.50 vs 5.67 (default) batch, 7.10 vs 7.88 causal.
  ADVIO-15: only the R-frame/merge variants survive (default has two maps and a 103-frame Lost stretch: 1.58 / 0.87 vs 0.89 / 0.69).
* **Chaos**: small input changes move single-run numbers by 40-100 % (Indoor-1 full 1.16 / default 0.83 / dataset-calibration extrinsic 0.64; Outdoor-1 fullcal 8.13 / 12.26 vs full 5.50 / 7.10). The monocular front end is chaotic (stella_vio/RESULTS.md); one run per row, so only >2x differences and the structural findings (single map, metric scale) are rankings.

### 13.4 Calibration honesty
* Gait: Mobile-GVIO sequences use a per-user constant fitted on the GT speed of the other three Mobile sequences only (leave-one-out); ADVIO uses the generic 0.389 (its person is 8 % above it: scale 0.84 on ADVIO-20, ADVIO-15 is outside the model with 7 epochs and a 1.24 batch scale).
* **Chosen / fitted on the test sequences themselves** (the task prescribed "from the existing fits"): camera-IMU rotation and time offset (`ext_fit.py` on the sequence's own visual trajectory + gyro; the dataset-calibration variant is the `fullcal` row), gyro bias (first <= 60 s of the sequence), the stella_vio defaults and R-frame / gyro parameters (tuned on `complex_environment` and Outdoor-1), the gf robust preset and gait settings (sections 11/12, same sequences),
  `loose_k = 5` (set a priori, not tuned; no run without it), `speed_align_metric` (added after the ADVIO-15 batch run failed with "fewer than 3 usable fixes": a test-driven fix, ADVIO-15 only affected) and the 12 s alignment wait of fix-free runs (below).
* The headline `full` config was fixed before any result was seen (it is "everything on"); it is not the best variant on every sequence (Indoor-1: default 0.83, fullcal 0.64; Outdoor-1: default+gnss 5.53). Two settings were added after seeing a first result: `speed_align_metric` (above) and the 12 s alignment wait of fix-free runs (ADVIO-15's causal output would otherwise start at 34 s of a 52 s sequence; 12 s is the only value tried). No other setting was changed after seeing a result.
* Exploratory, not adopted (all on `full`, both, outdoor sequences, batch / causal; baseline 5.50 / 7.10, 13.28 / 15.10, 11.76 / 12.32): `trust=0` 5.45 / 7.77, 9.37 / 14.65, 11.84 / 12.35 (Outdoor-2 batch better, nothing else); `trust_state_k=0` 5.43 / 7.95, 12.91 / 14.65, 11.84 / 12.35; fix sigma x2: 5.55 / 7.46, 12.92 / 17.70, 11.71 / 12.75; x4: 6.62 / 9.76, 12.63 / 18.80, 11.93 / 12.28. None improves the causal numbers materially (best: `trust=0` on Outdoor-2 14.65 vs 15.10, while Outdoor-1 gets worse, 7.77).

### 13.5 Baseline checks (all opt-in switches off, `phone_pipeline/check_baselines.sh`, log `runs/phone_pipeline/check_baselines.log`)
stella_vio fr1_xyz (`--set reinit_sec=0 init_max_level=0 init_confirm=1`, built per `tools/run_stella_port_replay.py`) `trajectory.tum` `cmp`-identical to stella_port's sv_run (787 poses) with the new `--wait-fixtures` option present; gnss_fusion: `gf_table.py` 893 numbers and `gf_gait_study.py fusion` 1578 numbers identical to the section-11 / 12 values saved before the change, `compare_py.py` (8 cases) and `test_geo.py` PASS.

### 13.6 Open issues
* Causal is worse than GNSS alone on all three outdoor sequences (+3 .. +24 %). The gait-only stream is better, so the weakness is how the 14 m fixes (correlated, biased) enter the 30 s window; a rigid-only (yaw + translation) geo-referencing of a gait-scaled stream, or a bias state per fix source, is the next experiment (not done here). **Done in section 14: a slowly varying similarity geo-referencing (`gf_georef`) makes the causal output 2-23 % better than GNSS alone on all three (rigid-only is not enough, the scale ridge is what wins).**
* The first 30 s of a causal run with fixes (12 s without) are unaligned (88-93 % of all frames covered; ADVIO-15 66 %); no pose before the first stella initialisation (about 4 s on ADVIO-15 -> batch coverage 93 %); pure PDR bridging of those stretches is not wired in.
* Scale is only as good as the gait constant: ADVIO-20 0.84 (a longer step person), ADVIO-15 outside the model; running / stairs / non-walking not covered (OTHER state gives no measurement). Long monocular maps keep their Sim3 shape error (Outdoor-1 5.4 m, ADVIO-20 4.4-5 m).
* The odometry is the final stella_vio trajectory (not a per-frame live stream), stella_vio at full resolution is about real time at 15 fps only, and the fusion treats a merge or loop correction as ordinary odometry.
* Single run per row, chaotic front end; everything is on six sequences of two datasets (ADVIO is CC BY-NC 4.0: internal benchmark only).

## 14. Live (causal) fusion that beats GNSS alone: slowly varying geo-referencing of the gait stream (2026-10-04)

Question (open issue 13.6): the section-13 causal output is 3-24 % worse than GNSS alone on Outdoor-1 / Outdoor-2 / ADVIO-20 while the fix-free gait stream is better. Why, and what fixes it without hurting batch or the other cases?
Only the fusion stage was re-run, on the saved pipeline inputs (`runs/phone_pipeline/<seq>/fuse_full_*/{odom,fix}.txt`, `speed.txt`; stella_vio was not re-run). Harness: `phone_pipeline/fuse_eval.py` (gf_run / gf_georef_run in a scratch dir, scored by `score.py`, the same code as the section-13 tables).

### 14.1 What the phone fixes are (measured, GT only used for this diagnosis)

Fix error after the best rigid fit to the GT (horizontal rms 5.6 / 13.9 / 8.6 m on Outdoor-1 / Outdoor-2 / ADVIO-20): a 5 s moving average removes nothing of it (low-pass rms 5.4 / 13.9 / 8.6 m, high-pass 0.35-0.45 m), lag-1 autocorrelation 0.99, lag 30 s 0.39-0.58, lag 60 s -0.02 .. 0.23.
So the fixes are a slowly varying common error (correlation time about 40 s) plus 0.3-0.5 m of white noise; the reported sigma is no use (Mobile-GVIO: constant 14.25 m; ADVIO-20: reported 5 m -> 8.6 m rms error, reported 10 m -> 8.4 m, i.e. uninformative).
The smoother's causal 30 s window therefore re-estimates yaw / scale / offset from fixes whose errors are common to the whole window: the causal yaw state differs from the batch yaw by -7 .. +9 deg (Outdoor-1), up to 30 deg (Outdoor-2), 10 deg (ADVIO-20) over the run (`*.nodes`), which bends the shape that the gait-scaled odometry had right.
Information bound: with the yaw taken from the full run (oracle) and only a causal translation offset from the fixes (exponentially weighted mean, time constants 30 s .. infinity) the SE3 error is 6.1 .. 5.6 / 13.7 .. 12.3 / 12.0 .. 11.8 m, i.e. at best level with GNSS alone: an offset taken from correlated fixes cannot beat the fixes. What does beat them is the long baseline: **yaw and scale** of the metric gait stream are determined by hundreds of metres of path and the offset error stays at the level of the fix bias, while the shape comes from the gait stream.

### 14.2 What was built (opt-in; `gnss_fusion/c/gf_georef.{h,c}`, driver `gf_georef_run.c`, MIT, C99, `<stdint.h> <math.h> <stdlib.h> <string.h> <limits.h>` only)

Two stages: (A) the existing smoother runs **without the fixes** (gait speed prior only: `fuse_<variant>_gait`, causal live + batch), (B) `gf_georef` maps that stream into ENU with ONE similarity (yaw psi, scale s, 2-D translation, vertical offset) estimated from all (stream position at the fix time, fix) pairs seen so far (causal) or all pairs (batch). Fixes never bend the shape.
`psi` = 2-D Procrustes angle (Huber re-weighted, threshold 2.5 x a median-based residual scale), `s = (S_cr/k + lam) / (S_aa/k + lam)` with `lam = sigma_res^2 / sigma_s^2`: the scale is shrunk towards 1 (gait stream is metric to ~15 %, `sigma_s = 0.15`), `sigma_res` = residual rms of the fit (online noise calibration; the reported sigmas are not used, `use_sigma=0`), `k = corr_s x fix rate = 40` fixes per independent sample (coloured-noise correction of the information in S_aa). A pair is only formed when the stream is continuous around the fix time (`max_gap_s = 2.5`), the fit starts with 8 pairs and a 30 m extent. Causal: the pose at each stream sample is mapped with the fit that exists at that moment (no revision of old output); `test_georef.py` checks that the first half of the causal output is byte-identical when the second half of stream and fixes is removed.
Use: `phone_pipeline/run.py georef <seq>` (writes `fuse_<variant>_georef/`), `gf_georef_run --stream live.txt --fix fixes.txt --out out.txt --mode causal|batch [key=value]`.

### 14.3 Result (ATE SE3 [m], batch / causal live; causal scored from +30 s as in section 13; one deterministic run each)

| configuration | Outdoor-1 | Outdoor-2 | ADVIO-20 | mean vs GNSS alone, batch / causal |
|---|---|---|---|---|
| GNSS alone | 5.73 | 14.66 | 12.00 | |
| full, gait only (no fixes, not geo-referenced) | 6.37 / 5.16 | 13.93 / 13.68 | 11.83 / 11.76 | +1.6 % / -6.2 % |
| full + gait + GNSS in the smoother (section 13 default, unchanged) | 5.50 / 7.10 | 13.28 / 15.10 | 11.76 / 12.32 | -5.1 % / +9.8 % |
| **full, gait stream + georef (this section, defaults)** | **5.36 / 4.71** | **4.81 / 11.34** | **11.76 / 11.79** | **-25.2 % / -14.1 %** |
| vs GNSS alone per sequence | -6.5 % / -17.8 % | -67 % / -22.6 % | -2.0 % / -1.8 % | |

**Causal now beats GNSS alone on all three outdoor sequences, and batch is not hurt** (5.36 / 4.81 / 11.76 vs the unchanged smoother's 5.50 / 13.28 / 11.76; the smoother batch stays available and unchanged). Sim3 of the georef rows 5.35 / 1.92 / 4.37 batch, 4.60 / 9.98 / 4.72 causal; rms distance to the fixes 7.3 / 15.6 / 6.0 m (batch), 8.2 / 16.7 / 6.0 m (causal): the output lives in the ENU frame of the fixes. Coverage 99.6-100 % batch, 98-99.7 % causal (scored from +30 s; the first aligned output is at 31 / 37 / 33 s, 92 / 92 / 89 % of all frames).
Honest reading: ADVIO-20's 2 % is within the run-to-run noise of the front end (the output scale is the 0.84 gait scale of that person, the fixes do not correct it: free scale gives the same 11.75); Outdoor-1's gain is the shape of the gait stream + averaged geo-reference; Outdoor-2's gain is mostly the scale: the gait constant of that user is 14 % short (13.9 m of gait-only SE3), the 450 fixes over 587 m recover it (scale 0.96 vs true, Sim3 1.92 m batch).
The result depends on the stream: the other stella_vio variants of the same pipeline (same table in `runs/phone_pipeline/tables.md`): default + georef 4.71 / 4.99, 37.03 / 17.81, 13.70 / 13.14 (the default config splits Outdoor-2 / ADVIO-20 into two maps and the gait-only stream is already bad: 37.5 m), rm + georef 4.96 / 5.45, 13.36 / 13.56, 13.68 / 13.16, fullcal + georef 13.43 / 10.35, 4.78 / 11.38, 11.76 / 11.79 (Outdoor-1 fullcal is the chaotic 8.13 / 12.26 case of section 13 for the gait-only stream as well).

Variants of the georef itself (same table layout; sweep `phone_pipeline/fuse_eval.py "label|G: key=val"`, log in `runs/phone_pipeline/` is not kept, numbers reproducible):

| variant | Outdoor-1 | Outdoor-2 | ADVIO-20 | mean batch / causal vs GNSS |
|---|---|---|---|---|
| default | 5.36 / 4.71 | 4.81 / 11.34 | 11.76 / 11.79 | -25.2 % / -14.1 % |
| rigid (scale = 1, `scale_sigma=0`) | 6.37 / 5.40 | 13.93 / 13.94 | 11.83 / 12.27 | +1.6 % / -2.8 % |
| free scale (`scale_sigma=100`) | 5.37 / 4.57 | 3.34 / 11.12 | 11.75 / 11.75 | -28.5 % / -15.5 % |
| weights 1/sigma_reported^2 | 5.36 / 4.71 | 4.81 / 11.34 | 12.00 / 11.93 | -24.6 % / -13.6 % |
| no Huber | 5.36 / 4.70 | 4.81 / 11.21 | 11.64 / 11.81 | -25.5 % / -14.4 % |
| forgetting 600 / 300 / 120 s | 5.37 / 4.89, 5.38 / 5.06, 5.41 / 5.43 | 5.49 / 11.60, 6.46 / 11.93, 9.68 / 13.18 | 11.83 / 11.84, 11.86 / 11.89, 11.73 / 11.95 | -23.4 / -12.3, -21.0 / -10.4, -13.9 / -5.2 % |
| min_extent 15 / 60 m (60 m: first output later) | 5.36 / 4.73, 5.36 / 4.24 | 4.81 / 11.38, 4.81 / 10.60 | 11.76 / 11.76, 11.76 / 11.78 | -25.2 / -14.0, -25.2 / -18.5 % |

### 14.4 The four ideas

1. **Rigid geo-referencing of the gait-scaled stream with a slowly varying offset: accepted in a modified form.** Rigid (yaw + translation, scale 1) is only -2.8 % causal and +1.6 % batch (it cannot correct the gait scale, Outdoor-2 13.9 m); a similarity whose scale is shrunk towards 1 (`sigma_s = 0.15`) is what wins. Long memory (no forgetting) is best on these walks (10 min); forgetting 120-600 s costs 2-9 points (a drifting odometry would want it: the knob exists, default off).
2. **GNSS bias state / coloured noise: not implemented as a state, tested two ways, rejected.** (a) The georef treats the fixes as white but lowers the weight of the scale information by the number of fixes per correlation time (`corr_s`, 1..80 s sweep: 12.2-14.6 % causal for all, i.e. insensitive; `corr_s=1` is even slightly better). (b) Exact generalised least squares with AR(1) fix errors (whitening by `z_i - phi_i z_{i-1}`, tau_c 40 / 80 s; python prototype `gnss_fusion/tools/georef_gls_proto.py`, run on the same gait stream): 10.36 / 10.46 / 11.22 and 13.83 / 10.95 / 11.58 vs white 5.02 / 11.06 / 11.51 (Outdoor-1 / -2 / ADVIO-20): worse, the whitened rows lose the long-baseline information (differences of nearly collinear rows) and the fix error is not a stationary AR(1). A Gauss-Markov bias state inside the 5-state smoother would need 7-state blocks (2.7x solver cost) and the measurements above say that the offset it would absorb is exactly what the long-memory fit averages anyway.
3. **Longer causal window / marginalisation in the smoother: rejected** (`gf_run`, section-13 inputs, SE3 batch / causal): window 60 / 120 / 240 s 5.50 / 6.69, 6.31, 6.30 (Outdoor-1), 13.28 / 17.85, 17.27, 17.30 (Outdoor-2), 11.76 / 12.15, 12.14, 12.13 (ADVIO-20) vs 7.10 / 15.10 / 12.32: Outdoor-1 -11 %, Outdoor-2 +14-18 %. yaw_rw 0.1 / 0.02 deg/sqrt(s): causal 6.09 / 21.58 / 12.64 and 6.01 / 22.58 / 12.31 (a wrong first alignment is frozen). Window 120 + yaw_rw 0.1: 6.70 / 14.17 / 13.01; window 240 + yaw_rw 0.02: 6.42 / 14.55 / 13.14; + trust=0: 6.10 / 16.45 / 12.16 and 6.13 / 13.83 / 12.92 (batch 4.82 / 7.90 / 11.92 for window 240 + yaw_rw 0.1 + trust=0, i.e. -21 % batch, but +3 % causal); `grow_s_mono=120` 7.30 / 13.83 / 12.36, + yaw_rw 0.1 9.33 / 11.22 / 14.85. No setting is better than GNSS alone on all three in the smoother (best mean +3.0 % causal), and every one trades one sequence against another (full list in `docs/rejected_trials.md`). The causal mean of the best smoother setting stays 17 points above the georef.
4. **Fix weighting, online sigma calibration, gating: partly accepted.** Reported sigma weights 1/sigma^2: no help (ADVIO-20 12.00 / 11.93 vs 11.76 / 11.79) because the reported sigmas are uninformative (14.1); the residual rms of the fit is the calibrated noise (it also sets the scale ridge). Gating against the odometry-predicted position = Huber re-weighting on the fit residual (threshold 2.5 x median-based sigma): neutral on the phones (no outlier bursts there, 4.70 vs 4.71), clearly helps on a synthetic 30 s burst of 60 m outliers (max error 1.83 vs 4.24 m, `test_georef.py`), kept as default. In the smoother: `gate_chi2=4` 7.10 / 15.28 / 12.35, `loss_k=1` 7.10 / 15.54 / 12.37 (vs 7.10 / 15.10 / 12.32): no help.

### 14.5 Settings, calibration honesty, leave-one-sequence-out

Physically set (not tuned on these runs): `sigma_s = 0.15` (gait scale uncertainty, section 12: 8-14 % per user), Huber 2.5 (same as the fix loss), `max_gap_s = 2.5` (the scorer's bracket), equal weights (14.1). **Measured on these three sequences:** `corr_s = 40` (autocorrelation, 14.1), `min_extent = 30` m and `min_fixes = 8` (a-priori round values; 15 and 60 m tried above). The fix-error statistics of 14.1 use the GT.
Leave-one-sequence-out over the grid `corr_s {1, 20, 40, 80} x sigma_s {0.1, 0.15, 0.3} x Huber {0, 2.5}` (24 settings, final code): **every one of the 24 settings beats GNSS alone causally on every sequence** (worst per-sequence causal ratio 0.989, batch 0.980); the setting picked on the other two sequences scores on the held-out one (causal / batch vs GNSS alone): Outdoor-1 4.76 (-17.0 %) / 5.37 (-6.3 %), Outdoor-2 11.17 (-23.8 %) / 6.21 (-57.6 %), ADVIO-20 11.80 (-1.7 %) / 11.75 (-2.0 %). The conclusion does not depend on a setting; the size of the Outdoor-2 batch gain does (3.3 .. 7.9 m, scale prior).
The stream itself carries the section-13.4 calibration remarks (sequence-fitted camera-IMU extrinsic / time offset / gyro bias, per-user gait constant fitted on the other Mobile sequences). Three sequences, two datasets, one front-end run each: a 2 % gain (ADVIO-20) is not a ranking.

### 14.6 Earlier case list (`gnss_fusion/tools/gf_georef_table.py`; stream = raw odometry of the case, metric: sigma_s 0.15, monocular: free scale; ATE SE3 [m]; smoother = `gf_table` robust preset batch / causal30 / live)

| case | GNSS alone | smoother batch / causal30 / live | georef batch / causal (coverage) |
|---|---|---|---|
| complex_rtk | 0.00 | 0.08 / 0.09 / 0.10 | 2.69 / 2.16 (100%) |
| complex_sim | 4.18 | 2.04 / 2.16 / 2.15 | 2.70 / 2.56 (100%) |
| complex_rtk_blk | 0.00 | 0.26 / 0.67 / 0.67 | 2.70 / 1.78 (100%) |
| complex_sim_blk | 4.33 | 1.88 / 2.17 / 2.16 | 2.69 / 2.01 (100%) |
| o1_okvis | 5.73 | 6.14 / 7.39 / 7.40 | 46.71 / 42.16 (99%) |
| o2_okvis | 14.66 | 14.41 / 16.68 / 16.70 | 7296.49 / 7322.62 (96%) |
| a15_okvis | 1.56 | 1.65 / 1.37 / 1.36 | - / - (0%) |
| a20_okvis | 12.00 | 11.93 / 12.13 / 12.13 | 58.72 / 11.91 (20%) |
| o1_orb3mono | 5.73 | 1.61 / 2.95 / 2.98 | 2.88 / 3.04 (100%) |
| o2_orb3mono | 14.66 | 2.64 / 2.58 / 2.52 | 1.37 / 4.30 (100%) |
| a15_orb3mono | 1.56 | 1.25 / 0.98 / 0.96 | - / - (0%) |
| a20_orb3mono | 12.00 | 12.12 / 12.51 / 12.46 | 12.58 / 14.77 (100%) |
| o1_stella | 5.73 | 5.02 / 6.94 / 6.94 | 16.34 / 5.51 (98%) |
| o2_stella | 14.66 | 14.56 / 17.66 / 17.67 | 50.56 / 41.38 (99%) |
| m14_okvis | 1.37 | 1.46 / 1.72 / 1.72 | 1.60 / 2.13 (72%) |
| o1d_okvis | 2.20 | 1.47 / 1.43 / 1.43 | 1.61 / 2.16 (39%) |
| o1_xrslam | 5.73 | 5.17 / 6.10 / 6.11 | 6.15 / 4.03 (100%) |
| o2_xrslam | 14.66 | 14.55 / 16.99 / 17.01 | 202593.75 / 142417.79 (100%) |
| a20_xrslam | 12.00 | 11.99 / 12.14 / 12.15 | 122356.12 / 80496.62 (100%) |

(coverage = share of the odometry sample times from +30 s that got a georef pose; a15 cases: 38 indoor fixes, no fit.) With `scale_sigma=100` for the metric-but-collapsing XRSLAM streams (Outdoor-2 / ADVIO-20) the errors are still 97 / 86 and 274 / 142 m. **The georef is not a general replacement**: one global similarity cannot follow a drifting or re-initialising odometry (OKVIS2 Outdoor-1 47 / 42 m against the smoother's 6.1 / 7.4, complex_rtk 2.7 / 2.2 m against 0.08 / 0.09 with cm-accurate RTK fixes, o1_stella 16.3 / 5.5 against 5.0 / 6.9, XRSLAM collapse); it is the right tool only where the stream's shape is better than the fixes' low-frequency error, which is what the gait-scaled stella_vio stream on the phone sequences is. Where the stream is good it also helps the earlier list (o1_xrslam causal 4.03 vs 6.10 / live 6.11 for the smoother, o2_orb3mono batch 1.37 vs 2.64, m14_okvis free scale 1.46 / 1.40 vs 1.46 / 1.72) but those are single cases. The smoother default for all these cases is unchanged (switches off = byte-identical outputs, 14.7). A switch between smoother and georef by the ratio of the georef residual rms to the reported fix sigma is conceivable (phones 0.45 / 1.3 / 0.5-0.9, failures 2.3-9363, o1_xrslam 0.59 where the georef is fine, o2_orb3mono 2.3 where it is fine too) but is not clean enough to adopt.

### 14.7 Baseline checks (new code is opt-in: `gf_georef*` are new files, `gf_fusion.c` / `gf_gait.c` / `gf_run.c` untouched; `phone_pipeline/check_baselines.sh` minus its stella_vio exact-port part, which was skipped because stella_vio is being changed by another agent)

`gf_table.py` 893 numbers and `gf_gait_study.py fusion` 1578 numbers identical to `gnss_fusion/work/pre13/*.json` (0 differ), `compare_py.py` (8 cases) numbers identical to the saved `pre13/compare_py.json` (only wall-clock fields differ), `test_geo.py` PASS, new `test_georef.py` PASS (5 checks). `scores.json` of the three outdoor sequences: all 331 earlier numbers per sequence unchanged after adding the georef rows (only the timing strings differ).

### 14.8 Open issues
* Causal gain on ADVIO-20 is small (-1.8 %; the 0.84 gait scale of that person is not corrected by the fixes) and one run per row on a chaotic front end; Outdoor-1 / -2 are -18 / -23 %.
* The georef is a global similarity: no drift model (forgetting costs accuracy on these 10-minute walks but would be needed for long VIO runs), no re-initialisation handling (new maps of the stream must already be metric-continuous, as the gait pipeline makes them), stationary starts only enter when the track spans 30 m. First aligned output at 31-37 s, as for the smoother.
* The two stages run the smoother without the fixes, so tracking-loss stretches are bridged by the gait speed (PDR-like) and fixes in holes longer than 2.5 s are not used; the old GNSS-only nodes of the fix-in-smoother path are not in the georef path.
* The fix statistics (40 s correlation, white part 0.3 m) come from two phone models / two datasets; other receivers or a better phone GNSS (dual frequency) will change `corr_s` and may favour the smoother again (complex_rtk: 0.08 m with the smoother). No automatic switch between the two.
* Python GLS prototype only for the coloured-noise variant; no Gauss-Markov bias state in the C smoother.

## 15. True live pipeline and the automatic smoother <-> georef switch (2026-10-04)

Questions: (1) what does the phone pipeline lose when every stage consumes the poses AS COMPUTED frame by frame (stella_vio's tracking pose at that moment, before later local BA / loop corrections) instead of the final trajectory? (2) can the choice between the smoother (section 13) and the geo-referencing (section 14) be made online from observable signals? (3) CPU per frame of every stage. Own code, nothing GPL read. Images of all six sequences were re-fetched with the section-9.2 scripts into the scratch dir (peak about 6 GB, deleted afterwards).

### 15.1 What was built (all opt-in, defaults unchanged)

* `stella_vio/sv_run --live-out F`: per tracked frame, at the moment it is computed: `t x y z q map_id rframe seg loop_accepted scale_cal up_n ux uy uz` (`sv_frame_result.live_*`; `sv_run.c` has `sv_run_frame_hook` and `sv_run_main()`). Off by default; with it off every output file is unchanged.
* `phone_pipeline/c/pp_live.c` (C99, stdio in the driver only): ONE process = sv_run driver + per-frame hook: IMU samples and fixes with time <= the frame time are fed to `gf_gait` / `gf_auto`; the live pose is gravity-aligned with the CURRENT up estimate of its map (frozen after 150 contributing frames; no sample before 5), flagged like `run.py make_odom` (new map id = NEW_FRAME, new segment = GAP|LOOSE, R-frame = LOOSE) and streamed into `gf_auto` (smoother with fixes A, fix-free gait smoother B -> `gf_georef`, switch). Output per frame: `pp.auto` (what a live consumer sees) plus `pp.sm`, `pp.geo`, `pp.odom` (the live odometry; replaying it through `gf_auto_run` reproduces the stream to print precision), `pp.speed`, `pp.sig` (signals), `pp.timing`. `phone_pipeline/run.py live <seq> --variants full`, `live_eval.py`, `live_report.py`.
* `gnss_fusion/c/gf_auto.{h,c}` + driver `gf_auto_run`: the streaming combination and the switch (below). `policy=0` is bit-for-bit (print precision) `gf_run` causal live, `policy=1` the two-stage georef (`tools/test_auto.py`, 6 checks incl. causality: the first half of the output is byte-identical when the second half of odometry and fixes is cut).
* Determinism: the final trajectory (`trajectory_maps.tum`, `trajectory_gz.tum`) written by the same pp_live run is `cmp`-identical to the section-13 `sv_<full>` files on all six sequences, so the live and final numbers below come from the same front-end run.

### 15.2 Live vs final (ATE SE3 [m], `full` stella_vio variant; one deterministic run per row; causal scored from +30 s, GNSS-free sequences from +12 s)

| sequence | GNSS alone | final traj: batch / causal | live odometry, whole-graph batch / causal replay | live, streamed: smoother / georef / AUTO (causal) | live vs final batch | live vs final causal |
|---|---|---|---|---|---|---|
| Indoor-1 | - | 1.16 / 1.02 | 0.79 / 1.21 | 1.21 / - / **1.21** | -32% | +19% |
| Indoor-2 | - | 0.31 / 0.31 | 1.06 / 1.10 | 1.10 / - / **1.10** | +243% | +257% |
| Outdoor-1 | 5.73 | 5.50 / 7.10 | 5.12 / 6.87 | 6.87 / 4.28 / **4.31** | -7% | -3% |
| Outdoor-2 | 14.66 | 13.28 / 15.10 | 13.52 / 15.65 | 15.65 / 11.33 / **11.33** | +2% | +4% |
| ADVIO-15 | - | 0.90 / 0.66 | 0.80 / 0.85 | 0.85 / - / **0.85** | -12% | +30% |
| ADVIO-20 | 12.00 | 11.76 / 12.32 | 11.87 / 12.30 | 12.30 / 11.85 / **11.82** | +1% | -0% |

("final" with fixes = smoother with gait + GNSS of section 13; georef rows final / live: Outdoor-1 5.36 / 4.71 vs 5.26 / 4.28, Outdoor-2 4.81 / 11.34 vs 4.72 / 11.33, ADVIO-20 11.76 / 11.79 vs 11.76 / 11.85 batch / causal. The final-vs-live columns compare the SAME fusion on final vs live poses; the "streamed" smoother equals the causal replay.)
Reading: the live cost is small on the outdoor sequences (within +-7 %, i.e. inside the run-to-run noise of a chaotic front end; the fix-dominated error hides it) and large on the short monocular indoor sequences, where the corridor loop closure / BA later rescales the map: Indoor-2 live-vs-final Sim3 scale of the poses 0.82 (extent 3.2 map units), Indoor-1 0.80; the gait speed prior corrects the scale online, but not the shape. Indoor-2 is the honest worst case: 0.31 -> 1.10 m (3.5x), still ahead of XRSLAM 0.98 / RD-VIO 1.06 only marginally. Indoor-1 batch gets better (0.79) and causal worse (1.21): the 1.16 final batch is itself the chaotic gyro-prior result of section 13 (default config 0.83). Outdoor differences of a few percent are not rankings.
Choices measured on the three small sequences (not on the test outcome of the outdoor ones): flagging loop-closure / scale-calibration frames as `GF_ODOM_GAP` (position jump of the map) costs 0.1-0.7 m (Indoor-1 1.49 / 1.61 vs 0.78 / 1.20, Indoor-2 1.16 / 1.26 vs 1.06 / 1.10, ADVIO-15 1.00 / 0.91 vs 0.80 / 0.85 batch / causal), so it is off (`--pp-jump 0`); gravity: running estimate worse than frozen after 150 frames (Indoor-1 0.88 / 1.28 vs 0.78 / 1.20), 30-400 frames identical, oracle (final) up vector 0.82 / 1.09 (the remaining gap is the early up error).

### 15.3 The switch (`gf_auto`, causal, observable signals only)

Both estimates run all the time; the output is the smoother A, the georef G or a cross-fade (weight up 1/20 s, down 1/2 s). G is used only while ALL hold: (a) G has >= 30 pairs and the stream is metric (`geo.scale_sigma <= 10`: gait stream, metric VIO; a free-scale monocular stream needs the smoother's per-window scale) and has not restarted in a new frame (raw streams); (b) the fixes are poor: reported sigma (EW mean) >= 4 m (a global similarity can only help where the smoother's 30 s window cannot average the fix error out; accurate fixes can bend the stream); (c) the stream agrees with the fixes: rho = rms prequential error of G at the fixes (EW 300 s, error of each fix against the fit that has not seen it) / max(reported sigma, 0.1 m) <= 0.75 to enter, < 1.125 to stay; (d) the smoother's own consistency test does not distrust the odometry (its distrusted share, EW 60 s, <= 0.2 to enter, < 0.3 to stay: slow drift and collapse of the stream); (e) no sudden failure: the latest prequential error <= 3 x max(rms, 2 sigma), otherwise weight 0 at once and G barred for 60 s. Signals logged per sample (`--sig`): pairs, fit residual, georef scale, EW prequential errors, reported/white fix noise, disagreement A-G, distrust share, new-frame/gap counts.
Rejected on the way (details `docs/rejected_trials.md`): a residual-trend ratio e_fast/e_slow <= 1.3 (kills the phone cases where the common fix error wanders: ratio 1.5-1.9 on p_o1/p_o2), a 0.5 m noise floor (RTK: 0.10 -> 0.44), scale-free mono georef (o2_orb3mono 4.14 vs 2.52, a20_orb3mono 14.60 vs 12.46), using the georef before the smoother is aligned (a collapsing XRSLAM stream gave 130-150 m).

### 15.4 Result over all cases (live/causal ATE SE3 [m]; earlier list: `gnss_fusion/tools/gf_auto_table.py`, stream = raw odometry; phone: pipeline inputs replayed through `gf_auto_run`, stream = gait smoother B)

| case | GNSS alone | smoother live | georef | AUTO | AUTO vs best of two | AUTO vs GNSS alone | smoother vs GNSS (current) |
|---|---|---|---|---|---|---|---|
| complex_rtk | 0.00 | 0.10 | 2.16 | 0.10 | +0% | n/a | n/a |
| complex_sim | 4.18 | 2.15 | 2.56 | 2.15 | +0% | -49% | -49% |
| complex_rtk_blk | 0.00 | 0.67 | 1.78 | 0.67 | +0% | n/a | n/a |
| complex_sim_blk | 4.33 | 2.16 | 2.01 | 2.16 | +8% | -50% | -50% |
| o1_okvis | 5.73 | 7.40 | 42.16 | 8.76 | **+18%** | **+53%** | +29% |
| o2_okvis | 14.66 | 16.70 | 7322.62 | 16.70 | +0% | +14% | +14% |
| a15_okvis | 1.56 | 1.36 | - | 1.36 | +0% | -13% | -13% |
| a20_okvis | 12.00 | 12.13 | 11.91 (20%) | 12.13 | +2% | +1% | +1% |
| o1_orb3mono | 5.73 | 2.98 | 3.00 | 2.98 | +0% | -48% | -48% |
| o2_orb3mono | 14.66 | 2.52 | 4.14 | 2.52 | +0% | -83% | -83% |
| a15_orb3mono | 1.56 | 0.96 | - | 0.96 | +0% | -39% | -39% |
| a20_orb3mono | 12.00 | 12.46 | 14.60 | 12.46 | +0% | +4% | +4% |
| o1_stella | 5.73 | 6.94 | 5.51 | 6.94 | **+26%** | +21% | +21% |
| o2_stella | 14.66 | 17.67 | 41.38 | 17.67 | +0% | +21% | +21% |
| m14_okvis | 1.37 | 1.72 | 2.13 | 1.72 | +0% | +25% | +25% |
| o1d_okvis | 2.20 | 1.43 | 2.16 | 1.43 | +0% | -35% | -35% |
| o1_xrslam | 5.73 | 6.11 | 4.03 | 4.03 | +0% | -30% | +7% |
| o2_xrslam | 14.66 | 17.01 | 142418 | 17.01 | +0% | +16% | +16% |
| a20_xrslam | 12.00 | 12.15 | 80497 | 12.15 | +0% | +1% | +1% |
| phone Outdoor-1, final odometry (gait stream) | 5.73 | 7.10 | 4.71 | 4.78 | +1% | -17% | +24% |
| phone Outdoor-2, final | 14.66 | 15.10 | 11.34 | 11.25 | -1% | -23% | +3% |
| phone ADVIO-20, final | 12.00 | 12.32 | 11.79 | 11.76 | +0% | -2% | +3% |
| phone Outdoor-1, LIVE odometry | 5.73 | 6.87 | 4.28 | 4.31 | +1% | -25% | +20% |
| phone Outdoor-2, LIVE | 14.66 | 15.65 | 11.33 | 11.33 | +0% | -23% | +7% |
| phone ADVIO-20, LIVE | 12.00 | 12.30 | 11.85 | 11.82 | +0% | -2% | +3% |

Target "never worse than ~10 % over the better of the two": met on 23 of 25 rows. **Not met: o1_okvis (+18 %, and +53 % vs GNSS alone against the smoother's own +29 %: the OKVIS stream is plausible for 190 s, the georef is used and then the stream drifts; the distrust share only crosses 0.2 at ~190 s) and o1_stella (+26 %: a free-scale monocular stream is excluded by rule (a); the georef would have given 5.51 vs 6.94)**; complex_sim_blk +8 % is inside the target. "Never worse than GNSS alone by more than the current worst (+29 %)" is violated only by o1_okvis (+53 %).
Leave-one-case-out (25 items = the 22 above + the 3 live phone rows; grid rho_on {0.75, 1.0} x distr_on {0.1, 0.15, 0.2} x sigma_min {4, 6} x fail_k {3, 4}; python simulation `tools/gf_auto_rule.py` of the saved streams, the C rows above are authoritative): the setting picked on the other 24 is the shipped one for 24 of 25 held-out cases and the held-out regret equals the full-data regret except where the held-out case itself is the o1_okvis / o1_stella failure (1.18 / 1.26). An earlier 729-point grid (with min span, distr_off) picked the same corner. **Settings were chosen on these 22 cases, not on independent data; the reported-sigma threshold (4 m) separates "consumer GNSS" from RTK / PX4 / simulated fixes by construction of the case list.** The simulation and the C switch disagree on one item (Outdoor-2 live: simulation 14.7 m, C 11.33 m); the C numbers are the measured ones.

### 15.5 CPU per frame (pp_live, thread CPU time, machine shared with other jobs: stella_vio rows are inflated by cache contention, the earlier idle-ish sv_run numbers were 36-68 ms/frame)

| sequence | frames | stella_vio ms/frame (mean / p99) | gait us/frame (about 6.7 IMU samples) | fusion all us/frame (mean / p99 / max) | of which smoother A | stream smoother B | georef + switch | fix handling (per frame equivalent) |
|---|---|---|---|---|---|---|---|---|
| Indoor-1 | 1779 | 39.8 / 100.1 | 2.9 | 30.8 / 165.7 / 418.9 | 5.7 | 6.7 | 2.1 | 0.0 |
| Indoor-2 | 1535 | 35.5 / 73.2 | 2.9 | 29.5 / 199.5 / 363.5 | 6.0 | 6.9 | 2.0 | 0.0 |
| Outdoor-1 | 5898 | 52.4 / 246.2 | 2.9 | 77.6 / 667.6 / 2788.2 | 21.0 | 8.1 | 4.5 | 19.0 |
| Outdoor-2 | 6737 | 52.6 / 204.4 | 2.6 | 73.0 / 653.4 / 2764.7 | 19.2 | 7.2 | 4.4 | 18.2 |
| ADVIO-15 | 1553 | 38.2 / 68.7 | 2.0 | 22.5 / 119.6 / 238.8 | 3.2 | 3.6 | 1.9 | 0.0 |
| ADVIO-20 | 9076 | 52.7 / 211.9 | 2.0 | 44.9 / 692.7 / 4172.6 | 18.9 | 6.6 | 3.1 | 0.2 |

The fusion (both smoothers, gait, georef, switch) costs 0.1-0.2 % of the front end; stella_vio is the whole budget (median 30-40 ms/frame, p99 up to 250 ms on keyframes / BA, max up to 0.47 s). Worst fusion call 4 ms (a window solve at a fix). `runs/phone_pipeline/live_tables.md`, `<seq>/live_full/{scores.json,pp.timing}`.

### 15.6 Baseline checks
`phone_pipeline/check_baselines.sh` (updated: compares with `gnss_fusion/work/pre15/`): stella_vio fr1_xyz with `reinit_sec=0 init_max_level=0 init_confirm=1` vs stella_port `sv_run`: 787 poses, `trajectory.tum` CMP-IDENTICAL (after the sv_run / sv_system changes); `gf_table.py` 893 numbers 0 differ; `gf_gait_study.py fusion` 1578 numbers 0 differ; `gf_georef_table.py` 140 numbers 0 differ; `compare_py.py` 8 cases and `test_geo.py`, `test_georef.py` pass; `test_auto.py` 6 checks pass. pp_live's final trajectory files are cmp-identical to the saved `sv_full/` on all six sequences, i.e. the default stella_vio trajectories are unchanged.

### 15.7 Open issues
* Live Indoor-2 (0.31 -> 1.10) and the short monocular sequences generally: the live map scale drifts until a loop closes; a better online scale (the gait prior is only a soft factor) or delayed-smoothing output would help; not tried.
* The switch fails on o1_okvis (+18 %) and the free-scale o1_stella (+26 %); the thresholds were selected on the case list (22 cases, 2 datasets plus drone/GVINS), the sigma_min = 4 m rule is a consumer-vs-good-GNSS proxy. Outdoor-1 final has distrust max 0.26 against the 0.2 entry threshold (it passes because the share only crosses it after the georef is already on): fragile.
* Free-scale monocular streams never use the georef in AUTO; the python simulation and the C switch differ on one live item.
* Tracking-loss stretches: pp_live emits GNSS-only nodes through `pp.auto` only when fixes arrive during a tracking loss; no output is produced for the frames in between.
* Timing was measured on a machine shared with other jobs (thread CPU time, cache contention).

## 16. Closing the live-vs-final gap indoors: gait scale servo inside the stella_vio mapping (2026-10-04)

Question (open issue 15.7): the live phone pipeline loses 3.5x on Indoor-2 (0.31 -> 1.10 m), 19 % on Indoor-1 and 30 % on ADVIO-15 against the final trajectory; the live map scale drifts until a loop closes. Can the gait speed be fed back into the mapping so that the live map stays metric-consistent, without hurting the outdoor sequences and the exact-port baselines? Own code and own understanding (no GPL code read). Four ideas were tried; one is accepted, scoped to the GNSS-free (indoor) pipeline.

### 16.1 Diagnosis (Indoor-2, section 15 live run; GT used for diagnosis only)

* The gait speed is not the problem: 3 s epochs are within about 10 % of the GT speed (27 epochs, mean ratio 1.13 incl. one start-up outlier).
* The live MAP is: walked metres per map unit (GT length / live-odometry length over 6 s windows) falls from 8.8 (12-18 s) to 3.7 (90-96 s), i.e. the map unit grows 2.4x while the person walks at a constant 1.1 m/s; the final trajectory does not have this (its scale is the one of the start, 9.11, a loop at 92 s and BA remove it). The fusion applies a scale that lags the drift (applied 7.0 / 6.2 / 5.3 vs true 5.3 / 4.8 / 3.7 over 78-96 s) and its error is 1.3-2.9 m in the last 20 s and about 2.2 m at 10-20 s (start-up scale from the first speed epochs, +20 %); between 30 and 80 s the live fusion is as good as the final one (0.2-0.4 m).
* Fusion-side remedies cannot fix that (16.5): the scale random walk, the speed sigma and the window were swept on the saved live odometry, flat within +-0.1 m.

### 16.2 What was built (all opt-in; with every switch off the trajectories are byte-identical)

`stella_vio` (`sv_system.c servo_step`, switches `--set servo=G servo_win=S servo_dmin=M servo_clip=C` + the explored variants `servo_mode servo_gate servo_href servo_dead servo_k`; host API `sv_system_push_speed(sys, t, v)`, trace `--servo-log F`):
the host (pp_live) pushes every gait epoch (3 s, mean speed of the trailing 6 s) to the mapping. After every new keyframe, once the mapper has finished, the mapping compares the metres walked over the last `servo_win` seconds (sum of v x 3 s over the last epochs) with the length of the map's keyframe chain over the same time interval. The ratio (metres per map unit) is held at the value of the first window with >= `servo_dmin` metres of the map (it is only a RELATIVE scale: the absolute gait calibration does not matter). The newest keyframe and the landmarks its mapping step created are scaled about the previous keyframe by `exp(clip(G ln(ratio / reference), +-C))` with the same routine that the existing bridged-part scale calibration uses (`scale_section`: keyframes, landmarks, the tracker's motion state and the frames referenced to the part). It is a proportional controller on a 6 s average, not a hard constraint: a gait outlier moves the map by at most C per keyframe and the visual BA of the next keyframes keeps the map self-consistent. The reference is dropped at a loop correction and at a map reset. Defaults of the switches: off (`servo_gain` 0), `servo_win` 12, `servo_dmin` 5, `servo_clip` 0.05; **the tested setting (`servo` variant of `run.py`) is `servo=0.5 servo_clip=0.2 servo_win=6 servo_dmin=2`**.
`phone_pipeline`: `pp_live` pushes its speed epochs to the mapping (`--pp-servo-noise S` = log-normal noise on what the servo sees, robustness test), `run.py live --variants servo` (= `full` + the setting above; output `live_servo/`), `run.py live --cfg "tag|sv sets|pp opts" --skips 0,30,60` (extra configurations in the same lock-step pass, output `<seq>/study16/<tag>_s<skip>/`, the canonical `live_full/` is never touched), `live_study.py` (paired perturbation study on pre-made fixtures), `study_report.py`, `live_report.py [variant]`, `auto_eval.py` (now also GNSS-free sequences, reproduces the section-15 streamed numbers exactly).

### 16.3 How the result was judged (noise)

A single run of the mono front end is chaotic, so every configuration is run from several start frames (`sv_run --skip N`, the same perturbation of the initialisation that `stella_vio/tools/init_study.py` uses for its window counts; `init_study.py` scores camera-only windows, whereas here the fused live output is needed, hence `live_study.py`): Indoor-1 / Indoor-2 / ADVIO-15 skips 0,10,20,30,40,50 (0-3 s), ADVIO-20 0,20,40,60, Outdoor-1 ten starts (0..90 and 300, 600, 900 frames), Outdoor-2 0,30,60. Comparisons are paired by start (better / worse by more than 0.02 m) and give the spread of the servo-off runs as the noise floor: servo off spans Indoor-1 1.07-1.33, Indoor-2 1.00-1.11, ADVIO-15 0.79-1.03, ADVIO-20 11.77-11.85, Outdoor-1 3.67-8.11 (!), Outdoor-2 11.33-11.40 [m]. The pipeline is deterministic, so each number is reproducible. **All settings were tuned on Indoor-1 / Indoor-2 / ADVIO-15 (test data of this study, no held-out set exists); the outdoor sequences were only used to check for harm (and they showed some).** Scored as in section 15: causal AUTO output (`gf_auto`) against GT, from +12 s (GNSS-free) / +30 s (fixes) after the first frame of the run.

### 16.4 Results

Tuning on the indoor sequences (causal ATE SE3 [m], mean over starts; servo off in brackets; `d` = `servo_dmin`):

| setting | Indoor-2 | Indoor-1 | ADVIO-15 |
|---|---|---|---|
| gain 0.2, clip 0.05, win 12, d5 (first guess) | 1.02 (1.07) | - | 0 steps (speed 0.43 m/s never reaches 5 m) |
| gain 1, clip 0.2, win 6, d5 | 0.61 (1.07) | 0.76 (1.21) | - |
| gain 1, clip 0.3, win 6, d5 | 0.69 | - | - |
| gain 2, clip 0.3, win 6, d5 | 0.62 | - | - |
| gain 1, clip 0.2, win 9, d5 | 0.72 | - | - |
| gain 0.5, clip 0.2, win 6, d5 | 0.59 | 0.83 | - |
| gain 0.5, clip 0.2, win 6, **d2** (**chosen**, `servo` variant) | **0.61 (1.06)**, 6 of 6 starts better | **0.79 (1.20)**, 6 of 6 | **0.84 (0.89)**, 2 better, 4 equal |
| gain 1, clip 0.2, win 6, d2 | - | 0.75 | 0.88 (0.83), 3 of 4 worse |
| gain 0.5, clip 0.1, win 6, d2 | - | 0.78 | - |

All servo settings with a sensible gain help every indoor start (24 of 24 paired comparisons); differences between gains / clips / windows are inside the start-to-start noise (+-0.1 m), so the choice between them is not significant. Gain 1 hurts ADVIO-15 slightly (3 of 4 starts), gain 0.5 does not.

**Live vs final per sequence** (causal AUTO ATE SE3 [m]; final = AUTO on the final trajectory's odometry, section 15.4; live s15 = the section-15 run; "off" / "servo" = this section's perturbation study, mean [min..max] over n starts; "pipeline" = the one official run `run.py live --variants servo` -> `live_servo/`, GNSS-free sequences only):

| sequence | final | live s15 | live, servo off (n) | live, servo (n) | **live/final off -> servo** | official run with servo | raw live map Sim3 ATE off / servo | servo policy |
|---|---|---|---|---|---|---|---|---|
| Indoor-1 | 1.02 | 1.21 | 1.20 [1.07..1.33] (6) | 0.79 [0.64..1.09] (6) | 1.18x -> **0.78x** | 0.64 (batch on live odometry 0.60 vs 0.79) | 2.99 / 0.81 | on |
| Indoor-2 | 0.31 | 1.10 | 1.06 [1.00..1.11] (6) | 0.61 [0.47..0.79] (6) | 3.42x -> **1.96x** | 0.67 (batch 0.66 vs 1.06) | 1.20 / 0.58 | on |
| ADVIO-15 | 0.66 | 0.85 | 0.89 [0.79..1.03] (6) | 0.84 [0.79..0.94] (6) | 1.36x -> 1.29x | 0.85 | 0.74 / 0.72 | on (neutral) |
| Outdoor-1 | 4.78 | 4.31 | 4.63 [3.67..8.11] (10) | 6.28 [4.21..11.89] (10) | 0.97x -> 1.32x | not run (off) | 14.8 / 20.4 | **off** |
| Outdoor-2 | 11.25 | 11.33 | 11.37 [11.33..11.40] (3) | 11.38 [11.21..11.54] (3) | 1.01x -> 1.01x | not run (off) | 6.6 / 8.3 | off (neutral) |
| ADVIO-20 | 11.76 | 11.82 | 11.81 [11.77..11.85] (4) | 11.63 [11.60..11.66] (4) | 1.00x -> 0.99x | not run (off) | 9.2 / 3.8 | off (4 of 4 better, 1.5 %) |

(`runs/phone_pipeline/study16_tables.md`, `<seq>/study16/results.json`, `<seq>/live_servo/scores.json`.) Reading: **the indoor gap is closed by 40-55 %** of the excess: Indoor-1 live now beats the final trajectory (0.64-0.79 vs 1.02; the final 1.02 is itself the chaotic gyro-prior result of section 13), Indoor-2 1.10 -> 0.61-0.67 (final 0.31: the remaining excess is the 2 m start-up error at 10-20 s of the fusion's first speed alignment and the last 20 s before the loop; excluding the first 40 s the servo run is 0.73 vs final 0.32 vs servo-off 1.23), ADVIO-15 is unchanged (its map is already consistent: raw map Sim3 0.72; the 30 % gap there is the rotation-only / bridged stretch and the start-up). The servo halves-to-quarters the live map's own error (raw live map Sim3 ATE: Indoor-1 2.99 -> 0.81, Indoor-2 1.20 -> 0.58, ADVIO-20 9.15 -> 3.82). Cost: +2 % CPU (ADVIO-20 484 -> 493 s, Outdoor-1 420 -> 429 s per run; one chain walk per keyframe).
Robustness to a bad gait signal (Indoor-2, 6 starts, log-normal noise on every speed epoch that the SERVO sees, the fusion keeps the clean speed): sigma 0.2 / 0.4 / 0.8 -> 0.77 / 0.79 / 0.73 m against 0.61 clean and 1.06 off: the benefit degrades gracefully but does not vanish even with 80 % speed noise (the window average and the clip bound the damage).

**Outdoor-1 is hurt (mean 4.63 -> 6.28, +36 % over 10 starts; worse in 9 of 10, +48 % over the seven starts of 0-90 frames (starts 75 and 90 give the same run), +5 % over the starts of 300 / 600 / 900 frames), Outdoor-2 neutral, ADVIO-20 marginally better.** The servo trace (`servo.log`) shows why: the gait detector reports 'walking' at 0.8-1.3 m/s during the first 12-15 s of Outdoor-1 while the GT 6 s chord speed is 0.03-0.45 m/s (the user is still, then starts), and the reference ratio of the first window comes from exactly those epochs: it is 8 % off the steady-walking ratio (0.92), so the controller fights every keyframe by 3-4 % (ln f -0.04) against a map that re-inflates, and around t = 830 s of the run a scale blow-up (metres per map unit falls 5-16x within seconds with the servo, 2.5x without) is not repaired by the per-keyframe correction. Variants that make the reference robust, all rejected (Indoor-2 / Indoor-1 6-start means for servo off 1.06 / 1.20, chosen setting 0.61 / 0.79; Outdoor-1: mean over the listed starts, servo off in the right-hand column):

| variant | Indoor-2 | Indoor-1 | Outdoor-1 |
|---|---|---|---|
| mode 1: reference = ratio over the whole history before the window (>= 10 m), gate 0.5, clip 0.2 (A) | 0.88 | 0.89 | 4.98 vs 4.39 (starts 0, 30, 60), +13 % |
| same, clip 0.05 (B) | 0.90 | 0.97 | 4.67 vs 4.39, +6 % |
| mode 0, gate 0.5 (no correction when ratio / reference is off by > 1.65x), clip 0.05 (C) | 0.79 | 0.91 | 5.11 vs 4.39, +16 % |
| mode 1, history >= 6 / 15 m, no gate (D / F) | 0.94 / 0.84 | 0.94 / 0.92 | - |
| mode 1, gate 1.0 (E) | 0.88 | 0.89 | - |
| dead band 0.1 / 0.2 on ln ratio (G / H) | 0.77 / 0.89 | 0.83 / 1.02 | 6.48 / 7.01 vs 5.40 (starts 0, 45, 75), +20 / +30 % |
| mode 1 + dead band 0.1 (I) | 1.14 | 0.97 | - |
| mode 2: reference = median of the first k non-overlapping windows, k = 2 / 3 / 5 | 0.64 / 0.83 / 0.90 | 0.79 / 0.94 / 1.00 | k = 3: 5.63 vs 5.00 (starts 0, 45, 75, 90), +13 % (maps better, fused worse) |

Every variant that protects the outdoor start loses part (mode 0 -> 1, gates, dead bands) or all (k >= 3) of the indoor benefit and still does not make Outdoor-1 neutral; the indoor sequences need the servo from the first seconds, because their drift is large and early. Policy: **servo on only where the pipeline has no fixes** (GNSS-free = indoor: `run.py live --variants servo`), off with fixes (the georef / smoother already fix the scale from the fixes and the section-15 AUTO output is not changed). The outdoor sequences with servo off are the section-15 numbers (bit-identical front end).

### 16.5 The other ideas

* **Faster loop closure** (idea 2), rejected: `loop_cont` (consecutive keyframes that must agree, stella 3) = 1 or 2 and `loop_matches` (Sim3 validation matches, stella 20) = 12 or 15 accept the same loop at the same frame on Indoor-1 (frame 1737 of 1779; the revisit begins at 1605 by GT, Indoor-2 1335 -> accepted 1382): the detection delay is the BoW candidate retrieval (the camera sees the old place from another viewpoint), not the thresholds. ATE with `loop_cont` 1 / 2 on Indoor-1: 1.30 vs 1.22 (3 starts), `loop_matches` identical. No false loop was produced (nor any new loop). Switches kept (`--set loop_cont=N loop_matches=N`, default 0 = stella's values).
* **Fusion-side scale handling** (idea 3a), rejected: on the saved live odometry `speed_scale_rw` 0.01 / 0.02 / 0.05 (default) / 0.1 / 0.2 gives Indoor-1 / Indoor-2 / ADVIO-15 1.76 / 1.55 / 0.87, 1.28 / 1.34 / 0.86, **1.21 / 1.10 / 0.85**, 1.27 / 1.04 / 0.86, 1.41 / 1.03 / 0.91; `speed_sigma_scale` 0.5 / 0.25: 1.23 / 1.03 / 0.83 and 1.25 / 1.01 / 0.80; `window_s` 20 / 15: 1.20 / 1.11 / 0.85 and 1.20 / 1.11 / 0.90; on the servo runs the same sweep is flat again (rw 0.1: Indoor-1 0.65 vs 0.64, Indoor-2 0.59 vs 0.67; rw 0.02: worse on Indoor-2, 0.81 vs 0.67). A tighter random walk helps one sequence and hurts the other: the live map's drift is what matters, not the filter.
* **Keyframe-corrected poses for past samples** (idea 3b), not implemented: the 30-80 s part of the live Indoor-2 run is already as good as the final one (0.2-0.4 m); the live loss is the start-up scale of the fusion (+20 % for 10 s, 1-2 m) and the drift before the loop, and a fixed-lag output would give up the live property. The servo attacks the cause instead.
* **Gravity-aligned map and gyro prior in live mode** (idea 4): already on in the live pipeline (`full` = `gravity=1 rframe=1 merge=1 gyro=1`, the `servo` variant adds only the servo); nothing to change.

### 16.6 Baseline checks

With the servo off: `trajectory.tum` of the new binaries (pp_live and sv_run) is `cmp`-identical to the section-13 `sv_full` runs on all six sequences (Indoor-1/2, ADVIO-15/20, Outdoor-1/2); stella_vio fr1_xyz exact-port check CMP-IDENTICAL to stella_port (787 poses); `gf_table.py` 893 numbers, `gf_gait_study.py fusion` 1578 numbers and `gf_georef_table.py` 140 numbers 0 differ; `compare_py.py` 8 cases, `test_geo.py`, `test_georef.py` pass (`phone_pipeline/check_baselines.sh`); `gnss_fusion/tools/test_auto.py` 6 of 6 PASS. ASan + UBSan build of `pp_live` on a servo run (Indoor-2, mode 1 + gate): no report.

### 16.7 Open issues

* Outdoor / long-walk use of the servo is not solved: the gait detector's 'walking' false positives during the start of Outdoor-1 poison the reference, and mono scale blow-ups (x8 within seconds) are not repaired by a per-keyframe correction. A reference that is validated against the map (e.g. agreement of three disjoint windows) or a one-shot section rescale on a detected blow-up are the next steps; none was tried.
* Indoor-2 still has twice the final error: the fusion's start-up scale (first alignment from 2-3 speed epochs at 12 s, +15-20 % for 10 s) and the last 20 s before the loop closes.
* Settings were tuned on the three indoor sequences themselves; there is no held-out indoor set (the outdoor runs are a harm check, not a validation).
* Disk: images re-fetched with the section-9.2 scripts (peak about 5 GB of JPEG + 1.4 GB fixtures, deleted afterwards); results of this section ~100 MB in `runs/phone_pipeline/*/study16/` (logs, servo traces, `results.json`).

## 17. Gait detector v2 (step regularity) and the Outdoor-1 servo question (2026-10-05)

Question: the gait scale servo of section 16 hurt Outdoor-1 (+36 %), and the servo trace pointed at false 'walking' at the start of the run (0.8-1.3 m/s reported while the GT chord speed is 0.03-0.45 m/s). Can a better gait detector fix that, so that the servo becomes usable with GNSS? Own code only (`gnss_fusion/c/gf_gait.{h,c}`, `tools/gait.py`, `check_gait.py`, `gait_diag.py`, `phone_pipeline/c/pp_live.c`); no GPL code read, no learned method. **Result: the detector diagnosis and fix work (false-walk seconds on Outdoor-1 15 -> 0, relative rms error of the walking speed 0.15 -> 0.06), but the servo's Outdoor-1 harm does NOT go away (6.04 vs 4.61 m with the new detector): the false walking explains a 38 % shift of the servo reference, not the harm. The servo stays off with fixes, and indoors it keeps the old detector.**

### 17.1 Diagnosis: what the IMU does at the false-walk stretches

`tools/gait_diag.py` (3 s epochs, 6 s window, GT = 0.5 s smoothed path speed and the 6 s chord speed over the same window; drone IMUs of `external/drone/ins_o1`, `ins_m14` included as a negative control: a drone is never 'walking'). Seconds per sequence, detector as of section 12 (generic constant):

| sequence | epochs with GT | GT walking s (chord > 0.8) | detected WALK s | false-walk s (WALK, v > 0.6, GT chord < 0.5) | over-speed s (WALK, v > 1.35 x GT path) | missed-walk s (GT chord > 0.8, not WALK) | false-stationary s |
|---|---|---|---|---|---|---|---|
| Outdoor-1 | 130 | 360 | 384 | **15** | **15** | 0 | 0 |
| Outdoor-2 | 149 | 435 | 447 | 9 | 6 | 0 | 0 |
| Indoor-1 | 38 | 99 | 111 | 6 | 6 | 0 | 0 |
| Indoor-2 | 30 | 75 | 87 | 0 | 0 | 0 | 0 |
| ADVIO-15 | 16 | 0 (shuffle 0.35-0.5 m/s) | 21 | 6 | 6 | 0 | 0 |
| ADVIO-20 | 99 | 282 | 291 | 0 | 15 | 3 | 0 |
| drone ins_o1 | 32 | 36 | 12 | 3 | 6 | 33 (flight, not walking) | 0 |
| drone ins_m14 | 59 | 93 | 102 | 9 | 18 | 3 | 0 |

There is no false-stationary stretch anywhere (the still detector is fine) and no relevant missed walking (the 33 / 3 s of the drones are flight). What the false-walk stretches are, from the raw IMU and GT:

* **Outdoor-1, 0-15 s (and the same pattern at 12-15 s of Outdoor-2, 15 s of Indoor-1, 45 s of Indoor-2, the starts / stops of ADVIO-20):** the person paces: GT x/y shows +-2 m back and forth along a line (net 0.0-1.9 m in 1 s steps, chord speed 0.02-0.43 m/s, path speed 0.3-0.8 m/s) with an ordinary step cadence of 1.5-1.9 Hz. The gait model sees cadence 1.5-1.9 Hz and reports 0.8-1.4 m/s (1.2-1.8x the path speed, 3-50x the chord speed). It is not phone handling, fidgeting, swaying or a stationary-with-noise case: gyro mean |w| is 0.28 rad/s in the bad epochs vs 0.32 in good walking, no rotation burst, and the cadence is inside the plausible band.
* What distinguishes it is the **regularity of the steps**: step intervals vary (coefficient of variation 0.13-0.24 vs 0.04-0.08 in steady walking) and step amplitudes vary and are small (amplitude CV 0.24-0.58 vs 0.11-0.20; median peak amplitude 1.2 vs 2.4 m/s^2 on the same phone; std of the vertical acceleration 1.0 vs 1.9).

Features tried as an 'is this walking' test, AUC for separating the 18 bad walking epochs (v > 1.35 x GT path, or v > 0.6 with chord < 0.5) from the 429 good ones of the six phone sequences (pooled, test data):

| feature | AUC | comment |
|---|---|---|
| step amplitude CV in the window | 0.84 | scale free, **used** |
| step interval CV in the window | 0.82 | scale free, **used** |
| vertical / horizontal energy ratio | 0.88 | phone / person dependent (good-epoch median 1.1 on ADVIO-20, 1.3-1.6 on the Mobile phone, 0.44-0.6 on the ADVIO-15 shuffle): not used |
| absolute amplitude / vertical std | 0.87 / 0.89 | same dependence, not used |
| gyro mean / std, cadence, steps in window, horizontal std | 0.45 / 0.45 / 0.34 (cad), 0.28, 0.50 | no information (cadence band 1.0-2.8 Hz is already in) |

### 17.2 Detector v2 (regularity gate; `reg`, off by default)

`gf_gait_config`: `reg` (0 = exactly the section-12 detector, 1 = irregular window is OTHER, no speed, 2 = still WALK with the speed but `regular = 0` and sigma x `reg_sigma_k` (3)), `reg_iv_cv` 0.15, `reg_amp_cv` 0.40, `reg_amp_min` 0 (off). A window is irregular if the coefficient of variation (population std / mean over the steps of the 6 s window, >= 3 steps) of the step intervals exceeds `reg_iv_cv` or that of the step amplitudes exceeds `reg_amp_cv` or the median amplitude is below `reg_amp_min`. New: `gf_gait_est.regular`, `gf_gait_config_set(cfg, "key", v)` (`gf_gait_run --cfg key=val`, `pp_live --pp-gait-set key=val`, env `GF_GAIT_DET=reg=2,...` for the study scripts, `GF_WORK=dir` for a scratch work dir so the canonical `gnss_fusion/work/` is untouched), `gait.py` has the same keys, `check_gait.py [--cfg=reg=2 ...]` C == python (max abs difference 5e-10 over cadence, speed, sigma, state, regular, k, heading, odometer, 6 sequences x 3 calibration modes, for reg = 0 / 1 / 2 and with other thresholds). ASan + UBSan clean. With `reg = 0` nothing changes (checks in 17.6). The thresholds (0.15, 0.40) were read off the Outdoor-1 start, i.e. on test data; leave-one-sequence-out selection of the two thresholds on a grid (cost = bad walking epochs kept + 0.5 x good epochs dropped, selected on the other five phone sequences) picks (0.2, 0.5-0.6) with held-out totals bad 12 / dropped 15 epochs (undetected: bad 18 / dropped 0; the fixed (0.15, 0.40): bad 8 / dropped 19), i.e. the gate is a modest gain and only the strict setting removes the Outdoor-1 start entirely.

Gait accuracy (section-12 style, 3 s epochs, 6 s window; walking = GT path speed > 0.5 m/s; before = `reg=0`, after = `reg=1` i.e. only regular windows get a speed; per-user constants refitted by the same held-out rule: 0.3795 / 0.3637 / 0.3775 / 0.3768 -> 0.3816 / 0.3676 / 0.3806 / 0.3791):

| sequence, calibration | WALK epochs / GT walking | median ratio | distance ratio | rms rel. error |
|---|---|---|---|---|
| Outdoor-1 generic | 127 / 128 -> 123 / 128 | 1.04 -> 1.04 | 1.06 -> 1.04 | **0.15 -> 0.06** |
| Outdoor-1 per-user | 127 -> 123 | 1.01 -> 1.02 | 1.03 -> 1.03 | 0.13 -> 0.05 |
| Outdoor-1 online GNSS | 127 -> 123 | 0.95 -> 0.99 | 0.95 -> 0.99 | 0.15 -> 0.05 |
| Outdoor-2 generic | 149 / 149 -> 144 / 149 | 0.98 -> 0.98 | 1.00 -> 0.99 | 0.08 -> 0.08 |
| Indoor-1 generic | 37 / 37 -> 34 / 37 | 1.09 -> 1.09 | 1.10 -> 1.09 | 0.15 -> 0.13 |
| Indoor-2 generic | 29 / 29 -> 21 / 29 | 1.09 -> 1.08 | 1.10 -> 1.09 | 0.13 -> 0.10 |
| ADVIO-15 generic | 3 / 4 -> 1 / 4 | 1.07 -> 1.07 | 1.17 -> 1.07 | 0.39 -> 0.07 |
| ADVIO-20 generic | 97 / 98 -> 95 / 98 | 0.91 -> 0.91 | 0.93 -> 0.92 | 0.20 -> 0.20 |

Seconds table with the detector on (trusted walking = WALK and regular): `reg=2` (0.15 / 0.40): false-walk s Outdoor-1 15 -> 0, Outdoor-2 9 -> 6, Indoor-1 6 -> 0, ADVIO-15 6 -> 0, ADVIO-20 0 -> 0, drone m14 9 -> 0, drone o1 3 -> 3; missed-walk s (the price: real walking that is irregular) Outdoor-2 0 -> 12, Indoor-2 0 -> 15, ADVIO-20 3 -> 6; the drone m14 flight is no longer 'walking' at all (102 s -> 0). With (0.17, 0.5): Outdoor-1 15 -> 3, Indoor-2 missed 3, Outdoor-2 missed 6.

### 17.3 Fusion (gait study, `gf_gait_study.py fusion` with `GF_GAIT_DET=reg=...`, 12 cases x batch / causal, GNSS runs and GNSS-free columns; section-12 values = reg off)

| detector | cells better / worse / neutral (0.05 m) vs reg off, generic constant | mean change | comment |
|---|---|---|---|
| reg=1 (irregular -> OTHER) | 6 / 8 / 28 of 42 | dominated by ADVIO-15 XRSLAM 12.5 -> 602 (no speed at all left, its scale collapses) | rejected for fusion |
| reg=2, sigma x 3 | 7 / 8 / 31 of 46 | +0.18 m (i2_okvis 15.95 -> 18.79, a15_xrslam 12.47 -> 15.48; Outdoor-1 stella causal GNSS-free 9.38 -> 9.38, Outdoor-1 XRSLAM GNSS-free batch 6.25 -> 5.97, Indoor-2 stella 0.38 -> 0.25 batch but per-user 0.27 -> 0.39) | neutral to slightly worse |
| reg=2, `reg_sigma_k` 1 | identical to reg off (the speeds and sigmas of WALK epochs are unchanged; `regular` is only a flag) | 0 | the setting for pipelines that only want the flag |

The GNSS fusion cases (Outdoor-1 / -2, ADVIO-20 with fixes) move by <= 0.1 m everywhere (their error is GNSS-limited); Outdoor-1 stella causal with per-user constant 7.27 -> 7.07 / 7.24, Outdoor-2 ORB-SLAM3 batch 0.81 -> 0.77 / 0.78. So the detector change is accuracy-neutral for the loose fusion, no promotion on that account.

### 17.4 Live pipeline with the servo on (`live_study.py`, paired by start, causal AUTO ATE SE3 [m] mean over starts; 'off' = `b17`, 'servo' = `s17` = the section-16 setting `servo=0.5 servo_clip=0.2 servo_win=6 servo_dmin=2`; b17 and s17 reproduce the section-16 `base` / `sv05` runs bit for bit on all four sequences, so the old numbers reproduce with the detector off)

A detector-aware servo was built for these runs: `pp_live` pushes to the servo only epochs with `regular = 1`; an irregular epoch gets the last regular speed (held up to 20 s, nothing before the first regular epoch) so that the servo's contiguous 6 s windows stay valid ('hold'; a plain drop breaks the windows: Indoor-2 0.85).

| sequence (starts) | servo off | servo, detector off (sec. 16) | detector v2 servo off | detector v2 + servo (hold) |
|---|---|---|---|---|
| Indoor-1 (6) | 1.20 | 0.79 | 1.20 (`reg_sigma_k` 1) | **0.77-0.78** (6 of 6 better than off) |
| Indoor-2 (6) | 1.06 | **0.61** | 1.04 (sigma x 3) / 1.06 (k 1) | 0.92-0.95; gain 2 clip 0.3 0.73, gain 1 0.88, drop instead of hold 0.85 |
| ADVIO-15 (6) | 0.89 | 0.84 | 1.00 (sigma x 3, 6 of 6 worse) / 0.89 (k 1) | 0.89 / 1.00 (all epochs irregular: the servo gets nothing) |
| Outdoor-1 (10; 9 without the duplicate start 90) | 4.61-4.72 | 6.28-6.32 (worse in 9 of 10) | 4.61 / 4.70 | **6.04 / 6.68 with servo gate 0.5** (worse in 9 of 10 / 8 of 9); detector off + gate 0.5: 5.67 |

Not run (images not fetchable within the disk budget of this task: ADVIO-20 2.4 GB zip, Outdoor-2 450 s over a 0.3 MB/s link): ADVIO-20 and Outdoor-2 live; their gait accuracy / fusion rows are in 17.2 / 17.3.

What the Outdoor-1 traces say (`runs/phone_pipeline/outdoor1/study16/{v2h,v2hg,sv05}_s*/servo.log`, start 30):
* The false walking does poison the reference, more than section 16 estimated: ratio of the first window (metres per map unit) 3.58 with the old detector against 2.60 with the new one (the steady walking ratio is 2.5-3.0), i.e. 38 % instead of 8 %. With the clean reference the servo is quiet before the blow-up (ln f +-0.05).
* The harm is independent of that: around 200-260 s of the run the map unit blows up in every configuration, servo on or off (GT metres per map unit 3.0 -> 0.8 without the servo, start 15; 3.4 -> 0.13 with it) and without the servo the map recovers within 60 s (0.8 -> 3.0) whereas with it, even when the gate switches the servo off at the blow-up (`servo_gate` 0.5: ln f = 0 afterwards), the map stays at 0.13-0.25 m per unit until the end (raw live-map Sim3 ATE 21.9 vs 34.8 for the no-servo run of start 0, 41.0 / 8.5 for start 15 etc.: the servo run is better on some starts and much worse on others, mean 18.0 vs 15.7). Resets / lost frames are the same in all configurations (starts 30 and 900 only), so it is not a tracking reset. The few-percent scale corrections applied earlier in the run change the later state of the map in a way that the blow-up then does not recover from; why, I did not establish (candidates: `scale_section` scaling the newest keyframe about its predecessor leaves the low-parallax landmarks of the running track slightly inconsistent; the front end is chaotic, 3.7-8.1 m spread between starts without the servo).
* Indoor-2 is the opposite: its servo gain partly comes from an over-large first reference (ratio 12.9 at the first window against the steady 8-9; section 16.1: true 8.8 falling to 3.7 while the map unit grows), which makes the servo saturate its clip (-0.2) at nearly every keyframe and so counter the drift of the map unit; with the clean reference (8.4, the irregular first epochs are skipped) the same gain 0.5 under-corrects (0.92-0.95), gain 2 gets 0.73. The indoor gain of the servo is therefore a controller-strength effect more than a reference effect.

### 17.5 Verdict and policy

* Target 'Outdoor-1 servo-on no worse than servo-off, keeping the indoor gains' is **not achieved**: with detector v2 Outdoor-1 servo-on is 31 % worse than off (6.04 vs 4.61), Indoor-2 loses most of its gain, Indoor-1 keeps it.
* The servo therefore stays **off with fixes** (and the policy of section 16 stands: on only GNSS-free, with the section-12 detector, `reg` off). Making it usable in GNSS runs needs a fix for the blow-up recovery (open issue below), not a better gait detector.
* Detector v2 ships as opt-in (`reg`, off by default). It is a better walking-speed estimator on its trusted epochs (rms rel. error 0.15 -> 0.06 on Outdoor-1, 0.39 -> 0.07 on ADVIO-15) and removes the drone false-walking, but it costs real walking seconds (irregular but correct turns), so the recommended use is the flag only (`reg=2 reg_sigma_k=1`: the fusion is bit-identical to reg off, consumers that need a trustworthy speed read `regular`). Candidate for the next study, not done: a servo that is gated by the map-unit blow-up (ratio of the 6 s windows jumping by > 3x within one epoch) and restarts its reference afterwards instead of pushing against the broken map.

### 17.6 Baseline checks

`phone_pipeline/check_baselines.sh` after all changes: stella exact-port cmp CMP-IDENTICAL (787 poses), `gf_table.py` 893 numbers, `gf_gait_study.py fusion` 1578 numbers, `gf_georef_table.py` 140 numbers: 0 differ (`reg` off everywhere); `test_auto.py` 6 of 6 PASS; `compare_py.py` 8 cases, `test_geo.py`, `test_georef.py` pass; `pp_live` with the detector off reproduces the section-16 live runs bit for bit (b17 = base, s17 = sv05 on Indoor-1, Indoor-2, ADVIO-15). `check_gait.py` C == python for reg 0 / 1 / 2.

### 17.7 Open issues

* Outdoor-1 map-unit blow-up at 200-260 s of the run (unrelated to the servo's presence, but made unrecoverable by it); the servo is only usable GNSS-free until this is understood.
* Indoor-2: the servo with the clean reference needs more gain; not tuned further (tuning on three test sequences is not evidence).
* ADVIO-20 / Outdoor-2 live runs with detector v2 are missing (no images, see 17.4).
* Thresholds (0.15, 0.40) were chosen by eye on Outdoor-1; the LOSO choice (0.2, 0.5-0.6) is weaker. The AUCs above are on 18 bad epochs.
* Disk: images fetched into `runs/phone_pipeline/_fetch17/` (about 2 GB, deleted afterwards), results of this section in `runs/phone_pipeline/*/study16/` (tags `b17 s17 v2 v2h v2hg v1g v2k v2kh w2h w2g2 v2h_g1 v2h_g2 v2h_g1d5`), `gnss_fusion/work_v2/`, `work_v2b/` (json only), logs `runs/phone_pipeline/*_study17*.log`.
