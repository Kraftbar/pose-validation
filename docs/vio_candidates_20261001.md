# Camera+IMU (VIO) and camera+IMU+GNSS candidates, 2026-10-01

Research and benchmarking only. No porting. Camera-only track is closed (bit-exact pure-C
stella_vslam port, see README "Full-SLAM Comparison" and `docs/slam_candidates_comparison_20260925.md`).
This note picks what to port next for camera+IMU and what exists for camera+IMU+GNSS.

Artifacts: `runs/vio_compare/CANDIDATES.md` (full candidate table with exact licences),
`runs/vio_compare/table.md` / `table.json` / `summary.md` (measured numbers, per run),
`tools/vio_eval.py`, `tools/vio_prep_gt.py`, `tools/vio_harness/` (scripts, driver, stubs).
Builds live in `external/vio/` (gitignored, 1.5 GB after cleanup; EuRoC images are deleted after each sequence).

## Part A: candidates (licence is the main filter)

Licences were read from the repos on 2026-10-01. Accuracy is from the papers, EuRoC ATE RMSE in metres
(averages over the sequences each paper reports; footnotes in `CANDIDATES.md`).

| System | Licence | Status | ROS needed | Mono-inertial | EuRoC avg (paper) |
|---|---|---|---|---|---|
| **OKVIS2** (ETH-MRL/TUM) | **BSD-3** | active (2026-08) | no | yes (one-camera config; paper is stereo) | stereo VI-SLAM ~0.03-0.045, on par with ORB-SLAM3 |
| **Basalt** (TUM) | **BSD-3** | active (release 0.1.7, 2026-03) | no | **no** (stereo only) | 0.051-0.072 (VIO) |
| **Kimera-VIO** (MIT) | **BSD-2** | active (2026-08) | no (needs GTSAM, OpenGV) | no | 0.11-0.14 |
| XRSLAM (OpenXRLab) | **Apache-2.0** (+ per-method extras) | active (2026-02) | no | yes | not published in README |
| ROVIO / MSCKF_VIO / OKVIS-1 | BSD-style | stale / ROS-only | yes (ROVIO, MSCKF) | ROVIO yes | 0.22 / 0.41 / 0.23 |
| ORB-SLAM3 | GPL-3.0 | slow (2024-07) | no (Pangolin link) | yes | mono-I 0.043, stereo-I 0.035 |
| DM-VIO | GPL-3.0 | light (2024-10) | no (needs GTSAM) | yes | mono 0.069 |
| OpenVINS | GPL-3.0 | active (2025-11) | no (lib), ROS for bag driver | yes | stereo 0.117 |
| VINS-Mono / VINS-Fusion | GPL-3.0 | stale | **yes** | yes | 0.11-0.18 / 0.14 |
| SVO Pro, SchurVINS, R-VIO2, EqVIO, HybVIO | GPL-3.0 | various | mostly ROS | yes | own papers |

GNSS-VIO:

| System | Licence | ROS | Notes |
|---|---|---|---|
| **OKVIS2-X** | **BSD-3** | no (optional ROS2) | factor-graph GNSS position factors, dropout tolerant, online GNSS-extrinsic init. Only permissive option. Tested on Hilti-Oxford, VBR (9 km), a GVINS sequence, and EuRoC with simulated RTK |
| GVINS | GPL-3.0 | yes (ROS1) | raw pseudorange/Doppler, stale since 2021 |
| IC-GVINS | GPL-3.0 | no | INS-centric, best-engineered, own Wuhan dataset |
| GICI-LIB (gici-open) | GPL-3.0 | optional | PPP/RTK, camera+IMU+GNSS |
| InGVIO | **no licence file** | yes (ROS Noetic) | invariant filter, raw GNSS; unusable as a source |
| R2-GVIO | AGPL-3.0 | yes | VINS-Fusion based, SYSU campus dataset |
| KF-GINS / OB-GINS | GPL-3.0 | no | GNSS/INS only, no camera |

Dataset note: EuRoC is hosted on the ETH Research Collection (DOI 10.3929/ethz-b-000690084) under
**"In Copyright - Non-Commercial Use Permitted"** (rightsstatements.org InC-NC 1.0), not Creative
Commons. Fine for benchmarking here, not redistributable, so it is never committed.
The old robotics.ethz.ch mirror hung, so sequences were streamed from the Research Collection
(nested zip, HTTP range requests; the server rate-limits with 429, the fetcher retries).

## Part B: what was benchmarked

Sequences: EuRoC MH_01_easy, MH_03_medium, V1_02_medium, V2_02_medium (ASL, cam0 (+cam1 for stereo), imu0).
Ground truth: `state_groundtruth_estimate0` (body = IMU frame). VIO systems report the IMU pose and are scored
against body GT; camera-only mono baselines report the camera pose and are scored against GT transformed to cam0
(`T_WB * T_BC`). Association: nearest GT sample within 5 ms. ATE via `benchmark.ate_rmse` (Sim3) and the same
`benchmark.umeyama_alignment` with `with_scale=False` (SE3, the metric-scale number). Coverage is estimated poses
divided by camera frames. RTF is wall time / sequence duration, one run per cell, all-core machine
(Ryzen 7 3700X 8c/16t), nothing else running except the occasional dataset download.

Systems run (all built/run without ROS):

| id | what | how obtained |
|---|---|---|
| `okvis2_stereo_slam`, `okvis2_mono_slam` | OKVIS2 VI-SLAM with loop closure, final (loop-closed) trajectory | built from source (ceres, brisk, DBoW2, opengv bundled), `okvis_app_synchronous` |
| `orbslam3_stereo_inertial`, `orbslam3_mono_inertial`, `orbslam3_mono` | ORB-SLAM3 | built from source, headless: Pangolin viewer/map-drawer stubbed, example real-time pacing removed |
| `basalt_stereo_vio`, `basalt_stereo_vo` | Basalt VIO and Basalt stereo VO (IMU off) | prebuilt GitLab release 0.1.7, `--show-gui 0` |
| `openvins_stereo`, `openvins_mono` | OpenVINS MSCKF (no loop closure) | built ROS-free (`ENABLE_ROS=OFF`, aruco off) with a small EuRoC driver around `VioManager` |
| `stella_mono` | camera-only baseline, stella_vslam, loop closure on | `run_euroc_slam` from the previous study, stock EuRoC mono config |

### Results (ATE RMSE m, SE3 / Sim3; mean over the 4 sequences)

| system | MH_01 SE3 / Sim3 | MH_03 SE3 / Sim3 | V1_02 SE3 / Sim3 | V2_02 SE3 / Sim3 | mean SE3 | mean Sim3 | coverage | RTF |
|---|---|---|---|---|---|---|---|---|
| okvis2_stereo_slam | 0.030 / 0.019 | 0.029 / 0.023 | 0.019 / 0.013 | 0.015 / 0.015 | **0.023** | 0.017 | 100% | 1.62 |
| orbslam3_stereo_inertial | 0.044 / 0.022 | 0.028 / 0.028 | 0.018 / 0.012 | 0.017 / 0.017 | 0.027 | 0.020 | 94% | 0.60 |
| basalt_stereo_vio | 0.066 / 0.021 | 0.062 / 0.052 | 0.045 / 0.036 | 0.049 / 0.047 | 0.056 | 0.039 | 100% | **0.14** |
| openvins_stereo | 0.071 / 0.070 | 0.125 / 0.118 | 0.080 / 0.074 | 0.042 / 0.040 | 0.079 | 0.075 | 88% | 0.32 |
| okvis2_mono_slam | 0.114 / 0.057 | 0.054 / 0.053 | 0.028 / 0.022 | 0.059 / 0.059 | 0.064 | 0.048 | 100% | 0.75 |
| orbslam3_mono_inertial | 0.083 / 0.040 | 0.053 / 0.046 | 0.032 / 0.029 | 0.042 / 0.032 | 0.053 | 0.037 | 87% | 0.33 |
| openvins_mono | 0.087 / 0.086 | 0.156 / 0.156 | 0.062 / 0.059 | 0.066 / 0.066 | 0.093 | 0.092 | 88% | 0.21 |
| basalt_stereo_vo (no IMU) | 0.091 / 0.048 | 1.550 / 1.500 | 0.516 / 0.511 | 474.2 / 2.085 | diverges | 1.036 | 100% | 0.13 |
| orbslam3_mono (no IMU) | 3.458 / 0.027 | 3.035 / 0.034 | 6.933 / 0.035 | 2.241 / 0.037 | scale-free | 0.033 | 96% | 0.30 |
| stella_mono (no IMU, baseline) | 3.545 / 0.137 | 1.901 / 0.041 | 1.172 / 0.029 | 1.496 / 0.151 | scale-free | 0.089 | 95% | 0.25 |

Per-run detail (wall time, scale, notes): `runs/vio_compare/table.md`.

### Reading the numbers

- Our measured values reproduce the published ones closely: ORB-SLAM3 stereo-inertial and OKVIS2 are
  the two top systems (0.023-0.027 m SE3), Basalt ~0.05-0.07, OpenVINS ~0.08-0.09, all consistent with the papers
  (mono ORB-SLAM3-I 0.037 Sim3 vs 0.043 paper; Basalt 0.056 vs 0.051-0.072; OpenVINS stereo 0.079 vs 0.117).
- The IMU is what makes the scale metric: Sim3 scale of every VIO row is 0.99-1.02. Camera-only mono (ORB-SLAM3 mono,
  stella mono) is accurate up to scale (Sim3 0.033 / 0.089) but its SE3 error is metres because scale is arbitrary.
  stella_mono is 2-3x worse than ORB-SLAM3 mono on Sim3 (MH_01 0.137, V2_02 0.151 vs 0.027 / 0.037).
- Coverage < 100% is initialisation: OpenVINS waits for motion (static init: first pose 45 s into MH_01), ORB-SLAM3
  mono-inertial also starts ~46 s into MH_01 (IMU initialisation plus map resets). OKVIS2 and Basalt output from the first frames.
- RTF caveats: OKVIS2 runs in its synchronous blocking mode with loop closure and a final full trajectory write, not tuned
  for real time (stereo 1.2-2.1x real time on 8 cores, mono 0.65-0.9). ORB-SLAM3's example normally paces to real time; pacing
  was removed here. Basalt (0.14) is the only system with clear real-time headroom and is also the one whose core is dependency-light.
- Reliability: ORB-SLAM3 mono-inertial segfaulted once on V2_02 during IMU init and passed on the second try (reported
  run is the retry, flagged in `run.json`). Basalt VO diverges without the IMU. Single runs only, no medians; the
  deterministic OKVIS2/Basalt/OpenVINS runs should reproduce, ORB-SLAM3 is multi-threaded and varies.

## GNSS

Only **OKVIS2-X (BSD-3)** is both permissive and ROS-free. It was *not* run: `okvis_multisensor_processing` hard-requires
PCL (LiDAR path), plus supereight2 (needs TBB) and GeographicLib. No root on this box and no PCL available, and the fix is a
CMake/source surgery job, so it was dropped after config attempts (time-box). Cheapest way to try it later: a machine with
`libpcl-dev libgeographiclib-dev libtbb-dev`, then run `okvis_app_synchronous` with a `gps_parameters` block on EuRoC plus a simulated
`mav0/gps0/data.csv` (`timestamp, x, y, z, hErr1, hErr2, vErr`, GT + 1 cm noise as in the paper) and score with `tools/vio_eval.py`.
The GNSS factors live in OKVIS2's own `okvis_ceres` graph, so porting OKVIS2 first leaves GNSS as an incremental factor.
Paper numbers (VBR Campus1, ATE m): VI 2.47, VI+GNSS 0.66; with 75 s GNSS dropout VI+GNSS recovers via global alignment.
All other GNSS-VIO (GVINS, IC-GVINS, GICI-LIB: GPL; R2-GVIO: AGPL; InGVIO: no licence) are reference only.
Cheap public GNSS-VIO datasets exist (GVINS campus bags, IC-GVINS Wuhan set, SYSU-Campus-GVI, Hilti-Oxford/VBR) but all are ROS bags
or need conversion, which was out of scope without a ROS-free consumer.

## Recommendation (permissive port target)

1. **OKVIS2 (BSD-3)** is the best port target for camera+IMU(+GNSS): top accuracy of all systems here
   (stereo 0.023 SE3, mono 0.064), supports one or two cameras, includes loop closure, ROS-free, and is the same lineage as OKVIS2-X which
   adds BSD-3 GNSS factors. Cost: 41k LOC and a hard Ceres dependency (a port needs its own sparse LM/Schur solver, as for the stella BA),
   BRISK features, DBoW2 vocabulary. A port should keep the estimator + frontend and drop the mapping/deep-learning extras, and tune for real time.
2. **Basalt (BSD-3)** is the fallback if stereo-only is acceptable: 4x faster than OKVIS2 here (RTF 0.14),
   no Ceres (own square-root marginalisation, Eigen + TBB), accuracy ~0.056 m. It has no mono mode and no loop closure in the VIO binary.
3. Mono-inertial: nothing permissive beats the GPL systems (ORB-SLAM3-I 0.037 Sim3 / 0.053 SE3). OKVIS2 mono (0.048 / 0.064) is the permissive option
   and matches ORB-SLAM3 mono-inertial on MH_03 / V1_02 (0.054 vs 0.053, 0.028 vs 0.032 SE3), but is worse on MH_01 (0.114 vs 0.083) and V2_02 (0.059 vs 0.042). XRSLAM (Apache-2.0) is the other
   permissive mono candidate but was not benchmarked here.
4. Keep ORB-SLAM3 (GPL) and OpenVINS (GPL) as measured references only.

## Build notes (for whoever repeats this)

- No root, no Eigen/OpenCV/Boost dev packages: every dependency was `apt-get download` + `dpkg -x` into `external/vio/deps/root`; OpenCV 4.6.0 was
  built from the GitHub tarball (core, imgproc, imgcodecs, highgui, calib3d, features2d, flann, video, videoio, no GDAL/GTK). The OpenCV copy used by the
  previous candidate study depends on libgdal/Qt that is not installed, which is why a clean OpenCV was needed. `libopencv_highgui` was replaced by a
  stub (imshow/waitKey no-ops) so OKVIS2's display calls do not throw.
- Gotchas: do not leave `-I$OCV/include` on `CPATH` while building OpenCV itself (stale `opencv_modules.hpp` makes `video` require `dnn`);
  ORB-SLAM3 + g2o must be built with identical `-std`/`-march` flags (aligned-new mismatch gives a double free in g2o);
  ORB-SLAM3 hangs on map reset if the stubbed Viewer never reports `isStopped()`.
- Disk: `external/vio` 1.5 GB (builds + deps), `runs/vio_compare` 58 MB, plus at most one sequence of images (1.3-2.7 GB) at a time.

### Addendum 2026-10-02: deterministic OKVIS2 stereo+IMU reference (okvis_port)

`okvis_port/reference/configs/okvis_stereo_euroc_deterministic.yaml` (stock `config/euroc.yaml` with 1 Ceres thread and
`parallelise_detection: false`) on MH_01_easy, cam0+cam1+imu0, single-threaded schedule of `okvis_port/reference/patches/0001-0006`.
Two runs (one with the M2 dumps enabled, one plain) are byte-identical: `final.csv` sha256 `673fa08fcef39d80...`,
`causal.csv` sha256 `04965fdc29d1fc24...` (a third, earlier count-only run with the pre-0006 binary has the same `final.csv`).
ATE vs the MH_01 GT (`tools/vio_eval.py`, `benchmark.umeyama_alignment`):

| run | SE3 | Sim3 | scale |
|---|---|---|---|
| okvis2_stereo_slam (stock, threaded, table above) | 0.030 | 0.019 | - |
| okvis2_stereo_det (deterministic reference) | 0.0193 | 0.0145 | 1.003 |
| okvis2_mono_det (deterministic mono reference, for comparison) | 0.169 | 0.054 | 1.039 |

Single sequence, one deterministic run each: the stock-vs-deterministic gap (0.030 vs 0.019) is within the spread that
loop-closure timing produces (mono stock 0.114 vs deterministic 0.169 goes the other way); do not read it as an improvement.

## Addendum 2026-10-02: XRSLAM, VINS-Fusion, Kimera-VIO, DM-VIO (mono-inertial, EuRoC)

Benchmark only; the GPL systems (VINS-Fusion, DM-VIO) were only executed, no code copied or ported. Single run per cell on a heavily shared machine (load average 30-60 on 16 cores), so wall-clock RTF is inflated 2-10x for everything here;
use the CPU-seconds in `run.json` (`cpu_s`) for XRSLAM and Kimera. Scoring identical to Part B (`tools/vio_eval.py`, `benchmark.umeyama_alignment`); results in `runs/vio_compare/{xrslam_mono,vinsf_mono,kimera_mono}/` and the regenerated `runs/vio_compare/table.md`.
Builds under `external/vio2/` (gitignored, ~2 GB without images). Scripts: `tools/vio_harness/run_xrslam.sh`, `xrslam/{main_headless.cpp,make_cfg.py}`, `run_kimera.sh`, `kimera_prep.py`, `gpl_glue/{run_vins.sh,play_folder.py,make_vins_cfg.py,run_dmvio.sh,dmvio_prep.py}`.

| system | licence | MH_01 SE3 / Sim3 | MH_03 SE3 / Sim3 | V1_02 SE3 / Sim3 | V2_02 SE3 / Sim3 | mean SE3 | pose coverage | CPU / RTF notes |
|---|---|---|---|---|---|---|---|---|
| xrslam_mono | Apache-2.0 | 0.138 / 0.137 | 0.172 / 0.169 | 0.093 / 0.093 | 0.106 / 0.106 | **0.127** | 94-98% of frames, scale 0.99-1.00 | CPU 190 s for MH_03 (132 s), single thread (threading off by default); wall RTF 0.55-0.70 loaded |
| kimera_mono (BSD-2) | BSD-2 | not run | 4.414 / 2.501 (scale 0.41) | 0.101 / 0.097 | 0.209 / 0.202 | n/a (3 seq: 1.57) | keyframe poses only (25% of frames, span 100%) | CPU 3700-5100 s per 100-130 s sequence, i.e. about 30-40 CPU-s per data second |
| vinsf_mono (GPL, ref) | GPL-3 | 0.272 / 0.220 | 0.140 / 0.135 | **592.7 / 1.796 (scale 0.00)** | 0.081 / 0.080 | n/a (V1_02 diverged) | about 45-49% of frames published, span 88-97% | real-time playback; wall dominated by backlog drain |
| dmvio_mono (GPL, ref) | GPL-3 | not run | not run | not run | not run | - | - | built, see below |
| (existing) okvis2_mono_slam / orbslam3_mono_inertial | BSD-3 / GPL | 0.114 / 0.083 | 0.054 / 0.053 | 0.028 / 0.032 | 0.059 / 0.042 | 0.064 / 0.053 | 100% / 87% | - |

Reading: XRSLAM is the first permissive system here that is metric, covers ~96% and never loses tracking on all four sequences (0.09-0.17 m SE3), but is 2x worse than OKVIS2 mono / ORB-SLAM3-I (0.064 / 0.053) and has no loop closure. Its output starts with one all-zero pose (identity init) that the scorer treats as a normal pose on EuRoC (position only; harmless) and the phone scorer drops.
Kimera-VIO (mono mode through the stereoVIOEuroc binary, EurocMono params, initialised at the GT pose as in the stock script) is accurate on V1_02 (0.10) but loses scale on MH_03 (0.41) and is slow (30-40 CPU-s per data second, i.e. far from real time even on 16 cores); MH_01 was not run (time). VINS-Fusion (no loop closure, shipped EuRoC mono-IMU config, freq 0) is good on MH_03 / V2_02 and fails on V1_02 (scale collapse, 592 m SE3).
Skipped: Kimera MH_01, VINS-Fusion loop closure, a second run of anything (no outlier repeats; machine overloaded, coordinator asked to wrap up).

### Build notes and failures (time-boxes respected)

- **XRSLAM** (OpenXRLab, Apache-2.0, tag `main` of 2026-02): `cmake -B build` superbuild fetches Eigen 3.3.7, Ceres 1.14 (miniglog), spdlog, yaml-cpp, argparse from GitHub (needs `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` with CMake 4.x, and a missing `#include <cstdint>` in `xrslam/src/xrslam/localizer/base64.h` under GCC 13). The stock PC player needs liteviz/GLFW/ImGui; replaced by `tools/vio_harness/xrslam/main_headless.cpp` (a headless loop over the same `XRSLAMPushSensorData` / `XRSLAMRunOneFrame` API, TUM output). Quirks: the EuRoC reader needs CRLF csv files and an `euroc://<abs path>` URL; `XRSLAMDestroy()` segfaults at teardown (the driver `_exit`s after flushing; the MH_01 run predates that and records exit 139 with a complete trajectory); undistortion is done by the reader (cv::undistort) using the config radtan coefficients. ~25 min including fixes.
- **VINS-Fusion** (HKUST, GPL-3): no ROS-free fork was tried; built with the robostack env of `external/gnss/` (catkin_make, Ceres 2.1, OpenCV 4.11) after: `-std=c++17`, a compat header for the removed `CV_*` macros (`CV_GRAY2BGR`, `CV_AA`, `CV_FONT_HERSHEY_SIMPLEX`, `CV_LOAD_IMAGE_*`), `global_fusion` dropped. Driven by a python rospy folder publisher (`gpl_glue/play_folder.py`, real time, no bag on disk) and the node's own `vio.csv`; about 40 min.
- **Kimera-VIO** (MIT-SPARK, BSD-2): built without root, ~80 min. GTSAM 4.2.0 (with `GTSAM_POSE3_EXPMAP`/`ROT3_EXPMAP`, no TBB; the installed `GTSAMConfig.cmake` had `timer` removed from the Boost components because the apt-extracted Boost 1.83 lacks `libboost_timer`, which is then linked from the conda 1.86 copy via `-lboost_timer`), OpenGV, DBoW2, Kimera-RPGO built from source. Kimera hard-requires OpenCV `viz` (VTK), which is not available: replaced by a no-op `opencv2/viz.hpp` stub (`external/vio2/vizstub/`, display only), and OpenCV 4.11 from the conda env (has `rgbd`) for the main build, with `-include cstdint`, `-fpermissive`, `-DKIMERA_BUILD_TESTS=OFF` (the googletest ExternalProject needs CMake <3.5 policy). Kimera has a mono mode (`params/EurocMono`) but its EuRoC provider insists on a `cam1/` folder (symlinked to cam0) and a `state_groundtruth_estimate0/{sensor.yaml,data.csv}` (real GT on EuRoC, dummy on phones).
- **DM-VIO** (GPL-3): GTSAM 4.2a6 built; `dmvio_dataset` builds headless after stubbing Pangolin (`pangolin/var/var.h`, a viewer class, `cv::resizeWindow`) and linking `-lboost_timer`. On the first test (ADVIO-15, 640x352 output) it was killed by the OOM killer within 40 s at 19 GB RSS (pre-processing / startup, cause not diagnosed; DSO-style pipelines allocate per-image pyramids and the interpolated imu.txt was written as required). Dropped as optional; no DM-VIO numbers.
- Not attempted: OKVIS-1, ROVIO, VINS-Mono ROS1 original.

## Note: learned SLAM/VO is out of scope (2026-10-03)

Decision: learned systems are not benchmarked or ported. The goal is a permissive, dependency-free C99 stack. Kept here as a reference only:

- DROID-SLAM (BSD-3) and DPVO / DPV-SLAM: strong published EuRoC/TUM accuracy, but they need a GPU and PyTorch.
- MASt3R-SLAM: non-commercial licence.
- Learned inertial odometry (TLIO, RNIN-VIO, AirIO): relevant to phone scale, but the weights' licences are unresolved. The gait speed prior (`gnss_fusion/c/gf_gait.c`) covers the same need without learning.

Untested classical permissive candidates remain open: MSCEqF (Apache-2.0) and RD-VIO (Apache-2.0) -- both tested 2026-10-03, see the last section.


## Classical candidate scouting (2026-10-03)

Scope: classical (non-learned) open-source VIO / VI-SLAM, 2019-2026, for (a) phones (mono camera + IMU) and (b) drones (mono/stereo + IMU, +GNSS). Learned systems stay out of scope (note above). Benchmark discipline as before: upstream algorithms are untouched, only build / setup / config fixes (all listed below), single runs, repeat only when a result looked like an outlier (none needed: every failure below is deterministic or an immediate crash). Machine shared with two other agents (load average 25-55), so CPU seconds are quoted, not wall RTF. Nothing committed, no data committed.

Artifacts: `runs/vio_compare/table_classical.md` (EuRoC), `runs/gnss_compare/more_systems2/{table.md,table.json,cpu.md}` (phones), `runs/drone_compare/table_classical.md` + `runs/drone_compare/<seq>/<system>/` (drones), scripts `tools/vio_harness/{run_msceqf.sh,run_rdvio.sh,run_eqvio.sh,run_rovio.sh,sqrtvins/run.sh,more_phone_score2.py,score_euroc3.py,drone_table3.py,summarize_classical.py,clean_tum.py,okvis_cfg.py,fetch_phone3.sh}` and the per-system folders `msceqf/ rdvio/ eqvio/ rovio/ sqrtvins/`. Builds in `external/vio3/` (gitignored).

### Survey

Legend: sensors M = monocular camera, S = stereo, I = IMU, G = GNSS. "ROS-free" = builds and runs without ROS (a ROS wrapper may exist). Accuracy = EuRoC ATE RMSE (m) quoted from the cited paper unless marked "ours".

| # | System (repo) | Licence (LICENSE file) | Last push | Sensors | Reported accuracy | Build deps | ROS-free | Notes / status here |
|---|---|---|---|---|---|---|---|---|
| 1 | **RD-VIO** (`Jianxff/rd_vio`, split of `openxrlab/xrslam`) | Apache-2.0 | 2024-04 (xrslam 2026-02) | M+I (mono only) | paper targets dynamic-scene mobile AR; EuRoC "works well on v101"; ours: 0.12-0.19 on 4 EuRoC, 0.7-1.1 m on the 2 indoor phone sequences | Ceres, Eigen 3.4, OpenCV, yaml-cpp (+Pangolin for its viewer, not needed) | yes | IMU-PARSAC (dynamic scenes), pure-rotation sub-frames, rolling-shutter readout parameter. **Same core as XRSLAM**; its setting.yaml has `parsac_flag: true` and a larger window, but switching the flag off gave bit-identical output on Outdoor-1. Tested. |
| 2 | **MSCEqF** (`aau-cns/MSCEqF`) | Apache-2.0 | 2026-07 | M+I (online extrinsic + intrinsic calibration) | 0.13-0.70 m position ATE on EuRoC (paper Table I, mean about 0.36, aligned with the initial state), comparable to OpenVINS MSCKF | Eigen, Lie++, yaml-cpp, Boost, OpenCV (fetched or found) | yes | static start is part of the design (zero-velocity update); built in 2 min. Tested. |
| 3 | **EqVIO** (`pvangoor/eqvio`) | GPL-3.0 (GIFT, LiePP submodules GPL-3.0) | 2024-08 | M+I | mean 0.16 m on EuRoC (paper), ties OpenVINS, 2x faster | Eigen, OpenCV, yaml-cpp, GIFT, LiePP, argparse | yes (ASL reader; ROS optional) | Reference only (GPL). Tested. |
| 4 | **ROVIO** (`ethz-asl/rovio`) | BSD-style (ASL 2014 copyright, 3-clause text) | 2024-01 code (CI pushes to 2026-09) | M+I (EKF, photometric patches) | 0.22 (ORB-SLAM3 paper Table II); ours 0.18 on V1_02 | ROS1 (catkin, roscpp, rosbag), kindr, lightweight_filtering | **no** (ROS1 node) | built in the robostack env, driven by a folder publisher. Tested. |
| 5 | **sqrtVINS** (`rpng/sqrtVINS`) | LGPL-3.0 | 2025-10 | M/S+I, OpenVINS-derived SR-filter, 100 ms dynamic init | claims 2x speed of SOTA on EuRoC/UZH-FPV (no table in abstract) | Eigen, OpenCV, Boost (ROS optional, `ENABLE_ROS=OFF`) | yes | reference only. Mono config diverged on V1_02 here (see below). |
| 6 | XRSLAM (`openxrlab/xrslam`) | Apache-2.0 | 2026-02 | M+I | ours 0.09-0.17 EuRoC (earlier addendum) | Ceres, Eigen, spdlog, yaml-cpp | yes | already tested (best phone result so far). |
| 7 | OKVIS2 / OKVIS2-X (`ethz-mrl`) | BSD-3 | 2026-08 | S/M+I(+G, depth, LiDAR) | 0.03 stereo | Ceres, BRISK, DBoW2 | yes | already tested (best on drones). |
| 8 | **SchurVINS** (`bytedance/SchurVINS`) | GPL-3.0 | 2025-01 | S/M+I, EKF with Schur-complement, SVO front end | EuRoC stereo mean 0.075 vs OpenVINS 0.096 (paper) | ROS, Sophus, glog | no | not tested (GPL, ROS). |
| 9 | **HybVIO** (`SpectacularAI/HybVIO`) | GPL-3.0 (commercial dual licence offered) | 2022-05 | M/S+I, phone-oriented, tested on ADVIO | best real-time results on EuRoC/TUM-VI (paper) | `mobile-cv-suite` (builds OpenCV, FFmpeg, ...), JSONL + MP4 input | yes | **not attempted**: needs FFmpeg + video re-encode of every sequence; GPL. Candidate for a phone reference if someone has time. |
| 10 | **SVO Pro** (`uzh-rpg/rpg_svo_pro_open`) | GPL-3.0 | 2024-01 | M/S+I(+G), semi-direct | UZH-FPV native | catkin, Ceres, OpenGV, glog | partly | not tested (heavy build, GPL). |
| 11 | MINS (`rpng/MINS`) | GPL-3.0 | 2026-09 | multi-sensor MSCKF (camera, IMU, LiDAR, GNSS, wheel) | own papers | ROS | no | not tested. |
| 12 | R-VIO2 (`rpng/R-VIO2`) | GPL-3.0 | 2024-09 | M+I, robocentric MSCKF | own paper | ROS | no | not tested. |
| 13 | DynaVINS (`url-kaist/dynaVINS`), SuperVINS (learned SuperPoint, excluded), PL-VINS (`cnqiangfu/PL-VINS`, line features), VID-Fusion | GPL-3.0 | 2025-08 / 2026-06 / 2023-04 / 2026-03 | M/S+I | own papers | ROS1 | no | VINS-Mono/Fusion derivatives; VINS-Fusion already failed on phones; not tested. |
| 14 | Maplab 2 (`ethz-asl/maplab`) | Apache-2.0 | 2024-05 | M/S+I, ROVIO front end + mapping | mapping tool | catkin, many deps | no | not tested (ROVIO is the VIO inside it). |
| 15 | S-MSCKF (`KumarRobotics/msckf_vio`) | "Penn Software MSCKF_VIO" BSD-like licence text | 2023-11 | **S**+I | 0.41 (ORB-SLAM3 paper) | ROS1 | no | stereo only, stale. |
| 16 | ICE-BA (`baidu/ICE-BA`) | Apache-2.0 | 2018-09 | M/S+I | accuracy numbers not collected | Eigen, OpenCV | yes | pre-2019, stale, not tested. |
| 17 | LARVIO (`PetWorm/LARVIO`) | **no LICENSE file** | 2024-04 | M+I MSCKF | own paper | ROS, SuiteSparse | no | unusable as a source. |
| 18 | LEVIO (`ETH-PBL/levio`) | MIT | 2026-03 | M+I, ORB/BRIEF + pose graph, 100 mW MCU target | "20 FPS < 100 mW" (paper), no ATE in abstract | GAP9 SDK C code, Python reference model | n/a | embedded research prototype, not tested. |
| 19 | Ctrl-VIO (`APRIL-ZJU/Ctrl-VIO`) | no LICENSE file | 2023-08 | M+I, continuous-time rolling-shutter | own paper | ROS | no | rolling-shutter specific, unusable licence. |
| 20 | ORB-SLAM3 forks | GPL-3.0 | - | - | - | - | - | GitHub search (2026-10-03) finds only ROS/ROS2 wrappers, Ubuntu-24.04 ports, and a YOLO dynamic-object variant; no fork with a documented robustness improvement of the inertial initialisation. |
| - | 360_visual_inertial_odometry (MIT, 360 camera), facebookresearch/visual_inertial_bundle_adjustment (MIT, offline BA refinement for Aria), voxel_svio (GPL, stereo), FLVIS (BSD-2, stereo/RGB-D, ROS1), FAST-LIVO2 (LiDAR) | | | | | | | out of scope (sensor set / not a VIO front end). |

### Ranking (promise for a permissive phone / drone front end, with what was measured)

| rank | system | licence | verdict after the runs below |
|---|---|---|---|
| 1 | RD-VIO (`Jianxff/rd_vio`) / XRSLAM | Apache-2.0 | only family (with XRSLAM) that gives metric, full-coverage mono VIO on the indoor phone sequences (0.80-0.93 m SE3, scale 1.01-1.05) and 0.12-0.19 m on EuRoC; collapses in scale on every outdoor phone walk; never initialises on the UZH-FPV flight. RD-VIO is the same core as XRSLAM, so it adds no new capability on phones (IMU-PARSAC on/off made no difference where it could be compared). |
| 2 | OKVIS2 / OKVIS2-X | BSD-3 | still the drone reference (previous study); nothing below beats it. |
| 3 | EqVIO | GPL-3.0 (reference only) | best EuRoC accuracy of the new filters (0.11-0.17 m, mean 0.138 on 3 sequences, scale 1.00) at 40-110 CPU-s per sequence; needs a static start (diverges on MH_01 and on every phone / drone sequence). |
| 4 | MSCEqF | Apache-2.0 | permissive, small (14 MB checkout, 2 min build), 0.18 m on V1_02 / V2_02, 0.67 on MH_03; crashes upstream on MH_01 and loses scale on every phone / drone sequence. Static-start filter by design. |
| 5 | ROVIO | BSD-3 style | EuRoC 0.18-0.74 m (mean 0.43), 25-37 CPU-s per sequence (cheapest of all); ROS1-only; `rovio_node` segfaults on 3 of the 9 non-EuRoC sequences (Indoor-1, m14, of5), scale collapse on the others. |
| 6 | sqrtVINS | LGPL-3.0 (reference only) | mono configuration diverged on V1_02 with three configs and float / double builds (30 min budget used up, not investigated further). |
| 7 | HybVIO | GPL-3.0 (+commercial) | the one phone-first design (ADVIO in its paper); not attempted (FFmpeg + video re-encode + its own OpenCV superbuild exceeded the budget). Worth a try by someone with root. |
| 8 | SchurVINS | GPL-3.0 | not attempted (ROS, GPL); EuRoC stereo mean 0.075 in paper. |
| 9 | SVO Pro | GPL-3.0 | not attempted (heavy catkin build, GPL); UZH-FPV is its native benchmark, so it is the first thing to try if the FPV class matters. |
| 10 | Maplab 2 / S-MSCKF / ICE-BA / LARVIO | Apache-2.0 / BSD-like / Apache-2.0 / none | stale (2018-2024), ROS1 or stereo-only; nothing to gain over ROVIO / OKVIS2. |

Nothing found beats OKVIS2 on the drone sequences or XRSLAM on the phone sequences.

### What was built (setup / config fixes only; no algorithm source changed)

| system | build | fixes (each documented) | time |
|---|---|---|---|
| MSCEqF `2cb653a` | `cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5` against the OpenCV 4.6 / Boost of `external/vio/deps`; Lie++, Eigen, yaml-cpp fetched by FetchContent | own headless driver `tools/vio_harness/msceqf/main_headless.cpp` (the example's `dataParser` needs a ground-truth file and is O(n^2): 5 min for MH_01 parsing alone); `make_cfg.py` writes phone / drone configs from the OKVIS yaml. **Upstream bug found**: with the stock config (`zero_velocity_update: enabled`) the upstream `msceqf_euroc` itself aborts on MH_01 with `std::out_of_range: map::at` in `MSCEqFState::clone` (reproduced with the unmodified example binary and a dummy GT file): the ZVU init path does not clear the tracks recorded before initialisation, so the first non-static frame asks for clones that do not exist. It works on V1_02 / V2_02 / MH_03 (static starts). For sequences that do not start static the config `zero_velocity_update: disabled` (acceleration-spike + disparity init, which clears the tracks) is used, `tools/vio_harness/msceqf/cfg/euroc_zvu_off.yaml`. | 2 min build, about 25 min diagnosing the crash |
| RD-VIO `Jianxff/rd_vio` | `-DTHREADING=OFF` (synchronous, deterministic) | `tools/vio_harness/rdvio/build_fix.patch`: Release -O2 instead of the forced `Debug` / `-Og`, examples (Pangolin viewer) not built, `#include <optional>` in two headers (gcc 13). Ceres 1.14 + Eigen 3.3.7 taken from the XRSLAM superbuild through small CMake config shims (`external/vio3/shim/`) because Ceres 2.2 removed `LocalParameterization`. Own driver `rdvio/main_headless.cpp` (undistorts with the radtan / equidistant coefficients: the library never applies them; flushes the output file before `_exit`). Settings = upstream `configs/setting.yaml` (IMU-PARSAC on, window 12). | about 45 min |
| EqVIO | cmake, GIFT / LiePP / argparse submodules cloned over https | `tools/vio_harness/eqvio/build_fix.patch`: `VIOVisualiser.h` ctor used a member that only exists with visualisation on (upstream does not compile with `EQVIO_BUILD_VISUALISATION=OFF`); `visualiser_stub.cpp` (no-op visualiser) and a stub `GL/glut.h`; `run_eqvio.sh` builds a wrapper dataset dir with a dummy ground-truth file the ASL reader insists on. Config = upstream `EQVIO_config_EuRoC_stationary.yaml` for every sequence (phones, drones included). | about 30 min |
| ROVIO | `catkin_make_isolated` in the robostack env (`external/gnss/mamba/envs/ros`), `-DMAKE_SCENE=OFF`, sources `ethz-asl/rovio`, `ethz-asl/kindr`, `lightweight_filtering` (bitbucket git, submodule path symlinked) | `-fpermissive -include opencv2/imgproc/types_c.h` for the removed `CV_GRAY2RGB`; the unrelated `feature_tracker_node` target does not compile with OpenCV 4 and is not built. Data replayed in real time (rate 1.0) from the fixture folders by `gpl_glue/play_folder.py`; `rovio/record_odom.py` writes `/rovio/odometry`; `rovio/make_cfg.py` writes `rovio.info` (qCM / MrMC from T_SC) and the camera yaml for phones and drones, upstream `rovio.info` + `euroc_cam0.yaml` for EuRoC. | about 35 min |
| sqrtVINS | `ov_srvins` with `-DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF`, float (default) and `-DUSE_FLOAT=OFF` | `-include fstream -include iomanip` (gcc 13); driver adapted from the OpenVINS `run_euroc.cpp`; mono = `max_cameras: 1`, `use_stereo: false`, cam1 and `cam_overlaps` removed from the kalibr chain. | about 40 min, no working result |
| HybVIO, SVO Pro, SchurVINS, others | not built | see ranking | - |

Disk: peak about 11 GB in `external/vio3` while the phone and drone images were on disk (each sequence's images were deleted after its last run; EuRoC sequences were fetched twice for that reason); at the end builds only, see the last line of this section.

### EuRoC results (mono + IMU, ATE RMSE m, SE3 / Sim3; scoring as in Part B, `tools/vio_harness/score_euroc3.py` on `tools/vio_eval.score`)

Data re-fetched from the ETH Research Collection (cam0 + imu0). Single runs.

| system | licence | MH_01 | MH_03 | V1_02 | V2_02 | mean SE3 | CPU s (MH_01 / MH_03 / V1_02 / V2_02) | coverage |
|---|---|---|---|---|---|---|---|---|
| **rdvio_mono** (RD-VIO, new) | Apache-2.0 | 0.166 / 0.164 | 0.190 / 0.187 | 0.124 / 0.124 | 0.130 / 0.129 | **0.153** | 293 / 200 / 106 / 157 | 93-98%, init 2.5-4.7 s |
| **msceqf_mono** (MSCEqF, new, stock cfg) | Apache-2.0 | crash (`map::at`, exit 134) | 0.673 / 0.664 | 0.175 / 0.175 | 0.185 / 0.183 | 0.344 (3 seq.) | - / 33 / 23 / 32 | 99-100% |
| msceqf_mono_zvuoff (MH_01 only) | Apache-2.0 | 51753 / 4.17 (scale 0.000) | | | | | 42 | 100% |
| eqvio_mono (EqVIO, new, GPL ref) | GPL-3.0 | 36342 / 4.25 (scale 0.000) | 0.112 / 0.112 | 0.136 / 0.134 | 0.166 / 0.165 | 0.138 (3 seq.) | 107 / 77 / 41 / 97 | 100% |
| rovio_mono (ROVIO, new) | BSD-3 style | 0.325 / 0.317 | 0.486 / 0.471 | 0.184 / 0.184 | 0.743 / 0.718 | 0.435 | 37 / 29 / 25 / 25 | 100% |
| sqrtvins_mono (sqrtVINS, new, LGPL ref) | LGPL-3.0 | not run | not run | 169 / 1.80 (scale 0.001, three configs, float + double) | not run | diverged | 16 | 95% |
| (ref) xrslam_mono | Apache-2.0 | 0.138 / 0.137 | 0.172 / 0.169 | 0.093 / 0.093 | 0.106 / 0.106 | 0.127 | 190 (MH_03) | 94-98% |
| (ref) okvis2_mono_slam | BSD-3 | 0.114 / 0.057 | 0.054 / 0.053 | 0.028 / 0.022 | 0.059 / 0.059 | 0.064 | - | 100% |
| (ref) orbslam3_mono_inertial | GPL-3.0 | 0.083 / 0.040 | 0.053 / 0.046 | 0.032 / 0.029 | 0.042 / 0.032 | 0.053 | - | 87% |

Reading: RD-VIO is the best of the new systems and sits with XRSLAM (same core; 0.153 vs 0.127), both about 2-3x worse than OKVIS2 mono / ORB-SLAM3-I. EqVIO matches its paper (0.16 mean) and has the best scale (1.00) of all filters but is a static-start filter: MH_01 diverges in both equivariant filters (the drone is handled in the first second, the first 0.5 s window is not static; ROVIO, XRSLAM and RD-VIO handle that start). MSCEqF is 2x worse than the paper on MH_03 (0.67 vs 0.34) and equal or better on V1_02 / V2_02 (0.18 vs 0.20, 0.19 vs 0.55); the paper aligns on the initial state, we align globally (Umeyama), so the numbers are only roughly comparable.

### Phone results (mono + IMU, 15 fps for Mobile-GVIO, 30 fps for ADVIO; same fixtures, GT, clock offsets and scorer as sections 9-10 of the GNSS document: `tools/vio_harness/more_phone_score2.py` -> `gnss_eval.score` -> `benchmark.umeyama_alignment`)

Outdoor-2 is the first 450 s. SE3 = metric alignment (scale not removed), Sim3 = scale free; scale is the Sim3 factor (1.0 = metric); numbers above 1e4 are shown in exponent form (diverged). Full per-run table with timing and gaps: `runs/gnss_compare/more_systems2/table.md`, CPU seconds / exit codes: `cpu.md` (ROVIO's CPU on crashed runs is 0 = not recorded).

| seq | system | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage | CPU s |
|---|---|---|---|---|---|---|
| indoor1 | MSCEqF | 9.97 | 9.90 | 0.930 | 99% | 59 |
| indoor1 | MSCEqF infl | 4253.83 | 16.24 | 0.002 | 99% | 48 |
| indoor1 | RD-VIO stock setting.yaml | 0.93 | 0.92 | 1.009 | 88% | 181 |
| indoor1 | RD-VIO XRSLAM settings | 0.83 | 0.68 | 1.026 | 92% | 173 |
| indoor1 | RD-VIO infl | 4.65 | 3.98 | 0.882 | 92% | 318 |
| indoor1 | EqVIO (GPL ref) | 2.3e+04 | 16.09 | 0.000 | 100% | 92 |
| indoor1 | ROVIO | 35.55 | 0.75 | 0.049 | 8% | 0 |
| indoor2 | MSCEqF | 1861.92 | 10.31 | 0.004 | 99% | 49 |
| indoor2 | RD-VIO stock setting.yaml | 0.80 | 0.50 | 1.052 | 93% | 148 |
| indoor2 | RD-VIO XRSLAM settings | 1.06 | 0.62 | 1.074 | 82% | 185 |
| indoor2 | EqVIO (GPL ref) | 3867.94 | 9.11 | 0.002 | 100% | 105 |
| indoor2 | ROVIO | 1.5e+04 | 9.73 | 0.000 | 81% | 85 |
| advio15 | MSCEqF | 2337.01 | 1.58 | 0.000 | 99% | 31 |
| advio15 | MSCEqF infl | 4020.04 | 1.57 | 0.000 | 99% | 26 |
| advio15 | RD-VIO stock setting.yaml | 768.55 | 1.58 | 0.001 | 97% | 318 |
| advio15 | RD-VIO XRSLAM settings | 907.64 | 1.59 | 0.001 | 97% | 208 |
| advio15 | RD-VIO infl | 1.69 | 0.96 | 23.459 | 98% | 283 |
| advio15 | EqVIO (GPL ref) | 3673.38 | 1.60 | 0.000 | 100% | 88 |
| advio15 | ROVIO | 1744.77 | 1.28 | 0.000 | 57% | 48 |
| outdoor1 | MSCEqF | 2.0e+05 | 58.82 | 0.000 | 100% | 226 |
| outdoor1 | RD-VIO stock setting.yaml | 2.0e+05 | 56.46 | 0.000 | 99% | 1084 |
| outdoor1 | RD-VIO XRSLAM settings | 4.77 | 4.77 | 1.001 | 99% | 818 |
| outdoor1 | RD-VIO infl | 34.62 | 33.52 | 1.165 | 99% | 1188 |
| outdoor1 | EqVIO (GPL ref) | 2.2e+05 | 58.00 | 0.000 | 100% | 518 |
| outdoor1 | ROVIO | 1.3e+05 | 55.72 | 0.000 | 100% | 170 |
| outdoor2 | MSCEqF | 1.2e+05 | 62.54 | 0.001 | 100% | 236 |
| outdoor2 | RD-VIO stock setting.yaml | 1.7e+05 | 67.37 | 0.000 | 98% | 1276 |
| outdoor2 | RD-VIO XRSLAM settings | 2.5e+05 | 66.64 | 0.000 | 98% | 1047 |
| outdoor2 | EqVIO (GPL ref) | 2.7e+05 | 62.20 | 0.000 | 100% | 455 |
| outdoor2 | ROVIO | 3.7e+05 | 62.10 | 0.000 | 99% | 235 |
| advio20 | MSCEqF | 5.4e+04 | 59.46 | 0.001 | 100% | 201 |
| advio20 | RD-VIO stock setting.yaml | 4.1e+04 | 56.42 | 0.001 | 100% | 2109 |
| advio20 | RD-VIO XRSLAM settings | 3.2e+04 | 59.72 | 0.001 | 99% | 1669 |
| advio20 | EqVIO (GPL ref) | 3.7e+05 | 28.27 | 0.000 | 56% | 622 |
| advio20 | ROVIO | 1.5e+05 | 56.59 | 0.000 | 78% | 251 |

Reference rows from the previous sections (not re-run), same sequences:

| seq | XRSLAM default (section 10) | XRSLAM infl | OKVIS2-X mono (section 9) | ORB-SLAM3 mono-I (section 9) | GNSS fixes alone |
|---|---|---|---|---|---|
| Indoor-1 | 0.84 SE3 / 0.73 Sim3, scale 1.02, 92% | - | 8.26 / 4.78, scale 0.73 | 0.51 on 2% | - |
| Indoor-2 | 0.98 / 0.56, 1.07, 82% | - | 27.0 / 9.19, 0.25 | 0.33 on 4% | - |
| ADVIO-15 | 1451 / 1.59, scale 0.00 | 1.55 / 1.51, 1.71 | 1.65 / 1.65, 1.04 | no map survives | - |
| Outdoor-1 | **6.44** / 6.10, 1.03, 99% | 35.2 / 27.5, 1.53 | 74.8 | 5.05 on 55% | 5.73 |
| Outdoor-2 | 252768 / 67.1, 0.00 | - | 8943 (diverged) | 8961 (diverged) | 14.7 |
| ADVIO-20 | 143044 / 60.2, 0.00 | 57265 / 56.7 | 58.8 / 55.8 (880 on the repeat) | no map survives | 12.0 |

Reading: the new systems reproduce, not improve, the XRSLAM picture. Metric and gap-free on the indoor sequences: RD-VIO only (0.80-1.06 m SE3 in both settings, init after 6-9 s, 82-93% coverage), within noise of XRSLAM. Outdoors RD-VIO with the XRSLAM settings matches XRSLAM on Outdoor-1 (4.77 vs 6.44 m, scale 1.001; both below the 5.73 m of the raw phone GNSS) and every other outdoor / ADVIO run of every system collapses in scale. Nothing is better than XRSLAM except that single Outdoor-1 number, which is 1.7 m better but one sequence, one run, with different settings.

### Drone results (same three sequences, conversions, calibrations and reference as `docs/drone_benchmark_20261002.md`; images re-converted with the same scripts; scorer `tools/drone_harness/score.py`, new rows in `runs/drone_compare/<seq>/<system>/`, summary `runs/drone_compare/table_classical.md`)

All new systems run mono (cam0) + IMU with the IMU noise densities and the T_SC of the OKVIS yaml of that sequence; no GNSS input (none of them has a GNSS interface). ATE m, SE3 / Sim3, coverage of frames; CPU seconds.

| seq | system | ATE SE3 (m) | ATE Sim3 (m) | scale | coverage (all / ref window) | CPU s | exit |
|---|---|---|---|---|---|---|---|
| m14 | msceqf_mono (NEW) | 36496.83 | 41.18 | 0.00 | 84% / 84% | 63 | 0 |
| m14 | rdvio_mono (NEW) | 8418.41 | 41.20 | 0.00 | 84% / 84% | 359 | 0 |
| m14 | rdvio_mono_xrsetting (NEW) | 4882.64 | 40.96 | 0.00 | 84% / 84% | 352 | 0 |
| m14 | eqvio_mono (NEW) | 313.02 | 36.16 | 0.06 | 100% / 100% | 127 | 0 |
| m14 | rovio_mono (NEW) | 348.88 | 0.36 | 0.00 | 12% / 12% | 0 | 139 |
| m14 | okvis2_mono | 5.06 | 1.45 | 1.13 | 100% / 100% | - | 0 |
| m14 | okvis2x_mono_nognss | 6.34 | 5.76 | 0.94 | 100% / 100% | - | 0 |
| m14 | okvis2x_mono_gnss | 1.39 | 1.33 | 1.01 | 100% / 100% | - | 0 |
| m14 | orb_mono_inertial | - | - | - | 0% / 0% | - | 139 |
| o1 | msceqf_mono (NEW) | 133.81 | 7.27 | 0.17 | 56% / 56% | 43 | 0 |
| o1 | msceqf_mono_zvu (NEW) | 33.35 | 11.70 | 0.42 | 100% / 100% | 55 | 0 |
| o1 | rdvio_mono (NEW) | 1.66 | 0.34 | 1.07 | 45% / 45% | 284 | 0 |
| o1 | rdvio_mono_xrsetting (NEW) | 2.83 | 0.38 | 1.11 | 47% / 47% | 267 | 0 |
| o1 | eqvio_mono (NEW) | 4.87 | 4.56 | 0.94 | 100% / 100% | 104 | 0 |
| o1 | rovio_mono (NEW) | 1.28 | 0.01 | 0.00 | 10% / 10% | 105 | 0 |
| o1 | okvis2_mono | 2.55 | 1.58 | 1.09 | 100% / 100% | - | 0 |
| o1 | okvis2x_mono_nognss | 0.97 | 0.18 | 1.04 | 100% / 100% | - | 0 |
| o1 | okvis2x_mono_gnss | 0.47 | 0.31 | 1.01 | 100% / 100% | - | 0 |
| o1 | orb_mono_inertial | 9.85 | 0.35 | 1.58 | 41% / 41% | - | 0 |
| of5 | msceqf_mono (NEW) | - | - | - | 0% / 0% | 84 | 0 |
| of5 | rdvio_mono (NEW) | - | - | - | 0% / 0% | 185 | 0 |
| of5 | rdvio_mono_xrsetting (NEW) | - | - | - | 0% / 0% | 187 | 0 |
| of5 | eqvio_mono (NEW) | - | - | - | 0% / 0% | 123 | 0 |
| of5 | rovio_mono (NEW) | - | - | - | 10% / 0% | 0 | 139 |
| of5 | okvis2_mono | 0.53 | 0.41 | 0.99 | 100% / 100% | - | 0 |
| of5 | okvis2_stereo | 1.20 | 0.20 | 1.04 | 100% / 100% | - | 0 |
| of5 | basalt_stereo | 2.30 | 0.40 | 1.09 | 100% / 100% | - | 0 |
| of5 | orb_stereo_inertial | 0.87 | 0.86 | 1.00 | 92% / 100% | - | 0 |
| of5 | orb_mono_inertial | 16.77 | 2.74 | 2.39 | 91% / 100% | - | 0 |

Reading: **nothing beats OKVIS2 on the drone sequences.** o1 is the only sequence where a new system produces a usable track: RD-VIO (both settings) initialises when the drone starts to move (53 s) and follows the flight with scale 1.07-1.11 (SE3 1.66 m / 2.83 m, Sim3 0.34 / 0.38 m on 45-47% of the frames); that is in the range of OKVIS2 mono (2.55 / 1.58, scored over the stationary part too) but behind OKVIS2-X without GNSS (0.97 / 0.18, 100%). m14 (low-texture desert, down-looking) and of5 (20 m/s FPV) break all of them; OKVIS2 mono (5.06 / 1.45 and 0.53 / 0.41) remains the only mono VIO that tracks both.

### Failure diagnosis

- **MSCEqF / EqVIO (equivariant filters) need a static start.** MH_01's first IMU window is not static (|a| = 8.5 m/s^2 at the first sample, the drone is being handled; GT moves 0.3 m in the first 0.5 s): both filters initialise from that window and run away (scale 0.000, Sim3 4.2 m). On the three EuRoC sequences with a calm start both are fine (EqVIO 0.11-0.17, MSCEqF 0.18-0.19 on the Vicon room). Phone walks start with the phone in motion and have no zero-velocity period: MSCEqF reports 226-9016 "Failed update" messages per sequence (no feature tracks long enough for the update), the position blows up. EqVIO has no failure flag at all and writes a divergent trajectory with full coverage. Changing gravity (9.81 to 8.5), the acceleration-spike threshold, the init window or the transition order in the MSCEqF config did not make MH_01 converge (each tried once on the first 30-40 s).
- **MSCEqF crash on MH_01** (stock config): upstream defect, see build table; no source patch applied.
- **Outdoor phone walks: every new system loses metric scale** (scale 0.000-0.001 on Outdoor-1, Outdoor-2, ADVIO-20 for MSCEqF, RD-VIO default, EqVIO, ROVIO), the same failure as XRSLAM default, OKVIS2-X and ORB-SLAM3-MI in sections 9-10 of the GNSS document: slow walking, distant scenery, no excitation of the accelerometer scale. This is a property of the sequences, not of one implementation.
- **RD-VIO settings matter more than the algorithm variant.** The stock `rd_vio/configs/setting.yaml` (window 12, 5 sub-frames, `min_keypoint_distance` 10, `parsac_flag: true`) gives scale 0.000 (SE3 197175 m) on Outdoor-1; with the XRSLAM `iphone_slam.yaml` values (window 10, 3 sub-frames, distance 25, parsac off) the same code gives 4.77 m SE3, scale 1.001, better than XRSLAM's own 6.44 m and than the raw GNSS fixes (5.73 m). Switching only `parsac_flag` off reproduces the stock result bit for bit (197175.70), so the IMU-PARSAC flag has no effect on these sequences and the window / keypoint-distance parameters are what matters.
- **ADVIO-15 / noise sensitivity** (same as XRSLAM): default IMU noise collapses the scale, the OKVIS-tuned densities (`infl`) fix it for RD-VIO on ADVIO-15 only in the Sim3 sense (SE3 1.69 m but scale 23), and break Indoor-1 (4.65 m SE3, scale 0.88 vs 0.93) and Outdoor-1 (34.6 m, scale 1.17 vs 4.77). MSCEqF infl is worse on all three.
- **UZH-FPV of5**: RD-VIO (both settings) and EqVIO never initialise (0 poses; 47 s on the ground, then 20 m/s, the visual-inertial initialisation needs parallax with a sane IMU window), MSCEqF reports a handful of poses then overflows, ROVIO initialises then segfaults. OKVIS2 mono (0.53 m) is untouched.
- **INSANE m14 (desert, down-looking camera, 3-19 m AGL)**: every new system initialises (32 s) and diverges (Sim3 36-41 m); OKVIS2 mono is the only one that tracks.
- **INSANE o1** (55 s static, then flight): RD-VIO initialises at 53 s when the drone starts to move and then tracks the flight (SE3 1.66 m, Sim3 0.34, scale 1.07 on the 45% of frames after initialisation); OKVIS2-X without GNSS 0.97 / 0.18 and OKVIS2 mono 2.55 / 1.58 over the full 99 s (scored with the stationary part, which is easy). MSCEqF with ZVU enabled (its intended mode) is at 33 m SE3 / scale 0.42.
- **ROVIO**: `rovio_node` segfaults after initialisation on Indoor-1, m14 and of5 (same binary runs EuRoC and the other 5 phone sequences without a crash); not debugged. On EuRoC MH_01 / V1_02 it is fine (0.33 / 0.18).
- **sqrtVINS**: mono diverges on V1_02 under the shipped srvins, standard and MSCKF configs, in float and double; the remaining EuRoC / phone runs were not started. Probably a mono-configuration problem of the shipped (stereo) config, not investigated.

### Licence notes

- Permissive and usable as a source: RD-VIO / XRSLAM, MSCEqF (and its Lie++ dependency, Apache-2.0), ROVIO (3-clause BSD text; kindr 3-clause BSD, lightweight_filtering ASL BSD), OKVIS2. Dependencies pulled in by the Apache builds: Ceres (BSD-3), Eigen (MPL-2), yaml-cpp (MIT), spdlog (MIT, bundled in rd_vio).
- GPL / LGPL, executed as references only, no code copied: EqVIO and its GIFT / LiePP submodules (GPL-3.0), sqrtVINS (LGPL-3.0). Glue that only drives them lives in `tools/vio_harness/eqvio/` and `tools/vio_harness/sqrtvins/` (patch + stub + driver; the GPL sources stay in `external/vio3/`).
- No LICENSE file: LARVIO, Ctrl-VIO (unusable). Datasets: Mobile-GVIO CC BY 4.0, ADVIO CC BY-NC 4.0, EuRoC InC-NC, INSANE BSD-2 + no-selling, UZH-FPV CC BY-NC-SA 3.0; no data is committed.

### Reproduce

```bash
# EuRoC (cam0 + imu0) into external/vio3/euroc; phones into external/vio3/phone/<seq> (tools/vio_harness/fetch_phone3.sh <seq>)
VIO_DATA_ROOT=$PWD/external/vio3/euroc external/gnss/venv/bin/python tools/vio_harness/fetch_seq_stream.py V1_02_medium
tools/vio_harness/run_msceqf.sh V1_02_medium; tools/vio_harness/run_rdvio.sh V1_02_medium; tools/vio_harness/run_eqvio.sh V1_02_medium; tools/vio_harness/run_rovio.sh V1_02_medium 1.0
external/gnss/venv/bin/python tools/vio_harness/score_euroc3.py msceqf_mono rdvio_mono eqvio_mono rovio_mono   # -> runs/vio_compare/table_classical.md
tools/vio_harness/run_rdvio.sh outdoor1 default            # NOISE=infl for the inflated densities; TAG=xrsetting SETTING=tools/vio_harness/rdvio/cfg/setting_xrslam_iphone.yaml for the XRSLAM settings
tools/vio_harness/run_msceqf.sh drone_o1                    # drone_m14 | drone_o1 | drone_of5, data from tools/drone_harness/{insane,fpv}_to_euroc.py into external/vio3/drone/
external/gnss/venv/bin/python tools/vio_harness/more_phone_score2.py; python3 tools/vio_harness/summarize_classical.py   # phones -> runs/gnss_compare/more_systems2/
python3 tools/vio_harness/clean_tum.py <trajectory files>; python3 tools/drone_harness/score.py o1 external/drone/ins_o1; python3 tools/vio_harness/drone_table3.py   # drones -> runs/drone_compare/table_classical.md
```

Disk at the end: `external/vio3` 0.35 GB (builds, sources, trajectories; all images deleted), peak about 11 GB with images on disk. Nothing committed.
