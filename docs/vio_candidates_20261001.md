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
