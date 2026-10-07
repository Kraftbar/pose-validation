# OKVIS2-X deterministic single-threaded reference (GNSS path)

Mirrors `okvis_port/reference/`. Never edits `external/gnss/OKVIS2-X` (BSD-3, commit 38043e4).
`python3 tools/build_okvis2x_reference.py` copies it (no `.git`/`build/`/weights) to `runs/okvis2x_port/reference_build/src`,
restores `okvis_multisensor_processing/CMakeLists.txt` from git HEAD (the working tree carries an older hand edit), adds the
`supereight2` submodule (not checked out in `external/`; cloned from github.com/ethz-mrl/supereight2 at the commit the X tree
records, f98c564, into `runs/okvis2x_port/deps/se2`), applies `patches/*.patch`, and builds `okvis_app_synchronous` with
`-O2 -DNDEBUG -ffp-contract=off -fno-fast-math` (no `-march=native`, max 4 jobs) against `external/vio/deps` (Eigen 3.4.0,
glog, Boost, OpenBLAS, OpenCV 4.6.0 stub highgui, TBB, GeographicLib: all in that prefix; CMake needs
`-DCMAKE_MODULE_PATH=<deps>/usr/share/cmake/geographiclib`). `BUILD_ROS2/USE_NN/HAVE_LIBREALSENSE=OFF`. Mapping (`okvis_mapping`)
and supereight2 cannot be switched off by an option (okvis_ceres / multisensor_processing link them), they are compiled but
unused (`enable_submapping: false`). `okvis2x_app_synchronous` (lidar/depth app) is not built; X's `okvis_app_synchronous`
already reads `gps0/data.csv` and writes the global trajectory. Provenance: `runs/okvis2x_port/reference_build/provenance.json`.

| patch | content |
|---|---|
| `0001-deterministic-single-threaded.patch` | realtime and background (loop-closure / GNSS full-graph) optimisation joined inside the frame that starts them; blocking publication push; `matchToMap*` segments run sequentially (same partition); OpenGV RANSAC RNG seeded 12345 |
| `0002-zero-init-descriptor-pool.patch` | zero-init of the `matchToMap` descriptor pool |
| `0003-drain-publication-queue.patch` | `stopThreading` drains the publication queue first |
| `0004-build-pcl-stub.patch` | PCL replaced by `pcl_stub/` (only debug PLY dumps use it); build only |

X-specific differences handled: X's app already calls `setBlocking(true)` (frame/IMU/GPS queues never drop); the stereo-init
matcher workers (`Frontend.cpp` ~2015) write disjoint slots and are left alone (as in okvis_port); GPS fixes are pushed by the
dataset reader in image-timestamp order, so no further patch was needed (2 runs identical, see below).

## Configs (`configs/`)
`okvis2x_{mono,stereo}_euroc_deterministic.yaml` = okvis_port deterministic configs plus the keys X requires
(`cam_model: pinhole`, `imu_parameters: s_a: [1,1,1]`, `output_parameters: display_topview: false`, `enable_submapping: false`).
`okvis2x_mono_euroc_gps_robust{false,true}_deterministic.yaml` = mono config + `gps_parameters` {data_type cartesian,
r_SA [0.05, 0.02, 0.10], yaw_error_threshold 1.0, robust_gps_init false | true}. (`do_loop_closures` stays true; config/gvins uses false.)

## Run
```
VIO_DATA_ROOT=$PWD/runs/okvis2x_port/data python3 tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,cam1,imu0,state_groundtruth_estimate0   # 2.6 GB
external/gnss/venv/bin/python tools/okvis2x_make_euroc_gps.py runs/okvis2x_port/data/MH_01_easy      # mav0/gps0/data.csv
python3 tools/okvis2x_run_reference.py MH_01_easy --tag gps_a_mono1 --config okvis2x_port/reference/configs/okvis2x_mono_euroc_gps_robustfalse_deterministic.yaml
external/gnss/venv/bin/python tools/okvis2x_eval.py runs/okvis2x_port/reference_runs/MH_01_easy/gps_a_mono1/global_final.csv --antenna
```
GNSS data: GT pose p + R r_SA, 5 Hz (760 fixes incl. 150 dropped by the blackout of 30 s starting 60 s after the first camera
frame), Gaussian noise 1 cm (x, y) / 2 cm (z), numpy seed 1, reported sigmas 0.01/0.01/0.02, written in the GT frame.
Outputs: `final.csv`, `causal.csv` (X adds columns NrGps, SID, gpsMode), `global_final.csv` (GNSS runs; antenna position `p_GA_G`).

## Results (MH_01_easy, wall 220-235 s mono, 450 s stereo, one thread)
Canonical OKVIS2 (okvis_port): mono final `dfe3b58e..` causal `cc29a746..`; stereo final `673fa08f..` causal `04965fdc..`.

| run (2 runs each) | determinism | final.csv sha256 | causal.csv sha256 |
|---|---|---|---|
| X, GNSS off, mono | identical | `c3ffb4ada4c6..` | `144d71718396..` |
| X, GNSS off, stereo | identical | `a1edc9e7cc86..` | `70ee15a6c17a..` |
| X, GNSS on, robust_gps_init false (a) | identical (+ global `0f711351270e..`) | `410c8a888fb6..` | `deabab007e3d..` |
| X, GNSS on, robust_gps_init true (b) | identical (+ global `3e495bd69d64..`) | `eb7aa974a95a..` | `1e97d2eb4eae..` |
| X, GNSS on, robust true, `data_r2` (20 Hz, 1/2 cm, seed 5, blackout 30 s at 60 s) = `gps_b_r2_{1,2}` | identical (+ global `4facea3e64a4..`) | `1613128333cc..` | `078b9fdd7c66..` |
| X, GNSS on, robust true, `data_r1` (20 Hz, 2/4 cm, seed 4, blackout 12 s at 70 s) = `gps_b_r1_{1,2}` | identical (+ global `5d1f47c3c982..`) | `b0f47380264e..` | `8811174e282b..` |

Base drift X vs OKVIS2 (GNSS off): different from the first row (final.csv initial quaternion differs in the 3rd digit).
Mono: position RMS difference final 0.149 m (max 0.297), causal 0.274 m (max 0.714); stereo: final 0.026 m (max 0.062), causal
0.044 m (max 0.154). ATE vs GT (`benchmark.ate_rmse`, Umeyama + scale; 3638 matched poses): OKVIS2 mono final 0.0536 / causal
0.0881, X mono final 0.0378 / causal 0.1959; X stereo final 0.0190 / causal 0.0306.

GNSS (mono): (a) log shows `First measurements added`, `GPS-VIO extrinsics have become observable` (state 338, first
`T_GW` estimate), two `Kicking off full graph optimisation due to gps loop closure` (states 1-338, 1129-1842, after the blackout),
`GPS fully re-Initialised`. `global_final.csv` vs GT antenna positions without any alignment: RMS 0.0394 m, max 0.177 m (ATE
Umeyama+scale 0.0344); final.csv (VIO frame) ATE 0.0342; causal 0.1037 (max 2.78 m: pre-alignment states).
(b) (5 Hz data) the init never completes: only `First measurements added` is logged; the cause IS now pinned down (HANDOVER_gnss_robust.md section 1:
`estimateRigidRansac` needs >= 2 * 20 = 40 points, the realtime graph's sliding window holds <= 27 fixes at 5 Hz); the 20 Hz datasets `data_r1` / `data_r2` (generated with
`tools/okvis2x_make_euroc_gps.py --rate 20`, see HANDOVER_gnss_robust.md) do initialise (r2: state 392, dropout, re-init; r1: state 900); trajectory is the GNSS-off one (final ATE 0.0376,
`global_final.csv` raw RMS 5.94 m = still in the VIO frame). Cause not pinned down (rejections are `DLOG`, compiled out by
`-DNDEBUG`); hypothesis: robust mode only uses the last 100 states for `checkForGpsInit`, then RANSAC (`estimateRigidRansac`)
and the 1 degree yaw-uncertainty gate (`ViGraph.cpp` ~1015-1135).

## Dump patches 0005-0015 (ports of okvis_port 0003-0014, observe-only)
`0005` imu-error, `0006` solver-trace, `0007` kinematics/camera, `0008` error-terms/manifolds, `0009` solver, `0010` graph/two-pose/problem,
`0011` vigraph mutation log (+ `EucmCamera::portHeader` stub, X-only), `0012` backend call log, `0013` frontend input log, `0014` ransac log,
`0015` place recognition (okvis_port 0006 = X 0003 already). Same env vars / record layouts as okvis_port. Run:
`python3 tools/okvis2x_run_reference.py MH_01_easy --tag x_m8 --consolidated` then
`python3 tools/check_okvis_port.py --runs-dir runs/okvis2x_port/reference_runs --seqs MH_01_easy --tag x_m8 --native-solve 1`.
Trajectory columns 1-17 (final and causal) are byte-identical with and without the dumps (X writes uninitialised garbage into columns 18-21 of
final.csv, so the file hash itself differs between builds). `experiments/exp-okvis2-behaviour-toggles.diff`: env-var switches (default off) that
restore OKVIS2 behaviour (updateLandmarks criterion, no T_GW block, single removeOutliers, 0.4 DBoW score) used for the bisection.
