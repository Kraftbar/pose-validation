# VIO harness (benchmark only, not product code)

Scripts used to produce `runs/vio_compare/` (see `docs/vio_candidates_20261001.md`). Everything here
targets a checkout under `external/vio/` (gitignored). Paths are hard-coded to this machine.

- `env.sh`: prefix env for a no-root box (deps are `apt-get download` + `dpkg -x` into `external/vio/deps/root`).
- `fetch_seq.py` + `remotezip.py`: stream one EuRoC ASL sequence out of the ETH Research Collection
  (HTTP range requests into the nested zip; robotics.ethz.ch hangs). Data is deleted after each sequence.
  EuRoC licence: "In Copyright - Non-Commercial Use Permitted". Never commit the data.
- `run_<system>.sh <seq> [mode]`, `run_all.sh <seq>`: run one system, write `runs/vio_compare/<system>/<seq>/{trajectory.tum,run.json}`.
- Scoring: `tools/vio_prep_gt.py` (GT -> TUM) and `tools/vio_eval.py` (uses `benchmark.umeyama_alignment/ate_rmse`).
- `orbslam3_stubs/`: headless replacements for ORB-SLAM3's Viewer.cc / MapDrawer.cc + fake `pangolin/pangolin.h`
  (copied over `orbslam3/src` and `orbslam3/stub/pangolin`); also removed the example's real-time `usleep` pacing
  and built g2o/DBoW2 with the same `-std=c++14 -march=native` as the library (otherwise: double free in g2o).
- `openvins/run_euroc.cpp`: ROS-free EuRoC driver around `ov_msckf::VioManager` (added to `ov_msckf/cmake/ROS1.cmake`,
  built with `-DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF`); `openvins/ov_cfg/` = upstream euroc_mav config, mono variant sets `max_cameras: 1`.
- Basalt = prebuilt GitLab release 0.1.7 (x86_64 Linux), run headless (`--show-gui 0`).
- OKVIS2 = `okvis_app_synchronous`, stub libopencv_highgui (no GUI), `okvis_mono_euroc.yaml` = euroc.yaml with cam1 removed.
