# VIO harness (benchmark only, not product code)

Scripts used to produce `runs/vio_compare/` (see `docs/vio_candidates_20261001.md`). Everything here
targets a checkout under `external/vio/` (gitignored). Paths are hard-coded to this machine.

- `env.sh`: prefix env for a no-root box (deps are `apt-get download` + `dpkg -x` into `external/vio/deps/root`).
- `fetch_seq.py` + `remotezip.py`: stream one EuRoC ASL sequence out of the ETH Research Collection
  (HTTP range requests into the nested zip; robotics.ethz.ch hangs). Data is deleted after each sequence.
  EuRoC licence: "In Copyright - Non-Commercial Use Permitted". Never commit the data.
- `fetch_seq_stream.py <seq> [cam0,imu0,...]`: same source as `fetch_seq.py` but parses the nested ASL zip as a stream and writes only the requested sensor folders
  (default cam0 + imu0 + sensor yamls): no 1.5 GB temporary zip, no cam1 (used by the okvis_port reference runs).
- `run_<system>.sh <seq> [mode]`, `run_all.sh <seq>`: run one system, write `runs/vio_compare/<system>/<seq>/{trajectory.tum,run.json}`.
- Scoring: `tools/vio_prep_gt.py` (GT -> TUM) and `tools/vio_eval.py` (uses `benchmark.umeyama_alignment/ate_rmse`).
- `orbslam3_stubs/`: headless replacements for ORB-SLAM3's Viewer.cc / MapDrawer.cc + fake `pangolin/pangolin.h`
  (copied over `orbslam3/src` and `orbslam3/stub/pangolin`); also removed the example's real-time `usleep` pacing
  and built g2o/DBoW2 with the same `-std=c++14 -march=native` as the library (otherwise: double free in g2o).
- `openvins/run_euroc.cpp`: ROS-free EuRoC driver around `ov_msckf::VioManager` (added to `ov_msckf/cmake/ROS1.cmake`,
  built with `-DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF`); `openvins/ov_cfg/` = upstream euroc_mav config, mono variant sets `max_cameras: 1`.
- Basalt = prebuilt GitLab release 0.1.7 (x86_64 Linux), run headless (`--show-gui 0`).
- OKVIS2 = `okvis_app_synchronous`, stub libopencv_highgui (no GUI), `okvis_mono_euroc.yaml` = euroc.yaml with cam1 removed.

## More systems (2026-10-02, builds in `external/vio2/`)

- `run_xrslam.sh <seq> [infl]`, `xrslam/{main_headless.cpp,make_cfg.py}`: XRSLAM (Apache-2.0) headless driver and device-config generator (phones from `tools/gnss_harness/robust_cfg/<seq>/okvis_default.yaml`).
- `run_kimera.sh <seq>`, `kimera_prep.py`: Kimera-VIO mono through `stereoVIOEuroc` + EurocMono params (no-op viz stub, GTSAM 4.2, OpenGV, DBoW2, Kimera-RPGO).
- `more_phone_score.py`, `more_fusion.py`: phone scorer (-> `runs/gnss_compare/more_systems/`) and the loose-fusion wrapper.
- `gpl_glue/run_vins.sh`, `play_folder.py`, `make_vins_cfg.py` (VINS-Fusion via the robostack env), `run_dmvio.sh`, `dmvio_prep.py` (DM-VIO, OOM): GPL reference drivers only.

## Classical candidate scouting (2026-10-03, builds in `external/vio3/`)

See the last section of `docs/vio_candidates_20261001.md`.
- `run_msceqf.sh`, `run_rdvio.sh`, `run_eqvio.sh`, `run_rovio.sh`, `sqrtvins/run.sh`: `<seq>` is an EuRoC name, a phone name (indoor1 indoor2 outdoor1 outdoor2 advio15 advio20) or `drone_{m14,o1,of5}`; env `NOISE=infl`, `TAG=...`, `SETTING=...`, `MSCEQF_ZVU=1`, `SYSNAME=...` select variants.
- `msceqf/`, `rdvio/`, `eqvio/`, `rovio/`, `sqrtvins/`: headless drivers (own csv readers), config generators (`make_cfg.py` from an OKVIS yaml via `okvis_cfg.py`), build fixes (`rdvio/build_fix.patch`, `eqvio/build_fix.patch`, `eqvio/visualiser_stub.cpp`). GPL / LGPL targets (EqVIO, sqrtVINS) are executed only.
- `fetch_phone3.sh <seq>`: re-creates the phone image fixtures in `external/vio3/phone/<seq>` (csv fixtures are reused from `external/vio2/phone`); `fetch_seq_stream.py` honours `VIO_DATA_ROOT`.
- Scoring: `score_euroc3.py` (-> `runs/vio_compare/table_classical.md`), `more_phone_score2.py` + `summarize_classical.py` + `phone_summary3.py` (-> `runs/gnss_compare/more_systems2/`), `drone_table3.py` (-> `runs/drone_compare/table_classical.md`), `clean_tum.py` (drops non-finite / truncated rows of diverged runs before scoring).
