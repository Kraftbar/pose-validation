# GNSS-VIO harness (benchmark only)

Everything builds under `external/gnss/` (gitignored). See `docs/gnss_vio_benchmark_20261001.md`.

- `bag_head.py`, `bag_subset.py`: pure-python ROS1 bag reader tolerant of truncated files (HTTP-range heads of 20-35 GB bags); subset writer.
- `http_zip.py`: remote zip member streaming (Zenodo). `gvins_bag_to_euroc.py`, `mobilegvio_to_euroc.py`: dataset -> EuRoC folder + GNSS/GT csv.
- `make_gps.py`, `mobile_make_gps.py`: GNSS input variants (RTK, simulated SPP-grade noise, blackout windows).
- `make_okvis2x_cfg.py`, `make_okvis2x_cfg_mobile.py`, `run_okvis2x.sh`: OKVIS2-X (BSD-3) configs/runner. `pcl_stub/`: minimal PCL stand-in (PLY dump only) so OKVIS2-X builds without PCL.
- `gnss_eval.py`, `score_all.py`, `estimate_offset.py`: scoring (uses `benchmark.umeyama_alignment`), table writer, GT clock-offset estimator.
- `gpl_glue/`: scripts that drive GPL reference systems (GVINS, IC-GVINS) through ROS; keep out of git if the repo must stay GPL-free.
- Own algorithm: `tools/gnss_loose_fusion.py` (numpy only).
- Phone/handheld robustness study (section 8 of the note): `fetch_complex_images.py` (stream the GVINS bag head, cam0 only), `make_layout.py` / `make_fixtures.py` (EuRoC / TUM-style folders + PGM fixtures for `sv_run`),
  `make_robust_cfgs.py` (configs in `robust_cfg/`), `gpl_glue/run_rob.sh` (stella upstream, stella port, ORB-SLAM3 mono / mono-inertial, OpenVINS, OKVIS2-X), `robust_score.py` (table), `robust_fusion.py` (loose smoother / Sim3 fit on a run),
  `robust_diag.py`, `frontend_probe.py`, `cam_imu_offset.py` (camera-IMU time offset from visual vs gyro angular speed).
