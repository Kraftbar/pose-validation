# phone_pipeline: end-to-end phone pipeline (images + IMU + GNSS fixes -> one continuous metric trajectory)

Own code, MIT (`LICENSE`). Roadmap item 2 (`docs/roadmap_research_20261003.md`). Everything numeric is C99 and deterministic; the python files only move files, build the
inputs of the C programs and score. Results and the full discussion: `docs/gnss_vio_benchmark_20261001.md` section 13; raw outputs `runs/phone_pipeline/` (`tables.md`, `<seq>/scores.json`).

```
images ----------------------> stella_vio/sv_run   multi-map ORB mono SLAM at full resolution (BSD-2 / MIT, deterministic)
IMU (gyro, accel) -----------/   --imu: gravity per map, optional gyro prior; --set rframe=1 merge=1 (R-frames through tracking failures, map merge)
                                 -> trajectory_maps.tum (t pose map-id rframe-flag segment-id), trajectory_gz.tum (gravity-aligned per map), gravity.txt
IMU --> gnss_fusion/c/gf_gait_run (step cadence -> walking speed, 3 s epochs / 6 s window, per-user constant from held-out sequences)
phone GNSS fixes (outdoor)
        \_______________________ gnss_fusion/c/gf_run   robust preset + gait speed prior (+ speed alignment where no fixes)
odometry flags from the SLAM output:                      new map id            -> GF_ODOM_NEW_FRAME (own unit / yaw / origin, aligned from its fixes or the gait speed)
                                                           new segment (bridge)  -> GF_ODOM_GAP | GF_ODOM_LOOSE
                                                           R-frame sample        -> GF_ODOM_LOOSE (link sigma x loose_k = 5: position is extrapolated)
                                                           tracking gap > 2 s    -> automatic gap; GNSS-only nodes (one per fix) keep the output continuous
        --> (section 14, `run.py georef`) gait-only stream + gnss_fusion/c/gf_georef_run: ONE slowly varying similarity from the fixes, the live output beats GNSS alone on all three outdoor sequences
        --> batch (whole-graph smoother) and causal (sliding window; `causal.live` = what an online consumer sees) trajectories, metric, in the ENU frame of the fixes when fixes exist
```

## Run

```bash
PY=external/gnss/venv/bin/python           # numpy + opencv (for the JPEG -> gray PGM feeder only)
make -C stella_vio && make -C gnss_fusion/c
# images: the fetch layout of tools/gnss_harness (cam0/data/*.jpg + cam0/data.csv, see docs/gnss_vio_benchmark_20261001.md 9.2); gray PGMs are produced just in time
$PY phone_pipeline/run.py stages outdoor1 --stream <fetch_dir>/outdoor1          # sv_run variants (lock-step), gait, every fusion mode -> runs/phone_pipeline/outdoor1/
$PY phone_pipeline/score.py outdoor1                                              # scores.json (ATE SE3 / Sim3 / scale / coverage, batch + causal)
$PY phone_pipeline/report.py > runs/phone_pipeline/tables.md                      # all tables
phone_pipeline/check_baselines.sh <tmpdir>                                        # exact-port + gnss_fusion regression checks
$PY phone_pipeline/run.py georef outdoor1 --variants default,rm,full,fullcal      # section 14: fuse_<variant>_georef/ from fuse_<variant>_gait + fixes (needs the gait stage); then score.py / report.py
$PY phone_pipeline/fuse_eval.py "label|key=val ..." "georef|G: scale_sigma=0.15"   # fusion-stage-only experiments on the saved inputs (scratch dir, no stella_vio); prints batch / causal vs GNSS alone
```
`run.py sv|fuse|gait|stages <seq>`: `--variants default,rm,full,fullcal`, `--fuse gait,gnss,both`, `--stream DIR`. Output of one run: `sv_<variant>/` (SLAM outputs), `speed.txt`, `fuse_<variant>_<gait|gnss|both>/`
(`odom.txt fix.txt batch.out causal.out causal.live *.nodes run.json`). Fusion needs only `trajectory_maps.tum`, `trajectory_gz.tum`, the IMU csv and the fixes; the same four
steps can be run by hand: `sv_run ... --imu imu.csv --imu-ext ext.txt --imu-toff s --imu-bg bx,by,bz --set gravity=1 [--set rframe=1 --set merge=1 --set gyro=1]`,
`gf_gait_run --imu imu.csv --epochs ep.txt --epoch-dt 3 --window 6 [--model c]`, then `gf_run --odom odom.txt --fix fix.txt --speed speed.txt --out batch.out --mode batch preset=robust metric=0 rsa=0,0,0 speed=1 speed_align=1 speed_scale_rw_rel=1 speed_align_metric=1 loose_k=5` (and `--mode causal --out-live`).

Variants (`run.py` `SV_VARIANTS`): `default` = stella_vio defaults + `gravity=1` (trajectory unchanged by gravity); `rm` = + `rframe=1 merge=1`; **`full` = + `gyro=1` (the pipeline)**;
`fullcal` = full with the dataset-calibration camera-IMU extrinsic instead of the sequence-fitted one (sensitivity). Fusion modes: `gait` (no fixes), `gnss` (fixes, no gait), `both`.
Sequences without usable fixes (Indoor-1/2: iPhone indoor fixes have sigma 20-67 m; ADVIO-15: 38 indoor fixes) run GNSS-free: metric scale and yaw come from the gait speed only, the output frame is arbitrary (metric, not geo-referenced).

## Live mode and automatic switch (section 15)
```bash
make -C phone_pipeline/c                                   # pp_live: stella_vio + gf_gait + gf_auto in one process (binary ignored by git)
$PY phone_pipeline/run.py live outdoor1 --variants full --stream <fetch_dir>/outdoor1 [--keep-jpeg] [--pp "--pp-set policy=1"]   # -> runs/phone_pipeline/outdoor1/live_full/
$PY phone_pipeline/live_eval.py outdoor1 && $PY phone_pipeline/live_report.py > runs/phone_pipeline/live_tables.md
$PY phone_pipeline/auto_eval.py --src both                 # replay of final / live odometry through gnss_fusion/c/gf_auto_run (smoother / georef / auto)
```
Each frame's pose is emitted when it is computed (`sv_run --live-out`, opt-in) and streamed through gait, the two smoothers, the georef and the switch; `pp.auto` is what a live consumer sees. Results (live vs final, switch table, CPU per stage): `docs/gnss_vio_benchmark_20261001.md` section 15, `runs/phone_pipeline/live_tables.md`. Short: live costs -7..+4 % on the outdoor sequences and 3.5x on Indoor-2 (0.31 -> 1.10 m); AUTO = georef on all three outdoor sequences (4.31 / 11.33 / 11.82 live), smoother elsewhere.

## Configs (`configs/<seq>.json`, one per dataset)
Camera intrinsics (dataset calibration, `tools/gnss_harness/robust_cfg/<seq>/port_camera.txt`), IMU csv, camera-IMU extrinsic (`imu_ext`: sequence-fitted rotation from `stella_vio/tools/ext_fit.py`; `imu_ext_cal`: dataset calibration),
camera-IMU time offset `imu_toff` (same fit; ADVIO -0.31 / -0.32 s: its video clock lags the IMU clock), gyro bias `imu_bg` (first <= 60 s visual fit, `runs/stella_vio/imu/gyro_pred.md`, only used by `gyro=1`), fixes file,
and the gait calibration mode: **Mobile-GVIO: per-user constant fitted on the GT speed of the other three Mobile sequences only (the sequence itself is held out, `gnss_fusion/work/gait_cal.json`); ADVIO: generic constant 0.389**
(another person and phone, no held-out calibration sequence of that user exists).

## Interface changes in the existing modules (all opt-in, defaults unchanged, checked by `check_baselines.sh`)
* `stella_vio/sv_run --wait-fixtures`: a missing fixture PGM is waited for (up to 10 min) so that a streaming feeder can produce and delete PGMs (peak disk 1.5 GB instead of 5-8 GB per sequence).
* `gnss_fusion`: `GF_ODOM_LOOSE` odometry flag + `loose_k` (link sigma multiplier, default 1: the flag does nothing); `speed_align_metric` (monocular speed alignment tests `seg_min_extent` on the metric path instead of the odometry units; default 0 = old behaviour; needed because stella maps are in arbitrary units, e.g. ADVIO-15 map 0 is 3.2 units long).
* Baselines: stella_vio fr1_xyz (`reinit_sec=0 init_max_level=0 init_confirm=1`) `trajectory.tum` is `cmp`-identical to stella_port's `sv_run` (787 poses); `gf_table.py` (893 numbers incl. section 11) and `gf_gait_study.py fusion` (1578 numbers, section 12) reproduce the saved values exactly; `compare_py.py` (C vs python, 8 cases) and `test_geo.py` pass (`runs/phone_pipeline/check_baselines.log`).

## Results
One deterministic run per row (the front end is chaotic: only >2x differences are rankings). ATE SE3 [m] against the phone GT, output resampled to the camera frames. Full matrix: `runs/phone_pipeline/tables.md`; discussion, calibration honesty, open issues: `docs/gnss_vio_benchmark_20261001.md` section 13.

### Headline: full pipeline (stella_vio gyro+R-frames+merge, gait, GNSS where usable), ATE SE3 [m], batch / causal

| sequence | full pipeline batch / causal | Sim3 batch / causal | scale ratio (est/true) batch | coverage batch / causal | rms distance to the fixes (geo-referencing check) batch / causal | GNSS alone | XRSLAM | RD-VIO (xrsetting) | OKVIS2-X | best earlier fusion, full-coverage odometry (batch / causal30) |
|---|---|---|---|---|---|---|---|---|---|---|
| Indoor-1 | **1.16 / 1.02** | 1.15 / 0.94 | 0.99 | 100% / 100% (90% of all frames) | n/a (no fixes) | n/a | 0.84 | 0.83 | 8.26 | 0.28 (i1_stella) / 0.35 (i1_stella) |
| Indoor-2 | **0.31 / 0.31** | 0.30 / 0.31 | 0.99 | 100% / 100% (88% of all frames) | n/a (no fixes) | n/a | 0.98 | 1.06 | 27.00 | 0.27 (i2_stella) / 0.27 (i2_stella) |
| Outdoor-1 | **5.50 / 7.10** | 5.50 / 6.71 | 1.00 | 100% / 100% (92% of all frames) | 2.50 / 9.63 | 5.73 | 6.44 | 4.77 | 32712 | 4.95 (o1_stella) / 5.82 (o1_xrslam) |
| Outdoor-2 | **13.28 / 15.10** | 13.12 / 9.55 | 0.98 | 100% / 100% (93% of all frames) | 5.44 / 17.81 | 14.66 | 252768 | 254661 | 8943 | 14.39 (o2_okvis) / 16.62 (o2_okvis) |
| ADVIO-15 | **0.90 / 0.66** | 0.83 / 0.65 | 1.24 | 93% / 85% (66% of all frames) | n/a (no fixes) | n/a | 1451 | 907.64 | 1.65 | 1.32 (a15_stella) / 0.54 (a15_stella) |
| ADVIO-20 | **11.76 / 12.32** | 4.97 / 5.27 | 0.84 | 100% / 100% (90% of all frames) | 2.89 / 4.80 | 12.00 | 143044 | 32225 | 58.84 | 11.72 (a20_stella) / 12.06 (a20_stella) |

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

### Timing (CPU seconds, shared machine; sv_run = whole stella_vio run incl. ORB extraction at full resolution, single thread)

| sequence | frames | sv_run default CPU s (ms/frame) | sv_run full CPU s (ms/frame) | gait us/IMU sample | gf_run batch s | gf_run causal s (us/frame) |
|---|---|---|---|---|---|---|
| Indoor-1 | 1779 | 72 (40) | 73 (41) | 23.5 | 0.01 | 0.02 (11) |
| Indoor-2 | 1535 | 56 (37) | 57 (37) | 4.7 | 0.01 | 0.02 (11) |
| Outdoor-1 | 5898 | 362 (61) | 363 (62) | 1.4 | 0.04 | 0.24 (41) |
| Outdoor-2 | 6737 | 449 (67) | 457 (68) | 1.4 | 0.04 | 0.31 (46) |
| ADVIO-15 | 1553 | 58 (37) | 56 (36) | 4.7 | 0.01 | 0.02 (11) |
| ADVIO-20 | 9076 | 615 (68) | 619 (68) | 1.4 | 0.05 | 0.22 (24) |

Section 14 (opt-in `georef` rows, the default fusion rows above are unchanged): geo-referencing the fix-free gait stream with one slowly varying similarity gives batch / causal live 5.36 / 4.71 (Outdoor-1), 4.81 / 11.34 (Outdoor-2), 11.76 / 11.79 (ADVIO-20) against GNSS alone 5.73 / 14.66 / 12.00: the causal output now beats GNSS alone on all three (-18 / -23 / -2 %) and batch is not hurt. Smoother-internal remedies (longer window, smaller yaw random walk, trust off, grow) all failed on at least one sequence (`docs/gnss_vio_benchmark_20261001.md` section 14, `docs/rejected_trials.md`).

Short version: ahead of XRSLAM / RD-VIO on Indoor-2, ADVIO-15, Outdoor-2, ADVIO-20 (their scale collapses on 3 of those), level on Outdoor-1, behind on Indoor-1 (full config; the default config is level). Against GNSS alone: batch 2-9 % better on the three outdoor sequences, causal 3-24 % worse (the gait-only stream without fixes is better than GNSS alone in causal mode but not geo-referenced).
