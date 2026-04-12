# SLAM Benchmark Results

Metric: **ATE** (Absolute Trajectory Error) after Umeyama similarity alignment
(monocular has no metric scale, so scale is corrected before computing error).

Ground truth sources currently tracked by the benchmark suite:
`test_freiburgxyz525.npz`, `external/twitchslam/videos/test_freiburgdesk525.npz`,
`external/twitchslam/videos/test_freiburgroom525.npz`, and
`external/twitchslam/videos/test_freiburgrpy525.npz`.

No GT available for kitti984, countryroad, or drone — those use heuristic score only
(points + 0.2×frames + 0.05×avg_inliers).

Canonical benchmark outputs live in `runs/benchmark/`:
`test_countryroad.json`, `test_drone.json`, `test_freiburgxyz525.json`,
`test_kitti984.json`, `summary.json`, `comparison_orbslam2.json`,
`gt_tracking.json`, `gt_tracking.csv`, `gt_tracking.md`, and
`orbslam2_trajectory.txt`.

Runs with non-default `--seconds` use suffixed filenames such as
`test_freiburgxyz525_1s.json` and `summary_1s.json` so they do not overwrite
the canonical 30-second results.

Older Freiburg experiment runs are archived under `runs/archive/benchmark/`.

Frozen Pure C baseline note: the current stable universal `simple_slam_c.c` build
has a verified `test_freiburgxyz525` 5-second ATE RMSE of **0.1461 m**.

## Current GT Tracking (30s)

Latest full GT-backed run was generated with:

```bash
python3 benchmark_native.py --all_gt --force
```

The generated tracking artifacts are:
`runs/benchmark/gt_tracking.json`, `runs/benchmark/gt_tracking.csv`, and
`runs/benchmark/gt_tracking.md`.

This section tracks the full **4 GT datasets × 5 implementations = 20 measured runs**.

Approx implementation size reference:

| Impl | Approx LOC | Counting rule |
|------|------------|---------------|
| `python` | ~618 | `simple_slam.py` |
| `cpp` | ~797 | `simple_slam_opt.cpp` |
| `c` | ~438 core (+274 shim) | `simple_slam_c.c` plus `simple_slam_c_shim.cpp/.h` for the OpenCV bridge |
| `pure_c` | ~438 | standalone `simple_slam_c.c` only |
| `pure_c_brief` | ~363 | promoted historical snapshot in `simple_slam_c_brief.c` |

| Sequence | Best Impl | Best ATE RMSE | Runner-up | Runner-up ATE RMSE | Notes |
|----------|-----------|---------------|-----------|--------------------|-------|
| `test_freiburgdesk525` | `python` | **0.6778 m** | `cpp` | 0.7194 m | Only GT set where Python currently leads |
| `test_freiburgroom525` | `cpp` | **1.5452 m** | `pure_c_brief` | 1.8414 m | BRIEF snapshot is the runner-up here |
| `test_freiburgrpy525` | `cpp` | **0.0977 m** | `pure_c_brief` | 0.0980 m | BRIEF snapshot edges out Python |
| `test_freiburgxyz525` | `cpp` | **0.1729 m** | `pure_c_brief` | 0.1760 m | BRIEF snapshot beats both current pure C and Python |

Full 4×5 ATE matrix for the current 30-second run:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` |
|----------|----------|-------|-----|----------|---------------|
| `test_freiburgdesk525` | **0.6778** | 0.7194 | 0.7563 | 0.7574 | 0.7412 |
| `test_freiburgroom525` | 1.8660 | **1.5452** | 1.8691 | 1.8691 | 1.8414 |
| `test_freiburgrpy525` | 0.0982 | **0.0977** | 0.0997 | 0.0997 | 0.0980 |
| `test_freiburgxyz525` | 0.1785 | **0.1729** | 0.1789 | 0.1782 | 0.1760 |

Cross-dataset tradeoff summary (mean over the 4 GT datasets):

| Impl | Approx LOC | Mean ATE RMSE | Mean Runtime (s) | Mean Speedup vs Python | GT Wins | GT Runner-up |
|------|------------|---------------|------------------|------------------------|---------|--------------|
| `python` | ~618 | 0.7051 | 16.982 | 1.00x | 1 | 0 |
| `cpp` | ~797 | **0.6338** | **8.209** | **2.07x** | **3** | 1 |
| `c` | ~438 core (+274 shim) | 0.7260 | 31.744 | 0.56x | 0 | 0 |
| `pure_c` | ~438 | 0.7261 | 28.235 | 0.83x | 0 | 0 |
| `pure_c_brief` | ~363 | 0.7141 | 39.179 | 0.44x | 0 | **3** |

Promoted historical pure C snapshot comparison:

| Sequence | `pure_c` current (~438 LOC) | `pure_c_brief` (~363 LOC) | Δ current - `brief` |
|----------|-----------------------------|--------------------------|-------------------|
| `test_freiburgdesk525` | 0.7574 | **0.7412** | +0.0162 |
| `test_freiburgroom525` | 1.8691 | **1.8414** | +0.0277 |
| `test_freiburgrpy525` | 0.0997 | **0.0980** | +0.0017 |
| `test_freiburgxyz525` | 0.1782 | **0.1760** | +0.0022 |

`pure_c_brief` is copied into the active repo as `simple_slam_c_brief.c`.
It matches the pure C source from `6f7fda6` and `2b688ed`, which were identical
for this file. Historical helper artifacts remain in `runs/benchmark_history/`.

---

## Results

### test_freiburgxyz525.mp4 — 640×480, 798 frames @ 25 fps, 31.9s

#### simple_slam.py

| Version | ATE RMSE | ATE Median | ATE Max | KF% | Points | Runtime | Notes |
|---------|----------|------------|---------|-----|--------|---------|-------|
| v1.21   | 0.1773 m | 0.1495 m   | 0.3289 m | 100% | 15000 (cap) | ~47s | KF trans threshold bug |
| v1.22   | 0.1778 m | 0.1521 m   | 0.3391 m |  52% | 11779       | ~78s | Trans threshold removed, res cap added |
| v1.23   | 0.1786 m | 0.1536 m   | 0.3337 m |  52% | 12096       | ~29s | BA gap bug fixed; large videos now run at speed |
| v1.24   | 0.1789 m | 0.1546 m   | 0.3312 m |  93% | 2577        | ~20s | Local Windowed BA + Sparsity + Sub-pixel refinement |

Settings: `--seconds 30 --ba_min_gap_sec 0.5` (BA enabled), all other params default.

#### ORB-SLAM2 (reference)

| System | ATE RMSE | ATE Median | ATE Max | Poses | Notes |
|--------|----------|------------|---------|-------|-------|
| ORB-SLAM2 monocular (measured) | 0.0752 m | 0.0643 m | 0.1953 m | 575 | Built from source, no display, per-frame trajectory |
| ORB-SLAM2 monocular (published) | 0.0090 m | — | — | 798 | Mur-Artal 2015, Table 1; loop closure + BA |
| simple_slam v1.24 | 0.1789 m | 0.1546 m | 0.3312 m | 750 | No loop closure, Local Windowed BA |
| geohot/twitchslam (compat mode) | 0.1763 m | 0.1496 m | 0.3259 m | 750 | Headless run with no-op optimizer fallback; Python/OpenCV/NumPy compatibility shim |

> Measured gap: **~2.4×** (575 vs 750 frames, both Umeyama-aligned).
> Published gap: **~20×** — ORB-SLAM2's published result uses loop closure + BA on a clean run.
> Our run had 2 failed initializations before converging; loop closure did not fire.
>
> `geohot/twitchslam` also runs on this sequence after local compatibility fixes,
> but only in a degraded compatibility mode: the original bundled `g2o`/`pangolin`
> binaries do not load on this Python environment, so the benchmark uses a no-op
> optimizer fallback. In that mode it reaches **0.1763 m RMSE** over 750 poses,
> but takes **753.7s** versus **19.9s** for `simple_slam v1.24`.
>
> To run ORB-SLAM2 yourself: `setup_orbslam2.sh` + `run_orbslam_benchmark.py`
> (builds from source; see build notes below).

---

## Dataset Info

| File | Resolution | Frames | FPS | Duration | GT |
|------|-----------|--------|-----|----------|----|
| test_freiburgxyz525.mp4 | 640×480   |  798 | 25  | 31.9s | test_freiburgxyz525.npz |
| test_kitti984.mp4       | 1242×375  |  268 | 30  |  8.9s | — |
| test_countryroad.mp4    | 1920×1080 |  781 | 30  | 26.1s | — |
| test_drone.mp4          | 3840×2160 |  628 | 30  | 21.0s | — |

---

## How to reproduce

```bash
# Full sequence, BA disabled (fast, ~78s)
python simple_slam.py \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 30 --timeout 90 \
  --no_imshow --no_plot \
  --ba_min_gap_sec 9999 \
  --metrics_out runs/benchmark/test_freiburgxyz525.json

# Compute ATE from saved metrics
python benchmark.py --seconds 30 --force

# Refresh ORB-SLAM2 comparison from the saved trajectory
python run_orbslam_benchmark.py --skip_run

# Benchmark `geohot/twitchslam` in local compatibility mode
python benchmark_twitchslam.py \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 30 --timeout 1200 \
  --out_json runs/benchmark_twitchslam/test_freiburgxyz525_30s.json

# Benchmark all GT-backed datasets and refresh the tracking tables
python3 benchmark_native.py --all_gt --force
```

---

## Changelog

| Version | Change | ATE impact |
|---------|--------|------------|
| v1.21 | Baseline | 0.1773 m RMSE |
| v1.22 | Removed `KEYFRAME_MAX_TRANS` (was scale-ambiguous, caused 100% KF rate); added `MAX_PROC_W=640` resolution cap | 0.1778 m RMSE (no change) |
| v1.23 | Fixed `_last_ba_time=0` bug — BA fired on frame 1 regardless of `--ba_min_gap_sec`; large videos (kitti, countryroad, drone) now process at normal speed | 0.1786 m RMSE (no change; accidental BA wasn't helping accuracy) |
| v1.24 | Local Windowed BA (10 frames) with Sparsity Mask + Sub-pixel refinement + Huber loss | 0.1789 m RMSE (plateau, but faster runtime) |
