# Pose Validation & SLAM Shootout

This repo benchmarks several monocular SLAM implementations on local video
datasets, with accuracy measured against TUM-style ground truth when available.
The active goal is a small, readable, library-free C pipeline whose behavior is
easy to inspect and change.

## Implementations

| Impl | Source | Notes |
|------|--------|-------|
| `python` | `simple_slam.py` | OpenCV/SciPy baseline. |
| `cpp` | `simple_slam_opt.cpp` | OpenCV C++ baseline and current best mean ATE. |
| `c` | `simple_slam_c.c` + `simple_slam_c_shim.*` | C core with OpenCV bridge. |
| `pure_c` | `simple_slam_c.c` | Standalone library-free C baseline. |
| `pure_c_brief` | `simple_slam_c_brief.c` | Promoted pure-C snapshot with BRIEF relocalization. |
| `pure_c_orb` | `simple_slam_c_orb.c` | Library-free ORB-style pipeline. |
| `pure_c_plus` | `simple_slam_c_plus.c` | Pure-C local BA / loop-closure variant; current best pure-C by mean ATE and active architectural focus. |

## Quick Start

```bash
# Fast 5-second regression check across GT-backed sequences
python3 check_regressions.py

# Canonical full GT benchmark
python3 benchmark_native.py --all_gt --force

# Single GT dataset while diagnosing
python3 benchmark_native.py --all_gt --video test_freiburgxyz525 --force

# Native build
cmake -S . -B build-native
cmake --build build-native -j
```

Use `--workers 4` with full benchmark invocations when you want parallel runs.
The generated benchmark artifacts live under `runs/benchmark/`.

## Ground Truth Datasets

Benchmark discovery includes top-level `test_*.mp4` files with adjacent `.npz`
ground truth and `external/twitchslam/videos/test_*.mp4` with adjacent `.npz`.

Currently tracked GT sequences:

| Sequence | Notes |
|----------|-------|
| `test_freiburgxyz525` | Linear motion, top-level dataset. |
| `test_freiburgrpy525` | Rotation-heavy sequence. |
| `test_freiburgroom525` | Wide room sequence and current hard case. |
| `test_freiburgdesk525` | Fast close-up desk motion. |

`test_kitti984`, `test_countryroad`, and `test_drone` do not currently have GT
in this repo and are scored only by heuristic metrics.

## Current Benchmark State

Source of truth for generated numbers:

- `runs/benchmark/gt_tracking.csv`
- `runs/benchmark/gt_tracking.md`
- `runs/benchmark/summary_all.json`

Latest canonical 30-second GT sweep:

| Sequence | Best Impl | Best ATE RMSE | Runner-up | Runner-up ATE RMSE |
|----------|-----------|---------------|-----------|--------------------|
| `test_freiburgdesk525` | `python` | **0.6784 m** | `cpp` | 0.7194 m |
| `test_freiburgroom525` | `cpp` | **1.5452 m** | `pure_c_plus` | 1.7591 m |
| `test_freiburgrpy525` | `cpp` | **0.0977 m** | `python` | 0.0984 m |
| `test_freiburgxyz525` | `cpp` | **0.1729 m** | `pure_c_plus` | 0.1768 m |

Full 4x7 ATE matrix:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | **0.6784** | 0.7194 | 0.7569 | 0.7569 | 0.7198 | 0.7567 | 0.7307 |
| `test_freiburgroom525` | 1.8654 | **1.5452** | 1.8689 | 1.8689 | 1.8518 | 1.8677 | 1.7591 |
| `test_freiburgrpy525` | 0.0984 | **0.0977** | 0.0998 | 0.0998 | 0.0992 | 0.0998 | 0.0990 |
| `test_freiburgxyz525` | 0.1785 | **0.1729** | 0.1777 | 0.1777 | 0.1782 | 0.1788 | 0.1768 |

Mean ATE over the four GT datasets:

| Impl | Mean ATE RMSE | GT Wins | Runner-up |
|------|---------------|---------|-----------|
| `cpp` | **0.6338** | **3** | 1 |
| `pure_c_plus` | 0.6914 | 0 | 2 |
| `python` | 0.7052 | 1 | 1 |
| `pure_c_brief` | 0.7123 | 0 | 0 |
| `pure_c_orb` | 0.7258 | 0 | 0 |
| `c` | 0.7258 | 0 | 0 |
| `pure_c` | 0.7258 | 0 | 0 |

## Benchmark Discipline

Any change to SLAM algorithm code or benchmark plumbing must be validated with:

```bash
python3 benchmark_native.py --all_gt --force
```

Single-dataset runs are for diagnosis only. Do not claim an improvement from a
single-GT run, a short suffixed run, or an instrumented build. If benchmark
behavior changes, regenerate the saved outputs before documenting the result.

ATE RMSE is the primary metric. Map density, keyframe count, runtime, and LOC
are diagnostics. Do not promote ATE-neutral changes across all GT sequences
just because they improve a secondary metric.

## Diagnostics

Plot per-frame error:

```bash
python3 tools/plot_frame_errors.py \
  --gt test_freiburgxyz525.npz \
  --output runs/plots/diag.svg \
  python=runs/benchmark/test_freiburgxyz525_5s.json \
  pure_c_plus=runs/benchmark/test_freiburgxyz525_pure_c_plus_5s.json
```

Inspect a saved run against GT:

```bash
python3 tools/diagnose_trace.py \
  runs/benchmark/test_freiburgxyz525_pure_c_plus_5s.json \
  test_freiburgxyz525.npz \
  --top_k 20 \
  --spikes \
  --by_method
```

`pure_c_plus` traces include `method` per frame: `0` init, `1` essential,
`2` PnP, `3` predicted fallback. They also include tracked feature count,
linked map-point count, BRIEF relinks, and points added, so room spikes can be
separated into pose-estimation, link-loss, and map-growth failures.
Use `--window START:END` to inspect a specific frame range; `pure_c_plus`
also reports PnP candidate inliers, predicted-LM inliers, E inliers,
pre/post-relink counts, and raw inter-frame translation jump.

For PnP solver diagnosis only, dump the exact `pure_c_plus` 2D/3D
correspondences and replay them through OpenCV:

```bash
python3 benchmark.py --all_gt --video test_freiburgroom525 --impl pure_c_plus --seconds 8 --force \
  --out_dir runs/pure_c_iter/pnp_dump --extra_args "--pnp_dump runs/pure_c_iter/pnp_dump/room_8s.pnp"
cmake -S . -B build-native
cmake --build build-native --target pnp_replay_opencv
build-native/pnp_replay_opencv --dump runs/pure_c_iter/pnp_dump/room_8s.pnp --window 120:170
build-native/pnp_replay_opencv --dump runs/pure_c_iter/pnp_dump/room_8s.pnp --window 120:170 --summary
python3 tools/analyze_pnp_replacements.py \
  --dump runs/pure_c_iter/pnp_dump/room_8s.pnp \
  --metrics runs/pure_c_iter/pnp_dump/test_freiburgroom525_pure_c_plus_8s.json \
  --gt external/twitchslam/videos/test_freiburgroom525.npz \
  --window 120:170
python3 tools/analyze_pnp_replacements.py \
  --dump runs/pure_c_iter/pnp_dump/room_8s.pnp \
  --metrics runs/pure_c_iter/pnp_dump/test_freiburgroom525_pure_c_plus_8s.json \
  --gt external/twitchslam/videos/test_freiburgroom525.npz \
  --sweep_rules
```

This is an offline diagnostic harness. It reports strict 2px/3px/5px inliers,
median reprojection error, positive-depth ratio, translation jump, and a
conservative `portable` flag. Do not make `pure_c_plus` depend on OpenCV at
runtime. On the current room 8s dump, frames 120:170 show AP3P beating DLT at
2px inliers on 28/51 frames, but only 17/51 pass the portable rule and only
3/51 are portable when pure DLT failed; this argues for selective scored
replacement, not wholesale AP3P substitution. The replacement analyzer labels
candidate frames by actual post-alignment ATE delta. On the same window,
26/51 single-frame AP3P replacements improve RMSE, but the biggest winners
include high-jump/high-median-error candidates while similar-looking frames are
catastrophic losers. Do not use a simple jump cap or local reprojection proxy as
the final scorer. The first offline rule-sweep winner on the 8s dump is:
replace only pure-PnP-failed frames where AP3P has at least +5 strict 2px
inliers over pure, median reprojection error is <= 60 px, and translation jump
is <= 500k. That rule replaces frames `159,161,166,168,198,199` and improves
offline 8s RMSE from 0.545953 to 0.525753. Treat this as a candidate to test
with a real pure-C AP3P implementation, not as a proven in-pipeline result.

Use `benchmark.ate_rmse` for ad-hoc ATE checks. A prior standalone Umeyama
implementation produced a false room improvement, so keep all ATE calculations
on the benchmark utility path.

## Active Pure-C Blocker

`pure_c_plus` is the active architectural focus and current best pure-C variant
by mean ATE. The major blocker is
`test_freiburgroom525`: the trajectory diverges before loop closure can help.
Diagnostics showed `|cur_t|` reaching roughly 16 m by frame 17 and tens of
kilometers before the first loop candidate.

Current conclusion:

- The room failure is rooted in PnP / `refine_pose_lm`, not loop closure.
- Naive LM clamps, catastrophic-step gates, without-replacement RANSAC sampling,
  and overdetermined DLT refits all regressed the canonical GT sweep.
- The current numbers appear to be a fragile fixed point where several
  sub-pathologies cancel each other.

Do not retry these rejected surgical fixes without a materially new hypothesis:

| Trial | Result |
|-------|--------|
| Trust-region LM in `refine_pose_lm` | Regressed all sequences; reduction-ratio retry collapsed map density and moved room to 1.8676. |
| Catastrophic-step gate in `refine_pose_lm` | Regressed all sequences; room +0.064 m. |
| PnP without-replacement 6-point samples | Regressed all sequences; room +0.033 m. |
| RANSAC inlier-set overdetermined DLT refit | Regressed all sequences; room +0.092 m. |
| E-fallback translation rescaling | Looked good in truncated runs; full sweep regressed desk. |
| Thumbnail-SAD loop threshold tweaks | Loop fires after trajectory is already divergent; no real ATE win. |
| IC-angle random BRIEF / projected BRIEF / prev-frame BRIEF | Rejected due to desk or room regressions. |
| PnP-failure BRIEF relink retry | ATE-neutral, room worsened slightly, and rpy timed out at 713/723 frames. |
| Candidate pose scoring over PnP/E/predicted | Fast 5s looked good, but full sweep collapsed map density and regressed room to 1.8653. |
| Predicted fallback when PnP fails with many links | Collapsed map density; room moved to 1.8631 and desk to 0.7537. |
| Reprojection gate on new E triangulations | Starved early room map growth even at loose thresholds; abandoned before full sweep. |
| Late E-frame new-point caps | Cap 40 moved room to 1.8190 with 16824 points; cap 80 moved room to 1.7833 with 22390 points. |
| Predicted-pose LM as gated PnP fallback | Gate 40 preserved early growth but moved full room to 1.7774 with 24599 points; gate 80 was a no-op. |
| PnP using only points with `obs >= 2` | Shifted room into E earlier; 8s room ATE worsened to 0.6005 despite higher point count. |
| Row-normalized PnP DLT equations | Changed map growth shape but worsened 8s room ATE to 0.6424. |
| BRIEF stale-link pruning before PnP | Thresholds 96 and 128 worsened 8s room ATE to 0.5855 and 0.5942; threshold 160 improved 8s to 0.5349 but regressed full room to 1.8035. |
| 3px PnP RANSAC inlier radius | Looser candidate scoring starved early map growth and worsened 8s room ATE to 0.6513 with 1756 points. |
| Linear EPnP-style sampled candidate | Improved 8s room to 0.5295 but regressed full room to 1.8168; requiring +10 inlier advantage worsened 8s to 0.6486. |
| Numeric P3P depth-solver candidate | Direct competition worsened 8s room to 0.5582; fallback-only use worsened 8s to 0.6522 and starved map growth. |
| PnP post-LM stability gate with E fallback | Exploded map growth to 8687 points but worsened 8s room ATE to 0.6420; more points/inliers were not an accuracy win. |

Next useful work should still start on the PnP side, but do not retry another
quick numeric P3P approximation. The next candidate generator should be a
well-formed P3P/AP3P implementation or should first replay saved dumps through
candidate scoring offline. Do not start by wrapping `refine_pose_lm` with
another clamp, step gate, or reduction-ratio trust region; those variants
repeatedly preserve short-run behavior while collapsing full-sweep map density.
The failed EPnP/P3P trials show that higher short-run inlier count is not
sufficient; candidate scoring needs pose stability and depth/cheirality sanity
too. If PnP candidate quality still fails, investigate initialization/keyframe
selection before retrying loop closure.

## Design Direction

Phase 1 is accuracy and robustness. `pure_c_plus` should close the mean ATE gap
to `cpp`; LOC is only a tiebreaker during this phase.

Phase 2 starts once `pure_c_plus` is within about `0.02 m` mean ATE of `cpp`.
At that point, reduce and simplify the implementation while preserving the
validated GT numbers.

Code style is deliberately "math on paper": matrices laid out as grids, one
equation per line, sparse comments where they clarify non-obvious numerical
steps. Avoid dataset-specific magic constants and hidden frame skipping.

## Native and Pure-C Build Notes

Native OpenCV-backed binaries:

```bash
cmake -S . -B build-native
cmake --build build-native -j

./build-native/simple_slam_opt \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 1 \
  --timeout 20 \
  --metrics_out runs/_smoke_simple_slam_opt.json
```

Standalone pure C:

```bash
gcc -O3 -march=native -fopenmp simple_slam_c.c -o simple_slam_pure_c -lm

./simple_slam_pure_c \
  --video_path test_freiburgxyz525.mp4 \
  --seconds 30 \
  --timeout 120 \
  --metrics_out runs/pure_c_metrics.json
```

`simple_slam_c.c` is shared by the OpenCV-shimmed `c` implementation and the
standalone `pure_c` binary. Do not assume those benchmark entries are identical:
one uses the shim path, the other is fully standalone.

## Published Reference Context

Published monocular ORB-SLAM numbers are available for `fr1/xyz` and
`fr1/desk`, but not for `fr1/room` or `fr1/rpy`.

| Sequence | Published mono reference | This repo current best |
|----------|--------------------------|------------------------|
| `fr1/xyz` | ORB-SLAM 2015: 0.009 m | `cpp`: 0.1729 m |
| `fr1/desk` | ORB-SLAM 2015: 0.017 m | `python`: 0.6784 m |
| `fr1/room` | No credible mono reference found | `cpp`: 1.5452 m |
| `fr1/rpy` | No credible mono reference found | `cpp`: 0.0977 m |

RGB-D ORB-SLAM2 and RTAB-Map results are useful lower bounds, but they are not
apples-to-apples comparisons because they use depth.

Reference links:

- ORB-SLAM: https://ar5iv.labs.arxiv.org/html/1502.00956
- ORB-SLAM2: https://ar5iv.labs.arxiv.org/html/1610.06475
- ORB-SLAM3: https://ar5iv.labs.arxiv.org/html/2007.11898
- RTAB-Map 2024 update: https://arxiv.org/html/2403.06341v1

## Repository Hygiene

This README is the canonical human-facing project guide. Generated benchmark
tables remain under `runs/benchmark/`. Keep one-off experiments in suffixed
files or dedicated folders such as `runs/pure_c_iter/`,
`runs/benchmark_history/`, or `runs/archive/`.
