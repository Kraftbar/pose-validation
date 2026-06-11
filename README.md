# Pose Validation & SLAM Shootout

This repo benchmarks several monocular SLAM implementations on local video
datasets, with accuracy measured against TUM-style ground truth when available.
The active goal is a small, readable, library-free C pipeline whose behavior is
easy to inspect and change.

## Implementations

| Impl | Source | Notes |
|------|--------|-------|
| `python` | `simple_slam.py` | OpenCV/SciPy baseline. |
| `cpp` | `simple_slam_opt.cpp` | OpenCV C++ baseline; best on `fr1/xyz` and main raw-state reference. |
| `c` | `simple_slam_c.c` + `simple_slam_c_shim.*` | C core with OpenCV bridge. |
| `pure_c` | `simple_slam_c.c` | Standalone library-free C baseline. |
| `pure_c_brief` | `simple_slam_c_brief.c` | Promoted pure-C snapshot with BRIEF relocalization. |
| `pure_c_orb` | `simple_slam_c_orb.c` | Library-free ORB-style pipeline. |
| `pure_c_plus` | `simple_slam_c_plus.c` | Pure-C local BA / loop-closure variant; current best reported mean ATE and active architectural focus. |

## Quick Start

```bash
# Fast 5-second regression check across GT-backed sequences
python3 check_regressions.py

# Canonical full GT benchmark
python3 benchmark_native.py --all_gt --force

# Experimental parallel full GT benchmark
python3 benchmark_native.py --all_gt --force --workers 4

# Single GT dataset while diagnosing
python3 benchmark_native.py --all_gt --video test_freiburgxyz525 --force

# Native build
cmake -S . -B build-native
cmake --build build-native -j
```

Use `--workers 4` only for exploratory wall-clock checks unless you intend to
promote the parallel sweep as the new baseline. The timeout bump can let
`pure_c_plus` process more frames than the serial 120-second canonical command.
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
| `test_freiburgdesk525` | `pure_c_plus` | **0.5716 m** | `python` | 0.6870 m |
| `test_freiburgroom525` | `pure_c_plus` | **1.1799 m** | `cpp` | 1.5452 m |
| `test_freiburgrpy525` | `pure_c_plus` | **0.0920 m** | `cpp` | 0.0977 m |
| `test_freiburgxyz525` | `cpp` | **0.1729 m** | `pure_c_plus` | 0.1763 m |

Full 4x7 ATE matrix:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | 0.6870 | 0.7194 | 0.7569 | 0.7569 | 0.7198 | 0.7577 | **0.5716** |
| `test_freiburgroom525` | 1.8659 | 1.5452 | 1.8689 | 1.8689 | 1.8518 | 1.8682 | **1.1799** |
| `test_freiburgrpy525` | 0.0982 | 0.0977 | 0.0998 | 0.0998 | 0.0992 | 0.0998 | **0.0920** |
| `test_freiburgxyz525` | 0.1789 | **0.1729** | 0.1777 | 0.1777 | 0.1782 | 0.1790 | 0.1763 |

Runtime matrix from the same sweep, in seconds:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | 16.186 | 7.158 | 31.732 | 32.998 | 41.830 | 31.046 | 28.565 |
| `test_freiburgroom525` | 15.817 | 7.664 | 32.534 | 27.003 | 45.591 | 36.195 | 34.443 |
| `test_freiburgrpy525` | 17.895 | 8.646 | 25.527 | 16.560 | 57.018 | 37.569 | 34.583 |
| `test_freiburgxyz525` | 20.286 | 9.060 | 52.415 | 64.216 | 36.901 | 40.237 | 36.284 |

Mean ATE over the four GT datasets:

| Impl | Mean ATE RMSE | GT Wins | Runner-up |
|------|---------------|---------|-----------|
| `pure_c_plus` | **0.5050** | **3** | 1 |
| `cpp` | 0.6338 | 1 | 2 |
| `python` | 0.7075 | 0 | 1 |
| `pure_c_brief` | 0.7123 | 0 | 0 |
| `c` | 0.7258 | 0 | 0 |
| `pure_c` | 0.7258 | 0 | 0 |
| `pure_c_orb` | 0.7262 | 0 | 0 |

## Benchmark Discipline

**Hard rule:** any change to SLAM algorithm code or benchmark plumbing must be
validated with:

```bash
python3 benchmark_native.py --all_gt --force
```

This applies to `simple_slam_c.c`, `pure_c_math.h`,
`simple_slam_c_brief.c`, `simple_slam_c_orb.c`, `simple_slam_c_plus.c`,
`simple_slam_opt.cpp`, `simple_slam.py`, `simple_slam_c_shim.cpp/.h`, and
benchmark plumbing.

Single-dataset runs are for diagnosis only. Do not claim an improvement from
one dataset, short suffixed runs, truncated timeouts, or instrumented builds.
Do not describe benchmark improvements in docs unless generated outputs from a
full clean `--all_gt` run have been refreshed.

ATE RMSE is the primary metric. Map density, keyframe count, runtime, and LOC
are diagnostics. If a candidate is ATE-neutral across all GT sequences, with
all deltas within about `0.01 m`, do not promote it just because it improves a
secondary metric. Watch for silent regressions: neutral ATE with a large
map-density drop elsewhere usually means the change traded one failure mode for
another. Record rejected trials with numbers in this README.

Parallel mode is available, but treat it as exploratory unless you intend to
promote the parallel sweep as the new baseline:

```bash
python3 benchmark_native.py --all_gt --force --workers 4
```

Each worker gets `nproc / workers` OpenMP threads and the per-run timeout is
auto-bumped. The longer timeout can change `pure_c_plus` frame counts and ATE
relative to the serial 120-second command, so use the serial command above for
canonical artifacts.
Deterministic impls (`cpp`, `c`, `pure_c`, `pure_c_brief`, `pure_c_plus`)
reproduce canonical ATE within +/- `0.0002`; `python` and `pure_c_orb` retain
their natural run-to-run variance.

Use `benchmark.ate_rmse` for ad-hoc ATE checks. A prior standalone Umeyama
implementation produced a false room improvement, so keep all ATE calculations
on the benchmark utility path.

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
python3 tools/compare_traces.py \
  --gt external/twitchslam/videos/test_freiburgroom525.npz \
  --window 120:180 \
  --run plus=runs/benchmark/test_freiburgroom525_pure_c_plus.json \
  --run cpp=runs/benchmark/test_freiburgroom525_cpp.json \
  --sort_by_delta
python3 tools/analyze_trace_crossover.py \
  --gt external/twitchslam/videos/test_freiburgroom525.npz \
  --a plus=runs/benchmark/test_freiburgroom525_pure_c_plus.json \
  --b cpp=runs/benchmark/test_freiburgroom525_cpp.json \
  --window 30 --persist 12 --margin 0.05
python3 tools/analyze_trace_state_windows.py --window 30
python3 tools/sweep_trace_health_rules.py
python3 tools/analyze_pnp_dump_quality.py \
  runs/pure_c_iter/pnp_dump/test_freiburgroom525_pure_c_plus_8s.json \
  external/twitchslam/videos/test_freiburgroom525.npz \
  runs/pure_c_iter/pnp_dump/room_8s.pnp \
  --top_k 12
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
with a real pure-C AP3P implementation, not as a proven in-pipeline result. On
the full 30s room dump, the best swept offline rule only improves RMSE from
1.759123 to 1.757817 (+0.001306), which is below the practical promotion
threshold; do not port this rule directly without a stronger full-room signal.
Use `tools/analyze_pnp_dump_quality.py` for cheap local-signal checks before
turning a PnP quality idea into code. Use `tools/compare_traces.py` for
raw cross-implementation diffs and `tools/analyze_trace_crossover.py` for
rolling plus-vs-C++ deltas. Current room diffs show `pure_c_plus` is better than
`cpp` in the early 51:80 window, then becomes persistently worse at frame 130.
The worst 30-frame room gap is 537:566; desk's worst 30-frame gap is later and
smaller, while rpy/xyz deltas are tiny. Use
`tools/analyze_trace_state_windows.py` for rolling multi-frame health windows
and `tools/sweep_trace_health_rules.py` to test simple live predicates over the
canonical plus traces before turning a trace signal into code. The next
comparison should inspect room-specific keyframe-map growth plus jump dynamics
instead of trying more global ORB threshold or scalar keyframe tweaks blindly.

For per-candidate `pure_c_plus` map-admission diagnosis:

```bash
python3 benchmark.py --all_gt --video test_freiburgroom525 --impl pure_c_plus --seconds 30 --force \
  --out_dir runs/pure_c_iter/admission_detail_room \
  --extra_args "--map_admission_detail_dump runs/pure_c_iter/admission_detail_room/detail.csv"
python3 tools/analyze_admission_detail.py \
  runs/pure_c_iter/admission_detail_room/test_freiburgroom525_pure_c_plus.json \
  external/twitchslam/videos/test_freiburgroom525.npz \
  runs/pure_c_iter/admission_detail_room/detail.csv \
  --top_k 12
```

The C++ baseline accepts the same diagnostic dump flag:

```bash
python3 benchmark.py --all_gt --video test_freiburgroom525 --impl cpp --seconds 30 --force \
  --out_dir runs/pure_c_iter/cpp_admission_detail_room \
  --extra_args "--map_admission_detail_dump runs/pure_c_iter/cpp_admission_detail_room/detail.csv"
python3 tools/analyze_admission_detail.py \
  runs/pure_c_iter/cpp_admission_detail_room/test_freiburgroom525_cpp.json \
  external/twitchslam/videos/test_freiburgroom525.npz \
  runs/pure_c_iter/cpp_admission_detail_room/detail.csv \
  --window 556:585 --top_k 8
```

`--map_admission_detail_dump` writes one row per accepted or failed
map-admission candidate with source, decision, pose method/inliers, match
indices, grid cell, baseline, reprojection, parallax, depth, LK forward-backward
error, track displacement, and ranking score. C++ rows use source `cpp_desc`,
descriptor match distance as score, and `fb_err=0`; `pure_c_plus` rows include
LK candidate quality. It complements `--map_admission_dump`, which is only a
per-keyframe summary. Use
`tools/analyze_admission_detail.py` to join those rows to GT-aligned per-frame
error and print method/source summaries, depth probes, and the worst admitted
frames. Add `--window START:END` to inspect a suspected rolling-state range
without re-running the benchmark.

For final map-point birth/lifetime diagnosis:

```bash
python3 benchmark.py --all_gt --video test_freiburgroom525 --impl pure_c_plus --seconds 30 --force \
  --out_dir runs/pure_c_iter/lifecycle_room \
  --extra_args "--map_lifecycle_dump runs/pure_c_iter/lifecycle_room/lifecycle.csv"
python3 tools/analyze_map_lifecycle.py \
  runs/pure_c_iter/lifecycle_room/test_freiburgroom525_pure_c_plus.json \
  external/twitchslam/videos/test_freiburgroom525.npz \
  runs/pure_c_iter/lifecycle_room/lifecycle.csv \
  --top_k 12
python3 tools/sweep_lifecycle_batch_rules.py \
  runs/pure_c_iter/lifecycle_room/test_freiburgroom525_pure_c_plus.json \
  external/twitchslam/videos/test_freiburgroom525.npz \
  runs/pure_c_iter/lifecycle_room/lifecycle.csv \
  runs/pure_c_iter/lifecycle_desk/test_freiburgdesk525_pure_c_plus.json \
  external/twitchslam/videos/test_freiburgdesk525.npz \
  runs/pure_c_iter/lifecycle_desk/lifecycle.csv \
  --live_only
```

`--map_lifecycle_dump` writes one row per map point with birth source/method,
birth reprojection/parallax/depth/LK stats, final observation counters, and
last-seen/span/staleness fields. It is useful for checking whether a proposed
admission or culling rule would target points that actually stop being tracked.
`tools/sweep_lifecycle_batch_rules.py` compares room and desk birth-frame
batches and can restrict itself to live-available signals with `--live_only`.

### C++ vs `pure_c_plus` Correspondence Diff (2026-04-30)

`simple_slam_opt` (the reference C++ pipeline) now also accepts `--pnp_dump`
and `--map_admission_detail_dump`, plus `--no_downscale` and
`--fx/--fy/--cx/--cy` so its dumps can be produced at the same intrinsics as
`pure_c_plus`. These flags are diagnostic-only; default behavior is unchanged,
so the benchmark hard rule does not require a fresh `--all_gt` sweep for this
addition.

To answer "does C++ win at PnP, at triangulation, or at matching?":

```bash
runs/pnp_diff/run.sh                          # default: test_freiburgxyz525.mp4 5s
runs/pnp_diff/run.sh test_freiburgroom525.mp4 8 120:170
```

The driver runs both binaries with `--pnp_dump`, then runs:

- `tools/diff_pnp_dumps.py` — per-frame correspondence overlap, PnP outcomes,
  shared 3D-point distance after Sim3 alignment.
- `tools/pnp_replay_pure_c.c` — verbatim port of `pure_c_plus`'s DLT-RANSAC PnP
  that runs on any dump (matches `tools/pnp_replay_opencv.cpp`).
- `tools/pnp_replay_opencv.cpp` — accepts either dump (formats are identical).

The four-cell matrix (each solver on each pipeline's correspondences) isolates
solver vs. matching/triangulation as the cause of any divergence.

Initial finding on `test_freiburgxyz525` window 10:120:

| | `pure_c_plus` | C++ |
|---|---|---|
| PnP correspondences/frame (median) | 67 | 0 |
| PnP succeeds | 108/111 | 11/111 |
| PnP RMSE (median, when ok) | 127 px | 2.9 px |

OpenCV PnP on `pure_c_plus`'s correspondences: 111/111 ok — the solver is not
the bottleneck. C++'s `uv_to_point` map only catches exact-pixel re-detections,
so `goodFeaturesToTrack`-style detections rarely build a usable PnP set; C++
falls back to essential-matrix pose almost every frame, which keeps drift low.
`pure_c_plus` instead leans on PnP heavily but with bad map points (median
reprojection RMSE > 100 px), which is consistent with the trajectory diverging
before loop closure can fire. The next experiment should be per-point
reprojection-error filtering before PnP, not another candidate-generator
variant. (Recorded under `Active Pure-C Blocker` for future sessions.)

## Active Pure-C Blocker

`pure_c_plus` is the active architectural focus and current best pure-C variant
by reported mean ATE. The promoted default now reports a causal smoothed output
center (base `output_smooth_alpha=0.040`, outlier
`output_smooth_outlier_alpha=0.003`, residual gate
`output_smooth_residual_k=3.50`, window `output_smooth_window=48`) with an
unstable-window alpha override (`output_smooth_unstable_window=48`,
`output_smooth_unstable_points_added=1200`,
`output_smooth_unstable_jump=250000`,
`output_smooth_unstable_jump_count=5`,
`output_smooth_unstable_alpha=0.15`,
`output_smooth_unstable_residual_k=3.00`,
`output_smooth_unstable_cap_residual=0`) and a low-linked-support lower-alpha
gate (`output_smooth_low_link_threshold=70`,
`output_smooth_low_link_window=20`, `output_smooth_low_link_count=4`,
`output_smooth_low_link_alpha=0.007`,
`output_smooth_low_link_include_current=1`) while preserving the unsmoothed
center as `raw_xyz` in each timeline row. It also promotes a mature-map low-PnP
E handoff (`pnp_low_e_fallback=1`, `pnp_low_e_min_jump=500000`,
`pnp_low_e_min_map_points=4000`, `pnp_low_e_min_pose_jump=250000`) that replaces
a weak, large-jump PnP pose only when the E pose has enough extra inlier support.
The room raw pose stream still shows the old jump/divergence failure mode, but
the exported trajectory is substantially more stable under the benchmark's GT
alignment.

Current conclusion:

- The best full-GT result is now a conditionally output-stabilized trajectory
  plus a mature-map weak-PnP E handoff, not a fixed internal map/pose state.
- The room raw-state failure is rooted in PnP / `refine_pose_lm`, not loop
  closure.
- Naive LM clamps, catastrophic-step gates, without-replacement RANSAC sampling,
  and overdetermined DLT refits all regressed the canonical GT sweep.
- Future raw-pose improvements should compare both `xyz` and `raw_xyz`, because
  smoothing can hide but not remove map-state pathologies.

Do not retry these rejected surgical fixes without a materially new hypothesis:

Note (2026-06-11): the rejected opt-in flags and their code paths were removed
from `pure_c_plus` in an experiment-residue cleanup (see Design Direction).
The trial records below stay as the institutional memory; the implementations
are recoverable from git history at commit `8df5dc7` and earlier.

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
| `pure_c_plus` feature-budget bump | Raising corner cap/replenish to 3000/1800 worsened 8s room to 0.6363 with 12265 points; 1500/900 worsened to 0.6520. |
| Extra pose LM iterations (`--pose_lm_iters 15`) | Strong 5s ATE wins on tuned desk/room windows (desk 0.2378, room 0.1159), but not a full-run fix: 30s desk stayed at 0.7538 and 30s room regressed to 1.8670. Kept as an opt-in diagnostic knob, not a default/profile. |
| Essential RANSAC budget sweep (`--essential_iters 250/750`) | Did not beat the 500-iteration default around the `pose_lm_iters 15` short-run wins; lower budget collapsed desk/room and higher budget was still worse than default. |
| Warmup-then-bounded keyframes (`--kf_warmup_frames 60 --kf_period 30 --kf_max_rot_deg 180`) | Reduced 5s keyframes roughly in half after warmup, but starved map growth and worsened desk/room 5s ATE. Kept as a cadence diagnostic only. |
| First-keyframe observation retention (`--first_kf_observations`) | Stores the previous frame as the first real KF and initializes new two-view points with `obs=2`; desk 5s was unchanged and room 5s worsened in both BA/no-BA tuned paths. Not a default. |
| Unique keyframe observation counting (`--unique_kf_observations`) | Guards `map.obs++` so one keyframe can only increment a map point once. Default desk/room 5s was unchanged, and fast-320/no-BA room stayed at 0.1694. Useful diagnostic, not an ATE lever so far. |
| Essential cheirality over all matches (`--essential_cheirality_max 0`) | Evaluates all matches instead of the first 32 for E pose disambiguation. Desk 5s moved only 0.4039 -> 0.4005, while room worsened 0.1578 -> 0.1666. Default remains 32. |
| Predicted-pose PnP reprojection gate (`--pnp_pred_reproj_gate 16`) | Improved fast-room 5s to 0.1430, but full room 30s regressed to 1.8613 and map growth exploded to 95144 points (60577 with max600). Another short-window trap. |
| Recent-keyframe PnP quality mask (`--pnp_quality_gate_px 8/16`) | Non-destructive mask over recent KF reprojection evidence before PnP. Fast-room 5s stayed around 0.170 and desk 5s worsened to 0.4026. A full-room 30s diagnostic at 16 px also worsened the old default path to 1.8419, so the simple “all recent obs bad” rule did not pick the right correspondences. |
| Current PnP dump quality signals | `tools/analyze_pnp_dump_quality.py` joins `--pnp_dump` rows with GT-aligned frame errors. On the current `pnp_min_obs=2` promoted profile, room 30s reproduced 1.6234 with 178 PnP frames and 19 PnP frames above 2.5 m error. `jump>500k` flags 22 PnP frames and catches 11 of those bad room frames, but the same rule flags 24 desk PnP frames with no desk frame above 2.5 m. `pnp_rmse>100 & inliers<=20` catches 8 bad room frames while flagging 76 room frames and 80 desk frames. Local PnP RMSE / LM RMSE / jump are diagnostic signals, not promotion-safe gates by themselves. Runs: `runs/pure_c_iter/current_pnp_minobs2_dump_room/`, `runs/pure_c_iter/current_pnp_minobs2_dump_desk/`. |
| Replay-derived numeric P3P scorer (`--pnp_p3p_max_mederr 120`) | Offline replay on the saved room 8s dump predicted a 0.0409 RMSE win by replacing pure-failed frames 166-170, and full-room replay predicted only a 0.018 RMSE win. Live feedback did not transfer: default room 8s worsened to 0.5862 unrestricted and 0.6141 with jump caps 5000/20000; fast room 8s worsened to 0.6516. |
| Delayed landmark promotion (`--new_point_obs 0`, old-default `--pnp_min_obs 1/2`) | Quarantining fresh triangulations from PnP did not help the old default room 8s: 0.6383 (`pnp_min_obs=1`) and 0.6232 (`pnp_min_obs=2`); fast 320/no-BA worsened to 0.6491. A full-room 30s `--pnp_min_obs 2` diagnostic also worsened the old default to 1.7851 with 31505 points and fewer PnP successes. In the later anchor/pyramid profile, `pnp_min_obs=2` became useful and is now promoted separately. |
| Observation residual stats (`--obs_stat_gate_px 8 --obs_stat_min_good 1/2`) | Maintains per-point good/bad reprojection counts from keyframe observations and filters PnP by those counts. Room 8s worsened to 0.6347 (`min_good=1`) and 0.6214 (`min_good=2`); fast room 8s worsened to 0.6522. Residuals scored against the current fragile poses are not reliable enough. |
| PnP warmup gate (`--pnp_start_frame 50/100`) | Delaying DLT-PnP and relying on E/predicted pose first did not help room 8s: start50 worsened to 0.5849, start100 to 0.6006, and fast 320/no-GBA start50 to 0.6200. Global PnP-vs-E timing is not enough; bad geometry still enters the map. |
| Triangulation source-quality gates (`--tri_min_parallax_deg`, `--tri_max_reproj_px`) | Gating new landmarks at map entry was deeper than the PnP masks but still regressed room 8s: parallax/reproj settings landed around 0.631-0.646, and reproj-only 16 px landed at 0.6432 vs the same-binary baseline 0.6014. The simple gate mostly starves/reshapes the map. |
| Older-keyframe triangulation source (`--tri_source_kf_gap`) | Tracking an older KF into the current frame produced very low early room ATE (~0.083) but only completed 56/200 frames under a 90s 8s-room budget and left only 66 points. This confirms larger-baseline geometry may be meaningful, but this implementation shape is too slow and too sparse to promote. |
| Tentative track promotion (`--candidate_tracks`) | Adds a real unpromoted candidate layer carried by LK (`cand_idx`) and promotes only after age/observation plus parallax/reprojection checks. It completed room 8s but regressed: default promotion 0.6456 with 160 points, obs4/age8 0.6499 with 117 points, loose age2/reproj16 0.6349 with 382 points, and very loose 0.6447 with 1609 points. Separating candidate tracks is necessary architecture, but this simple adjacent-track promotion is still map-starving/geometry-weak. |
| Candidate best-baseline promotion (`--candidate_tracks` with observation history) | Stores up to 8 candidate observations and triangulates from the widest-baseline pair instead of always anchor-current. Very loose room 8s improved the first candidate layer (0.6259 vs 0.6447), but still regressed vs baseline; no-gate promotion was 0.6283 and age1/no-gate was 0.6501. Best-baseline pairing helps a little, but the current LK candidate population is still not selecting tracks that produce a healthy PnP map. |
| Ranked/grid-balanced candidate promotion (`--candidate_promote_per_cell`) | Scores ready candidates by baseline, pixel displacement, and track length, then promotes the best per image grid cell. Loose room 8s improved only marginally over best-baseline promotion: per-cell 4 reached 0.6221 with 765 points, per-cell 8 reached 0.6231 with 1593 points, and stricter reprojection gates regressed to 0.641-0.652. Balanced promotion helps density/spread, but the candidate tracks themselves remain too weak. |
| LK-quality candidate gate (`--candidate_max_fb_err`, `--candidate_min_disp`) | Adds LK forward-backward error and displacement to tracked corners and filters candidate creation before promotion. Best room 8s was `--candidate_max_fb_err 0.25 --candidate_promote_per_cell 8`, improving same-binary room 8s 0.6141 -> 0.5869, but desk 8s was 0.6009 and room 30s regressed to 1.8452 with only 5549 points. Good diagnostic signal that LK quality matters, but still a short-window trap. |
| Tracker/matcher population dump (`--track_dump`) | Per-frame CSV for pure C LK and C++ ORB matching: matches/tracked count, map-linked count, mean FB error, mean displacement, and 8x6 grid occupancy. On room 8s, pure C tracked far more features (mean 662, grid 36.5, mean displacement 15.1) than C++ (mean 166, grid 28.4, displacement 8.8), yet ATE was much worse (0.6141 vs 0.2728). Raw track volume/spread is not the issue; C++ wins through descriptor/keyframe/map admission quality. Dumps: `runs/track_compare/pure_room_8s/tracks.csv`, `runs/track_compare/cpp_room_8s/tracks.csv`. |
| Map-admission dump (`--map_admission_dump`) | Per-keyframe CSV for candidate/admitted map points, baseline, reprojection error, parallax, and depth. On room 8s, C++ admitted 4149/4662 candidates (89.0%) with 1.70 px mean reprojection, 56.7 mean depth, and 1.17 mean baseline. Pure C admitted 2543/4785 (53.1%) but finite accepted rows averaged 415 px reprojection, 21150 mean depth, and 13204 mean baseline; several rows include infinite reprojection from near-zero depth. Reproj/depth gating (`--tri_max_reproj_px 4 --tri_max_depth 120 --tri_max_depth_ratio 12`) cleaned accepted points to 1.40 px but starved the map to 121 points and worsened room 8s to 0.6453. C++-like keyframe cadence (`--kf_min_inliers 20 --kf_period 999 --kf_max_rot_deg 180`) also starved the map to 148 points and worsened to 0.6480. Conclusion: the gap is not a single threshold; pure C needs C++-style descriptor/keyframe admission, not just LK volume or post-hoc gates. Dumps: `runs/map_admission_compare/pure_room_8s/admission.csv`, `runs/map_admission_compare/cpp_room_8s/admission.csv`. |
| Descriptor-mediated map admission (`--descriptor_map_admission`) | Opt-in pure-C path that keeps LK for pose/observation tracking but creates new map points from BRIEF Lowe-ratio matches, with an independent current-frame corner set for admission. It did not close the gap: room 8s was 0.6140 with 6015 points using default descriptor admission, essentially baseline ATE with worse admission geometry (finite accepted rows averaged ~4033 px reprojection, huge depth/baseline). Strict BRIEF thresholds (`--descriptor_admission_max_hamming 35 --descriptor_admission_ratio 0.6`) worsened to 0.6212, and E-only pose timing (`--pnp_start_frame 999`) worsened to 0.6331. Descriptor admission alone is not sufficient while the pure-C pose scale/keyframe state is already corrupting triangulation. Runs: `runs/descriptor_admission/`. |
| Descriptor-primary map admission (`--descriptor_primary_admission`) | C++-style opt-in variant that makes descriptor E-inliers the only source of new landmarks, forces descriptor-relative pose for triangulation, and bounds additions with `--descriptor_primary_map_cap` plus optional `--admission_max_new_points`. It still did not transfer C++ quality: room 8s regressed to 0.6335 with 4648 points, and anchor + primary regressed to 0.6504. The admission dump improved over the old descriptor add-on but remained far from C++: 4648 accepted points over 189 rows, 84 rows with infinite mean reprojection, and finite rows averaging ~66 px reprojection. Adding reprojection/depth gates starved the map and still regressed (`tri_max_reproj_px=4`: 223 points, 0.6487; `16`: 747 points, 0.6446). Older descriptor-source keyframes (`--descriptor_source_kf_gap 5/10`) reduced map growth and improved finite reprojection to ~38 px at gap10, but still regressed room 8s (gap5 0.6357, gap10 0.6429 after making source-gap mode wait for a qualifying KF instead of falling back to adjacent frames). Matcher-quality filters also failed: mutual BRIEF (`--descriptor_mutual_admission`) regressed primary to 0.6474 and source-gap5 to 0.6446, while stricter ratio 0.6 regressed primary to 0.6441 and source-gap5 to 0.6394. Re-testing primary admission under the current low-PnP/smoother profile still regressed room 30s to 1.6864 with only 7664 points. This means the remaining gap is not just "use descriptors for admission", "triangulate from older keyframes", or one-way ratio ambiguity; pure C still needs better source-pose/keyframe geometry or a stronger descriptor family before triangulation. Runs: `runs/descriptor_primary/`, `runs/pure_c_iter/descriptor_primary_current_room/`. |
| Oriented BRIEF (`--oriented_brief`) | ORB-adjacent pure-C probe that steers the BRIEF sampling pattern by an intensity-centroid orientation. It did not help descriptor-primary alone (room 8s 0.6341) or anchor-only (0.6312), but it did improve the best short-window anchor cadence path: `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief` reached room 8s ATE 0.5254 vs 0.5845 without orientation and 0.6141 baseline. Orientation is now part of the promoted default only together with anchor E, pyramid features, and full keyframe cooldown; by itself it was not a full-run fix. Runs: `runs/oriented_brief/`. |
| FAST-style corners (`--fast_corners`) | ORB-adjacent detector probe using a FAST-9 circle test with score-ranked nonmax selection. It did not transfer the ORB frontend advantage as implemented: with the best anchor cadence, FAST + oriented BRIEF regressed room 8s to 0.6208 and FAST without orientation regressed to 0.6284; descriptor-primary + FAST + oriented BRIEF regressed to 0.6503. This suggests the simple FAST selector is starving/reshaping features rather than reproducing OpenCV ORB's pyramid/distribution behavior. The useful ORB-like signal remains descriptor orientation, not this first detector approximation. Runs: `runs/fast_corners/`. |
| Pyramid/distributed features (`--pyramid_features`, `--distributed_features`) | Adds ORB-like feature population shaping without new tuning knobs: Harris candidates can be selected through a fixed 8x6 grid, and `--pyramid_features` adds a second half-resolution Harris level before grid selection. Flat grid selection alone regressed the best oriented-anchor room 8s path to 0.6362, and FAST+grid regressed to 0.6184. The two-level Harris pyramid was the first frontend population win: `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief --pyramid_features` reached room 8s ATE 0.4806 vs the prior 0.5254 best, with fewer points/KFs (4917/80 at 8s). It is now enabled by default only as part of the full promoted anchor/oriented/keyframe profile. Runs: `runs/pyramid_features/`. |
| Harris feature spacing (`--feature_min_dist`) | Opt-in nonmax spacing for the Harris/distributed selector. The default remains `0`; the selector now skips the O(selected) spacing test when that test cannot reject anything. A short-window `--feature_min_dist 4` probe looked promising (room 8s 0.6285 -> 0.6086, desk 8s 0.5785 -> 0.5722), but the 30s result reproduced the usual desk/room split: desk improved 0.7327 -> 0.7075 while room regressed 1.6234 -> 1.8329. Do not promote spacing without a policy that preserves room. Runs: `runs/pure_c_iter/feature_min_dist4_*`, `runs/benchmark/`. |
| Promoted anchor/pyramid keyframe profile | The new default is the strongest full-GT profile found so far: `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief --pyramid_features --kf_min_interval 2 --pnp_min_obs 2`. It improves mean pure-C-plus ATE from 0.6953 to 0.6576 and room from 1.7686 to 1.6234, with rpy becoming a small win at 0.0974. Adding `pnp_min_obs=2` to the prior promoted profile improved desk 0.7565 -> 0.7327, room 1.6362 -> 1.6234, rpy 0.0975 -> 0.0974, and xyz 0.1788 -> 0.1770. The late-only sibling (`--late_kf_cooldown` instead of full interval) was milder but weaker before the PnP-observation promotion: mean 0.6791, room 1.6895, desk 0.7514. Removing `--anchor_e_pose` starved the map and regressed mean to 0.7198, so anchor E is required for this profile. Runs: `runs/pure_c_iter/pyramid_kfmin2_fullgt/`, `runs/pure_c_iter/pyramid_late_fullgt/`, `runs/pure_c_iter/pyramid_kfmin2_noanchor_fullgt/`, `runs/pure_c_iter/current_pnp_minobs2_diag/`, `runs/pure_c_iter/current_pnp_minobs2_rpy_xyz_diag/`, `runs/benchmark/`. |
| Post-promotion keyframe/anchor scalar probes | Desk-only probes after the pre-`pnp_min_obs=2` promoted profile did not find a balanced recovery. Removing full cooldown (`--kf_min_interval 0`) moved desk 0.7565 -> 0.7512 but room 1.6362 -> 1.8511; a temporary delayed-cooldown-start build regressed both (desk 0.7574, room 1.8667), so that knob was removed. Anchor caps 300/900 gave desk 0.7520/0.7520 but room 1.8482/1.8146. Rotation triggers 5/15 deg gave desk 0.7475/0.7481 but room 1.7748/1.8472, and period 10 gave desk 0.7540, room 1.8498. Keep the promoted defaults unless a new policy can preserve room. Runs: `runs/pure_c_iter/default_no_kfmin_diag/`, `runs/pure_c_iter/kf_start50_diag/`, `runs/pure_c_iter/anchor300_diag/`, `runs/pure_c_iter/anchor900_diag/`, `runs/pure_c_iter/kf_rot5_diag/`, `runs/pure_c_iter/kf_rot15_diag/`, `runs/pure_c_iter/kf_period10_diag/`. |
| Post-promotion health/bootstrap policy probes | Existing health policies did not recover the desk/room tradeoff on the pre-`pnp_min_obs=2` promoted profile. `--healthy_keyframes` left desk unchanged at 0.7565 and regressed room to 1.7544; `--map_hygiene` only nudged desk to 0.7560 and regressed room to 1.7933. A temporary early non-PnP rotation-keyframe build showed the trap: limiting it to frame 100 recovered desk to 0.7277, but shifted keyframe parity across later frames and regressed room to 1.7630; the frame-200 variant was weaker on desk (0.7526) and still broke room. The bootstrap knob was removed. Runs: `runs/pure_c_iter/current_healthy_keyframes_diag/`, `runs/pure_c_iter/current_map_hygiene_diag/`, `runs/pure_c_iter/bootstrap_e_keyframes_100_diag/`, `runs/pure_c_iter/bootstrap_e_keyframes_diag/`. |
| Admission finite-only sanity (`--admission_finite_only`) | Opt-in non-threshold sanity check that rejects newly triangulated landmarks only when reprojection/depth/XYZ statistics are non-finite. It directly targets the `inf` admission rows seen in `runs/pyramid_features/room_8s_best_admission.csv`, but it still regressed the best pyramid+late-cooldown room 8s path to 0.6183. This means the pathological rows are a symptom of the admission/pose state, not a standalone fixable class; a viable policy needs batch-level ranking/deferment rather than a finite-only filter. |
| Post-promotion geometric admission gates | Pre-`pnp_min_obs=2` promoted-profile map-admission dumps still showed huge accepted reprojection rows (`runs/pure_c_iter/current_admission_dump_diag/`). Hard global gates recover desk but throw away room: `--tri_max_reproj_px 500` gave desk 0.7398 but room 1.8326, `--tri_max_reproj_px 2000` gave desk 0.7433 but room 1.8206, and `--admission_finite_only` nearly recovered desk to 0.7176 but regressed room to 1.8002 with 355 KFs / 17000 points. Pairing finite-only with `--kf_min_interval 3` reduced growth but still regressed room to 1.8474 while desk fell back to 0.7368. A temporary delayed quality-gate switch preserving the first 60 frames did not rescue the idea: `--tri_max_reproj_px 500` from frame 60 gave desk 0.7550 and room 1.8588, so the code was removed. This confirms geometric admission quality is a desk lever, but the current hard filters are not selective enough to preserve the room profile. Runs: `runs/pure_c_iter/tri_reproj500_diag/`, `runs/pure_c_iter/tri_reproj2000_diag/`, `runs/pure_c_iter/current_finite_admission_diag/`, `runs/pure_c_iter/finite_kfmin3_diag/`, `runs/pure_c_iter/tri_reproj500_start60_diag/`. |
| Admission inlier-support gates (`--admission_min_inliers`, `--admission_pnp_min_inliers`) | Keeping fallback keyframes but skipping new landmark admission on weak-support frames was too blunt. On the current promoted profile, `--admission_min_inliers 40` starved the map and regressed desk/room to 0.7559/1.8375 with 4252/7458 points; `20` still regressed to 0.7560/1.8607. PnP-only gates also failed: `--admission_pnp_min_inliers 20` gave 0.7550/1.8263, and the surgical 13-inlier cutoff gave 0.7438/1.8632. Runs: `runs/pure_c_iter/admission_min_inliers40_diag/`, `runs/pure_c_iter/admission_min_inliers20_diag/`, `runs/pure_c_iter/admission_pnp_min_inliers20_diag/`, `runs/pure_c_iter/admission_pnp_min_inliers13_diag/`. |
| Map-admission detail dump (`--map_admission_detail_dump`) | Per-candidate CSV for accepted and triangulation-failed admission attempts, with `tools/analyze_admission_detail.py` for GT-joined summaries. On the current promoted profile, full 30s desk reproduced 0.7327 with 11031 accepted rows and 12424 `tri_fail` rows; full 30s room reproduced 1.6234 with 16443 accepted rows and 14203 `tri_fail` rows. Joining accepted rows to GT-aligned frame error shows room high-error frames correlate with very large admitted depths (`err >= 2 m` median depth 644k vs `err < 1 m` median 6.3k), but desk also has 4935 accepted rows above 100k depth with only 0.77 m mean frame error. Reprojection, parallax, and depth are useful per-frame/per-method signals, not a safe standalone global gate. Runs: `runs/pure_c_iter/admission_detail_desk/`, `runs/pure_c_iter/admission_detail_room/`. |
| C++ admission-detail comparison | Added the same opt-in detail dump to `simple_slam_opt.cpp`; default C++ behavior is unchanged. The dump reproduced canonical C++ room/desk ATE at 1.5452/0.7194. In the room rolling-state trigger windows 352:365 and 556:585, C++ accepted descriptor landmarks with median depths 45.8/40.9 and median reprojection 0.3 px, with no accepted rows above 100k depth. `pure_c_plus` in the same windows accepted LK landmarks with median depths 1.41M/583k and median reprojection 222/88.6 px; the bad PnP batches alone had median depths 1.48M/1.38M. The desk trigger window shows the same scale split: C++ median depth 36.7 and 0.3 px reprojection vs `pure_c_plus` median depth 199k and 244 px reprojection, although desk ATE stays low. Full C++ room admitted 13331 points with median depth 21.3, p90 depth 51.6, median reprojection 0.3 px, only one accepted row above 100k, and none above 500k. This points at descriptor source geometry / correspondence population as the gap, not post-hoc filtering of already-bad LK landmarks. Runs: `runs/pure_c_iter/cpp_admission_detail_room/`, `runs/pure_c_iter/cpp_admission_detail_desk/`. |
| Map lifecycle dump (`--map_lifecycle_dump`) | Per-map-point CSV for birth source/method/stats plus final `obs`, `good_obs`, `bad_obs`, last-seen frame, span, and staleness, with `tools/analyze_map_lifecycle.py` for GT-joined summaries. On the current promoted profile, 30s room and desk reproduced 1.6234/0.7327. Default culling leaves all points alive, so lifetime must be read through observation span/staleness: room had median span 6 frames and 14373/16443 points stale by >=100 frames; desk had the same median span and 9286/11031 stale. Room high-error birth frames still correlate with large depth, but desk has similar stale/short-lived high-depth points without >2 m frame errors. This is a better diagnostic view, not a standalone culling/admission policy. Runs: `runs/pure_c_iter/lifecycle_room/`, `runs/pure_c_iter/lifecycle_desk/`. |
| Lifecycle batch-rule sweep | `tools/sweep_lifecycle_batch_rules.py` tests birth-frame batch predicates against room and desk lifecycle dumps. Future-aware rules using span/staleness can look selective, but they are not live policy candidates. With `--live_only`, the best zero-desk rule (`rows>=100 & med_depth>750k & med_inliers<=60`) catches only 7/66 bad room birth batches, while looser rules catch more room failures but flag desk batches too. This is useful triage, but it does not by itself justify another hard gate. |
| Live lifecycle batch guard (removed) | A temporary opt-in buffered direct LK admissions and rejected batches matching the best zero-desk live-only lifecycle rule: at least 100 accepted candidates, median depth above 750k, and inliers <=60. The live result did not transfer: it rejected 2079 candidates over 11 room frames, cut the map to 13954 points, and regressed room 30s from 1.6234 to 1.8255. The code was removed. Run: `runs/pure_c_iter/batch_guard_live_room/`. |
| Descriptor-confidence observation weighting (`--desc_weight_pnp`, `--desc_weight_ba`, `--desc_weight_gate`, `--desc_weight_scale`, `--refine_point_huber`) | Per-corner appearance confidence `weight = exp(-scale * hamming(map_desc, current BRIEF))` fed into PnP LM refinement, local-BA robust weights, a hard BA observation gate, and an optional Huber-ish robust weight in map-point refinement. Every mechanism regressed full 30s desk/room in isolation and in combination (baseline 0.5716/1.1799): all-on 0.6877/1.6057, BA-weight only 0.6943/1.5510, PnP-weight only 0.7026/1.2689, gate-0.5 only 0.7094/1.2525, point-refine Huber only 0.7143/1.5415. The drift-detector reformulation (weight vs per-frame refreshed descriptors via `--update_map_descriptors`) also failed: refresh alone 0.7219/1.2126, refresh+BA-weight 0.6938/1.4415, refresh+gate 0.6889/1.3677. Two conclusions: BRIEF distance to the stored map descriptor grows with viewpoint change, so weighting by it suppresses exactly the wide-baseline observations that constrain geometry best; and map-point refinement is so sensitive that even a plain Huber robust weight there (no descriptors involved) was the worst desk regression of the set. The experiment code was removed after rejection; the probes ran via temporary opt-in knobs whose off-state reproduced canonical ATE exactly. Runs: `runs/pure_c_iter/desc_weight/`. |
| Trace crossover diagnostic | `tools/analyze_trace_crossover.py` compares two GT-aligned traces, finds sustained frame-error crossovers, and prints rolling delta windows plus pose/map health and inferred keyframe reasons around the crossover. On canonical room, `pure_c_plus` beats C++ strongly in 51:80 (30-frame mean delta -1.196 m), then becomes persistently worse at frame 130 with margin 0.05 for 12 frames. The worst room window is 537:566 (mean delta +1.386 m), driven by low-inlier PnP/E jump frames such as 504, 497, 342, and 208. That window has 14 plus keyframes, 11 inferred from low inliers, but the earlier 115:145 crossover context also has 12/15 low-inlier keyframes while plus is still better than C++, so low-inlier keyframes alone are not the missing signal. Desk's worst 30-frame gap is smaller (+0.365 m at 534:563), and rpy/xyz rolling deltas are only centimeter-scale. This points at room-specific long-run pose/keyframe-map dynamics, not a broad all-sequence scalar fix. |
| Trace state-window diagnostic | `tools/analyze_trace_state_windows.py` summarizes rolling live-state windows from the metrics JSON: method counts, low-inlier PnP, keyframe reasons, point growth, jump counts, and linked support. The strongest current window-level signal is not a single low-inlier or jump frame, but high recent map growth plus repeated large jumps: `points_added>=1000 & jump500k>=4` flags 46 room windows with 12 high-error windows and 634 high-error room frame hits, while it flags only non-tail desk/rpy/xyz windows under the current diagnostic thresholds. This is still not a live policy by itself because overlapping windows exaggerate counts and suppressing those additions may change the trajectory, but it is a better next hypothesis than direct depth/jump/inlier gates. |
| Rolling state-window admission guard (removed) | Temporary opt-in used the live prior-30-frame version of the best state-window signal: suppress new landmark admission when the previous 30 frames had at least 1000 added points and at least four `trans_jump > 500k` frames. The timing looked plausible offline: room triggers at 352:365 and 556:585, desk only at 550:558 with no desk tail hits. The live result still did not transfer: room 30s regressed from 1.6234 to 1.6540, although max error fell 6.6242 -> 5.7169 and the map only shrank 16443 -> 16289 points. Code removed. Run: `runs/pure_c_iter/state_window_guard_room/`. |
| Rolling state-window PnP-depth guard (removed) | Follow-up to the suppress-all guard: only reject direct LK PnP admissions during the rolling unstable state when candidate depth is above 500k, preserving E admissions. Admission-detail windows made this look more selective: bad room trigger windows were mostly high-depth PnP admissions, while the desk trigger window had only E admissions under 500k depth. The live result was worse: it rejected 403 candidates over frames 353, 497, 576, and 578, grew the room map to 17024 points, and regressed room 30s to 1.7880. Code removed. Run: `runs/pure_c_iter/state_pnp_depth_guard_room/`. |
| Trace health rule sweep | `tools/sweep_trace_health_rules.py` sweeps simple live predicates from the existing metrics JSON over the canonical `pure_c_plus` traces. It uses sequence-specific tail thresholds for diagnosis only (`room>2.5m`, `desk>1.0m`, `rpy>0.15m`, `xyz>0.25m`). No simple existing trace predicate is promotion-ready: `pnp&inl<=16&jump>500k` catches 7/69 high-error room frames but also flags 22 desk, 14 rpy, and 2 xyz frames; `jump>1m` catches 24/69 high-error room frames but also flags 75 rpy frames. The narrower `KF&add>=100&jump>500k` avoids high-error desk/xyz frames but catches only 4/69 room high-error frames and still flags 23 rpy frames. This argues for new state or a richer multi-frame signal, not another direct gate on the fields already in the trace. |
| Current-profile depth admission gates | The per-candidate dump made depth look tempting, but live gates still changed the trajectory/map shape in the wrong direction. Global `--tri_max_depth 1000000` regressed room 30s to 1.8199, and `5000000` regressed to 1.8353 vs baseline 1.6234. A temporary PnP-only admission depth gate was also rejected and removed: 1e6 gave room 1.8322, 2e6 gave 1.8137. Depth is a correlation on bad room frames, not a standalone admission policy. Runs: `runs/pure_c_iter/tri_depth1m_room_diag/`, `runs/pure_c_iter/tri_depth5m_room_diag/`, `runs/pure_c_iter/admission_pnp_depth1m_diag/`, `runs/pure_c_iter/admission_pnp_depth2m_diag/`. |
| PnP batch-median depth gate (removed) | Frame-level admission-detail analysis showed PnP batches with median depth above 500k flag 8/13 bad room PnP-admission frames while desk has no frames above 2 m error. The live opt-in did not transfer. Skipping PnP batches above median depth 500k regressed room to 1.8202, 1M regressed to 1.7649, and the very loose 2M case was only ATE-neutral/slightly worse at 1.6277 while rejecting just 11 rows across 2 frames; desk at 2M was unchanged at 0.7327. Removed because it adds stateful batch buffering without an ATE win. Runs: `runs/pure_c_iter/admission_pnp_batch_med500k_diag/`, `runs/pure_c_iter/admission_pnp_batch_med1m_diag/`, `runs/pure_c_iter/admission_pnp_batch_med2m_diag/`, `runs/pure_c_iter/admission_pnp_batch_med2m_desk_diag/`. |
| E-batch health gate (removed) | A temporal admission-detail probe combined E-pose method, accepted batch size >=80, median depth >500k, and translation jump >500k. Offline it looked more selective than depth alone (16 room batches, 1 desk batch), but the live opt-in rejected 1331 rows over 10 room frames and regressed room 30s to 1.8369 while only reducing max error from 6.62 to 3.01. This is another example of suppressing a visible spike class while damaging the trajectory/map state. Runs: `runs/pure_c_iter/admission_e_batch_health_diag/`, `runs/pure_c_iter/admission_e_batch_health2_diag/`. |
| Adaptive admission batch filter (removed) | Temporary code buffered direct-LK admission candidates and applied a finite/reprojection gate only when a keyframe batch looked unhealthy. It reproduced the same desk/room split instead of solving it: default trigger 100 px, max 500 px, bad ratio 0.15 gave desk 0.7434 and room 1.7786; relaxed trigger 200 px / bad ratio 0.25 gave desk 0.7482 and room 1.8182. The code was removed. Simple batch-health thresholding is not selective enough without a stronger signal about which keyframes/landmarks room needs to keep. Runs: `runs/pure_c_iter/adaptive_admission_diag/`, `runs/pure_c_iter/adaptive_admission_relaxed_diag/`. |
| Batch-ranked LK admission (`--admission_batch_ranked`) | Buffers LK triangulation candidates for a keyframe, ranks them by reprojection error, and commits a batch capped by current pose inlier support instead of pushing landmarks immediately. A cap of 1x and 2x inlier support starved room 8s (0.6406 and 0.6109). The current internal 4x-inlier support cap is the first admission-policy win: with `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief --pyramid_features --late_kf_cooldown --admission_batch_ranked`, room 8s improved to 0.3619 with 4628 points and 94 KFs. The 30s gate still regressed to 1.8313 with 15229 points and 362 KFs, and pairing batch admission with full `--kf_min_interval 2` regressed room 8s to 0.6164. This validates batch admission as a short-window lever, but long-run stability still needs better temporal/deferment logic. Runs: `runs/pyramid_features/`. |
| Deferred batch admission (`--admission_batch_deferred`) | Reuses the batch-ranked LK selector but stores selected landmarks as tentative candidates first, promoting them only after the existing multi-observation candidate readiness check. This was too sparse for the current pose/map loop: default candidate survival regressed room 8s to 0.6258 with 2429 points, and a shorter survival window (`--candidate_min_age 2 --candidate_min_obs 3`) regressed to 0.6430. The current system still needs immediate enough geometry for PnP; simple deferment starves it before improving long-run health. |
| Healthy keyframes (`--healthy_keyframes`) | Centralizes keyframe decisions in `should_make_keyframe` and, after the first 200 frames, suppresses low-inlier fallback KFs unless the frame has enough linked map support. Applying it from frame 0 destroyed the oriented-anchor 8s win (0.6425). Starting the policy after frame 200 preserved the 8s win exactly (0.5254) but did not fix room 30s: point/KF growth fell only modestly (32230/682 -> 30009/676) and ATE worsened to 1.8532. A real long-run fix needs a stronger keyframe/map health signal than linked-count gating. Runs: `runs/healthy_keyframes/`. |
| Map hygiene (`--map_hygiene`) | Reuses the existing recent-window reprojection culler outside joint BA. Applying it immediately broke the oriented-anchor 8s win (0.6273). Starting it after frame 200 preserved the 8s win exactly (0.5254), but room 30s still failed and worsened to 1.8627 with 31070 points and 687 KFs. Marking all-bad recent-window observations as unusable is not enough once the long-run trajectory/map state has drifted. Runs: `runs/map_hygiene/`. |
| Delayed initialization (`--delayed_init_frames`) | Opt-in map-start delay. With descriptor-primary admission it suppresses early landmarks, stores keyframes, then initializes from the first stored keyframe once the delay is reached. This produced a very sparse map and regressed room 8s (`20`: 340 points, 0.6430; `40`: 334 points, 0.6451). Letting default LK admission start after the delay was also not a win (`20`: 5474 points, 0.6160; `40`: 4150 points, 0.6510). This rules out a simple "wait, then initialize from the first KF" fix; a viable delayed init would need actual multi-frame track survival / bundle initialization, not just a delayed two-view triangulation. Runs: `runs/delayed_init/`. |
| E-constrained triangulation pose (`--triangulate_with_e_pose`, `--triangulate_relative_frame`) | Tests replacing the PnP/LM world pose used for map admission with the current E relative pose. `--triangulate_with_e_pose` improved admission reprojection from ~415 px to ~61 px but admitted 9646 points and slightly worsened room 8s to 0.6166; descriptor + E pose worsened to 0.6389. `--triangulate_relative_frame` triangulates in the previous-camera local frame before transforming into world, fixing the dump baseline to 1.0 but still leaving ~63 px reprojection and worsening room 8s to 0.6383; descriptor + relative frame worsened to 0.6477. Re-testing under the current low-PnP/smoother profile still regressed room 30s badly: E-pose admission 1.7019 and relative-frame admission 1.6936. This narrows the remaining issue: pure-C E/LK correspondences themselves are not producing C++-quality two-view geometry (C++ is ~1.7 px), independent of world-scale pose corruption. Runs: `runs/triangulate_pose/`, `runs/pure_c_iter/tri_e_pose_current_room/`, `runs/pure_c_iter/tri_relative_current_room/`. |
| E-inlier correspondence dump (`--e_inlier_dump`) | Per-E-inlier CSV for pair coordinates, displacement, match score, linked map point, LK FB error, and 8x6 grid cell. On room 8s, pure C E inliers came from only 40 frames but averaged 137.7 inliers/frame with mean displacement 15.65 px (p90 27.9); C++ pose E inliers came from 122 frames with 42.5/frame and mean displacement 7.69 px (p90 13.4), and C++ triangulation inliers were similar at 48.1/frame and 7.64 px. Pure C is not lacking raw E inliers; it gets bursty, longer-displacement LK inliers, while C++ has steadier descriptor correspondences. Next promising lever is an LK/E admission policy that matches C++'s displacement/temporal profile (cap or rank E inliers by displacement/FB/grid before pose/admission), not another triangulation-pose variant. Dumps: `runs/e_inlier_compare/pure_room_8s/e_inliers.csv`, `runs/e_inlier_compare/cpp_room_8s/e_inliers.csv`. |
| LK/E inlier shaping (`--shape_e_inliers`) | Ranks/caps pure-C LK matches before E by displacement target, FB error, and 8x6 grid count. A strict C++-like profile (`max_disp=14`, `grid_cap=1`, `max=64`) produced C++-like displacement (mean 8.67 px, p90 12.5) but starved E to 12.6 inliers/frame and only 223 room-8s points. The best 8s profile found (`--e_shape_max_disp 30 --e_shape_target_disp 8 --e_shape_grid_cap 3 --e_shape_max_matches 144 --e_shape_max_fb_err 0.5`) improved room 8s slightly, 0.6141 -> 0.6047, and spread E across 196 frames with 22.1 inliers/frame, but room 30s was 1.7979 with only 7820 points, so this is another short-window diagnostic rather than a default. Re-testing that profile under the current smoother regressed the full GT sweep to desk/room/rpy/xyz 0.6366/1.5696/0.0973/0.1755. It confirms the direction is correspondence population shaping, but simple pre-E caps still starve long-run geometry. Runs: `runs/e_shape/`, `runs/pure_c_iter/e_shape_current_best_allgt/`. |
| LK/E pose-only shaping probe (removed) | A temporary split used shaped E matches for pose but raw E matches for map admission to avoid the map starvation above. Under the current smoother it still regressed the full pure-C-plus sweep: desk/room/rpy/xyz were 0.6327/1.5494/0.0962/0.1768 with the best known shaping profile. The code was removed. Run: `runs/pure_c_iter/e_shape_pose_only_current_best_allgt/`. |
| Descriptor anchor E pose (`--anchor_e_pose`) | Maintains a BRIEF descriptor anchor set from keyframes, matches it into independent current corners, and uses that E pose when PnP fails while keeping LK admission separate. Fixing LK admission after anchor pose avoided the initial map starvation. Default anchor cap 600 gave a small room-8s improvement (0.6141 -> 0.6117) with 8581 points, but room 30s did not complete even with a 300s timeout (654/750 frames, 29788 points, ATE 1.7147). Lowering anchors to 200 reduced map growth but regressed room 8s to 0.6250. Bounded admission (`--admission_max_new_points 160 --admission_grid_cap 8`) reduced room-8s map growth to 5834 points but regressed to 0.6310; ranked bounded admission (`--admission_ranked`, target 8/12 px) regressed further to 0.6491/0.6442. The best short-window anchor variant so far is keyframe cadence (`--kf_period 20 --kf_max_rot_deg 45`), improving room 8s to 0.5845 with 7610 points, but room 30s still only reached 704/750 frames under the 300s wrapper and had ATE 1.7744 with 27648 points. A direct keyframe cooldown (`--kf_min_interval 2/3`) cut room-8s KFs/points but regressed to 0.6408/0.6415, and combining cadence with moderate admission caps (`240/g10`, `320/g12`) also regressed to 0.6505/0.6446. Anchor E remains useful architecturally, but throttling volume after the fact is not enough; this branch still lacks a 30s-stable admission/keyframe quality policy. Runs: `runs/anchor_e/`. |
| TwitchSLAM reference benchmark | TwitchSLAM is only modestly more accurate on room while being far slower: room 8s ATE was 0.3373 in 58.6s wall time vs the current pure-C short-window best at 0.3619, and room 30s with a 300s timeout processed only 654/750 frames with ATE 1.4422. Treat it as a robustness reference, not a target implementation. Runs: `runs/benchmark_twitchslam/`. |
| Room full-run fast profile sweeps (`pnp_dlt_iters=100/200`, LK 7/3) | All worsened the current fast-320/no-BA room 30s result: PnP100 1.8334, PnP200 1.8265, LK7/3 1.8139 vs existing PnP150 1.6861. A full-room old-default `--pnp_dlt_iters 2000` probe also worsened to 1.8185 and reduced total PnP successes, so late PnP collapse is not just too few RANSAC samples. More points/inliers again correlated with worse ATE. |
| Global BA hotspot and bounded-BA probes | Per-frame profiling on room 30s found `global_ba` as the dominant cost: baseline processed 657 frames with ATE 1.7231 and spent 83.2/125.4s in `global_ba`. A blunt `--global_ba_max_points 5000` completed 750 frames and cut time to 56.7s, but worsened ATE to 1.7715; adding `--ba_interval 2` cut time to 45.6s but worsened to 1.7855 and grew the map to 30969 points. Naive bounded global BA was not better: 80 KFs/5k active points reached 1.8265 in 63.5s, 200 KFs/10k reached 1.8358 in 53.7s, and ranking active points by window observations did not change that result. Running full global BA less often (`--global_ba_interval 20`) still spent 65.2s in `global_ba`, processed only 710 frames, and worsened to 1.8112. Conclusion: global BA is the hotspot, but simple caps, lower frequency, and naive bounded windows do not preserve the stabilizing effect. Runs: `runs/pure_c_iter/hotspot_profile/`, `runs/pure_c_iter/global_ba_cap/`, `runs/pure_c_iter/ba_throttle/`, `runs/pure_c_iter/bounded_global_ba*/`, `runs/pure_c_iter/global_ba_interval20/`. |
| Global/local BA observation index | Default BA now builds temporary point-to-keyframe-corner observation indexes, preserving the BA schedules while avoiding the old map-point x keyframe x corner scans. After fixing benchmark rebuild dependencies for `simple_slam_c_plus_*.h`, the rebuilt promoted default room profile spends only 0.71s in local BA and 0.39s in global BA over 750 frames (`runs/pure_c_iter/profile_rebuilt_default/room_profile.csv`). Latest canonical `pure_c_plus` runtimes are 35.3/43.3/42.4/42.0s for desk/room/rpy/xyz, with full-room reported ATE 1.1799 and mean pure-C-plus reported ATE 0.5050. |
| Current-smoother BA cadence probe | Re-testing `--global_ba_interval 5` after the observation-index and output-smoother wins still did not beat the promoted default. Full GT desk/room/rpy/xyz moved to 0.7108/1.5640/0.0944/0.1755, so less frequent global BA remains a raw-state regression rather than a runtime/accuracy win. Run: `runs/pure_c_iter/global_ba_interval5_current_allgt/`. |
| BA every 5 frames with global BA off | Room 30s moved to 1.7793 and desk 30s to 0.7563; near existing behavior, not a full-run win. |
| Joint Schur-complement local BA + map-point culling (`--joint_ba`) | Mathematically correct (5-KF window, 1 fixed gauge KF, χ²(2,0.95)=5.991 Huber, LM with diagonal damping and step rejection, Schur-eliminated points; cull at 8 px reproj. when ALL window obs are bad). Full sweep regressed every sequence: desk +0.034, room +0.039, rpy +0.018, xyz neutral (`runs/joint_ba_sweep/`). Pose-only joint BA (skip dP back-sub) on 8s room also regressed (0.635 vs 0.545). Code retained behind `--joint_ba` flag (`joint_local_ba` in `simple_slam_c_plus_backend_ba.h`, `cull_map_points_window` in `simple_slam_c_plus_backend_map.h`); useful baseline for future work that pairs BA with proper keyframe selection / initialization. Default behavior unchanged. |
| Overnight room probes (2026-05-15) | Several narrow, opt-in probes failed to beat the promoted room 30s baseline of 1.6234: rigid DLT/PnP scoring 1.8564, final rigid-pose validation 1.6575, validation after 3000/5000 map points 1.6884/1.7038, map descriptor refresh 1.6438, RANSAC seed 42 1.6897, local BA with oldest local KF fixed 1.8413, PnP-only depth cap 2M/5M 1.8204/1.8020, `pnp_quality_gate_px` 32/64 1.8524/1.7586, and observation-stat gates 4/16 px 1.7615/1.7112. These reduce selected spikes or map volume but damage the trajectory/map balance, so none should be promoted. |
| LM step-acceptance probe (removed) | A trial robust-cost/backtracking guard for `refine_pose_lm` rejected LM steps that did not lower reprojection cost. Despite being a cleaner optimizer rule, it changed the map/pose balance badly: room 30s regressed from the smoother-only 1.2726 to 1.7276 and map growth fell to 10888 points. The code was removed. Run: `runs/pure_c_iter/lm_step_accept_room/`. |
| Non-destructive predicted-pose PnP mask (removed) | Temporary code built a PnP-only predicted-reprojection mask instead of unlinking tracks like old `--pnp_pred_reproj_gate`. A 64 px mask was a strong room-only diagnostic (1.1545), but the full pure-C-plus GT sweep regressed desk/rpy/xyz to 0.7190/0.0959/0.1764; stricter/looser room probes also failed (16 px 1.4997, 48 px 1.5276, 80 px 1.6534, 128 px 1.7420), and unstable-window-only gating was still 1.4560. The code was removed; the signal is real but needs a better live acceptance rule. Runs: `runs/pure_c_iter/pnp_pred_mask*_room/`, `runs/pure_c_iter/pnp_pred_mask64_allgt/`. |
| Broad Low-PnP E fallback probe (not promoted) | Re-testing `--pnp_low_e_fallback --pnp_low_e_min_jump 500000` under the current smoother improved mean ATE to 0.5370 by moving desk 0.6435 -> 0.6193 and room 1.2726 -> 1.2571, but regressed rpy/xyz to 0.0952/0.1763. Retuning the output smoother on those traces lowered mean further offline but did not remove the rpy/xyz regressions, and stricter variants were worse (`min_jump=1000000`: room 1.4773; `max_inliers=12`: room 1.4701; `min_gain=16`: mean 0.5513; `min_inliers=48`: room 1.4261). The broad form remains rejected; it led to the narrower promoted mature-map handoff below. Runs: `runs/pure_c_iter/low_pnp_e_*`. |
| Mature-map Low-PnP E handoff (promoted) | The promoted default enables the low-PnP E handoff only after the map has at least 4000 points, the weak PnP pose is far from both the predicted pose and previous pose (`500000` and `250000` world units), and E has at least 24 inliers with an 8-inlier gain over PnP. Before the final smoother retune, a clean `python3 benchmark_native.py --all_gt --force` moved full-GT `pure_c_plus` desk/room/rpy/xyz from 0.6435/1.2726/0.0933/0.1742 to 0.6124/1.2571/0.0922/0.1763, lowering mean 0.5459 -> 0.5345. The base retuned smoother later reported 0.5633/1.1982/0.0920/0.1763, mean 0.5075; the current low-linked-support smoother gate reports 0.5716/1.1799/0.0920/0.1763, mean 0.5050. Earlier gates were rejected: 3000 points regressed rpy to 0.0965, 4500 points protected rpy but regressed desk to 0.6980, 5000/6000 points regressed room to 1.3280, and refined-PnP previous-jump vetoes regressed desk/room. Runs: `runs/pure_c_iter/low_pnp_e_map*_allgt/`, `runs/pure_c_iter/low_pnp_e_pose*_allgt/`, `runs/benchmark/`. |
| Wider Low-PnP E handoff probes (not promoted) | The current raw room trace has a large 20-inlier PnP spike, but widening the handoff ceiling did not transfer. `--pnp_low_e_max_inliers 20` regressed room 30s to 1.5357, and a surgical variant with both jump gates raised to 5M still regressed to 1.3112. Keep the current 16-inlier ceiling unless a richer acceptance signal can distinguish the catastrophic frame from the harmful replacements. Runs: `runs/pure_c_iter/low_pnp_e_max20_room/`, `runs/pure_c_iter/low_pnp_e_max20_jump5m_room/`. |
| Current-smoother PnP/admission probes | More DLT-PnP RANSAC budget and looser admission support both regressed the full current-smoother sweep. `--pnp_dlt_iters 1000` produced desk/room/rpy/xyz 0.7250/1.5882/0.0940/0.1761, and `--admission_pnp_min_inliers 16` produced 0.6847/1.5409/0.0991/0.1763. The remaining issue is not simply too few PnP samples or too strict an admission inlier count. Runs: `runs/pure_c_iter/pnp_dlt1000_current_allgt/`, `runs/pure_c_iter/admission_min16_current_allgt/`. |
| Pre-low-link current-profile scalar probes (not promoted) | Re-testing simple raw-state scalars under the low-PnP/base-smoother profile did not beat that promoted default room 30s ATE 1.1982. PnP quality gating (`--pnp_quality_gate_px 32`) regressed to 1.7203, delaying PnP to frame 120 regressed to 1.6373, anchor caps 300/900 regressed to 1.5368/1.4056, keyframe periods 15/30 regressed to 1.5156/1.5570, and `--kf_min_interval 3` regressed to 1.4846. The anchor/keyframe/PnP timing profile remains better than these scalar changes. Runs: `runs/pure_c_iter/pnp_quality32_current_room/`, `runs/pure_c_iter/pnp_start120_current_room/`, `runs/pure_c_iter/anchor*_current_room/`, `runs/pure_c_iter/kf_period*_current_room/`, `runs/pure_c_iter/kf_min3_current_room/`. |
| Current-profile P3P fallback probe | Enabling the existing numeric P3P fallback under the current low-PnP/smoother profile changed the room map/keyframe profile and regressed room 30s to 1.3028. The fallback is not a drop-in rescue for late weak-PnP frames; a future P3P/AP3P path needs a stricter candidate acceptance rule before promotion. Run: `runs/pure_c_iter/p3p_fallback_current_room/`. |
| Predicted-LM pose fallback probe (removed) | A temporary opt-in tried replacing the selected PnP/E pose with the existing refined predicted pose when predicted-LM inliers were stronger. Broad room use (`min_inliers=40`, gain 10, selected-vs-predicted jump >=1000) changed 45 poses and regressed room to 1.5131; a stricter 100k jump gate was a no-op at 1.1982. The code was removed. Runs: `runs/pure_c_iter/pred_lm_fallback_room_g10_j1k/`, `runs/pure_c_iter/pred_lm_fallback_room_g10_j100k/`. |
| Current-profile room probes after handoff/smoother (not promoted) | Room-only diagnostics on the 0.5633/1.1982/0.0920/0.1763 profile did not find a raw-state fix. `--healthy_keyframes` regressed room to 1.2473, `--admission_batch_ranked` to 1.5484, `--pnp_normalize_world` to 1.6502, `--normalize_world_scale` to 1.5559, `--map_hygiene` to 1.5884, `--pnp_score_rigid` and `--pnp_validate_rigid` to 1.4938, `--tri_max_depth_pnp 500000` to 1.6096, `--subpixel_features` to 1.2573, and `--feature_min_dist 2` to 1.6507. `--distributed_features` was a no-op on that room trace. Runs: `runs/pure_c_iter/healthy_keyframes_current_room/`, `runs/pure_c_iter/admission_batch_ranked_current_room/`, `runs/pure_c_iter/pnp_normalize_world_current_room/`, `runs/pure_c_iter/normalize_world_scale_current_room/`, `runs/pure_c_iter/map_hygiene_current_room/`, `runs/pure_c_iter/pnp_score_rigid_current_room/`, `runs/pure_c_iter/pnp_validate_rigid_current_room/`, `runs/pure_c_iter/tri_pnp_depth500k_current_room/`, `runs/pure_c_iter/subpixel_current_room/`, `runs/pure_c_iter/feature_min_dist2_current_room/`. |
| Conditional causal output smoother (`output_smooth_alpha=0.040`, `output_smooth_outlier_alpha=0.003`, `output_smooth_residual_k=3.50`, `output_smooth_window=48`, unstable window 48, low-link gate 70/20/4 with current frame) | Promoted after clean `python3 benchmark_native.py --all_gt --force` sweeps. It applies a causal residual-adaptive EWMA only to reported camera centers, leaves map/pose state unchanged, and stores the unfiltered center as `raw_xyz`. The base smoother moved full-GT `pure_c_plus` ATE desk 0.7327 -> 0.6424, room 1.6234 -> 1.3501, rpy 0.0974 -> 0.0941, xyz 0.1770 -> 0.1739; mean moved 0.6576 -> 0.5651. The mature-map low-PnP E handoff plus the prior smoother reported 0.6124/1.2571/0.0922/0.1763, mean 0.5345. The first handoff retune (`alpha=0.035`, window 24, unstable alpha 0.30, unstable residual gate 2.50, unstable `points_added>=1500`) moved the clean full-GT sweep to 0.5897/1.2115/0.0921/0.1761, mean 0.5174. A bounded follow-up sweep over those `raw_xyz` traces promoted the base retuned defaults (`alpha=0.040`, outlier alpha 0.003, residual gate 3.50, window 48, unstable alpha 0.15, unstable residual gate 3.00, unstable `points_added>=1200`, `jump_count>=5`, residual cap off), and the clean full-GT sweep reported 0.5633/1.1982/0.0920/0.1763, mean 0.5075. The first low-link gate (`threshold=80`, `window=16`, `count=2`, `alpha=0.003`) moved mean to 0.5063. The latest promoted low-linked-support gate includes the current frame, lowers alpha to 0.007 when at least 4 of the prior 20 samples plus current frame have fewer than 70 linked map points, and clean full-GT reports 0.5716/1.1799/0.0920/0.1763, mean 0.5050. This is a small mean win with a deliberate desk tradeoff, and it still only changes reported `xyz`, not `raw_xyz` or map/pose state. Rejected local retunes: pose-jump 100k was 0.6193/1.2571/0.0922/0.1763, pose-jump 500k was 0.7439/1.8260/0.0922/0.1763, and map maturity 3000 was 0.6124/1.2571/0.0965/0.1763. |
| Output-smoother rejected follow-ups | Earlier offline causal sweeps over the validated `raw_xyz` traces found only marginal scalar gains after the base adaptive default. After the 0.5075 mean default, the first promoted low-link gate (`threshold=80`, `window=16`, `count=2`, `alpha=0.003`) landed at mean 0.5063. A second bounded offline sweep promoted the include-current low-link gate (`threshold=70`, `window=20`, `count=4`, `alpha=0.007`) after focused and clean full-GT validation landed at mean 0.5050. Wider low-link offline candidates remain rejected because they crossed the desk guardrail despite slightly better predicted means: `threshold=80`, `window=24`, `count=6`, `alpha=0.01` predicted mean 0.5056 but desk 0.5756, and a broader combined retune predicted mean 0.5066 with desk 0.5819. The unconditional residual-cap variant predicted mean 0.5639 but traded desk 0.6424 -> 0.6686 for room 1.3501 -> 1.3195, so the unconditional form remains rejected. The current promoted smoother keeps the cap off and uses only lower-alpha unstable/low-link modes. |
| Causal linear output filters (not promoted) | Offline causal linear extrapolation filters over the current `raw_xyz` traces were not sequence-safe. The best cases helped xyz by only 0.0005-0.0017 m while regressing desk by 0.038-0.052 m and room by 0.088-0.141 m, so they are worse than the promoted residual-adaptive EWMA despite using more recent motion information. |

Next useful work should preserve the promoted anchor/pyramid/keyframe/PnP-
observation profile plus output smoother, then attack the raw-state problem
visible in `raw_xyz`. The smoother is a practical ATE win, but it adds output
lag and does not make the internal map geometry healthier. Future raw-pose work
should target source geometry / correspondence population before triangulation,
or a well-formed P3P/AP3P candidate generator validated first through saved
dumps. Do not add another hard depth, jump, mean-reprojection, or recent-KF PnP
mask without a materially new signal; those policies repeatedly reduced visible
spikes while regressing the live trajectory. If joint BA is revisited, pair it
with better initialization/keyframe policy/PnP candidates; joint BA alone is
documented to regress.

## Design Direction

Phase 1 is accuracy and robustness. `pure_c_plus` now beats `cpp` on mean
reported ATE with the output smoother enabled, but raw-state robustness remains
the active accuracy target; LOC is only a tiebreaker during this phase.

Phase 2 starts once the raw-state path, not just the smoothed output trajectory,
is within about `0.02 m` mean ATE of `cpp`. At that point, reduce and simplify
the implementation while preserving the validated GT numbers.

Experiment-residue cleanup (2026-06-11): `pure_c_plus` was reduced from 6498 to
3949 LOC by deleting the code paths behind rejected opt-in flags, with the
default (promoted) behavior preserved exactly — the full `--all_gt` sweep
reproduces canonical ATE and identical map point counts on all four GT
sequences. Removed flag families: `--candidate_*` track promotion,
`--descriptor_*` admission variants, `--tri_*` triangulation gates,
`--admission_*` ranking/batch/deferral, `--shape_e_inliers`/`--e_shape_*`,
`--pnp_p3p_*` numeric P3P, `--pnp_quality_*`/`--obs_stat_*`/
`--pnp_pred_reproj_gate` masks, `--pnp_score_rigid`/`--pnp_validate_rigid`/
`--pnp_normalize_world`/`--pnp_dlt_pretest*`, `--fast_corners`/
`--subpixel_features`/`--feature_min_dist`/`--distributed_features` (the
two-level pyramid + grid selection is now the only frontend path),
`--healthy_keyframes`/`--late_kf_cooldown`/`--map_hygiene`/
`--kf_warmup_frames`/`--first_kf_observations`/`--unique_kf_observations`,
`--update_map_descriptors`, `--pnp_start_frame`, and `--delayed_init_frames`.
The `Corner.cand_idx` field, the candidate/admission data structures, and the
per-frame `pnp_p3p_*` timeline keys in the metrics JSON were removed with them.
All diagnostics dumps (`--pnp_dump`, `--track_dump`, `--map_admission_*`,
`--map_lifecycle_dump`, `--e_inlier_dump`) and the BA/anchor/smoother knobs
remain. Raw-state robustness work (the active blocker) is unaffected; removed
implementations are recoverable from commit `8df5dc7`.

Bit-exact speed pass (2026-06-12): two output-preserving optimizations make
`pure_c_plus` roughly 20% faster on the full sweep (room 43.3 s -> 34.4 s, desk
35.3 s -> 28.6 s in the canonical tables). Harris candidate collection now
precomputes per-pixel gradient products once instead of recomputing them in
each of the nine overlapping structure-tensor windows (identical float
accumulation order), and the BRIEF orientation centroid accumulates its
moments in integers (the previous double accumulation was already exact, so
the values match bit for bit). Both changes were verified by per-frame
`xyz`/`raw_xyz` byte comparison against the pre-change binary, not just ATE.
A separable rewrite of `blur_3x3` was tried and reverted: bit-exact but slower
(2.99 s -> 5.31 s on room) because the uint16 intermediate costs more memory
traffic than the 9-tap recompute saved. Profiling notes for future passes: the
anchor-E block dominates frame time (extraction ~7.8 s of which the candidate
qsort is the floor — replacing `qsort` changes float-tie ordering and is NOT
bit-exact, so that is an algorithm-change experiment, not an optimization);
LK bilinear sampling (~6 s) could hoist the interpolation weights once per
iteration since all 49 window samples share the same fractional offsets, but
the rewrite must preserve `get_pixel_bilinear`'s out-of-bounds-returns-0
behavior for the ±1 gradient probes at the image border.

Code style is deliberately "math on paper": matrices laid out as grids, one
equation per line, sparse comments where they clarify non-obvious numerical
steps. Avoid dataset-specific magic constants and hidden frame skipping.
Keep experimental knobs explicit: defaults should be named in `Config`
initialization, and repeated mode checks should live behind small helper
predicates instead of open-coded flag combinations. Negative probes stay
documented as diagnostics; they should not become implicit default behavior.
Image buffers that are owned by long-lived structures should use small image
wrappers with explicit clone/free helpers instead of open-coded
`malloc(width * height)` copies and scattered frees.

The BRIEF loop-detector work is parked on branch `feat/brief-loop-detector`.

## Development Workflow

Work on the current branch by default. Do not create a branch unless there is a
clear repo-specific reason.

Do not overwrite canonical benchmark summaries for one-off experiments unless
the experiment is intended to become the new baseline. Save exploratory runs
with explicit suffixes or dedicated folders, for example
`runs/benchmark/*_1s.json`, `runs/benchmark/*_5s.json`,
`runs/benchmark_history/`, or `runs/pure_c_iter/`. If an old experiment is
promoted into the active repo, give it a stable in-tree name and add it to the
benchmark tables.

Do not commit or push unless explicitly asked. If a commit is requested, use
the repo's existing Git identity; do not set or override
`git config user.name` or `git config user.email`.

Before closing a session that touched algorithm files or added diagnostic
tooling, update the relevant README section so the next worker does not have to
re-derive the conclusion. New diagnostic tooling goes under `## Diagnostics`.
New findings, hypotheses, or rejected trials about `pure_c_plus` go under
`## Active Pure-C Blocker`; a dated subsection is fine when the finding does
not yet fit a rejected-trial row.

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

`benchmark.py` rebuilds standalone pure-C binaries when their source inputs are
newer than the binary. For `pure_c_plus`, this must include
`simple_slam_c_plus_*.h`; otherwise header-only algorithm changes can silently
benchmark a stale `simple_slam_pure_c_plus` executable.

`simple_slam_c.c` is shared by the OpenCV-shimmed `c` implementation and the
standalone `pure_c` binary. Do not assume those benchmark entries are identical:
one uses the shim path, the other is fully standalone.

## Published Reference Context

Published monocular ORB-SLAM numbers are available for `fr1/xyz` and
`fr1/desk`, but not for `fr1/room` or `fr1/rpy`.

| Sequence | Published mono reference | This repo current best |
|----------|--------------------------|------------------------|
| `fr1/xyz` | ORB-SLAM 2015: 0.009 m | `cpp`: 0.1729 m |
| `fr1/desk` | ORB-SLAM 2015: 0.017 m | `pure_c_plus`: 0.5716 m |
| `fr1/room` | No credible mono reference found | `pure_c_plus`: 1.1799 m |
| `fr1/rpy` | No credible mono reference found | `pure_c_plus`: 0.0920 m |

RGB-D ORB-SLAM2 and RTAB-Map results are useful lower bounds, but they are not
apples-to-apples comparisons because they use depth.

Reference links:

- ORB-SLAM: https://ar5iv.labs.arxiv.org/html/1502.00956
- ORB-SLAM2: https://ar5iv.labs.arxiv.org/html/1610.06475
- ORB-SLAM3: https://ar5iv.labs.arxiv.org/html/2007.11898
- RTAB-Map 2024 update: https://arxiv.org/html/2403.06341v1

## Repository Hygiene

This README is the canonical project guide and session handoff. Generated
benchmark tables remain under `runs/benchmark/`. Keep one-off experiments in
suffixed files or dedicated folders such as `runs/pure_c_iter/`,
`runs/benchmark_history/`, or `runs/archive/`.
