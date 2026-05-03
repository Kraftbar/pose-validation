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
| `test_freiburgdesk525` | `python` | **0.6793 m** | `cpp` | 0.7194 m |
| `test_freiburgroom525` | `cpp` | **1.5452 m** | `pure_c_plus` | 1.7591 m |
| `test_freiburgrpy525` | `cpp` | **0.0977 m** | `python` | 0.0982 m |
| `test_freiburgxyz525` | `cpp` | **0.1729 m** | `pure_c_plus` | 0.1768 m |

Full 4x7 ATE matrix:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | **0.6793** | 0.7194 | 0.7569 | 0.7569 | 0.7198 | 0.7567 | 0.7307 |
| `test_freiburgroom525` | 1.8659 | **1.5452** | 1.8689 | 1.8689 | 1.8518 | 1.8673 | 1.7591 |
| `test_freiburgrpy525` | 0.0982 | **0.0977** | 0.0998 | 0.0998 | 0.0992 | 0.0996 | 0.0990 |
| `test_freiburgxyz525` | 0.1786 | **0.1729** | 0.1777 | 0.1777 | 0.1782 | 0.1790 | 0.1768 |

Mean ATE over the four GT datasets:

| Impl | Mean ATE RMSE | GT Wins | Runner-up |
|------|---------------|---------|-----------|
| `cpp` | **0.6338** | **3** | 1 |
| `pure_c_plus` | 0.6914 | 0 | 2 |
| `python` | 0.7055 | 1 | 1 |
| `pure_c_brief` | 0.7123 | 0 | 0 |
| `pure_c_orb` | 0.7257 | 0 | 0 |
| `c` | 0.7258 | 0 | 0 |
| `pure_c` | 0.7258 | 0 | 0 |

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

Parallel mode is acceptable for full sweeps:

```bash
python3 benchmark.py --all_gt --impl all --workers 4
```

Each worker gets `nproc / workers` OpenMP threads and the per-run timeout is
auto-bumped so OpenMP-heavy implementations still finish all frames.
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
Use `tools/compare_traces.py` for cross-implementation diffs. Initial room
diffs show `pure_c_plus` already beats `pure_c_orb` through most of frames
120:180, but trails `cpp` badly after roughly frame 129; `cpp` has about
2x-3x as many map points in that window, so the next comparison should inspect
why C++ builds/keeps that early map structure instead of trying more ORB
threshold tweaks blindly.

### C++ vs `pure_c_plus` Correspondence Diff (2026-04-30)

`simple_slam_opt` (the reference C++ pipeline) now also accepts `--pnp_dump`,
plus `--no_downscale` and `--fx/--fy/--cx/--cy` so its dump can be produced at
the same intrinsics as `pure_c_plus`. These flags are diagnostic-only; default
behavior is unchanged, so the benchmark hard rule does not require a fresh
`--all_gt` sweep for this addition.

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
| `pure_c_plus` feature-budget bump | Raising corner cap/replenish to 3000/1800 worsened 8s room to 0.6363 with 12265 points; 1500/900 worsened to 0.6520. |
| Extra pose LM iterations (`--pose_lm_iters 15`) | Strong 5s ATE wins on tuned desk/room windows (desk 0.2378, room 0.1159), but not a full-run fix: 30s desk stayed at 0.7538 and 30s room regressed to 1.8670. Kept as an opt-in diagnostic knob, not a default/profile. |
| Essential RANSAC budget sweep (`--essential_iters 250/750`) | Did not beat the 500-iteration default around the `pose_lm_iters 15` short-run wins; lower budget collapsed desk/room and higher budget was still worse than default. |
| Warmup-then-bounded keyframes (`--kf_warmup_frames 60 --kf_period 30 --kf_max_rot_deg 180`) | Reduced 5s keyframes roughly in half after warmup, but starved map growth and worsened desk/room 5s ATE. Kept as a cadence diagnostic only. |
| First-keyframe observation retention (`--first_kf_observations`) | Stores the previous frame as the first real KF and initializes new two-view points with `obs=2`; desk 5s was unchanged and room 5s worsened in both BA/no-BA tuned paths. Not a default. |
| Unique keyframe observation counting (`--unique_kf_observations`) | Guards `map.obs++` so one keyframe can only increment a map point once. Default desk/room 5s was unchanged, and fast-320/no-BA room stayed at 0.1694. Useful diagnostic, not an ATE lever so far. |
| Essential cheirality over all matches (`--essential_cheirality_max 0`) | Evaluates all matches instead of the first 32 for E pose disambiguation. Desk 5s moved only 0.4039 -> 0.4005, while room worsened 0.1578 -> 0.1666. Default remains 32. |
| Predicted-pose PnP reprojection gate (`--pnp_pred_reproj_gate 16`) | Improved fast-room 5s to 0.1430, but full room 30s regressed to 1.8613 and map growth exploded to 95144 points (60577 with max600). Another short-window trap. |
| Recent-keyframe PnP quality mask (`--pnp_quality_gate_px 8/16`) | Non-destructive mask over recent KF reprojection evidence before PnP. Fast-room 5s stayed around 0.170 and desk 5s worsened to 0.4026, so the simple “all recent obs bad” rule did not pick the right correspondences. |
| Replay-derived numeric P3P scorer (`--pnp_p3p_max_mederr 120`) | Offline replay on the saved room 8s dump predicted a 0.0409 RMSE win by replacing pure-failed frames 166-170, and full-room replay predicted only a 0.018 RMSE win. Live feedback did not transfer: default room 8s worsened to 0.5862 unrestricted and 0.6141 with jump caps 5000/20000; fast room 8s worsened to 0.6516. |
| Delayed landmark promotion (`--new_point_obs 0`, `--pnp_min_obs 1/2`) | Quarantining fresh triangulations from PnP did not help room 8s: default path worsened to 0.6383 (`pnp_min_obs=1`) and 0.6232 (`pnp_min_obs=2`); fast 320/no-BA worsened to 0.6491. |
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
| Descriptor-primary map admission (`--descriptor_primary_admission`) | C++-style opt-in variant that makes descriptor E-inliers the only source of new landmarks, forces descriptor-relative pose for triangulation, and bounds additions with `--descriptor_primary_map_cap` plus optional `--admission_max_new_points`. It still did not transfer C++ quality: room 8s regressed to 0.6335 with 4648 points, and anchor + primary regressed to 0.6504. The admission dump improved over the old descriptor add-on but remained far from C++: 4648 accepted points over 189 rows, 84 rows with infinite mean reprojection, and finite rows averaging ~66 px reprojection. Adding reprojection/depth gates starved the map and still regressed (`tri_max_reproj_px=4`: 223 points, 0.6487; `16`: 747 points, 0.6446). Older descriptor-source keyframes (`--descriptor_source_kf_gap 5/10`) reduced map growth and improved finite reprojection to ~38 px at gap10, but still regressed room 8s (gap5 0.6357, gap10 0.6429 after making source-gap mode wait for a qualifying KF instead of falling back to adjacent frames). Matcher-quality filters also failed: mutual BRIEF (`--descriptor_mutual_admission`) regressed primary to 0.6474 and source-gap5 to 0.6446, while stricter ratio 0.6 regressed primary to 0.6441 and source-gap5 to 0.6394. This means the remaining gap is not just "use descriptors for admission", "triangulate from older keyframes", or one-way ratio ambiguity; pure C still needs better source-pose/keyframe geometry or a stronger descriptor family before triangulation. Runs: `runs/descriptor_primary/`. |
| Oriented BRIEF (`--oriented_brief`) | ORB-adjacent pure-C probe that steers the BRIEF sampling pattern by an intensity-centroid orientation. It did not help descriptor-primary alone (room 8s 0.6341) or anchor-only (0.6312), but it did improve the best short-window anchor cadence path: `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief` reached room 8s ATE 0.5254 vs 0.5845 without orientation and 0.6141 baseline. The full 30s gate still failed on accuracy despite completing all 750 frames: 1.8246 ATE with 32230 points and 682 KFs. This is the strongest ORB-like signal so far, but it still needs a long-run map/keyframe health policy. Runs: `runs/oriented_brief/`. |
| FAST-style corners (`--fast_corners`) | ORB-adjacent detector probe using a FAST-9 circle test with score-ranked nonmax selection. It did not transfer the ORB frontend advantage as implemented: with the best anchor cadence, FAST + oriented BRIEF regressed room 8s to 0.6208 and FAST without orientation regressed to 0.6284; descriptor-primary + FAST + oriented BRIEF regressed to 0.6503. This suggests the simple FAST selector is starving/reshaping features rather than reproducing OpenCV ORB's pyramid/distribution behavior. The useful ORB-like signal remains descriptor orientation, not this first detector approximation. Runs: `runs/fast_corners/`. |
| Pyramid/distributed features (`--pyramid_features`, `--distributed_features`) | Adds opt-in ORB-like feature population shaping without new tuning knobs: Harris candidates can be selected through a fixed 8x6 grid, and `--pyramid_features` adds a second half-resolution Harris level before grid selection. Flat grid selection alone regressed the best oriented-anchor room 8s path to 0.6362, and FAST+grid regressed to 0.6184. The two-level Harris pyramid is the first frontend population win: `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief --pyramid_features` reached room 8s ATE 0.4806 vs the prior 0.5254 best, with fewer points/KFs (4917/80 at 8s). The 30s gate still failed: pyramid alone reached 1.8514 ATE with 20292 points and 581 KFs over 743 frames; adding `--healthy_keyframes --map_hygiene` completed all 750 frames and improved to 1.7022 with 22336 points and 600 KFs, still not stable enough to promote. Runs: `runs/pyramid_features/`. |
| Late keyframe cooldown (`--late_kf_cooldown`) | Applies the existing keyframe minimum-interval idea only after frame 200, preserving the early dense-map behavior that made `--pyramid_features` work. Full `--kf_min_interval 2` improved room 30s to 1.6362 with 15791 points and 324 KFs, but sacrificed the 8s win (0.5135 vs 0.4806); interval 3 over-starved the 8s path (0.6471). `--late_kf_cooldown` keeps room 8s exactly at 0.4806 while improving room 30s over pyramid alone to 1.6895 with 16992 points and 344 KFs. Pairing it with `--map_hygiene` worsened to 1.7306, and the earlier combined health gates also worsened. This is a useful stability tradeoff, not a complete long-run fix. Runs: `runs/pyramid_features/`. |
| Admission finite-only sanity (`--admission_finite_only`) | Opt-in non-threshold sanity check that rejects newly triangulated landmarks only when reprojection/depth/XYZ statistics are non-finite. It directly targets the `inf` admission rows seen in `runs/pyramid_features/room_8s_best_admission.csv`, but it still regressed the best pyramid+late-cooldown room 8s path to 0.6183. This means the pathological rows are a symptom of the admission/pose state, not a standalone fixable class; a viable policy needs batch-level ranking/deferment rather than a finite-only filter. |
| Batch-ranked LK admission (`--admission_batch_ranked`) | Buffers LK triangulation candidates for a keyframe, ranks them by reprojection error, and commits a batch capped by current pose inlier support instead of pushing landmarks immediately. A cap of 1x and 2x inlier support starved room 8s (0.6406 and 0.6109). The current internal 4x-inlier support cap is the first admission-policy win: with `--anchor_e_pose --kf_period 20 --kf_max_rot_deg 45 --oriented_brief --pyramid_features --late_kf_cooldown --admission_batch_ranked`, room 8s improved to 0.3619 with 4628 points and 94 KFs. The 30s gate still regressed to 1.8313 with 15229 points and 362 KFs, and pairing batch admission with full `--kf_min_interval 2` regressed room 8s to 0.6164. This validates batch admission as a short-window lever, but long-run stability still needs better temporal/deferment logic. Runs: `runs/pyramid_features/`. |
| Deferred batch admission (`--admission_batch_deferred`) | Reuses the batch-ranked LK selector but stores selected landmarks as tentative candidates first, promoting them only after the existing multi-observation candidate readiness check. This was too sparse for the current pose/map loop: default candidate survival regressed room 8s to 0.6258 with 2429 points, and a shorter survival window (`--candidate_min_age 2 --candidate_min_obs 3`) regressed to 0.6430. The current system still needs immediate enough geometry for PnP; simple deferment starves it before improving long-run health. |
| Healthy keyframes (`--healthy_keyframes`) | Centralizes keyframe decisions in `should_make_keyframe` and, after the first 200 frames, suppresses low-inlier fallback KFs unless the frame has enough linked map support. Applying it from frame 0 destroyed the oriented-anchor 8s win (0.6425). Starting the policy after frame 200 preserved the 8s win exactly (0.5254) but did not fix room 30s: point/KF growth fell only modestly (32230/682 -> 30009/676) and ATE worsened to 1.8532. A real long-run fix needs a stronger keyframe/map health signal than linked-count gating. Runs: `runs/healthy_keyframes/`. |
| Map hygiene (`--map_hygiene`) | Reuses the existing recent-window reprojection culler outside joint BA. Applying it immediately broke the oriented-anchor 8s win (0.6273). Starting it after frame 200 preserved the 8s win exactly (0.5254), but room 30s still failed and worsened to 1.8627 with 31070 points and 687 KFs. Marking all-bad recent-window observations as unusable is not enough once the long-run trajectory/map state has drifted. Runs: `runs/map_hygiene/`. |
| Delayed initialization (`--delayed_init_frames`) | Opt-in map-start delay. With descriptor-primary admission it suppresses early landmarks, stores keyframes, then initializes from the first stored keyframe once the delay is reached. This produced a very sparse map and regressed room 8s (`20`: 340 points, 0.6430; `40`: 334 points, 0.6451). Letting default LK admission start after the delay was also not a win (`20`: 5474 points, 0.6160; `40`: 4150 points, 0.6510). This rules out a simple "wait, then initialize from the first KF" fix; a viable delayed init would need actual multi-frame track survival / bundle initialization, not just a delayed two-view triangulation. Runs: `runs/delayed_init/`. |
| E-constrained triangulation pose (`--triangulate_with_e_pose`, `--triangulate_relative_frame`) | Tests replacing the PnP/LM world pose used for map admission with the current E relative pose. `--triangulate_with_e_pose` improved admission reprojection from ~415 px to ~61 px but admitted 9646 points and slightly worsened room 8s to 0.6166; descriptor + E pose worsened to 0.6389. `--triangulate_relative_frame` triangulates in the previous-camera local frame before transforming into world, fixing the dump baseline to 1.0 but still leaving ~63 px reprojection and worsening room 8s to 0.6383; descriptor + relative frame worsened to 0.6477. This narrows the remaining issue: pure-C E/LK correspondences themselves are not producing C++-quality two-view geometry (C++ is ~1.7 px), independent of world-scale pose corruption. Runs: `runs/triangulate_pose/`. |
| E-inlier correspondence dump (`--e_inlier_dump`) | Per-E-inlier CSV for pair coordinates, displacement, match score, linked map point, LK FB error, and 8x6 grid cell. On room 8s, pure C E inliers came from only 40 frames but averaged 137.7 inliers/frame with mean displacement 15.65 px (p90 27.9); C++ pose E inliers came from 122 frames with 42.5/frame and mean displacement 7.69 px (p90 13.4), and C++ triangulation inliers were similar at 48.1/frame and 7.64 px. Pure C is not lacking raw E inliers; it gets bursty, longer-displacement LK inliers, while C++ has steadier descriptor correspondences. Next promising lever is an LK/E admission policy that matches C++'s displacement/temporal profile (cap or rank E inliers by displacement/FB/grid before pose/admission), not another triangulation-pose variant. Dumps: `runs/e_inlier_compare/pure_room_8s/e_inliers.csv`, `runs/e_inlier_compare/cpp_room_8s/e_inliers.csv`. |
| LK/E inlier shaping (`--shape_e_inliers`) | Ranks/caps pure-C LK matches before E by displacement target, FB error, and 8x6 grid count. A strict C++-like profile (`max_disp=14`, `grid_cap=1`, `max=64`) produced C++-like displacement (mean 8.67 px, p90 12.5) but starved E to 12.6 inliers/frame and only 223 room-8s points. The best 8s profile found (`--e_shape_max_disp 30 --e_shape_target_disp 8 --e_shape_grid_cap 3 --e_shape_max_matches 144 --e_shape_max_fb_err 0.5`) improved room 8s slightly, 0.6141 -> 0.6047, and spread E across 196 frames with 22.1 inliers/frame, but room 30s was 1.7979 with only 7820 points, so this is another short-window diagnostic rather than a default. It confirms the direction is correspondence population shaping, but simple pre-E caps still starve long-run geometry. Runs: `runs/e_shape/`. |
| Descriptor anchor E pose (`--anchor_e_pose`) | Maintains a BRIEF descriptor anchor set from keyframes, matches it into independent current corners, and uses that E pose when PnP fails while keeping LK admission separate. Fixing LK admission after anchor pose avoided the initial map starvation. Default anchor cap 600 gave a small room-8s improvement (0.6141 -> 0.6117) with 8581 points, but room 30s did not complete even with a 300s timeout (654/750 frames, 29788 points, ATE 1.7147). Lowering anchors to 200 reduced map growth but regressed room 8s to 0.6250. Bounded admission (`--admission_max_new_points 160 --admission_grid_cap 8`) reduced room-8s map growth to 5834 points but regressed to 0.6310; ranked bounded admission (`--admission_ranked`, target 8/12 px) regressed further to 0.6491/0.6442. The best short-window anchor variant so far is keyframe cadence (`--kf_period 20 --kf_max_rot_deg 45`), improving room 8s to 0.5845 with 7610 points, but room 30s still only reached 704/750 frames under the 300s wrapper and had ATE 1.7744 with 27648 points. A direct keyframe cooldown (`--kf_min_interval 2/3`) cut room-8s KFs/points but regressed to 0.6408/0.6415, and combining cadence with moderate admission caps (`240/g10`, `320/g12`) also regressed to 0.6505/0.6446. Anchor E remains useful architecturally, but throttling volume after the fact is not enough; this branch still lacks a 30s-stable admission/keyframe quality policy. Runs: `runs/anchor_e/`. |
| Room full-run fast profile sweeps (`pnp_dlt_iters=100/200`, LK 7/3) | All worsened the current fast-320/no-BA room 30s result: PnP100 1.8334, PnP200 1.8265, LK7/3 1.8139 vs existing PnP150 1.6861. More points/inliers again correlated with worse ATE. |
| BA every 5 frames with global BA off | Room 30s moved to 1.7793 and desk 30s to 0.7563; near existing behavior, not a full-run win. |
| Joint Schur-complement local BA + map-point culling (`--joint_ba`) | Mathematically correct (5-KF window, 1 fixed gauge KF, χ²(2,0.95)=5.991 Huber, LM with diagonal damping and step rejection, Schur-eliminated points; cull at 8 px reproj. when ALL window obs are bad). Full sweep regressed every sequence: desk +0.034, room +0.039, rpy +0.018, xyz neutral (`runs/joint_ba_sweep/`). Pose-only joint BA (skip dP back-sub) on 8s room also regressed (0.635 vs 0.545). Code retained behind `--joint_ba` flag (`joint_local_ba`, `cull_map_points_window` in `simple_slam_c_plus.c`); useful baseline for future work that pairs BA with proper keyframe selection / initialization. Default behavior unchanged. |

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

The joint-BA trial closes one bigger door: a mathematically correct
Schur-complement local BA with Huber kernel, LM damping, and step rejection
*by itself* is not sufficient to move pure_c_plus toward ORB-SLAM accuracy on
the current pipeline; it regressed every GT sequence. This is consistent with
the "fragile fixed point" pattern, but the negative result is more informative
than the prior PnP/LM tweaks because joint BA is the textbook fix for
"reprojection RMSE > 100 px" map points that the C++/`pure_c_plus`
correspondence diff identified. The most likely remaining culprits, in
priority order, are: (a) initialization / first-keyframe geometry (joint BA
inherits the gauge from the very first KF, which is fixed in `local_ba`'s
window only after it has already accumulated bad triangulations); (b)
keyframe selection that adds KFs faster than the map is healthy; and (c) the
PnP candidate generator (still the dominant pose source on most frames). Pair
joint BA with one of (a)/(b)/(c) before re-running the sweep — joint BA alone
is documented to regress.

## Design Direction

Phase 1 is accuracy and robustness. `pure_c_plus` should close the mean ATE gap
to `cpp`; LOC is only a tiebreaker during this phase.

Phase 2 starts once `pure_c_plus` is within about `0.02 m` mean ATE of `cpp`.
At that point, reduce and simplify the implementation while preserving the
validated GT numbers.

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

This README is the canonical project guide and session handoff. Generated
benchmark tables remain under `runs/benchmark/`. Keep one-off experiments in
suffixed files or dedicated folders such as `runs/pure_c_iter/`,
`runs/benchmark_history/`, or `runs/archive/`.
