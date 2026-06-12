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
| `test_freiburgdesk525` | `pure_c_plus` | **0.5716 m** | `python` | 0.6777 m |
| `test_freiburgroom525` | `pure_c_plus` | **1.1799 m** | `cpp` | 1.5452 m |
| `test_freiburgrpy525` | `pure_c_plus` | **0.0920 m** | `cpp` | 0.0977 m |
| `test_freiburgxyz525` | `cpp` | **0.1729 m** | `pure_c_plus` | 0.1763 m |

Full 4x7 ATE matrix:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | 0.6777 | 0.7194 | 0.7569 | 0.7569 | 0.7198 | 0.7570 | **0.5716** |
| `test_freiburgroom525` | 1.8660 | 1.5452 | 1.8689 | 1.8689 | 1.8518 | 1.8690 | **1.1799** |
| `test_freiburgrpy525` | 0.0983 | 0.0977 | 0.0998 | 0.0998 | 0.0992 | 0.0998 | **0.0920** |
| `test_freiburgxyz525` | 0.1785 | **0.1729** | 0.1777 | 0.1777 | 0.1782 | 0.1789 | 0.1763 |

Runtime matrix from the same sweep, in seconds:

| Sequence | `python` | `cpp` | `c` | `pure_c` | `pure_c_brief` | `pure_c_orb` | `pure_c_plus` |
|----------|----------|-------|-----|----------|-----------------|--------------|---------------|
| `test_freiburgdesk525` | 14.225 | 6.619 | 28.810 | 24.524 | 37.688 | 28.720 | 21.012 |
| `test_freiburgroom525` | 15.684 | 7.318 | 29.674 | 20.061 | 39.276 | 30.103 | 24.841 |
| `test_freiburgrpy525` | 15.722 | 7.606 | 23.317 | 12.398 | 52.009 | 31.585 | 24.782 |
| `test_freiburgxyz525` | 17.840 | 8.245 | 47.109 | 47.312 | 32.935 | 33.550 | 24.821 |

Mean ATE over the four GT datasets:

| Impl | Mean ATE RMSE | GT Wins | Runner-up |
|------|---------------|---------|-----------|
| `pure_c_plus` | **0.5050** | **3** | 1 |
| `cpp` | 0.6338 | 1 | 2 |
| `python` | 0.7051 | 0 | 1 |
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
another. Record rejected trials with numbers in `docs/rejected_trials.md`.

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

Everyday commands:

```bash
# Plot per-frame error against GT
python3 tools/plot_frame_errors.py \
  --gt test_freiburgxyz525.npz \
  --output runs/plots/diag.svg \
  pure_c_plus=runs/benchmark/test_freiburgxyz525_pure_c_plus_5s.json

# Inspect a saved run against GT
python3 tools/diagnose_trace.py \
  runs/benchmark/test_freiburgxyz525_pure_c_plus_5s.json \
  test_freiburgxyz525.npz \
  --top_k 20 --spikes --by_method
```

`pure_c_plus` traces include per-frame `method` (`0` init, `1` essential, `2`
PnP, `3` predicted fallback), tracked/linked counts, BRIEF relinks, points
added, PnP/E inlier detail, and raw inter-frame translation jump. Use
`--window START:END` to focus a frame range.

The full dump/replay harnesses — PnP dump + OpenCV/pure-C replay,
map-admission detail, map lifecycle, trace crossover / state-window /
health-rule sweeps, E-inlier comparison, and the C++ vs `pure_c_plus`
correspondence diff with its recorded findings — live in
`docs/diagnostics.md`.


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

### Birth-geometry finding (2026-06-12)

Fresh admission-detail dumps on the current profile pinned the bad-birth
mechanism. The E-inlier mask that gates admission certifies the *re-estimated
E geometry* (Sampson threshold 1e-4 normalized, ~5 px), but triangulation then
uses the *world pose pair* — a different geometry. Accepted-birth depth tracks
the world baseline almost exactly (corr 0.947 in log space), 82% of room
births triangulate across a baseline above 1e4 world units at ~90 px birth
reprojection, and the rare baseline<1 births are C++-quality (depth ~1.1,
0.56 px). Both room and desk show the same consecutive-frame world-baseline
explosion (~2 -> 1e5 within 200 frames, ~15%/frame compounding during
unstable stretches), so scale runaway alone does not separate the sequences.
Trace analysis further showed essential is the de-facto primary pose method
(579/750 room frames, E-fraction ~0.9 after frame 100) and E-method frames
carry the runaway (median raw step 162k vs 3.7k for PnP frames): the E
decomposition's unit-norm translation injects an uncalibrated world step at
every E frame, and once |t| is large, rotation jitter times |t| randomly
walks the centers. Two map-anchored scale calibrations were then rejected
(see the trials log): `--tri_map_scale` at the triangulation level and
`--e_step_map_scale` at the pose-composition level. Both improved room 8s
(~0.31 vs 0.34) and regressed both sequences at 30s, because a relative
anchor rides on the drifting map instead of stopping the drift. Conclusion:
relative (map-anchored) scale calibration is ruled out at both levels; a fix
for the scale feedback needs a signal that does not itself derive from the
drifting map/pose state — e.g. enforcing inter-frame translation-magnitude
continuity through the E handoffs, or making PnP the actual primary method so
E never free-runs for hundreds of consecutive frames.

Do not retry rejected surgical fixes without a materially new hypothesis: the
full trial-by-trial log (100 entries with numbers, run folders, and
conclusions) lives in `docs/rejected_trials.md`. The headline dead ends: LM
step/clamp/gate surgery in `refine_pose_lm`; hard depth, jump, reprojection,
and inlier admission gates in every tested form; candidate-track and
descriptor-primary admission variants; naive bounded or throttled global BA;
and short-window (5s/8s) wins that never survived the 30s sweep. Removed
implementations are recoverable from commit `8df5dc7` and earlier.

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
traffic than the 9-tap recompute saved.

LK template hoist (2026-06-12): the third bit-exact pass cut LK tracking from
5.19 s to 3.36 s on room 30s by sampling each point's template window
(gradients `Ix`/`Iy` and reference intensities) once per pyramid level instead
of on every solver iteration — those five of the six bilinear probes per
window sample depend only on the fixed template position, so only the moving
`cg`/`p_g` probe stays in the iteration loop. The skip-guard set and float
accumulation order are unchanged, verified by per-frame `xyz`/`raw_xyz`
byte-identity on all four 30s sequences. The originally proposed
shared-bilinear-weight variant was rejected during design: `(lx + l_dx) + x`
can round when the sum crosses a binade, so the 49 window samples do NOT
provably share one fractional offset, and per-sample weights were kept for the
remaining probe. Canonical `pure_c_plus` runtimes after this pass:
24.2/29.8/30.0/29.8 s for desk/room/rpy/xyz.

Hotspot pass (2026-06-12, second batch): four more bit-exact changes, again
verified by per-frame `xyz`/`raw_xyz` byte-identity on all four 30s sequences
plus a clean full sweep. Fine instrumentation first corrected the earlier
profiling note: the candidate qsort is only ~0.44 s of the ~5.5 s anchor
extraction — the real costs were the nonmax scan (~3.0 s) and the Jacobi
eigensolver (~5.9 s of PnP plus shares of essential/triangulate via
`svd_3x3`; ~87M rotations per room run). The changes: (1) nonmax suppression
is now a separable 7x7 sliding max — "no neighbor strictly greater" is exactly
"window max <= val", and max is comparison-only, so the boolean is unchanged;
(2) `jacobi_nxn` uses a two-pass pivot search (unrolled value-only max
reduction, then first row-major index holding that value — the same pair the
branchy argmax picked) and a stack buffer for n <= 12; (3) Harris scratch
buffers are persistent (they sit above the glibc mmap threshold, so per-call
malloc/free paid page faults every frame; reuse is safe because every cell
read is written first on each call); (4) PnP inlier counting bails once even
counting every remaining candidate cannot beat the best hypothesis (neither
the best-update nor the 0.8n break can change). Probes that did NOT help and
were dropped: `sincos` (glibc small-angle sin/cos are already cheap; atan2
dominates and is value-determining), removing `collapse(2)`, and `omp simd`
on the response loops (already at their memory/vector floor). Canonical
`pure_c_plus` runtimes after this pass: 21.0/24.8/24.8/24.8 s — and the
shared `jacobi_nxn` fix speeds up `c`/`pure_c`/`pure_c_brief`/`pure_c_orb`
for free (e.g. `pure_c` xyz 61.6 -> 47.3 s). Remaining room 30s hotspots:
Jacobi trig (~2.8 s, atan2-bound and value-determining), Harris response
(~2.3 s, fixed accumulation order), BRIEF anchor build (~1.8 s), and
`gray_blur` (~2.5 s, separable rewrite already tried and rejected).

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
tooling, update the relevant docs so the next worker does not have to re-derive
the conclusion. New diagnostic tooling goes in `docs/diagnostics.md`. Rejected
trials with numbers go in `docs/rejected_trials.md`. New findings and
hypotheses about `pure_c_plus` go under `## Active Pure-C Blocker`; a dated
subsection is fine when the finding does not yet fit a rejected-trial row.

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

This README is the canonical project guide and session handoff. Long-form
records live in `docs/`: `docs/rejected_trials.md` (the full rejected-trials
log) and `docs/diagnostics.md` (the dump/replay cookbook). Generated benchmark
tables remain under `runs/benchmark/`. Keep one-off experiments in suffixed
files or dedicated folders such as `runs/pure_c_iter/`,
`runs/benchmark_history/`, or `runs/archive/`.
