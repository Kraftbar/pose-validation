# AGENTS.md

## Repo Purpose
This repo benchmarks multiple SLAM implementations across local monocular datasets and compares their accuracy, runtime, and code size.

## Implementations
- `python`: `simple_slam.py`
- `cpp`: `simple_slam_opt.cpp`
- `c`: OpenCV-linked `simple_slam_c.c` via `simple_slam_c_shim.cpp/.h`
- `pure_c`: standalone `simple_slam_c.c`
- `pure_c_brief`: standalone `simple_slam_c_brief.c`

## Canonical Benchmark Commands
- All GT-backed datasets: `python3 benchmark_native.py --all_gt --force`
- Single GT dataset: `python3 benchmark_native.py --all_gt --video test_freiburgxyz525 --force`

## GT Dataset Discovery
Benchmark discovery includes:
- top-level `test_*.mp4` files with adjacent `.npz`
- `external/twitchslam/videos/test_*.mp4` files with adjacent `.npz`

Current GT-backed datasets:
- `test_freiburgxyz525`
- `test_freiburgrpy525`
- `test_freiburgroom525`
- `test_freiburgdesk525`

## Diagnostic Tools
- `tools/diagnose_trace.py`: Analyzes SLAM metrics JSON against GT NPZ.
  Usage: `python3 tools/diagnose_trace.py runs/your_metrics.json external/twitchslam/videos/your_gt.npz`
  Surfaces per-frame translation/rotation error and tracking method.

## Source of Truth
Use these first when summarizing benchmark state:
- `runs/benchmark/gt_tracking.csv`
- `runs/benchmark/gt_tracking.md`
- `runs/benchmark/summary_all.json`
- `BENCHMARKS.md`

## Promotion Rule
- ATE RMSE is the primary accuracy metric. If a candidate change is ATE-neutral across all GT sequences (all deltas within ~0.01 m), do **not** promote it, even if it fixes a diagnosed sub-pathology (e.g. a map-growth plateau) or improves point/keyframe counts. Map density is a diagnostic, not a goal.
- Watch for silent regressions on sequences other than the one you diagnosed. A neutral ATE with a large map-density drop on another GT sequence is a signal the change trades one failure mode for another.
- Record rejected trials in `PURE_C_RECOVERY.md` with numbers, so future agents don't retry the same change.

## Benchmark Discipline
- **Hard rule:** any change to SLAM algorithm code (`simple_slam_c.c`, `pure_c_math.h`, `simple_slam_c_brief.c`, `simple_slam_opt.cpp`, `simple_slam.py`, `simple_slam_c_shim.cpp/.h`) or to benchmark plumbing must be validated with **all GT-backed datasets** before reporting results or committing. Run `python3 benchmark_native.py --all_gt --force`. This covers every `test_*.mp4` that has an adjacent `.npz` GT file (currently the four Freiburg sequences; new GT datasets added later are picked up automatically).
- A single-GT run (e.g. `benchmark.py --video test_freiburgxyz525 --seconds 30`) is allowed only for diagnosis and iteration while shaping a hypothesis. Do not use a single-GT result as the basis for a promotion decision or for claiming an improvement.
- Do not describe benchmark improvements in docs unless the saved outputs from `--all_gt` have been regenerated on a clean (non-instrumented) build.

## Experiment Versioning
- Do not overwrite canonical benchmark summaries for one-off experiments unless the experiment is intended to become the new baseline.
- Save exploratory runs with explicit suffixes or dedicated folders, for example:
  - `runs/benchmark/*_1s.json`
  - `runs/benchmark/*_5s.json`
  - `runs/benchmark_history/`
  - `runs/pure_c_iter/`
- If you promote an old experiment into the active repo, give it a stable in-tree name (like `simple_slam_c_brief.c`) and add it to the benchmark tables.
- When changing benchmark-visible behavior, keep naming/versioning clear enough that a later agent can tell which results are canonical and which are exploratory.

## Git Workflow
- Do not create a new branch by default. Work on the current branch unless the user explicitly asks for a branch or there is a clear repo-specific reason to isolate the work.
- Do not commit or push unless the user explicitly asks.
- If a commit is requested, use the repo's existing Git identity. Do not set or override `git config user.name` / `user.email` to an agent-specific name.

## Design Philosophy

### Phase 1 — Close the ATE gap (active)
- **Pure C Focus:** The primary goal is a library-free C implementation. The new `simple_slam_c_orb.c` provides a standard ORB-style pipeline (Scale Pyramid, FAST-9, ORB descriptors, Hamming distance, PnP, and Motion-only BA).
- **Simplicity & Separation of Concerns:** Maintain clean, modular code and clear architectural boundaries.
- **Benchmark Driven:** Accuracy (ATE RMSE) is the primary goal. `pure_c_orb` has achieved parity with other pure C implementations across all GT sequences.
- **LOC is a tiebreaker, not first-class.** Currently suspended while refining `pure_c_orb`.

### Phase 2 — Compress (gated)
- **Gate:** activate phase 2 once `pure_c_orb` has mean ATE within 0.02 m of `cpp` across the GT sequences.
- **LOC is High Priority:** Once gated in, golfing the implementation while maintaining accuracy is the next step.

## Important Notes
- `pure_c_brief` is the promoted BRIEF-relocalization snapshot kept in-tree.
- The old BRIEF branch was deleted after promotion.
- Do not assume `c` and `pure_c` are equivalent; `c` uses the OpenCV shim and is tracked separately.
- Do not assume only one GT dataset exists.
- If benchmark results change, update generated outputs first, then sync `BENCHMARKS.md` / `README.md`.
