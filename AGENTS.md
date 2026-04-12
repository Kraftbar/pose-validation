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

## Source of Truth
Use these first when summarizing benchmark state:
- `runs/benchmark/gt_tracking.csv`
- `runs/benchmark/gt_tracking.md`
- `runs/benchmark/summary_all.json`
- `BENCHMARKS.md`

## Benchmark Discipline
- If you change SLAM behavior, tracking logic, optimization, pose estimation, or benchmark plumbing, run a benchmark before finishing.
- Start with the smallest useful benchmark first (for example a single GT dataset or a short `--seconds` run), then scale up if needed.
- For meaningful algorithm changes, refresh the all-GT benchmark with: `python3 benchmark_native.py --all_gt --force`
- Do not describe benchmark improvements in docs unless the saved outputs have been regenerated.

## Experiment Versioning
- Do not overwrite canonical benchmark summaries for one-off experiments unless the experiment is intended to become the new baseline.
- Save exploratory runs with explicit suffixes or dedicated folders, for example:
  - `runs/benchmark/*_1s.json`
  - `runs/benchmark/*_5s.json`
  - `runs/benchmark_history/`
  - `runs/pure_c_iter/`
- If you promote an old experiment into the active repo, give it a stable in-tree name (like `simple_slam_c_brief.c`) and add it to the benchmark tables.
- When changing benchmark-visible behavior, keep naming/versioning clear enough that a later agent can tell which results are canonical and which are exploratory.

## Design Philosophy
- **Pure C Focus:** The primary goal is a library-free C implementation. All architectural improvements should eventually be ported or implemented directly in `simple_slam_c.c`.
- **Simplicity & Separation of Concerns:** Maintain clean, modular code and clear architectural boundaries. Avoid over-engineering while pushing for industry-standard accuracy.
- **LOC is High Priority:** Small code size (LOC) is a first-class metric alongside ATE RMSE and runtime. Strive for the most "concise yet correct" implementation.
- **Benchmark Driven:** Accuracy (ATE RMSE) remains the primary goal. Improvements must be empirically verified against GT datasets.

## Important Notes
- `pure_c_brief` is the promoted BRIEF-relocalization snapshot kept in-tree.
- The old BRIEF branch was deleted after promotion.
- Do not assume `c` and `pure_c` are equivalent; `c` uses the OpenCV shim and is tracked separately.
- Do not assume only one GT dataset exists.
- If benchmark results change, update generated outputs first, then sync `BENCHMARKS.md` / `README.md`.
