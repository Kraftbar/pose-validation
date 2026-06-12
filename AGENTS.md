# AGENTS.md

Agent-facing rules for this repo. Read `README.md` first for the project guide:
implementations, benchmark commands, GT dataset list, current blockers, and
build notes. Long-form records live in `docs/`: `docs/rejected_trials.md`
(every rejected trial with numbers) and `docs/diagnostics.md` (dump/replay
cookbook).

## Source of Truth

Use these first when summarizing benchmark state:

- `runs/benchmark/gt_tracking.csv`
- `runs/benchmark/gt_tracking.md`
- `runs/benchmark/summary_all.json`
- `README.md`

## Benchmark Discipline

- **Hard rule:** any change to SLAM algorithm code (`simple_slam_c.c`,
  `pure_c_math.h`, `simple_slam_c_brief.c`, `simple_slam_c_orb.c`,
  `simple_slam_c_plus.c`, `simple_slam_opt.cpp`, `simple_slam.py`,
  `simple_slam_c_shim.cpp/.h`) or benchmark plumbing must be validated with:

  ```bash
  python3 benchmark_native.py --all_gt --force
  ```

- A single-GT run is allowed only for diagnosis and iteration. Do not claim an
  improvement from one dataset, short suffixed runs, truncated timeouts, or
  instrumented builds.
- Do not describe benchmark improvements in docs unless generated outputs from
  the full clean `--all_gt` run have been refreshed.
- For ad-hoc ATE checks, reuse `benchmark.ate_rmse`; do not reimplement Umeyama.

## Promotion Rule

- ATE RMSE is the primary accuracy metric.
- If a candidate is ATE-neutral across all GT sequences (all deltas within about
  `0.01 m`), do **not** promote it just because it improves map density,
  keyframe count, runtime, or code size.
- Watch for silent regressions on other sequences. A neutral ATE with a large
  map-density drop elsewhere is a signal the change trades one failure mode for
  another.
- Record rejected trials with numbers in `docs/rejected_trials.md`.

## Parallel Benchmarks

Parallel mode is acceptable for full sweeps:

```bash
python3 benchmark.py --all_gt --impl all --workers 4
```

Each worker gets `nproc / workers` OpenMP threads and the per-run timeout is
auto-bumped so OpenMP-heavy implementations still finish all frames.
Deterministic impls (`cpp`, `c`, `pure_c`, `pure_c_brief`, `pure_c_plus`)
reproduce canonical ATE within +/- `0.0002`; `python` and `pure_c_orb` retain
their natural run-to-run variance.

## Experiment Versioning

- Do not overwrite canonical benchmark summaries for one-off experiments unless
  the experiment is intended to become the new baseline.
- Save exploratory runs with explicit suffixes or dedicated folders, for example:
  `runs/benchmark/*_1s.json`, `runs/benchmark/*_5s.json`,
  `runs/benchmark_history/`, or `runs/pure_c_iter/`.
- If an old experiment is promoted into the active repo, give it a stable
  in-tree name and add it to the benchmark tables.
- Keep naming/versioning clear enough that later agents can tell canonical
  results from exploratory artifacts.

## Design Constraints

- Phase 1: close the ATE gap. `pure_c_plus` is the active architectural focus
  and current best pure-C variant by mean ATE. LOC is only a tiebreaker.
- Phase 2: once `pure_c_plus` is within about `0.02 m` mean ATE of `cpp`, reduce
  and simplify the implementation while preserving validated accuracy.
- Code style is "math on paper": matrices laid out as grids, one equation per
  line, no broad `clang-format` churn.
- Do not retry rejected `pure_c_plus` room fixes without a materially new
  hypothesis. README lists the current blocker; `docs/rejected_trials.md`
  lists the rejected attempts.
- The BRIEF loop-detector work is parked on branch `feat/brief-loop-detector`.

## Git Workflow

- Work on the current branch by default. Do not create a branch unless the user
  asks or there is a clear repo-specific reason.
- Do not commit or push unless the user explicitly asks.
- If a commit is requested, use the repo's existing Git identity. Do not set or
  override `git config user.name` / `user.email`.

## Important Notes

- `pure_c_brief` is the promoted BRIEF-relocalization snapshot kept in-tree.
- Do not assume `c` and `pure_c` are equivalent; `c` uses the OpenCV shim and is
  tracked separately.
- Do not assume only one GT dataset exists.
- If benchmark results change, update generated outputs first, then sync
  `README.md`.
