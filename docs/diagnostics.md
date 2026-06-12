# Diagnostics Cookbook

Full command recipes and recorded findings for the diagnostic dump/replay
tooling. `README.md` keeps only the short everyday commands; this file is the
reference for the deeper harnesses.

## Per-frame error and trace inspection

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
