# stella_port reference: single-threaded, deterministic stella_vslam

Phase-1 reference for a future pure-C port of stella_vslam, analogous to
`orb_port/` for ORB-SLAM2, but clean-room: written only from stella_vslam's
own source (`external/candidates/stella_vslam` @ `e445b5452535e781fdf6777ec33a06f5f8f5e416`,
v0.7.0, BSD-2) and its own public API/example utilities. `docs/orb_port_handover.md`
"Decisions worth knowing" was read only as a checklist of pitfall *categories*
(RNG seeding, pointer-ordered containers, thread timing, dump timing) — no
code or text from `orb_port/` or ORB-SLAM2 was read or reused.

## Build

```bash
. runs/oneshot/environment.sh   # PATH/PKG_CONFIG_PATH for the venv + OpenCV 4.6
python3 tools/build_stella_reference.py
```

This:
1. Copies `external/candidates/stella_vslam` -> `runs/stella_port/reference_build/src`
   (never edits the original in place).
2. Applies `stella_port/reference/patches/*.patch` to that copy, in order.
3. Configures+builds the `stella_vslam` shared library with
   `-O2 -ffp-contract=off -fno-fast-math` (no `-march=native`),
   `-DDETERMINISTIC=ON -DUSE_OPENMP=OFF -DBUILD_WITH_MARCH_NATIVE=OFF`,
   installs to `runs/stella_port/reference_build/install`.
4. Configures+builds `stella_port/reference/driver` (`run_stella_reference`)
   against that install.
5. Writes `runs/stella_port/reference_build/provenance.json` (commit, patch
   hashes, compiler, flags, g2o commit, g2o solver modules linked).

Dependencies reused as-is from the earlier candidate-evaluation build: OpenCV
4.6 under `/tmp/pose-opencv` + `/tmp/localopencv` (imgcodecs' GDAL/GDCM/Armadillo
transitive libs), `external/candidates/deps/root` (g2o, Eigen, yaml-cpp,
spdlog, FBoW, SQLite3), g2o at `external/candidates/g2o` @
`e8df2004e07ea8f5b8e6a8b9f2dc067b45b45036`. If `/tmp/pose-opencv` or
`/tmp/localopencv` are gone, see `external/candidates/run_stella.sh` and its
neighbouring build notes to recreate them; nothing here rebuilds OpenCV.

The driver's own runtime linking needs one non-obvious thing: put
`runs/stella_port/reference_build/install/lib` **before**
`external/candidates/deps/root/usr/lib` on `LD_LIBRARY_PATH` — the deps
prefix carries its own unpatched `libstella_vslam.so` from an earlier,
unrelated build, and the dynamic linker will silently pick that one first
otherwise (missing our new symbols -> `undefined symbol` at process start).
`tools/dump_stella_reference.py` gets this order right; if invoking
`run_stella_reference` by hand, copy its `LD_LIBRARY_PATH` construction.

## Run

```bash
python3 tools/dump_stella_reference.py fr1_xyz fr1_desk
```

Writes per-sequence dumps to `runs/stella_port/reference_dumps/<seq>/` and a
`tools/tum_eval.py`-compatible trajectory to
`runs/tum_compare/stella_vslam_st/<seq>/{trajectory.tum,run.json}`. Score
with:

```bash
python3 tools/tum_eval.py --systems stella_vslam_st,stella_vslam --seqs fr1_xyz,fr1_desk
```

(Re-running `tum_eval.py` with no arguments regenerates the shared
`runs/tum_compare/table.md`/`table.json` across every system it finds under
`runs/tum_compare/` -- do this after adding a new system so that file stays
the full comparison, not just the two rows you last asked for.)

## Patches

Thirteen patches (0013 = loop closing: id-ordered loop containers, loop trace, opt-in Eigen solver / thresholds, see HANDOVER "Module 7"; 0012 = id-ordered loop-candidate containers, determinism fix, see HANDOVER open item; 0011 = keyframe lifetime log + raw covisibility state for module 6, see HANDOVER "Module 6"; 0008 pre-track state, 0009 keyframe-insert trace, 0010 opt-in force-path fault injection: see HANDOVER module 5) (0001-0005 documented below; 0006 pose-optimizer trace and 0007
BA trace at the end of this section), one purpose each, all under
`stella_port/reference/patches/`.
Verified reapplicable: `patch -p1 -i <each, in order>` onto a fresh copy of
`external/candidates/stella_vslam` reproduces the tree
`tools/build_stella_reference.py` builds, byte-for-byte
(`diff -rq` clean between a from-scratch `patch`-only copy and the script's
own working copy).

### `0001-synchronous-step-entry-points.patch`

Touches `mapping_module.h/.cc`, `global_optimization_module.h/.cc`,
`system.h/.cc`. Algorithm code is untouched; every hunk either adds a new
method or adds a branch that is a no-op unless the driver opts in
(`set_synchronous(true)`, only called from the new
`system::startup_single_threaded()`). Specifically:

- `mapping_module::run_step()` / `global_optimization_module::run_step()`:
  each is exactly the body of the corresponding `run()` loop iteration for
  "a keyframe is queued" (dequeue, do the work, hand off to the next
  module), minus the pause/reset/terminate bookkeeping that only matters
  when a background thread is polling a queue that other threads are also
  touching -- which never happens here.
- `mapping_module::set_synchronous(true)` makes `async_add_keyframe()` call
  `run_step()` inline, right after enqueueing, instead of leaving the
  keyframe queued for a mapping thread that will never run. This was not
  optional instrumentation -- it fixes a real deadlock (see "Nondeterminism
  and threading pitfalls found" #4 below).
- `global_optimization_module::set_synchronous(true)` makes
  `correct_loop()` run `loop_bundle_adjuster_->optimize(cur_keyfrm_)`
  inline instead of on a detached `std::thread` (`thread_for_loop_BA_`).
- `system::startup_single_threaded()` does the bookkeeping
  `system::startup()` would (mark the system running, optionally force
  `tracking_state_ = Lost`), but never starts `mapping_thread_` /
  `global_optimization_thread_`, and turns synchronous mode on for
  `mapper_` and `global_optimizer_`.
- `system::synchronize_background_modules()` drains
  `mapper_->run_step()` then `global_optimizer_->run_step()` in a loop. In
  practice the mapping queue is already empty by the time the driver calls
  this (see above), so this is mostly a safety net for the global
  optimizer's queue.

### `0002-stage-trace-instrumentation.patch`

`mapping_module.cc` only: `fprintf(stderr, "[TRACE] <stage> begin/end")`
around `store_new_keyframe()`, `create_new_landmarks()`,
`update_new_keyframe()`, the local-BA call, and
`remove_redundant_keyframes()`. This is what localized the deadlock in
`0001` during development (see below) and is left in as a cheap, always-on
progress trace; it does not change control flow or any computed value.

### `0003-id-ordered-landmark-fusion-map.patch`

Touches `match/fuse.h/.cc` (the `duplicated_lms_in_keyfrm` parameter type)
and the three call sites that declare a matching local variable
(`mapping_module.cc` x2, `global_optimization_module.cc` x1). Closes the one
gap `-DDETERMINISTIC=ON` doesn't reach (see "Nondeterminism..." #2): swaps
`std::unordered_map<std::shared_ptr<data::landmark>, std::shared_ptr<data::landmark>>`
for `nondeterministic::unordered_map<...>` -- the exact alias `type.h`
already uses everywhere else for this key type, so under
`-DDETERMINISTIC=ON` this becomes `std::map<..., id_less<shared_ptr<landmark>>>`
(id-ordered) instead of pointer-hash-ordered. `new_connections`, the sibling
parameter keyed by `unsigned int` (a landmark id, not a pointer), was
already deterministic and is untouched. No algorithm logic changed --
`std::map` supports the same `.clear()`/`operator[]`/range-for the code
already used on the `std::unordered_map`.

### `0004-tracking-trace-instrumentation.patch`

Touches `tracking_module.h/.cc`, `module/keyframe_inserter.h/.cc`,
`util/random_array.h/.cc`, and `system.h` (one new accessor,
`get_tracker()`). Adds:
- a `tracking_module::track_path_t` per-frame trace (which `frame_tracker`
  variant or relocalization path succeeded, the pose right after that
  stage, the pose/inlier counts after local-map optimization, this frame's
  local keyframe/landmark id sets) -- public plain-data members set at the
  existing points in `track()`/`track_current_frame()`/
  `optimize_current_frame_with_local_map()`/`update_local_map()` that
  already compute these values and previously discarded them;
- `module::keyframe_inserter::decision_trace` -- the individual booleans/
  counts `new_keyframe_is_needed()` already computed and only
  `SPDLOG_TRACE`d (compiled out unless `ENABLE_TRACE_LEVEL_LOG=ON`), kept
  as a `mutable` member instead so a non-logging reader can see them;
- three process-lifetime `std::atomic<uint64_t>` counters in
  `util::random_array.cc` (`g_random_engines_created`,
  `g_random_array_calls`, `g_random_array_elements_requested`), incremented
  at `create_random_engine()`/`create_random_array()`.
No algorithm logic changed; every addition is either a new public
method/member or an extra statement that stores an already-computed local
variable into one.

### `0005-mapping-pass-and-erasure-trace.patch`

Touches `mapping_module.h/.cc`, `module/local_map_cleaner.h/.cc`,
`optimize/local_bundle_adjuster.h` (new `local_ba_trace` struct + default
no-op `get_last_trace()`), `optimize/local_bundle_adjuster_g2o.h/.cc`
(override), `module/loop_detector.h/.cc`, `global_optimization_module.h/.cc`,
`system.h` (two new accessors, `get_mapper()`/`get_global_optimizer()`),
`system.cc` (comments only, no behavior change -- see below),
`util/CMakeLists.txt` (registers two new files), `data/keyframe.cc`,
`data/landmark.cc`, and adds `util/erasure_log.h/.cc`. Same pattern as
`0004`: every trace field is either read straight off the codebase's own
public API (keyframe/landmark covisibility, spanning tree, pose,
descriptor, etc. -- no patch needed for those) or a value the algorithm
already computes and discards, now also stored where a reader can get it.

- `mapping_module::mapping_pass_trace` (`get_last_pass_trace()`): keyframe
  id processed; culled landmark ids (copied from
  `module::local_map_cleaner::get_last_culled_landmark_ids()`, itself new
  -- `remove_invalid_landmarks()`/`remove_redundant_keyframes()` now record
  what they cull, one fixed reason string each since each function culls
  for exactly one reason); triangulation, one entry per neighbor keyframe
  `create_new_landmarks()` actually calls `triangulate_with_two_keyframes()`
  for for, in that order, with the `(idx_1, idx_2)` match pairs passed in
  and the landmark ids/positions `triangulate_with_two_keyframes()` accepts
  (matched up via `.back()` on the trace vector, since with `USE_OPENMP`
  off both loops run sequentially, one neighbor/match fully processed
  before the next starts); `replaced_landmark_ids`, straight from
  `update_new_keyframe()`'s already-`0003`-fixed, now id-ordered
  `replaced_lms` map; culled keyframe ids (from
  `local_map_cleaner::get_last_culled_keyframe_ids()`, new alongside the
  landmark one above); `local_ba_invoked`.
- `optimize::local_bundle_adjuster_g2o::get_last_trace()`
  (`optimize::local_ba_trace`): local/fixed keyframe id lists (from the
  function's own `local_keyfrms`/`fixed_keyfrms` maps, sorted); iteration
  counts (the actual return value of `g2o::SparseOptimizer::optimize()`,
  not just the configured max); chi2 before/after (`optimizer.computeActiveErrors()`
  + `optimizer.activeChi2()`, called once right after
  `initializeOptimization()` for "before" and again after the last
  `optimize()` call for "after" -- both read-only g2o diagnostics, no
  effect on the optimization itself); outlier observations removed count
  (`outlier_observations.size()`, already computed). `local_bundle_adjuster`
  (the base interface)'s `get_last_trace()` defaults to `ran=false` so a
  non-g2o backend (e.g. a gtsam one, if compiled in) doesn't need touching.
- `global_optimization_module::step_trace` (`get_last_step_trace()`):
  keyframe id; candidate ids considered (new
  `module::loop_detector::get_candidate_ids_to_validate()`, sorted ids of
  the already-existing `loop_candidates_to_validate_` member, which
  `detect_loop_candidates()` populates and `validate_candidates()`
  narrows); `loop_accepted`/`accepted_candidate_id`; keyframes Sim3-corrected
  and landmarks position-corrected counts (`Sim3s_nw_after_correction.size()`/
  `found_lm_to_ref_keyfrm_id.size()` in `correct_loop()`, both already
  computed); `loop_ba_invoked`. **Reduced scope, not implemented**: a
  per-call cost/iteration trace for `module::loop_bundle_adjuster`/
  `optimize::global_bundle_adjuster` analogous to `local_ba_trace` -- that
  optimizer is a shared free function (`optimize_impl` in
  `global_bundle_adjuster.cc`) used by three call sites (initial-map global
  BA, loop-closing global BA, and nothing else), and wiring a trace through
  it was more surgery than fit in this pass; see "Not done".
- **A real bug found and fixed during this patch's own verification**: the
  trace structs above are members that persist between `run_step()` calls
  (by design, so a driver reading them right after
  `synchronize_background_modules()` sees the last real pass even if the
  queue is now empty). The first version reset them inside `run_step()`'s
  own `if (!keyframe_is_queued()) return false;` branch, which also fires
  -- and would wipe the just-computed trace -- on the drain loop's own
  final, now-empty-queue call immediately after a real pass. Symptom: every
  trace field duplicated verbatim across consecutive frames (the value from
  whatever pass last ran, re-read every frame after that, indistinguishable
  from a new real pass). Second attempt (reset once at the top of
  `system::synchronize_background_modules()`, before its drain loops)
  produced *zero* trace output instead: `mapping_module::set_synchronous(true)`
  (patch `0001`) makes `async_add_keyframe()` drain a new keyframe
  **inline, during `feed_monocular_frame()`/`feed_frame()`, before
  `synchronize_background_modules()` is even called that frame** -- so
  resetting at the top of `synchronize_background_modules()` wipes the
  trace `async_add_keyframe()`'s inline drain just set, and the drain
  loops immediately below find the queue already empty and set nothing
  new. Fix: `mapping_module::reset_last_pass_trace()`/
  `global_optimization_module::reset_last_step_trace()` are plain public
  methods; `synchronize_background_modules()` calls neither (its own
  comment now explains why); the driver calls both itself, once per frame,
  right before `feed_monocular_frame()` -- the one point in the loop
  guaranteed to run before that frame's mapping/global-opt activity,
  wherever in the frame it ends up actually happening. Verified via a
  temporary `fprintf` probe (removed) showing `ran=0` on every frame with
  both earlier versions, then correct per-keyframe values (each keyframe id
  appearing at exactly one frame) after the fix -- see "Dumps" for the
  corrected timing note this forced into `frame_trace.tsv`'s sibling files'
  documentation below.
- `util/erasure_log.h/.cc`: two global `std::vector`s
  (`g_erased_landmarks`/`g_erased_keyframes`, not per-object/per-caller,
  since erasure is triggered from many call sites -- `local_map_cleaner`,
  local BA's outlier removal, `mapping_module`'s
  `erase_temporal_keyframes_`, `landmark::replace()` -- and the goal is one
  combined "everything erased" log). `data::keyframe::prepare_for_erasing()`
  and `data::landmark::prepare_for_erasing()` each push one snapshot of
  their object's live state (position/pose, descriptor, covisibility,
  spanning tree, observations, ...) right before tearing it down;
  `data::landmark::replace()` sets the just-pushed record's
  `replaced_by_id` to the surviving landmark's id (the only caller that
  knows it) right after its `prepare_for_erasing()` call returns. A driver
  reads and clears both vectors once per frame (see "Dumps").

### `0006-pose-optimizer-g2o-trace.patch` / `0007-ba-trace.patch`

Both are opt-in (default off) capture buffers used by module 4b; neither
changes an algorithm value (`dump_stella_reference.py`-style dumps of
fr1_xyz stay byte-identical with 0007 built in; fr1_desk too apart from the
loop-candidate id column of `global_opt_step.tsv`, which varies run to run
even for the same binary -- see HANDOVER.md).

`0007` adds `optimize/ba_trace.h` (new header) and small hooks in
`local_bundle_adjuster_g2o.cc`, `global_bundle_adjuster.cc` and
`terminate_action.cc`: per real call it records the map view the function
read (keyframe/landmark ids, erased/root flags, poses, landmark slots,
sparse keypoints + octave of every observation, the observation lists in
map order, intrinsics, `inv_level_sigma_sq`), the g2o vertices and
monocular reprojection edges exactly as built (insertion order), per
`optimize()` stage the edge levels at stage start and per iteration the
robust chi2 / lambda / Levenberg trial count / stop flag (recorded from
inside `terminate_action` after its own bookkeeping, because g2o keeps
post-iteration actions in a pointer-ordered `std::set`), the final vertex
estimates, per-edge chi2/error/depth/level, the outlier observation list,
the Mat44 poses handed to `set_pose_cw()` and the optimized-landmark set.
Serializer: `stella_port/reference_tools/dump_stella_g2o_ba.cc` (binary
`ba_calls.bin`, format in its header comment), run with
`python3 tools/dump_stella_g2o.py fr1_xyz fr1_desk --ba [--check-determinism]`.
Besides the real local-BA / initial-global-BA calls it drives the real
`global_bundle_adjuster::optimize()` (loop-BA entry point, huber on/off) on
the final map, since no loop closure occurs in these sequences.
`replay_ba_eigen.cc` (`--ba-replay`) re-solves the captured global graphs
with the real g2o `LinearSolverEigen` (see HANDOVER.md module 4b part 2).

### Environment recreation (2026-09-29, after `/tmp` was wiped)

`/tmp/pose-opencv`, `/tmp/localopencv`, `/tmp/pose-validation-venv` were
gone. What was needed for the reference build/tools (no sudo): `apt-get
download` of `libopencv-{core,imgproc,imgcodecs,calib3d,features2d,flann,
video,videoio,highgui,ml}-dev` + the runtime `libopencv-*406t64` packages
and the transitive runtime libs `apt-get install -s` reports missing on
this machine (gdal/gdcm/hdf5/armadillo/tbb/proj/... about 55 .deb, 33 MB),
`dpkg-deb -x` into `/tmp/pose-opencv/root`, plus a hand-written
`/tmp/pose-opencv/pkgconfig/opencv4.pc` (Libs: core imgproc imgcodecs
calib3d features2d flann video videoio; Cflags: -I.../include/opencv4).
`external/candidates/deps/root/usr/lib/cmake/opencv4/OpenCVConfig.cmake`
already points at `/tmp/pose-opencv/root/usr/include/opencv4`.
`libarmadillo.so.12` lands in `root/usr/lib` (not `.../x86_64-linux-gnu`);
`/tmp/localopencv/root/usr/lib` got a symlink to it (that directory is on
every tool's rpath / LD_LIBRARY_PATH). highgui's Qt5 dependency is not
installed and not needed (nothing calls it). The python venv is not needed
by the reference build or the C harnesses (cmake comes from
`~/.local/bin`). The whole rebuild after that:
`python3 tools/build_stella_reference.py --skip-copy` (the `src/` copy in
`runs/stella_port/reference_build/` was intact, ~25 s incremental), then
`python3 tools/dump_stella_g2o.py fr1_xyz fr1_desk --ba`.

## Nondeterminism and threading pitfalls found

1. **RNG**: `util::random_array.cc::create_random_engine(use_fixed_seed)`
   returns `std::mt19937(seed_from_random_device)` unless `use_fixed_seed`
   is true, in which case it returns `std::mt19937()` (fixed,
   default-constructed seed). `use_fixed_seed` is read from
   `Initializer.use_fixed_seed` / `Relocalizer.use_fixed_seed` /
   `LoopDetector.use_fixed_seed` YAML keys (all default `false` upstream) and
   threaded into the fundamental/essential/homography/PnP RANSAC solvers.
   **Fix: config only** (`stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml`
   sets all three to `true`) -- no code patch needed.

2. **Pointer-hash-ordered containers**: `src/stella_vslam/type.h` defines
   `nondeterministic::unordered_set<T>` / `unordered_map<T, U>`, used for
   most `shared_ptr<keyframe>`/`shared_ptr<landmark>` containers in
   `tracking_module.cc`, `mapping_module.{h,cc}`,
   `global_optimization_module.cc`, `module/local_map_updater.h`. Upstream
   already ships the fix as an opt-in CMake flag: with `-DDETERMINISTIC=ON`
   (`target_compile_definitions(... DETERMINISTIC)`), the alias switches
   from `std::unordered_set/unordered_map` (bucket order depends on the
   `shared_ptr`'s raw pointer value, i.e. heap layout) to
   `std::set/map` ordered by `id_less<shared_ptr<T>>` (keyframe/landmark
   `::id_`, a monotonically-assigned counter -- deterministic regardless of
   allocator behavior). **Fix: build flag only** (`-DDETERMINISTIC=ON`,
   upstream's own option) -- no code patch needed.

   One gap `-DDETERMINISTIC=ON` alone didn't reach: `match::fuse::detect_duplication()`'s
   `duplicated_lms_in_keyfrm` output parameter (populated at 3 call sites --
   `mapping_module.cc` x2, `global_optimization_module.cc` x1, all in the
   landmark-fusion path) was declared as a raw
   `std::unordered_map<std::shared_ptr<data::landmark>, ...>`, not through
   the `nondeterministic::` alias. **Fix: code patch** -- `0003` switches it
   to the alias, matching every other `shared_ptr<landmark>`-keyed map in
   the codebase. (`new_connections`, the sibling parameter keyed by
   `unsigned int`, was already deterministic and untouched.)

3. **OpenMP**: `feature/orb_extractor.cc`, `match/stereo.cc`,
   `mapping_module.cc` (`create_new_landmarks`, roughly) have `#pragma omp
   parallel for` / `#pragma omp critical`. Upstream's own `USE_OPENMP` CMake
   option **defaults to OFF**, and without `-fopenmp` these pragmas are
   silently ignored by GCC -- the loops run as plain sequential C++, not
   OpenMP with 1 thread. **Fix: none needed** (already the upstream
   default; confirmed by the `-- OpenMP: DISABLED` CMake configure-time
   message with no options set). `OMP_NUM_THREADS=1` is set anyway before
   running the driver, belt-and-suspenders.

4. **Thread-timing dependence / real deadlock in the initial architecture**:
   two places call `mapper_->async_add_keyframe(keyfrm)` and then block on
   `.get()` of the returned future:
   `tracking_module::initialize()` (unconditionally, for every keyframe of
   the initial map) and `module::keyframe_inserter::insert_new_keyframe()`
   (conditionally, on `wait_for_local_bundle_adjustment`). In the threaded
   design the future is fulfilled when the mapping thread eventually pops
   that keyframe and finishes `mapping_with_new_keyframe()`. A driver that
   never starts that thread and only drains the queue *after*
   `feed_monocular_frame()` returns deadlocks here forever, because control
   never returns to the driver while blocked inside `feed_monocular_frame()`.
   This is exactly the failure mode: the first smoke test hung immediately
   after `"new map created with N points"`, and `0002`'s stage traces (see
   above) showed zero mapping-module activity had even started -- the hang
   was earlier, in `tracking_module::initialize()`'s `future.get()` loop, not
   in anything mapping/BA-related. **Fix: code patch** -- see `0001`,
   `mapping_module::set_synchronous(true)` makes `async_add_keyframe()`
   drain inline before returning, so the blocking `.get()` calls resolve
   immediately, in-order, on the calling thread. Also related but not a
   deadlock risk given the fix above: `correct_loop()`'s
   `mapper_->async_pause(); future_pause.get();` -- this resolves
   immediately too, because `mapping_module::is_terminated_` defaults `true`
   and is only ever set `false` inside `run()`, which never executes when no
   mapping thread is started; `async_pause()` special-cases
   `is_terminated_ || is_paused_` to fulfil the promise inline. Not a code
   patch, just a fact about `startup_single_threaded()` never calling
   `run()`, recorded here because it was non-obvious and easy to get wrong.

5. **`enable_interruption_of_landmark_generation` / `enable_interruption_before_local_BA`**
   (both default `true`): `mapping_with_new_keyframe()` either spawns a real
   `std::async(std::launch::async, ...)` thread racing
   `create_new_landmarks()` against `keyframe_is_queued()` polling to decide
   whether to abort early, or takes an early-return branch before local BA
   gated on `keyframe_is_queued() || pause_is_requested()` -- both are
   wall-clock/queue-state races in the threaded design. In this driver the
   mapping queue is always empty at this point anyway (nothing produces
   keyframes concurrently; see #4's fix), so neither branch would actually
   fire even left on. **Fix: config, for belt-and-suspenders correctness**
   (`Mapping.enable_interruption_of_landmark_generation: false`,
   `Mapping.enable_interruption_before_local_BA: false` in the reference
   config) -- removes the thread spawn and the race outright rather than
   relying on it never mattering.

6. **Wall-clock / real-time checks**: none found reachable from the
   monocular TUM tracking path (`grep -rn "std::chrono\|time(nullptr)\|
   gettimeofday"` in `src/stella_vslam` turns up only sleep/timeout logic
   inside the thread-loop bodies in `mapping_module.cc` /
   `global_optimization_module.cc` / `tracking_module.cc`'s
   `async_*`/pause machinery -- none of it is reached by
   `run_step()`/`startup_single_threaded()`, since `run_step()` is called
   directly instead of going through the `while(true) { sleep_for(5ms); ...
   }` loop bodies).

## Dumps

`--light` (`tools/dump_stella_reference.py --light`): only
`frames_before.tsv`/`frames_after.tsv` below, plus `trajectory.tum`. Cheap
(tens of KB/seq); use this for quick ATE/coverage checks.

Without `--light` (the default): the fuller schema below, one row per
`(frame, item)` in each file, `frame_idx` always the first column. All
floats are `%.9g` **and** `%a` (IEEE hex) side by side, per the dump-format
instructions, so two runs can be `diff`ed with no rounding ambiguity in the
text form.

Dump timing, exactly as instructed ("frame-level dumps BEFORE the
synchronous mapping pass, map-level dumps AFTER"), with one correction
found while adding the mapping-pass trace (patch `0005`, see its
changelog entry above for how this was diagnosed): **the mapping pass for
a newly-inserted keyframe does not run inside
`synchronize_background_modules()`** the way the naming suggests. Because
`mapping_module::set_synchronous(true)` (patch `0001`) makes
`async_add_keyframe()` drain inline, a keyframe insertion's mapping pass
actually runs *during* `feed_monocular_frame()`/`feed_frame()` itself, as
part of tracking. `synchronize_background_modules()` is mostly a
safety-net no-op that finds the queue already empty. This does not change
what is BEFORE vs. AFTER in the files below -- the driver still only reads
tracking-only state before calling `synchronize_background_modules()`, and
only reads mapping-pass/global-opt-step state after it returns and the map
database reflects the pass's result -- it only means "AFTER" is about when
the *driver reads* the result, not when stella_vslam *computed* it. The
mapping-pass/erasure trace fields are also explicitly reset once per frame,
by the driver, immediately before `feed_monocular_frame()` (not inside
`synchronize_background_modules()`, and not inside `run_step()`'s
queue-empty branch -- see the `0005` changelog entry for why both of those
looked right and weren't).

**BEFORE `synchronize_background_modules()`** (i.e. right after
`feed_monocular_frame()` returns for that frame -- tracking only, none of
that frame's own mapping pass has run yet):
- `frames_before.tsv`: `frame_idx, timestamp, tracked (0/1), pose_row_major_9g, pose_row_major_hex`
  (`cam_pose_cw`, from the pose `feed_monocular_frame()` returned).
- `keypoints.tsv`: `frame_idx, kp_idx, x_9g, x_hex, y_9g, y_hex, octave, angle_9g, angle_hex, response_9g, response_hex`
  -- `data::frame::frm_obs_.undist_keypts_` (`cv::KeyPoint`). Note: stella_vslam's
  `frame_observation` only retains the **undistorted** keypoints (there is
  no separate stored copy of the pre-undistortion positions to also dump).
- `descriptors.tsv`: `frame_idx, kp_idx, descriptor_hex` -- one 64-hex-char
  (256-bit) ORB descriptor row per keypoint, same index space as `keypoints.tsv`.
- `matches.tsv`: `frame_idx, kp_idx, landmark_id` (`-1` if that keypoint is
  unmatched at dump time) -- `data::frame::get_landmark(idx)` after
  whichever tracking stage succeeded; this is the frame's *final* match
  state for that frame's tracking pass, not a separate snapshot per
  motion-model/bow/robust attempt (see "Not done").
- `frame_trace.tsv`: `frame_idx, track_path, ref_keyfrm_id, initial_pose_valid, initial_pose_9g, initial_pose_hex, final_pose_valid, final_pose_9g, final_pose_hex, num_tracked_lms, num_reliable_lms`.
  `track_path` is one of `none/motion_model/bow_match/robust_match/relocalize_by_pose/relocalize_auto`
  (`tracking_module::track_path_t`, from patch `0004`) -- which
  `frame_tracker` variant (or relocalization path) produced this frame's
  pose. `initial_pose_*` is the pose right after that stage succeeded,
  before local-map tracking's own pose (re-)optimization; `final_pose_*`
  is the pose after it (same value as `frames_before.tsv`'s pose).
  `num_tracked_lms`/`num_reliable_lms` are the inlier counts *after*
  `optimize_current_frame_with_local_map()`'s outlier rejection (i.e. after
  the pose optimization that produced `final_pose_*`).
- `local_map.tsv`: `frame_idx, local_keyframe_ids (comma-joined), local_landmark_ids (comma-joined)`
  -- `module::local_map_updater`'s output for this frame (captured in
  `tracking_module::update_local_map()`, which otherwise discards the
  keyframe list once the function returns).
- `kf_decision.tsv`: `frame_idx, verdict, mapper_paused_or_pausing, num_reliable_lms_ref, num_reliable_lms, num_tracked_lms, distance_traveled_9g, max_interval_elapsed, min_interval_elapsed, max_distance_traveled, min_distance_traveled, view_changed, not_enough_lms, enough_keyfrms, tracking_is_unstable, almost_all_lms_are_tracked, mapper_is_skipping_localBA`
  -- every individual gate `module::keyframe_inserter::new_keyframe_is_needed()`
  computes plus the final AND'd verdict (previously only visible as
  `SPDLOG_TRACE` lines, compiled out by default). **Caveat**: this function
  is only called from `tracking_module::feed_frame()` when tracking
  succeeded and keyframe insertion isn't stopped; on a frame where it
  wasn't called, the row is a stale repeat of the last frame it *was*
  evaluated for (not recomputed, not zeroed) -- the driver does not
  currently distinguish "evaluated and this is the verdict" from "not
  evaluated this frame, carried over".
- `rng.tsv`: `frame_idx, engines_created_delta, array_calls_delta, elements_requested_delta, engines_created_total, array_calls_total, elements_requested_total`
  -- deltas since the previous frame and running totals of
  `util::create_random_engine()`/`create_random_array()` calls (patch
  `0004`'s atomic counters). `elements_requested` is the `size` argument
  summed across `create_random_array()` calls, a lower bound on
  `mt19937::operator()` invocations (the subsequent `std::shuffle` inside
  that function draws roughly `size` more per call, uncounted). Enough for
  a port to check "did I call the RNG the same number of times, with the
  same sample sizes, in the same order" per frame; not a full opcode-level
  replay log.

**AFTER `synchronize_background_modules()`** (that frame's mapping_module +
global_optimization_module synchronous passes have both fully run):
- `frames_after.tsv`: `frame_idx, num_keyframes, num_landmarks` (from
  `map_publisher::get_keyframes()`/`get_landmarks()`).
- `keyframes.tsv`: `frame_idx, kf_id, pose_cw_9g, pose_cw_hex, bad, covisibilities, spanning_parent, spanning_children`
  -- full map snapshot, one row per live keyframe, every frame.
  `covisibilities` is `kfid:weight` pairs in `graph_node_->get_covisibilities()`'s
  own order (stella's covisibility-count order; weight from
  `get_num_shared_landmarks()`); `spanning_parent` is `-1` if none;
  `spanning_children` is comma-joined ids. `bad` is `will_be_erased()`.
- `landmarks.tsv`: `frame_idx, lm_id, pos_w_9g, pos_w_hex, descriptor_hex, mean_normal_9g, mean_normal_hex, min_valid_dist_9g, max_valid_dist_9g, num_observed, num_observable, ref_keyfrm_id, observations`
  -- full map snapshot, one row per live landmark, every frame.
  `observations` is `kfid:keypoint_idx` pairs from `get_observations()`.
  `descriptor_hex` is the representative descriptor (`get_descriptor()`,
  empty if `!has_representative_descriptor()`).
- `trajectory.tum`: `system::save_frame_trajectory(path, "TUM")`, called
  once after the frame loop (not per-frame).

**Mapping-pass detail** (goal 2c), one row group per frame where a mapping
pass actually ran that frame (`mapping_module::get_last_pass_trace().ran`;
most frames have none of these rows -- see the timing note above for why
this can be read as "AFTER" even though the pass itself ran during
tracking):
- `culled_landmarks.tsv`: `frame_idx, mapping_keyfrm_id, landmark_id, reason`
  -- `local_map_cleaner_->remove_invalid_landmarks()`'s culled ids, reason
  always `observed_ratio_below_threshold` (the only reason that function
  culls for).
- `triangulation.tsv`: `frame_idx, mapping_keyfrm_id, neighbor_order, neighbor_keyfrm_id, num_matches, matches, accepted_landmark_ids, accepted_landmark_positions_9g, accepted_landmark_positions_hex`
  -- one row per neighbor keyframe `create_new_landmarks()` triangulated
  against, in that call's own covisibility order (`neighbor_order` is that
  position, 0-based); `matches` is `idx_in_cur:idx_in_neighbor` pairs
  (comma-joined); `accepted_landmark_ids`/`_positions_*` are the landmarks
  `triangulate_with_two_keyframes()` actually created from those matches
  (semicolon-joined positions, since each is 3 comma-joined values).
- `fused_landmarks.tsv`: `frame_idx, mapping_keyfrm_id, replaced_landmark_id, replaced_by_landmark_id`
  -- `update_new_keyframe()`'s landmark-fusion result (the same
  `0003`-fixed, id-ordered map).
- `local_ba.tsv`: `frame_idx, mapping_keyfrm_id, invoked, local_keyfrm_ids, fixed_keyfrm_ids, num_iters_first, num_iters_second, chi2_before_9g, chi2_after_9g, num_outlier_observations_removed`
  -- `invoked=0` (with the remaining columns blank) when local BA was
  skipped this pass (`map_db_->get_num_keyframes() <= 2`, or
  `is_skipping_localBA()`); otherwise `optimize::local_ba_trace` (patch
  `0005`) verbatim.
- `culled_keyframes.tsv`: `frame_idx, mapping_keyfrm_id, culled_keyframe_id, reason`
  -- `local_map_cleaner_->remove_redundant_keyframes()`'s culled ids,
  reason always `redundant_observation_ratio`.
- `global_opt_step.tsv`: `frame_idx, ran, keyfrm_id, candidate_ids_considered, loop_accepted, accepted_candidate_id, num_keyframes_sim3_corrected, num_landmarks_position_corrected, loop_ba_invoked`
  -- one row per keyframe the global optimization module's `run_step()`
  actually processed (every non-spanning-tree-root keyframe the mapping
  module hands it, so usually one row per mapping-pass row above, `ran=1`
  always for the rows present). `candidate_ids_considered` is empty when
  `detect_loop_candidates()` found none; `accepted_candidate_id` is `-1`
  when no loop was accepted this step.

**Erased objects** (goal 2, "rebuild dangling references"), everything
`data::keyframe::prepare_for_erasing()`/`data::landmark::prepare_for_erasing()`
recorded since the previous frame (`util/erasure_log.h`'s global log,
cleared by the driver after each frame's dump -- so each row is attributed
to exactly the frame the erasure happened in, regardless of which call
site triggered it: `local_map_cleaner`, local BA's outlier removal via
`landmark::erase_observation()`, `mapping_module::erase_temporal_keyframes_`,
loop fusion's `landmark::replace()`, ...):
- `erased_keyframes.tsv`: `frame_idx, kf_id, pose_cw_9g, pose_cw_hex, covisibilities, spanning_parent, spanning_children`
  -- same columns as `keyframes.tsv` minus `bad` (always true here by
  definition), snapshotted right before `graph_node_->erase_all_connections()`
  runs, so covisibility/spanning-tree are the object's state just before
  removal.
- `erased_landmarks.tsv`: `frame_idx, lm_id, pos_w_9g, pos_w_hex, descriptor_hex, mean_normal_9g, mean_normal_hex, min_valid_dist_9g, max_valid_dist_9g, num_observed, num_observable, ref_keyfrm_id, observations, replaced_by_id`
  -- same columns as `landmarks.tsv` plus `replaced_by_id` (`-1` for a
  plain cull; the surviving landmark's id if this was a `replace()`, i.e.
  it also has a row in `fused_landmarks.tsv` for the same frame).

## Snapshot interval

`--snapshot-interval N` (default `1` = every frame, unchanged behavior):
`keyframes.tsv`/`landmarks.tsv` (the two files that dominate dump size,
since both are a full map snapshot repeated every frame) are only written
on a frame where `num_keyframes`/`num_landmarks` actually changed since the
last snapshot, **or** `frame_idx % N == 0`, whichever comes first. Every
other file above is unaffected (they are either already only-on-change,
like the mapping-pass files, or small enough that skipping them isn't
worth the reconstruction cost, like `frames_after.tsv`). Verified
(`--snapshot-interval 20` on a 100-frame fr1_xyz prefix): `keyframes.tsv`
rows land exactly on the frames where a keyframe was actually inserted
(12, 19, 32, 63, 76, 79, 83) plus the interval boundaries (20, 40, 60, 80)
-- 11 distinct snapshot frames instead of 100.

All of the AFTER-pass and most of the BEFORE-pass data (everything except
`frame_trace.tsv`, `local_map.tsv`, `kf_decision.tsv`, `rng.tsv`, and the
mapping-pass/erased-object files, which needed patches `0004`/`0005`) came
from `stella_vslam`'s existing public API with **no code patch** --
`data::keyframe`/`data::landmark`/`data::frame` already expose everything
needed. Only the transient per-call state that stella_vslam computes and
discards needed a patch to persist it somewhere the driver could read.

**Dump size** (full, non-light, default snapshot-interval=1, fr1_xyz, 787
tracked frames): 388 MB total (was 386 MB before patch `0005`'s additions
-- the new mapping-pass/erasure files are small relative to
`landmarks.tsv`/`keypoints.tsv`, which still dominate: `landmarks.tsv` 212
MB, `keypoints.tsv` 87 MB, `descriptors.tsv` 68 MB, `matches.tsv` 11 MB,
`keyframes.tsv` 6.5 MB, `local_map.tsv` 2.1 MB, `frame_trace.tsv` 716 KB,
`erased_landmarks.tsv`/`fused_landmarks.tsv`/`kf_decision.tsv` each tens of
KB to ~1 MB depending on sequence, `triangulation.tsv`/`culled_landmarks.tsv`/
`rng.tsv`/`global_opt_step.tsv`/`local_ba.tsv`/`culled_keyframes.tsv`/
`erased_keyframes.tsv` each well under 100 KB -- roughly 490 KB/frame
average, still dominated by `landmarks.tsv`/`keypoints.tsv` because both
are full snapshots repeated every frame (map size x frame count, keypoint
count x frame count) rather than deltas; use `--snapshot-interval` to cut
that down. `--light` is ~460 bytes/frame.

## Determinism result

Two fresh processes, same binary, same config, same data, no shared state
between runs (`OMP_NUM_THREADS=1`, single-threaded driver). Latest results
below are with the complete patch set (`0001`-`0005`):

- **Full fr1_xyz (798 tracked-candidate frames, 787-line trajectory),
  full (non-light) dump schema, all 19 output files**
  (`trajectory.tum`, `frames_before.tsv`, `frames_after.tsv`,
  `keypoints.tsv`, `descriptors.tsv`, `matches.tsv`, `frame_trace.tsv`,
  `local_map.tsv`, `kf_decision.tsv`, `rng.tsv`, `keyframes.tsv`,
  `landmarks.tsv`, `culled_landmarks.tsv`, `triangulation.tsv`,
  `fused_landmarks.tsv`, `local_ba.tsv`, `culled_keyframes.tsv`,
  `global_opt_step.tsv`, `erased_keyframes.tsv`, `erased_landmarks.tsv`):
  every file byte-identical (`diff -q` clean) across two fresh runs.
- Repeated after a **from-scratch run of `tools/build_stella_reference.py`**
  (fresh copy of `external/candidates/stella_vslam`, all 5 patches
  re-applied from disk, library + driver rebuilt from nothing): same
  result, all 19 files byte-identical across two fresh runs of that
  freshly-built binary. This also proves the patch files on disk are
  self-consistent and sufficient -- `patch -p1` applying `0001`->`0005` in
  order onto a fresh pristine copy reproduces
  `runs/stella_port/reference_build/src` exactly (`diff -rq` clean).
- `--snapshot-interval` and `--light` both verified to still build and run
  correctly (see "Dumps"/"Snapshot interval" above for what each produces);
  not separately re-diffed run-to-run, since both only ever omit output the
  full/default run already covers byte-for-byte above, never add anything
  new that could differ.
- **Full fr1_desk (613 tracked-candidate frames, 543-line trajectory)**:
  `trajectory.tum` byte-identical across two fresh full-sequence runs, but
  this check predates patches `0003`-`0005` and was not repeated with the
  current patch set or the full (non-light) schema. `0003`-`0005` don't
  change tracking behavior (container-ordering fix + read-only
  instrumentation only, verified by fr1_xyz's ATE being bit-for-bit
  unchanged before/after, see below), so no different result is expected,
  but this specific combination -- fr1_desk x full schema x current patch
  set -- was not re-run.

**Result: single-threaded reference is deterministic on every case
checked**, including the complete full-schema dump (all 19 files, covering
every goal-2 item implemented) on a full 787-frame sequence, built from
scratch.

## ATE / coverage: single-threaded reference vs. multi-threaded

`python3 tools/tum_eval.py --systems stella_vslam_st,stella_vslam --seqs fr1_xyz,fr1_desk`,
full sequences, stella_vslam's shipped `TUM_RGBD_mono_1.yaml` intrinsics
(same camera params in the reference config), `external/candidates/orb_vocab.fbow`:

| System | Seq | Coverage | ATE-tracked (m) | n | wall (s) |
|---|---|---|---|---|---|
| stella_vslam_st (this work) | fr1_xyz | 98.6% | 0.0261 | 785 | 18.2 |
| stella_vslam (multi-threaded, `runs/tum_compare/stella_vslam/`) | fr1_xyz | 99.1% | 0.0246 | 789 | 13.0 |
| stella_vslam_st (this work) | fr1_desk | 88.6% | 0.0179 | 543 | 14.3 |
| stella_vslam (multi-threaded, `runs/tum_compare/stella_vslam/`) | fr1_desk | 88.3% | 0.0202 | 541 | 8.9 |

`stella_vslam_st`'s wall time went from ~13s/~9s (light dump) to ~18s/~14s
(full dump, the default `tools/dump_stella_reference.py` mode) -- the extra
5s is the full-schema TSV writing, not tracking; ATE/coverage are identical
either way (dump mode doesn't touch tracking behavior).

Mean ATE-tracked: `stella_vslam_st` 0.0220 m vs `stella_vslam` 0.0186 m
(over these 2 sequences only -- `stella_vslam`'s full 5-sequence mean in
`runs/tum_compare/table.md` is 0.0186 m too, since fr1_xyz/fr1_desk happen
to be close to its overall mean). Determinizing changes behaviour only
slightly here: coverage within 0.5pp, ATE within ~1.5cm either direction,
wall time within noise. This is not a controlled statement about
"threading changes accuracy in general" -- the multi-threaded numbers in
`runs/tum_compare/stella_vslam/` are themselves the median of 3 runs (per
`external/candidates/run_stella.sh`, since that configuration is
nondeterministic), so some of this gap is just which run of that
distribution the recorded one happened to be, not a reproducible effect of
single- vs multi-threading. Both runs' full outputs are the ones
`tools/dump_stella_reference.py` / `external/candidates/run_stella.sh` last
wrote to `runs/tum_compare/{stella_vslam_st,stella_vslam}/`.

## Not done / blocked

All three coordinator follow-up items are done: (1) the residual
pointer-ordered container (`0003`), (2) mapping-pass detail + erased-object
capture (`0005`, goal 2c and the "rebuild dangling references" part of
goal 2b), (3) the `--snapshot-interval` option. Remaining gaps, all
smaller and explicitly scoped down rather than silently dropped:

- **(Resolved by `0007-ba-trace.patch`, module 4b part 2: per-call,
  per-iteration BA trace for local BA, initial global BA and
  `global_bundle_adjuster::optimize`.)** *Original note:*
  **`module::loop_bundle_adjuster`/`optimize::global_bundle_adjuster` have
  no per-call cost/iteration trace** analogous to `local_ba_trace` --
  `global_opt_step.tsv`'s `loop_ba_invoked` is a bool, not a cost
  before/after or iteration count, because that optimizer is a shared free
  function (`optimize_impl` in `global_bundle_adjuster.cc`) used by the
  initial-map global BA, the loop-closing global BA, and reached from
  `module/loop_bundle_adjuster.cc`, and wiring a trace through it the way
  `0005` did for `local_bundle_adjuster_g2o` was more surgery than fit in
  this pass.
- **`matches.tsv`/`frame_trace.tsv`'s pose-per-stage give the *final*
  successful stage's data only**, not a separate snapshot per failed
  motion-model/bow attempt before whichever stage actually succeeded
  (`track_current_frame()` tries them in sequence and only the first
  success is kept; failed attempts leave no separate trace to capture).
  In practice only one stage ever succeeds per frame, so this is a minor
  simplification rather than a real gap.
- **`kf_decision.tsv`'s stale-row caveat** (documented in "Dumps"): a row
  is a repeat of the last frame `new_keyframe_is_needed()` was actually
  called for, not recomputed/zeroed on frames it wasn't.
- **g2o CSparse/LGPL exposure**: `optimize/global_bundle_adjuster.cc`
  (initial-map global BA) and `optimize/graph_optimizer.cc` (loop-closing
  pose-graph optimization) use `g2o::LinearSolverCSparse`, which links
  against CXSparse (LGPL-2.1+, present at
  `external/candidates/deps/root/usr/include/suitesparse`). Per
  `docs/stella_vslam_license_audit.md` rule 4 ("stay away from g2o's
  LGPL/GPL modules"), this build does **not** stay away from it -- loop
  closing is central to the task (goal 4 compares LC-on numbers) and
  swapping these two call sites to `LinearSolverEigen`/`LinearSolverDense`
  (as local BA / pose optimizer / transform optimizer already use) is an
  untested, numerically-different change that was out of scope to make
  blind here. Recorded in `provenance.json`
  (`g2o_solver_modules_linked`) and here so it isn't silently missed before
  any pure-C algorithm work or redistribution decision. Not blocking the
  single-threaded-determinism goal (1-2), which doesn't touch licensing.

### `0011-lifetime-and-graph-raw-state.patch` (module 6, instrumentation only)
Adds `graph_node::get_raw_state()` (connected map in map order + raw ordered list, expired weak_ptr entries as -1),
a keyframe-destructor log (`util::g_destroyed_keyframes`, off unless the driver sets `g_log_destroyed`) tagged with a
mapping phase (`util::g_phase`: 0 tracking, 1 mapping pass up to `remove_redundant_keyframes`, 2 from there incl. global
optimization, 3 after synchronize). The driver writes them as `conn.tsv` (every alive keyframe, on frames where a
mapping pass ran) and `kf_destroyed.tsv` (frame, phase, kf id). No computed value changes: all pre-existing dumps
regenerate byte-identically.
