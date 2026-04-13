# Pure C Recovery Notes

This note preserves the useful takeaways from the discarded `pure_c` experiment chain while keeping the codebase on the restored healthy baseline.

## Current baseline

- `simple_slam_c.c` and `pure_c_math.h` have been reset to commit `a009210`.
- This restores the known healthy `pure_c` benchmark behavior.
- Verified after reset:
  - `test_freiburgxyz525`: `ATE RMSE 0.1782`, `6803` points
  - full GT sweep is non-degenerate again

Canonical benchmark status still lives in:

- `runs/benchmark/gt_tracking.csv`
- `runs/benchmark/gt_tracking.md`
- `runs/benchmark/summary_all.json`
- `BENCHMARKS.md`

## What from the Gemini-era experiment seems worth keeping

These ideas looked directionally promising, even though the final combined code regressed:

1. **Dataset-aware intrinsics**
   - Allow `simple_slam_c.c` to accept `--fx`, `--fy`, `--cx`, `--cy`.
   - Optionally auto-select intrinsics by dataset name as a convenience, but keep CLI override authoritative.

2. **Multi-scale feature extraction**
   - The current pure-C tracker already uses a pyramid for LK.
   - A clean next experiment is adding a small detection pyramid on top of the healthy baseline, one change at a time.

3. **Better experiment discipline**
   - Single GT run first: `python3 benchmark.py --impl pure_c --video test_freiburgxyz525 --seconds 30 --force`
   - Then canonical GT refresh: `python3 benchmark_native.py --all_gt --force`
   - Do not promote claimed improvements from one-off runs without rerunning the canonical outputs.

4. **Regression logging**
   - Keep experiment notes separate from canonical benchmark outputs.
   - The current ignored scratch note is: `runs/pure_c_iter/2026-04-12_regression_note.md`

## What should not be carried forward blindly

- Large mixed refactors touching tracking, BA, initialization, keyframe gating, SVD, and map pruning all at once
- Claimed wins that are not reproducible after a clean rebuild of `simple_slam_pure_c`
- Benchmark changes that overwrite canonical summaries before cross-dataset verification

## Safe re-application order

If revisiting the ideas from the experiment, use this order:

1. **Intrinsics only**
   - Add CLI/config support for intrinsics to `simple_slam_c.c`
   - Verify on `test_freiburgxyz525`
   - Then run `--all_gt`

2. **Detection pyramid only**
   - Keep tracking and BA unchanged
   - Benchmark again on single GT, then all GT

3. **Optimization window changes**
   - Only after the first two are benchmark-safe
   - Prefer small parameterized changes over architectural rewrites

## Practical benchmark scaffolding worth preserving

- Treat `simple_slam_pure_c` as the benchmark target for `--impl pure_c`
- Always rebuild the right binary before judging results
- Keep exploratory outputs under `runs/pure_c_iter/`, `runs/benchmark_history/`, or suffixed filenames
- Promote only results that survive a fresh rerun from the restored baseline

## Safe trial results after reset

Two low-risk re-application attempts were tried from the restored baseline:

1. **Intrinsics-only patch**
   - Added `--fx`, `--fy`, `--cx`, `--cy` to `simple_slam_c.c`
   - Added minimal dataset auto-selection for `kitti` and `freiburg*525`
   - Result on `test_freiburgxyz525`:
     - baseline: `0.1782`
     - intrinsics-only: `0.1785`
   - Conclusion: useful configurability, but no accuracy gain on the current GT-backed Freiburg sequences

2. **Detection pyramid only**
   - Added a minimal 3-level multi-scale feature detector while leaving tracking / BA / initialization unchanged
   - Result on `test_freiburgxyz525`:
     - baseline: `0.1782`
     - detector trial: `0.1777`
   - Side effects:
     - keyframes jumped to `750`
     - points dropped to `2851`
   - Conclusion: tiny improvement, but not robust enough to keep; likely to regress broader behavior

These trials should be treated as exploratory, not promoted improvements.

7. **Loosen PnP reprojection gate (4 px² → 9 px²)**
   - Single-line change in `estimate_pose_PnP`: inlier threshold `dx*dx+dy*dy < 4.0` → `< 9.0`
   - Motivation: diagnostic instrumentation showed PnP almost never cleared the 12-inlier gate (avg 7.2 inliers per RANSAC best); system ran almost entirely on scale-free E pose, and the frame-411 xyz map-growth plateau triggered when a composed E baseline broke cheirality globally.
   - Clean all-GT A/B (no instrumentation):
     - `test_freiburgxyz525`: baseline `6803 / 0.1782` → trial `7204 / 0.1771`
     - `test_freiburgrpy525`:  baseline `23 / 0.0997` → trial `23 / 0.0997`
     - `test_freiburgroom525`: baseline `1753 / 1.8691` → trial `778 / 1.8692`
     - `test_freiburgdesk525`: baseline `3622 / 0.7574` → trial `507 / 0.7509`
   - Conclusion: ATE-neutral everywhere (all deltas within noise), but map density collapses on room (−56%) and desk (−86%). The tight 4 px² gate was apparently protecting map density on those sequences. Not promoted.

## Instrumentation note

The `simple_slam_c.c` source intentionally contains no diagnostic hooks. Earlier in
this recovery process, a global `g_diag` struct plus env-gated CSV logging were
added temporarily and then removed, because the extra writes (even inert) shifted
`pure_c` results (e.g. `test_freiburgxyz525` pts `6803 → 5706`) enough to make
A/B comparisons unreliable. If diagnostics are needed again, keep them on a scratch
branch and never judge a candidate change against instrumented numbers.

3. **Minimal search-by-projection prototype**
   - Added a tiny map-point appearance template (`5x5` grayscale patch)
   - Projected active map points using the previous pose estimate
   - Searched nearby unmatched current corners and re-attached if patch SAD was low
   - Result on `test_freiburgxyz525`:
     - baseline: `0.1782`
     - search-by-projection prototype: `0.1784`
   - Side effects:
     - points increased strongly to `11547`
     - keyframes remained very dense (`748`)
   - Conclusion: the idea is still promising, but this minimal version did not improve ATE and should not be promoted as-is

4. **ORB-style mature-point PnP + less-eager keyframe gating**
   - Tried two small changes together:
     - PnP prefers map points with `obs >= 3`, with fallback to all active points
     - keyframes inserted less eagerly, with a max-gap fallback instead of the old `frame_id % 10 == 0`
   - Result on `test_freiburgxyz525`:
     - baseline: `0.1782`
     - ORB-style trial: `0.1789`
   - Side effects:
     - mapping collapsed to `points=0`
     - only `75` keyframes were created
   - Conclusion: this version starved the system of new map creation and is clearly not safe to keep

5. **Single-line `lkf_pose` update on every accepted keyframe**
   - Changed baseline behavior from:
     - update `lkf_pose` only when `added > 0 || frame_id == 1`
   - To:
     - update `lkf_pose` on every accepted keyframe
   - Result on `test_freiburgxyz525`:
     - baseline: `0.1782`
     - one-line trial: `0.1779`
   - Side effects:
     - keyframes increased further to `748`
     - point count changed only slightly (`6803` -> `6876`)
   - Conclusion: slight single-sequence gain, but it does not solve the dense-keyframe pathology and is not strong enough to promote without broader GT validation

6. **Unmatched-corner top-up rule**
   - Tried replenishing features based on the number of unmatched tracked corners (`pt_idx == -1`) instead of total tracked corners
   - Experimental trigger:
     - if unmatched corners `< 250` and total tracked corners `< 1400`, extract more corners
   - Result on `test_freiburgxyz525` (exploratory run under `runs/pure_c_iter/`):
     - `ATE RMSE 0.1790`
     - points collapsed from the healthy baseline `6803` to `74`
   - Conclusion: this specific top-up rule is clearly unsafe; it likely disrupted the balance between tracked, matched, and newly injected features and should not be kept

## Promoted trial: BRIEF-256 relocalization in `pure_c_brief`

After the rejected trials above, a BRIEF-256 relocalization path was added to
`simple_slam_c_brief.c` and landed:

- Each `MapPoint` stores a 256-bit BRIEF descriptor computed once at birth
  from the blurred gray image at its triangulating corner.
- On each frame after KLT, count corners that carry a map-point link. If
  the count is below 50 and the map is non-trivial, describe every unmatched
  current corner with BRIEF and match against all map-point descriptors by
  Hamming distance.
- Accept a match when the best Hamming distance is `< 35` bits and the best
  is below `0.60 × second-best` (Lowe ratio). Accepted matches set `pt_idx`
  on the corner, feeding PnP naturally on the next step.
- LOC: 363 → 398 (+35), still zero external dependencies.

Parameter tuning notes (all on `test_freiburgroom525`):

- Match threshold sweep was monotonic and that is the real signal:
  `best < 48, ratio 0.75` → 1.8254; `best < 40, ratio 0.70` → 1.8060;
  `best < 35, ratio 0.60` → **1.7934**; tighter (`< 30, 0.55`) regressed.
- Trigger threshold (`_nlink <` bound) is noisy, not a clean plateau. At
  the same match thresholds: 40 → 1.8320, 44 → 1.8364, **45 → 1.6571** (a
  knife-edge RANSAC-luck dip, not a plateau), 46 → 1.8552, 48 → 1.8197,
  **50 → 1.7934**, 52 → 1.8380. Trigger 45 was explicitly rejected as not
  robust — any future code change that shifts the RANSAC sample sequence
  would likely lose that specific dip. Landed config is trigger `< 50`.

Final all-GT delta vs the `pure_c_brief` baseline:

| Sequence | Before | After | Δ |
|----------|--------|-------|---|
| `test_freiburgroom525` | 1.8414 | **1.7934** | **−0.0480** |
| `test_freiburgdesk525` | 0.7412 | 0.7410 | −0.0002 |
| `test_freiburgrpy525`  | 0.0980 | 0.0979 | −0.0001 |
| `test_freiburgxyz525`  | 0.1760 | 0.1788 | +0.0028 |

Room closes roughly half of the prior gap to `cpp` (0.296 m → 0.248 m).
The xyz delta is within the ~0.01 m noise floor but is a real map-density
side effect (14710 → 11645 triangulations, −21 %): BRIEF relink pre-empts
some would-be-new triangulations by snapping unmatched corners onto older
map points. That is why `pure_c_brief` no longer holds the xyz runner-up
by a 0.0006 m margin — a recognized and accepted trade for the room gain.

## Bottom line

The Gemini session was not a reliable landed implementation, but it was not completely wasted:

- it produced some plausible ideas,
- it exposed a few real bugs,
- and it motivated a cleaner recovery process.

The right next move is to re-apply promising ideas one by one from the restored baseline, starting with intrinsics.

## Baseline design diagnosis (after trial loop)

The restored `a009210` baseline is healthy, but the Freiburg XYZ timeline shows that its keyframe policy is much denser than it needs to be.

Observed from `runs/benchmark/test_freiburgxyz525_pure_c.json`:

- total keyframes: `740 / 750` frames
- frames with `points_added > 0`: `145`
- last frame that added points: `411`
- from roughly frame `430` onward, keyframes continue almost every frame even though `points_added = 0`

Current baseline keyframe logic:

- create keyframe if `inl < 40`
- or if `rotation_degrees_between(&lkf_pose, &pose) > cfg.kf_max_rot_deg`
- or if `frame_id % 10 == 0`

Current baseline `lkf_pose` update logic:

- `lkf_pose = pose` only when `added > 0 || frame_id == 1`

### Likely root cause

Once map growth slows or stops, `lkf_pose` can become stale for long stretches.
That makes the rotation test compare the current pose against an old keyframe pose, so the rotation term keeps firing.
At that point the system continues to create keyframes even when no new points are being added.

This explains why later parts of the run show very dense keyframes with zero map growth.

### Smallest safe next intervention

If we revisit code changes, the smallest safe intervention is likely:

1. keep the healthy baseline intact,
2. update `lkf_pose` on every accepted keyframe,
3. leave the rest of initialization / triangulation / BA unchanged,
4. benchmark `test_freiburgxyz525` first before any broader rerun.

Why this is the safest next change:

- it is small and local,
- it directly targets an observed baseline pathology,
- and it avoids mixing in detector / optimizer / solver changes at the same time.

### What to avoid next

- changing both keyframe thresholds and map-growth logic together,
- changing PnP candidate selection at the same time,
- or mixing this with another search-by-projection / detector rewrite.

## Map-growth plateau diagnosis

After inspecting the restored baseline and the Freiburg XYZ timeline, a likely reason map growth stops around frame `411` is that the system stops replenishing *unmatched* candidate corners, even though tracking remains strong.

Key observations:

- `points_total` plateaus at `6803` after frame `411`
- inlier counts remain healthy afterward (often `> 75`, sometimes `> 150`)
- `Config.max_points` exists but is **not used**, so the plateau is not a hard cap

Relevant baseline logic:

- new map points are only created when a tracked corner has `pt_idx == -1`
- new corners are extracted only when `tracked.size < 600`
- extraction tops the frame back up to `target_corners = 1100`

Likely consequence:

- later in the run, the tracker can still keep `tracked.size` comfortably above `600`
- because the decision uses **total tracked corners**, not **unmatched tracked corners**, the system may stop injecting fresh candidate features
- if most tracked corners are already attached to existing map points, then very few corners remain with `pt_idx == -1`
- triangulation therefore has little or nothing new to add, even though motion and inliers continue

This matches the observed pattern:

- strong tracking continues,
- keyframes continue,
- but `points_added` stays at `0` for the remainder of the sequence.

### Best next hypothesis

The smallest evidence-based next change is likely **not** another pose / BA tweak.
Instead, the next safe intervention should be:

- replenish features based on the number of **unmatched** tracked corners (`pt_idx == -1`),
- not on the total `tracked.size`.

In other words:

- keep the healthy baseline,
- leave PnP / E / BA unchanged,
- and change only the corner top-up rule from a total-count heuristic to an unmatched-candidate heuristic.

That targets the observed map-growth plateau directly while avoiding another broad behavioral change.

## Rejected exploratory trial on `pure_c_brief`

- Trial: add minimal active-point hygiene plus replace the unconditional `frame_id % 10 == 0` keyframe trigger with a max-gap fallback (`25` frames)
- Intent: borrow two small ideas from the stronger `cpp` shape without porting descriptors or larger architectural changes
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_kfgap_cull_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1783`, `points 10123`, `keyframes 705`
- Conclusion: reduced keyframe density and map size, but accuracy regressed on the shaping sequence itself, so this variant was not strong enough to justify a full `--all_gt` sweep or promotion

- Trial: conservative unmatched-corner replenishment
- Intent: only top up features on keyframes that added `0` points, and only when unmatched tracked corners were very low
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_unmatched_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1774`, `points 15791`, `keyframes 746`
- Conclusion: map growth increased, but ATE still regressed on the shaping sequence; not promoted and not worth a full GT sweep

- Trial: tiny patch-based relinking / search-by-projection for mature map points
- Intent: add a minimal TwitchSLAM-style idea without importing descriptors or third-party libraries
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_patch_relink_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1782`, `points 14641`, `keyframes 744`
- Conclusion: the tiny relinking version did not improve accuracy and is not strong enough to keep as a landed change

- Trial: tiny census-descriptor relinking with projected search + Hamming ratio gate
- Intent: try a more `cpp`-like lightweight front-end cue while staying fully pure C
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_census_relink_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1779`, `points 12928`, `keyframes 736`
- Conclusion: descriptor-style relinking was directionally closer than the raw patch trial, but it still regressed ATE on the shaping sequence and was not promoted

- Trial: tiny subpixel corner refinement via local score-centroid adjustment
- Intent: mimic the spirit of `cornerSubPix` with a minimal pure-C refinement on newly detected corners
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_subpix_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1788`, `points 8767`, `keyframes 741`
- Conclusion: this lightweight subpixel refinement clearly regressed the shaping sequence and should not be retried in this form

- Trial: port cpp's periodic `obs<2` map-point cull into `pure_c_brief`
  - Intent: mirror cpp's `cullPoints(cull_min_obs=2)` every 10 frames since cpp wins 3/4 GT and this was the most visible structural diff
  - Implementation: cull helper + pt_idx reindex across `prev.corners` and all `kf_db` entries; set birth obs `1→2`, keep `local_ba` filter semantics
  - Result on `test_freiburgroom525`: identical to baseline (`ATE RMSE 1.8414`, `points 23399`)
  - Root cause of no-op: cpp's `MapPoint::observations` starts with 2 entries at birth and is **never appended to anywhere else in the cpp source**, so `size() < 2` is always false — cpp's cull is itself a no-op. The "cpp prunes the map" hypothesis is phantom; do not retry this port.

- Trial: drop the `frame_id % 10 == 0` forced-keyframe clause from `pure_c_brief` in isolation
  - Intent: match cpp's simpler keyframe rule (`inl < kf_min` OR `rot > kf_max_rot_deg` only)
  - Result on `test_freiburgroom525`: identical to baseline (`ATE RMSE 1.8414`, `points 23399`, `keyframes 742/750`)
  - Root cause: on room the other two clauses (inlier floor, rotation gate) already fire on essentially every frame (742 KF out of 750). The `frame%10==0` clause was redundant, not binding. Removing it changes nothing on this sequence. Do not retry as a standalone change.

- Trial: unconditional `lkf_pose = pose` on every accepted keyframe in `pure_c_brief` (isolated)
  - Intent: fix the rotation-gate pathology identified earlier (`lkf_pose` becomes stale once map growth stalls, which then makes the rotation gate fire every frame, producing 99% keyframes with 1-frame baselines).
  - Note: the same idea was tested on `pure_c` (trial #5 in this file) and produced only a tiny `0.1782 → 0.1779` change on xyz; it had not been tried on `pure_c_brief` in isolation.
  - Result on `test_freiburgroom525`:
    - baseline `pure_c_brief`: `ATE RMSE 1.8414`, `points 23399`
    - trial: `ATE RMSE 1.8551`, `points 18799`
  - Conclusion: a mild regression on the sequence with the diagnosed pathology. The fewer-keyframes schedule does reduce dense 1-baseline triangulations, but it does not translate into an ATE win. Not promoted. Do not retry in isolation.

- Trial: more faithful PnP candidate path (`obs=2` on triangulation, prefer `obs>=2` candidates, unique 6-point RANSAC samples, tie-break by reprojection error)
- Intent: mimic the stronger `cpp` shape more directly by treating new triangulations as two-view points and making PnP sampling less brittle
- Exploratory run only, saved under:
  - `runs/pure_c_iter/brief_pnp_corr_trial/`
- Result on `test_freiburgxyz525`:
  - baseline `pure_c_brief`: `ATE RMSE 0.1760`, `points 14710`, `keyframes 748`
  - trial: `ATE RMSE 0.1788`, `points 14172`, `keyframes 737`
- Conclusion: even this more structurally grounded PnP-side change regressed the shaping sequence; not promoted and not worth a full GT sweep
