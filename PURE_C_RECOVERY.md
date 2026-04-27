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

- Trial: projected-search BRIEF in `pure_c_brief` (layered on top of promoted brute-force BRIEF-256 reloc)
  - Intent: replace O(N_corners × N_map) brute-force Hamming with a per-corner candidate set. Project each map point into the current frame using the previous-frame pose and intrinsics (`fx,fy,cx,cy`), then only evaluate map points within a pixel search radius of each unlinked corner. Expected to help xyz (fewer false relinks to distant points preempting triangulations) while preserving or widening the room gain.
  - Implementation: `brief_relink(..., const Pose* prior, double fx, double fy, double cx, double cy)`, pre-projects all map points once per call, gates by `vis && (du*du+dv*dv) <= SR^2` before Hamming. Kept the same decision thresholds as brute-force (`best < 35`, Lowe ratio `0.60`, trigger `_nlink < 50`).
  - Radius sweep on spot-check sequences (note: spot-check frame counts were lower than the canonical benchmark, so these numbers were optimistic):
    - `SR=30`: room 1.8401 (worse than brute-force 1.7934), xyz 0.1814 (worse)
    - `SR=60`: room 1.7373, xyz 0.1825
    - `SR=80`: room 1.6711, xyz 0.1788
  - Full canonical `benchmark_native.py --all_gt --force` at `SR=80` (full 750/723/613 frame counts):
    - `test_freiburgdesk525`: 0.7410 → 0.7546 (**+0.0136**, outside ~0.01 m noise floor)
    - `test_freiburgroom525`: 1.7934 → 1.7821 (−0.0113, barely above noise)
    - `test_freiburgrpy525`:  0.0979 → 0.0994 (+0.0015, within noise)
    - `test_freiburgxyz525`:  0.1788 → 0.1788 (unchanged)
    - Mean ATE: 0.7028 → 0.7037 (slight net regression)
  - Conclusion: the headline room gain evaporates when measured on the full sequence (n=750 vs spot-check n=621), and desk incurs a real regression above the noise floor. Per the active promotion rule (all-GT validation, no promotion for ATE-neutral changes), projected-search at `SR=80` is rejected. Brute-force BRIEF retained.
  - Takeaway: prev-frame pose as a reloc prior is brittle under rotation/drift on desk and room. A useful future variant would gate radius by rotation magnitude, or use the propagated (constant-velocity) pose rather than the raw prev pose. Not implementing until a cleaner hypothesis emerges.

- Trial: IC-angle rotated BRIEF in `pure_c_brief`
  - Intent: make the in-tree BRIEF relocalization path rotation-aware without importing the full OpenCV ORB descriptor table.
  - Implementation: kept the same 256-pair random BRIEF layout, added an intensity-centroid angle over a `31x31` patch, and rotated descriptor sample offsets both at map-point birth and during relocalization matching.
  - Full canonical `benchmark_native.py --all_gt --force`:
    - `test_freiburgdesk525`: 0.7410 → 0.7528 (**+0.0118**)
    - `test_freiburgroom525`: 1.7934 → 1.8392 (**+0.0458**)
    - `test_freiburgrpy525`:  0.0979 → 0.0996 (+0.0017)
    - `test_freiburgxyz525`:  0.1788 → 0.1782 (−0.0006, within noise)
  - Saved trial artifacts under `runs/pure_c_iter/2026-04-13_rbrief_ic_angle/`.
  - Conclusion: rejected. A tiny xyz gain does not offset the clear desk and room regressions, so this intermediate "rotated random BRIEF" step should not be retried. If ORB-style work resumes, skip to a more faithful descriptor+matching port closer to the `cpp` path.

- Trial: prev-frame BRIEF kNN augmentation in `pure_c_brief`
  - Intent: port the smallest useful slice of the `cpp` front-end by caching BRIEF descriptors on the previous frame, extracting fresh current-frame corners, kNN-matching prev→curr with Lowe ratio `0.80`, and merging only mapped-point carry-forward matches into the existing KLT track set before PnP / E.
  - Implementation: no OpenCV or ORB pattern import; reused the in-tree BRIEF-256 descriptor, built a prev-frame descriptor cache, detected up to `1200` fresh current corners each frame, and accepted descriptor matches with `best < 48` and ratio `0.80`. To avoid obvious map blow-up, only previous corners with `pt_idx != -1` were allowed to add new current tracks.
  - Full canonical `benchmark_native.py --all_gt --force`:
    - `test_freiburgdesk525`: 0.7410 → 0.7490 (+0.0080)
    - `test_freiburgroom525`: 1.7934 → 1.8371 (**+0.0437**)
    - `test_freiburgrpy525`:  0.0979 → 0.0989 (+0.0010)
    - `test_freiburgxyz525`:  0.1788 → 0.1779 (−0.0009)
    - Mean ATE: 0.7028 → 0.7157 (net regression)
  - Saved trial artifacts under `runs/pure_c_iter/2026-04-13_prev_frame_brief_match/`.
  - Conclusion: rejected. The hoped-for room gain went the wrong way, and even the constrained "mapped-points only" merge still degraded mean ATE. Future descriptor ports should avoid bolting noisy BRIEF matches directly into the KLT track set; a more faithful `cpp`-style path likely needs its own standalone prev/curr feature sets rather than hybridizing against the tracked corner array.

- Trial: separate descriptor-only pose fallback in `pure_c_brief`
  - Intent: test the cleaner version of the same idea without polluting the KLT track array. Each frame builds an independent BRIEF feature set, matches prev→curr by Hamming kNN (`best < 48`, Lowe ratio `0.80`), and only uses descriptor matches as a rescue E-pose fallback when PnP fails and the normal LK-based E path is weak. No descriptor-based triangulation was allowed.
  - Implementation: added a per-frame BRIEF feature cache (`feat_corners`, `feat_descs`, `feat_ok`) beside the existing tracked corners; descriptor E was chosen only when LK-E either failed outright or had `< 24` inliers and the descriptor path exceeded it by `> 12` inliers.
  - Full canonical `benchmark_native.py --all_gt --force`:
    - `test_freiburgdesk525`: 0.7410 → 0.7480 (+0.0070)
    - `test_freiburgroom525`: 1.7934 → 1.8243 (**+0.0309**)
    - `test_freiburgrpy525`:  0.0979 → 0.0989 (+0.0010)
    - `test_freiburgxyz525`:  0.1788 → 0.1788 (+0.0000)
  - Saved trial artifacts under `runs/pure_c_iter/2026-04-13_desc_pose_fallback/`.
  - Conclusion: rejected. Keeping descriptor pose rescue separate from KLT tracks avoids map corruption, but it still hurts room materially and does not buy anything on xyz. The likely lesson is that random BRIEF plus this corner detector is not close enough to the `cpp` ORB front-end to be a useful prev/curr pose source by itself.


- Trial: global BA every 10 keyframes in `simple_slam_c_plus.c` (scratch fork of `simple_slam_c_brief.c`)
  - Intent: see whether a periodic full-map BA pass layered on top of the existing `local_ba` reduces ATE by refining all keyframe poses and all map points together every 10 keyframes. Added `refine_map_point_against_kfs` (Gauss-Newton 3x3 normal equations per point) and `global_ba(kf_db, map, ..., iters=3)` with OpenMP parallel loops over keyframes and map points.
  - Implementation: `simple_slam_c_plus.c` is `simple_slam_c_brief.c` plus the two new routines and a call site of `if (kf_db.size > 0 && kf_db.size % 10 == 0) global_ba(&kf_db, &map, fx, fy, cx, cy, 3);` right after `local_ba`. No other algorithmic changes.
  - Full canonical `benchmark_native.py --all_gt --force`:
    - `test_freiburgdesk525`: 0.7198 → 0.7515 (**+0.0317**, clear regression)
    - `test_freiburgroom525`: 1.8518 → 1.8304 (−0.0214)
    - `test_freiburgrpy525`:  0.0992 → 0.0987 (−0.0005, within noise)
    - `test_freiburgxyz525`:  0.1782 → 0.1781 (−0.0001, within noise)
    - Point counts grow substantially (e.g. xyz 13429 → 19616, room 21261 → 30745, rpy 27637 → 34101, desk 18874 → 23273) and runtime grows ~1.5x on every sequence.
  - Conclusion: rejected. Per the AGENTS.md promotion rule, desk's +0.0317 m regression is well outside the ~0.01 m noise floor and disqualifies promotion; the room improvement is only marginally above noise and does not compensate. Map-density growth is expected (more BA iterations lock in more triangulations) but is not a goal. Scratch sources `simple_slam_c_plus.c` and binary `simple_slam_pure_c_plus` are not promoted into the in-tree benchmark roster.
  - Takeaway: a naive "global BA every N keyframes" at full-map scope is not ATE-positive here. A future variant worth trying would be a windowed BA (last K keyframes + their co-visible map points) rather than all-keyframes/all-points, so the denser map edits do not drag desk away from its current fit.

- Trial: E-fallback translation rescaling in `simple_slam_c_plus.c` (2026-04-25)
  - Diagnosis: `pure_c_plus` lags `cpp` on room (1.7591 vs 1.5452, +0.2139). Per-frame instrumentation revealed PnP fails on 282/470 frames on room — between frames 160–319 it's 100% E-fallback (159 consecutive frames). Each `estimate_pose_E` returns a unit-length translation; composing 159 unit-translations onto the camera path collapses Umeyama scale to 0.0000 and explains the per-frame error spike at frame ~100.
  - Hypotheses tested:
    1. **Disable global_ba** — DISPROVEN. Room got worse (1.7591 → 1.8518) and scale=0.0000 unchanged. global_ba is helping, not hurting.
    2. **A1: rescale rel.t by ||last_rel.t||** — single-step magnitude. Single-GT timeout artifact made this look great (room 1.7591 → 1.0542 at 30s wall-clock cap). Compounds noise multiplicatively → trajectory magnitudes blow up to 1e15+.
    3. **A2: rescale rel.t by median of last 8 PnP step magnitudes** — added 8-element ring buffer. Single-GT (30s wall-clock) showed dramatic mean ATE win (0.6914 → 0.5204). Trajectory magnitudes bounded at 1e5–1e6.
    4. **A3: replace rel.t with last_rel.t directly (E rotation + predicted translation)** — DISPROVEN. Worse than A2 on 3 of 4 sequences; magnitudes explode to 1e40 due to direct positive feedback loop.
    5. **A4: streak-gated A2 (rescale only on 2nd+ consecutive E-fallback)** — marginally worse mean than A2 (0.5285 vs 0.5204).
  - **A2 full canonical sweep (`benchmark_native.py --all_gt --force`, 750 frames on room with `--timeout 120`):**
    - `test_freiburgdesk525`: 0.7307 → 0.7534 (**+0.0227**, clear regression)
    - `test_freiburgroom525`: 1.7591 → 1.7475 (−0.0116, marginal)
    - `test_freiburgrpy525`:  0.0990 → 0.0992 (~0.0000)
    - `test_freiburgxyz525`:  0.1768 → 0.1784 (+0.0016)
    - Mean: 0.6914 → 0.6696 (−0.0218)
  - Conclusion: rejected. The dramatic 5–10s wins in single-GT testing were a `--timeout 30` (default wall-clock cap) artifact — the binary terminated room at frame 445 of 750 before divergence had time to accumulate. With the canonical 750-frame run, A2 is essentially ATE-neutral with a desk regression (+0.023 m) outside the 0.01 m band, and a marginal room improvement (−0.012 m) just outside the band the other way. Classic "trades one failure mode for another."
  - **Methodological lesson:** any `simple_slam_pure_c_plus` direct invocation MUST pass `--timeout 120` (or higher) to match canonical benchmark conditions. Default `--timeout 30` is wall-clock, not video-clock, and silently truncates pure_c_plus runs on room (it processes ~14 fps, so 30s wall-clock = ~445 frames of video). **Single-GT diagnostic results from runs that terminate early are not transferable to canonical conditions.**
  - Takeaway: scale-from-PnP-history alone doesn't fix the underlying issue — that PnP itself fails for 160 consecutive frames on room. Future work should target the PnP failure root cause (e.g. inlier threshold, RANSAC iterations, or relink behaviour mid-streak) rather than papering over the E-fallback's unit-translation. Or, accept that monocular sequences with insufficient parallax structurally need scale priors (IMU, known depth, etc.) — beyond the scope of a pure SLAM port.

- Investigation: loop closure on `room` and the BA-divergence red herring (2026-04-27)
  - Diagnosis: `pure_c_plus` ships loop closure (thumbnail SAD detector + NCC patch verify + 5-pt RANSAC pose recovery + direct `pose = loop_pose` overwrite). On the canonical 30s `room` run (1.7591 m baseline), instrumented the detector and counted **0 fires** in 30s — verifying empirically that loop closure has been a no-op since `04f4321` landed.
  - SAD distribution measured per-frame (`bs` = sum-of-abs-diff over 16x16 thumbnails), 696 candidate evaluations on `room`:
    - `min=4356  p10=13468  p50=17145  p90=20273  max=24221  mean=16989`
    - Threshold was `3500`, **below the minimum observed** — the detector mathematically cannot fire.
  - Hypotheses tested:
    1. **Bump SAD threshold to 6000** (catch the bottom ~1% of pairs). Standalone test reported room ATE 1.0043 m (huge "win"). Sweep reported 1.7767 m. The 1.0043 turned out to be a bug in my own Umeyama implementation; benchmark.py's `ate_rmse` gave 1.7767 m for both the standalone JSON and the sweep JSON. Lesson: do not trust ad-hoc Umeyama; always reuse `benchmark.ate_rmse`. Net: threshold change is noise.
    2. **Replace thumbnail SAD with BRIEF Hamming-sum** detection. Added `Brief256 sig[64]` per `KFEntry`, computed on insertion from up to 64 stride-sampled corners. New scoring: for each query BRIEF, find min Hamming to any candidate-KF BRIEF, sum across the query signature. Distribution on `room`:
       - `min=2385  p10=2861  p50=3372  p90=3874  max=4375` — narrow dynamic range (~2x). Threshold `4000` fired **662 times in 30s** (every-frame false positive). Tightened to `2700` (below p10): **29 fires**, ATE 1.7505 m (vs 1.7591 baseline, ~0 delta).
    3. **Diagnostic on what `verify_loop` actually returns**: print `|cur_t|`, `|loop_t|`, `|kf_t|` on each fire. First fire at frame 189 already showed `|cur_t| = 44 km` while the matched-KF position was 3.6 m. Many later loop closures matched against KFs whose own `|kf_t|` was 1108 m, 3741 m, 293122 m — meaning prior local_ba/global_ba had already mutated those KFs to garbage. **The trajectory is exploding long before any loop fires.** Loop closure is a red herring.
    4. **Per-step magnitude trace** (loop closure disabled, instrumented after PnP+refine, after `local_ba`, after `global_ba`):
       - Frames 1-4: `|t|` between 0.4 m and 3.2 m (sane, monocular unit-scale).
       - Frame 17: `post_track |t| = 16.66` (inl=18). Tracking step jumped 5x.
       - Frame 20: `post_track |t| = 143.1` (inl=16). Already 100x.
       - Frame 22: `post_track |t| = 379.3` (inl=18). KFs all carry the bad max into BA, BA preserves it.
       - **The explosion is in `estimate_pose_PnP` / `refine_pose_lm`, not `local_ba` / `global_ba`.** BA just inherits.
    5. **Hypothesis: unbounded LM step in `refine_pose_lm`.** Inspected the function — `solve_6x6` produces `dx[6]` from a Hessian with only `+1e-6` diagonal floor; `dx` is applied directly to translation with no magnitude check. With sparse inliers (~15-20) the Hessian is poorly conditioned and `solve_6x6` can return `|dt|` of tens of meters per iteration.
    6. **A1: clip per-iter LM step (`|dt| > 0.3 m` or `|dr| > 0.1 rad` → break out).** Trace: explosion bounded from 379 m max → ~14 m max. ATE on room: 1.7591 → **1.8652** (regression).
    7. **A2: A1 + sanity-gate `estimate_pose_PnP` output against motion-model `predicted` (reject if `||t_pnp - t_predicted|| > 1 m`, fall back to predicted).** Trace: bounded further to ~6 m max, scale 1.4e-8. ATE on room: **1.8591** (still regression).
    8. **A2 full canonical sweep** crashed midway through `rpy` because the trajectory exploded to `xyz ≈ 7.9e122` — JSON Python's parser can't read. **A regression on rpy from 0.099 m → divergent.** Both safety nets are net-negative.
  - Conclusion: rejected. The naive read ("LM step is unbounded → clip it") was wrong. `refine_pose_lm` is *implicitly* a safety net — when PnP returns a bad pose, the LM iterations pull back toward the data, even if individual `dx` magnitudes are large. The 0.3 m clamp prevents those legitimate large corrections, so PnP errors stick and propagate via the motion model. Once the trajectory diverges, both the PnP gate's `predicted` reference and `refine_pose_lm`'s starting pose are pathological, and the safety nets accept obviously broken poses.
  - Takeaway: the real fix for `pure_c_plus`/`room` (and the underlying robustness across sequences) needs:
    1. **Trust-region LM in `refine_pose_lm`** — accept a step only if reprojection error decreases, otherwise increase `lambda` and retry. The current code never compares pre/post error; it just applies `dx` blindly.
    2. **P3P or non-minimal PnP** — replace the 6-correspondence linear DLT inside the 500-iter RANSAC with a 4-point P3P plus iterative refinement, or at least a 12+ correspondence overdetermined DLT. The minimal-sample DLT amplifies inlier noise.
    3. **Map-aware PnP failure recovery** — when consecutive frames produce wild PnP results, the relink path (BRIEF rematch against the map) should fire before falling back to the motion model, not after.
    4. The existing thumbnail-SAD loop-closure detector should either be removed or rewritten on top of BRIEF; the BRIEF detector code is preserved in `git stash@{0}` ("brief loop detector wip") for future reference.
    5. The `verify_loop` pose recovery (5-pt essential RANSAC) is not metric-scale-consistent with the trajectory; it must be replaced with PnP against the loop-KF's map points (which already carry BRIEF descriptors) before any loop closure can usefully correct drift.
  - Methodological lessons:
    - Always re-use `benchmark.ate_rmse` for ad-hoc ATE checks; my standalone Umeyama produced a 0.77 m phantom delta.
    - When investigating divergence, instrument each pipeline stage's output independently before assuming the obvious culprit (BA was the wrong suspect; PnP+refine were the actual ones).
    - "Add a clamp" is rarely a safe one-liner in iterative numerical code: the clamp removes the implicit-correction property that the unclamped iterations relied on.
