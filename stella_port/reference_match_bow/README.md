# BoW descriptor matching — validated C leaf

`sv_match_bow.{h,c}` ports two ORB methods from stella_vslam e445b545:
`match::bow_tree::match_frame_and_keyframe` and `match_keyframes`.
It is C99 with libc/libm only. It borrows the existing `sv_bow_feat_vector`
and `sv_keypoint` types without modifying them. Derived source preserves
the AIST/stella-cv BSD-2 notices. No ORB-SLAM2 source was used.

## Scope and integration

- `sv_match_bow_frame`: result indexed by frame keypoint, holding landmark
  tokens from the keyframe.
- `sv_match_bow_keyframes`: result indexed by first-keyframe keypoint,
  holding tokens from the second keyframe.
- Token 0 means absent; nonzero `uint64_t` tokens belong to the caller.
  The reference adapter uses native landmark ID + 1, including UINT32_MAX.
- Both methods preserve sorted BoW node traversal and the supplied order of
  each node's feature indices. No result sorting is applied. Equal distances
  keep the first best candidate while updating the second-best distance.
- Landmarks absent or pending erasure are skipped on the keyframe sides.
  The frame's own landmark state is ignored. Distinct keypoints associated
  with the same landmark token are not deduplicated.
- ORB Hamming threshold is inclusive at 50; angle difference is inclusive at
  30 degrees. Angle wrapping follows `util::angle::diff` (float subtraction,
  double-literal wrap). Lowe ratio multiplication is float, rejection is
  strict `<`, and the initial best/second distances are 256.
- Only second-view **keypoints** are reserved after accepting a match.
  There is no added orientation histogram or landmark-ID uniqueness rule.

Views borrow memory and need caller serialization. Outputs must not alias
inputs. Invalid input or allocation failure returns -1 without changing
outputs. Ratios must be finite and nonnegative; feature nodes must be sorted,
indices valid and unique across nodes, and indexed angles finite. Empty views
and empty feature nodes are supported. See the header for the full contract.

This leaf does not include `match_for_triangulation`, continuous tracking,
geometry, pose optimization, or the map model. It also does not resolve the
separate [BoW database candidate-order gate](../reference_bow_db/README.md).

## Real reference

`dump_main.cc` links the existing installed stella library and invokes its
actual matcher methods on real `frame`, `keyframe`, and `landmark` objects.
No library patches, private-field replacements or substitute algorithms are
used. The public landmark erasure method sets the pending-erasure state before
attaching it to a keyframe. The frame-matching output is prefilled to check
upstream's replacement semantics.

The builder reads the working module-2 compiler/link commands but writes only
`runs/stella_port/match_bow/build/`. Provenance records compiler, native binary,
upstream commit, matching source checksums, installed headers, and explicit
linked libraries; it does not hash every transitive system dependency.
The original and built-copy matcher/angle sources must be identical.

Real-image fixtures reuse all 798 xyz and 598 desk frame descriptors, angles,
and BoW feature-node vectors from the validated prior dumps. They attach
**controlled synthetic landmark states**, not a recorded continuous map.
Each sequence tests pairs separated by 1 and 50 frames, both methods, angle
filter enabled/disabled, ratios .6/.75/.8, and periodic self-matches.
Thus these results establish matching parity across real image inputs;
they do not establish tracking coverage or ATE.

Synthetic fixtures exercise empty/disjoint views, Hamming distances
0/1/49/50/51/256, angles at and adjacent to 30 degrees, angle wrapping,
ties, reversed feature-index order, ratio equality, absent/erased landmarks,
aliased landmark tokens, and the maximum native landmark ID. Every case
runs in two fresh native processes; complete outputs are byte-identical.

## Validation

Final published harness, normal and ASan/UBSan builds:

| Fixture | Matcher calls | Compared counts/output slots | Mismatches |
|---|---:|---:|---:|
| Synthetic | 13,280 | 115,384 | 0 |
| fr1_xyz | 6,244 | 7,572,290 | 0 |
| fr1_desk | 4,628 | 5,525,958 | 0 |
| **Total** | **24,152** | **13,213,632** | **0** |

18 API checks also pass. No ASan/UBSan findings. Leak detection is disabled
(`detect_leaks=0`) for this environment, so this is not a leak-check claim.
Negative tests reject one corrupted landmark token (exit 1), empty coverage,
missing expected output, and extra expected rows (exit 2).

The shared runner discovers exactly `check_sv_match_bow.c sv_match_bow.c`.
Its existing `<seq> <fixtures_dir> <dump_dir> [max_frames]` ABI was exercised
on desk. This harness always checks the full dedicated pair fixtures and
ignores `max_frames`; missing fixtures fail. It does not depend on images or
on the unfinished initializer harness. No shared-runner changes were needed.

Artifacts under `runs/stella_port/match_bow/`:

- `build/provenance.json`: native build inputs and binary hash.
- `fixtures/provenance.json`: imported input hashes, case counts,
  command/expected hashes, two-process determinism.
- `final_checks/results.json`, `final_sanitized/results.json`: published
  source/header hashes, fixture hashes, commands and results.
- `negative_checks/results.json`: negative fixtures and shared-runner ABI.
- `final_validation.json`: final hash verification and consolidated results.

`checks/` and `sanitized/` are earlier runs of the identical staged harness
before it moved to `stella_port/c/check_sv_match_bow.c`.

## Reproduce

```bash
python3 tools/build_stella_match_bow.py
# Fresh paths: fixture/check scripts refuse to overwrite prior outputs.
python3 tools/dump_stella_match_bow.py --skip-build \
  --out runs/stella_port/match_bow/fixtures_repeat
python3 tools/check_stella_match_bow.py \
  --fixtures runs/stella_port/match_bow/fixtures_repeat \
  --out runs/stella_port/match_bow/checks_repeat
python3 tools/check_stella_match_bow.py --sanitize \
  --fixtures runs/stella_port/match_bow/fixtures_repeat \
  --out runs/stella_port/match_bow/sanitized_repeat
```

Input format: `F id n`, followed by n rows of `angle_hex descriptor_hex
landmark_token erased`, then a node count and rows `node_id count indices...`.
`Q id mode first_id second_id ratio orientation` invokes mode 0 (frame) or 1
(keyframes). Output is `Q id match_count length tokens...`, including all
unmatched zero slots. Native/C comparisons require every output count and
slot to match exactly, without tolerances or canonicalization.
