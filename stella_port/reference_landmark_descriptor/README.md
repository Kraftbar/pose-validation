# Landmark representative descriptor — validated C leaf

`sv_landmark_descriptor.{h,c}` implements the ORB selection part of pinned
stella_vslam e445b545 `landmark::compute_descriptor`. It uses only C99/libc,
preserves AIST/stella-cv BSD-2 notices, and uses no ORB-SLAM2 source.

## Integration contract

Pass observations containing unique keyframe IDs, borrowed 32-byte descriptors,
and the **keyframe** pending-erasure flag. Input order is arbitrary. The leaf:

1. Orders observations by keyframe ID, matching upstream's observation map.
2. Ignores observations from keyframes pending erasure.
3. Computes each descriptor's Hamming distances to all live descriptors,
   including self-distance zero.
4. Uses the lower median, index `(live_count - 1) / 2` after sorting.
5. Selects the first strict minimum; ties keep the smallest live keyframe ID.

The result owns a copy of the selected descriptor and reports its original
input index, keyframe ID and median. An optional array reports each input
row's median (`UINT16_MAX` for erased keyframes). A single row scratch buffer
replaces upstream's quadratic distance matrix without changing integer results.

The caller still owns the map, observation lifetime, descriptor cache and its
invalidation flags. This function does not change them or synchronize access.
It returns -1 for invalid input, allocation failure, or no live observations,
leaving outputs unchanged. The native method throws on an all-erased snapshot;
empty observations violate its precondition. Those cases are checked only as
C API errors, not claimed as successful native selections. See the header.

## Reference and fixture scope

`dump_main.cc` constructs real upstream keyframes and a landmark, adds the
observations through the public API in deliberately unsorted order, then calls
the installed library's real `compute_descriptor`/`get_descriptor`. It also
records every row median using upstream's actual descriptor-distance helper,
and verifies its traced selection against the real method's returned bytes.
That trace supplies the selected-observation tie identity, which the public
method does not expose when multiple observations have identical bytes.

Erased keyframes are set up through the real public erasure method with a
valid temporary spanning parent, before adding the landmark observations.
No private-field patch or substitute landmark implementation is used.

The existing reference has erasure-log instrumentation elsewhere in
`landmark.cc`; the builder requires the entire `compute_descriptor` method
to match the pinned original exactly. Both source files, matcher header,
observation-order types, installed headers, compiler/link commands, explicit
library files and binary are hashed. Transitive system dependencies are not
all hashed. Builds write only `runs/stella_port/landmark_descriptor/build/`.

The xyz/desk fixtures use eight anchor BoW nodes per frame and up to eight
consecutive frames per observation cloud. Descriptor bytes come from the
existing real-image dumps for all 798 xyz and 598 desk frames. They include
controlled keyframe-erasure flags and shuffled insertion order. These are
**synthetic observation clouds drawn from real images**, not tracked landmark
histories or an end-to-end map replay.

Synthetic cases cover 1–127 observations, odd/even lower medians, identical
descriptors, complementary descriptors, Hamming extremes, repeated distances,
first-minimum ties, reversed insertion order, erased lowest-ID observations,
and UINT32_MAX keyframe IDs.

## Results

Every native fixture is byte-identical across two fresh reference processes.
Normal and ASan/UBSan builds of the published C harness pass:

| Fixture | Selections | Compared values | Mismatches |
|---|---:|---:|---:|
| Synthetic | 320 | 20,138 | 0 |
| fr1_xyz | 6,384 | 241,310 | 0 |
| fr1_desk | 4,784 | 180,556 | 0 |
| **Total** | **11,488** | **442,004** | **0** |

Each case compares the selected index, ID, median, all 32 descriptor bytes,
and every input row's median. Fourteen API checks also pass, including invalid
inputs, no live rows, unchanged outputs on failure, and descriptor-copy lifetime.
No sanitizer findings; `detect_leaks=0`, so leaks were not checked.

Negative harness checks reject a corrupted descriptor byte (exit 1), empty
coverage, missing reference output, and extra reference rows (exit 2).
The published harness is discovered with exactly two sources and passes the
shared-runner ABI on desk. It always checks the full dedicated fixture and
ignores `max_frames`; unsupported/missing fixtures fail. No shared-runner edits.

Final evidence is under `runs/stella_port/landmark_descriptor/`:
`build/provenance.json`, `fixtures/provenance.json`, `final_checks/results.json`,
`final_sanitized/results.json`, `negative_checks/results.json`, and
`final_validation.json`. `checks/` contains the initial harness compilation
warning; `checks_v2/` and `sanitized/` are successful runs before the harness
was published under `stella_port/c/`.

This completes descriptor selection only. Mean normal/scale prediction, map
lifecycle, tracking integration and ATE remain separate work.

## Reproduce

```bash
python3 tools/build_stella_landmark_descriptor.py
python3 tools/dump_stella_landmark_descriptor.py --skip-build \
  --out runs/stella_port/landmark_descriptor/fixtures_repeat
python3 tools/check_stella_landmark_descriptor.py \
  --fixtures runs/stella_port/landmark_descriptor/fixtures_repeat \
  --out runs/stella_port/landmark_descriptor/checks_repeat
python3 tools/check_stella_landmark_descriptor.py --sanitize \
  --fixtures runs/stella_port/landmark_descriptor/fixtures_repeat \
  --out runs/stella_port/landmark_descriptor/sanitized_repeat
```

Fixture/check output folders must be fresh. Input is `Q case_id count` followed
by rows `keyframe_id erased descriptor_hex`. Expected output is
`Q case_id selected_input_index selected_keyframe_id median descriptor_hex count`
followed by `count` medians. Comparisons use exact integer/byte equality.
