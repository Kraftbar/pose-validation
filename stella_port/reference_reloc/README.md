# Stella relocalization and EPnP leaf

Completed 2026-09-30 against stella_vslam `e445b545`, using only the local
stella/Eigen/OpenCV/g2o reference and existing stella C helpers. No ORB-SLAM
sources were used. Nothing was committed.

## Delivered

- `../c/sv_pnp.{h,c}`: EPnP control points, barycentric coordinates, 12×12
  SVD, three beta initializations, Gauss–Newton refinement, rigid alignment,
  angular inlier test, four-point RANSAC and optional inlier recomputation.
- `../c/sv_relocalizer.{h,c}`: BoW candidate query, candidate/neighbor matching,
  optional robust matching, PnP, pose optimization, two projection searches,
  and three local-map refinement passes. Existing BoW, essential solver and
  g2o pose ports are reused.
- `sv_reloc_tracking_glue`: automatic relocalization branch from
  `tracking_module::track`, including BoW computation and successful-update
  semantics for the last relocalization frame ID/timestamp. Caller supplies
  the current frame and inherited reference keyframe and continues normal
  local-map tracking afterwards.
- `../c/check_sv_pnp.c` and `../c/check_sv_reloc.c` are discovered automatically
  by `tools/check_stella_port.py` through their explicit source lists. The
  shared runner and shared tracking implementation were not edited.
- `c/sv_eigen_pnp.{h,c}` isolates Eigen-derived SVD, QR and product evaluation
  code under MPL-2.0. Stella algorithm files retain BSD notices. C binaries
  require only the standard C library and libm, not the native dependencies.

## Validation

Every reference case ran in two fresh processes; traces and extracted PnP
inputs were byte-identical. Both ordinary and ASan/UBSan builds passed:

| Harness | Sequence | Cases | Mismatched / compared values |
|---|---|---:|---:|
| PnP | fr1_xyz | 54 | 0 / 3,063,102 |
| PnP | fr1_desk | 36 | 0 / 1,852,080 |
| Relocalization | fr1_xyz | 48 | 0 / 2,865,714 |
| Relocalization | fr1_desk | 40 | 0 / 1,908,842 |

PnP covers 37 distinct real correspondence sets, recomputation on/off,
plus 0-, 3-, 10-, and 12-identical-point fixtures on each sequence.
Relocalization uses 11 recorded maps, eight cases each: automatic BoW,
supplied candidate, empty database, forced high observation threshold,
neighbor search disabled, robust matching, automatic tracking branch,
and failed automatic tracking branch. Comparisons include ordered stage
traces, poses, landmark associations, reference IDs and tracking timestamps.

Additional direct C/native comparisons pass: 500 Eigen shape tests (including
500 large blocked Gram products), 100 EPnP tests (0/270,480), and 100 RANSAC
tests (0/5,096,202). Four negative fixtures fail as required: empty case list,
missing trace, truncated trace and changed trace value. `-Wall -Wextra` is
clean for the two new algorithm translation units.

ASan/UBSan reported no findings. Leak detection was disabled
(`ASAN_OPTIONS=detect_leaks=0`); this is not a leak-check claim.

Evidence lives in `runs/stella_port/reference_reloc/`:
`validation.json`, `validation_sanitized.json`, `negative_checks.json`,
`check_{math,pose,ransac}.validation.log`, native `*.build.json`, and
`fixtures/<sequence>/{inputs.json,pnp/reference.json,relocalization/reference.json}`.
The manifests record hashes, commands and native dependency provenance.

## Reference changes and limitations

Observer patches are local to this directory; the shared reference build
and tracking dumps are unchanged. Numbering was checked across the stella
tree before creation:

- **0015:** renamed, standalone PnP observer with ordered internal traces.
- **0016:** reject alignment SVD `NumericalIssue` instead of reading
  uninitialized singular vectors.
- **0017:** renamed relocalizer observer; forwards fixed-seed configuration
  to the optional robust matcher (upstream omitted that argument).
- **0018:** initialize EPnP output to quiet NaNs when no candidate has a finite
  error, so RANSAC rejects it rather than reading an uninitialized pose.

The two PnP guards intentionally define previously undefined degenerate
behavior. The all-identical synthetic fixture is checked against the guarded
reference only; an unmodified upstream run produced a different pose.
All real PnP fixtures additionally check the public result against the
unmodified installed solver. Relocalization does the same except robust
mode, whose previously random-device seed is intentionally fixed.
No tolerance, reordered comparison, or fixture substitution is used.

The C product evaluation follows the pinned SSE2 Eigen cache profile
(L1=32768 bytes, `mr=nr=4`, maximum inner block 504). Large inlier-refinement
products need this blocking order for bit equality. Different Eigen/compiler/
CPU profiles require fresh verification; bit equality is not promised across
those profiles. `check_blocking.cc` records the native profile.

The desk frame-550 input is retained but **excluded**: snapshot 549 contains
26 covisibility links to keyframe 3, absent from its keyframe table. Native
reconstruction fails with `map::at`. No missing graph state was fabricated.
The remaining desk frames are 80, 110, 150, 250 and 400; xyz uses 80, 150,
250, 400, 550 and 700. Historical attempts are preserved in the fixture tree.

These are controlled lost-state replays on native map snapshots. They do not
validate naturally occurring loss during a continuous C run or establish ATE.
External pose-request candidate selection and the rest of the top-level
tracking loop remain outside this automatic-relocalization leaf.

## Reproduce

With existing immutable fixtures:

```bash
python3 stella_port/reference_reloc/validate.py
python3 stella_port/reference_reloc/validate.py --sanitize
python3 stella_port/reference_reloc/negative_checks.py
```

To create a new reference fixture tree, preserve/move an existing tree first;
the extraction and dump scripts refuse to overwrite it:

```bash
python3 stella_port/reference_reloc/build_reference.py
python3 stella_port/reference_reloc/extract_maps.py
python3 stella_port/reference_reloc/dump.py
python3 stella_port/reference_reloc/dump_pnp.py
python3 stella_port/reference_reloc/prepare_suite.py
```

Native builds reuse the pinned flags from the existing frame/BoW reference
build. Its dependency installation must be present, including the locally
extracted OpenCV libraries under `/tmp/pose-opencv/root`. Builds hash all
resolved headers/libraries. `build_reference.py` regenerates only this leaf's
isolated observer sources. Case lists are exposed only after verifying both
reference passes and all input hashes.
