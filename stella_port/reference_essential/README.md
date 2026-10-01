# Essential 5-point solver and robust tracking fallback

Validated 2026-09-29 against the real pinned stella_vslam e445b545 and
Eigen 3.4 reference, using the existing SSE2 build flags (`-O2`,
`-ffp-contract=off`, `-fno-fast-math`). This is the tracking-side
`match::robust` leaf. No ORB-SLAM source was used. Nothing was committed.

## Result

| Check | fr1_xyz | fr1_desk |
|---|---:|---:|
| Full solver traces: mismatches / values | 0 / 1,057,531 | 0 / 25,285,393 |
| Forced-BoW tracking replay: mismatches / values | 0 / 989,673 | 0 / 679,624 |
| Tracking frames checked | 785 | 541 |
| Robust fallback frames checked (no skips) | 1 | 27 |
| Normal tracking replay: mismatches / values | 0 / 989,176 | 0 / 679,504 |

Every fallback input was dumped by two fresh reference processes; all 28
binary dumps are byte-identical. The trace-instrumented reference is also
checked against the original installed solver and the real upstream robust
matcher, including essential matrix bits, masks and landmark assignments.

The C solver and forced-BoW tracking replay pass AddressSanitizer and
UndefinedBehaviorSanitizer on both sequences with the same exact totals.
LeakSanitizer cannot run under this sandbox's ptrace; leak detection was
therefore disabled, not declared passing. Empty, truncated and deliberately
bit-corrupted fixtures are rejected. Detailed commands, outputs and source
hashes: `runs/stella_port/reference_essential/validation/`.

These are teacher-forced component/tracking checks against reference map
snapshots, not autonomous full-sequence SLAM or an ATE improvement claim.

## Code and boundaries

- `../c/sv_eigen_fullpivlu.{h,c}`: Eigen full-pivot LU, kernel and triangular
  solve, up to 10x10. MPL-2.0.
- `../c/sv_eigen_eigensolver.{h,c}`: Eigen 10x10 Hessenberg decomposition,
  real Schur iteration, eigenvalues and complex eigenvectors. MPL-2.0.
- `../c/sv_solve_essential_5pt.{h,c}`: five-bearing constraint basis,
  polynomial elimination, action matrix and real candidate extraction.
  Retains libmv MIT and stella BSD notices.
- `../c/sv_solve_essential_ransac.{h,c}`: sampling, candidate scoring,
  selection, inliers and optional 8-point refinement. Reuses validated
  `sv_rng`, `sv_eigen_svd` and `sv_linalg`; no external library dependency.
- `../c/sv_match_robust.c`: descriptor/orientation brute-force matching,
  five-point RANSAC, landmark assignment and pose optimization for
  `sv_tr_robust_match_based_track`. Uses the existing bearing conversion
  and pose optimizer. The failure-before-match-threshold path preserves
  the current frame's pose and landmark associations, as upstream does.
- `../c/check_sv_essential_5pt.c`: shared-runner ABI and explicit source list;
  compares sampling indices, constraints, basis, polynomials, eliminated and
  action matrices, eigenvalues/vectors, every candidate, every candidate's
  cost/count/mask, running best and final refined result.

`check_sv_track.c` no longer skips robust frames. Dependency declarations
were updated in `check_sv_track.c`, `check_sv_track_bow.c` and
`check_sv_mapping.c`; the latter's mapping code was not edited.

Scope limits: the production tracking path always uses five samples and a
fresh fixed-seed RNG. The RANSAC API additionally accepts >=8 samples, but
6/7-sample SVD-nullspace variants are explicitly rejected and unported.
Five-point rank-deficient input returns no candidates: upstream's dynamic
kernel then cannot fit its fixed 9x4 assignment, so we do not claim exact
behavior for that undefined/asserting case. No captured sample takes it.
The reference trace format uses the local x86-64 little-endian C layout;
it is not a portable interchange format. The trace harness requires the
captured >=8-inlier refinement path; all 28 fixtures exercise it.
This task does not implement mapping's no-vocabulary robust triangulation,
relocalization, initialization/reset glue, stereo or loop closing.

## Reproduce

The existing shared reference build and force-BoW dumps are read-only.
The following extraction/dump commands intentionally refuse to overwrite
fixture directories/files; retain the current evidence before regenerating.

```bash
python3 stella_port/reference_essential/instrument.py
python3 stella_port/reference_essential/build.py dump_reference --build-only
python3 stella_port/reference_essential/extract_inputs.py
python3 stella_port/reference_essential/dump.py fr1_xyz fr1_desk
```

`trace.patch` is the reviewable patch applied to the isolated solver copy
under `runs/stella_port/reference_essential/instrumented/`. It renames the
namespace and adds observers only. The reference does not execute C solver
code; it shares only the trace structure declaration. `extract_inputs.py`
reconstructs the current/reference keyframe observations from the recorded
pre-tracking reference ID, source-frame timestamp and previous map snapshot.
`inputs.json`, `reference.json` and `dump_reference.build.json` retain the
input, dump, binary, build and dependency hashes.

Rerun the published C checks using a new results directory:

```bash
python3 stella_port/reference_essential/validate.py \
  --out runs/stella_port/reference_essential/validation_repeat
```

The leaf also runs through `tools/check_stella_port.py`; no runner change
was needed. Its shared-runner invocation reads all 28 recorded fallback
cases even if a frame limit is supplied. Direct usage:

```bash
runs/stella_port/reference_essential/validation/check_sv_essential_5pt \
  fr1_desk runs/stella_port/reference_essential/fixtures/fr1_desk
```

## Numerical findings and supplemental oracles

- Eigen's multiple-RHS triangular solve uses width-4 panels. Sequential
  subtraction within panels and accumulated updates between panels matter.
- Hessenberg/Schur reductions preserve SSE2 two-lane grouping and the
  original Householder/Jacobi updates, shifts and stopping tests.
- Eigen's complex-vector normalization uses packet complex division:
  multiplying by the real norm and dividing by norm squared differs from
  ordinary scalar division by the norm. Signed zeros are retained.
- The final 9x4 basis/product has eight left-associated rows and one scalar
  balanced row: `(p0+p1)+(p2+p3)`.
- The inlier threshold is stella's `util::cos` polynomial at one degree,
  not `cosf`. Float casts and the double literal in accumulated cost matter.

Standalone native oracles (`python3 .../build.py <name>`):

| Oracle | Cases | Result |
|---|---:|---|
| `check_lu` | 5,000 | LU, kernel and solves exact |
| `check_schur` | 100 | H, Q, Schur T and U exact |
| `check_eigen` | 1,008 | All eigenvalues/vectors exact, including zero, repeated roots, Jordan blocks and extreme scales |
| `check_minimal` | 500 | Basis through final candidate coefficients exact |
| `check_ransac` | 100 | Nonminimal E, validity, costs, chosen E and masks exact; refinement on/off |

No numeric tolerance, alternate solver, candidate reordering or fixture
patching was used to obtain a pass.
