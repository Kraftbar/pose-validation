# Independently trained ORB vocabulary for stella

Work is isolated in `tools/vocab/`, `stella_port/vocab/` and
`runs/stella_port/vocab/`. The canonical `external/candidates/orb_vocab.fbow`,
reference dumps and harnesses remain unchanged. No commit or promotion is
part of this task.

## Source and licensing

The trainer is new MIT-licensed C99 code using only libc and libm. It does
not read the old vocabulary and does not use an existing vocabulary trainer.
The only learned inputs are descriptors extracted from selected images in
`freiburg2_large_no_loop` and `freiburg3_teddy`.

TUM's [dataset license statement](https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license)
identifies the data as CC BY 4.0. The artifact is distributed as CC BY 4.0,
with attribution and transformations recorded in
[`stella_port/vocab/ATTRIBUTION.md`](../stella_port/vocab/ATTRIBUTION.md).
This permits commercial reuse with attribution; it is not an MIT/BSD-only
artifact. See the [CC BY 4.0 terms](https://creativecommons.org/licenses/by/4.0/).

The training sequences are excluded from the five comparison recordings.
This is a recording split within TUM, not a claim of different buildings or
independent scene distributions. No comparison images, GT trajectories,
old-vocabulary centers or old-vocabulary weights enter training.

## Fixed candidate specification

`own_orb_v1`: branching factor 10, maximum depth 6, at most 15 Lloyd updates
per split, SplitMix64 seed 20260930. The candidate is specified before its
comparison results are inspected; no selection by test-set ATE is performed.

- Extract every sixth original RGB PNG with stella's ORB API, scale factor
  1.2, eight levels, FAST thresholds 20/7, min-area 800.
- Convert BGR images from OpenCV decoding to grayscale correctly; cap each
  image at 1,200 descriptors using uniform deterministic indices.
- Choose initial centers with k-means++ D² weighting on Hamming distance.
- Update each binary center by bit majority (ties zero); assign equal-distance
  descriptors to the earliest center. Remove empty clusters, terminate
  singleton/identical clusters, and cap depth/iterations.
- Calculate document frequencies by actually routing descriptors through
  the finished tree. Leaf weight is `log((N+1)/(df+1))+1`, explicitly smoothed
  IDF. Native FBoW performs TF accumulation and its existing L2 normalization.
- Export an explicit little-endian FBoW header and blocks, with 32-byte
  alignment and the native leaf-marker convention. No raw C structs are
  serialized. The reader format was checked against FBoW's MIT source;
  its notice is retained.

FBoW compatibility is verified by loading the output in the real native
library and the already-validated C reader, then comparing all BoW words,
float weights, feature-vector groups and consecutive-image scores bitwise.
That comparison validates serialization/consumption, not training quality.

## Artifact and validation

The release contains **486,271 words**, 85,107 blocks and 571,378 tree nodes,
trained from **754,938 descriptors in 962 images**. File size is 38,128,064
bytes (36.36 MiB); leaf depths are 4–6. All leaves have nonzero training
support. SHA-256:
`4bd76eabd355a54225cf0b2b057afd565aba48d42e70dcf7477b4dfab9b624a5`.

Two separate extraction processes produced identical descriptor files;
two separate training processes produced identical vocabulary files.
Native FBoW and the C reader matched on **0/2,865,573** comparisons covering
all training documents, word weights, feature groups and adjacent-image
scores. Independent structural inspection checks reachability, parent links,
unique word IDs, finite positive weights and file-layout bounds. Synthetic
random/identical/few-unique inputs are deterministic and compatible; malformed
inputs are rejected. ASan/UBSan passes the three synthetic trainer cases and the full
754,938-descriptor training run with byte-identical output; leak detection
is disabled. The full-data sanitizer record is `validation_sanitized.json`.

Evidence: `runs/stella_port/vocab/{training/sources.json,training_images.json,
train1.json,train2.json,compatibility.log,structure.json,tests/results.json,
tests/sanitized.json}`. Binary/header/library build hashes are in
`{extract,check_compat,train}.build.json`. Release metadata and attribution
travel with the `.fbow` file under `stella_port/vocab/`.

## Evaluation protocol

Use the existing unmodified optimized `run_tum_rgbd_slam` binary, its original
monocular configurations, LC enabled, no sleeping, and complete sequences:
`fr1_xyz`, `fr1_desk`, `fr1_floor`, `fr2_xyz`, `fr3_long_office`.
Three fresh processes per vocabulary and sequence, two concurrent runs,
OMP/BLAS thread counts one. These are asynchronous native SLAM runs and retain
run-to-run variance. A 1,800-second backstop is a failure if hit, not a shorter
trajectory to score as success.

The native example pairs each RGB frame with the closest depth timestamp
within 0.1s even in monocular mode and timestamps it at their average. This
filters 15 desk images: 598 loaded versus 613 RGB images. Completion is checked
against that native loader count; coverage retains all RGB frames as its
denominator, consistently with the existing comparison scorer.

Metrics reuse `tools/tum_eval.py`, hence `benchmark.ate_rmse` for Sim3
alignment and nearest GT association within 0.02s. Both tracked-only ATE and
hold-last-pose ATE, coverage, runtime, and keyframe ATE are retained per run.
Summary uses the median-ATE run of three and that same run's coverage, with
all three ATE ranges shown. A failed run prevents reporting a three-run median.
No canonical benchmark tables are rewritten and these measurements do not
establish a promoted baseline or a pure-C full-pipeline result.

## Completed full-sequence A/B (2026-09-30)

All **30 runs completed** the native loader's full input with exit code zero.
Both variants used identical binary/library hashes, configurations and thread
settings. These are fresh exploratory measurements, not the older published
comparison numbers.

| Sequence | Original ATE m / coverage | Own ATE m / coverage | Original ATE range | Own ATE range |
|---|---:|---:|---:|---:|
| fr1_xyz | 0.0267 / 99.6% | 0.0320 / 98.7% | 0.0132–0.0301 | 0.0148–0.0329 |
| fr1_desk | 0.0234 / 88.4% | 0.0181 / 88.6% | 0.0202–0.6658 | 0.0172–0.0201 |
| fr1_floor | 0.0233 / 95.7% | 0.0270 / 94.0% | 0.0222–0.1971 | 0.0215–0.1855 |
| fr2_xyz | 0.0066 / 97.1% | 0.0124 / 98.0% | 0.0042–0.0076 | 0.0043–0.0141 |
| fr3_long_office | 0.0222 / 99.4% | 0.0196 / 99.2% | 0.0221–0.0493 | 0.0139–0.0216 |

Mean of the five per-sequence median ATEs: **original 0.02044 m;
own 0.02183 m** (difference +0.00138 m). This is close measured accuracy,
not an accuracy improvement or proof of equivalence. The small sample and
large asynchronous outliers limit stronger conclusions. The original
vocabulary remains canonical.

Coverage deserves separate attention: own-vocabulary floor coverage ranges
from 86.9% to 94.0%, versus 95.7% to 97.8% for the original in these runs.
Own xyz coverage also ranges from 94.2% to 99.2%, versus 99.6% to 99.7%
for the original. Mean ATE alone would hide these differences. Runtime medians, tracked and
hold-last-pose ATE, keyframe ATE, coverage ranges and all individual runs are
retained in `runs/stella_port/vocab/ab/summary.json`; the table above selects
coverage from the same median-ATE run, not the best-coverage run.

The delivered result is a reproducible, attributed alternative with similar
ATE on this small TUM set. It resolves the old artifact's undocumented
training provenance for users choosing this alternative; it is not a claim
that all stella/dependency licensing questions or generalization tests are
resolved. There is no silent replacement in existing fixtures or benchmarks.

## Reproduction

```bash
python3 tools/vocab/fetch_training.py
python3 tools/vocab/build.py
python3 tools/vocab/train_release.py
python3 tools/vocab/inspect_vocab.py stella_port/vocab/own_orb_v1.fbow
/home/nybo/venvs/pose/bin/python3 tools/vocab/run_ab.py
/home/nybo/venvs/pose/bin/python3 tools/vocab/summarize_ab.py
```

The fetch/extraction retains archives and selected RGB images under the
reserved run directory. Native extraction requires the existing stella build
and locally extracted image-decoding libraries. The trained artifact is
separate from those dependencies; future C use loads it with `sv_bow_load_memory`.
Training refuses to overwrite an existing release. The A/B runner accepts
only matching cached hashes; failed runs are retained rather than silently
replaced.

Small-case validation is `python3 tools/vocab/test_trainer.py` after building
`check_compat` with `tools/vocab/build.py`. It checks deterministic training,
identical/few-unique descriptors, native/C compatibility and rejected malformed
input. Run artifact hashes and commands are retained with the results.
