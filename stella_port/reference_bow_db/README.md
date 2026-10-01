# BoW database / candidate lookup

Independent C leaf for stella_vslam `data::bow_database` at commit
`e445b545`. Uses the existing `sv_bow_score`; no vocabulary/scoring rewrite,
Eigen port, initializer dependency, map-model dependency or ORB-SLAM2 source.
Derived C files retain the AIST/stella-cv BSD-2 notices. FBoW remains in the
existing MIT-marked `sv_bow.c`.

## Contract

`sv_bow_db.{h,c}` implements the inverted file, insertion, first-match erasure,
clear, shared-word counting, rejection, common-word threshold and scoring.
The caller supplies stable keyframe wrappers containing a unique ID and a
borrowed immutable `sv_bow_vector`. No file I/O, OpenCV, C++ or external
library is used by the C leaf; only C99/libc/libm.

Semantics worth preserving during integration:

- Add appends to each word's posting list. It **does not deduplicate** a
  repeated insertion of the same object. Common-word counts include these
  duplicates. Erase removes only the first matching ID from each word's list
  and retains empty word buckets until clear.
- Rejection uses canonical object identity. Erasure uses keyframe ID.
- The threshold is `uint32_t(float(ratio * max_common_words))`. A candidate
  must have **strictly more** common words than this threshold.
- `bow_vocabulary_util::score` returns `float`, although FBoW itself returns
  `double`. The C leaf explicitly narrows `sv_bow_score` before comparing it
  with `min_score`. Equality with `min_score` passes. Scores are compared by
  their exact 32-bit representation in the fixtures.
- No covisibility score accumulation, temporal filtering or bad-keyframe
  filtering is invented here: the pinned database does none of these.
- This is a single-threaded leaf. The caller serializes concurrent use.
  Invalid input/allocation errors return -1; queries leave the previous
  result intact on error. Result rows are owned; their keyframe pointers are
  borrowed. Free results with `sv_bow_db_result_free`.

## Candidate-order integration gate

Upstream builds the return vector from an
`unordered_set<shared_ptr<keyframe>>`. The C API deliberately defines its
result rows in **ascending keyframe ID**. Candidate membership, shared-word
counts, thresholds, score bits, best score, and posting-list insertion order
match the reference. **Raw candidate traversal order is not ported.**

The observer retains the actual upstream vector in `raw_order{1,2}.tsv`;
only the canonical comparison is ID-sorted. Both fresh upstream processes
happened to produce the same raw order on these fixtures, but it differs
from ID order in 20 synthetic, 265 xyz and 57 desk queries. For example,
synthetic query 2 returns `[3,4,4294967295,2,1]` upstream.

Do not silently wire the ID-sorted output into a claim of exact continuous
relocalization/loop-detection parity. Stella's relocalizer consumes the
candidate vector and can stop at the first success. The coordinator must
choose and validate a traversal policy at that integration boundary (or
explicitly reproduce the reference's container order). This leaf establishes
exact lookup membership/numerics, not order-equivalent downstream behavior.

## Reference and fixtures

`dump_main.cc` calls the **real already-built stella library** and real
`keyframe::make_keyframe` objects with supplied BoW vectors. The keyframes
have empty geometric observations because this database never reads them.
The FBoW scoring adapter ignores the vocabulary pointer, so no vocabulary
load or replacement scoring implementation is needed.

An observer subclass exposes the existing protected counting/scoring methods
and reads posting lists. No upstream code is patched. The observer checks
its intermediate score map against the actual `acquire_keyframes` return
membership. Original source hashes for the database and scoring adapter are
checked against the source used by the installed reference build.

The standalone builder reuses module 2's compiler/link flags, links the
installed reference read-only, and writes only under
`runs/stella_port/bow_db/`. Provenance records the upstream commit, commands,
compiler, explicit linked library files and installed stella/FBoW headers,
source hashes and binary
hash. The reference's existing dependencies stay reference-only; they are
not linked into or distributed with the C leaf.

Fixtures use all **798 xyz and 598 desk** frame BoW vectors captured by module
2. These are controlled database lifecycles using real frame vectors, **not
a replay of the SLAM system's actual keyframe-insertion history**: every
seventh frame is inserted, at most 24 distinct keyframes remain indexed,
and every frame supplies queries. Additional queries reject recent keys;
periodic duplicate insert/erase operations and complete posting snapshots
exercise persistent state. Erase-all and clear conclude each sequence.

Synthetic coverage includes empty/disjoint vectors, zero-weight support,
32-bit maximum word/keyframe IDs, ratio 0/1/>1, strict common-word boundaries,
min-score equality and adjacent floats, all/self/duplicate rejection, repeated
add/erase, missing-ID erase, retained empty buckets, clear and re-add.

Each fixture is run in **two fresh native processes**. Canonical output bytes
match for all three. Raw-order logs remain available alongside them; they
are not discarded or silently asserted equivalent to the C order.

## Reproduce

From the repository root, with the existing reference install and module-2
BoW fixtures present:

```bash
python3 tools/dump_stella_bow_db.py
python3 tools/check_stella_bow_db.py
python3 tools/check_stella_bow_db.py --sanitize
```

The dump/check commands refuse existing output directories. For another
run, use `--out <fresh-directory>`; the checker also accepts
`--fixtures <fixture-root>`. `--skip-build` reuses the isolated observer.
No shared reference rebuild is performed. Full evidence is under
`runs/stella_port/bow_db/{build,fixtures,checks,final_sanitized,negative_checks}`.

The published `check_sv_bow_db.c` has the existing `SV_PORT_SOURCES` header
and runner ABI. It finds its fixture at
`<stella-run-root>/bow_db/fixtures/<sequence>/`. Missing fixtures fail; new
sequences need corresponding BoW database fixtures before the shared runner
can include this leaf. The runner's optional max-frames argument does not
truncate this lifecycle test. No changes were made to the shared runner.

## Results

| Case | Input vectors | Database queries | Exact serialized rows checked | Result |
|---|---:|---:|---:|---|
| Synthetic | 7 | 67 | 401 | PASS |
| fr1_xyz | 798 | 1642 | 1,362,931 | PASS |
| fr1_desk | 598 | 1232 | 982,453 | PASS |

All membership, intermediate and posting-snapshot rows match without
tolerances. An additional 20 API checks cover invalid inputs, identity-based
rejection, ID-based erasure and query error-state preservation. Normal and
ASan+UBSan builds pass; no sanitizer diagnostics. Leak detection is disabled
for the previously documented ptrace limitation; leaks were not checked.

The negative checks reject an altered score bit (exit 1), a zero-query input
(exit 2), and a missing reference file (exit 2). The published harness also
passes on desk through the exact shared-runner argument shape. Native build
inputs retain their before-build hashes. No initializer files, shared C types,
shared reference files or canonical benchmark outputs were modified. No ATE
or full tracking claim is made, and nothing was committed.

## Fixture text protocol

`commands.txt` uses whitespace-separated records:

- `K id n (word_id hex_float_weight)...`: define a stable keyframe/query vector.
- `A id`, `E id`, `C`: add, erase, clear.
- `Q query_id vector_id min_score ratio n_reject rejected_ids...`: query.
- `S`: snapshot the complete inverted file, including empty buckets.

Canonical output contains `Q query_id max_common threshold best_score_bits
n_common n_accepted`, followed by ID-sorted `R id common_words scored
score_bits accepted` rows. A snapshot is `S n_buckets` followed by word-ID
sorted `W word_id n_entries keyframe_ids...` rows with **original posting
order and multiplicity**. Hex score fields encode float bits, not rounded
decimal strings. The checker compares the complete serialized stream and
rejects empty coverage, missing/trailing rows and malformed command input.
