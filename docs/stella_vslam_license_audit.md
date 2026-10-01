# stella_vslam license audit (2026-09-25)

Update (2026-10-01): the implemented C port now has a per-file SPDX inventory
and consolidated [NOTICE](../stella_port/NOTICE) / [license texts](../stella_port/LICENSES/).
These supersede the preliminary distribution conclusion below: the port
includes MPL-2.0 Eigen adaptations and an Apache-2.0 OpenCV fastAtan2 port,
as well as BSD/MIT components. OpenGV's BSD-3 notice is now retained for PnP.
The inspected g2o core/type source headers are BSD-2, not BSD-3. The own
vocabulary is complete and separately CC-BY-4.0 (see [record](stella_vocab.md)).
No declared GPL/LGPL implementation was identified in `stella_port/`, but
the libstdc++ behavior models require the provenance qualification recorded
in NOTICE; this pass does not certify the whole directory as GPL-free.
Validation: all 120 files in `stella_port/c` and the two production Eigen
PnP helper files have SPDX headers. A lexical comparison with the saved
pre-edit sources found no changed C tokens in any of those 122 files; the
20 harness source lists are unchanged. The full `sv_run` C driver builds
with the usual `-O2 -ffp-contract=off -fno-fast-math` flags and its only
direct shared-library dependencies are `libm.so.6` and `libc.so.6`.
Audit snapshots, hashes, the build command and scan results are under
`runs/stella_port/license_audit/`. This was a notices/comments-only pass;
no algorithm, benchmark registration or canonical benchmark result changed,
and no full benchmark sweep was run. Nothing was committed.

The original pre-port assessment follows for historical context.

Checked: `external/candidates/stella_vslam` @ e445b545 (package version 0.7.0).

| Item | License | Evidence | Port impact |
|---|---|---|---|
| stella_vslam core (`src/stella_vslam`) | BSD-2 (AIST 2019 `LICENSE.original` + stella-cv 2022 `LICENSE.fork`) | README: versions < 0.3 must be treated as GPL (ORB-SLAM2 derivative); "The similarities with ORB_SLAM2 in the original version have been removed by #252" — GitHub PR #252 "Remove GPL infringing code", merged 2022-02-06. 0.7.0 ≫ 0.3. | OK — port only from ≥ 0.3 sources (pin e445b545). Keep BSD-2 notices (AIST + stella-cv). |
| `solver/essential_5pt.h` | MIT (libmv) | README license list | OK, keep notice |
| `solver/pnp_solver.cc` | BSD-3 (OpenGV) | README | OK, keep notice |
| `feature/orb_extractor.cc`, `orb_point_pairs.h` | BSD-3 (OpenCV) | README | OK, keep notice |
| 3rd/FBoW, json, spdlog, tinycolormap | MIT | LICENSE files in 3rd/ | FBoW needed (BoW); json/spdlog/tinycolormap not needed in a C port |
| g2o (external dependency) | BSD core; csparse_extension LGPL-2.1+; viewer/incremental GPL; CHOLMOD GPL parts | g2o README | Do not port csparse_extension/CHOLMOD paths; write our own sparse/dense solver from BSD core semantics (stella uses dense/eigen solvers for its problem sizes — verify per call site) |
| ORB vocabulary `orb_vocab.fbow` (stella-cv/FBoW_orb_vocab) | repo says MIT | Only two commits ("Initial commit", "Add orb_vocab.fbow"); **training provenance undocumented** (may be converted from ORB-SLAM2's GPL `ORBvoc.txt`) | **Open.** Recommendation: train our own vocabulary (FBoW/DBoW-style k-means tree on ORB descriptors from a permissively licensed image set) and ship that; keep the stella file only for reference comparisons. |

## Rules for the port
1. Clean room with respect to ORB-SLAM2: port from stella_vslam ≥ 0.3 source and
   its dumps only. Never copy from `orb_port/` (GPL-derived), even where the
   algorithm is the same; our ORB port is used only to know *what questions to
   ask*, not as source text.
2. Preserve BSD-2/BSD-3/MIT notices in the ported files that derive from them.
3. Vocabulary: own trained vocabulary before any public release.
4. Stay away from g2o's LGPL/GPL modules.

Conclusion: a stella_vslam-derived pure-C port can be distributed under a
permissive license, conditional on (3). Not legal advice.
