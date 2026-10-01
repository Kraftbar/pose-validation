# Own ORB vocabulary, v1

`own_orb_v1.fbow` is an independently trained FBoW-compatible ORB vocabulary.
Keep `own_orb_v1.json` and `ATTRIBUTION.md` with it when redistributing.

- Vocabulary/data: **CC BY 4.0**, attribution required.
- New trainer/tools: **MIT**, see `LICENSE-CODE.txt`.
- 486,271 words; 38,128,064 bytes; no descriptors or weights from the old
  vocabulary were used in training.
- Training: 962 held-out-recording RGB images from TUM
  `freiburg2_large_no_loop` and `freiburg3_teddy`.

[Full method, validation and A/B results](../../docs/stella_vocab.md).
The canonical vocabulary and pure-C port harnesses still use the original
artifact. V1 is a separate candidate, not a silently promoted replacement.

For a native stella run, supply `-v /absolute/path/to/own_orb_v1.fbow`.
For the C port, load its bytes through the existing `sv_bow_load_memory` API.

Fresh native A/B: 30 complete trials across five TUM sequences. Mean of
per-sequence median ATEs was 0.02183 m (own) versus 0.02044 m (original).
Coverage regressed on floor and in one xyz trial; see the full report.
These results do not justify automatic promotion. Nothing committed.
