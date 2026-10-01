# Training-data attribution

The vocabulary is derived from RGB images in the **TUM RGB-D SLAM Dataset
and Benchmark**, by Jürgen Sturm, Nikolas Engelhard, Felix Endres,
Wolfram Burgard and Daniel Cremers. Please retain this attribution with
`own_orb_v1.fbow` and its metadata.

Dataset source and license statement (checked 2026-09-30):
https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license

The dataset provider specifies **CC BY 4.0**. The vocabulary artifact is
provided under CC BY 4.0 with the same attribution; it is not represented as
BSD/MIT-only data. License: https://creativecommons.org/licenses/by/4.0/
Legal text: https://creativecommons.org/licenses/by/4.0/legalcode.en

Training sequences: `freiburg2_large_no_loop` and `freiburg3_teddy`.
Their original download links, archive hashes, selected image hashes,
extraction parameters and vocabulary hash are recorded in the manifests.
The five comparison sequences are excluded from training. They belong to
the same benchmark collection: this is a split by recording, not a claim
of independent buildings or environments.

Changes: sample every sixth RGB frame; convert images to grayscale; extract
stella ORB descriptors; cap each image at 1,200 uniformly selected descriptors;
cluster those descriptors and calculate smoothed document-frequency weights.
The artifact contains binary cluster centers and weights, not RGB/depth images.
Neither the authors nor TUM endorse this vocabulary or these measurements.

Publication: J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers,
“A Benchmark for the Evaluation of RGB-D SLAM Systems”, IROS, 2012.

The independent trainer and accompanying new tools use the MIT license in
`LICENSE-CODE.txt`. They do not load the old vocabulary during training.
Extraction and compatibility tools link the existing stella, OpenCV and FBoW
libraries; their existing licenses apply to those dependencies. The trainer
itself uses only the C standard library and libm. FBoW file-layout compatibility
was implemented from FBoW's MIT-licensed format definitions; its notice is
retained in `NOTICE-FBOW.txt`.
