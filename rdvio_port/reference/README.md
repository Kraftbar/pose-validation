# RD-VIO deterministic single-threaded reference

Mirrors `okvis_port/reference/`. Never edits `external/vio3/rd_vio`; `tools/build_rdvio_reference.py` extracts `git archive HEAD` of it to
`runs/rdvio_port/reference_build/src`, applies `patches/*.patch` in order, and builds `librdvio.a` (`-DTHREADING=OFF`,
`-O2 -DNDEBUG -ffp-contract=off -fno-fast-math`) plus `driver/rdvio_ref_driver.cpp` against Ceres 2.2.0 (pristine okvis2 submodule, built once with
`runs/rdvio_port/reference_build/build_ceres.sh`: EIGENSPARSE=ON, SUITESPARSE/CXSPARSE=OFF, CXX_THREADS, `-ffp-contract=off`), Eigen 3.4.0 and OpenCV 4.6.0 from
`external/vio/deps`. Provenance (upstream sha, patch sha256) in `runs/rdvio_port/reference_build/provenance.json`.

| patch | content |
|---|---|
| `0001-release-build-no-fastmath.patch` | Release `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math` instead of Debug `-Og -ffast-math -msse3 -mtune=native`; examples off; static lib; `<optional>` |
| `0002-ceres22-manifold.patch` | Ceres 1.14 `LocalParameterization` -> Ceres 2.2 `Manifold` |
| `0003-preintegration-dump.patch` | M1 instrumentation (`RDVIO_PORT_DUMP_DIR`, `RDVIO_PORT_DUMP_EVERY`), record layouts in `rdvio_port/c/rd_imu.h`; no numerical effect |
| `0004-m2-factor-geometry-dump.patch` | M2 instrumentation: reprojection / reprojection-prior / rotation-prior `Evaluate`, `solve_rotation_2pt`, `solve_essential_5pt`, `solve_homography_4pt`, `decompose_*`, `triangulate_point` (2-view, n-view), `Track::triangulate / triangulation_angle / get_ / set_landmark_point` (layouts in `rd_factor.h`, `rd_geom.h`); wrappers around renamed `*_impl` functions, no numerical effect |
| `0005-m3-ransac-dump.patch` | M3 instrumentation: `find_essential/rotation/homography_matrix[_parsac]` (inputs, model, mask) and a full op stream of `PoissonDiskFilter` (layouts in `rd_ransac.h`, `rd_poisson.h`); no numerical effect |

| `0006-m4-solver-dump.patch` | M4 instrumentation (rd_vio side): per-Solve() snapshot in `Solver::solve` (options, parameter / residual blocks with factor payloads, `RDVIO_PORT_SOLVE_DUMP_DIR / _EVERY / _FULL_EVERY`); needs the patched Ceres copy; layouts in `rdvio_port/c/rd_solve.h`; no numerical effect |
| `ceres_patches/0006-ceres-solver-dump.patch` | the Ceres-side hooks (adapted from okvis 0008, plus the sparse Schur system record), applied by `tools/build_rdvio_reference.py --ceres` to a `git archive` copy of the pristine okvis2 submodule (`reference_build/ceres-src`) and installed as `ceres-install-m4` (the pristine `ceres-install` stays) |
| `0007-m5-marginalization-dump.patch` | M5 instrumentation: channel `marg` (`RDVIO_PORT_DUMP_EVERY=marg=n`, `RDVIO_PORT_MARG_FULL_EVERY=n`) records the inputs / outputs (and every n-th call the stage matrices: information matrix after the Schur complements, eigenvalues, eigenvectors) of `CeresMarginalizationFactor::marginalize`; the 0006 solve snapshot gets the factor state as payload of type 6 (`port_payload`). Layouts in `rdvio_port/c/rd_marg.h`, `rd_solve.h`; no numerical effect |

Data: `VIO_DATA_ROOT=$PWD/runs/rdvio_port/data python3 tools/vio_harness/fetch_seq_stream.py MH_01_easy cam0,imu0` (~1.3 GB; delete `cam0/data` afterwards).
Solve dump (M4, ~280 MB on MH_01): `... --solve-dump --solve-every 5 --solve-full-every 4`. Run: `python3 tools/run_rdvio_reference.py MH_01_easy --tag run1 [--dump --dump-every "integ=1,pred=1,pie=100,plus=500"]`; score with
`external/gnss/venv/bin/python` (numpy). Check the C port: `python3 tools/check_rdvio_port.py [--modules m1,m2,m3,m4,m5] [--tag m1 --tag2 m23 --tag3 m23 --tag4 m5 --tag5 m5] --oracle [--sanitize]` (m5 dump: `run_rdvio_reference.py ... --dump --dump-every "integ=0,...,marg=1" --solve-dump --solve-every 5`; the run of tag m23 with every channel on carries all 617 marginalisations of MH_01).
Outputs, dumps and data are EuRoC-derived (non-commercial): keep under gitignored `runs/`, never commit.
