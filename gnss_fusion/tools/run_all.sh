#!/bin/sh
# SPDX-License-Identifier: MIT (own code)
# Rebuild and re-run every validation / study (needs external/gnss data from the GNSS-VIO study; python = a numpy env, e.g. external/gnss/venv).
set -e
PY=${PY:-/home/nybo/github/pose-validation/external/gnss/venv/bin/python}
cd "$(dirname "$0")/.."
make -C c
$PY tools/test_geo.py
$PY tools/compare_py.py complex_rtk complex_sim complex_rtk_blk complex_sim_blk o1_okvis o2_okvis o1_orb3mono o2_orb3mono
$PY tools/gf_table.py
$PY tools/gf_studies.py outliers
$PY tools/gf_studies.py loss
$PY tools/gf_studies.py gaps
$PY tools/gf_studies.py drone
$PY tools/gf_studies.py blackout
$PY tools/gf_studies.py velocity
$PY tools/check_gait.py
$PY tools/gf_gait_study.py all --workers 8
$PY tools/gf_gait_report.py > /dev/null
