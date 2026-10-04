#!/bin/bash
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
# study_all.sh <tag> "<extra sv_run args>" ["seq seq .."] [--imu] : multi-start robustness windows (default: the 7 study sequences; mh01 / v102 also known);
# prints "success/windows" per sequence and the total. --imu adds the IMU arguments of run_eval.IMU_CFG for the sequences that have them.
cd "$(dirname "$0")/../.."; PY=external/gnss/venv/bin/python; tag=$1; ex=$2; seqs=${3:-"fr1_xyz fr1_desk fr1_floor fr2_xyz fr3_long_office outdoor1 complex"}; imu=${4:-}; tot=0; win=0; totseg=0; line=""
run() { s=$1; shift; out=$($PY stella_vio/tools/init_study.py st_${tag}_$s $s "$@" $imu ${BIN:+--bin $BIN} ${WORKERS:+--workers $WORKERS} --extra "$ex" 2>&1 | tail -n1); a=$(echo "$out" | sed 's/^[^:]*: success \([0-9]*\)\/\([0-9]*\).*/\1 \2/'); sg=$(echo "$out" | sed 's/.*segment-scored success \([0-9]*\)\/.*/\1/'); set -- $a; tot=$((tot+$1)); win=$((win+$2)); totseg=$((totseg+${sg:-$1})); line="$line $s=$1/$2(seg ${sg:-$1})"; }
for s in $seqs; do case $s in
  fr1_xyz|fr1_desk|fr1_floor|fr2_xyz|fr3_long_office) run $s --n 300 --step 100 --ok 0.05 --cov 0.6;;
  outdoor1) run $s --n 450 --step 150 --ok 2.0 --cov 0.6;;
  complex) run $s --n 600 --step 300 --ok 1.0 --cov 0.6;;
  mh01|v102) run $s --n 600 --step 300 --ok 1.0 --cov 0.6;;
esac; done
echo "$tag:$line  TOTAL $tot/$win (segment-scored $totseg/$win)"
