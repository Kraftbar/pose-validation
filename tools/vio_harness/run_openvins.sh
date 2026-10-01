#!/bin/bash
# run_openvins.sh <seq> <mono|stereo>  -> runs/vio_compare/openvins_<mode>/<seq>
set -u
V=/home/nybo/github/pose-validation/external/vio; R=/home/nybo/github/pose-validation/runs/vio_compare
source $V/env.sh
SEQ=$1; MODE=$2
OUT=$R/openvins_$MODE/$SEQ; mkdir -p $OUT
t0=$(date +%s.%N)
$V/open_vins/ov_msckf/build/run_euroc $V/ov_cfg/$MODE/estimator_config.yaml $V/data/$SEQ/mav0 $OUT/trajectory.tum > $OUT/log.txt 2>&1
rc=$?
t1=$(date +%s.%N)
python3 - <<P
import json
json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'frame':'body','notes':'OpenVINS MSCKF $MODE (no loop closure; custom EuRoC driver)'},open('$OUT/run.json','w'))
P
