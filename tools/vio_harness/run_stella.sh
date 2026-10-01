#!/bin/bash
# run_stella.sh <seq> -> runs/vio_compare/stella_mono/<seq>  (camera-only baseline, loop closure on)
set -u
V=/home/nybo/github/pose-validation/external/vio; R=/home/nybo/github/pose-validation/runs/vio_compare
C=/home/nybo/github/pose-validation/external/candidates
source $V/env.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$C/deps/root/usr/lib:$C/deps/root/usr/lib/x86_64-linux-gnu
SEQ=$1
OUT=$R/stella_mono/$SEQ; mkdir -p $OUT
t0=$(date +%s.%N)
timeout 1500 $V/stella_build/run_euroc_slam -v $C/orb_vocab.fbow -d $V/data/$SEQ/mav0 -c $C/stella_vslam/example/euroc/EuRoC_mono.yaml --no-sleep --auto-term --viewer none --eval-log-dir $OUT --log-level warn > $OUT/log.txt 2>&1
rc=$?
t1=$(date +%s.%N)
cp $OUT/frame_trajectory.txt $OUT/trajectory.tum 2>/dev/null
python3 - <<P
import json
json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'frame':'cam0','notes':'stella_vslam mono (camera only, LC on)'},open('$OUT/run.json','w'))
P
