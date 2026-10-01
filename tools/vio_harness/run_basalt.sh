#!/bin/bash
# run_basalt.sh <seq> [vio|vo]   -> runs/vio_compare/basalt_stereo_{vio,vo}/<seq>
set -u
V=/home/nybo/github/pose-validation/external/vio; R=/home/nybo/github/pose-validation/runs/vio_compare
SEQ=$1; MODE=${2:-vio}
B=$V/basalt/release; export LD_LIBRARY_PATH=$B/lib:${LD_LIBRARY_PATH:-}
OUT=$R/basalt_stereo_$MODE/$SEQ; mkdir -p $OUT
IMU=1; [ $MODE = vo ] && IMU=0
t0=$(date +%s.%N)
$B/bin/basalt_vio --dataset-path $V/data/$SEQ --cam-calib $B/data/euroc_ds_calib.json --dataset-type euroc --config-path $B/data/euroc_config.json --show-gui 0 --use-imu $IMU --num-threads 4 --save-trajectory tum --result-path $OUT/result.txt > $OUT/log.txt 2>&1
rc=$?
t1=$(date +%s.%N)
mv -f trajectory.txt $OUT/trajectory.tum 2>/dev/null; rm -f stats_*.ubjson
python3 - <<P
import json
json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'frame':'body','notes':'stereo, prebuilt release 0.1.7' + (', IMU off (VO)' if $IMU==0 else '')},open('$OUT/run.json','w'))
P
