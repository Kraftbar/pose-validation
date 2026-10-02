#!/bin/bash
# run_okvis2x.sh <tag> <seq_dir> <cfg.yaml>   -> external/gnss/out/<tag>/{okvis2-*.csv, run.json}
G=/home/nybo/github/pose-validation/external/gnss
TAG=$1; SEQ=$2; CFG=$3
OUT=$G/out/$TAG; mkdir -p $OUT
source /home/nybo/github/pose-validation/external/vio/env.sh
t0=$(date +%s.%N)
$G/OKVIS2-X/build/okvis_app_synchronous $CFG $SEQ $OUT > $OUT/log.txt 2>&1
rc=$?
t1=$(date +%s.%N)
python3 -c "import json;json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'cfg':'$CFG','seq':'$SEQ'},open('$OUT/run.json','w'))"
rm -f $OUT/*.g2o $OUT/*final_map.csv
