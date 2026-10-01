#!/bin/bash
# run_okvis2.sh <seq> <stereo|mono>   -> runs/vio_compare/okvis2_<mode>_slam/<seq>
set -u
V=/home/nybo/github/pose-validation/external/vio; R=/home/nybo/github/pose-validation/runs/vio_compare
source $V/env.sh
SEQ=$1; MODE=${2:-stereo}
CFG=$V/okvis2/config/euroc.yaml; [ $MODE = mono ] && CFG=$V/okvis_mono_euroc.yaml
OUT=$R/okvis2_${MODE}_slam/$SEQ; mkdir -p $OUT
rm -f $V/data/$SEQ/mav0/okvis2-*
t0=$(date +%s.%N)
$V/okvis2/build/okvis_app_synchronous $CFG $V/data/$SEQ/mav0 > $OUT/log.txt 2>&1
rc=$?
t1=$(date +%s.%N)
cp $V/data/$SEQ/mav0/okvis2-slam-final_trajectory.csv $OUT/final.csv 2>/dev/null
cp $V/data/$SEQ/mav0/okvis2-slam_trajectory.csv $OUT/causal.csv 2>/dev/null
rm -f $V/data/$SEQ/mav0/okvis2-*
python3 -c "
import sys
out=open('$OUT/trajectory.tum','w')
for l in open('$OUT/final.csv'):
    p=[x.strip() for x in l.split(',')]
    if not p[0].isdigit(): continue
    out.write(' '.join([str(int(p[0])*1e-9)]+p[1:8])+'\\n')
"
python3 - <<P
import json
json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'frame':'body','notes':'$MODE VI-SLAM (loop closure on), final trajectory'},open('$OUT/run.json','w'))
P
