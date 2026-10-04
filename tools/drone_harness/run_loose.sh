#!/bin/bash
# run_loose.sh <runs_dir> <vio_system> <seq_dir> [--causal 30]  -> <runs_dir>/loose_<vio_system>/{trajectory.tum,global_final.csv,run.json}
set -u; R=$1; S=$2; SEQ=$3; shift 3; O=$R/loose_$S; mkdir -p $O
t0=$(date +%s.%N)
python3 /home/nybo/github/pose-validation/tools/gnss_loose_fusion.py --traj $R/$S/trajectory.tum --gps $SEQ/mav0/gps0/data.csv --rsa 0,0,0 --out $O/trajectory.tum "$@" > $O/log.txt 2>&1; rc=$?
t1=$(date +%s.%N); cp $O/trajectory.tum $O/global_final.csv
python3 -c "import json;json.dump({'system':'loose_$S','wall_s':$t1-$t0,'exit_code':$rc},open('$O/run.json','w'))"
