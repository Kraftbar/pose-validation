#!/bin/bash
# run_orbslam3.sh <seq> <mono_inertial|stereo_inertial|mono>   -> runs/vio_compare/orbslam3_<mode>/<seq>
set -u
V=/home/nybo/github/pose-validation/external/vio; R=/home/nybo/github/pose-validation/runs/vio_compare
source $V/env.sh
SEQ=$1; MODE=$2
O=$V/orbslam3; E=$O/Examples
case $SEQ in MH_0*) TS=MH0${SEQ:4:1};; V1_0*) TS=V10${SEQ:4:1};; V2_0*) TS=V20${SEQ:4:1};; esac
OUT=$R/orbslam3_$MODE/$SEQ; mkdir -p $OUT; W=$(mktemp -d); cd $W
t0=$(date +%s.%N)
case $MODE in
 mono_inertial)  $E/Monocular-Inertial/mono_inertial_euroc $O/Vocabulary/ORBvoc.txt $E/Monocular-Inertial/EuRoC.yaml $V/data/$SEQ $E/Monocular-Inertial/EuRoC_TimeStamps/$TS.txt run > $OUT/log.txt 2>&1;;
 stereo_inertial) $E/Stereo-Inertial/stereo_inertial_euroc $O/Vocabulary/ORBvoc.txt $E/Stereo-Inertial/EuRoC.yaml $V/data/$SEQ $E/Stereo-Inertial/EuRoC_TimeStamps/$TS.txt run > $OUT/log.txt 2>&1;;
 mono)           $E/Monocular/mono_euroc $O/Vocabulary/ORBvoc.txt $E/Monocular/EuRoC.yaml $V/data/$SEQ $E/Monocular/EuRoC_TimeStamps/$TS.txt run > $OUT/log.txt 2>&1;;
esac
rc=$?
t1=$(date +%s.%N)
ls $W > $OUT/files.txt
FR=body; [ $MODE = mono ] && FR=cam0
cp $W/f_run.txt $OUT/f_run.txt 2>/dev/null; cp $W/kf_run.txt $OUT/kf_run.txt 2>/dev/null
python3 - <<P
import json
json.dump({'wall_s':$t1-$t0,'exit_code':$rc,'frame':'$FR','notes':'ORB-SLAM3 $MODE (realtime pacing removed, viewer stubbed)'},open('$OUT/run.json','w'))
P
cp $W/f_run.txt $OUT/trajectory.tum 2>/dev/null
rm -rf $W
