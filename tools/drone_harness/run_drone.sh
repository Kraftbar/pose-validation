#!/bin/bash
# run_drone.sh <system> <seq_dir> <cfg_dir> <out_dir>
# systems: okvis2_stereo okvis2_mono okvis2x_mono_gnss okvis2x_mono_nognss basalt_stereo orb_stereo_inertial orb_mono_inertial stella_mono
# seq_dir = EuRoC layout (mav0/{cam0,cam1,imu0,gps0}); cfg_dir from calib.py; writes <out_dir>/<system>/{trajectory.tum,run.json,log.txt}
set -u
SYS=$1; SEQ=$2; CFG=$3; OUTB=$4
V=/home/nybo/github/pose-validation/external/vio; G=/home/nybo/github/pose-validation/external/gnss; C=/home/nybo/github/pose-validation/external/candidates
source $V/env.sh
OUT=$OUTB/$SYS; mkdir -p $OUT; cp $CFG/*$SYS*.yaml $CFG/basalt_calib.json $OUT/ 2>/dev/null
W=$(mktemp -d); NFR=$(($(wc -l < $SEQ/mav0/cam0/data.csv)-1))
t0=$(date +%s.%N)
case $SYS in
 okvis2_stereo|okvis2_mono)
   rm -f $SEQ/mav0/okvis2-*
   timeout 3600 $V/okvis2/build/okvis_app_synchronous $CFG/$SYS.yaml $SEQ/mav0 > $OUT/log.txt 2>&1; rc=$?
   cp $SEQ/mav0/okvis2-slam-final_trajectory.csv $OUT/final.csv 2>/dev/null; cp $SEQ/mav0/okvis2-slam_trajectory.csv $OUT/causal.csv 2>/dev/null
   rm -f $SEQ/mav0/okvis2-*; SRC=$OUT/final.csv;;
 okvis2x_mono_gnss|okvis2x_mono_nognss)
   mkdir -p $W/o; timeout 3600 $G/OKVIS2-X/build/okvis_app_synchronous $CFG/$SYS.yaml $SEQ/mav0 $W/o > $OUT/log.txt 2>&1; rc=$?
   ls $W/o > $OUT/files.txt; cp $W/o/okvis2-*final_trajectory.csv $OUT/ 2>/dev/null; SRC=$(ls $OUT/okvis2-slam-final_trajectory.csv 2>/dev/null || ls $OUT/*final_trajectory.csv | head -1)
   for gf in $W/o/okvis2-*-global-final_trajectory.csv; do [ -f "$gf" ] && cp $gf $OUT/global_final_raw.csv; done;;
 basalt_stereo)
   B=$V/basalt/release; export LD_LIBRARY_PATH=$B/lib:${LD_LIBRARY_PATH:-}; cd $W
   timeout 3600 $B/bin/basalt_vio --dataset-path $SEQ --cam-calib $CFG/basalt_calib.json --dataset-type euroc --config-path $B/data/euroc_config.json --show-gui 0 --use-imu 1 --num-threads 4 --save-trajectory tum --result-path $OUT/result.txt > $OUT/log.txt 2>&1; rc=$?
   mv -f $W/trajectory.txt $OUT/trajectory.tum 2>/dev/null; SRC=;;
 orb_stereo_inertial|orb_mono_inertial)
   O=$V/orbslam3; cd $W; cut -d, -f1 $SEQ/mav0/cam0/data.csv | grep -v '#' > times.txt
   if [ $SYS = orb_stereo_inertial ]; then EX=$O/Examples/Stereo-Inertial/stereo_inertial_euroc; else EX=$O/Examples/Monocular-Inertial/mono_inertial_euroc; fi
   timeout 3600 $EX $O/Vocabulary/ORBvoc.txt $CFG/$SYS.yaml $SEQ times.txt run > $OUT/log.txt 2>&1; rc=$?
   cp $W/f_run.txt $OUT/trajectory.tum 2>/dev/null; SRC=;;
 stella_mono)
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$C/deps/root/usr/lib:$C/deps/root/usr/lib/x86_64-linux-gnu
   timeout 3600 $V/stella_build/run_euroc_slam -v $C/orb_vocab.fbow -d $SEQ/mav0 -c $CFG/stella_mono.yaml --no-sleep --auto-term --viewer none --eval-log-dir $OUT --log-level warn > $OUT/log.txt 2>&1; rc=$?
   cp $OUT/frame_trajectory.txt $OUT/trajectory.tum 2>/dev/null; SRC=;;
esac
t1=$(date +%s.%N)
if [ -n "${SRC:-}" ] && [ -f "$SRC" ]; then
python3 - <<P
out=open('$OUT/trajectory.tum','w')
for l in open('$SRC'):
    p=[x.strip() for x in l.split(',')]
    if not p[0].isdigit(): continue
    out.write(' '.join([str(int(p[0])*1e-9)]+p[1:8])+'\n')
P
fi
python3 -c "import json;json.dump({'system':'$SYS','wall_s':$t1-$t0,'exit_code':$rc,'n_frames':$NFR},open('$OUT/run.json','w'))"
rm -rf $W; rm -f $OUT/*.g2o
