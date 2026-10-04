#!/bin/bash
# run_xrslam.sh <seq> [noise]   EuRoC seq (MH_01_easy ...) -> runs/vio_compare/xrslam_mono/<seq>/ ; phone seq (indoor1 indoor2 outdoor1 outdoor2 advio15 advio20) -> external/vio2/phone_out/<seq>_xrslam[_infl]/
# XRSLAM (Apache-2.0) built under external/vio2/xrslam (see tools/vio_harness/xrslam/README in vio_harness README); headless driver main_headless.cpp
set -u
R=/home/nybo/github/pose-validation; X=$R/external/vio2/xrslam; source $R/external/vio/env.sh
SEQ=$1; NOISE=${2:-default}
if [[ $SEQ == MH_* || $SEQ == V* ]]; then
  D=$R/external/vio/data/$SEQ/mav0; OUT=$R/runs/vio_compare/xrslam_mono/$SEQ; SC=$X/configs/euroc_slam.yaml; DC=$X/configs/euroc_sensor.yaml; TUM=$OUT/trajectory.tum
else
  D=$R/external/vio2/phone/$SEQ/mav0; T=xrslam; [ $NOISE != default ] && T=xrslam_$NOISE
  OUT=$R/external/vio2/phone_out/${SEQ}_$T; SC=$X/configs/iphone_slam.yaml; DC=$OUT/sensor.yaml; TUM=$OUT/traj.txt
  mkdir -p $OUT; $R/external/gnss/venv/bin/python $R/tools/vio_harness/xrslam/make_cfg.py $SEQ $DC $NOISE
fi
mkdir -p $OUT
if [[ $SEQ != MH_* && $SEQ != V* ]]; then   # the XRSLAM EuRoC reader needs CRLF csv files
  S=$OUT/ds; rm -rf $S; mkdir -p $S/cam0 $S/imu0; ln -s $D/cam0/data $S/cam0/data
  sed 's/$/\r/' $D/cam0/data.csv > $S/cam0/data.csv; sed 's/$/\r/' $D/imu0/data.csv > $S/imu0/data.csv; D=$S
fi
t0=$(date +%s.%N)
/usr/bin/time -f 'CPU_USER %U CPU_SYS %S MAXRSS_KB %M' -o $OUT/time.txt $X/build/xrslam-pc/player/xrslam-pc-player -sc $SC -dc $DC --tum $TUM euroc://$D > $OUT/log.txt 2>&1
rc=$?; t1=$(date +%s.%N)
$R/external/gnss/venv/bin/python - <<P
import json
tt=open('$OUT/time.txt').read().strip().splitlines()[-1].split()
json.dump({'wall_s': $t1-$t0, 'exit_code': $rc, 'cpu_s': float(tt[1])+float(tt[3]), 'maxrss_mb': int(tt[5])/1024, 'frame': 'body', 'notes': 'XRSLAM mono-inertial (Apache-2.0), noise=$NOISE, no loop closure'}, open('$OUT/run.json','w'))
P
tail -2 $OUT/log.txt
