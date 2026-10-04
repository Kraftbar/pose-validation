#!/bin/bash
# run_rdvio.sh <seq> [noise]   EuRoC seq -> runs/vio_compare/rdvio_mono/<seq>/ ; phone seq -> external/vio3/phone_out/<seq>_rdvio[_infl]/
# RD-VIO = Jianxff/rd_vio (Apache-2.0, plain-CMake split of xrslam; IMU-PARSAC on by default in its setting.yaml). Build: external/vio3/rd_vio/build (-DTHREADING=OFF), driver tools/vio_harness/rdvio/main_headless.cpp
set -u
R=/home/nybo/github/pose-validation; RV=$R/external/vio3/rd_vio; source $R/external/vio/env.sh; PY=$R/external/gnss/venv/bin/python
SEQ=$1; NOISE=${2:-default}; SC=${SETTING:-$RV/configs/setting.yaml}
if [[ $SEQ == MH_* || $SEQ == V* ]]; then
  D=$R/external/vio3/euroc/$SEQ/mav0; OUT=$R/runs/vio_compare/rdvio_mono/$SEQ; DC=$RV/configs/euroc_sensor.yaml; TUM=$OUT/trajectory.tum
elif [[ $SEQ == drone_* ]]; then
  N=${SEQ#drone_}; case $N in m14) DD=ins_m14; CD=ins_m14_cfg;; o1) DD=ins_o1; CD=ins_o1_cfg;; of5) DD=of5; CD=fpv/of5_cfg;; esac
  D=$R/external/vio3/drone/$DD/mav0; OUT=$R/runs/drone_compare/$N/rdvio_mono${TAG:+_$TAG}; DC=$OUT/sensor.yaml; TUM=$OUT/trajectory.tum; mkdir -p $OUT
  $PY $R/tools/vio_harness/rdvio/make_cfg.py $R/external/drone/$CD/okvis2_mono.yaml $DC okvis
else
  D=$R/external/vio3/phone/$SEQ/mav0; T=rdvio; [ $NOISE != default ] && T=rdvio_$NOISE; [ -n "${TAG:-}" ] && T=${T}_$TAG
  OUT=$R/external/vio3/phone_out/${SEQ}_$T; DC=$OUT/sensor.yaml; TUM=$OUT/traj.txt
  mkdir -p $OUT; $PY $R/tools/vio_harness/rdvio/make_cfg.py $SEQ $DC $NOISE
fi
mkdir -p $OUT
t0=$(date +%s.%N)
/usr/bin/time -f 'CPU_USER %U CPU_SYS %S MAXRSS_KB %M' -o $OUT/time.txt $RV/build/rdvio_headless $DC $SC $D $TUM ${MAXS:-1e18} > $OUT/log.txt 2>&1
rc=$?; t1=$(date +%s.%N)
$PY - <<P
import json
tt=open('$OUT/time.txt').read().strip().splitlines()[-1].split()
json.dump({'wall_s': $t1-$t0, 'exit_code': $rc, 'cpu_s': float(tt[1])+float(tt[3]), 'maxrss_mb': int(tt[5])/1024, 'frame': 'body', 'notes': 'RD-VIO rd_vio mono-inertial (Apache-2.0), noise=$NOISE, setting=$(basename $SC), no loop closure'}, open('$OUT/run.json','w'))
P
grep -a "FRAMES" $OUT/log.txt | tail -1
