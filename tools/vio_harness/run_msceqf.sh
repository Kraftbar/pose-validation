#!/bin/bash
# run_msceqf.sh <seq> [cfg]   EuRoC seq (MH_01_easy ...) -> runs/vio_compare/msceqf_mono/<seq>/ ; phone seq (indoor1 indoor2 outdoor1 outdoor2 advio15 advio20) -> external/vio3/phone_out/<seq>_msceqf/
# MSCEqF (Apache-2.0, aau-cns) built under external/vio3/MSCEqF/build; driver tools/vio_harness/msceqf/main_headless.cpp
set -u
R=/home/nybo/github/pose-validation; B=$R/external/vio3/MSCEqF/build; source $R/external/vio/env.sh; PY=$R/external/gnss/venv/bin/python
SEQ=$1; CFG=${2:-}; NOISE=${NOISE:-default}   # NOISE=infl -> OKVIS-tuned phone IMU densities (phones only)
if [[ $SEQ == MH_* || $SEQ == V* ]]; then
  D=$R/external/vio3/euroc/$SEQ/mav0; OUT=$R/runs/vio_compare/${SYSNAME:-msceqf_mono}/$SEQ; TUM=$OUT/trajectory.tum; MAXS=${MAXS:-1e18}
  [ -z "$CFG" ] && CFG=$R/external/vio3/MSCEqF/examples/euroc/config/config.yaml   # upstream stock EuRoC config (ZVU enabled); zvu-off variant: tools/vio_harness/msceqf/cfg/euroc_zvu_off.yaml
elif [[ $SEQ == drone_* ]]; then   # drone_m14 | drone_o1 | drone_of5 (INSANE mars_14 / outdoor_1 first 100 s / UZH-FPV outdoor_forward_5)
  MAXS=${MAXS:-1e18}; N=${SEQ#drone_}; case $N in m14) DD=ins_m14; CD=ins_m14_cfg;; o1) DD=ins_o1; CD=ins_o1_cfg;; of5) DD=of5; CD=fpv/of5_cfg;; esac
  D=$R/external/vio3/drone/$DD/mav0; OUT=$R/runs/drone_compare/$N/${SYSNAME:-msceqf_mono}; TUM=$OUT/trajectory.tum; mkdir -p $OUT; CFG=$OUT/config.yaml
  $PY $R/tools/vio_harness/msceqf/make_cfg.py $R/external/drone/$CD/okvis2_mono.yaml $CFG okvis
else
  T=msceqf; [ $NOISE != default ] && T=msceqf_$NOISE; D=$R/external/vio3/phone/$SEQ/mav0; OUT=$R/external/vio3/phone_out/${SEQ}_$T; TUM=$OUT/traj.txt; MAXS=${MAXS:-1e18}
  mkdir -p $OUT; CFG=$OUT/config.yaml; $PY $R/tools/vio_harness/msceqf/make_cfg.py $SEQ $CFG $NOISE
fi
mkdir -p $OUT
t0=$(date +%s.%N)
/usr/bin/time -f 'CPU_USER %U CPU_SYS %S MAXRSS_KB %M' -o $OUT/time.txt $B/msceqf_headless $CFG $D $TUM $MAXS > $OUT/log.txt 2>&1
rc=$?; t1=$(date +%s.%N)
$PY - <<P
import json
tt=open('$OUT/time.txt').read().strip().splitlines()[-1].split()
json.dump({'wall_s': $t1-$t0, 'exit_code': $rc, 'cpu_s': float(tt[1])+float(tt[3]), 'maxrss_mb': int(tt[5])/1024, 'frame': 'body', 'notes': 'MSCEqF mono-inertial (Apache-2.0), cfg=$(basename $CFG), no loop closure'}, open('$OUT/run.json','w'))
P
grep -a "FRAMES\|what()" $OUT/log.txt | tail -2
