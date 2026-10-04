#!/bin/bash
# run_rovio.sh <seq> [rate]   ROVIO (ethz-asl, BSD-style licence) mono-inertial via ROS1 (robostack env external/gnss/mamba/envs/ros), data replayed from the EuRoC-layout folder by play_folder.py at wall-clock rate (default 1.0).
# EuRoC -> runs/vio_compare/rovio_mono/<seq>/ ; phone -> external/vio3/phone_out/<seq>_rovio/ ; drone_<n> -> runs/drone_compare/<n>/rovio_mono/
set +u
R=/home/nybo/github/pose-validation; W=$R/external/vio3/ws_rovio; SEQ=$1; RATE=${2:-1.0}; PORT=$((11800 + RANDOM % 300)); PY=$R/external/gnss/venv/bin/python
if [[ $SEQ == MH_* || $SEQ == V* ]]; then D=$R/external/vio3/euroc/$SEQ/mav0; OUT=$R/runs/vio_compare/rovio_mono/$SEQ; TUM=$OUT/trajectory.tum; CD=$OUT/cfg; mkdir -p $CD; cp $R/external/vio3/rovio/cfg/rovio.info $R/external/vio3/rovio/cfg/euroc_cam0.yaml $CD/; mv $CD/euroc_cam0.yaml $CD/cam0.yaml
elif [[ $SEQ == drone_* ]]; then N=${SEQ#drone_}; case $N in m14) DD=ins_m14; CF=ins_m14_cfg;; o1) DD=ins_o1; CF=ins_o1_cfg;; of5) DD=of5; CF=fpv/of5_cfg;; esac
  D=$R/external/vio3/drone/$DD/mav0; OUT=$R/runs/drone_compare/$N/rovio_mono; TUM=$OUT/trajectory.tum; CD=$OUT/cfg; $PY $R/tools/vio_harness/rovio/make_cfg.py $R/external/drone/$CF/okvis2_mono.yaml $CD
else D=$R/external/vio3/phone/$SEQ/mav0; OUT=$R/external/vio3/phone_out/${SEQ}_rovio; TUM=$OUT/traj.txt; CD=$OUT/cfg; $PY $R/tools/vio_harness/rovio/make_cfg.py $SEQ $CD; fi
mkdir -p $OUT
source $R/external/gnss/rosenv.sh; source $W/devel_isolated/setup.bash 2>/dev/null || source $W/install_isolated/setup.bash
export ROS_MASTER_URI=http://localhost:$PORT ROS_HOME=$OUT/rosdata
roscore -p $PORT > $OUT/roscore.log 2>&1 & RC=$!; sleep 4
python3 $R/tools/vio_harness/rovio/record_odom.py $TUM > $OUT/rec.log 2>&1 & RR=$!
$W/devel_isolated/rovio/lib/rovio/rovio_node _filter_config:=$CD/rovio.info _camera0_config:=$CD/cam0.yaml > $OUT/log.txt 2>&1 & RV=$!; sleep 3
t0=$(date +%s.%N)
python3 $R/tools/vio_harness/gpl_glue/play_folder.py $D $RATE ${MAXS:-1e9} > $OUT/play.log 2>&1
sleep 5; t1=$(date +%s.%N)
echo "cpu_ticks $(awk '{print $14+$15}' /proc/$RV/stat) hz $(getconf CLK_TCK)" > $OUT/cpu.txt
kill $RV; sleep 1; kill -INT $RR; sleep 2; kill $RC; sleep 1
$PY - <<P
import json
tk=open('$OUT/cpu.txt').read().split()
ok=len(tk)==4 and tk[1].isdigit()   # rovio_node gone before the cpu read = it crashed (segfault)
json.dump({'wall_s': $t1-$t0, 'exit_code': 0 if ok else 139, 'cpu_s': int(tk[1])/int(tk[3]) if ok else 0, 'frame': 'body', 'notes': 'ROVIO mono-inertial (BSD-style), play rate $RATE, no loop closure'}, open('$OUT/run.json','w'))
P
wc -l $TUM
