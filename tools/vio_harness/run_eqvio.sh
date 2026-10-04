#!/bin/bash
# run_eqvio.sh <seq> [config]   EqVIO (GPL-3, reference only; executed, never copied) mono-inertial, ASL mode. EuRoC -> runs/vio_compare/eqvio_mono/<seq>/ ; phone -> external/vio3/phone_out/<seq>_eqvio/
# Wrapper dataset dir with symlinks (+ dummy ground-truth file the reader insists on, + generated cam0/sensor.yaml for phones). Output IMUState.csv -> TUM.
set -u
R=/home/nybo/github/pose-validation; E=$R/external/vio3/eqvio; source $R/external/vio/env.sh; PY=$R/external/gnss/venv/bin/python
SEQ=$1; CFG=${2:-$E/configs/EQVIO_config_EuRoC_stationary.yaml}
if [[ $SEQ == MH_* || $SEQ == V* ]]; then D=$R/external/vio3/euroc/$SEQ/mav0; OUT=$R/runs/vio_compare/eqvio_mono/$SEQ; TUM=$OUT/trajectory.tum; SENS=$D/cam0/sensor.yaml
elif [[ $SEQ == drone_* ]]; then
  N=${SEQ#drone_}; case $N in m14) DD=ins_m14; CD=ins_m14_cfg;; o1) DD=ins_o1; CD=ins_o1_cfg;; of5) DD=of5; CD=fpv/of5_cfg;; esac
  D=$R/external/vio3/drone/$DD/mav0; OUT=$R/runs/drone_compare/$N/eqvio_mono; TUM=$OUT/trajectory.tum; SENS=$OUT/sensor.yaml; mkdir -p $OUT; $PY $R/tools/vio_harness/eqvio/make_sensor.py $R/external/drone/$CD/okvis2_mono.yaml $SENS
else D=$R/external/vio3/phone/$SEQ/mav0; OUT=$R/external/vio3/phone_out/${SEQ}_eqvio; TUM=$OUT/traj.txt; SENS=$OUT/sensor.yaml; mkdir -p $OUT; $PY $R/tools/vio_harness/eqvio/make_sensor.py $SEQ $SENS; fi
mkdir -p $OUT; W=$OUT/ds; rm -rf $W; mkdir -p $W/mav0/cam0 $W/mav0/imu0 $W/mav0/state_groundtruth_estimate0
ln -s $D/cam0/data $W/mav0/cam0/data; ln -s $D/cam0/data.csv $W/mav0/cam0/data.csv; cp $SENS $W/mav0/cam0/sensor.yaml; ln -s $D/imu0/data.csv $W/mav0/imu0/data.csv
echo '#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []' > $W/mav0/state_groundtruth_estimate0/data.csv
t0=$(date +%s.%N)
/usr/bin/time -f 'CPU_USER %U CPU_SYS %S MAXRSS_KB %M' -o $OUT/time.txt $E/build/eqvio_opt $W/ $CFG --mode asl -o $OUT/res ${EQ_EXTRA:-} > $OUT/log.txt 2>&1
rc=$?; t1=$(date +%s.%N)
$PY - <<P
import json, csv
rows=[]
try:
    for r in list(csv.reader(open('$OUT/res/IMUState.csv')))[1:]:
        v=[float(x) for x in r[:8]]
        if v[0]<1e8: continue
        rows.append([v[0],v[1],v[2],v[3],v[5],v[6],v[7],v[4]])   # t p qx qy qz qw (file: qw qx qy qz)
except Exception as e: print('no IMUState', e)
open('$TUM','w').write(''.join(' '.join(repr(x) for x in r)+'\n' for r in rows))
tt=open('$OUT/time.txt').read().strip().splitlines()[-1].split()
json.dump({'wall_s': $t1-$t0, 'exit_code': $rc, 'cpu_s': float(tt[1])+float(tt[3]), 'maxrss_mb': int(tt[5])/1024, 'frame': 'body', 'notes': 'EqVIO mono-inertial (GPL ref), cfg=$(basename $CFG)'}, open('$OUT/run.json','w'))
print(len(rows),'poses')
P
rm -rf $OUT/ds
