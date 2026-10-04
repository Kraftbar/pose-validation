#!/bin/bash
# run_kimera.sh <seq>  Kimera-VIO (BSD-2) mono-inertial through the stereoVIOEuroc binary with the EurocMono params (cam1 symlinked to cam0, ignored in mono mode).
# Builds: external/vio2/Kimera-VIO/build (GTSAM 4.2, OpenGV, DBoW2, Kimera-RPGO; a no-op opencv2/viz stub, since viz needs VTK). EuRoC -> runs/vio_compare/kimera_mono/<seq>/ ; phone -> external/vio2/phone_out/<seq>_kimera/
# EuRoC: initialised at the GT pose (autoInitialize 0, the stock protocol). Phones: autoInitialize 1 (gravity alignment), dummy GT file.
R=/home/nybo/github/pose-validation; SEQ=$1; source $R/external/vio/env.sh; V=$R/external/vio2; K=$V/Kimera-VIO; C=$R/external/gnss/mamba/envs/ros
export LD_LIBRARY_PATH=$VROOT/lib/x86_64-linux-gnu:$V/install-4.2/lib:$OCV/lib:$C/lib
PY=$R/external/gnss/venv/bin/python
if [[ $SEQ == MH_* || $SEQ == V* ]]; then SRC=$R/external/vio/data/$SEQ/mav0; OUT=$R/runs/vio_compare/kimera_mono/$SEQ; TUM=$OUT/trajectory.tum; else SRC=$V/phone/$SEQ/mav0; OUT=$V/phone_out/${SEQ}_kimera; TUM=$OUT/traj.txt; fi
mkdir -p $OUT/o; DS=$V/kdata/$SEQ; rm -rf $DS; mkdir -p $DS/mav0/{cam0,imu0,state_groundtruth_estimate0}; M=$DS/mav0
ln -s $SRC/cam0/data $M/cam0/data; cp $SRC/cam0/data.csv $M/cam0/; ln -s cam0 $M/cam1; cp $SRC/imu0/data.csv $M/imu0/
printf '%%YAML:1.0\nsensor_type: visual-inertial\nT_BS:\n  cols: 4\n  rows: 4\n  data: [1.0, 0.0, 0.0, 0.0,\n         0.0, 1.0, 0.0, 0.0,\n         0.0, 0.0, 1.0, 0.0,\n         0.0, 0.0, 0.0, 1.0]\n' > $M/state_groundtruth_estimate0/sensor.yaml
cp $K/params/EurocMono/*.yaml $OUT/ 2>/dev/null; P=$OUT/params; rm -rf $P; cp -r $K/params/EurocMono $P
$PY $R/tools/vio_harness/kimera_prep.py $SEQ $P $M
t0=$(date +%s.%N)
/usr/bin/time -f 'CPU_USER %U CPU_SYS %S MAXRSS_KB %M' -o $OUT/time.txt $K/build/stereoVIOEuroc --flagfile=$P/flags/stereoVIOEuroc.flags --flagfile=$P/flags/Mesher.flags --flagfile=$P/flags/VioBackend.flags --flagfile=$P/flags/RegularVioBackend.flags --flagfile=$P/flags/Visualizer3D.flags --logtostderr=1 --log_prefix=0 --dataset_path=$DS --params_folder_path=$P --initial_k=1 --final_k=10000000 --vocabulary_path=$K/vocabulary/ORBvoc.yml --use_lcd=0 --visualize=false --dataset_type=0 --log_output=1 --output_path=$OUT/o/ > $OUT/log.txt 2>&1
rc=$?; t1=$(date +%s.%N)
$PY - <<P2
import json
rows=[]
try:
    for l in open('$OUT/o/traj_vio.csv'):
        if l[0]=='#': continue
        p=l.split(','); rows.append([int(p[0])*1e-9, *map(float,p[1:4]), float(p[5]),float(p[6]),float(p[7]),float(p[4])])
except Exception as e: print(e)
open('$TUM','w').write(''.join(' '.join(repr(x) for x in r)+'\n' for r in rows))
tt=open('$OUT/time.txt').read().strip().splitlines()[-1].split()
json.dump({'wall_s': $t1-$t0, 'exit_code': $rc, 'cpu_s': float(tt[1])+float(tt[3]) if len(tt)>3 else None, 'frame': 'body', 'notes': 'Kimera-VIO mono-inertial (BSD-2), regular VIO backend, no loop closure'}, open('$OUT/run.json','w'))
print(len(rows),'poses')
P2
rm -rf $OUT/o/frontend_images $DS
