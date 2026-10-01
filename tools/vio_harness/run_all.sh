#!/bin/bash
# run_all.sh <seq>: all systems serially on one EuRoC ASL sequence in data/<seq>
V=/home/nybo/github/pose-validation/external/vio
S=$1
cd /tmp/bw
for m in vio vo; do $V/run_basalt.sh $S $m; done
for m in stereo mono; do $V/run_okvis2.sh $S $m; done
for m in stereo mono; do $V/run_openvins.sh $S $m; done
for m in mono_inertial stereo_inertial mono; do $V/run_orbslam3.sh $S $m; done
$V/run_stella.sh $S
echo ALLDONE $S
