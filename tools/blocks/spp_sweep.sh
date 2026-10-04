#!/bin/bash
# sweep of RTKLIB SPP options on the GVINS complex_environment raw GNSS; results table -> runs/blocks/spp_sweep.md
cd /home/nybo/github/pose-validation
PY=external/gnss/venv/bin/python; B=external/blocks/build/rtklib_spp; T=external/blocks/gnss_txt; O=runs/blocks/spp
echo "| config | n | H rms | H med | H p95 | U rms | Vh med | Vh p68 | Vh p95 | V3d rms | nsat |"
echo "|---|---|---|---|---|---|---|---|---|---|---|"
for cfg in "sys=G elmin=15" "sys=GEC elmin=15" "sys=GEC elmin=15 snrmin=30" "sys=GEC elmin=15 snrmin=35" "sys=GEC elmin=25" "sys=GEC elmin=15 fde=1" "sys=GEC elmin=15 snrmin=35 fde=1" "sys=GECR elmin=15 snrmin=35" "sys=GEC elmin=30 snrmin=35"; do
  n=$(echo $cfg | tr ' =' '__'); $B $T $O/sw_$n.txt mode=l1 $cfg 2>/dev/null
  $PY tools/blocks/spp_score.py $O/sw_$n.txt --row "$cfg"
done
