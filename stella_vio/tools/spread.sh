#!/bin/bash
# SPDX-License-Identifier: MIT (project-authored benchmark tooling)
# spread.sh <tag> <seq> "<extra>" : same config under 7 perturbations of init_hamm (50..64) -> spread of the whole-run ATE (chaos of initialisation / loop-closure luck)
cd "$(dirname "$0")/../.."; PY=external/gnss/venv/bin/python; tag=$1; seq=$2; ex=$3; res=""
for h in 50 52 54 56 58 60 64; do ( $PY stella_vio/tools/run_eval.py sp_${tag}_h$h --seqs $seq --extra "--set init_hamm=$h $ex" 2>&1 | grep "^| $seq" | cut -d'|' -f3 > runs/stella_vio/sp_${tag}_h$h.res ) & done; wait
for h in 50 52 54 56 58 60 64; do res="$res $(cat runs/stella_vio/sp_${tag}_h$h.res | tr -d ' ')"; done
echo "$tag $seq:$res"
