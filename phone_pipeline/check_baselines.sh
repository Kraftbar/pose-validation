#!/bin/sh
# SPDX-License-Identifier: MIT (own code)
# Baseline checks for the opt-in interface changes made for the phone pipeline:
#  1. stella_vio exact-port check: fr1_xyz trajectory.tum of stella_vio/sv_run (reinit_sec=0 init_max_level=0 init_confirm=1) is cmp-identical to stella_port's sv_run
#     (built from stella_port/c exactly like tools/run_stella_port_replay.py does, into a temp dir; stella_port itself is not touched)
#  2. gnss_fusion: gf_table.py (608 numbers, section 11) and gf_gait_study.py fusion (section 12) re-run and compared with the saved JSONs of before the change
#     (gnss_fusion/work/pre15/{table,gait_fusion,georef_table}.json, saved before section 15), compare_py.py (C vs python) and test_geo.py.
# usage: check_baselines.sh <tmpdir>      (python with numpy: external/gnss/venv/bin/python)
set -e
T=${1:?tmpdir}
R=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-$R/external/gnss/venv/bin/python}
mkdir -p "$T/a" "$T/b"
cd "$R"
srcs=$(head -1 stella_port/c/sv_run.c | sed 's/.*SV_PORT_SOURCES: //; s/ \*\/.*//')
gcc -std=c99 -O2 -ffp-contract=off -fno-fast-math -o "$T/sv_run_port" $(for s in $srcs; do echo stella_port/c/$s; done) -lm 2>/dev/null
make -C stella_vio >/dev/null 2>&1
D=runs/orb_port/paper_bench/data/rgbd_dataset_freiburg1_xyz; V=external/candidates/orb_vocab.fbow; FX=runs/stella_port/fixtures/fr1_xyz
"$T/sv_run_port" $V $D $FX "$T/a" --no-snap >/dev/null 2>&1
stella_vio/sv_run $V $D $FX "$T/b" --no-snap --lean --set reinit_sec=0 --set init_max_level=0 --set init_confirm=1 >/dev/null 2>&1
echo "stella exact-port check: $(wc -l < "$T/a/trajectory.tum") / $(wc -l < "$T/b/trajectory.tum") poses"
cmp "$T/a/trajectory.tum" "$T/b/trajectory.tum" && echo "stella_vio vs stella_port fr1_xyz trajectory.tum: CMP-IDENTICAL"
make -C gnss_fusion/c >/dev/null
cd gnss_fusion
$PY tools/test_geo.py
$PY tools/test_georef.py   # new opt-in georef module (section 14)
$PY tools/compare_py.py complex_rtk complex_sim complex_rtk_blk complex_sim_blk o1_okvis o2_okvis o1_orb3mono o2_orb3mono
$PY tools/gf_table.py > "$T/gf_table.log" 2>&1
$PY tools/gf_gait_study.py fusion --workers 4 > "$T/gf_gait.log" 2>&1
$PY tools/gf_georef_table.py > "$T/gf_georef.log" 2>&1   # section 14 table (140 numbers), reference work/pre15/georef_table.json
$PY - <<EOF
import json
def flat(d,p=''):
    if isinstance(d,dict):
        for k,v in d.items(): yield from flat(v,p+'/'+str(k))
    elif isinstance(d,list):
        for i,v in enumerate(d): yield from flat(v,p+'[%d]'%i)
    elif isinstance(d,(int,float)) and not isinstance(d,bool): yield p,d
for name in ('table','gait_fusion','georef_table'):
    a=dict(flat(json.load(open('work/pre15/%s.json'%name)))); b=dict(flat(json.load(open('work/%s.json'%name))))
    n=0; bad=[]
    for k in a:
        if any(s in k for s in ('wall','timing','stdout')): continue
        n+=1
        if k not in b or abs(a[k]-b[k])>1e-9*max(1,abs(a[k])): bad.append((k,a[k],b.get(k)))
    print('%s.json: %d numbers compared, %d differ %s'%(name,n,len(bad),bad[:5]))
EOF
