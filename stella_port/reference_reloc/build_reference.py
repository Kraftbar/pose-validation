#!/usr/bin/env python3
"""Rebuild isolated observers, without changing the shared native build."""
import subprocess,sys
from pathlib import Path
from build import build,ROOT,env
HERE=Path(__file__).resolve().parent
for script in ['instrument.py','guard_pnp.py','guard_pnp_empty.py','instrument_reloc.py','extract_tracking_glue.py']:
 subprocess.run([sys.executable,str(HERE/script)],check=True)
profile=build('check_blocking',['stella_port/reference_reloc/check_blocking.cc'])
check=subprocess.run([str(profile)],env=env(),capture_output=True,text=True,check=True)
if not check.stdout.startswith('cache 32768 ') or 'mr 4 nr 4' not in check.stdout.splitlines()[0]:
 raise RuntimeError('Reference Eigen cache/kernel profile differs from the pinned C evaluation: '+check.stdout)
pnp='runs/stella_port/reference_reloc/instrumented/pnp_solver.cc'
build('dump_pnp',['stella_port/reference_reloc/dump_pnp.cc',pnp])
build('dump_reloc',['stella_port/reference_reloc/dump_reloc.cc',pnp,'runs/stella_port/reference_reloc/instrumented/relocalizer.cc'])
