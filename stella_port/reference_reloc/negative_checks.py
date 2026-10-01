#!/usr/bin/env python3
"""The published harness must reject missing, truncated, and corrupt evidence."""
import json,subprocess,tempfile
from pathlib import Path
from build import OUT
source=OUT/'fixtures/fr1_xyz/pnp'
case=json.loads((source/'reference.json').read_text())['cases'][0]
results=[]
with tempfile.TemporaryDirectory(prefix='sv-pnp-negative-') as temp:
 base=Path(temp);folder=base/'pnp';folder.mkdir()
 (folder/case['input']).write_bytes((source/case['input']).read_bytes())
 original=(source/case['trace']).read_bytes()
 listing=f"{case['input']} {case['trace']} {case['recompute']}\n"
 for name in ['empty_cases','missing_trace','truncated_trace','changed_value']:
  (folder/'cases.txt').write_text('' if name=='empty_cases' else listing)
  if name=='truncated_trace':(folder/case['trace']).write_bytes(original[:-1])
  if name=='changed_value':
   import struct
   offset=8+struct.unpack_from('<I',original)[0]
   changed=bytearray(original);changed[offset]^=1;(folder/case['trace']).write_bytes(changed)
  p=subprocess.run([str(OUT/'check_sv_pnp_validation'),'negative',str(base)],capture_output=True,text=True)
  assert p.returncode!=0,name
  results.append(dict(case=name,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
(OUT/'negative_checks.json').write_text(json.dumps(results,indent=2)+'\n')
print('4 negative fixtures rejected')
