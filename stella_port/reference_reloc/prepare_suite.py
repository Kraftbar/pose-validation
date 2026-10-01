#!/usr/bin/env python3
"""Verify captured fixture hashes before exposing nonempty runner case lists."""
import json
from build import OUT,sha
for seq,expected in [('fr1_xyz',48),('fr1_desk',40)]:
 folder=OUT/'fixtures'/seq
 inputs=json.loads((folder/'inputs.json').read_text())
 for frame,h in inputs['maps'].items():assert sha(folder/(frame+'.map'))==h
 for kind in ['relocalization','pnp']:
  dest=folder/kind;ref=json.loads((dest/'reference.json').read_text());cases=ref['cases'];assert cases
  if kind=='relocalization':assert len(cases)==expected
  lines=[]
  for c in cases:
   assert sha(dest/c['trace'])==c['sha256']
   assert sha(dest/c['trace'].replace('.pass1.','.pass2.'))==c['sha256']
   if kind=='pnp':
    assert sha(dest/c['input'])==c['input_sha256']
    lines.append(f"{c['input']} {c['trace']} {c['recompute']}\n")
   else:
    for p,h in c['pnp_inputs'].items():assert sha(dest/p)==h
    lines.append(f"{c['frame']} {c['mode']} {c['trace']}\n")
  (dest/'cases.txt').write_text(''.join(lines))
  print(seq,kind,len(cases),'verified')
