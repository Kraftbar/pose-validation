#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import json
from common import ROOT,OUT,sha
sources=json.loads((OUT/'training/sources.json').read_text());records=[]
for seq in sources:
 folder=OUT/'training'/seq['sequence']
 frames=[line.split() for line in (folder/'rgb.txt').read_text().splitlines() if line.strip() and not line.startswith('#')]
 for i in range(0,len(frames),6):
  p=folder/frames[i][1];records.append(dict(sequence=seq['sequence'],index=i,timestamp=frames[i][0],path=str(p),sha256=sha(p)))
(OUT/'training_images.txt').write_text(''.join(r['path']+'\n' for r in records))
(OUT/'training_images.json').write_text(json.dumps(dict(stride=6,max_descriptors_per_image=1200,images=records),indent=2)+'\n')
print(len(records),'training images')
