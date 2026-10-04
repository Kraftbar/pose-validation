#!/usr/bin/env python3
"""CPU-seconds / wall / exit code of every classical-candidate run (phones: external/vio3/phone_out, drones: runs/drone_compare/<seq>/<sys>/run.json) -> runs/gnss_compare/more_systems2/cpu.md.
CPU seconds, not wall time, because the machine is shared (load average 25-55 during these runs)."""
import json
from pathlib import Path
R = Path('/home/nybo/github/pose-validation')
rows = []
for d in sorted((R / 'external/vio3/phone_out').glob('*/')):
    if (d / 'run.json').exists():
        j = json.loads((d / 'run.json').read_text()); rows.append((d.name, j))
for d in sorted((R / 'runs/drone_compare').glob('*/*/')):
    if d.name.split('_')[0] in ('msceqf', 'rdvio', 'eqvio', 'rovio') and (d / 'run.json').exists():
        rows.append((f'drone_{d.parent.name}_{d.name}', json.loads((d / 'run.json').read_text())))
L = ['| run | CPU s | wall s | exit code | max RSS MB |', '|---|---|---|---|---|']
for n, j in rows: L.append(f"| {n} | {j.get('cpu_s', 0):.0f} | {j['wall_s']:.0f} | {j['exit_code']} | {j.get('maxrss_mb', 0):.0f} |")
(R / 'runs/gnss_compare/more_systems2/cpu.md').write_text('\n'.join(L) + '\n'); print('\n'.join(L))
