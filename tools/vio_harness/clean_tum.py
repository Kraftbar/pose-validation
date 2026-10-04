#!/usr/bin/env python3
"""clean_tum.py <trajectory.tum>... : drop non-finite / malformed rows (diverged filters print nan/inf; unflushed final line) so the shared scorers can run. Reports how many rows were dropped."""
import sys, math
for p in sys.argv[1:]:
    ls = open(p).read().splitlines(); ok = []
    for l in ls:
        v = l.split()
        try:
            x = [float(a) for a in v]
        except ValueError:
            continue
        if len(x) == 8 and x[0] > 1e8 and all(math.isfinite(a) for a in x) and (x[4]**2+x[5]**2+x[6]**2+x[7]**2) > 0.25 and abs(x[1]) < 1e6 and abs(x[2]) < 1e6 and abs(x[3]) < 1e6: ok.append(l)
    if len(ok) != len(ls): print(p, 'dropped', len(ls) - len(ok), 'of', len(ls))
    open(p, 'w').write('\n'.join(ok) + ('\n' if ok else ''))
