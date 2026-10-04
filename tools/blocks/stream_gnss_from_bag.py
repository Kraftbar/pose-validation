#!/usr/bin/env python3
"""Stream the GVINS-Dataset complex_environment bag over HTTP and keep only /ublox_driver/* raw messages (never stores the bag).
Own code; reuses tools/gnss_harness/bag_head.iter_msgs (raw mode) through a forward-only file wrapper.
usage: stream_gnss_from_bag.py out.pkl [max_seconds=440]"""
import sys, pickle, urllib.request, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'gnss_harness'))
from bag_head import iter_msgs
URL = 'https://huggingface.co/datasets/Shawn202606/GVINS-Dataset/resolve/main/complex_environment.bag'

class Fwd:
    def __init__(s, r): s.r, s.pos = r, 0
    def read(s, n):
        b = bytearray()
        while len(b) < n:
            c = s.r.read(n - len(b))
            if not c: break
            b += c
        s.pos += len(b); return bytes(b)
    def tell(s): return s.pos
    def seek(s, n, w): assert w == 1; s.read(n)

out, maxs = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 440.0
f = Fwd(urllib.request.urlopen(URL, timeout=120))
data, cds, t0, n = {}, {}, None, 0
# topics=None -> all topics; filter ourselves (raw)
for topic, t, body, cd in iter_msgs(f, None, raw=True, size=1 << 62):
    if t0 is None: t0 = t
    if (t - t0) * 1e-9 > maxs: break
    n += 1
    if n % 20000 == 0: print(n, round((t - t0) * 1e-9, 1), f.pos / 1e6, 'MB', flush=True)
    if topic.startswith('/ublox_driver'):
        data.setdefault(topic, []).append((t, body)); cds[topic] = {k: v for k, v in cd.items()}
pickle.dump({'data': data, 'cd': cds, 't0': t0}, open(out, 'wb'))
print('done', {k: len(v) for k, v in data.items()}, f.pos / 1e6, 'MB')
