#!/usr/bin/env python3
"""Download the first N bytes of a public Google Drive file in 256 MB range chunks (resumable, retries on 'quota exceeded' html).
usage: fetch_drive_head.py <file_id> <out> <n_bytes>"""
import sys, os, time, urllib.request
fid, out, n = sys.argv[1], sys.argv[2], int(float(sys.argv[3]))
CH = 256 * 2**20
pos = os.path.getsize(out) if os.path.exists(out) else 0
while pos < n:
    end = min(pos + CH, n) - 1
    req = urllib.request.Request(f'https://drive.usercontent.google.com/download?id={fid}&export=download&confirm=t', headers={'Range': f'bytes={pos}-{end}'})
    try:
        d = urllib.request.urlopen(req, timeout=120).read()
        if d[:5] == b'<!DOC' or len(d) != end - pos + 1: raise RuntimeError(f'bad chunk {len(d)}')
    except Exception as e:
        print('retry', pos, e, flush=True); time.sleep(20); continue
    with open(out, 'ab') as f: f.write(d)
    pos += len(d); print(pos / 1e9, flush=True)
