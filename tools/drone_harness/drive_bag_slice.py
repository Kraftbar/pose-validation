#!/usr/bin/env python3
"""Fetch a time window of a (huge) public Google Drive ROS1 bag by HTTP range requests: read the bag index at the end of the file,
download only the chunks overlapping [t_a, t_b] (seconds since the bag start), and write them as a local sequential bag (no index) that
tools/gnss_harness/bag_head.py can read. usage: drive_bag_slice.py <file_id> <out.bag> <t_a> <t_b> [--list]"""
import sys, struct, time, urllib.request
fid, out, ta, tb = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
URL = f'https://drive.usercontent.google.com/download?id={fid}&export=download&confirm=t'

def get(a, b, tries=3000):
    for k in range(tries):
        try:
            d = urllib.request.urlopen(urllib.request.Request(URL, headers={'Range': f'bytes={a}-{b}'}), timeout=180).read()
            if d[:5] == b'<!DOC' or len(d) != b - a + 1: raise RuntimeError(f'bad {len(d)}')
            return d
        except Exception as e:
            print('retry', a, e, flush=True); time.sleep(3)
    raise SystemExit('giving up')

def fields(hd):
    d = {}; i = 0
    while i < len(hd):
        l = struct.unpack('<I', hd[i:i + 4])[0]; k, v = hd[i + 4:i + 4 + l].split(b'=', 1); d[k.decode('latin1')] = v; i += 4 + l
    return d

head = get(0, 4095)
hl = struct.unpack('<I', head[13:17])[0]; h = fields(head[17:17 + hl])
index_pos = struct.unpack('<Q', h['index_pos'])[0]; nconn = struct.unpack('<I', h['conn_count'])[0]; nchunk = struct.unpack('<I', h['chunk_count'])[0]
# total size via a 1-byte range probe
req = urllib.request.Request(URL, headers={'Range': 'bytes=0-0'}); r = urllib.request.urlopen(req, timeout=60); total = int(r.headers['Content-Range'].split('/')[1]); r.read()
tail = get(index_pos, total - 1)
i = 0; conns = []; chunks = []
while i < len(tail):
    hl = struct.unpack('<I', tail[i:i + 4])[0]; d = fields(tail[i + 4:i + 4 + hl]); dl = struct.unpack('<I', tail[i + 4 + hl:i + 8 + hl])[0]
    body = tail[i + 8 + hl:i + 8 + hl + dl]; i += 8 + hl + dl; op = d['op'][0]
    if op == 7: conns.append(tail[i - 8 - hl - dl - 0:i])
    elif op == 6:
        cp = struct.unpack('<Q', d['chunk_pos'])[0]; s, ns = struct.unpack('<II', d['start_time']); e, ne = struct.unpack('<II', d['end_time'])
        chunks.append((cp, s + ns * 1e-9, e + ne * 1e-9))
chunks.sort(); t0 = chunks[0][1]
print('chunks', len(chunks), 'bag span', chunks[-1][2] - t0, 'index_pos', index_pos, 'total', total)
sel = []
for k, (cp, s, e) in enumerate(chunks):
    end = chunks[k + 1][0] if k + 1 < len(chunks) else index_pos
    if '--probe' in sys.argv:
        if (s - t0) % 10 < 0.5: sel.append((cp, end))
    elif e - t0 >= ta and s - t0 <= tb: sel.append((cp, end))
print('selected', len(sel), 'chunks', sum(b - a for a, b in sel) / 1e9, 'GB')
if '--list' in sys.argv: raise SystemExit
with open(out, 'wb') as f:
    f.write(head[:13]); 
    # minimal file-header record (op=3) padded; reader skips it
    hdr = b'op=\x03'; f.write(struct.pack('<I', len(hdr)) + hdr + struct.pack('<I', 0))
    for cp, end in sel:
        pos = cp
        while pos < end:
            e2 = min(pos + 16 * 2**20, end) - 1
            f.write(get(pos, e2)); pos = e2 + 1
        print('chunk', cp, flush=True)
print('done', out)
