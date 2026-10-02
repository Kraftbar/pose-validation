#!/usr/bin/env python3
"""fetch_seq_stream.py MH_01_easy [cam0,cam1,imu0,...] -> external/vio/data/<seq>/mav0/...

Like fetch_seq.py but never stores the 1.5 GB inner zip: the nested ASL zip is
parsed as a *stream* of local file headers (no central directory needed) and only
the requested sensor folders are written (default: cam0 + imu0 + sensor yamls),
so peak extra disk is the extracted payload only. EuRoC licence: non-commercial,
never commit the data.
"""
import io, os, struct, sys, zipfile, zlib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "external", "vio"))
from remotezip import HTTPFile

B = 'https://www.research-collection.ethz.ch/server/api/core/bitstreams/'
OUT = {'machine_hall': B + '7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/content',
       'vicon_room1': B + '02ecda9a-298f-498b-970c-b7c44334d880/content',
       'vicon_room2': B + 'ea12bc01-3677-4b4c-853d-87c7870b8c44/content'}


class Stream:
    def __init__(self, f):
        self.f, self.buf = f, b''

    def read(self, n):
        while len(self.buf) < n:
            d = self.f.read(1 << 22)
            if not d:
                break
            self.buf += d
        r, self.buf = self.buf[:n], self.buf[n:]
        return r

    def unread(self, d):
        self.buf = d + self.buf


def main():
    seq = sys.argv[1]
    want = (sys.argv[2] if len(sys.argv) > 2 else 'cam0,imu0').split(',')
    grp = {'MH': 'machine_hall', 'V1': 'vicon_room1', 'V2': 'vicon_room2'}[seq[:2]]
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "external", "vio", "data", seq)
    z = zipfile.ZipFile(io.BufferedReader(HTTPFile(OUT[grp]), buffer_size=1 << 22))
    s = Stream(z.open(f'{grp}/{seq}/{seq}.zip'))
    nfiles = 0
    while True:
        h = s.read(30)
        if len(h) < 30 or h[:4] != b'PK\x03\x04':
            break
        (_, flag, method, _, _, crc, csz, usz, nlen, elen) = struct.unpack('<HHHHHIIIHH', h[4:30])
        name = s.read(nlen).decode()
        s.read(elen)
        keep = (not name.endswith('/')) and any(('/' + w + '/') in name or name.endswith('sensor.yaml') and w in name
                                                  or name.endswith('body.yaml') for w in want)
        out = None
        if keep:
            p = os.path.join(root, name)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            out = open(p, 'wb')
        if method == 0 and not (flag & 8):
            left = usz
            while left:
                d = s.read(min(left, 1 << 22))
                if out:
                    out.write(d)
                left -= len(d)
        else:
            dec = zlib.decompressobj(-15)
            while not dec.eof:
                d = s.read(1 << 20)
                if not d:
                    break
                o = dec.decompress(d)
                if out:
                    out.write(o)
            s.unread(dec.unused_data)
            if flag & 8:  # data descriptor (optionally with signature)
                dd = s.read(4)
                if dd == b'PK\x07\x08':
                    s.read(12)
                else:
                    s.read(8)
        if out:
            out.close()
            nfiles += 1
    print('done', seq, nfiles, 'files')


if __name__ == '__main__':
    main()
