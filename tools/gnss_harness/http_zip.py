"""Minimal seekable HTTP range file (own code) for reading members of a remote zip (Zenodo). Default urllib UA (Zenodo 403s browser-like UAs on ranges)."""
import io, urllib.request, time
class HTTPFile(io.RawIOBase):
    def __init__(s, url, chunk=16 << 20):
        s.url, s.pos, s.chunk, s.cache = url, 0, chunk, None
        r = urllib.request.Request(url, headers={'Range': 'bytes=0-0'})
        s.size = int(urllib.request.urlopen(r, timeout=60).headers['Content-Range'].split('/')[1])
    def seekable(s): return True
    def readable(s): return True
    def tell(s): return s.pos
    def seek(s, o, w=0):
        s.pos = o if w == 0 else s.pos + o if w == 1 else s.size + o
        return s.pos
    def readinto(s, b):
        if s.pos >= s.size: return 0
        c = s.cache
        if not c or not (c[0] <= s.pos < c[0] + len(c[1])):
            st = s.pos; e = min(st + s.chunk, s.size) - 1
            for t in range(20):
                try:
                    d = urllib.request.urlopen(urllib.request.Request(s.url, headers={'Range': f'bytes={st}-{e}'}), timeout=300).read(); break
                except Exception as ex:
                    time.sleep(5)
            s.cache = c = (st, d)
        o = s.pos - c[0]; n = min(len(b), len(c[1]) - o)
        b[:n] = c[1][o:o + n]; s.pos += n
        return n


class ZipMemberStream:
    """Sequential file-like (read/tell/seek(n,1)) over a DEFLATE member of a remote zip: one ranged GET + zlib; nothing is stored on disk."""
    def __init__(s, url, header_offset, max_out=None):
        import zlib, struct
        r = urllib.request.urlopen(urllib.request.Request(url, headers={'Range': f'bytes={header_offset}-{header_offset + 4095}'}), timeout=60).read()
        nl, el = struct.unpack('<HH', r[26:30]); s.start = header_offset + 30 + nl + el
        s.resp = urllib.request.urlopen(urllib.request.Request(url, headers={'Range': f'bytes={s.start}-'}), timeout=300)
        s.d = zlib.decompressobj(-15); s.buf = b''; s.pos = 0; s.eof = False
    def _fill(s, n):
        while len(s.buf) < n and not s.eof:
            c = s.resp.read(4 << 20)
            if not c: s.eof = True; break
            s.buf += s.d.decompress(c)
    def read(s, n):
        s._fill(n); out, s.buf = s.buf[:n], s.buf[n:]; s.pos += len(out); return out
    def tell(s): return s.pos
    def seek(s, o, w=1):
        assert w == 1
        while o > 0:
            k = min(o, 64 << 20); got = s.read(k); o -= len(got)
            if not got: break
        return s.pos
