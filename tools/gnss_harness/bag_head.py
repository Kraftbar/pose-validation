"""Sequential ROS1 bag reader that tolerates a truncated file (HTTP-range-downloaded head of a big bag).
Own code (pure python: lz4 + rosbags typestore for deserialisation). Usage: for topic, t_ns, msg in iter_msgs(path, topics)."""
import struct
from pathlib import Path
import lz4.frame
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

GNSS_COMM = Path('/home/nybo/github/pose-validation/external/gnss/gnss_comm/msg')


def make_store():
    ts = get_typestore(Stores.ROS1_NOETIC)
    types = {}
    for name in ['GnssTimeMsg', 'GnssPVTSolnMsg', 'GnssTimePulseInfoMsg']:
        types.update(get_types_from_msg((GNSS_COMM / f'{name}.msg').read_text(), f'gnss_comm/msg/{name}'))
    types.update(get_types_from_msg('Header header\nuint32 trigger_id\nuint32 event_id\ntime timestamp_host\n',
                                    'gvins/msg/LocalSensorExternalTrigger'))
    ts.register(types)
    return ts


def _fields(hd):
    d = {}; i = 0
    while i < len(hd):
        l = struct.unpack('<I', hd[i:i + 4])[0]
        k, v = hd[i + 4:i + 4 + l].split(b'=', 1)
        d[k.decode('latin1')] = v; i += 4 + l
    return d


def iter_msgs(path, topics=None, max_bytes=None, raw=False, size=None):
    store = make_store()
    f = path if hasattr(path, 'read') else open(path, 'rb'); f.read(13)
    conns = {}  # id -> (topic, type)
    CD = {}  # topic -> connection data fields (type, md5sum, message_definition)
    if size is None: size = Path(path).stat().st_size
    while True:
        pos = f.tell()
        h = f.read(4)
        if len(h) < 4: return
        hl = struct.unpack('<I', h)[0]; hd = f.read(hl)
        dlb = f.read(4)
        if len(hd) < hl or len(dlb) < 4: return
        dl = struct.unpack('<I', dlb)[0]
        if f.tell() + dl > size: return  # truncated chunk
        d = _fields(hd); op = d['op'][0]
        if op == 5:
            data = f.read(dl)
            if d['compression'] == b'lz4': data = lz4.frame.decompress(data)
            elif d['compression'] != b'none': raise RuntimeError(d['compression'])
            i = 0
            while i < len(data):
                hl2 = struct.unpack('<I', data[i:i + 4])[0]; h2 = _fields(data[i + 4:i + 4 + hl2])
                dl2 = struct.unpack('<I', data[i + 4 + hl2:i + 8 + hl2])[0]
                body = data[i + 8 + hl2:i + 8 + hl2 + dl2]; i += 8 + hl2 + dl2
                o2 = h2['op'][0]
                if o2 == 7:
                    cd = _fields(body)
                    conns[struct.unpack('<I', h2['conn'])[0]] = (h2['topic'].decode(), cd['type'].decode()); CD[h2['topic'].decode()] = cd
                elif o2 == 2:
                    cid = struct.unpack('<I', h2['conn'])[0]
                    topic, typ = conns[cid]
                    if raw and (topics is None or topic in topics):
                        sec, nsec = struct.unpack('<II', h2['time']); yield topic, sec * 10**9 + nsec, body, CD[topic]
                    elif (topics is None or topic in topics) and f"{typ.split('/')[0]}/msg/{typ.split('/')[1]}" in store.fielddefs:
                        sec, nsec = struct.unpack('<II', h2['time'])
                        pk, nm = typ.split('/'); yield topic, sec * 10**9 + nsec, store.deserialize_ros1(body, f'{pk}/msg/{nm}')
        elif op == 7:
            cd = _fields(f.read(dl))
            conns[struct.unpack('<I', d['conn'])[0]] = (d['topic'].decode(), cd['type'].decode()); CD[d['topic'].decode()] = cd
        else:
            f.seek(dl, 1)
