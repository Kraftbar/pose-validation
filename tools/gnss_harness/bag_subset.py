#!/usr/bin/env python3
"""Copy a time window / topic subset of a (possibly truncated) ROS1 bag into a new complete, indexed bag (raw bytes, no ROS needed).
usage: bag_subset.py in.bag out.bag max_seconds topic [topic...]   (time measured on the sensor header stamp of /imu0 is not needed; uses bag time)"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from bag_head import iter_msgs
from rosbags.rosbag1 import Writer

inp, outp, maxs, topics = sys.argv[1], sys.argv[2], float(sys.argv[3]), set(sys.argv[4:])
t0 = None; conns = {}
w = Writer(outp); w.open()
if True:
    for topic, t, body, cd in iter_msgs(inp, topics, raw=True):
        if t0 is None: t0 = t
        if (t - t0) * 1e-9 > maxs: break
        if topic not in conns:
            conns[topic] = w.add_connection(topic, cd['type'].decode().replace('/', '/msg/'), msgdef=cd['message_definition'].decode(), md5sum=cd['md5sum'].decode(),
                                            callerid=cd.get('callerid', b'').decode() or None, latching=int(cd.get('latching', b'0') or 0))
        w.write(conns[topic], t, body)
w.close(); print('wrote', outp)
