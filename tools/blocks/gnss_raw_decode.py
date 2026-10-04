#!/usr/bin/env python3
"""Decode the pickle written by stream_gnss_from_bag.py into plain text files (own code; message layout from gnss_comm/msg/*.msg).
usage: gnss_raw_decode.py raw.pkl outdir"""
import sys, pickle, struct
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'gnss_harness'))
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
M = Path('/home/nybo/github/pose-validation/external/gnss/gnss_comm/msg')
ts = get_typestore(Stores.ROS1_NOETIC); types = {}
# GnssObsMsg is in msg dir; Header is std
for n in ['GnssTimeMsg', 'GnssObsMsg', 'GnssMeasMsg', 'GnssEphemMsg', 'GnssGloEphemMsg', 'StampedFloat64Array', 'GnssPVTSolnMsg']:
    types.update(get_types_from_msg((M / f'{n}.msg').read_text(), f'gnss_comm/msg/{n}'))
ts.register(types)
def des(name, body): return ts.deserialize_ros1(body, f'gnss_comm/msg/{name}')
pk = pickle.load(open(sys.argv[1], 'rb')); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
D = pk['data']
# obs: one line per (epoch, sat, signal): week tow sat freq cn0 lli code psr psr_std cp cp_std dopp dopp_std status
with open(out / 'obs.txt', 'w') as f:
    f.write('# week tow sat freq cn0 lli code psr psr_std cp cp_std dopp dopp_std status\n')
    for t, b in D['/ublox_driver/range_meas']:
        m = des('GnssMeasMsg', b)
        for o in m.meas:
            for i in range(len(o.freqs)):
                f.write('%d %.6f %d %.1f %.1f %d %d %.4f %.3f %.4f %.3f %.4f %.3f %d\n' % (
                    o.time.week, o.time.tow, o.sat, o.freqs[i], o.CN0[i], o.LLI[i], o.code[i], o.psr[i], o.psr_std[i], o.cp[i], o.cp_std[i], o.dopp[i], o.dopp_std[i], o.status[i]))
with open(out / 'eph.txt', 'w') as f:
    f.write('# sat week toe_tow toc_tow ttr_tow iode iodc health code ura A e i0 omg OMG0 M0 delta_n OMG_dot i_dot cuc cus crc crs cic cis af0 af1 af2 tgd0 tgd1\n')
    for t, b in D['/ublox_driver/ephem']:
        e = des('GnssEphemMsg', b)
        v = [e.sat, e.week, e.toe.tow, e.toc.tow, e.ttr.tow, e.iode, e.iodc, e.health, e.code, e.ura, e.A, e.e, e.i0, e.omg, e.OMG0, e.M0, e.delta_n, e.OMG_dot, e.i_dot,
             e.cuc, e.cus, e.crc, e.crs, e.cic, e.cis, e.af0, e.af1, e.af2, e.tgd0, e.tgd1, e.toe.week]
        f.write(' '.join(repr(float(x)) if k >= 9 else str(int(x)) if k in (0,1,5,6,7,8) else '%.3f' % x for k, x in enumerate(v)) + '\n')
with open(out / 'iono.txt', 'w') as f:
    for t, b in D['/ublox_driver/iono_params']:
        m = des('StampedFloat64Array', b); f.write('%d %s\n' % (t, ' '.join('%.10g' % x for x in m.data)))
with open(out / 'pvt.txt', 'w') as f:   # receiver (RTK/u-blox) solution = ground truth
    f.write('# week tow lat lon h fix carr nsv h_acc v_acc vn ve vd vel_acc\n')
    for t, b in D['/ublox_driver/receiver_pvt']:
        m = des('GnssPVTSolnMsg', b)
        f.write('%d %.4f %.9f %.9f %.4f %d %d %d %.4f %.4f %.4f %.4f %.4f %.4f\n' % (m.time.week, m.time.tow, m.latitude, m.longitude, m.altitude, m.fix_type, m.carr_soln, m.num_sv, m.h_acc, m.v_acc, m.vel_n, m.vel_e, m.vel_d, m.vel_acc))
with open(out / 'glo.txt', 'w') as f:
    f.write('# sat toe_week toe_tow tof_tow freqo iode health age ura px py pz vx vy vz ax ay az tau_n gamma dtau\n')
    for t, b in D['/ublox_driver/glo_ephem']:
        e = des('GnssGloEphemMsg', b)
        f.write('%d %d %.3f %.3f %d %d %d %d %.3f' % (e.sat, e.toe.week, e.toe.tow, e.ttr.tow, e.freqo, e.iode, e.health, e.age, e.ura) + ' %.17g' * 12 % (e.pos_x, e.pos_y, e.pos_z, e.vel_x, e.vel_y, e.vel_z, e.acc_x, e.acc_y, e.acc_z, e.tau_n, e.gamma, e.delta_tau_n) + '\n')
print('ok')
