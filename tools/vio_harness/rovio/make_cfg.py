#!/usr/bin/env python3
"""ROVIO config generator (benchmark glue). usage: make_cfg.py <seq|okvis.yaml> <out_dir>
Writes out_dir/rovio.info (upstream cfg/rovio.info with Camera0 qCM/MrMC replaced by the T_SC of the okvis yaml) and out_dir/cam0.yaml (plumb_bob, 4 radtan coefficients + 0).
qCM = quaternion IMU->camera (Hamilton) = q(R_SC^T); MrMC = camera position in the IMU frame = t_SC."""
import sys, re
import numpy as np
from pathlib import Path
sys.path.insert(0, '/home/nybo/github/pose-validation/tools/vio_harness'); import okvis_cfg
seq, out = sys.argv[1], Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
c = okvis_cfg.load(seq); T = np.array(c['T']).reshape(4, 4); R = T[:3, :3].T; t = T[:3, 3]
def quat(R):  # w x y z
    tr = np.trace(R)
    if tr > 0: s = np.sqrt(tr + 1) * 2; return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2; q = np.zeros(4); q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; q[0] = (R[k, j] - R[j, k]) / s; return q
q = quat(R)
info = Path('/home/nybo/github/pose-validation/external/vio3/rovio/cfg/rovio.info').read_text()
i0 = info.index('Camera0'); i1 = info.index('Camera1')
blk = info[i0:i1]
for k, v in (('qCM_x', q[1]), ('qCM_y', q[2]), ('qCM_z', q[3]), ('qCM_w', q[0]), ('MrMC_x', t[0]), ('MrMC_y', t[1]), ('MrMC_z', t[2])):
    blk = re.sub(rf'({k}\s+)[-0-9.eE+]+', rf'\g<1>{v!r}', blk, count=1)
(out / 'rovio.info').write_text(info[:i0] + blk + info[i1:])
dc = c['dc']
(out / 'cam0.yaml').write_text(f"""image_width: {c['dim'][0]}
image_height: {c['dim'][1]}
camera_name: cam0
camera_matrix:
  rows: 3
  cols: 3
  data: [{c['fl'][0]}, 0.0, {c['pp'][0]}, 0.0, {c['fl'][1]}, {c['pp'][1]}, 0.0, 0.0, 1.0]
distortion_model: {'plumb_bob' if c['model']=='radtan' else 'equidistant'}
distortion_coefficients:
  rows: 1
  cols: {5 if c['model']=='radtan' else 4}
  data: [{', '.join(str(x) for x in (dc + [0.0] if c['model']=='radtan' else dc))}]
""")
