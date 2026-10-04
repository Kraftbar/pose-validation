# SPDX-License-Identifier: MIT (project-authored diagnostic tooling)
"""H4 (rolling shutter / exposure stamping) as a timing-sensitivity test: shift the camera stamps against the IMU by delta (a rolling shutter with readout T_r read from the
first row is equivalent to a delta = T_r/2 mid-frame offset; frame stamp = exposure start vs middle is another +-exposure/2) and measure the change of the scale ratio of the
independent closed-form estimator (vis_scale.py) and of the gyro-vs-visual rotation fit.   usage: h4_timing_sens.py"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import common as C
import vis_scale as V

if __name__ == '__main__':
    res = {}
    for seq, tk in (('outdoor1', 'orb3'), ('outdoor1', 'stella'), ('indoor2', 'stella'), ('outdoor2', 'orb3'), ('advio20', 'orb3')):
        d = V.load_ds(seq, tk); row = {}
        for dl in (-0.1, -0.05, -0.03, -0.015, 0.0, 0.015, 0.03, 0.05, 0.1, 0.3):
            rr = V.summ(V.run(seq, tk, 4.0, stride=2.0, delay=dl, d=d))
            row[dl] = rr['med']
        res[f'{seq}_{tk}'] = row
        print(f'{seq}_{tk}: scale ratio (L=4 s) vs camera-IMU shift ' + ' '.join(f'{k*1e3:+.0f}ms:{v:.3f}' for k, v in row.items()))
    (C.OUT / 'h4_timing_sens.json').write_text(json.dumps(res))
