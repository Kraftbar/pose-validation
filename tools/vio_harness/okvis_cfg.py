"""Parse the first camera + IMU noise of an OKVIS2 yaml (phone fixtures: tools/gnss_harness/robust_cfg/<seq>/okvis_default.yaml; drones: external/drone/<seq>_cfg/okvis2_mono.yaml)."""
import re
from pathlib import Path
ROOT = Path('/home/nybo/github/pose-validation')
def load(src):
    p = Path(src) if str(src).endswith('.yaml') else ROOT / 'tools/gnss_harness/robust_cfg' / src / 'okvis_default.yaml'
    t = p.read_text()
    def arr(key):
        m = re.search(key + r':[^\[]*\[([^\]]*)\]', t, re.S); return [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
    def num(key):
        m = re.search(key + r':\s*([-0-9.eE+]+)', t); return float(m.group(1)) if m else None
    dt = re.search(r'distortion_type:\s*(\w+)', t).group(1)
    return dict(T=arr('T_SC'), dim=[int(x) for x in arr('image_dimension')], fl=arr('focal_length'), pp=arr('principal_point'), dc=arr('distortion_coefficients'),
                model='equidistant' if dt.startswith('equi') else 'radtan',
                imu=dict(g=num('sigma_g_c'), a=num('sigma_a_c'), bg=num('sigma_gw_c'), ba=num('sigma_aw_c')))
