#!/usr/bin/env python3
"""Build and validate the reserved leaf and both tracking replay harnesses."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
from build import ROOT, OUT, sha


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True, help='new result directory')
    args = ap.parse_args()
    dest = args.out.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    cdir = ROOT / 'stella_port/c'
    results = []
    sources = set()
    env = {**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0', 'UBSAN_OPTIONS': 'halt_on_error=1'}
    for seq in ['fr1_xyz', 'fr1_desk']:
        folder = OUT / 'fixtures' / seq
        ref = json.loads((folder / 'reference.json').read_text())
        if sha(OUT / 'dump_reference') != ref['binary_sha256']:
            raise RuntimeError('reference binary changed; retain/regenerate its provenance')
        for frame, digest in ref['traces'].items():
            for suffix in ['trace', 'pass1', 'pass2']:
                if sha(folder / f'{frame}.{suffix}') != digest:
                    raise RuntimeError(f'fixture changed: {seq} {frame} {suffix}')

    def run(name, cmd, expected=0):
        p = subprocess.run(list(map(str, cmd)), capture_output=True, text=True, env=env)
        (dest / (name + '.log')).write_text(p.stdout + p.stderr)
        results.append({'name': name, 'command': list(map(str, cmd)), 'returncode': p.returncode})
        (dest / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
        if p.returncode != expected:
            raise RuntimeError(f'{name}: expected {expected}, got {p.returncode}; see log')
        print(name, 'PASS', flush=True)
        return p

    for name in ['check_sv_essential_5pt', 'check_sv_track', 'check_sv_track_bow']:
        first = (cdir / (name + '.c')).read_text().splitlines()[0]
        files = first.split(':', 1)[1].split('*/')[0].split()
        sources.update(cdir / s for s in files)
        for san in [False, True] if name != 'check_sv_track' else [False]:
            label = name + ('_san' if san else '')
            exe = dest / label
            flags = ['-fsanitize=address,undefined', '-fno-omit-frame-pointer'] if san else []
            cmd = ['gcc', '-std=c99', '-O2', '-g', '-ffp-contract=off', '-fno-fast-math',
                   *flags, *[str(cdir / s) for s in files], '-lm', '-o', str(exe)]
            run(label + '_build', cmd)
            for seq in ['fr1_xyz', 'fr1_desk']:
                p = run(label + '_' + seq, [exe, seq, ROOT / 'runs/stella_port/fixtures' / seq,
                                             ROOT / 'runs/stella_port/reference_dumps' / seq])
                m = re.search(r': (\d+)/(\d+)\s*$', p.stdout)
                if not m or int(m[1]) or not int(m[2]):
                    raise RuntimeError('missing successful nonempty comparison table')
                if name == 'check_sv_track_bow':
                    count = 1 if seq == 'fr1_xyz' else 27
                    if f'robust fallback frames checked={count}' not in p.stderr:
                        raise RuntimeError('robust fallback coverage missing')

    exe = dest / 'check_sv_essential_5pt'
    negative = dest / 'negative'
    negative.mkdir()
    (negative / 'frames.txt').write_text('')
    run('reject_empty', [exe, 'empty', negative], 2)
    (negative / 'frames.txt').write_text('80\n')
    data = bytearray((OUT / 'fixtures/fr1_xyz/80.trace').read_bytes())
    (negative / '80.trace').write_bytes(data[:8])
    run('reject_truncated', [exe, 'truncated', negative], 2)
    n, = struct.unpack_from('I', data)
    data[8 + 56 * n + 24] ^= 1  # flip one captured constraint coefficient bit
    (negative / '80.trace').write_bytes(data)
    run('reject_changed_coefficient', [exe, 'changed', negative], 1)
    # Record exact inputs, including included harnesses and shared headers.
    sources.update(cdir.glob('*.h'))
    sources.update((ROOT / 'stella_port/reference_essential').glob('*.*'))
    manifest = {str(p.relative_to(ROOT)): sha(p) for p in sorted(sources) if p.is_file()}
    (dest / 'sources.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('All checks passed. LeakSanitizer disabled: unavailable under sandbox ptrace.')


if __name__ == '__main__':
    main()
