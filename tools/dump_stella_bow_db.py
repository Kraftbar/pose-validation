#!/usr/bin/env python3
"""Generate lifecycle/query fixtures and observe real stella in two processes.

Only candidate order (pointer-hashed, not a stable upstream contract) is
recorded separately. Canonical membership, score bits, intermediate counts,
thresholds, and posting-list snapshots must match byte for byte.
"""
import argparse
from collections import deque
import csv
import json
from pathlib import Path
import shutil
import struct
import subprocess

from build_stella_bow_db import ROOT, OUT, build, runtime_env, sha


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def adjacent(x, delta):
    bits = struct.unpack('<I', struct.pack('<f', x))[0]
    return struct.unpack('<f', struct.pack('<I', bits+delta))[0]


class Commands:
    def __init__(self):
        self.lines = []
        self.queries = 0
        self.keys = 0

    def key(self, key, words):
        words = sorted(words.items())
        self.lines.append('K '+str(key)+' '+str(len(words))+''.join(
            f' {w} {float(v).hex()}' for w, v in words))
        self.keys += 1

    def query(self, key, minimum=0.0, ratio=0.8, reject=()):
        self.lines.append(f'Q {self.queries} {key} {float(f32(minimum)).hex()} '
                          f'{float(f32(ratio)).hex()} {len(reject)}'+''.join(f' {i}' for i in reject))
        self.queries += 1

    def op(self, text):
        self.lines.append(text)

    def save(self, folder):
        (folder/'commands.txt').write_text('\n'.join(self.lines)+'\n')
        return {'keyframes_or_query_vectors': self.keys, 'queries': self.queries,
                'commands': len(self.lines)}


def synthetic():
    c = Commands()
    vectors = {0: {}, 1: {1:.5,2:.5,3:.5,4:.5}, 2: {1:.5,2:.5,3:.5},
               3: {2:1.0}, 4: {1:0.0}, 5: {99:1.0},
               4294967295: {1:.5,2:.5,3:.5,4294967295:.5}}
    for key, words in vectors.items(): c.key(key, words)
    c.op('S'); c.query(1); c.op('E 1'); c.op('A 0'); c.query(0); c.op('S')
    for key in [1, 2, 3, 4, 5, 4294967295]: c.op(f'A {key}')
    c.op('S')
    for ratio in [0, .5, adjacent(.75,-1), .75, .8, 1, 1.25]:
        for minimum in [0, adjacent(.5,-1), .5, adjacent(.5,1), 1, adjacent(1,1), -.25]:
            c.query(1, minimum, ratio)
    for reject in [(1,), (1,2), (1,2,3,4,5,4294967295), (0,), (1,1)]:
        c.query(1, reject=reject)
    c.query(4, ratio=0); c.query(0); c.query(5)
    c.op('A 1'); c.op('S'); c.query(1); c.query(1, reject=(1,))
    for _ in range(3):
        c.op('E 1'); c.op('S'); c.query(1, ratio=.5)
    c.op('C'); c.op('S'); c.query(1)
    c.op('A 1'); c.op('A 2'); c.op('E 2'); c.op('A 2'); c.op('S'); c.query(2)
    c.op('C'); c.op('C'); c.op('S'); c.query(0)
    return c


def real(sequence):
    path = ROOT/'runs/stella_port/reference_frame_bow'/sequence/'bow_vec.tsv'
    frames = {}
    with path.open() as f:
        for r in csv.DictReader(f, delimiter='\t'):
            frames.setdefault(int(r['frame_idx']), {})[int(r['word_id'])] = float.fromhex(r['weight_hex'])
    if not frames or sorted(frames) != list(range(max(frames)+1)):
        raise RuntimeError(f'Missing frame vectors: {path}')
    c = Commands()
    for frame, words in sorted(frames.items()): c.key(frame, words)
    live = deque()
    for frame in sorted(frames):
        if frame % 7 == 0:
            c.op(f'A {frame}'); live.append(frame)
            if len(live) > 24: c.op(f'E {live.popleft()}')
        c.query(frame)
        c.query(frame, minimum=.05, ratio=.5, reject=tuple(list(live)[-2:]))
        if frame % 31 == 0:
            c.query(frame, ratio=0)
            c.op('S')
        if frame % 97 == 0:
            c.op(f'A {live[-1]}'); c.query(frame)
            c.op(f'E {live[-1]}'); c.query(frame)
    c.op('S')
    for frame in live: c.op(f'E {frame}')
    c.query(max(frames), ratio=0); c.op('S'); c.op('C'); c.op('S'); c.query(0)
    return c, path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-build', action='store_true')
    ap.add_argument('--out', type=Path, default=OUT/'fixtures')
    args = ap.parse_args()
    binary = OUT/'build/dump_bow_db' if args.skip_build else build()
    args.out.mkdir(parents=True, exist_ok=False)
    before = sha(binary)
    report = {}
    for name in ['synthetic', 'fr1_xyz', 'fr1_desk']:
        folder = args.out/name; folder.mkdir()
        source = None
        if name == 'synthetic': commands = synthetic()
        else: commands, source = real(name)
        info = commands.save(folder)
        for pass_id in [1, 2]:
            with (folder/f'pass{pass_id}.log').open('w') as log:
                subprocess.run([str(binary), str(folder/'commands.txt'), str(folder/f'pass{pass_id}.tsv'),
                                str(folder/f'raw_order{pass_id}.tsv')], env=runtime_env(),
                               stdout=log, stderr=log, check=True, timeout=600)
        if sha(folder/'pass1.tsv') != sha(folder/'pass2.tsv'):
            raise RuntimeError(f'Nondeterministic canonical reference: {name}')
        shutil.copyfile(folder/'pass1.tsv', folder/'expected.tsv')
        info.update(commands_sha256=sha(folder/'commands.txt'), expected_sha256=sha(folder/'expected.tsv'),
                    deterministic=True, raw_order_equal=sha(folder/'raw_order1.tsv') == sha(folder/'raw_order2.tsv'))
        if source: info.update(bow_input=str(source.relative_to(ROOT)), bow_input_sha256=sha(source))
        report[name] = info
        print(name, info, flush=True)
    if sha(binary) != before: raise RuntimeError('Native binary changed')
    (args.out/'provenance.json').write_text(json.dumps({'binary_sha256':before,
        'generator_sha256':sha(Path(__file__)), 'build_provenance_sha256':sha(OUT/'build/provenance.json'),
        'cases':report},indent=2)+'\n')


if __name__ == '__main__':
    main()
