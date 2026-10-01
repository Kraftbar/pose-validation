#!/usr/bin/env python3
"""Build an isolated observer against the existing real stella library.

Reads the working module-2 compiler/link flags. Never rebuilds or writes
the shared reference. Provenance records explicit linked library files and
the installed stella/FBoW headers (not every transitive system dependency).
"""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'runs/stella_port/match_bow'
REF = ROOT / 'runs/stella_port/reference_build'
DEPS = ROOT / 'external/candidates/deps/root/usr'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def runtime_env():
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, [REF/'install/lib', DEPS/'lib',
        DEPS/'lib/x86_64-linux-gnu', Path('/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu'),
        Path('/tmp/localopencv/root/usr/lib/x86_64-linux-gnu'), Path('/tmp/localopencv/root/usr/lib')]))
    env['OMP_NUM_THREADS'] = '1'
    return env


def build():
    build_dir = OUT/'build'
    build_dir.mkdir(parents=True, exist_ok=True)
    target = REF/'frame_bow_tool_build/CMakeFiles/dump_frame_bow.dir'
    flags = []
    for line in (target/'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags += shlex.split(line.split('=', 1)[1])
    source = ROOT/'stella_port/reference_match_bow/dump_main.cc'
    binary = build_dir/'dump_match_bow'
    obj = build_dir/'dump_main.o'
    compile_cmd = ['/usr/bin/c++', *flags, '-c', str(source), '-o', str(obj)]
    link = shlex.split((target/'link.txt').read_text())
    cmd = []
    for arg in link:
        if arg.endswith('.o') or arg.startswith('-Wl,--dependency-file='):
            continue
        cmd.append(arg)
    cmd[cmd.index('-o')+1] = str(binary)
    cmd.insert(1, str(obj))
    inputs = {p: sha(p) for p in [source, target/'flags.make', target/'link.txt']}
    for arg in cmd:
        p = Path(arg)
        if p.is_absolute() and p.is_file() and ('.so' in p.name or p.suffix == '.a'):
            inputs[p] = sha(p)
    for root in [REF/'install/include/stella_vslam', DEPS/'include/fbow']:
        for p in root.rglob('*.h'):
            inputs[p] = sha(p)
    pinned = ROOT/'external/candidates/stella_vslam'
    for name in ['match/bow_tree.cc', 'match/bow_tree.h', 'match/base.h', 'util/angle.cc']:
        original = pinned/'src/stella_vslam'/name
        built = REF/'src/src/stella_vslam'/name
        if sha(original) != sha(built):
            raise RuntimeError(f'Unexpected reference modification: {name}')
        inputs[original] = sha(original)
    with (build_dir/'build.log').open('w') as log:
        for command in [compile_cmd, cmd]:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=log, check=True)
    if any(sha(p) != h for p, h in inputs.items()):
        raise RuntimeError('Reference inputs changed during build')
    provenance = {'upstream_commit': subprocess.check_output(['git', '-C', str(pinned), 'rev-parse', 'HEAD'], text=True).strip(),
        'commands': [compile_cmd, cmd], 'input_sha256': {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p): h for p, h in inputs.items()},
        'binary_sha256': sha(binary), 'compiler': subprocess.check_output(['/usr/bin/c++','--version'],text=True).splitlines()[0]}
    (build_dir/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(binary)
    return binary


if __name__ == '__main__':
    build()
