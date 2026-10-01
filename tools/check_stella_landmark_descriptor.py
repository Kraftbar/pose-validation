#!/usr/bin/env python3
"""Standalone Landmark-descriptor build/check; does not touch the shared runner."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
from build_stella_landmark_descriptor import ROOT, OUT, sha


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sanitize', action='store_true')
    ap.add_argument('--fixtures', type=Path, default=OUT/'fixtures')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()
    dest = args.out or OUT/('sanitized' if args.sanitize else 'checks')
    dest.mkdir(parents=True, exist_ok=False)
    c = ROOT/'stella_port/c'
    harness = c/'check_sv_landmark_descriptor.c'
    sources = [harness, c/'sv_landmark_descriptor.c']
    binary = dest/'check_sv_landmark_descriptor'
    flags = ['-std=c99','-O2','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-g']
    if args.sanitize: flags += ['-fsanitize=address,undefined','-fno-omit-frame-pointer']
    cmd = ['gcc', *flags, '-I'+str(c), *map(str,sources), '-lm','-o',str(binary)]
    hashes = {str(p.relative_to(ROOT)):sha(p) for p in [*sources,c/'sv_landmark_descriptor.h']}
    fixture_hashes = {str(p.resolve()): sha(p) for name in ['synthetic','fr1_xyz','fr1_desk']
                      for p in [args.fixtures/name/'commands.txt', args.fixtures/name/'expected.tsv']}
    with (dest/'build.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=log,check=True)
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:halt_on_error=1', UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    results = {}
    for name in ['api','synthetic','fr1_xyz','fr1_desk']:
        if name == 'api': command = [str(binary),'--selftest']
        else:
            folder = args.fixtures/name
            command = [str(binary),'--case',name,str(folder/'commands.txt'),str(folder/'expected.tsv')]
        p = subprocess.run(command,cwd=ROOT,env=env,capture_output=True,text=True,timeout=600)
        (dest/(name+'.log')).write_text(p.stdout+p.stderr)
        m = re.search(r': (\d+)/(\d+)\s*$',p.stdout)
        ok = p.returncode == 0 and m and int(m[1]) == 0 and int(m[2]) > 0
        results[name] = {'returncode':p.returncode,'ok':bool(ok),'stdout':p.stdout,'stderr':p.stderr}
        print(name,p.returncode,p.stdout.strip(),flush=True)
    changed = [p for p,h in hashes.items() if sha(ROOT/p) != h]
    changed += [p for p,h in fixture_hashes.items() if sha(Path(p)) != h]
    result = {'command':cmd,'source_sha256':hashes,'fixture_sha256':fixture_hashes,'sources_changed':changed,'sanitize':args.sanitize,'results':results}
    (dest/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    return int(bool(changed) or not all(r['ok'] for r in results.values()))


if __name__ == '__main__':
    raise SystemExit(main())
