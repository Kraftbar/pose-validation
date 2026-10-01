#!/usr/bin/env python3
"""Build this leaf's two discovered harnesses and verify all captured cases."""
import argparse,json,os,subprocess,sys
from pathlib import Path
from build import ROOT,OUT,sha
sys.path.insert(0,str(ROOT/'tools'))
import check_stella_port as runner
ap=argparse.ArgumentParser();ap.add_argument('--sanitize',action='store_true');args=ap.parse_args()
subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_suite.py'))],check=True)
results=[];sources=set()
for name,files in runner.discover_harnesses():
 if name not in ['check_sv_pnp','check_sv_reloc']:continue
 sources.update(runner.C_DIR/f for f in files)
 binary=OUT/(name+('_san' if args.sanitize else '_validation'))
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if args.sanitize else []
 command=['gcc','-std=c99','-O2','-g','-ffp-contract=off','-fno-fast-math',*flags,'-o',str(binary),*[str(runner.C_DIR/f) for f in files],'-lm']
 subprocess.run(command,check=True)
 for seq in ['fr1_xyz','fr1_desk']:
  env=dict(os.environ)
  if args.sanitize:env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
  cmd=[str(binary),seq,'-',str(ROOT/'runs/stella_port/reference_dumps'/seq)]
  p=subprocess.run(cmd,env=env,capture_output=True,text=True)
  print(name,seq,p.returncode,p.stdout.strip(),flush=True)
  if p.stderr:print(p.stderr,flush=True)
  results.append(dict(harness=name,sequence=seq,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr,binary_sha256=sha(binary),command=cmd,build_command=command))
# Include headers and staged harness bodies as well as compilation units.
sources.update((ROOT/'stella_port/reference_reloc').rglob('*.h'))
sources.update((ROOT/'stella_port/reference_reloc').glob('*.c'))
sources.update((ROOT/'stella_port/c').glob('*.h'))
report=dict(sanitizers=args.sanitize,leak_check=False if args.sanitize else None,results=results,sources={str(p.resolve()):sha(p) for p in sorted(sources)})
(OUT/('validation_sanitized.json' if args.sanitize else 'validation.json')).write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(any(r['returncode'] for r in results))
