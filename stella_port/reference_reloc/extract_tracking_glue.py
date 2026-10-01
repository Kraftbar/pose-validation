#!/usr/bin/env python3
"""Compile the exact auto-relocalization branch with a minimal member owner."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/'external/candidates/stella_vslam/src/stella_vslam/tracking_module.cc';s=p.read_text();start=s.index('        // Compute the BoW representations to perform relocalization',s.index('bool tracking_module::track('));end=s.index('\n    }\n\n    // update the local map',start);body=s[start:end]
out=ROOT/'runs/stella_port/reference_reloc/instrumented/tracking_glue.hpp'
out.write_text('''// Extracted unchanged from tracking_module::track automatic relocalization branch.
struct TrackingBranch {
 stella_vslam::data::frame& curr_frm_;
 stella_vslam::data::bow_database* bow_db_;
 stella_vslam::data::bow_vocabulary* bow_vocab_;
 stella_vslam::reloc_reference::relocalizer& relocalizer_;
 unsigned last_reloc_frm_id_=17;
 double last_reloc_frm_timestamp_=3.0;
 bool run(){bool succeeded=false;
'''+body+'\n return succeeded;\n }\n};\n')
(out.parent/'tracking_glue_sources.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()},indent=2)+'\n')
