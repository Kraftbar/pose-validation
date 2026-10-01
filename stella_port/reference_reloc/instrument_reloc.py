#!/usr/bin/env python3
from pathlib import Path
import difflib,re,json,hashlib
ROOT=Path(__file__).resolve().parents[2];SRC=ROOT/'external/candidates/stella_vslam/src/stella_vslam/module';OUT=ROOT/'runs/stella_port/reference_reloc/instrumented'
original={n:(SRC/n).read_text() for n in ['relocalizer.h','relocalizer.cc']};output={}
output['relocalizer.h']=original['relocalizer.h'].replace('STELLA_VSLAM_MODULE_RELOCALIZER_H','SV_OBSERVED_RELOCALIZER_H').replace('namespace module {','namespace reloc_reference {').replace('"stella_vslam/solve/pnp_solver.h"','"pnp_solver.h"').replace('solve::pnp_solver','reloc_reference::pnp_solver')
s=original['relocalizer.cc'].replace('namespace module {','namespace reloc_reference {').replace('"stella_vslam/module/relocalizer.h"','"relocalizer.h"\n#include "reloc_trace.hpp"').replace('solve::pnp_solver','reloc_reference::pnp_solver')
def after(old,new):
 global s
 assert old in s,old
 s=s.replace(old,old+'\n'+new)
after('    const auto reloc_candidates = bow_db->acquire_keyframes(curr_frm.bow_vec_, 0.0f, num_common_words_thr_ratio_);','    reloc_trace::ids("candidates", reloc_candidates);')
after('        const auto& candidate_keyfrm = reloc_candidates.at(i);','        reloc_trace::scalar("candidate", candidate_keyfrm->id_);')
after('        bool ok = reloc_by_candidate(curr_frm, candidate_keyfrm, use_robust_matcher);','        reloc_trace::scalar("candidate_ok", ok);\n        reloc_trace::frame("candidate", curr_frm);')
after('    curr_frm.invalidate_pose();','    reloc_trace::frame("failed", curr_frm);')
for call,name in [('bool ok = relocalize_by_pnp_solver(curr_frm, candidate_keyfrm, use_robust_matcher, inlier_indices, matched_landmarks);','pnp'),('ok = optimize_pose(curr_frm, candidate_keyfrm, outlier_flags);','optimize'),('ok = refine_pose(curr_frm, candidate_keyfrm, already_found_landmarks);','refine'),('ok = refine_pose_by_local_map(curr_frm, candidate_keyfrm);','local')]:
 after('    '+call,f'    reloc_trace::scalar("{name}_ok", ok);\n    reloc_trace::frame("{name}", curr_frm);')
after('                                                : bow_matcher_.match_frame_and_keyframe(candidate_keyfrm, curr_frm, matched_landmarks);','    reloc_trace::scalar("initial_matches", num_matches);\n    reloc_trace::ids("initial_landmarks", matched_landmarks);')
after('    // Setup an PnP solver with the current 2D-3D matches','    reloc_trace::ids("expanded_landmarks", matched_landmarks);')
after('    auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, already_found_landmarks, 10, 100);','    reloc_trace::scalar("projection10", num_found);\n    reloc_trace::frame("projection10", curr_frm);')
after('    auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, already_found_landmarks1, 3, 64);','    reloc_trace::scalar("projection3", num_additional);\n    reloc_trace::frame("projection3", curr_frm);')
after('    auto nearest_covisibility = local_map_updater.get_nearest_covisibility();','    reloc_trace::ids("local_keys", local_keyfrms);\n    reloc_trace::ids("local_landmarks", local_landmarks);')
after('        if (!found_proj_candidate) {','            reloc_trace::scalar("local_visible", found_proj_candidate);')
after('        // acquire more 2D-3D matches by projecting the local landmarks to the current frame','        reloc_trace::scalar("local_visible", found_proj_candidate);')
after('        auto num_additional_matches = projection_matcher.match_frame_and_landmarks(curr_frm, local_landmarks, lm_to_reproj, lm_to_x_right, lm_to_scale, margin);','        reloc_trace::scalar("local_additional", num_additional_matches);\n        reloc_trace::frame("local_projection", curr_frm);')
after('        curr_frm.set_pose_cw(optimized_pose);','        reloc_trace::scalar("local_valid", num_valid_obs);')
after('    // Setup PnP solver','    if (reloc_trace::pnp_input) reloc_trace::pnp_input(valid_bearings, octaves, valid_points, scale_factors);')
# Native robust matcher otherwise ignores Relocalizer.use_fixed_seed. This
# observer always forwards the selected deterministic seed policy.
s=s.replace('robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks)','robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks, use_fixed_seed_)').replace('robust_matcher_.match_frame_and_keyframe(curr_frm, ngh_keyfrm, additional_matched_landmarks)','robust_matcher_.match_frame_and_keyframe(curr_frm, ngh_keyfrm, additional_matched_landmarks, use_fixed_seed_)')
output['relocalizer.cc']=s
patch='Isolated relocalizer observer, namespace rename and deterministic seed forwarding for the optional robust matcher.\n\n'
for name,new in output.items():
 (OUT/name).write_text(new)
 patch+=''.join(difflib.unified_diff(original[name].splitlines(True),new.splitlines(True),fromfile='a/src/stella_vslam/module/'+name,tofile='b/src/stella_vslam/module/'+name))
pdir=ROOT/'stella_port/reference_reloc/patches';old=list(pdir.glob('*relocalizer-trace.patch'))
if old:
 assert len(old)==1 and old[0].read_text()==patch
 target=old[0]
else:
 ns=[int(p.name[:4]) for p in (ROOT/'stella_port').rglob('*.patch') if re.match(r'^\d{4}-',p.name)]
 target=pdir/f'{max(ns,default=0)+1:04d}-relocalizer-trace.patch'
 with target.open('x') as f:f.write(patch)
(OUT/'relocalizer_sources.json').write_text(json.dumps({str(SRC/n):hashlib.sha256((SRC/n).read_bytes()).hexdigest() for n in original},indent=2)+'\n')
print(target.relative_to(ROOT))
