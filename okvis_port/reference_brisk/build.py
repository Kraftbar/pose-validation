#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Isolated pinned BRISK reference build; patches only add trace observations."""
from pathlib import Path
import shutil,subprocess,json,hashlib,os,difflib
R=Path(__file__).resolve().parents[2]; O=R/'runs/okvis_port/reference_brisk'; S=R/'external/vio/okvis2/external/brisk'; D=O/'src'; HERE=Path(__file__).resolve().parent
assert subprocess.check_output(['git','-C',str(S),'rev-parse','HEAD'],text=True).strip() == '1ef8b42a5c2fdd0e0c976f3ab7b179806381f570', 'BRISK pin changed'
assert subprocess.check_output(['git','-C',str(S.parent.parent),'rev-parse','HEAD'],text=True).strip() == 'a2ea00688cd10988aae7bd52ab7935ce9a657ec0', 'OKVIS2 pin changed'
O.mkdir(parents=True,exist_ok=True)
shutil.copytree(S,D,dirs_exist_ok=True,ignore=shutil.ignore_patterns('.git','images'))
changes={
'src/harris-score-calculator.cc':[('HarrisScoresSSE(_img, _scores);','HarrisScoresSSE(_img, _scores);\n  ok_trace("scores", _scores.data, _scores.total()*4);')],
'include/brisk/internal/scale-space-layer-inl.h':[('_scoreCalculator.Get2dMaxima(points, _absoluteThreshold);','_scoreCalculator.Get2dMaxima(points, _absoluteThreshold);\n    ok_trace("maxima", points.data(), points.size()*sizeof(points[0]));')],
'include/brisk/internal/uniformity-enforcement-inl.h':[('std::sort(points.begin(), points.end());','std::sort(points.begin(), points.end());\n  ok_trace("sorted", points.data(), points.size()*sizeof(points[0]));'),('points.assign(pt_tmp.begin(), pt_tmp.end());','points.assign(pt_tmp.begin(), pt_tmp.end());\n  ok_trace("selected", points.data(), points.size()*sizeof(points[0]));')],
'src/brisk-descriptor-extractor.cc':[
('ksize = keypoints.size();\n    AllocateDescriptors','ksize = keypoints.size();\n    ok_trace("filtered", keypoints.data(),keypoints.size()*sizeof(cv::KeyPoint));\n    AllocateDescriptors'),
('IntegralImage8(image, &_integral);','IntegralImage8(image, &_integral);\n      ok_trace("integral",_integral.data,_integral.total()*4);'),
('      // compute angle','      float tracewarp[6] = {warpptr ? warpptr[0] : 0,warpptr ? warpptr[1] : 0,warpptr ? warpptr[2] : 0,warpptr ? warpptr[3] : 0,sigmaScale,float(directional)};\n      ok_trace("warp",tracewarp,sizeof(tracewarp));\n      // compute angle'),
('          int direction0 = 0;','          ok_trace("orientation_values",_values,points_*4);\n          int direction0 = 0;'),
('      setDescriptorBits(k, _values, &descriptors);','      ok_trace("values",_values,points_*4);\n      setDescriptorBits(k, _values, &descriptors);')]
}
patch=[]
for name,edits in changes.items():
 p=D/name;old=p.read_text();new=old
 for a,b in edits:assert new.count(a)==1,(name,a,new.count(a));new=new.replace(a,b)
 p.write_text(new);patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
(HERE/'trace.patch').write_text(''.join(patch))
cmake=f'''cmake_minimum_required(VERSION 3.16)
project(ok_brisk_reference LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math -include {HERE}/trace.hpp")
set(BRISK_BUILD_DEMO OFF CACHE BOOL "" FORCE)
set(OpenCV_DIR "{R}/external/vio/deps/opencv/lib/cmake/opencv4")
add_subdirectory("{D}" brisk)
find_package(OpenCV REQUIRED COMPONENTS core features2d imgproc imgcodecs calib3d)
add_executable(dump_brisk "{HERE}/dump_main.cpp" "{R}/external/vio/okvis2/okvis_cv/src/CameraBase.cpp")
target_include_directories(dump_brisk PRIVATE "{R}/external/vio/okvis2/okvis_cv/include" "{R}/external/vio/okvis2/okvis_util/include" "{R}/external/vio/okvis2/okvis_kinematics/include" "{R}/external/vio/deps/root/usr/include/eigen3")
target_link_libraries(dump_brisk PRIVATE brisk ${{OpenCV_LIBS}})
'''
(O/'CMakeLists.txt').write_text(cmake)
subprocess.run(['cmake','-S',str(O),'-B',str(O/'build')],check=True)
subprocess.run(['cmake','--build',str(O/'build'),'-j2'],check=True)
prov={'okvis2':subprocess.check_output(['git','-C',str(S.parent.parent),'rev-parse','HEAD'],text=True).strip(),'brisk':subprocess.check_output(['git','-C',str(S),'rev-parse','HEAD'],text=True).strip(),'flags':'-O2 -DNDEBUG -ffp-contract=off -fno-fast-math -mssse3','source_sha256':{str(p.relative_to(S)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(S.rglob('*')) if p.is_file() and p.suffix in ('.h','.cc')},'trace_patch_sha256':hashlib.sha256((HERE/'trace.patch').read_bytes()).hexdigest()}
prov.update({'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0], 'opencv_core_sha256':hashlib.sha256((R/'external/vio/deps/opencv/lib/libopencv_core.so.4.6.0').read_bytes()).hexdigest(), 'opencv_cvconfig_sha256':hashlib.sha256((R/'external/vio/deps/opencv/include/opencv4/opencv2/cvconfig.h').read_bytes()).hexdigest(), 'effective_brisk_flags':(O/'build/brisk/CMakeFiles/brisk.dir/flags.make').read_text(), 'frontend_parameters':{'uniformity_radius':38,'absolute_threshold':150,'octaves':0,'max_num_keypoints':700,'rotation_invariance':True,'scale_invariance':False,'version':2,'pattern_scale':1}})
(O/'provenance.json').write_text(json.dumps(prov,indent=2)+'\n')
