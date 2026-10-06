#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Produce observe-only patch 0008 from upstream OpenCvImage (no shared edits)."""
from pathlib import Path
import difflib
ROOT=Path(__file__).resolve().parents[2]
rel='src/rdvio_extra/src/opencv_image.cpp'
a=(ROOT/'external/vio3/rd_vio'/rel).read_text();b=a
b=b.replace('#include <rdvio/extra/opencv_image.h>','#include <rdvio/extra/opencv_image.h>\n#include "port_cv_dump.h"')
b=b.replace('    gftt(max_points)->detect(image, cvkeypoints);','    gftt(max_points)->detect(image, cvkeypoints);\n    std::vector<KeyPoint> port_cv_unsorted;\n    if (portcv::file()) port_cv_unsorted = cvkeypoints;\n    if (cvkeypoints.empty()) portcv::detect(image, gftt(max_points)->getMaxFeatures(), port_cv_unsorted, cvkeypoints);')
b=b.replace('        std::vector<vector<2>> new_keypoints;','        portcv::detect(image, gftt(max_points)->getMaxFeatures(), port_cv_unsorted, cvkeypoints);\n        std::vector<vector<2>> new_keypoints;')
b=b.replace('        Mat cvstatus, cverr;','        Mat cvstatus, cverr;\n        portcv::track_begin(image, next_cvimage->image, curr_cvpoints, next_cvpoints);')
b=b.replace('        for (size_t i = 0; i < next_cvpoints.size(); ++i) {','        portcv::forward(next_cvpoints, cvstatus, cverr);\n        for (size_t i = 0; i < next_cvpoints.size(); ++i) {',1)
b=b.replace('        }\n    }\n\n    std::vector<size_t> l;','            portcv::reverse(reverse_pts, reverse_status, reverse_err, result_status);\n        }\n    }\n\n    std::vector<size_t> l;')
b=b.replace('    clahe(clipLimit, width, height)->apply(image, image);','    cv::Mat port_cv_input;\n    if (portcv::file()) port_cv_input = image.clone();\n    clahe(clipLimit, width, height)->apply(image, image);')
b=b.replace('                            (int)level_num(), true);','                            (int)level_num(), true);\n    portcv::preprocess(port_cv_input, image, image_pyramid);')
patch=''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
h=(ROOT/'rdvio_port/reference_cv/stream_dump.hpp').read_text()
patch+=''.join(difflib.unified_diff([],h.splitlines(True),fromfile='/dev/null',tofile='b/src/rdvio_extra/src/port_cv_dump.h'))
(ROOT/'rdvio_port/reference/patches/0008-m7-image-stream.patch').write_text(patch)
