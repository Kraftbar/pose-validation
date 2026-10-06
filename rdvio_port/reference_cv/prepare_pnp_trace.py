#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Observe a namespaced copy of OpenCV EPnP; keep public solvePnP as oracle.
Only I/O hooks and class/include names change. Native copy output is also
checked against the installed library in dump_pnp.cpp, before writing fixtures.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'runs/rdvio_port/reference_cv_m7b';OUT.mkdir(exist_ok=True,parents=True)
SRC=ROOT/'external/opencv_4.6.0_src/modules/calib3d/src'
h=(SRC/'epnp.h').read_text().replace('#include "precomp.hpp"','#include <opencv2/opencv.hpp>').replace('epnp','rd_epnp_trace')
(OUT/'epnp_trace.h').write_text(h)
s=(SRC/'epnp.cpp').read_text().replace('#include "precomp.hpp"','#include <opencv2/opencv.hpp>\n#include "m7b_io.hpp"').replace('#include "epnp.h"','#include "epnp_trace.h"').replace('epnp::','rd_epnp_trace::').replace('epnp(', 'rd_epnp_trace(').replace('~epnp(', '~rd_epnp_trace(')
s=s.replace('  compute_barycentric_coordinates();','  rec("cws",cws,sizeof(cws));\n  compute_barycentric_coordinates();\n  rec("alphas",alphas.data(),number_of_correspondences*4*sizeof(double));')
s=s.replace('  compute_L_6x10(ut, l_6x10);','  rec("mtm",mtm,sizeof(mtm));\n  rec("ut",ut,sizeof(ut));\n  compute_L_6x10(ut, l_6x10);')
s=s.replace('  int N = 1;','  rec("betas",Betas,sizeof(Betas));\n  rec("errors",rep_errors,sizeof(rep_errors));\n  rec("Rs",Rs,sizeof(Rs));\n  rec("ts",ts,sizeof(ts));\n  int N = 1;')
(OUT/'epnp_trace.cpp').write_text(s)
