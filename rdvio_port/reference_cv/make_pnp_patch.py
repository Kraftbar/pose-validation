#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Observe every RD-VIO solve_pnp_{4,6}pt result; no numerical changes."""
from pathlib import Path
import difflib
ROOT=Path(__file__).resolve().parents[2]
rel='src/rdvio_geometry/include/rdvio/geometry/pnp.h'
a=(ROOT/'external/vio3/rd_vio'/rel).read_text();b=a
helper=r'''
// M7b observe-only, enabled only by RDVIO_CV_PNP_DUMP.
#include <cstdio>
#include <cstdlib>
namespace rdvio { namespace portcvpnp {
inline FILE *file() {
 static FILE *f=[](){const char *p=std::getenv("RDVIO_CV_PNP_DUMP");if(!p)return (FILE*)nullptr;
   FILE *q=std::fopen(p,"wb");if(!q)std::abort();if(std::fwrite("RDPNPR1\0",1,8,q)!=8)std::abort();return q;}();
 return f;
}
inline void write(const void *p,size_t n) {if(n&&std::fwrite(p,1,n,file())!=n)std::abort();}
template<size_t N> inline void dump(const std::array<vector<3>,N>&X,const std::array<vector<2>,N>&x,const matrix<4>&T) {
 if(!file())return;
 uint32_t n=N;write(&n,4);
 for(size_t i=0;i<N;i++)for(int j=0;j<3;j++){double a=X[i][j];write(&a,8);}
 for(size_t i=0;i<N;i++)for(int j=0;j<2;j++){double a=x[i][j];write(&a,8);}
 write(T.data(),16*8);
 if(std::fflush(file()))std::abort();
}
}}
'''
b=b.replace('namespace rdvio {',helper+'\nnamespace rdvio {',1)
b=b.replace('    poses.push_back(pose);','    portcvpnp::dump(Xs, xs, pose);\n    poses.push_back(pose);')
patch=''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
(ROOT/'rdvio_port/reference/patches/0010-m7b-pnp-dump.patch').write_text(patch)
