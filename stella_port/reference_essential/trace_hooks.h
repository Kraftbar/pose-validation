#ifndef SV_ESSENTIAL_REFERENCE_TRACE_H
#define SV_ESSENTIAL_REFERENCE_TRACE_H
#include "sv_solve_essential_5pt.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <cfloat>
namespace essential_trace {
extern FILE *file;
extern sv_essential_5pt_trace minimal;
inline void raw(const void *p,size_t n){if(file && std::fwrite(p,1,n,file)!=n)std::abort();}
template<class M> void matrix(double *out,const M &m){for(int j=0;j<m.cols();j++)for(int i=0;i<m.rows();i++)out[i+m.rows()*j]=m(i,j);}
inline void begin(const std::vector<unsigned> &indices){minimal={};unsigned n=indices.size();raw(&n,4);raw(indices.data(),4*n);}
inline void finish_minimal(const std::vector<stella_vslam::Mat33_t> &es){minimal.count=es.size();raw(&minimal,sizeof(minimal));unsigned n=es.size();raw(&n,4);for(auto &e:es)raw(e.data(),72);}
inline void inliers(float cost,unsigned count,const std::vector<bool>&mask){raw(&cost,4);raw(&count,4);for(bool b:mask){unsigned char c=b;raw(&c,1);}}
inline void best(float cost,unsigned count,const stella_vslam::Mat33_t &e){raw(&cost,4);raw(&count,4);if(cost<FLT_MAX)raw(e.data(),72);}
}
#endif
