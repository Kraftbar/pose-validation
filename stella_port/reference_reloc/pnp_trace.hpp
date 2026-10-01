#ifndef SV_PNP_REFERENCE_TRACE_HPP
#define SV_PNP_REFERENCE_TRACE_HPP
#include "sv_pnp.h"
#include <vector>
namespace reloc_trace {
extern sv_pnp_trace_fn callback;
extern void *context;
inline void emit(const char *name,const double *p,unsigned n){if(callback)callback(context,name,p,n);}
template<class M>void mat(const char*name,const M&m){std::vector<double>v(m.size());for(int j=0;j<m.cols();j++)for(int i=0;i<m.rows();i++)v[i+m.rows()*j]=m(i,j);emit(name,v.data(),v.size());}
template<class V>void vecs(const char*name,const V&v){std::vector<double>a;for(auto&p:v)for(int i=0;i<p.size();i++)a.push_back(p[i]);emit(name,a.data(),a.size());}
inline void scalar(const char*name,double v){emit(name,&v,1);}
}
#endif
