#include <stella_vslam/solve/pnp_solver.h>
#include "pnp_solver.h"
#include "pnp_trace.hpp"
#include "sv_pnp.h"
#include <random>
#include <cstring>
#include <cstdio>
namespace reloc_trace {sv_pnp_trace_fn callback=nullptr;void*context=nullptr;}
using namespace stella_vslam;
struct Entry {std::string name;std::vector<double>v;};
struct Check {std::vector<Entry>entries;size_t pos=0,bad=0,total=0;int test=0;};
void capture(void*u,const char*name,const double*p,unsigned n){auto&c=*(Check*)u;c.entries.push_back({name,{p,p+n}});}
void compare(void*u,const char*name,const double*p,unsigned n){auto&c=*(Check*)u;if(c.pos>=c.entries.size() || c.entries[c.pos].name!=name || c.entries[c.pos].v.size()!=n){printf("stage order changed %s\n",name);exit(2);}auto&e=c.entries[c.pos++];for(unsigned i=0;i<n;i++){c.total++;if(memcmp(p+i,e.v.data()+i,8)){if(c.bad<6)printf("test%d %s[%u] %a %a\n",c.test,name,i,p[i],e.v[i]);c.bad++;}}}
int main(){std::mt19937 rng(4117);std::uniform_real_distribution<double>d(-1,1);size_t bad=0,total=0;
 for(int test=0;test<100;test++){
  unsigned n=test%2?4:8+(test*3)%93; eigen_alloc_vector<Vec3_t>b(n),p(n);std::vector<double>bb(n*3),pp(n*3);
  for(unsigned i=0;i<n;i++){p[i]=Vec3_t(d(rng),d(rng),3+d(rng));b[i]=(p[i]+Vec3_t(.2,-.1,.3)).normalized();for(int j=0;j<3;j++){bb[3*i+j]=b[i][j];pp[3*i+j]=p[i][j];}}
  Check c;c.test=test;reloc_trace::callback=capture;reloc_trace::context=&c;
  Mat33_t nr;Vec3_t nt;double ne=reloc_reference::pnp_solver::compute_pose(b,p,nr,nt,10);
  reloc_trace::callback=nullptr;Mat33_t orig_r;Vec3_t orig_t;double oe=solve::pnp_solver::compute_pose(b,p,orig_r,orig_t,10);
  if(memcmp(nr.data(),orig_r.data(),72)||memcmp(nt.data(),orig_t.data(),24)||memcmp(&ne,&oe,8)){puts("patch changed result");return 2;}
  double r[9],t[3],error;sv_pnp_compute_pose(bb.data(),pp.data(),n,10,r,t,&error,compare,&c);
  if(c.pos!=c.entries.size()){puts("missing stages");return 2;}
  bad+=c.bad;total+=c.total;
  if(c.bad)break;
 }
 printf("pose: %zu/%zu\n",bad,total);return bad?1:0;
}
