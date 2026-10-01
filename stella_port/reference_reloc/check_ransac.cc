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
int main(){std::mt19937 rng(417);std::uniform_real_distribution<double>d(-1,1);size_t bad=0,total=0;
 std::vector<float>scales(8);scales[0]=1;for(int i=1;i<8;i++)scales[i]=scales[i-1]*1.2f;
 for(int test=0;test<100;test++){
  unsigned n=test<12?test:12+(test*3)%93; eigen_alloc_vector<Vec3_t>b(n),p(n);std::vector<double>bb(n*3),pp(n*3);std::vector<int>oct(n);
  for(unsigned i=0;i<n;i++){p[i]=Vec3_t(d(rng),d(rng),3+d(rng));b[i]=(p[i]+Vec3_t(.2,-.1,.3)).normalized();if(i%7==0)b[i]=Vec3_t(d(rng),d(rng),1).normalized();for(int j=0;j<3;j++){bb[3*i+j]=b[i][j];pp[3*i+j]=p[i][j];}oct[i]=i%8;}
  Check c;c.test=test;reloc_trace::callback=capture;reloc_trace::context=&c;bool refine=test%2;
  reloc_reference::pnp_solver observed(b,oct,p,scales,10,true,10);observed.find_via_ransac(30,refine);
  reloc_trace::callback=nullptr;solve::pnp_solver original(b,oct,p,scales,10,true,10);original.find_via_ransac(30,refine);
  if(observed.solution_is_valid()!=original.solution_is_valid() || observed.get_inlier_flags()!=original.get_inlier_flags()){puts("patch changed result");return 2;}
  if(observed.solution_is_valid()){auto a=observed.get_best_cam_pose(),o=original.get_best_cam_pose();if(memcmp(a.data(),o.data(),128)){puts("patch changed pose");return 2;}}
  sv_pnp_result result;std::vector<unsigned char>mask(n);
  if(sv_pnp_ransac(bb.data(),pp.data(),oct.data(),n,scales.data(),8,10,30,10,refine,nullptr,&result,mask.data(),compare,&c)){puts("C failed");return 2;}
  if(c.pos!=c.entries.size()){puts("missing stages");return 2;}
  if(result.valid!=observed.solution_is_valid()){puts("validity changed");return 2;}
  if(result.valid){auto r=observed.get_best_rotation();auto t=observed.get_best_translation();if(memcmp(r.data(),result.rotation,72)||memcmp(t.data(),result.translation,24)){puts("final pose changed");return 2;}}
  auto expected=observed.get_inlier_flags();for(unsigned i=0;i<expected.size();i++)if(mask[i]!=expected[i]){puts("mask changed");return 2;}
  bad+=c.bad;total+=c.total;if(c.bad)break;
 }
 printf("ransac: %zu/%zu\n",bad,total);return bad?1:0;
}
