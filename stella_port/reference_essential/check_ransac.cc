#include <stella_vslam/solve/essential_solver.h>
#include "sv_solve_essential_ransac.h"
#include <random>
#include <cstring>
#include <cstdio>
using namespace stella_vslam;
int main(){std::mt19937 rng(4927);std::uniform_real_distribution<double>d(-1,1);long bad[5]={},total[5]={};
 for(int test=0;test<100;test++){
  int n=8+(test*17)%123; eigen_alloc_vector<Vec3_t>a(n),b(n);std::vector<std::pair<int,int>>pairs;std::vector<double>x(3*n),y(3*n);
  for(int i=0;i<n;i++){
   Vec3_t p(d(rng),d(rng),3+d(rng));a[i]=p.normalized();
   b[i]=(p+Vec3_t(.2,-.1,.3)).normalized();if(i%5==0 && test%3==0)b[i]=Vec3_t(d(rng),d(rng),1).normalized();
   for(int j=0;j<3;j++){x[3*i+j]=a[i][j];y[3*i+j]=b[i][j];}pairs.emplace_back(i,i);
  }
  auto check=[&](int k,double c,double r){total[k]++;if(memcmp(&c,&r,8)){bad[k]++;if(bad[k]<4)printf("test%d stage%d %a %a\n",test,k,c,r);}};
  double e[9];sv_essential_nonminimal(x.data(),y.data(),n,e);auto ne=solve::essential_solver::compute_E_21_nonminimal(a,b);for(int i=0;i<9;i++)check(0,e[i],ne.data()[i]);
  solve::essential_solver solver(a,b,pairs,true);bool recompute=test%2;solver.find_via_ransac(30,recompute);
  sv_essential_result r;std::vector<unsigned char>mask(n);int rc=sv_essential_ransac(x.data(),y.data(),n,30,recompute,5,nullptr,&r,mask.data(),nullptr,nullptr);
  if(rc){printf("error test%d\n",test);return 1;}
  check(1,r.valid,solver.solution_is_valid());check(2,r.cost,solver.get_best_cost());
  if(r.valid && solver.solution_is_valid()){auto ne=solver.get_best_E_21();for(int i=0;i<9;i++)check(3,r.E[i],ne.data()[i]);}
  auto in=solver.get_inlier_matches();for(int i=0;i<n;i++)check(4,mask[i],in[i]);
 }
 long fail=0;for(int i=0;i<5;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
