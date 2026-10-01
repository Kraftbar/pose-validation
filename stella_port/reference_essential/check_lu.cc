#include <Eigen/Dense>
#include "sv_eigen_fullpivlu.h"
#include <random>
#include <cstdio>
#include <cstring>
int main(){
 std::mt19937 rng(9182);std::uniform_real_distribution<double>d(-1,1);long bad[4]={},total[4]={};
 for(int t=0;t<5000;t++){
  int n=t%2?10:9;Eigen::MatrixXd a(n,n);
  for(int j=0;j<n;j++)for(int i=0;i<n;i++)a(i,j)=i>=(t%3? n:5)?0:d(rng);
  Eigen::FullPivLU<Eigen::MatrixXd> lu(a);sv_eigen_fullpivlu c;sv_eigen_fullpivlu_compute(a.data(),n,&c);
  auto check=[&](int k,const double*x,const double*y,int len){for(int i=0;i<len;i++){total[k]++;if(memcmp(x+i,y+i,8)){bad[k]++;if(bad[k]<3)printf("t%d stage%d i%d %a %a\n",t,k,i,x[i],y[i]);}}};
  check(0,c.a,lu.matrixLU().data(),n*n);
  if(c.rank!=lu.rank()){printf("rank mismatch\n");return 1;}
  double v[100];int dim=sv_eigen_fullpivlu_kernel(&c,v);Eigen::MatrixXd ker=lu.kernel();check(1,v,ker.data(),n*(dim?dim:1));
  Eigen::MatrixXd b(n,10);for(int i=0;i<n*10;i++)b.data()[i]=d(rng);
  Eigen::MatrixXd x=lu.solve(b);sv_eigen_fullpivlu_solve(&c,b.data(),10,v);check(2,v,x.data(),n*10);
 }
 long fail=0;for(int i=0;i<3;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
