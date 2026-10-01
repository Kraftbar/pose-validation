#include <Eigen/Dense>
#include "sv_eigen_eigensolver.h"
#include <random>
#include <cstdio>
#include <cstring>
using Mat=Eigen::Matrix<double,10,10>;
int main(){std::mt19937 rng(512);std::uniform_real_distribution<double>d(-1,1);long bad[4]={},total[4]={};
 for(int c=0;c<100;c++){
  Mat a;for(int i=0;i<100;i++)a.data()[i]=d(rng);
  Eigen::HessenbergDecomposition<Mat> h(a);Mat eh=h.matrixH(),eq=h.matrixQ();double ch[100],cq[100];sv_eigen_hessenberg10(a.data(),ch,cq);
  auto check=[&](int k,const double*x,const double*y){for(int i=0;i<100;i++){total[k]++;if(memcmp(x+i,y+i,8)){bad[k]++;if(bad[k]<4)printf("case%d stage%d i%d %a %a\n",c,k,i,x[i],y[i]);}}};
  check(0,ch,eh.data());check(1,cq,eq.data());
  Eigen::RealSchur<Mat> s(a);sv_eigen_realschur10(a.data(),ch,cq);check(2,ch,s.matrixT().data());check(3,cq,s.matrixU().data());
 }
 long fail=0;for(int i=0;i<4;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
