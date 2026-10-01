#include <Eigen/Dense>
#include "sv_eigen_eigensolver.h"
#include <random>
#include <cstdio>
#include <cstring>
using Mat=Eigen::Matrix<double,10,10>;
int main(){std::mt19937 rng(512);std::uniform_real_distribution<double>d(-1,1);long bad[4]={},total[4]={};
 for(int c=0;c<1008;c++){
  Mat a;for(int i=0;i<100;i++)a.data()[i]=d(rng);
  if(c%5==0)for(int j=0;j<10;j++)for(int i=j+1;i<10;i++)a(i,j)=0;
  if(c==1000)a.setZero();
  if(c==1001)a.setIdentity();
  if(c==1002){a.setZero();for(int i=0;i<10;i++)a(i,i)=i;}
  if(c==1003){a.setIdentity();for(int i=0;i<9;i++)a(i,i+1)=1;}
  if(c==1004){a.setZero();for(int i=0;i<10;i+=2){a(i,i+1)=-1;a(i+1,i)=1;}}
  if(c==1005)a*=1.e-200;
  if(c==1006)a*=1.e200;
  if(c==1007)a.setConstant(1.0);
  Eigen::EigenSolver<Mat> s(a);auto vec=s.eigenvectors();double er[10],ei[10],vr[100],vi[100];
  if(sv_eigen_eigensolver10(a.data(),er,ei,vr,vi)){printf("failed case%d\n",c);return 1;}
  auto check=[&](int k,double x,double y,int i){total[k]++;if(memcmp(&x,&y,8)){bad[k]++;if(bad[k]<4)printf("case%d stage%d i%d %a %a\n",c,k,i,x,y);}};
  for(int i=0;i<10;i++){check(0,er[i],s.eigenvalues()[i].real(),i);check(1,ei[i],s.eigenvalues()[i].imag(),i);}
  for(int i=0;i<100;i++){check(2,vr[i],vec.data()[i].real(),i);check(3,vi[i],vec.data()[i].imag(),i);}
 }
 long fail=0;for(int i=0;i<4;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
