#include <Eigen/Dense>
#include "c/sv_eigen_pnp.h"
#include <random>
#include <cstring>
#include <cstdio>
int main(){std::mt19937 rng(173);std::uniform_real_distribution<double>d(-1,1);long bad[6]={},total[6]={};
 for(int t=0;t<500;t++){
  int cols=3+t%3,rows=6;if(t%5==3)rows=cols=12;if(t%5==4)rows=cols=3;
  Eigen::MatrixXd a(rows,cols);for(int i=0;i<a.size();i++)a.data()[i]=d(rng);
  Eigen::JacobiSVD<Eigen::MatrixXd>s(a,Eigen::ComputeFullU|Eigen::ComputeFullV);
  double u[144],v[144],sv[12],x[12],g[144];sv_pnp_eigen_svd(a.data(),rows,cols,u,v,sv);
  auto check=[&](int stage,const double *x,const double *y,int n){for(int i=0;i<n;i++){total[stage]++;if(memcmp(x+i,y+i,8)){bad[stage]++;if(bad[stage]<4)printf("t%d shape%dx%d stage%d i%d %a %a\n",t,rows,cols,stage,i,x[i],y[i]);}}};
  check(0,u,s.matrixU().data(),rows*rows);check(1,v,s.matrixV().data(),cols*cols);check(2,sv,s.singularValues().data(),cols);
  Eigen::VectorXd b(rows);for(int i=0;i<rows;i++)b[i]=d(rng);Eigen::VectorXd sol=s.solve(b);
  sv_pnp_eigen_svd_solve(a.data(),rows,cols,b.data(),x);check(3,x,sol.data(),cols);
  Eigen::Matrix<double,6,4>qa;Eigen::Matrix<double,6,1>qb;
  for(int i=0;i<24;i++)qa.data()[i]=d(rng);for(int i=0;i<6;i++)qb[i]=d(rng);
  Eigen::Vector4d qx=qa.householderQr().solve(qb);double cx[4];sv_pnp_eigen_qr_solve(qa.data(),qb.data(),cx);check(5,cx,qx.data(),4);
  Eigen::MatrixXd gram=a.transpose()*a;sv_pnp_eigen_gram(a.data(),rows,cols,g);check(4,g,gram.data(),cols*cols);
  int nr=480+t*3;Eigen::MatrixXd large(nr,12);
  for(int i=0;i<large.size();i++)large.data()[i]=d(rng);
  Eigen::Matrix<double,12,12> lg=large.transpose()*large;
  sv_pnp_eigen_gram(large.data(),nr,12,g);check(4,g,lg.data(),144);
 }
 long fail=0;for(int i=0;i<6;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
