#include <stella_vslam/solve/essential_5pt.h>
#include <random>
// Access-only observer shim; the implementation remains the installed library.
#define private public
#include <stella_vslam/solve/essential_solver.h>
#undef private
#include "sv_solve_essential_5pt.h"
#include <random>
#include <cstdio>
#include <cstring>
using namespace stella_vslam;
int main(){std::mt19937 rng(8741);std::uniform_real_distribution<double>d(-1,1);long bad[7]={},total[7]={};
 for(int test=0;test<500;test++){
  eigen_alloc_vector<Vec3_t> a(5),b(5);double x[15],y[15],out[90];
  for(int i=0;i<5;i++){for(int j=0;j<3;j++){a[i][j]=d(rng);b[i][j]=d(rng);}a[i].normalize();b[i].normalize();for(int j=0;j<3;j++){x[3*i+j]=a[i][j];y[3*i+j]=b[i][j];}}
  sv_essential_5pt_trace ct;int nc=sv_essential_5pt(x,y,out,&ct);
  bool success;Eigen::Matrix<double,9,4> basis=find_nullspace_of_epipolar_constraint(a,b,success);
  auto poly=form_polynomial_constraint_matrix(basis);Eigen::FullPivLU<Mat10_t> lu(poly.block<10,10>(0,0));Mat10_t elim=lu.solve(poly.block<10,10>(0,10));
  Mat10_t action=Mat10_t::Zero();action.block<3,10>(0,0)=elim.block<3,10>(0,0);action.row(3)=elim.row(4);action.row(4)=elim.row(5);action.row(5)=elim.row(7);action(6,0)=-1;action(7,1)=-1;action(8,3)=-1;action(9,6)=-1;
  Eigen::EigenSolver<Mat10_t> eig(action);auto vec=eig.eigenvectors();
  auto check=[&](int k,double c,double r,int i){total[k]++;if(memcmp(&c,&r,8)){bad[k]++;if(bad[k]<3)printf("test%d stage%d i%d %a %a\n",test,k,i,c,r);}};
  for(int i=0;i<36;i++)check(0,ct.basis[i],basis.data()[i],i);
  for(int i=0;i<200;i++)check(1,ct.polynomial[i],poly.data()[i],i);
  for(int i=0;i<100;i++)check(2,ct.eliminated[i],elim.data()[i],i);
  for(int i=0;i<10;i++){check(3,ct.eigen_real[i],eig.eigenvalues()[i].real(),i);check(3,ct.eigen_imag[i],eig.eigenvalues()[i].imag(),i);}
  for(int i=0;i<100;i++){check(4,ct.vectors_real[i],vec.data()[i].real(),i);check(4,ct.vectors_imag[i],vec.data()[i].imag(),i);}
  std::vector<std::pair<int,int>> pairs;solve::essential_solver solver(a,b,pairs,true);auto es=solver.compute_E_21_minimal(a,b);check(5,nc,es.size(),0);
  for(int k=0;k<nc && k<(int)es.size();k++)for(int i=0;i<9;i++)check(6,out[9*k+i],es[k].data()[i],i);
 }
 long fail=0;for(int i=0;i<7;i++){printf("stage%d %ld/%ld\n",i,bad[i],total[i]);fail+=bad[i];}return fail?1:0;
}
