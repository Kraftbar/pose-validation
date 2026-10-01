/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include "sv_solve_essential_ransac.h"
#include "sv_solve_essential_5pt.h"
#include "sv_eigen_svd.h"
#include "sv_linalg.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

int sv_essential_nonminimal(const double *b1,const double *b2,unsigned n,double E[9]){
 if(n<8 || !b1 || !b2 || !E)return -1;
 double *a=malloc((size_t)n*9*sizeof(double));if(!a)return -1;
 for(unsigned i=0;i<n;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)a[i+n*(3*j+k)]=b2[3*i+j]*b1[3*i+k];
 double v[81],s[9];int rank;sv_eigen_jacobisvd_Nx9_v(a,(int)n,v,s,&rank);free(a);
 double init[9],u[9],vv[9],sv[3],scaled[9],vt[9];
 for(int i=0;i<9;i++)init[(i%3)*3+i/3]=v[i+72];
 sv_eigen_jacobisvd_3x3(init,u,vv,sv);sv[2]=0;
 for(int j=0;j<3;j++)for(int i=0;i<3;i++)scaled[i+3*j]=u[i+3*j]*sv[j];
 sv_mat3_transpose(vv,vt);sv_mat3_mul(scaled,vt,E);return 0;
}
unsigned sv_essential_check_inliers(const double E[9],const double *b1,const double *b2,unsigned n,unsigned char *mask,float *cost){
 double Et[9];sv_mat3_transpose(E,Et);
 /* util::cos at one degree: argument is converted to float, then its
  * first polynomial branch is taken. This is NOT libm cosf. */
 const float angle=(float)(3.14159265358979323846/180.0);
 const float sq=angle*angle;
 const float threshold=.99940307f+sq*(-.49558072f+.03679168f*sq);
 unsigned count=0;*cost=0;
 for(unsigned i=0;i<n;i++){
  double p1[3],p2[3],cross[3];
  sv_mat3_mulv(E,b1+3*i,p2);sv_vec3_cross(p2,b2+3*i,cross);
  float c2=(float)(sv_vec3_norm(cross)/sv_vec3_norm(p2));
  sv_mat3_mulv(Et,b2+3*i,p1);sv_vec3_cross(p1,b1+3*i,cross);
  float c1=(float)(sv_vec3_norm(cross)/sv_vec3_norm(p1));
  float worst=c2<c1?c2:c1;
  mask[i]=threshold<worst;count+=mask[i];
  *cost=(float)(*cost+(1.0-(mask[i]?worst:threshold)));
 }
 return count;
}
int sv_essential_ransac(const double *b1,const double *b2,unsigned n,unsigned iters,int recompute,unsigned sample,sv_mt19937 *rng,sv_essential_result *result,unsigned char *mask,sv_essential_trace_fn trace,void *user){
 if(!result || (n && (!b1 || !b2 || !mask)) || (sample!=5 && sample<8) || n>2147483647u)return -1;
 memset(result,0,sizeof(*result));if(n<sample)return 0;
 result->cost=FLT_MAX;memset(mask,0,n);
 double *x=malloc((size_t)n*3*sizeof(double)),*y=malloc((size_t)n*3*sizeof(double));
 unsigned *indices=malloc((size_t)sample*sizeof(unsigned));unsigned char *temp=malloc(n);
 if(!x || !y || !indices || !temp){free(x);free(y);free(indices);free(temp);return -1;}
 sv_mt19937 local;if(!rng){sv_mt19937_init_default(&local);rng=&local;}
 int rc=0;
 for(unsigned iter=0;iter<iters;iter++){
  sv_create_random_array(sample,0,n-1,rng,indices);
  for(unsigned i=0;i<sample;i++){memcpy(x+3*i,b1+3*indices[i],24);memcpy(y+3*i,b2+3*indices[i],24);}
  double candidates[90];int nc;sv_essential_5pt_trace minimal;
  if(sample==5)nc=sv_essential_5pt(x,y,candidates,trace?&minimal:NULL);
  else{nc=1;if(sv_essential_nonminimal(x,y,sample,candidates))nc=-1;}
  if(nc<0){rc=-1;break;}
  float costs[10];unsigned counts[10];
  for(int k=0;k<nc;k++){
   counts[k]=sv_essential_check_inliers(candidates+9*k,b1,b2,n,temp,costs+k);
   if(counts[k]>sample && result->cost>costs[k]){
    result->cost=costs[k];result->inliers=counts[k];memcpy(result->E,candidates+9*k,72);memcpy(mask,temp,n);
   }
  }
  if(trace)trace(user,iter,indices,sample,candidates,(unsigned)nc,costs,counts,sample==5?&minimal:NULL,result);
 }
 result->valid=result->cost<FLT_MAX;
 if(!rc && recompute && result->valid && result->inliers>=8){
  unsigned count=0;for(unsigned i=0;i<n;i++)if(mask[i]){memcpy(x+3*count,b1+3*i,24);memcpy(y+3*count,b2+3*i,24);count++;}
  rc=sv_essential_nonminimal(x,y,count,result->E);
  if(!rc)result->inliers=sv_essential_check_inliers(result->E,b1,b2,n,mask,&result->cost);
 }
 free(x);free(y);free(indices);free(temp);return rc;
}
