/* SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause */
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


/* stella_vslam e445b545 solve/pnp_solver.cc; EPnP and RANSAC. */
#include "sv_pnp.h"
#include "sv_linalg.h"
#include "sv_eigen_svd.h"
#include "sv_eigen_pnp.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
typedef struct {sv_pnp_trace_fn fn;void *user;} trace_ctx;
static void emit(trace_ctx *c,const char *name,const double *p,unsigned n){if(c->fn)c->fn(c->user,name,p,n);}
static void scalar(trace_ctx *c,const char *name,double v){emit(c,name,&v,1);}
static void centroid(const double *p,unsigned n,double c[3]){
    c[0]=c[1]=c[2]=0;for(unsigned i=0;i<n;i++)for(int k=0;k<3;k++)c[k]+=p[3*i+k];
    for(int k=0;k<3;k++)c[k]/=n;
}
static void control_points(const double *p,unsigned n,double cw[12],double *pw0,trace_ctx *tr){
    centroid(p,n,cw);
    for(unsigned i=0;i<n;i++)for(int k=0;k<3;k++)pw0[i+n*k]=p[3*i+k]-cw[k];
    double gram[9],u[9],v[9],s[3];sv_pnp_eigen_gram(pw0,(int)n,3,gram);
    emit(tr,"PW0",pw0,n*3);emit(tr,"PW0tPW0",gram,9);
    sv_pnp_eigen_svd(gram,3,3,u,v,s);emit(tr,"control_U",u,9);emit(tr,"control_D",s,3);
    for(int j=1;j<4;j++){double k=sqrt(s[j-1]/n);for(int i=0;i<3;i++)cw[3*j+i]=cw[i]+k*u[i+3*(j-1)];}
}
static void barycentric(const double *p,unsigned n,const double *cw,double *alpha,trace_ctx *tr){
    double cc[9],u[9],v[9],s[3],diag[9]={0},tmp[9],ut[9],inv[9];
    for(int j=0;j<3;j++)for(int i=0;i<3;i++)cc[i+3*j]=cw[3*(j+1)+i]-cw[i];
    sv_eigen_jacobisvd_3x3(cc,u,v,s);for(int i=0;i<3;i++)diag[i+3*i]=s[i]>1e-6?1/s[i]:0;
    sv_mat3_mul(v,diag,tmp);sv_mat3_transpose(u,ut);sv_mat3_mul(tmp,ut,inv);emit(tr,"CC_inv",inv,9);
    for(unsigned i=0;i<n;i++){
        double diff[3];for(int k=0;k<3;k++)diff[k]=p[3*i+k]-cw[k];
        sv_mat3_mulv(inv,diff,alpha+4*i+1);
        alpha[4*i]=((1.0-alpha[4*i+1])-alpha[4*i+2])-alpha[4*i+3];
    }
}
static void compute_l_rho(const double *u,const double *cw,double l[60],double rho[6]){
    const int aa[6]={0,0,0,1,1,2},bb[6]={1,2,3,2,3,3};
    const int x[10]={0,0,1,0,1,2,0,1,2,3},y[10]={0,1,1,2,2,2,3,3,3,3};
    for(int k=0;k<6;k++){
        double d[4][3],c[3];
        for(int i=0;i<4;i++)for(int j=0;j<3;j++)d[i][j]=u[3*aa[k]+j+12*(11-i)]-u[3*bb[k]+j+12*(11-i)];
        for(int i=0;i<10;i++)l[k+6*i]=(x[i]==y[i]?1.0:2.0)*sv_vec3_dot(d[x[i]],d[y[i]]);
        for(int i=0;i<3;i++)c[i]=cw[3*aa[k]+i]-cw[3*bb[k]+i];
        rho[k]=sv_vec3_dot(c,c);
    }
}
static void initial_betas(const double *l,const double *rho,int mode,double *betas,trace_ctx *tr){
    int n=mode==2?3:mode==3?5:4;double a[30],b[5];int cols[4]={0,1,3,6};
    for(int j=0;j<n;j++)for(int i=0;i<6;i++)a[i+6*j]=l[i+6*(mode==4?cols[j]:j)];
    sv_pnp_eigen_svd_solve(a,6,n,rho,b);emit(tr,"beta_solution",b,(unsigned)n);
    if(mode==4){
        betas[0]=sqrt(fabs(b[0]));
        for(int i=1;i<4;i++)betas[i]=(b[0]<0?-b[i]:b[i])/betas[0];
    }else{
        if(b[0]<0){betas[0]=sqrt(-b[0]);betas[1]=b[2]<0?sqrt(-b[2]):0;}
        else{betas[0]=sqrt(b[0]);betas[1]=b[2]>0?sqrt(b[2]):0;}
        if(b[1]<0)betas[0]=-betas[0];
        betas[2]=mode==3?b[3]/betas[0]:0;betas[3]=0;
    }
}
static void gn_system(const double *l,const double *rho,const double *beta,double *a,double *b){
    for (unsigned int i = 0; i < 6; ++i) {
        a[i+6*0] = 2 * l[i+6*0] * beta[0] + l[i+6*1] * beta[1]
                  + l[i+6*3] * beta[2] + l[i+6*6] * beta[3];
        a[i+6*1] = l[i+6*1] * beta[0] + 2 * l[i+6*2] * beta[1]
                  + l[i+6*4] * beta[2] + l[i+6*7] * beta[3];
        a[i+6*2] = l[i+6*3] * beta[0] + l[i+6*4] * beta[1]
                  + 2 * l[i+6*5] * beta[2] + l[i+6*8] * beta[3];
        a[i+6*3] = l[i+6*6] * beta[0] + l[i+6*7] * beta[1]
                  + l[i+6*8] * beta[2] + 2 * l[i+6*9] * beta[3];

        b[i] = rho[i] - (l[i+6*0] * beta[0] * beta[0] + l[i+6*1] * beta[0] * beta[1] + l[i+6*2] * beta[1] * beta[1] + l[i+6*3] * beta[0] * beta[2] + l[i+6*4] * beta[1] * beta[2] + l[i+6*5] * beta[2] * beta[2] + l[i+6*6] * beta[0] * beta[3] + l[i+6*7] * beta[1] * beta[3] + l[i+6*8] * beta[2] * beta[3] + l[i+6*9] * beta[3] * beta[3]);
    }
}
static void estimate(const double *p,const double *pc,unsigned n,double *r,double *t,trace_ctx *tr){
    double pc0[3],pw0[3],cm[9]={0},u[9],v[9],vt[9],s[3],prod[3];
    centroid(pc,n,pc0);centroid(p,n,pw0);
    for(unsigned k=0;k<n;k++)for(int j=0;j<3;j++)for(int i=0;i<3;i++)cm[i+3*j]+=(pc[3*k+i]-pc0[i])*(p[3*k+j]-pw0[j]);
    for(int i=0;i<9;i++)if(!isfinite(cm[i])){
        /* Match the isolated reference's documented NumericalIssue guard;
         * upstream otherwise reads uninitialized singular vectors. */
        emit(tr,"CM",cm,9);scalar(tr,"CM_failed",1);
        for(int k=0;k<9;k++)r[k]=NAN;
        for(int k=0;k<3;k++)t[k]=NAN;
        return;
    }
    sv_pnp_eigen_svd(cm,3,3,u,v,s);sv_mat3_transpose(v,vt);
    emit(tr,"CM",cm,9);emit(tr,"CM_U",u,9);emit(tr,"CM_Vt",vt,9);
    sv_pnp_eigen_mul3(u,vt,r);
    if(sv_mat3_det(r)<0){
        double sign[9]={1,0,0,0,1,0,0,0,-1},tmp[9];sv_mat3_mul(u,sign,tmp);
        sv_pnp_eigen_mixed_mul3(tmp,vt,r);
    }
    sv_mat3_mulv(r,pw0,prod);for(int i=0;i<3;i++)t[i]=pc0[i]-prod[i];
}
static double reprojection(const double *b,const double *p,unsigned n,const double *r,const double *t){
    double error=0;for(unsigned i=0;i<n;i++){
        double pc[3];sv_mat3_mulv(r,p+3*i,pc);for(int j=0;j<3;j++)pc[j]+=t[j];
        error+=1.0-sv_vec3_dot(pc,b+3*i)/sv_vec3_norm(pc);
    }return error/n;
}
int sv_pnp_compute_pose(const double *b,const double *p,unsigned n,unsigned iters,double r[9],double t[3],double *error,sv_pnp_trace_fn trace,void *user){
    if(!b || !p || !r || !t || !error || n<4 || n>2147483647u/24u)return -1;
    trace_ctx tr={trace,user};
    double *pw0=malloc((size_t)n*3*sizeof(double)),*alpha=malloc((size_t)n*4*sizeof(double));
    double *m=malloc((size_t)n*24*sizeof(double)),*pc=malloc((size_t)n*3*sizeof(double));
    if(!pw0 || !alpha || !m || !pc){free(pw0);free(alpha);free(m);free(pc);return -1;}
    double cw[12];control_points(p,n,cw,pw0,&tr);emit(&tr,"controls",cw,12);
    barycentric(p,n,cw,alpha,&tr);emit(&tr,"alphas",alpha,4*n);
    for(unsigned k=0;k<n;k++)for(int j=0;j<4;j++){
        double a=alpha[4*k+j],u=b[3*k]/b[3*k+2],v=b[3*k+1]/b[3*k+2];
        m[2*k+2*n*(3*j)]=a;m[2*k+2*n*(3*j+1)]=0;m[2*k+2*n*(3*j+2)]=-a*u;
        m[2*k+1+2*n*(3*j)]=0;m[2*k+1+2*n*(3*j+1)]=a;m[2*k+1+2*n*(3*j+2)]=-a*v;
    }
    emit(&tr,"M",m,24*n);
    double gram[144],u[144],v[144],s[12],l[60],rho[6];sv_pnp_eigen_gram(m,2*n,12,gram);emit(&tr,"MtM",gram,144);
    sv_pnp_eigen_svd(gram,12,12,u,v,s);emit(&tr,"U12",u,144);
    compute_l_rho(u,cw,l,rho);emit(&tr,"L",l,60);emit(&tr,"rho",rho,6);
    *error=DBL_MAX;
    for(int k=0;k<9;k++)r[k]=NAN;
    for(int k=0;k<3;k++)t[k]=NAN;
    for(int mode=2;mode<=4;mode++){
        double beta[4];initial_betas(l,rho,mode,beta,&tr);emit(&tr,"betas",beta,4);
        for(unsigned k=0;k<iters;k++){
            double a[24],bb[6],step[4];gn_system(l,rho,beta,a,bb);emit(&tr,"GN_A",a,24);emit(&tr,"GN_B",bb,6);
            sv_pnp_eigen_qr_solve(a,bb,step);for(int j=0;j<4;j++)beta[j]+=step[j];emit(&tr,"GN_betas",beta,4);
        }
        emit(&tr,"refined",beta,4);
        double cc[12]={0};for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<3;k++)cc[3*i+k]+=beta[j]*u[3*i+k+12*(11-j)];
        emit(&tr,"ccs",cc,12);
        for(unsigned i=0;i<n;i++)for(int k=0;k<3;k++)pc[3*i+k]=((alpha[4*i]*cc[k]+alpha[4*i+1]*cc[3+k])+alpha[4*i+2]*cc[6+k])+alpha[4*i+3]*cc[9+k];
        if((pc[2]>0)!=(b[2]>0))for(unsigned i=0;i<n*3;i++)pc[i]*=-1;
        emit(&tr,"pcs",pc,3*n);
        double rr[9],tt[3];estimate(p,pc,n,rr,tt,&tr);emit(&tr,"rotation",rr,9);emit(&tr,"translation",tt,3);
        double e=reprojection(b,p,n,rr,tt);scalar(&tr,"error",e);
        if(e<*error){*error=e;memcpy(r,rr,9*sizeof(double));memcpy(t,tt,3*sizeof(double));}
    }
    free(pw0);free(alpha);free(m);free(pc);return 0;
}

/* stella util::cos(float), including the angle wrapping used by OpenCV's
 * cvFloor. Default ORB scales take the first polynomial branch. */
static float radial_threshold(float scale){
    const float pi=3.14159265358979f,half=pi/2.0f,twopi=2.0f*pi,inv=1.0f/twopi;
    float v=(float)(scale*(1.0*3.14159265358979323846/180.0));
    v=v-floorf(v*inv)*twopi;v=0.0f<v?v:-v;
    float sign=1;
    if(v<half){}
    else if(v<pi){v=pi-v;sign=-1;}
    else if(v<3.0f*half){v=v-pi;sign=-1;}
    else v=twopi-v;
    float v2=v*v;return sign*(0.99940307f+v2*(-0.49558072f+0.03679168f*v2));
}
static unsigned check_inliers(const double *b,const double *p,const float *threshold,unsigned n,
                              const double *r,const double *t,unsigned char *mask,double *cost){
    unsigned count=0;*cost=0;
    for(unsigned i=0;i<n;i++){
        double pc[3];sv_mat3_mulv(r,p+3*i,pc);for(int j=0;j<3;j++)pc[j]+=t[j];
        double cosine=sv_vec3_dot(pc,b+3*i)/sv_vec3_norm(pc);
        mask[i]=threshold[i]<cosine;
        if(mask[i]){*cost+=1-cosine;count++;}
        else *cost+=1-threshold[i]; /* integer 1: upstream subtraction is float */
    }return count;
}
int sv_pnp_ransac(const double *b,const double *p,const int *octaves,unsigned n,const float *scales,unsigned levels,
                  unsigned min_inliers,unsigned iters,unsigned gn_iters,int recompute,sv_mt19937 *rng,
                  sv_pnp_result *result,unsigned char *mask,sv_pnp_trace_fn trace,void *user){
    if(!result || !scales || !levels || (n && (!b || !p || !octaves || !mask)) || n>2147483647u/24u)return -1;
    for(unsigned i=0;i<n;i++)if(octaves[i]<0 || (unsigned)octaves[i]>=levels)return -1;
    memset(result,0,sizeof(*result));if(n)memset(mask,0,n);
    if(n<4 || n<min_inliers)return 0;
    float *threshold=malloc((size_t)n*sizeof(float));unsigned char *temp=malloc(n);
    double *x=malloc((size_t)n*3*sizeof(double)),*y=malloc((size_t)n*3*sizeof(double));
    if(!threshold || !temp || !x || !y){free(threshold);free(temp);free(x);free(y);return -1;}
    for(unsigned i=0;i<n;i++)threshold[i]=radial_threshold(scales[octaves[i]]);
    trace_ctx tr={trace,user};sv_mt19937 local;if(!rng){sv_mt19937_init_default(&local);rng=&local;}
    result->cost=DBL_MAX;int rc=0;
    for(unsigned iter=0;iter<iters;iter++){
        unsigned indices[4];sv_create_random_array(4,0,n-1,rng,indices);
        for(int i=0;i<4;i++){scalar(&tr,"sample",indices[i]);memcpy(x+3*i,b+3*indices[i],24);memcpy(y+3*i,p+3*indices[i],24);}
        double r[9],t[3],error,cost;
        if(sv_pnp_compute_pose(x,y,4,gn_iters,r,t,&error,trace,user)){rc=-1;break;}
        unsigned count=check_inliers(b,p,threshold,n,r,t,temp,&cost);
        scalar(&tr,"cost",cost);scalar(&tr,"inliers",count);for(unsigned i=0;i<n;i++)scalar(&tr,"mask",temp[i]);
        if(count>min_inliers && result->cost>cost){result->cost=cost;result->inliers=count;memcpy(result->rotation,r,72);memcpy(result->translation,t,24);memcpy(mask,temp,n);}
    }
    result->valid=result->cost<DBL_MAX;scalar(&tr,"valid",result->valid);
    if(!rc && recompute && result->valid){
        unsigned count=0;for(unsigned i=0;i<n;i++)if(mask[i]){memcpy(x+3*count,b+3*i,24);memcpy(y+3*count,p+3*i,24);count++;}
        double error;rc=sv_pnp_compute_pose(x,y,count,gn_iters,result->rotation,result->translation,&error,trace,user);
        /* Upstream deliberately retains the original RANSAC mask. */
    }
    free(threshold);free(temp);free(x);free(y);return rc;
}
