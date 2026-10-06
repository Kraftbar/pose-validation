#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Reproducible C99 structural adaptation; original arithmetic retained.
Input: Apache-2 OpenCV 4.6 epnp.cpp. No GPL sources or other port used.
"""
import re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
s=(ROOT/'external/opencv_4.6.0_src/modules/calib3d/src/epnp.cpp').read_text()
s=s[s.index('void epnp::choose_control_points'):s.rindex('\n}')]
s=s.replace('{}','{0}')
a=s.index('void epnp::copy_R_and_t');b=s.index('double epnp::',a);s=s[:a]+s[b:]
s=s.replace('CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);','double pwbuffer[18]; CvMat pwm = cvMat(number_of_correspondences,3,CV_64F,pwbuffer); CvMat *PW0=&pwm;')
s=s.replace('CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);','double mbuffer[144]; CvMat mm=cvMat(2*number_of_correspondences,12,CV_64F,mbuffer); CvMat *M=&mm;')
s=re.sub(r'  cvReleaseMat\([^;]+;','',s)
s=s.replace('void epnp::compute_pose(Mat& R, Mat& t)','void epnp::compute_pose(double *R, double *t)')
s=s.replace('  Mat(3, 1, CV_64F, ts[N]).copyTo(t);','  memcpy(t,ts[N],3*sizeof(double));')
s=s.replace('  Mat(3, 3, CV_64F, Rs[N]).copyTo(R);','  memcpy(R,Rs[N],9*sizeof(double));')
a=s.index('  if (max_nr != 0');b=s.index('  double * pA =',a);s=s[:a]+s[b:]
# Observe state using a callback, never supply expected values to C.
s=s.replace('  compute_barycentric_coordinates();','  emit(e,"cws",cws,sizeof(cws));\n  compute_barycentric_coordinates();\n  emit(e,"alphas",alphas,number_of_correspondences*4*sizeof(double));')
s=s.replace('  compute_L_6x10(ut, l_6x10);','  emit(e,"mtm",mtm,sizeof(mtm));\n  emit(e,"ut",ut,sizeof(ut));\n  compute_L_6x10(ut, l_6x10);')
s=s.replace('  int N = 1;','  emit(e,"betas",Betas,sizeof(Betas));\n  emit(e,"errors",rep_errors,sizeof(rep_errors));\n  emit(e,"Rs",Rs,sizeof(Rs));\n  emit(e,"ts",ts,sizeof(ts));\n  int N = 1;')
names=re.findall(r'(?:void|double) epnp::(\w+)\(',s)
s=s.replace('epnp::','')
for name in names:
 s=re.sub(r'\b'+name+r'\(([^)]*)\)',lambda m:name+'(e'+(', '+m[1] if m[1].strip() and m[1].strip()!='void' else '')+')',s)
# Definitions need typed context; calls already use e.
s=re.sub(r'^(void|double) (\w+)\(e',r'static \1 \2(E *e',s,flags=re.M)
for field in ['number_of_correspondences','pws','us','alphas','pcs','cws','ccs','fu','fv','uc','vc','A1','A2']:
 s=re.sub(r'\b'+field+r'\b','e->'+field,s)
# Restore trace labels altered by token replacement.
s=s.replace('"e->cws"','"cws"').replace('"e->alphas"','"alphas"')
for name in ['dist2','dot','find_betas_approx_1','find_betas_approx_2','find_betas_approx_3','compute_A_and_b_gauss_newton']:
 s=re.sub(r'(static (?:void|double) '+name+r'\([^{}]+?\)\n\{)',r'\1\n  (void)e;',s)
prototypes='\n'.join(m.group(0).split('{')[0].rstrip()+';' for m in re.finditer(r'^static (?:void|double) \w+\([^{}]+?\)\n\{',s,re.M))
pre='''/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * C99 adaptation of OpenCV 4.6 calib3d/src/epnp.cpp. EPnP by Vincent
 * Lepetit, Francesc Moreno-Noguer and Pascal Fua, incorporated by OpenCV.
 * Retained notices: ../reference_cv/LICENSE-M7b-OpenCV.
 * Structural translation script: reference_cv/transcribe_epnp.py. */
#include "rd_cv_pnp.h"
#include "rd_cv_pnp_math.h"
#include <math.h>
#include <string.h>
typedef struct {
 int number_of_correspondences;
 double pws[18],us[12],alphas[24],pcs[18],cws[4][3],ccs[4][3];
 double fu,fv,uc,vc,A1[6],A2[6];
 const rd_cv_pnp_trace *trace;
} E;
static void emit(E *e,const char *label,const void *data,size_t n) {
 if(e->trace&&e->trace->emit)e->trace->emit(e->trace->user,label,data,n);
}
/* Tiny private matrix views replace only the OpenCV C API's allocation and
 * dispatch; all numerical operations live in rd_cv_pnp_math.c. */
typedef struct {int rows,cols;struct {double *db;} data;} CvMat;
#define CV_64F 0
#define CV_SVD 0
#define CV_SVD_MODIFY_A 1
#define CV_SVD_U_T 2
static CvMat cvMat(int r,int c,int type,double *p) {(void)type;CvMat m={r,c,{p}};return m;}
static double cvmGet(const CvMat *m,int r,int c) {return m->data.db[r*m->cols+c];}
static void cvmSet(CvMat *m,int r,int c,double x) {m->data.db[r*m->cols+c]=x;}
static void cvSetZero(CvMat *m) {memset(m->data.db,0,(size_t)m->rows*m->cols*sizeof(double));}
static void cvMulTransposed(const CvMat *a,CvMat *dst,int t) {(void)t;rd_cv_pnp_mtm(a->data.db,a->rows,a->cols,dst->data.db);}
static void cvSVD(const CvMat *a,CvMat *w,CvMat *u,CvMat *v,int flags) {
 double ut[144],vt[144];int m=a->rows,n=a->cols;
 rd_cv_pnp_svd(a->data.db,m,n,w->data.db,ut,vt);
 if(u)for(int i=0;i<n;i++)for(int j=0;j<m;j++)u->data.db[(flags&CV_SVD_U_T)?i*m+j:j*n+i]=ut[i*m+j];
 if(v)for(int i=0;i<n;i++)for(int j=0;j<n;j++)v->data.db[j*n+i]=vt[i*n+j];
}
static void cvInvert(const CvMat *a,CvMat *b,int flag) {(void)flag;rd_cv_pnp_inverse(a->data.db,a->rows,b->data.db);}
static void cvSolve(const CvMat *a,const CvMat *b,CvMat *x,int flag) {(void)flag;rd_cv_pnp_solve(a->data.db,a->rows,a->cols,b->data.db,x->data.db);}
'''
post='''
int rd_cv_pnp(int n,const double *X,const double *x,double T[16],double *rvec,double *tvec,const rd_cv_pnp_trace *trace) {
 if((n!=4&&n!=6)||!X||!x||!T)return 0;
 E e={0};e.number_of_correspondences=n;e.fu=e.fv=1;e.trace=trace;
 for(int i=0;i<3*n;i++)e.pws[i]=(float)X[i];
 for(int i=0;i<2*n;i++)e.us[i]=(float)x[i];
 double R[9],t[3],r[3];float rf[3],tf[3],Rf[9];
 compute_pose(&e,R,t);
 emit(&e,"R",R,sizeof(R));
 rd_cv_pnp_rodrigues_vector(R,r);
 if(rvec)memcpy(rvec,r,sizeof(r));
 if(tvec)memcpy(tvec,t,sizeof(t));
 for(int i=0;i<3;i++){rf[i]=(float)r[i];tf[i]=(float)t[i];}
 rd_cv_pnp_rodrigues_float(rf,Rf);
 for(int i=0;i<16;i++)T[i]=0;
 T[15]=1;
 for(int i=0;i<3;i++){for(int j=0;j<3;j++)T[j*4+i]=Rf[i*3+j];T[12+i]=tf[i];}
 return 1;
}
void rd_cv_pnp6(void *ctx,const double X[6][3],const double x[6][2],double T[16]) {(void)ctx;rd_cv_pnp(6,&X[0][0],&x[0][0],T,NULL,NULL,NULL);}
void rd_cv_pnp4(void *ctx,const double X[4][3],const double x[4][2],double T[16]) {(void)ctx;rd_cv_pnp(4,&X[0][0],&x[0][0],T,NULL,NULL,NULL);}
'''
(ROOT/'rdvio_port/c/rd_cv_pnp.c').write_text(pre+prototypes+'\n'+s+post)
