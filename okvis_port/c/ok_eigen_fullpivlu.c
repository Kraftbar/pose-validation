/* SPDX-License-Identifier: MPL-2.0 */
/* Copied from stella_port/c/sv_eigen_fullpivlu.c (same author, MPL-2.0) with the identifiers renamed sv_eigen_ -> ok_eigen_; no other change. */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 * Copyright (C) 2006-2009 Benoit Jacob; 2009 Gael Guennebaud.
 * Eigen 3.4 FullPivLU.h and SSE2 TriangularSolverMatrix.h, panel width 4. */
#include "ok_eigen_fullpivlu.h"
#include <math.h>
#include <float.h>
#include <string.h>
#define A(r,c) a[(r)+n*(c)]
static void swap(double *a,double *b){double t=*a;*a=*b;*b=t;}
static void iswap(int *a,int *b){int t=*a;*a=*b;*b=t;}
int ok_eigen_fullpivlu_compute(const double *input,int n,ok_eigen_fullpivlu *lu){
    if(!input || !lu || n<1 || n>10)return -1;
    double *a=lu->a;memcpy(a,input,(size_t)n*n*sizeof(double));
    lu->n=n;lu->nonzero=n;lu->maxpivot=0;
    for(int i=0;i<n;i++)lu->p[i]=lu->q[i]=i;
    for(int k=0;k<n;k++){
        double biggest=-1;int row=k,col=k;
        for(int j=k;j<n;j++)for(int i=k;i<n;i++)if(fabs(A(i,j))>biggest){biggest=fabs(A(i,j));row=i;col=j;}
        if(biggest==0){lu->nonzero=k;break;}
        if(biggest>lu->maxpivot)lu->maxpivot=biggest;
        for(int j=0;j<n;j++)swap(&A(k,j),&A(row,j));
        for(int i=0;i<n;i++)swap(&A(i,k),&A(i,col));
        iswap(&lu->p[k],&lu->p[row]);iswap(&lu->q[k],&lu->q[col]);
        for(int i=k+1;i<n;i++)A(i,k)/=A(k,k);
        for(int j=k+1;j<n;j++)for(int i=k+1;i<n;i++)A(i,j)-=A(i,k)*A(k,j);
    }
    lu->rank=0;double threshold=lu->maxpivot*(DBL_EPSILON*n);
    for(int i=0;i<lu->nonzero;i++)lu->rank+=fabs(A(i,i))>threshold;
    return 0;
}
/* SSE2 Eigen multiple-RHS triangular solve: panel updates accumulate before
 * subtraction, within-panel updates subtract each term, diagonal reciprocal. */
static void triangular(const double *a,int ld,int n,double *b,int bd,int cols,int lower){
    for(int start=0;start<n;start+=4){
        int width=n-start<4?n-start:4;
        for(int k=0;k<width;k++){
            int i=lower?start+k:n-start-k-1;
            double inv=lower?1.0:1.0/a[i+ld*i];
            for(int j=0;j<cols;j++){
                double v=(b[i+bd*j]*=inv);
                for(int t=k+1;t<width;t++){
                    int r=lower?start+t:n-start-t-1;
                    b[r+bd*j]-=v*a[r+ld*i];
                }
            }
        }
        int s=lower?start:n-start-width;
        for(int j=0;j<cols;j++)for(int t=start+width;t<n;t++){
            int r=lower?t:n-t-1;double acc=0;
            for(int k=0;k<width;k++)acc+=a[r+ld*(s+k)]*b[s+k+bd*j];
            b[r+bd*j]+=(-1.0)*acc;
        }
    }
}
int ok_eigen_fullpivlu_solve(const ok_eigen_fullpivlu *lu,const double *b,int cols,double *out){
    if(!lu || !b || !out || cols<1 || cols>10)return -1;
    int n=lu->n;double c[100];
    for(int j=0;j<cols;j++)for(int i=0;i<n;i++)c[i+n*j]=b[lu->p[i]+n*j];
    triangular(lu->a,n,n,c,n,cols,1);
    triangular(lu->a,n,lu->rank,c,n,cols,0);
    memset(out,0,(size_t)n*cols*sizeof(double));
    for(int j=0;j<cols;j++)for(int i=0;i<lu->rank;i++)out[lu->q[i]+n*j]=c[i+n*j];
    return 0;
}
int ok_eigen_fullpivlu_kernel(const ok_eigen_fullpivlu *lu,double *out){
    if(!lu || !out)return -1;
    int n=lu->n,r=lu->rank,d=n-r,pivots[10],p=0;
    memset(out,0,(size_t)n*(d?d:1)*sizeof(double));
    if(!d)return 0;
    double threshold=lu->maxpivot*(DBL_EPSILON*n),m[100]={0};
    for(int i=0;i<lu->nonzero;i++)if(fabs(lu->a[i+n*i])>threshold)pivots[p++]=i;
    for(int i=0;i<r;i++)for(int j=i;j<n;j++)m[i+r*j]=lu->a[pivots[i]+n*j];
    for(int j=0;j<r;j++)for(int i=0;i<r;i++)swap(&m[i+r*j],&m[i+r*pivots[j]]);
    triangular(m,r,r,m+r*r,r,d,0);
    for(int j=r-1;j>=0;j--)for(int i=0;i<r;i++)swap(&m[i+r*j],&m[i+r*pivots[j]]);
    for(int j=0;j<d;j++){
        for(int i=0;i<r;i++)out[lu->q[i]+n*j]=-m[i+r*(r+j)];
        out[lu->q[r+j]+n*j]=1;
    }
    return d;
}
