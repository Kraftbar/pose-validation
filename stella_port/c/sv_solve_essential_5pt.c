/* SPDX-License-Identifier: BSD-2-Clause AND MIT */
// Copyright (c) 2011 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


/* The stella_vslam solver adapter below also retains its BSD notice. */
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


#include "sv_solve_essential_5pt.h"
#include "sv_eigen_fullpivlu.h"
#include "sv_eigen_eigensolver.h"
#include <string.h>

enum {
    coeff_xxx,
    coeff_xxy,
    coeff_xyy,
    coeff_yyy,
    coeff_xxz,
    coeff_xyz,
    coeff_yyz,
    coeff_xzz,
    coeff_yzz,
    coeff_zzz,
    coeff_xx,
    coeff_xy,
    coeff_yy,
    coeff_xz,
    coeff_yz,
    coeff_zz,
    coeff_x,
    coeff_y,
    coeff_z,
    coeff_1
};

static void deg_one_poly_product(const double *poly1,const double *poly2,double *product) {
    memset(product,0,20*sizeof(double));

    product[coeff_xx] = poly1[coeff_x] * poly2[coeff_x]; // x*x'
    product[coeff_xy]
        = poly1[coeff_x] * poly2[coeff_y] + poly1[coeff_y] * poly2[coeff_x];               // x*y' + y*x'
    product[coeff_xz] = poly1[coeff_x] * poly2[coeff_z] + poly1[coeff_z] * poly2[coeff_x]; // x*z' + z*x'
    product[coeff_yy] = poly1[coeff_y] * poly2[coeff_y];                                   // y * y'
    product[coeff_yz] = poly1[coeff_y] * poly2[coeff_z] + poly1[coeff_z] * poly2[coeff_y]; // y*z' + z * y'
    product[coeff_zz] = poly1[coeff_z] * poly2[coeff_z];                                   // z * z'
    product[coeff_x] = poly1[coeff_x] * poly2[coeff_1] + poly1[coeff_1] * poly2[coeff_x];  // x * c' + c * x'
    product[coeff_y] = poly1[coeff_y] * poly2[coeff_1] + poly1[coeff_1] * poly2[coeff_y];  // y * c' + c * y'
    product[coeff_z] = poly1[coeff_z] * poly2[coeff_1] + poly1[coeff_1] * poly2[coeff_z];  // z * c' + c * z'
    product[coeff_1] = poly1[coeff_1] * poly2[coeff_1];                                    // c * c'


}

static void deg_two_poly_product(const double *poly1,const double *poly2,double *product) {
    

    product[coeff_xxx] = poly1[coeff_xx] * poly2[coeff_x];
    product[coeff_xxy] = poly1[coeff_xx] * poly2[coeff_y]
                         + poly1[coeff_xy] * poly2[coeff_x];
    product[coeff_xxz] = poly1[coeff_xx] * poly2[coeff_z]
                         + poly1[coeff_xz] * poly2[coeff_x];
    product[coeff_xyy] = poly1[coeff_xy] * poly2[coeff_y]
                         + poly1[coeff_yy] * poly2[coeff_x];
    product[coeff_xyz] = poly1[coeff_xy] * poly2[coeff_z]
                         + poly1[coeff_yz] * poly2[coeff_x]
                         + poly1[coeff_xz] * poly2[coeff_y];
    product[coeff_xzz] = poly1[coeff_xz] * poly2[coeff_z]
                         + poly1[coeff_zz] * poly2[coeff_x];
    product[coeff_yyy] = poly1[coeff_yy] * poly2[coeff_y];
    product[coeff_yyz] = poly1[coeff_yy] * poly2[coeff_z]
                         + poly1[coeff_yz] * poly2[coeff_y];
    product[coeff_yzz] = poly1[coeff_yz] * poly2[coeff_z]
                         + poly1[coeff_zz] * poly2[coeff_y];
    product[coeff_zzz] = poly1[coeff_zz] * poly2[coeff_z];
    product[coeff_xx] = poly1[coeff_xx] * poly2[coeff_1]
                        + poly1[coeff_x] * poly2[coeff_x];
    product[coeff_xy] = poly1[coeff_xy] * poly2[coeff_1]
                        + poly1[coeff_x] * poly2[coeff_y]
                        + poly1[coeff_y] * poly2[coeff_x];
    product[coeff_xz] = poly1[coeff_xz] * poly2[coeff_1]
                        + poly1[coeff_x] * poly2[coeff_z]
                        + poly1[coeff_z] * poly2[coeff_x];
    product[coeff_yy] = poly1[coeff_yy] * poly2[coeff_1]
                        + poly1[coeff_y] * poly2[coeff_y];
    product[coeff_yz] = poly1[coeff_yz] * poly2[coeff_1]
                        + poly1[coeff_y] * poly2[coeff_z]
                        + poly1[coeff_z] * poly2[coeff_y];
    product[coeff_zz] = poly1[coeff_zz] * poly2[coeff_1]
                        + poly1[coeff_z] * poly2[coeff_z];
    product[coeff_x] = poly1[coeff_x] * poly2[coeff_1]
                       + poly1[coeff_1] * poly2[coeff_x];
    product[coeff_y] = poly1[coeff_y] * poly2[coeff_1]
                       + poly1[coeff_1] * poly2[coeff_y];
    product[coeff_z] = poly1[coeff_z] * poly2[coeff_1]
                       + poly1[coeff_1] * poly2[coeff_z];
    product[coeff_1] = poly1[coeff_1] * poly2[coeff_1];


}

void sv_essential_5pt_polynomial(const double basis[36],double out[200]){
 double E[3][3][20]={0},EET[3][3][20],p[3][20],q[20],a[20],b[20];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<4;k++)E[i][j][16+k]=basis[3*i+j+9*k];
 for(int k=0;k<3;k++){
  deg_one_poly_product(E[0][(k+1)%3],E[1][(k+2)%3],a);
  deg_one_poly_product(E[0][(k+2)%3],E[1][(k+1)%3],b);
  for(int v=0;v<20;v++)q[v]=a[v]-b[v];
  deg_two_poly_product(q,E[2][k],p[k]);
 }
 for(int v=0;v<20;v++)out[10*v]=(p[0][v]+p[1][v])+p[2][v];
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){
  if(i<=j){
   for(int k=0;k<3;k++)deg_one_poly_product(E[i][k],E[j][k],p[k]);
   for(int v=0;v<20;v++)EET[i][j][v]=(p[0][v]+p[1][v])+p[2][v];
  }else memcpy(EET[i][j],EET[j][i],sizeof(EET[i][j]));
 }
 for(int v=0;v<20;v++)q[v]=.5*((EET[0][0][v]+EET[1][1][v])+EET[2][2][v]);
 for(int i=0;i<3;i++)for(int v=0;v<20;v++)EET[i][i][v]-=q[v];
 int row=1;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){
  for(int k=0;k<3;k++)deg_two_poly_product(EET[i][k],E[k][j],p[k]);
  for(int v=0;v<20;v++)out[row+10*v]=(p[0][v]+p[1][v])+p[2][v];
  row++;
 }
}
int sv_essential_5pt(const double *b1,const double *b2,double out[90],sv_essential_5pt_trace *trace){
 if(!b1 || !b2 || !out)return -1;
 sv_essential_5pt_trace local;sv_essential_5pt_trace *t=trace?trace:&local;
 memset(t,0,sizeof(*t));
 for(int i=0;i<5;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)t->constraint[i+9*(3*j+k)]=b2[3*i+j]*b1[3*i+k];
 sv_eigen_fullpivlu lu;sv_eigen_fullpivlu_compute(t->constraint,9,&lu);t->rank=lu.rank;
 if(lu.rank!=5)return 0;
 sv_eigen_fullpivlu_kernel(&lu,t->basis);
 sv_essential_5pt_polynomial(t->basis,t->polynomial);
 sv_eigen_fullpivlu_compute(t->polynomial,10,&lu);
 sv_eigen_fullpivlu_solve(&lu,t->polynomial+100,10,t->eliminated);
 int rows[6]={0,1,2,4,5,7};
 for(int i=0;i<6;i++)for(int j=0;j<10;j++)t->action[i+10*j]=t->eliminated[rows[i]+10*j];
 t->action[6]=-1;t->action[7+10]=-1;t->action[8+30]=-1;t->action[9+60]=-1;
 if(sv_eigen_eigensolver10(t->action,t->eigen_real,t->eigen_imag,t->vectors_real,t->vectors_imag))return -1;
 int n=0;
 for(int s=0;s<10;s++)if(t->eigen_imag[s]==0){
  for(int i=0;i<9;i++){
   double p[4];for(int k=0;k<4;k++)p[k]=t->basis[i+9*k]*t->vectors_real[6+k+10*s];
   double sum=i==8 ? (p[0]+p[1])+(p[2]+p[3]) : ((p[0]+p[1])+p[2])+p[3];
   out[9*n+(i%3)*3+i/3]=sum;
  }n++;
 }
 t->count=n;return n;
}
