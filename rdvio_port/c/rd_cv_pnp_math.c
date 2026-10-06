/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * Modified C99 adaptation of OpenCV 4.6 core/lapack.cpp and calibration.cpp.
 * Full retained notices in ../reference_cv/LICENSE-M7b-OpenCV. */
#include "rd_cv_pnp_math.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
static uint32_t next_rng(uint64_t *s) {*s=(uint64_t)(uint32_t)*s*4164903690U+(*s>>32);return (uint32_t)*s;}
static double cvhypot(double a,double b) {
    a=fabs(a);b=fabs(b);
    if(a>b) {b/=a;return a*sqrt(1+b*b);}
    if(b>0) {a/=b;return b*sqrt(1+a*a);}
    return 0;
}
static void jacobi(double *At,int m,int n,double *_W,double *Vt) {
    double W[12];
    int n1=n;
    double minval=DBL_MIN, eps=DBL_EPSILON*10;
    int astep=m, vstep=n;
    int i, j, k, iter, max_iter = (m>30?m:30);
    double c, s;
    double sd;


    for( i = 0; i < n; i++ )
    {
        for( k = 0, sd = 0; k < m; k++ )
        {
            double t = At[i*astep + k];
            sd += (double)t*t;
        }
        W[i] = sd;

        if( Vt )
        {
            for( k = 0; k < n; k++ )
                Vt[i*vstep + k] = 0;
            Vt[i*vstep + i] = 1;
        }
    }

    for( iter = 0; iter < max_iter; iter++ )
    {
        int changed = 0;

        for( i = 0; i < n-1; i++ )
            for( j = i+1; j < n; j++ )
            {
                double *Ai = At + i*astep, *Aj = At + j*astep;
                double a = W[i], p = 0, b = W[j];

                for( k = 0; k < m; k++ )
                    p += (double)Ai[k]*Aj[k];

                if( fabs(p) <= eps*sqrt((double)a*b) )
                    continue;

                p *= 2;
                double beta = a - b, gamma = cvhypot((double)p, beta);
                if( beta < 0 )
                {
                    double delta = (gamma - beta)*0.5;
                    s = (double)sqrt(delta/gamma);
                    c = (double)(p/(gamma*s*2));
                }
                else
                {
                    c = (double)sqrt((gamma + beta)/(gamma*2));
                    s = (double)(p/(gamma*c*2));
                }

                a = b = 0;
                for( k = 0; k < m; k++ )
                {
                    double t0 = c*Ai[k] + s*Aj[k];
                    double t1 = -s*Ai[k] + c*Aj[k];
                    Ai[k] = t0; Aj[k] = t1;

                    a += (double)t0*t0; b += (double)t1*t1;
                }
                W[i] = a; W[j] = b;

                changed = 1;

                if( Vt )
                {
                    double *Vi = Vt + i*vstep, *Vj = Vt + j*vstep;
                    /* SSE double givens: subtract in vector lanes; scalar remainder
                     * writes -s*Vi + c*Vj. Preserve signed-zero behavior. */
                    for(k=0;k<n/2*2;k++) {
                        double t0=c*Vi[k]+s*Vj[k];
                        double t1=c*Vj[k]-s*Vi[k];
                        Vi[k]=t0;Vj[k]=t1;
                    }

                    for( ; k < n; k++ )
                    {
                        double t0 = c*Vi[k] + s*Vj[k];
                        double t1 = -s*Vi[k] + c*Vj[k];
                        Vi[k] = t0; Vj[k] = t1;
                    }
                }
            }
        if( !changed )
            break;
    }

    for( i = 0; i < n; i++ )
    {
        for( k = 0, sd = 0; k < m; k++ )
        {
            double t = At[i*astep + k];
            sd += (double)t*t;
        }
        W[i] = sqrt(sd);
    }

    for( i = 0; i < n-1; i++ )
    {
        j = i;
        for( k = i+1; k < n; k++ )
        {
            if( W[j] < W[k] )
                j = k;
        }
        if( i != j )
        {
            {double tmp=W[i];W[i]=W[j];W[j]=tmp;}
            if( Vt )
            {
                for( k = 0; k < m; k++ )
                    {double tmp=At[i*astep + k];At[i*astep + k]=At[j*astep + k];At[j*astep + k]=tmp;}

                for( k = 0; k < n; k++ )
                    {double tmp=Vt[i*vstep + k];Vt[i*vstep + k]=Vt[j*vstep + k];Vt[j*vstep + k]=tmp;}
            }
        }
    }

    for( i = 0; i < n; i++ )
        _W[i] = (double)W[i];

    if( !Vt )
        return;

    uint64_t rng=0x12345678;
    for( i = 0; i < n1; i++ )
    {
        sd = i < n ? W[i] : 0;

        for( int ii = 0; ii < 100 && sd <= minval; ii++ )
        {
            // if we got a zero singular value, then in order to get the corresponding left singular vector
            // we generate a random vector, project it to the previously computed left singular vectors,
            // subtract the projection and normalize the difference.
            const double val0 = (double)(1./m);
            for( k = 0; k < m; k++ )
            {
                double val = (next_rng(&rng) & 256) != 0 ? val0 : -val0;
                At[i*astep + k] = val;
            }
            for( iter = 0; iter < 2; iter++ )
            {
                for( j = 0; j < i; j++ )
                {
                    sd = 0;
                    for( k = 0; k < m; k++ )
                        sd += At[i*astep + k]*At[j*astep + k];
                    double asum = 0;
                    for( k = 0; k < m; k++ )
                    {
                        double t = (double)(At[i*astep + k] - sd*At[j*astep + k]);
                        At[i*astep + k] = t;
                        asum += fabs(t);
                    }
                    asum = asum > eps*100 ? 1/asum : 0;
                    for( k = 0; k < m; k++ )
                        At[i*astep + k] *= asum;
                }
            }
            sd = 0;
            for( k = 0; k < m; k++ )
            {
                double t = At[i*astep + k];
                sd += (double)t*t;
            }
            sd = sqrt(sd);
        }

        s = (double)(sd > minval ? 1/sd : 0.);
        for( k = 0; k < m; k++ )
            At[i*astep + k] *= s;
    }
}
void rd_cv_pnp_svd(const double *a,int m,int n,double *w,double *ut,double *vt) {
    for(int i=0;i<n;i++)for(int j=0;j<m;j++)ut[i*m+j]=a[j*n+i];
    jacobi(ut,m,n,w,vt);
}
void rd_cv_pnp_mtm(const double *a,int m,int n,double *ata) {
    for(int i=0;i<n;i++)for(int j=i;j<n;j++) {
        double s=0;
        for(int k=0;k<m;k++)s+=a[k*n+i]*a[k*n+j];
        ata[i*n+j]=ata[j*n+i]=s;
    }
}
void rd_cv_pnp_solve(const double *a,int m,int n,const double *b,double *x) {
    double w[12],ut[144],vt[144],threshold=0;
    rd_cv_pnp_svd(a,m,n,w,ut,vt);
    for(int i=0;i<n;i++) {x[i]=0;threshold+=w[i];}
    threshold*=DBL_EPSILON*2;
    for(int i=0;i<n;i++) {
        if(fabs(w[i])<=threshold)continue;
        double wi=1/w[i],s=0;
        for(int j=0;j<m;j++)s+=ut[i*m+j]*b[j];
        s*=wi;
        for(int j=0;j<n;j++)x[j]+=s*vt[i*n+j];
    }
}
void rd_cv_pnp_inverse(const double *a,int n,double *inv) {
    double w[12],ut[144],vt[144],threshold=0;
    rd_cv_pnp_svd(a,n,n,w,ut,vt);
    memset(inv,0,(size_t)n*n*sizeof(double));
    for(int i=0;i<n;i++)threshold+=w[i];
    threshold*=DBL_EPSILON*2;
    for(int i=0;i<n;i++) {
        if(fabs(w[i])<=threshold)continue;
        double wi=1/w[i],buf[12];
        for(int j=0;j<n;j++)buf[j]=ut[i*n+j]*wi;
        for(int r=0;r<n;r++)for(int c=0;c<n;c++)inv[r*n+c]+=vt[i*n+r]*buf[c];
    }
}
void rd_cv_pnp_rodrigues_vector(const double R[9],double r[3]) {
    double w[3],ut[9],vt[9],M[9];
    for(int i=0;i<9;i++)if(!(R[i]>=-100&&R[i]<100)) {memset(r,0,3*sizeof(double));return;}
    rd_cv_pnp_svd(R,3,3,w,ut,vt);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++) {
        double s=0;for(int k=0;k<3;k++)s+=ut[k*3+i]*vt[k*3+j];M[i*3+j]=s;
    }
    r[0]=M[7]-M[5];r[1]=M[2]-M[6];r[2]=M[3]-M[1];
    double s=sqrt((r[0]*r[0]+r[1]*r[1]+r[2]*r[2])*.25);
    double c=(M[0]+M[4]+M[8]-1)*.5;
    if(c>1)c=1;
    if(c< -1)c=-1;
    double theta=acos(c);
    if(s<1e-5) {
        if(c>0) {r[0]=r[1]=r[2]=0;return;}
        for(int i=0;i<3;i++) {double t=(M[i*3+i]+1)*.5;r[i]=sqrt(t>0?t:0);}
        r[1]*=M[1]<0?-1:1;r[2]*=M[2]<0?-1:1;
        if(fabs(r[0])<fabs(r[1])&&fabs(r[0])<fabs(r[2])&&(M[5]>0)!=(r[1]*r[2]>0))r[2]=-r[2];
        theta/=sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
        for(int i=0;i<3;i++)r[i]*=theta;
    } else {
        double vth=1/(2*s);vth*=theta;for(int i=0;i<3;i++)r[i]*=vth;
    }
}
void rd_cv_pnp_rodrigues_float(const float rv[3],float R[9]) {
    double r[3]={rv[0],rv[1],rv[2]};
    double theta=sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
    if(theta<DBL_EPSILON) {for(int i=0;i<9;i++)R[i]=(i%4==0);return;}
    double c=cos(theta),s=sin(theta),c1=1-c,inv=theta?1/theta:0;
    for(int i=0;i<3;i++)r[i]*=inv;
    double skew[9]={0,-r[2],r[1],r[2],0,-r[0],-r[1],r[0],0};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)R[i*3+j]=(float)((c*(i==j)+c1*(r[i]*r[j]))+s*skew[i*3+j]);
}
