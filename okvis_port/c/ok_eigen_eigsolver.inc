/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 * Copyright (C) 2008-2009 Gael Guennebaud; 2010,2012 Jitse Niesen; 2009-2010 Benoit Jacob (Householder / Jacobi).
 * Size-templated body (included with EN = 8 or 10) of the C adaptation of Eigen 3.4 HessenbergDecomposition, RealSchur,
 * EigenSolver<Matrix<double,EN,EN>> (compute(m, true) + eigenvectors()) in the SSE2 evaluation order of the reference build
 * (-O2 -ffp-contract=off). Derived from stella_port/c/sv_eigen_eigensolver.c (same author, MPL-2.0) and generalised from 10 to EN. */
#define M(a,r,c) (a)[(r)+EN*(c)]
#define T(r,c) M(t,r,c)
#define U(r,c) M(u,r,c)
static int mini(int a,int b){return a<b?a:b;}
static double maxd(double a,double b){return a<b?b:a;}
/* Dynamic unaligned contiguous Redux, two 2-lane accumulators. */
static double redux(const double *v,int n){
    if(!n)return 0;
    int end=n/2*2,end4=n/4*4;
    if(end){
        double a=v[0],b=v[1];
        if(end>2){
            double c=v[2],d=v[3];
            for(int i=4;i<end4;i+=4){a+=v[i];b+=v[i+1];c+=v[i+2];d+=v[i+3];}
            a+=c;b+=d;
            if(end>end4){a+=v[end4];b+=v[end4+1];}
        }
        double sum=a+b;for(int i=end;i<n;i++)sum+=v[i];return sum;
    }
    return v[0];
}
static double house(double *v,int n,double *tau){
    double sq[10]={0};for(int i=1;i<n;i++)sq[i-1]=v[i]*v[i];
    double tail=redux(sq,n-1),a=v[0];
    if(tail<=DBL_MIN){*tau=0;for(int i=1;i<n;i++)v[i]=0;return a;}
    double beta=sqrt(a*a+tail);if(a>=0)beta=-beta;
    for(int i=1;i<n;i++)v[i]/=a-beta;
    *tau=(beta-a)/beta;return beta;
}
static void left(double *a,int r,int c,int nr,int nc,const double *e,double tau){
    if(nr==1){for(int j=c;j<c+nc;j++)M(a,r,j)*=1-tau;return;}
    if(tau==0)return;
    double w[10],scaled[10];int n=nr-1;
    for(int j=0;j<nc;j++){
        double p[10];for(int i=0;i<n;i++)p[i]=e[i]*M(a,r+1+i,c+j);
        if(nc==1)w[j]=redux(p,n);
        else{
            double l0=0,l1=0;int i=0;
            for(;i+1<n;i+=2){l0+=p[i];l1+=p[i+1];}
            w[j]=l0+l1;for(;i<n;i++)w[j]+=p[i];
        }
        w[j]+=M(a,r,c+j);
    }
    for(int j=0;j<nc;j++)M(a,r,c+j)-=tau*w[j];
    for(int i=0;i<n;i++)scaled[i]=tau*e[i];
    for(int j=0;j<nc;j++)for(int i=0;i<n;i++)M(a,r+1+i,c+j)-=scaled[i]*w[j];
}
static void right(double *a,int r,int c,int nr,int nc,const double *e,double tau){
    if(nc==1){for(int i=r;i<r+nr;i++)M(a,i,c)*=1-tau;return;}
    if(tau==0)return;
    double w[10];
    for(int i=0;i<nr;i++){
        double sum=0;for(int j=1;j<nc;j++)sum+=M(a,r+i,c+j)*e[j-1];
        w[i]=sum+M(a,r+i,c);
    }
    for(int i=0;i<nr;i++)M(a,r+i,c)-=tau*w[i];
    for(int j=1;j<nc;j++)for(int i=0;i<nr;i++)M(a,r+i,c+j)-=(tau*w[i])*e[j-1];
}
void EF(hessenberg)(const double a[EN*EN],double h[EN*EN],double q[EN*EN]){
    memcpy(h,a,sizeof(double)*EN*EN);double coeff[(EN-1)];
    for(int i=0;i<(EN-1);i++){
        double v[10];int n=(EN-1)-i;for(int k=0;k<n;k++)v[k]=M(h,i+1+k,i);
        double beta=house(v,n,&coeff[i]);M(h,i+1,i)=beta;
        for(int k=1;k<n;k++)M(h,i+1+k,i)=v[k];
        left(h,i+1,i+1,n,n,v+1,coeff[i]);
        right(h,0,i+1,EN,n,v+1,coeff[i]);
    }
    memset(q,0,sizeof(double)*EN*EN);for(int i=0;i<EN;i++)M(q,i,i)=1;
    for(int k=(EN-2);k>=0;k--)left(q,k+1,k+1,(EN-1)-k,(EN-1)-k,h+EN*k+k+2,coeff[k]);
    for(int j=0;j<EN;j++)for(int i=j+2;i<EN;i++)M(h,i,j)=0;
}
static void givens(double p,double q,double *c,double *s){
    if(q==0){*c=p<0?-1:1;*s=0;}
    else if(p==0){*c=0;*s=q<0?1:-1;}
    else if(fabs(p)>fabs(q)){double t=q/p,u=sqrt(1+t*t);if(p<0)u=-u;*c=1/u;*s=-t*(*c);}
    else{double t=p/q,u=sqrt(1+t*t);if(q<0)u=-u;*s=-1/u;*c=-t*(*s);}
}
static void split(double *t,double *u,int iu,double shift){
    double p=.5*(T(iu-1,iu-1)-T(iu,iu));
    double q=p*p+T(iu,iu-1)*T(iu-1,iu);
    T(iu,iu)+=shift;T(iu-1,iu-1)+=shift;
    if(q>=0){
        double z=sqrt(fabs(q)),c,s;givens(p>=0?p+z:p-z,T(iu,iu-1),&c,&s);
        for(int j=iu-1;j<EN;j++){double x=T(iu-1,j),y=T(iu,j);T(iu-1,j)=c*x-s*y;T(iu,j)=s*x+c*y;}
        for(int i=0;i<=iu;i++){double x=T(i,iu-1),y=T(i,iu);T(i,iu-1)=c*x-s*y;T(i,iu)=s*x+c*y;}
        T(iu,iu-1)=0;
        for(int i=0;i<EN;i++){double x=U(i,iu-1),y=U(i,iu);U(i,iu-1)=c*x-s*y;U(i,iu)=s*x+c*y;}
    }
    if(iu>1)T(iu-1,iu-2)=0;
}
static void francis(double *t,double *u,int il,int im,int iu,const double first[3]){
    for(int k=im;k<=iu-2;k++){
        double v[3],tau;
        for(int i=0;i<3;i++)v[i]=k==im?first[i]:T(k+i,k-1);
        double beta=house(v,3,&tau);
        if(beta!=0){
            if(k==im && k>il)T(k,k-1)=-T(k,k-1);
            else if(k!=im)T(k,k-1)=beta;
            left(t,k,k,3,EN-k,v+1,tau);
            right(t,0,k,mini(iu,k+3)+1,3,v+1,tau);
            right(u,0,k,EN,3,v+1,tau);
        }
    }
    double v[2]={T(iu-1,iu-2),T(iu,iu-2)},tau,beta;
    beta=house(v,2,&tau);
    if(beta!=0){
        T(iu-1,iu-2)=beta;
        left(t,iu-1,iu-1,2,EN+1-iu,v+1,tau);
        right(t,0,iu-1,iu+1,2,v+1,tau);
        right(u,0,iu-1,EN,2,v+1,tau);
    }
    for(int i=im+2;i<=iu;i++){T(i,i-2)=0;if(i>im+2)T(i,i-3)=0;}
}
int EF(realschur)(const double a[EN*EN],double t[EN*EN],double u[EN*EN]){
    double scale=0;for(int i=0;i<EN*EN;i++)scale=maxd(scale,fabs(a[i]));
    if(scale<DBL_MIN){memset(t,0,sizeof(double)*EN*EN);memset(u,0,sizeof(double)*EN*EN);for(int i=0;i<EN;i++)U(i,i)=1;return 0;}
    double norm=0,b[EN*EN];for(int i=0;i<EN*EN;i++)b[i]=a[i]/scale;
    EF(hessenberg)(b,t,u);
    for(int j=0;j<EN;j++){double v[10];int n=mini(EN,j+2);for(int i=0;i<n;i++)v[i]=fabs(T(i,j));norm+=redux(v,n);}
    double small=maxd(norm*(DBL_EPSILON*DBL_EPSILON),DBL_MIN),exshift=0;
    int iu=(EN-1),iter=0,total=0;
    if(norm!=0)while(iu>=0){
        int il=iu;
        while(il>0){double s=fabs(T(il-1,il-1))+fabs(T(il,il));s=maxd(s*DBL_EPSILON,small);if(fabs(T(il,il-1))<=s)break;il--;}
        if(il==iu){T(iu,iu)+=exshift;if(iu>0)T(iu,iu-1)=0;iu--;iter=0;}
        else if(il==iu-1){split(t,u,iu,exshift);iu-=2;iter=0;}
        else{
            double sh[3]={T(iu,iu),T(iu-1,iu-1),T(iu,iu-1)*T(iu-1,iu)};
            if(iter==10){exshift+=sh[0];for(int i=0;i<=iu;i++)T(i,i)-=sh[0];double s=fabs(T(iu,iu-1))+fabs(T(iu-1,iu-2));sh[0]=sh[1]=.75*s;sh[2]=-.4375*s*s;}
            if(iter==30){double s=(sh[1]-sh[0])/2.;s=s*s+sh[2];if(s>0){s=sqrt(s);if(sh[1]<sh[0])s=-s;s=s+(sh[1]-sh[0])/2.;s=sh[0]-sh[2]/s;exshift+=s;for(int i=0;i<=iu;i++)T(i,i)-=s;sh[0]=sh[1]=sh[2]=.964;}}
            iter++;if(++total>40*EN)return -1;
            double v[3];int im;
            for(im=iu-2;im>=il;im--){
                double d=T(im,im),r=sh[0]-d,s=sh[1]-d;
                v[0]=(r*s-sh[2])/T(im+1,im)+T(im,im+1);v[1]=T(im+1,im+1)-d-r-s;v[2]=T(im+2,im+1);
                if(im==il)break;
                double lhs=T(im,im-1)*(fabs(v[1])+fabs(v[2]));
                double rhs=v[0]*(fabs(T(im-1,im-1))+fabs(d)+fabs(T(im+1,im+1)));
                if(fabs(lhs)<DBL_EPSILON*rhs)break;
            }
            francis(t,u,il,im,iu,v);
        }
    }
    for(int i=0;i<EN*EN;i++)t[i]*=scale;
    return 0;
}
/* Strided row/contiguous column dot has no packet access. */
static double rowdot(const double *t,int row,int col,int start,int len){
    double sum=M(t,row,start)*M(t,start,col);
    for(int i=1;i<len;i++)sum+=M(t,row,start+i)*M(t,start+i,col);
    return sum;
}
static double balanced(const double *a,int n){
    if(n==1)return a[0];
    int half=n/2;return balanced(a,half)+balanced(a+half,n-half);
}
/* std::complex<double> operator/ (libgcc __divdc3 as of GCC 12+, Smith's method with the Baudin-Smith range guards):
 * written from the published algorithm, see ok_cdiv in ok_eigen.c. */
static void cdiv_pair_out(double ar,double ai,double br,double bi,double *re,double *im){ok_cdiv(ar,ai,br,bi,re,im);}
/* Eigen normalizes complex columns with SSE2 complex/complex division,
 * even when the divisor is real. Preserve multiplication before division. */
static void normalize_coeff(double *r,double *i,double n){
    double a=*r,b=*i,denom=n*n+0.0;
    *r=(a*n+(-(b*(-0.0))))/denom;
    *i=(a*(-0.0)+b*n)/denom;
}
int EF(eigensolver)(const double a[EN*EN],double er[EN],double ei[EN],double vr[EN*EN],double vi[EN*EN]){
    double t[EN*EN],u[EN*EN];
    if(EF(realschur)(a,t,u))return -1;
    for(int i=0;i<EN;){
        if(i==(EN-1) || T(i+1,i)==0){er[i]=T(i,i);ei[i]=0;if(!isfinite(er[i]))return -1;i++;}
        else{
            double p=.5*(T(i,i)-T(i+1,i+1)),t0=T(i+1,i),t1=T(i,i+1);
            double maxval=maxd(fabs(p),maxd(fabs(t0),fabs(t1)));
            t0/=maxval;t1/=maxval;double p0=p/maxval;
            double z=maxval*sqrt(fabs(p0*p0+t0*t1));
            er[i]=er[i+1]=T(i+1,i+1)+p;ei[i]=z;ei[i+1]=-z;
            if(!isfinite(er[i]) || !isfinite(z))return -1;
            i+=2;
        }
    }
    double norm=0;
    for(int j=0;j<EN;j++){
        int start=j>0?j-1:0;double sum=fabs(T(j,start));
        for(int i=start+1;i<EN;i++)sum+=fabs(T(j,i));
        norm+=sum;
    }
    if(norm!=0){
        for(int n=(EN-1);n>=0;n--){
            double p=er[n],q=ei[n];
            if(q==0){
                double lastr=0,lastw=0;int l=n;T(n,n)=1;
                for(int i=n-1;i>=0;i--){
                    double w=T(i,i)-p,r=rowdot(t,i,n,l,n-l+1);
                    if(ei[i]<0){lastw=w;lastr=r;}
                    else{
                        l=i;
                        if(ei[i]==0)T(i,n)=w!=0?-r/w:-r/(DBL_EPSILON*norm);
                        else{
                            double x=T(i,i+1),y=T(i+1,i);
                            double denom=(er[i]-p)*(er[i]-p)+ei[i]*ei[i];
                            double value=(x*lastr-lastw*r)/denom;T(i,n)=value;
                            T(i+1,n)=fabs(x)>fabs(lastw)?(-r-w*value)/x:(-lastr-y*value)/lastw;
                        }
                        double value=fabs(T(i,n));
                        if((DBL_EPSILON*value)*value>1)for(int k=i;k<EN;k++)T(k,n)/=value;
                    }
                }
            }
            else if(q<0 && n>0){
                double lastra=0,lastsa=0,lastw=0;int l=n-1;
                if(fabs(T(n,n-1))>fabs(T(n-1,n))){T(n-1,n-1)=q/T(n,n-1);T(n-1,n)=-(T(n,n)-p)/T(n,n-1);}
                else{
                    double cr,ci;cdiv_pair_out(0,-T(n-1,n),T(n-1,n-1)-p,q,&cr,&ci);
                    T(n-1,n-1)=cr;T(n-1,n)=ci;
                }
                T(n,n-1)=0;T(n,n)=1;
                for(int i=n-2;i>=0;i--){
                    double ra=rowdot(t,i,n-1,l,n-l+1),sa=rowdot(t,i,n,l,n-l+1),w=T(i,i)-p;
                    if(ei[i]<0){lastw=w;lastra=ra;lastsa=sa;}
                    else{
                        l=i;
                        if(ei[i]==0){
                            double cr,ci;cdiv_pair_out(-ra,-sa,w,q,&cr,&ci);T(i,n-1)=cr;T(i,n)=ci;
                        }
                        else{
                            double x=T(i,i+1),y=T(i+1,i);
                            double rr=(er[i]-p)*(er[i]-p)+ei[i]*ei[i]-q*q,ii=(er[i]-p)*2*q;
                            if(rr==0 && ii==0)rr=DBL_EPSILON*norm*(fabs(w)+fabs(q)+fabs(x)+fabs(y)+fabs(lastw));
                            double cr,ci;cdiv_pair_out(x*lastra-lastw*ra+q*sa,x*lastsa-lastw*sa-q*ra,rr,ii,&cr,&ci);
                            T(i,n-1)=cr;T(i,n)=ci;
                            if(fabs(x)>fabs(lastw)+fabs(q)){
                                T(i+1,n-1)=(-ra-w*T(i,n-1)+q*T(i,n))/x;
                                T(i+1,n)=(-sa-w*T(i,n)-q*T(i,n-1))/x;
                            }
                            else{
                                cdiv_pair_out(-lastra-y*T(i,n-1),-lastsa-y*T(i,n),lastw,q,&cr,&ci);
                                T(i+1,n-1)=cr;T(i+1,n)=ci;
                            }
                        }
                        double value=maxd(fabs(T(i,n-1)),fabs(T(i,n)));
                        if((DBL_EPSILON*value)*value>1)for(int k=i;k<EN;k++){T(k,n-1)/=value;T(k,n)/=value;}
                    }
                }
                n--;
            }
            else return -1;
        }
        for(int j=(EN-1);j>=0;j--){
            double tmp[10];
            for(int i=0;i<EN;i++){double sum=0;for(int k=0;k<=j;k++)sum+=U(i,k)*T(k,j);tmp[i]=sum;}
            for(int i=0;i<EN;i++)U(i,j)=tmp[i];
        }
    }
    for(int j=0;j<EN;j++){
        double squares[10];
        if(fabs(ei[j])<=fabs(er[j])*(2*DBL_EPSILON) || j==(EN-1)){
            for(int i=0;i<EN;i++){M(vr,i,j)=U(i,j);M(vi,i,j)=0;squares[i]=U(i,j)*U(i,j);}
            double n=sqrt(balanced(squares,EN));
            if(n>0)for(int i=0;i<EN;i++){normalize_coeff(&M(vr,i,j),&M(vi,i,j),n);}
        }
        else{
            for(int i=0;i<EN;i++){M(vr,i,j)=M(vr,i,j+1)=U(i,j);M(vi,i,j)=U(i,j+1);M(vi,i,j+1)=-U(i,j+1);squares[i]=U(i,j)*U(i,j)+U(i,j+1)*U(i,j+1);}
            double n=sqrt(balanced(squares,EN));
            if(n>0)for(int i=0;i<EN;i++){normalize_coeff(&M(vr,i,j),&M(vi,i,j),n);normalize_coeff(&M(vr,i,j+1),&M(vi,i,j+1),n);}
            j++;
        }
    }
    return 0;
}

