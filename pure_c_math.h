#ifndef PURE_C_MATH_H
#define PURE_C_MATH_H

#include <math.h>
#include <string.h>
#include <stdlib.h>

static void jacobi_nxn(const double* A, int n, double* W, double* V) {
    double* M = malloc(n * n * sizeof(double));
    memcpy(M, A, n * n * sizeof(double));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) V[i * n + j] = (i == j) ? 1.0 : 0.0;
    }
    for (int iter = 0; iter < 100; iter++) {
        double max_offdiag = 0.0; int p = 0, q = 1;
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                double val = fabs(M[i * n + j]);
                if (val > max_offdiag) { max_offdiag = val; p = i; q = j; }
            }
        }
        if (max_offdiag < 1e-15) break;
        double app = M[p * n + p], aqq = M[q * n + q], apq = M[p * n + q];
        double phi = 0.5 * atan2(2.0 * apq, aqq - app);
        double c = cos(phi), s = sin(phi);
        for (int i = 0; i < n; i++) {
            double mip = M[i * n + p], miq = M[i * n + q];
            M[i * n + p] = c * mip - s * miq; M[i * n + q] = s * mip + c * miq;
        }
        for (int i = 0; i < n; i++) {
            double mpi = M[p * n + i], mqi = M[q * n + i];
            M[p * n + i] = c * mpi - s * mqi; M[q * n + i] = s * mpi + c * mqi;
        }
        for (int i = 0; i < n; i++) {
            double vip = V[i * n + p], viq = V[i * n + q];
            V[i * n + p] = c * vip - s * viq; V[i * n + q] = s * vip + c * viq;
        }
    }
    for (int i = 0; i < n; i++) W[i] = M[i * n + i];
    free(M);
}

static void svd_3x3(const double A[9], double W[3], double U[9], double V[9]) {
    double AtA[9] = {0};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++) AtA[i*3+j]+=A[k*3+i]*A[k*3+j];
    jacobi_nxn(AtA, 3, W, V);
    for(int i=0;i<3;i++) W[i]=sqrt(fabs(W[i]));
    for(int i=0;i<3;i++)for(int j=0;j<3;j++){
        double s=0; for(int k=0;k<3;k++) s+=A[i*3+k]*V[k*3+j];
        U[i*3+j]=s/(W[j]+1e-12);
    }
}

static void project_to_SO3(const double Rin[9], double Rout[9]) { double W[3],U[9],V[9],VT[9]; svd_3x3(Rin,W,U,V); for(int i=0;i<3;i++)for(int j=0;j<3;j++)VT[i*3+j]=V[j*3+i]; double M[9]; for(int r=0;r<3;r++)for(int c=0;c<3;c++){double s=0; for(int k=0;k<3;k++) s+=U[r*3+k]*VT[k*3+c]; M[r*3+c]=s;} double d=M[0]*(M[4]*M[8]-M[5]*M[7])-M[1]*(M[3]*M[8]-M[5]*M[6])+M[2]*(M[3]*M[7]-M[4]*M[6]); if(d<0) for(int i=0;i<9;i++) M[i]=-M[i]; memcpy(Rout,M,9*sizeof(double)); }

static void pnp_unpack(const double P[12], double R[9], double t[3]) { R[0]=P[0];R[1]=P[1];R[2]=P[2];R[3]=P[4];R[4]=P[5];R[5]=P[6];R[6]=P[8];R[7]=P[9];R[8]=P[10]; t[0]=P[3];t[1]=P[7];t[2]=P[11]; double den=R[6]*R[6]+R[7]*R[7]+R[8]*R[8]; if(den<1e-12) return; double sc=1.0/sqrt(den); if(P[11]*sc<0) sc=-sc; for(int j=0;j<9;j++) R[j]*=sc; for(int j=0;j<3;j++) t[j]*=sc; }

static int solve_6x6(double A[36], double b[6], double x[6]) {
    for (int i = 0; i < 6; i++) {
        int pivot = i;
        for (int j = i + 1; j < 6; j++) if (fabs(A[j*6+i]) > fabs(A[pivot*6+i])) pivot = j;
        for (int j = i; j < 6; j++) { double t = A[i*6+j]; A[i*6+j] = A[pivot*6+j]; A[pivot*6+j] = t; }
        double t = b[i]; b[i] = b[pivot]; b[pivot] = t;
        if (fabs(A[i*6+i]) < 1e-18) return 0;
        for (int j = i + 1; j < 6; j++) {
            double f = A[j*6+i] / A[i*6+i];
            for (int k = i + 1; k < 6; k++) A[j*6+k] -= f * A[i*6+k];
            b[j] -= f * b[i];
        }
    }
    for (int i = 5; i >= 0; i--) {
        double s = b[i];
        for (int j = i + 1; j < 6; j++) s -= A[i*6+j] * x[j];
        x[i] = s / A[i*6+i];
    }
    return 1;
}

#endif
