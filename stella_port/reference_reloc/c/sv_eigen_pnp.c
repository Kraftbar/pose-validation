/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * See sv_eigen_svd.h for the Eigen sources this follows.
 */
#include "sv_eigen_pnp.h"
#include "../../c/sv_eigen_qr.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SV_EPS 2.2204460492503131e-16
#define SV_MIN DBL_MIN

static inline double *cp(double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }
static inline const double *ccp(const double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }

/* JacobiRotation<double>::makeJacobi(x,y,z) -- Jacobi.h line 92. */
static void make_jacobi(double x, double y, double z, double *c, double *s) {
    double deno = 2.0 * fabs(y);
    if (deno < SV_MIN) {
        *c = 1.0;
        *s = 0.0;
        return;
    }
    double tau = (x - z) / deno;
    double w = sqrt(tau * tau + 1.0);
    double t = (tau > 0.0) ? 1.0 / (tau + w) : 1.0 / (tau - w);
    double sign_t = (t > 0.0) ? 1.0 : -1.0;
    double n = 1.0 / sqrt(t * t + 1.0);
    *s = -sign_t * (y / fabs(y)) * fabs(t) * n;
    *c = n;
}

/* internal::real_2x2_jacobi_svd -- misc/RealSvd2x2.h. Real scalar case. */
static void real_2x2_jacobi_svd(double m00, double m01, double m10, double m11,
                                 double *lc, double *ls, double *rc, double *rs) {
    double t = m00 + m11;
    double d = m10 - m01;
    double rot1_c, rot1_s;
    if (fabs(d) < SV_MIN) {
        rot1_s = 0.0;
        rot1_c = 1.0;
    } else {
        double u = t / d;
        double tmp = sqrt(1.0 + u * u);
        rot1_s = 1.0 / tmp;
        rot1_c = u / tmp;
    }
    /* m.applyOnTheLeft(0,1,rot1) */
    double n00 = rot1_c * m00 + rot1_s * m10;
    double n01 = rot1_c * m01 + rot1_s * m11;
    double n11 = -rot1_s * m01 + rot1_c * m11;
    double rc_, rs_;
    make_jacobi(n00, n01, n11, &rc_, &rs_);
    /* j_left = rot1 * j_right.transpose() ; transpose() = (c, -s) */
    *lc = rot1_c * rc_ + rot1_s * rs_;
    *ls = rot1_s * rc_ - rot1_c * rs_;
    *rc = rc_;
    *rs = rs_;
}

/* MatrixBase::applyOnTheLeft(p,q,j): rotates rows p,q, looping the `ncols`
 * column entries of those rows (stride `ld` between successive entries). */
static void apply_rows(double *m, int ld, int ncols, int p, int q, double c, double s) {
    double *rp = m + p;
    double *rq = m + q;
    for (int i = 0; i < ncols; ++i) {
        double xi = rp[(size_t)i * ld];
        double yi = rq[(size_t)i * ld];
        rp[(size_t)i * ld] = c * xi + s * yi;
        rq[(size_t)i * ld] = -s * xi + c * yi;
    }
}

/* MatrixBase::applyOnTheRight(p,q,j): rotates columns p,q (each `nrows`
 * contiguous entries), using j.transpose() internally -- see Jacobi.h. */
static void apply_cols(double *m, int ld, int nrows, int p, int q, double c, double s) {
    double *cpp = cp(m, ld, p);
    double *cq = cp(m, ld, q);
    for (int i = 0; i < nrows; ++i) {
        double xi = cpp[i];
        double yi = cq[i];
        cpp[i] = c * xi - s * yi;
        cq[i] = s * xi + c * yi;
    }
}

/* Shared engine for JacobiSVD::compute() steps 2-4 (main sweep, sign fix,
 * sort) once workMatrix/U/V have been initialized by the (square or
 * QR-preconditioned) step 1. `work` is n x n (ld n). `U`, if non-NULL, is
 * Urows x n (ld Urows, Urows==n for our full-U square/QR-preconditioned
 * uses). `V`, if non-NULL, is Vrows x (>=n) (ld Vrows; only its first n
 * columns are touched, matching diagSize < cols for the N==8 shape). */
static void jacobi_svd_core(double *work, int n,
                             double *U, int Urows,
                             double *V, int Vrows,
                             double *sv, double scale, int *nonzero_out) {
    double maxDiagEntry = 0.0;
    for (int i = 0; i < n; ++i) {
        double a = fabs(work[i + (size_t)i * n]);
        if (a > maxDiagEntry) maxDiagEntry = a;
    }
    const double precision = 2.0 * SV_EPS;

    int finished = 0;
    while (!finished) {
        finished = 1;
        for (int p = 1; p < n; ++p) {
            for (int q = 0; q < p; ++q) {
                double threshold = precision * maxDiagEntry;
                if (threshold < SV_MIN) threshold = SV_MIN;
                double wpq = work[p + (size_t)q * n];
                double wqp = work[q + (size_t)p * n];
                if (fabs(wpq) > threshold || fabs(wqp) > threshold) {
                    finished = 0;
                    double mpp = work[p + (size_t)p * n];
                    double mqq = work[q + (size_t)q * n];
                    double lc, ls, rc, rs;
                    real_2x2_jacobi_svd(mpp, wpq, wqp, mqq, &lc, &ls, &rc, &rs);

                    apply_rows(work, n, n, p, q, lc, ls);
                    /* U.applyOnTheRight(p,q, j_left.transpose()): the
                     * transpose passed in cancels the one apply_cols()
                     * already bakes in for applyOnTheRight's own
                     * convention, leaving the *direct* (lc,ls) formula on
                     * U's columns -- i.e. pass (lc,-ls) here, not (lc,ls),
                     * so apply_cols's internal (c,-s) becomes (lc,ls). */
                    if (U) apply_cols(U, Urows, Urows, p, q, lc, -ls);

                    apply_cols(work, n, n, p, q, rc, rs);
                    if (V) apply_cols(V, Vrows, Vrows, p, q, rc, rs);

                    double a1 = fabs(work[p + (size_t)p * n]);
                    double a2 = fabs(work[q + (size_t)q * n]);
                    if (a1 > maxDiagEntry) maxDiagEntry = a1;
                    if (a2 > maxDiagEntry) maxDiagEntry = a2;
                }
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        double a = work[i + (size_t)i * n];
        sv[i] = fabs(a);
        if (U && a < 0.0) {
            double *ci = cp(U, Urows, i);
            for (int r = 0; r < Urows; ++r) ci[r] = -ci[r];
        }
    }
    for (int i = 0; i < n; ++i) sv[i] *= scale;

    int nonzero = n;
    for (int i = 0; i < n; ++i) {
        int pos = i;
        double best = sv[i];
        for (int j = i + 1; j < n; ++j) {
            if (sv[j] > best) { best = sv[j]; pos = j; }
        }
        if (best == 0.0) { nonzero = i; break; }
        if (pos != i) {
            double t = sv[i]; sv[i] = sv[pos]; sv[pos] = t;
            if (U) {
                double *ci = cp(U, Urows, i), *cpos = cp(U, Urows, pos);
                for (int r = 0; r < Urows; ++r) { double tt = ci[r]; ci[r] = cpos[r]; cpos[r] = tt; }
            }
            if (V) {
                double *ci = cp(V, Vrows, i), *cpos = cp(V, Vrows, pos);
                for (int r = 0; r < Vrows; ++r) { double tt = ci[r]; ci[r] = cpos[r]; cpos[r] = tt; }
            }
        }
    }
    if (nonzero_out) *nonzero_out = nonzero;
}


static double *col_ptr(double *b,int n,int c){return b+n*c;}
static const double *ccol_ptr(const double *b,int n,int c){return b+n*c;}
static double redux_sumsq(const double *v, long n) {
    if (n <= 0) return 0.0;
    long alignedStart = 0;
    if (alignedStart > n) alignedStart = n;
    const long packetSize = 2;
    long alignedSize = ((n - alignedStart) / packetSize) * packetSize;
    long alignedSize2 = ((n - alignedStart) / (2 * packetSize)) * (2 * packetSize);
    long alignedEnd2 = alignedStart + alignedSize2;
    long alignedEnd = alignedStart + alignedSize;
    double res;
    if (alignedSize) {
        double p0_0 = v[alignedStart] * v[alignedStart];
        double p0_1 = v[alignedStart + 1] * v[alignedStart + 1];
        if (alignedSize > packetSize) {
            double p1_0 = v[alignedStart + 2] * v[alignedStart + 2];
            double p1_1 = v[alignedStart + 3] * v[alignedStart + 3];
            long idx;
            for (idx = alignedStart + 4; idx < alignedEnd2; idx += 4) {
                p0_0 += v[idx] * v[idx];
                p0_1 += v[idx + 1] * v[idx + 1];
                p1_0 += v[idx + 2] * v[idx + 2];
                p1_1 += v[idx + 3] * v[idx + 3];
            }
            p0_0 += p1_0;
            p0_1 += p1_1;
            if (alignedEnd > alignedEnd2) {
                p0_0 += v[alignedEnd2] * v[alignedEnd2];
                p0_1 += v[alignedEnd2 + 1] * v[alignedEnd2 + 1];
            }
        }
        res = p0_0 + p0_1;
        { long idx; for (idx = 0; idx < alignedStart; ++idx) res += v[idx] * v[idx]; }
        { long idx; for (idx = alignedEnd; idx < n; ++idx) res += v[idx] * v[idx]; }
    } else {
        res = v[0] * v[0];
        long idx;
        for (idx = 1; idx < n; ++idx) res += v[idx] * v[idx];
    }
    return res;
}

/* Same LinearVectorizedTraversal/NoUnrolling shape as redux_sumsq, but for
 * a genuine two-operand dot product (Func==scalar_sum_op over the
 * elementwise a[i]*b[i] expression). This is the path Eigen actually takes
 * for `essential.adjoint() * bottom` when `bottom` has exactly one column:
 * confirmed by instrumenting a scratch copy of Eigen (Redux.h / Dot.h) at
 * runs/stella_port/reference_init/eigen_instrumented and tracing a failing
 * fixture -- for ncols==1 the product evaluator dispatches to
 * dot_nocheck<...>::run (`a.transpose().binaryExpr<conj_prod>(b).sum()`),
 * i.e. plain redux over a product expression, NOT the RowMajor
 * general_matrix_vector_product kernel used for ncols>=2 (see
 * apply_householder_left below). Its stride-4 two-accumulator grouping
 * (packet_res0 over indices {0,1,4,5,8,9,...}, packet_res1 over
 * {2,3,6,7,...}, merged at the end) differs from the GEMV kernel's simple
 * consecutive-pair (lane0=evens, lane1=odds) grouping used elsewhere in
 * this file -- a 1-ULP-class mismatch traced to exactly this distinction
 * (R[7,8] on one N=16 fixture, always at the k==cols-2 step where the
 * trailing block is 1 column wide). */
static double redux_dot(const double *a, const double *b, long n) {
    if (n <= 0) return 0.0;
    const long packetSize = 2;
    long alignedSize = (n / packetSize) * packetSize;
    long alignedSize2 = (n / (2 * packetSize)) * (2 * packetSize);
    double res;
    if (alignedSize) {
        double p0_0 = a[0] * b[0];
        double p0_1 = a[1] * b[1];
        if (alignedSize > packetSize) {
            double p1_0 = a[2] * b[2];
            double p1_1 = a[3] * b[3];
            long idx;
            for (idx = 4; idx < alignedSize2; idx += 4) {
                p0_0 += a[idx] * b[idx];
                p0_1 += a[idx + 1] * b[idx + 1];
                p1_0 += a[idx + 2] * b[idx + 2];
                p1_1 += a[idx + 3] * b[idx + 3];
            }
            p0_0 += p1_0;
            p0_1 += p1_1;
            if (alignedSize > alignedSize2) {
                p0_0 += a[alignedSize2] * b[alignedSize2];
                p0_1 += a[alignedSize2 + 1] * b[alignedSize2 + 1];
            }
        }
        res = p0_0 + p0_1;
        { long idx; for (idx = alignedSize; idx < n; ++idx) res += a[idx] * b[idx]; }
    } else {
        res = a[0] * b[0];
        long idx;
        for (idx = 1; idx < n; ++idx) res += a[idx] * b[idx];
    }
    return res;
}

static double col_norm(const double *base, int ld, int col, int row0, int n) {
    return sqrt(redux_sumsq(ccol_ptr(base, ld, col) + row0, n));
}

/* --------------------------------------------------------------------
 * makeHouseholderInPlace: Eigen/src/Householder/Householder.h.
 * Operates on x = column `col` of `base`, rows [row0, row0+len).
 * On return: x[0] -> beta (returned), x[1..] -> essential vector, *tau set.
 * -------------------------------------------------------------------- */
static double make_householder_in_place(double *base, int ld, int col, int row0, int len, double *tau) {
    double *x = col_ptr(base, ld, col) + row0;
    double c0 = x[0];
    double beta;
    if (len == 1) {
        *tau = 0.0;
        beta = c0;
        return beta;
    }
    double tailSqNorm = redux_sumsq(x + 1, len - 1);
    if (tailSqNorm <= SV_MIN) {
        *tau = 0.0;
        beta = c0;
        for (int i = 1; i < len; ++i) x[i] = 0.0;
        return beta;
    }
    beta = sqrt(c0 * c0 + tailSqNorm);
    if (c0 >= 0.0) beta = -beta;
    double denom = c0 - beta;
    for (int i = 1; i < len; ++i) x[i] = x[i] / denom;
    *tau = (beta - c0) / beta;
    return beta;
}

/* --------------------------------------------------------------------
 * applyHouseholderOnTheLeft on the block base[r0..r0+nrows, c0..c0+ncols),
 * leading dimension ld, essential vector given explicitly (length nrows-1).
 * tmp is scratch, length >= ncols.
 * The `tmp[j] = dot(essential, bottom_col_j)` step replicates Eigen's
 * general_matrix_vector_product<...,RowMajor,...>::run kernel (see
 * GeneralMatrixVector.h): for double/SSE2 it always pairs up (j,j+1) from
 * index 0 (no alignment probing -- the kernel hardcodes Unaligned loads),
 * accumulates two lanes independently, adds them (predux) after the last
 * full pair, then appends any single leftover element -- NOT the same
 * alignedStart-aware splitting used by plain redux (squaredNorm/norm).
 * -------------------------------------------------------------------- */
static void apply_householder_left(double *base, int ld, int r0, int c0, int nrows, int ncols,
                                    const double *essential, double tau, double *tmp, double *scaled_essential) {
    if (nrows == 1) {
        double f = 1.0 - tau;
        for (int j = 0; j < ncols; ++j) *(col_ptr(base, ld, c0 + j) + r0) *= f;
        return;
    }
    if (tau == 0.0) return;
    int n = nrows - 1;
    if (ncols == 1) {
        /* Eigen dispatches `essential.adjoint() * bottom` to dot_nocheck
         * (plain redux over a product expression), not the GEMV kernel,
         * when the "matrix" operand collapses to a single column. */
        const double *bcol = ccol_ptr(base, ld, c0) + (r0 + 1);
        tmp[0] = redux_dot(essential, bcol, n);
    } else {
        for (int j = 0; j < ncols; ++j) {
            const double *bcol = ccol_ptr(base, ld, c0 + j) + (r0 + 1);
            double lane0 = 0.0, lane1 = 0.0;
            int i = 0;
            for (; i + 2 <= n; i += 2) {
                lane0 += essential[i] * bcol[i];
                lane1 += essential[i + 1] * bcol[i + 1];
            }
            double cc = lane0 + lane1;
            for (; i < n; ++i) cc += essential[i] * bcol[i];
            tmp[j] = cc;
        }
    }
    for (int j = 0; j < ncols; ++j) tmp[j] += *(col_ptr(base, ld, c0 + j) + r0);
    for (int j = 0; j < ncols; ++j) *(col_ptr(base, ld, c0 + j) + r0) -= tau * tmp[j];
    for (int i = 0; i < n; ++i) scaled_essential[i] = tau * essential[i];
    for (int j = 0; j < ncols; ++j) {
        double *bcol = col_ptr(base, ld, c0 + j) + (r0 + 1);
        double t = tmp[j];
        for (int i = 0; i < n; ++i) bcol[i] -= scaled_essential[i] * t;
    }
}


int sv_pnp_eigen_svd(const double *a,int rows,int cols,double *u,double *v,double *s) {
    if(rows<cols || rows>12 || cols<1)return -1;
    double scale=0;for(int i=0;i<rows*cols;i++)if(fabs(a[i])>scale)scale=fabs(a[i]);
    if(scale==0)scale=1;
    double work[144]={0};
    memset(u,0,(size_t)rows*rows*sizeof(double));
    memset(v,0,(size_t)cols*cols*sizeof(double));
    if(rows==cols){
        for(int i=0;i<rows*cols;i++)work[i]=a[i]/scale;
        for(int i=0;i<cols;i++){u[i+rows*i]=1;v[i+cols*i]=1;}
    }else{
        double qr[144],hc[12],maxpivot;int perm[12],nz;
        for(int i=0;i<rows*cols;i++)qr[i]=a[i]/scale;
        sv_eigen_qr_colpiv(qr,rows,cols,hc,perm,&nz,&maxpivot);
        sv_eigen_qr_colpiv_householderq_full(qr,rows,cols,hc,u);
        for(int j=0;j<cols;j++){
            v[perm[j]+cols*j]=1;
            for(int i=0;i<=j;i++)work[i+cols*j]=qr[i+rows*j];
        }
    }
    jacobi_svd_core(work,cols,u,rows,v,cols,s,scale,NULL);
    return 0;
}
void sv_pnp_eigen_svd_solve(const double *a,int rows,int cols,const double *b,double *x){
    double u[144],v[144],s[12],tmp[12];sv_pnp_eigen_svd(a,rows,cols,u,v,s);
    int rank=0;double threshold=fmax(s[0]*(cols*SV_EPS),SV_MIN);
    for(int k=0;k<cols;k++)if(s[k]>=threshold)rank++;
    for(int k=0;k<rank;k++){
        double l0=0,l1=0;int i=0;
        for(;i+1<rows;i+=2){l0+=u[i+rows*k]*b[i];l1+=u[i+1+rows*k]*b[i+1];}
        double sum=l0+l1;for(;i<rows;i++)sum+=u[i+rows*k]*b[i];
        tmp[k]=(1.0/s[k])*sum;
    }
    for(int i=0;i<cols;i++){double sum=0;for(int k=0;k<rank;k++)sum+=v[i+cols*k]*tmp[k];x[i]=sum;}
}
/* GeneralMatrixMatrix product of a dynamic transpose with its source.
 * The pinned SSE2 gebp kernel accumulates each cache panel in order.
 * Reference cache profile: L1=32768, mr=nr=4 doubles; max_kc=504. */
void sv_pnp_eigen_gram(const double *a,int rows,int cols,double *out){
    for(int j=0;j<cols;j++)for(int i=0;i<cols;i++){
        double sum=0;
        if(rows+2*cols<20)sum=redux_dot(a+rows*i,a+rows*j,rows);
        else {
            int kc=rows;
            if(rows>504)kc=rows%504==0?504:504-8*((503-rows%504)/(8*(rows/504+1)));
            for(int start=0;start<rows;start+=kc){
                double panel=0;int end=start+kc<rows?start+kc:rows;
                for(int k=start;k<end;k++)panel+=a[k+rows*i]*a[k+rows*j];
                sum+=panel;
            }
        }
        out[i+cols*j]=sum;
    }
}

void sv_pnp_eigen_qr_solve(const double a[24],const double b[6],double x[4]){
    double qr[24],hc[4],tmp[6],scaled[6],c[6];
    memcpy(qr,a,sizeof(qr));memcpy(c,b,sizeof(c));
    for(int k=0;k<4;k++){
        double beta=make_householder_in_place(qr,6,k,k,6-k,hc+k);
        qr[k+6*k]=beta;
        apply_householder_left(qr,6,k,k+1,6-k,3-k,qr+k+1+6*k,hc[k],tmp,scaled);
    }
    for(int k=0;k<4;k++)apply_householder_left(c,6,k,0,6-k,1,qr+k+1+6*k,hc[k],tmp,scaled);
    for(int k=3;k>=0;k--)if(c[k]!=0){c[k]/=qr[k+6*k];for(int i=0;i<k;i++)c[i]-=c[k]*qr[i+6*k];}
    memcpy(x,c,4*sizeof(double));
}

void sv_pnp_eigen_mul3(const double *a,const double *b,double *out){
    for(int j=0;j<3;j++)for(int i=0;i<3;i++)out[i+3*j]=(a[i]*b[3*j]+a[i+3]*b[1+3*j])+a[i+6]*b[2+3*j];
}

void sv_pnp_eigen_mixed_mul3(const double *tmp,const double *vt,double *r){
        /* Dynamic-row product: each 3-row column has an alternating
         * alignment prefix; scalar coefficients use the 3-term tree. */
        for(int j=0;j<3;j++)for(int i=0;i<3;i++){
            double a=tmp[i]*vt[3*j],b=tmp[i+3]*vt[1+3*j],c=tmp[i+6]*vt[2+3*j];
            r[i+3*j]=((j%2==0 && i==2)||(j%2==1 && i==0))?a+(b+c):(a+b)+c;
        }
}
