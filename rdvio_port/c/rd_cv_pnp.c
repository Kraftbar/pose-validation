/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
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
static void choose_control_points(E *e);
static void compute_barycentric_coordinates(E *e);
static void fill_M(E *e, CvMat * M,
      const int row, const double * as, const double u, const double v);
static void compute_ccs(E *e, const double * betas, const double * ut);
static void compute_pcs(E *e);
static void compute_pose(E *e, double *R, double *t);
static double dist2(E *e, const double * p1, const double * p2);
static double dot(E *e, const double * v1, const double * v2);
static void estimate_R_and_t(E *e, double R[3][3], double t[3]);
static void solve_for_sign(E *e);
static double compute_R_and_t(E *e, const double * ut, const double * betas,
           double R[3][3], double t[3]);
static double reprojection_error(E *e, const double R[3][3], const double t[3]);
static void find_betas_approx_1(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas);
static void find_betas_approx_2(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas);
static void find_betas_approx_3(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas);
static void compute_L_6x10(E *e, const double * ut, double * l_6x10);
static void compute_rho(E *e, double * rho);
static void compute_A_and_b_gauss_newton(E *e, const double * l_6x10, const double * rho,
          const double betas[4], CvMat * A, CvMat * b);
static void gauss_newton(E *e, const CvMat * L_6x10, const CvMat * Rho, double betas[4]);
static void qr_solve(E *e, CvMat * A, CvMat * b, CvMat * X);
static void choose_control_points(E *e)
{
  // Take C0 as the reference points centroid:
  e->cws[0][0] = e->cws[0][1] = e->cws[0][2] = 0;
  for(int i = 0; i < e->number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      e->cws[0][j] += e->pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    e->cws[0][j] /= e->number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  double pwbuffer[18]; CvMat pwm = cvMat(e->number_of_correspondences,3,CV_64F,pwbuffer); CvMat *PW0=&pwm;

  double pw0tpw0[3 * 3] = {0}, dc[3] = {0}, uct[3 * 3] = {0};
  CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
  CvMat DC      = cvMat(3, 1, CV_64F, dc);
  CvMat UCt     = cvMat(3, 3, CV_64F, uct);

  for(int i = 0; i < e->number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.db[3 * i + j] = e->pws[3 * i + j] - e->cws[0][j];

  cvMulTransposed(PW0, &PW0tPW0, 1);
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);



  for(int i = 1; i < 4; i++) {
    double k = sqrt(dc[i - 1] / e->number_of_correspondences);
    for(int j = 0; j < 3; j++)
      e->cws[i][j] = e->cws[0][j] + k * uct[3 * (i - 1) + j];
  }
}

static void compute_barycentric_coordinates(E *e)
{
  double cc[3 * 3] = {0}, cc_inv[3 * 3] = {0};
  CvMat CC     = cvMat(3, 3, CV_64F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)
      cc[3 * i + j - 1] = e->cws[j][i] - e->cws[0][i];

  cvInvert(&CC, &CC_inv, CV_SVD);
  double * ci = cc_inv;
  for(int i = 0; i < e->number_of_correspondences; i++) {
    double * pi = &e->pws[0] + 3 * i;
    double * a = &e->alphas[0] + 4 * i;

    for(int j = 0; j < 3; j++)
    {
      a[1 + j] =
          ci[3 * j    ] * (pi[0] - e->cws[0][0]) +
          ci[3 * j + 1] * (pi[1] - e->cws[0][1]) +
          ci[3 * j + 2] * (pi[2] - e->cws[0][2]);
    }
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

static void fill_M(E *e, CvMat * M,
      const int row, const double * as, const double u, const double v)
{
  double * M1 = M->data.db + row * 12;
  double * M2 = M1 + 12;

  for(int i = 0; i < 4; i++) {
    M1[3 * i    ] = as[i] * e->fu;
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (e->uc - u);

    M2[3 * i    ] = 0.0;
    M2[3 * i + 1] = as[i] * e->fv;
    M2[3 * i + 2] = as[i] * (e->vc - v);
  }
}

static void compute_ccs(E *e, const double * betas, const double * ut)
{
  for(int i = 0; i < 4; i++)
    e->ccs[i][0] = e->ccs[i][1] = e->ccs[i][2] = 0.0f;

  for(int i = 0; i < 4; i++) {
    const double * v = ut + 12 * (11 - i);
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
        e->ccs[j][k] += betas[i] * v[3 * j + k];
  }
}

static void compute_pcs(E *e)
{
  for(int i = 0; i < e->number_of_correspondences; i++) {
    double * a = &e->alphas[0] + 4 * i;
    double * pc = &e->pcs[0] + 3 * i;

    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * e->ccs[0][j] + a[1] * e->ccs[1][j] + a[2] * e->ccs[2][j] + a[3] * e->ccs[3][j];
  }
}

static void compute_pose(E *e, double *R, double *t)
{
  choose_control_points(e);
  emit(e,"cws",e->cws,sizeof(e->cws));
  compute_barycentric_coordinates(e);
  emit(e,"alphas",e->alphas,e->number_of_correspondences*4*sizeof(double));

  double mbuffer[144]; CvMat mm=cvMat(2*e->number_of_correspondences,12,CV_64F,mbuffer); CvMat *M=&mm;

  for(int i = 0; i < e->number_of_correspondences; i++)
    fill_M(e, M, 2 * i, &e->alphas[0] + 4 * i, e->us[2 * i], e->us[2 * i + 1]);

  double mtm[12 * 12] = {0}, d[12] = {0}, ut[12 * 12] = {0};
  CvMat MtM = cvMat(12, 12, CV_64F, mtm);
  CvMat D   = cvMat(12,  1, CV_64F, d);
  CvMat Ut  = cvMat(12, 12, CV_64F, ut);

  cvMulTransposed(M, &MtM, 1);
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);


  double l_6x10[6 * 10] = {0}, rho[6] = {0};
  CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
  CvMat Rho    = cvMat(6,  1, CV_64F, rho);

  emit(e,"mtm",mtm,sizeof(mtm));
  emit(e,"ut",ut,sizeof(ut));
  compute_L_6x10(e, ut, l_6x10);
  compute_rho(e, rho);

  double Betas[4][4] = {0}, rep_errors[4] = {0};
  double Rs[4][3][3] = {0}, ts[4][3] = {0};

  find_betas_approx_1(e, &L_6x10, &Rho, Betas[1]);
  gauss_newton(e, &L_6x10, &Rho, Betas[1]);
  rep_errors[1] = compute_R_and_t(e, ut, Betas[1], Rs[1], ts[1]);

  find_betas_approx_2(e, &L_6x10, &Rho, Betas[2]);
  gauss_newton(e, &L_6x10, &Rho, Betas[2]);
  rep_errors[2] = compute_R_and_t(e, ut, Betas[2], Rs[2], ts[2]);

  find_betas_approx_3(e, &L_6x10, &Rho, Betas[3]);
  gauss_newton(e, &L_6x10, &Rho, Betas[3]);
  rep_errors[3] = compute_R_and_t(e, ut, Betas[3], Rs[3], ts[3]);

  emit(e,"betas",Betas,sizeof(Betas));
  emit(e,"errors",rep_errors,sizeof(rep_errors));
  emit(e,"Rs",Rs,sizeof(Rs));
  emit(e,"ts",ts,sizeof(ts));
  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  memcpy(t,ts[N],3*sizeof(double));
  memcpy(R,Rs[N],9*sizeof(double));
}

static double dist2(E *e, const double * p1, const double * p2)
{
  (void)e;
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

static double dot(E *e, const double * v1, const double * v2)
{
  (void)e;
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

static void estimate_R_and_t(E *e, double R[3][3], double t[3])
{
  double pc0[3] = {0}, pw0[3] = {0};

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < e->number_of_correspondences; i++) {
    const double * pc = &e->pcs[3 * i];
    const double * pw = &e->pws[3 * i];

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {
    pc0[j] /= e->number_of_correspondences;
    pw0[j] /= e->number_of_correspondences;
  }

  double abt[3 * 3] = {0}, abt_d[3] = {0}, abt_u[3 * 3] = {0}, abt_v[3 * 3] = {0};
  CvMat ABt   = cvMat(3, 3, CV_64F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

  cvSetZero(&ABt);
  for(int i = 0; i < e->number_of_correspondences; i++) {
    double * pc = &e->pcs[3 * i];
    double * pw = &e->pws[3 * i];

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = dot(e, abt_u + 3 * i, abt_v + 3 * j);

  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - dot(e, R[0], pw0);
  t[1] = pc0[1] - dot(e, R[1], pw0);
  t[2] = pc0[2] - dot(e, R[2], pw0);
}

static void solve_for_sign(E *e)
{
  if (e->pcs[2] < 0.0) {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
        e->ccs[i][j] = -e->ccs[i][j];

    for(int i = 0; i < e->number_of_correspondences; i++) {
      e->pcs[3 * i    ] = -e->pcs[3 * i];
      e->pcs[3 * i + 1] = -e->pcs[3 * i + 1];
      e->pcs[3 * i + 2] = -e->pcs[3 * i + 2];
    }
  }
}

static double compute_R_and_t(E *e, const double * ut, const double * betas,
           double R[3][3], double t[3])
{
  compute_ccs(e, betas, ut);
  compute_pcs(e);

  solve_for_sign(e);

  estimate_R_and_t(e, R, t);

  return reprojection_error(e, R, t);
}

static double reprojection_error(E *e, const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < e->number_of_correspondences; i++) {
    double * pw = &e->pws[3 * i];
    double Xc = dot(e, R[0], pw) + t[0];
    double Yc = dot(e, R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(e, R[2], pw) + t[2]);
    double ue = e->uc + e->fu * Xc * inv_Zc;
    double ve = e->vc + e->fv * Yc * inv_Zc;
    double u = e->us[2 * i], v = e->us[2 * i + 1];

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }

  return sum2 / e->number_of_correspondences;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

static void find_betas_approx_1(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas)
{
  (void)e;
  double l_6x4[6 * 4] = {0}, b4[4] = {0};
  CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
  CvMat B4    = cvMat(4, 1, CV_64F, b4);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  cvSolve(&L_6x4, Rho, &B4, CV_SVD);

  if (b4[0] < 0) {
    betas[0] = sqrt(-b4[0]);
    betas[1] = -b4[1] / betas[0];
    betas[2] = -b4[2] / betas[0];
    betas[3] = -b4[3] / betas[0];
  } else {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

static void find_betas_approx_2(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas)
{
  (void)e;
  double l_6x3[6 * 3] = {0}, b3[3] = {0};
  CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);
  CvMat B3     = cvMat(3, 1, CV_64F, b3);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
  } else {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) betas[0] = -betas[0];

  betas[2] = 0.0;
  betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

static void find_betas_approx_3(E *e, const CvMat * L_6x10, const CvMat * Rho,
             double * betas)
{
  (void)e;
  double l_6x5[6 * 5] = {0}, b5[5] = {0};
  CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
  CvMat B5    = cvMat(5, 1, CV_64F, b5);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];
  betas[2] = b5[3] / betas[0];
  betas[3] = 0.0;
}

static void compute_L_6x10(E *e, const double * ut, double * l_6x10)
{
  const double * v[4];

  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  double dv[4][6][3] = {0};

  for(int i = 0; i < 4; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      b++;
      if (b > 3) {
        a++;
        b = a + 1;
      }
    }
  }

  for(int i = 0; i < 6; i++) {
    double * row = l_6x10 + 10 * i;

    row[0] =        dot(e, dv[0][i], dv[0][i]);
    row[1] = 2.0f * dot(e, dv[0][i], dv[1][i]);
    row[2] =        dot(e, dv[1][i], dv[1][i]);
    row[3] = 2.0f * dot(e, dv[0][i], dv[2][i]);
    row[4] = 2.0f * dot(e, dv[1][i], dv[2][i]);
    row[5] =        dot(e, dv[2][i], dv[2][i]);
    row[6] = 2.0f * dot(e, dv[0][i], dv[3][i]);
    row[7] = 2.0f * dot(e, dv[1][i], dv[3][i]);
    row[8] = 2.0f * dot(e, dv[2][i], dv[3][i]);
    row[9] =        dot(e, dv[3][i], dv[3][i]);
  }
}

static void compute_rho(E *e, double * rho)
{
  rho[0] = dist2(e, e->cws[0], e->cws[1]);
  rho[1] = dist2(e, e->cws[0], e->cws[2]);
  rho[2] = dist2(e, e->cws[0], e->cws[3]);
  rho[3] = dist2(e, e->cws[1], e->cws[2]);
  rho[4] = dist2(e, e->cws[1], e->cws[3]);
  rho[5] = dist2(e, e->cws[2], e->cws[3]);
}

static void compute_A_and_b_gauss_newton(E *e, const double * l_6x10, const double * rho,
          const double betas[4], CvMat * A, CvMat * b)
{
  (void)e;
  for(int i = 0; i < 6; i++) {
    const double * rowL = l_6x10 + i * 10;
    double * rowA = A->data.db + i * 4;

    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    cvmSet(b, i, 0, rho[i] -
     (
      rowL[0] * betas[0] * betas[0] +
      rowL[1] * betas[0] * betas[1] +
      rowL[2] * betas[1] * betas[1] +
      rowL[3] * betas[0] * betas[2] +
      rowL[4] * betas[1] * betas[2] +
      rowL[5] * betas[2] * betas[2] +
      rowL[6] * betas[0] * betas[3] +
      rowL[7] * betas[1] * betas[3] +
      rowL[8] * betas[2] * betas[3] +
      rowL[9] * betas[3] * betas[3]
      ));
  }
}

static void gauss_newton(E *e, const CvMat * L_6x10, const CvMat * Rho, double betas[4])
{
  const int iterations_number = 5;

  double a[6*4] = {0}, b[6] = {0}, x[4] = {0};
  CvMat A = cvMat(6, 4, CV_64F, a);
  CvMat B = cvMat(6, 1, CV_64F, b);
  CvMat X = cvMat(4, 1, CV_64F, x);

  for(int k = 0; k < iterations_number; k++)
  {
    compute_A_and_b_gauss_newton(e, L_6x10->data.db, Rho->data.db,
    betas, &A, &B);
    qr_solve(e, &A, &B, &X);
    for(int i = 0; i < 4; i++)
    betas[i] += x[i];
  }
}

static void qr_solve(E *e, CvMat * A, CvMat * b, CvMat * X)
{
  const int nr = A->rows;
  const int nc = A->cols;
  if (nc <= 0 || nr <= 0)
      return;

  double * pA = A->data.db, * ppAkk = pA;
  for(int k = 0; k < nc; k++)
  {
    double * ppAik1 = ppAkk, eta = fabs(*ppAik1);
    for(int i = k + 1; i < nr; i++)
    {
      double elt = fabs(*ppAik1);
      if (eta < elt) eta = elt;
      ppAik1 += nc;
    }
    if (eta == 0)
    {
      e->A1[k] = e->A2[k] = 0.0;
      //cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    }
    else
    {
      double * ppAik2 = ppAkk, sum2 = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++)
      {
        *ppAik2 *= inv_eta;
        sum2 += *ppAik2 * *ppAik2;
        ppAik2 += nc;
      }
      double sigma = sqrt(sum2);
      if (*ppAkk < 0)
      sigma = -sigma;
      *ppAkk += sigma;
      e->A1[k] = sigma * *ppAkk;
      e->A2[k] = -eta * sigma;
      for(int j = k + 1; j < nc; j++)
      {
        double * ppAik = ppAkk, sum = 0;
        for(int i = k; i < nr; i++)
        {
          sum += *ppAik * ppAik[j - k];
          ppAik += nc;
        }
        double tau = sum / e->A1[k];
        ppAik = ppAkk;
        for(int i = k; i < nr; i++)
        {
          ppAik[j - k] -= tau * *ppAik;
          ppAik += nc;
        }
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->data.db;
  for(int j = 0; j < nc; j++)
  {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)
    {
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= e->A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++)
    {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->data.db;
  pX[nc - 1] = pb[nc - 1] / e->A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--)
  {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++)
    {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / e->A2[i];
  }
}

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
