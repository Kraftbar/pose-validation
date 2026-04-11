#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <omp.h>

#include "pure_c_math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { const char* video_path; double seconds, timeout; const char* metrics_out; int kf_min_inliers; double kf_max_rot_deg; int max_points; } Config;
typedef struct { float x, y; int pt_idx; } Corner;
typedef struct { int query_idx, train_idx; float score; } Match;
typedef struct { int frame_id, inliers, is_keyframe, points_added, points_total; double xyz[3]; } FrameStat;
typedef struct { Corner* data; int size, cap; } CornerVec;
typedef struct { Match* data; int size, cap; } MatchVec;
typedef struct { FrameStat* data; int size, cap; } FrameStatVec;
typedef struct { double m[16]; } Pose;
typedef struct { CornerVec corners; Pose pose; unsigned char* gray; } FrameLite;
typedef struct { double x, y, z; int obs; } MapPoint;
typedef struct { MapPoint* data; int size, cap; } Map;

static void* xrealloc(void* ptr, size_t size) { void* out = realloc(ptr, size); if(!out){ fprintf(stderr, "out of memory\n"); exit(1); } return out; }
static void corner_vec_push(CornerVec* vec, Corner v) { if(vec->size == vec->cap){ vec->cap = vec->cap ? vec->cap * 2 : 256; vec->data = (Corner*)xrealloc(vec->data, (size_t)vec->cap * sizeof(Corner)); } vec->data[vec->size++] = v; }
static void match_vec_push(MatchVec* vec, Match v) { if(vec->size == vec->cap){ vec->cap = vec->cap ? vec->cap * 2 : 256; vec->data = (Match*)xrealloc(vec->data, (size_t)vec->cap * sizeof(Match)); } vec->data[vec->size++] = v; }
static void frame_stat_vec_push(FrameStatVec* vec, FrameStat v) { if(vec->size == vec->cap){ vec->cap = vec->cap ? vec->cap * 2 : 256; vec->data = (FrameStat*)xrealloc(vec->data, (size_t)vec->cap * sizeof(FrameStat)); } vec->data[vec->size++] = v; }
static void map_push(Map* m, MapPoint p) { if(m->size == m->cap){ m->cap = m->cap ? m->cap * 2 : 1024; m->data = (MapPoint*)xrealloc(m->data, (size_t)m->cap * sizeof(MapPoint)); } m->data[m->size++] = p; }

static void pose_identity(Pose* p) { memset(p->m, 0, sizeof(p->m)); for(int i=0; i<4; i++) p->m[i*4+i]=1.0; }
static void pose_from_rt(const double R[9], const double t[3], Pose* p) { pose_identity(p); for(int r=0; r<3; r++){ for(int c=0; c<3; c++) p->m[r*4+c]=R[r*3+c]; p->m[r*4+3]=t[r]; } }
static void pose_get_rotation(const Pose* p, double R[9]) { for(int r=0; r<3; r++) for(int c=0; c<3; c++) R[r*3+c]=p->m[r*4+c]; }
static void pose_get_translation(const Pose* p, double t[3]) { t[0]=p->m[3]; t[1]=p->m[7]; t[2]=p->m[11]; }
static void mat3_transpose(const double A[9], double T[9]) { for(int r=0; r<3; r++) for(int c=0; c<3; c++) T[r*3+c]=A[c*3+r]; }
static void mat3_mul(const double A[9], const double B[9], double C[9]) { for(int r=0; r<3; r++) for(int c=0; c<3; c++){ double s=0; for(int k=0; k<3; k++) s+=A[r*3+k]*B[k*3+c]; C[r*3+c]=s; } }
static void mat3_vec_mul(const double A[9], const double x[3], double y[3]) { for(int r=0; r<3; r++){ double s=0; for(int k=0; k<3; k++) s+=A[r*3+k]*x[k]; y[r]=s; } }
static double mat3_det(const double A[9]) { return A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]); }
static void pose_compose_relative(const Pose* rel, const Pose* prev, Pose* out) { double Rrel[9], Rprev[9], Rout[9], trel[3], tprev[3], tout[3]; pose_get_rotation(rel,Rrel); pose_get_rotation(prev,Rprev); pose_get_translation(rel,trel); pose_get_translation(prev,tprev); mat3_mul(Rrel,Rprev,Rout); mat3_vec_mul(Rrel,tprev,tout); tout[0]+=trel[0]; tout[1]+=trel[1]; tout[2]+=trel[2]; pose_from_rt(Rout,tout,out); }
static void camera_center_from_pose(const Pose* pose, double c[3]) { double R[9], Rt[9], t[3], tmp[3]; pose_get_rotation(pose,R); pose_get_translation(pose,t); mat3_transpose(R,Rt); mat3_vec_mul(Rt,t,tmp); c[0]=-tmp[0]; c[1]=-tmp[1]; c[2]=-tmp[2]; }
static double rotation_degrees_between(const Pose* a, const Pose* b) { double Ra[9], Rb[9], Rat[9], R[9]; pose_get_rotation(a,Ra); pose_get_rotation(b,Rb); mat3_transpose(Ra,Rat); mat3_mul(Rb,Rat,R); double tr=R[0]+R[4]+R[8], cos_th=(tr-1.0)*0.5; if(cos_th>1)cos_th=1; if(cos_th<-1)cos_th=-1; return acos(cos_th)*180.0/M_PI; }
static void normalize_point(double fx, double fy, double cx, double cy, Corner p, double out[3]) { out[0]=((double)p.x-cx)/fx; out[1]=((double)p.y-cy)/fy; out[2]=1.0; }

static int triangulate_point(const Pose* p1, const Pose* p2, Corner pt1, Corner pt2, double fx, double fy, double cx, double cy, double X[3]) {
  double K[9]={fx,0,cx, 0,fy,cy, 0,0,1}, R1[9], R2[9], t1[3], t2[3], Rt1[12], Rt2[12], P1[12], P2[12], A[16], VT[4];
  pose_get_rotation(p1,R1); pose_get_rotation(p2,R2); pose_get_translation(p1,t1); pose_get_translation(p2,t2);
  for(int r=0; r<3; r++){ Rt1[r*4+0]=R1[r*3+0]; Rt1[r*4+1]=R1[r*3+1]; Rt1[r*4+2]=R1[r*3+2]; Rt1[r*4+3]=t1[r]; Rt2[r*4+0]=R2[r*3+0]; Rt2[r*4+1]=R2[r*3+1]; Rt2[r*4+2]=R2[r*3+2]; Rt2[r*4+3]=t2[r]; }
  for(int r=0; r<3; r++) for(int c=0; c<4; c++){ P1[r*4+c]=K[r*3+0]*Rt1[c]+K[r*3+1]*Rt1[c+4]+K[r*3+2]*Rt1[c+8]; P2[r*4+c]=K[r*3+0]*Rt2[c]+K[r*3+1]*Rt2[c+4]+K[r*3+2]*Rt2[c+8]; }
  for(int c=0; c<4; c++){ A[0*4+c]=pt1.x*P1[2*4+c]-P1[0*4+c]; A[1*4+c]=pt1.y*P1[2*4+c]-P1[1*4+c]; A[2*4+c]=pt2.x*P2[2*4+c]-P2[0*4+c]; A[3*4+c]=pt2.y*P2[2*4+c]-P2[1*4+c]; }
  double AtA[16]={0}, V[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  for(int r=0; r<4; r++) for(int c=0; c<4; c++) for(int k=0; k<4; k++) AtA[r*4+c]+=A[k*4+r]*A[k*4+c];
  for(int it=0; it<30; it++){ int p=0, q=1; double mv=0; for(int i=0; i<3; i++) for(int j=i+1; j<4; j++) if(fabs(AtA[i*4+j])>mv){mv=fabs(AtA[i*4+j]); p=i; q=j;}
    if(mv<1e-15)break; double ph=0.5*atan2(2.0*AtA[p*4+q], AtA[q*4+q]-AtA[p*4+p]), co=cos(ph), si=sin(ph);
    for(int i=0; i<4; i++){ double r1=AtA[i*4+p], r2=AtA[i*4+q]; AtA[i*4+p]=co*r1-si*r2; AtA[i*4+q]=si*r1+co*r2; }
    for(int i=0; i<4; i++){ double r1=AtA[p*4+i], r2=AtA[q*4+i]; AtA[p*4+i]=co*r1-si*r2; AtA[q*4+i]=si*r1+co*r2; }
    for(int i=0; i<4; i++){ double v1=V[i*4+p], v2=V[i*4+q]; V[i*4+p]=co*v1-si*v2; V[i*4+q]=si*v1+co*v2; } }
  int bi=0; double mw=AtA[0]; for(int i=1; i<4; i++) if(AtA[i*4+i]<mw){mw=AtA[i*4+i]; bi=i;} for(int i=0; i<4; i++) VT[i]=V[i*4+bi];
  if(fabs(VT[3])<1e-8)return 0; X[0]=VT[0]/VT[3]; X[1]=VT[1]/VT[3]; X[2]=VT[2]/VT[3];
  double z1=R1[6]*X[0]+R1[7]*X[1]+R1[8]*X[2]+t1[2], z2=R2[6]*X[0]+R2[7]*X[1]+R2[8]*X[2]+t2[2];
  return (isfinite(X[0]) && z1>0 && z2>0);
}

static void enforce_essential_constraints(double E[9]) { double W[3], U[9], V[9], S[9]={0}, tmp[9], out[9]; svd_3x3(E,W,U,V); double sig=0.5*(W[0]+W[1]); S[0]=sig; S[4]=sig; S[8]=0; mat3_mul(U,S,tmp); mat3_mul(tmp,V,out); memcpy(E,out,9*sizeof(double)); }

static int decompose_and_choose_pose(const double E[9], const CornerVec* p_pts, const CornerVec* c_pts, const MatchVec* matches, const unsigned char* mask, double fx, double fy, double cx, double cy, Pose* out_rel) {
  double W[3], U[9], V[9], R1[9], R2[9], t[3]; Pose cands[4]; int bi=-1, bc=-1; svd_3x3(E,W,U,V); t[0]=U[2]; t[1]=U[5]; t[2]=U[8];
  double Wm[9]={0,-1,0, 1,0,0, 0,0,1}, Wmt[9]={0,1,0, -1,0,0, 0,0,1}, tmp[9], VT[9]; for(int i=0;i<3;i++)for(int j=0;j<3;j++)VT[i*3+j]=V[j*3+i];
  mat3_mul(U,Wm,tmp); mat3_mul(tmp,VT,R1); mat3_mul(U,Wmt,tmp); mat3_mul(tmp,VT,R2);
  if(mat3_det(R1)<0)for(int i=0;i<9;i++)R1[i]=-R1[i]; if(mat3_det(R2)<0)for(int i=0;i<9;i++)R2[i]=-R2[i];
  pose_from_rt(R1,t,&cands[0]); pose_from_rt(R1,(double[]){-t[0],-t[1],-t[2]},&cands[1]); pose_from_rt(R2,t,&cands[2]); pose_from_rt(R2,(double[]){-t[0],-t[1],-t[2]},&cands[3]);
  for(int i=0;i<4;i++){ Pose id; pose_identity(&id); int g=0; for(int j=0;j<matches->size && j<32;j++){ double X[3]; if(mask && !mask[j])continue; if(triangulate_point(&id,&cands[i],p_pts->data[matches->data[j].query_idx],c_pts->data[matches->data[j].train_idx],fx,fy,cx,cy,X))g++; } if(g>bc){bc=g; bi=i;} }
  if(bi>=0){ *out_rel=cands[bi]; return 1; } return 0;
}

// Hartley-normalized 8-point DLT on K-normalized homogeneous points.
// n1,n2 are stride-3 arrays; idxs selects n rows. Returns 1 on success.
static int fit_E_hartley(const double* n1, const double* n2, const int* idxs, int n, double E[9]) {
  if (n < 8) return 0;
  double mx1 = 0, my1 = 0, mx2 = 0, my2 = 0;
  for (int i = 0; i < n; i++) {
    const double* a = &n1[idxs[i]*3]; const double* b = &n2[idxs[i]*3];
    mx1 += a[0]; my1 += a[1]; mx2 += b[0]; my2 += b[1];
  }
  mx1 /= n; my1 /= n; mx2 /= n; my2 /= n;
  double d1 = 0, d2 = 0;
  for (int i = 0; i < n; i++) {
    const double* a = &n1[idxs[i]*3]; const double* b = &n2[idxs[i]*3];
    double ax = a[0] - mx1, ay = a[1] - my1;
    double bx = b[0] - mx2, by = b[1] - my2;
    d1 += sqrt(ax*ax + ay*ay); d2 += sqrt(bx*bx + by*by);
  }
  d1 /= n; d2 /= n;
  double s1 = (d1 > 1e-12) ? sqrt(2.0) / d1 : 1.0;
  double s2 = (d2 > 1e-12) ? sqrt(2.0) / d2 : 1.0;
  double AtA[81] = {0};
  for (int i = 0; i < n; i++) {
    const double* a = &n1[idxs[i]*3]; const double* b = &n2[idxs[i]*3];
    double x1 = s1 * (a[0] - mx1), y1 = s1 * (a[1] - my1);
    double x2 = s2 * (b[0] - mx2), y2 = s2 * (b[1] - my2);
    double A[9] = { x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1.0 };
    for (int r = 0; r < 9; r++) for (int c = 0; c < 9; c++) AtA[r*9+c] += A[r]*A[c];
  }
  double W[9], V[81];
  jacobi_9x9(AtA, W, V);
  int bi = 0; double mw = W[0];
  for (int k = 1; k < 9; k++) if (W[k] < mw) { mw = W[k]; bi = k; }
  double En[9]; for (int k = 0; k < 9; k++) En[k] = V[k*9 + bi];
  // Denormalize: E = T2^T * En * T1 where T1,T2 are Hartley transforms.
  double tmp[9] = {
    s2*En[0], s2*En[1], s2*En[2],
    s2*En[3], s2*En[4], s2*En[5],
    -s2*mx2*En[0] - s2*my2*En[3] + En[6],
    -s2*mx2*En[1] - s2*my2*En[4] + En[7],
    -s2*mx2*En[2] - s2*my2*En[5] + En[8],
  };
  E[0] = s1*tmp[0]; E[1] = s1*tmp[1]; E[2] = -s1*mx1*tmp[0] - s1*my1*tmp[1] + tmp[2];
  E[3] = s1*tmp[3]; E[4] = s1*tmp[4]; E[5] = -s1*mx1*tmp[3] - s1*my1*tmp[4] + tmp[5];
  E[6] = s1*tmp[6]; E[7] = s1*tmp[7]; E[8] = -s1*mx1*tmp[6] - s1*my1*tmp[7] + tmp[8];
  return 1;
}

static int estimate_pose_E(const CornerVec* p_pts, const CornerVec* c_pts, const MatchVec* matches, double fx, double fy, double cx, double cy, Pose* out_rel, unsigned char** out_mask, int* out_inliers) {
  const double th=1e-4; const int iters=150; double best_E[9]={0}; int best_inl=0; if(matches->size<8)return 0;
  double *n1=malloc(matches->size*3*sizeof(double)), *n2=malloc(matches->size*3*sizeof(double));
  for(int i=0;i<matches->size;i++){ normalize_point(fx,fy,cx,cy,p_pts->data[matches->data[i].query_idx],&n1[i*3]); normalize_point(fx,fy,cx,cy,c_pts->data[matches->data[i].train_idx],&n2[i*3]); }
  unsigned char* bmask=calloc(matches->size,1);
  unsigned char* used=malloc((size_t)matches->size);
  for(int i=0; i<iters; i++){
    int smp[8]; memset(used,0,(size_t)matches->size); double cand_E[9];
    for(int j=0;j<8;j++){ int idx; do{idx=rand()%matches->size;}while(used[idx]); used[idx]=1; smp[j]=idx; }
    fit_E_hartley(n1, n2, smp, 8, cand_E);
    enforce_essential_constraints(cand_E); int inl=0; double Et[9]; mat3_transpose(cand_E,Et);
    for(int j=0;j<matches->size;j++){ double *x1=&n1[j*3], *x2=&n2[j*3], Ex1[3]={cand_E[0]*x1[0]+cand_E[1]*x1[1]+cand_E[2], cand_E[3]*x1[0]+cand_E[4]*x1[1]+cand_E[5], cand_E[6]*x1[0]+cand_E[7]*x1[1]+cand_E[8]}, Etx2[3]={Et[0]*x2[0]+Et[1]*x2[1]+Et[2], Et[3]*x2[0]+Et[4]*x2[1]+Et[5], Et[6]*x2[0]+Et[7]*x2[1]+Et[8]}, num=x2[0]*Ex1[0]+x2[1]*Ex1[1]+Ex1[2], den=Ex1[0]*Ex1[0]+Ex1[1]*Ex1[1]+Etx2[0]*Etx2[0]+Etx2[1]*Etx2[1]+1e-12; if(num*num/den<th)inl++; }
    if(inl>best_inl){best_inl=inl; memcpy(best_E,cand_E,9*sizeof(double)); if(inl > matches->size*0.95) break; }
  }
  if(best_inl>=8){
    double Et[9]; mat3_transpose(best_E,Et); int* iidx=malloc(matches->size*sizeof(int)); int k=0;
    for(int j=0;j<matches->size;j++){ double *x1=&n1[j*3],*x2=&n2[j*3],Ex1[3]={best_E[0]*x1[0]+best_E[1]*x1[1]+best_E[2],best_E[3]*x1[0]+best_E[4]*x1[1]+best_E[5],best_E[6]*x1[0]+best_E[7]*x1[1]+best_E[8]}, Etx2[3]={Et[0]*x2[0]+Et[1]*x2[1]+Et[2],Et[3]*x2[0]+Et[4]*x2[1]+Et[5],Et[6]*x2[0]+Et[7]*x2[1]+Et[8]}, num=x2[0]*Ex1[0]+x2[1]*Ex1[1]+Ex1[2], den=Ex1[0]*Ex1[0]+Ex1[1]*Ex1[1]+Etx2[0]*Etx2[0]+Etx2[1]*Etx2[1]+1e-12; if(num*num/den<th){bmask[j]=1; iidx[k++]=j;} }
    if(k >= 8) { fit_E_hartley(n1, n2, iidx, k, best_E); enforce_essential_constraints(best_E); }
    free(iidx);
  }
  free(n1); free(n2); free(used);
  if(best_inl>=8 && decompose_and_choose_pose(best_E,p_pts,c_pts,matches,bmask,fx,fy,cx,cy,out_rel)){ *out_mask=bmask; *out_inliers=best_inl; return 1; }
  free(bmask); return 0;
}

static int estimate_pose_PnP(const Map* map, const CornerVec* corners, double fx, double fy, double cx, double cy, Pose* out_pose, int* out_inl) {
  int n=0; for(int i=0; i<corners->size; i++) if(corners->data[i].pt_idx != -1) n++;
  if(n < 12) return 0;
  int* ids = malloc(n * sizeof(int)); int k=0; for(int i=0; i<corners->size; i++) if(corners->data[i].pt_idx != -1) ids[k++] = i;
  double best_P[12]; int best_inl = 0;
  for(int it=0; it<150; it++){
    double AtA[144]={0}, W[12], V[144], P[12];
    for(int i=0; i<6; i++){
      int idx = ids[rand()%n]; MapPoint p = map->data[corners->data[idx].pt_idx];
      double u = (corners->data[idx].x - cx)/fx, v = (corners->data[idx].y - cy)/fy;
      double r1[12]={p.x,p.y,p.z,1, 0,0,0,0, -u*p.x,-u*p.y,-u*p.z,-u}, r2[12]={0,0,0,0, p.x,p.y,p.z,1, -v*p.x,-v*p.y,-v*p.z,-v};
      for(int r=0; r<12; r++) for(int c=0; c<12; c++) AtA[r*12+c] += r1[r]*r1[c] + r2[r]*r2[c];
    }
    jacobi_12x12(AtA, W, V); int bi=0; double mw=W[0]; for(int j=1;j<12;j++) if(W[j]<mw){mw=W[j]; bi=j;}
    for(int j=0; j<12; j++) P[j] = V[j*12+bi];
    int inl=0; double R[9]={P[0],P[1],P[2], P[4],P[5],P[6], P[8],P[9],P[10]}, t[3]={P[3],P[7],P[11]};
    double sc = 1.0 / sqrt(R[6]*R[6] + R[7]*R[7] + R[8]*R[8]); if(P[11]*sc < 0) sc = -sc;
    for(int j=0; j<9; j++) R[j]*=sc; for(int j=0; j<3; j++) t[j]*=sc;
    for(int j=0; j<n; j++){
      MapPoint p = map->data[corners->data[ids[j]].pt_idx];
      double cx_p = R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0], cy_p = R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1], cz_p = R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2];
      if(cz_p < 0.1) continue;
      double u = fx*cx_p/cz_p + cx, v = fy*cy_p/cz_p + cy;
      double dx = u - corners->data[ids[j]].x, dy = v - corners->data[ids[j]].y;
      if(dx*dx + dy*dy < 4.0) inl++;
    }
    if(inl > best_inl){ best_inl = inl; memcpy(best_P, P, 12*sizeof(double)); }
    if(inl > n * 0.8) break;
  }
  if(best_inl >= 12){
    double R[9]={best_P[0],best_P[1],best_P[2], best_P[4],best_P[5],best_P[6], best_P[8],best_P[9],best_P[10]}, t[3]={best_P[3],best_P[7],best_P[11]};
    double sc = 1.0 / sqrt(R[6]*R[6] + R[7]*R[7] + R[8]*R[8]); if(best_P[11]*sc < 0) sc = -sc;
    for(int j=0; j<9; j++) R[j]*=sc; for(int j=0; j<3; j++) t[j]*=sc;
    double W[3], U[9], V[9], Ro[9]; svd_3x3(R, W, U, V);
    double VT[9]; for(int i=0;i<3;i++)for(int j=0;j<3;j++)VT[i*3+j]=V[j*3+i]; mat3_mul(U, VT, Ro);
    if(mat3_det(Ro) < 0) for(int j=0;j<9;j++) Ro[j]=-Ro[j];
    pose_from_rt(Ro, t, out_pose); *out_inl = best_inl; free(ids); return 1;
  }
  free(ids); return 0;
}

static void refine_pose_lm(const Map* map, const CornerVec* corners, double fx, double fy, double cx, double cy, Pose* pose) {
  double R[9], t[3]; pose_get_rotation(pose, R); pose_get_translation(pose, t);
  double lambda = 1e-3;
  for (int iter = 0; iter < 10; iter++) {
    double H[36] = {0}, b[6] = {0};
    for (int i = 0; i < corners->size; i++) {
      if (corners->data[i].pt_idx == -1) continue;
      MapPoint p = map->data[corners->data[i].pt_idx];
      double cp[3] = { R[0]*p.x+R[1]*p.y+R[2]*p.z+t[0], R[3]*p.x+R[4]*p.y+R[5]*p.z+t[1], R[6]*p.x+R[7]*p.y+R[8]*p.z+t[2] };
      if (cp[2] < 0.1) continue;
      double inv_z = 1.0 / cp[2], inv_z2 = inv_z * inv_z;
      double u = fx * cp[0] * inv_z + cx, v = fy * cp[1] * inv_z + cy;
      double du = corners->data[i].x - u, dv = corners->data[i].y - v;
      double err2 = du*du + dv*dv;
      double weight = (err2 > 4.0) ? 2.0 / sqrt(err2) : 1.0; 
      double J[2][6] = {
        { fx*inv_z, 0, -fx*cp[0]*inv_z2, -fx*cp[0]*cp[1]*inv_z2, fx*(1+cp[0]*cp[0]*inv_z2), -fx*cp[1]*inv_z },
        { 0, fy*inv_z, -fy*cp[1]*inv_z2, -fy*(1+cp[1]*cp[1]*inv_z2), fy*cp[0]*cp[1]*inv_z2, fy*cp[0]*inv_z }
      };
      for (int r = 0; r < 6; r++) {
        b[r] += weight * (J[0][r] * du + J[1][r] * dv);
        for (int c = 0; c < 6; c++) H[r*6+c] += weight * (J[0][r]*J[0][c] + J[1][r]*J[1][c]);
      }
    }
    for(int i=0; i<6; i++) H[i*6+i] += lambda * H[i*6+i] + 1e-6;
    double dx[6]; if (!solve_6x6(H, b, dx)) break;
    t[0] += dx[0]; t[1] += dx[1]; t[2] += dx[2];
    double dr[9] = { 1, -dx[5], dx[4], dx[5], 1, -dx[3], -dx[4], dx[3], 1 }, Rn[9];
    mat3_mul(dr, R, Rn);
    double W[3], U[9], V[9], VT[9]; svd_3x3(Rn, W, U, V);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)VT[i*3+j]=V[j*3+i]; mat3_mul(U, VT, R);
    if (dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2] < 1e-8) break;
  }
  pose_from_rt(R, t, pose);
}

typedef struct { FILE* pipe; int w, h; } FFmpegCap;
static FFmpegCap* ffmpeg_open(const char* p){ FFmpegCap* c=malloc(sizeof(FFmpegCap)); c->w=640; c->h=480; char cmd[1024]; snprintf(cmd,1024,"ffmpeg -hide_banner -loglevel error -i \"%s\" -f rawvideo -pix_fmt bgr24 -s 640x480 -",p); c->pipe=popen(cmd,"r"); if(!c->pipe){free(c);return NULL;} return c; }
static int ffmpeg_read(FFmpegCap* c, unsigned char* b){ return fread(b,1,(size_t)c->w*c->h*3,c->pipe) == (size_t)c->w*c->h*3; }
static void ffmpeg_close(FFmpegCap* c){ if(c){pclose(c->pipe);free(c);} }
static void bgr_to_gray(const unsigned char* b, int w, int h, unsigned char* g){ for(int i=0;i<w*h;i++) g[i]=(unsigned char)(0.299f*b[i*3+2]+0.587f*b[i*3+1]+0.114f*b[i*3]); }
static void downsample2x(const unsigned char* src, int sw, int sh, unsigned char* dst){ int dw=sw/2, dh=sh/2; for(int y=0;y<dh;y++)for(int x=0;x<dw;x++) dst[y*dw+x]=src[(y*2)*sw+(x*2)]; }

static float get_pixel_bilinear(const unsigned char* g, int w, int h, float x, float y) {
  int ix=(int)x, iy=(int)y; float dx=x-ix, dy=y-iy; if(ix<0||ix>=w-1||iy<0||iy>=h-1) return 0;
  return (1-dx)*(1-dy)*g[iy*w+ix] + dx*(1-dy)*g[iy*w+ix+1] + (1-dx)*dy*g[(iy+1)*w+ix] + dx*dy*g[(iy+1)*w+ix+1];
}

static void blur_3x3(const unsigned char* src, int w, int h, unsigned char* dst) {
  #pragma omp parallel for collapse(2)
  for(int y=1; y<h-1; y++) for(int x=1; x<w-1; x++) {
    int s = src[(y-1)*w+x-1] + 2*src[(y-1)*w+x] + src[(y-1)*w+x+1] + 2*src[y*w+x-1] + 4*src[y*w+x] + 2*src[y*w+x+1] + src[(y+1)*w+x-1] + 2*src[(y+1)*w+x] + src[(y+1)*w+x+1];
    dst[y*w+x] = (unsigned char)(s >> 4);
  }
}

// Shi-Tomasi detector with per-cell bucket selection for spatial uniformity.
// Splits the image into BUCKET_GRID_X x BUCKET_GRID_Y cells and picks the
// strongest (max/num_cells) local maxima in each cell.
#define BUCKET_GRID_X 10
#define BUCKET_GRID_Y 8
#define CORNER_SCORE_THRESH 0.1f

static void extract_corners_pure(const unsigned char* g, int w, int h, CornerVec* c, int max) {
  float* s = calloc((size_t)w * h, sizeof(float));
  #pragma omp parallel for collapse(2)
  for (int y = 2; y < h - 2; y++) for (int x = 2; x < w - 2; x++) {
    float Ixx = 0, Iyy = 0, Ixy = 0;
    for (int i = -1; i <= 1; i++) for (int j = -1; j <= 1; j++) {
      float gx = (float)g[(y+i)*w + x+j+1] - g[(y+i)*w + x+j-1];
      float gy = (float)g[(y+i+1)*w + x+j] - g[(y+i-1)*w + x+j];
      Ixx += gx*gx; Iyy += gy*gy; Ixy += gx*gy;
    }
    float det = Ixx*Iyy - Ixy*Ixy, tr = Ixx + Iyy;
    s[y*w + x] = 0.5f * (tr - sqrtf(tr*tr - 4.0f*det + 1e-6f));
  }

  const int per_cell = (max / (BUCKET_GRID_X * BUCKET_GRID_Y)) + 1;
  const int cell_w = w / BUCKET_GRID_X;
  const int cell_h = h / BUCKET_GRID_Y;
  // Per-cell selection: keep a descending-score top-K list, then push them.
  float*  top_s = malloc((size_t)per_cell * sizeof(float));
  int*    top_x = malloc((size_t)per_cell * sizeof(int));
  int*    top_y = malloc((size_t)per_cell * sizeof(int));
  for (int gy = 0; gy < BUCKET_GRID_Y && c->size < max; gy++) {
    for (int gx = 0; gx < BUCKET_GRID_X && c->size < max; gx++) {
      int y0 = gy * cell_h, y1 = (gy + 1) * cell_h;
      int x0 = gx * cell_w, x1 = (gx + 1) * cell_w;
      if (y0 < 5) y0 = 5; if (y1 > h - 5) y1 = h - 5;
      if (x0 < 5) x0 = 5; if (x1 > w - 5) x1 = w - 5;
      int nt = 0;
      for (int y = y0; y < y1; y++) for (int x = x0; x < x1; x++) {
        float val = s[y*w + x];
        if (val < CORNER_SCORE_THRESH) continue;
        int ok = 1;
        for (int i = -3; i <= 3 && ok; i++)
          for (int j = -3; j <= 3 && ok; j++)
            if (s[(y+i)*w + x+j] > val) ok = 0;
        if (!ok) continue;
        // Insert into descending top-K list.
        if (nt < per_cell) {
          int p = nt;
          while (p > 0 && top_s[p-1] < val) { top_s[p] = top_s[p-1]; top_x[p] = top_x[p-1]; top_y[p] = top_y[p-1]; p--; }
          top_s[p] = val; top_x[p] = x; top_y[p] = y; nt++;
        } else if (val > top_s[per_cell - 1]) {
          int p = per_cell - 1;
          while (p > 0 && top_s[p-1] < val) { top_s[p] = top_s[p-1]; top_x[p] = top_x[p-1]; top_y[p] = top_y[p-1]; p--; }
          top_s[p] = val; top_x[p] = x; top_y[p] = y;
        }
      }
      for (int i = 0; i < nt && c->size < max; i++) {
        corner_vec_push(c, (Corner){(float)top_x[i], (float)top_y[i], -1});
      }
    }
  }
  free(top_s); free(top_x); free(top_y); free(s);
}

static void track_corners_pure_lk(const unsigned char* p_g, const unsigned char* c_g, int w, int h, const CornerVec* p_pts, CornerVec* c_pts, MatchVec* m, float init_dx, float init_dy) {
  unsigned char *p_g1=malloc((w/2)*(h/2)), *c_g1=malloc((w/2)*(h/2)), *p_g2=malloc((w/4)*(h/4)), *c_g2=malloc((w/4)*(h/4)), *p_g3=malloc((w/8)*(h/8)), *c_g3=malloc((w/8)*(h/8));
  downsample2x(p_g,w,h,p_g1); downsample2x(c_g,w,h,c_g1); downsample2x(p_g1,w/2,h/2,p_g2); downsample2x(c_g1,w/2,h/2,c_g2); downsample2x(p_g2,w/4,h/4,p_g3); downsample2x(c_g2,w/4,h/4,c_g3);
  Corner* res=malloc(p_pts->size*sizeof(Corner)); int* ok=calloc(p_pts->size,sizeof(int));
  #pragma omp parallel for
  for(int i=0; i<p_pts->size; i++){
    Corner p=p_pts->data[i]; float dx=init_dx, dy=init_dy;
    for(int level=3; level>=0; level--){
      const unsigned char *pg, *cg; int lw, lh; float sc;
      if(level==3){ pg=p_g3; cg=c_g3; lw=w/8; lh=h/8; sc=0.125f; } else if(level==2){ pg=p_g2; cg=c_g2; lw=w/4; lh=h/4; sc=0.25f; } else if(level==1){ pg=p_g1; cg=c_g1; lw=w/2; lh=h/2; sc=0.5f; } else { pg=p_g; cg=c_g; lw=w; lh=h; sc=1.0f; }
      float lx=p.x*sc, ly=p.y*sc; float l_dx=dx*sc, l_dy=dy*sc;
      for(int it=0; it<10; it++){
        float G[4]={0}, b[2]={0}; for(int y=-3; y<=3; y++)for(int x=-3; x<=3; x++){
          float cur_x=lx+x, cur_y=ly+y, nxt_x=lx+l_dx+x, nxt_y=ly+l_dy+y; if(cur_x<1||cur_x>=lw-1||cur_y<1||cur_y>=lh-1||nxt_x<1||nxt_x>=lw-1||nxt_y<1||nxt_y>=lh-1)continue;
          float Ix = (get_pixel_bilinear(pg,lw,lh,cur_x+1,cur_y) - get_pixel_bilinear(pg,lw,lh,cur_x-1,cur_y)) * 0.5f;
          float Iy = (get_pixel_bilinear(pg,lw,lh,cur_x,cur_y+1) - get_pixel_bilinear(pg,lw,lh,cur_x,cur_y-1)) * 0.5f;
          float It = get_pixel_bilinear(cg,lw,lh,nxt_x,nxt_y) - get_pixel_bilinear(pg,lw,lh,cur_x,cur_y);
          G[0]+=Ix*Ix; G[1]+=Ix*Iy; G[3]+=Iy*Iy; b[0]-=Ix*It; b[1]-=Iy*It; }
        G[2]=G[1]; float det=G[0]*G[3]-G[1]*G[2]; if(fabs(det)<1e-6)break;
        float vx=(G[3]*b[0]-G[1]*b[1])/det, vy=(G[0]*b[1]-G[2]*b[0])/det; l_dx+=vx; l_dy+=vy; if(vx*vx+vy*vy<0.01)break;
      }
      dx=l_dx/sc; dy=l_dy/sc;
    }
    float back_dx = -dx, back_dy = -dy;
    for(int it=0; it<5; it++){
      float G[4]={0}, b[2]={0}; for(int y=-3; y<=3; y++)for(int x=-3; x<=3; x++){
        float cur_x=p.x+dx+x, cur_y=p.y+dy+y, nxt_x=p.x+dx+back_dx+x, nxt_y=p.y+dy+back_dy+y; if(cur_x<1||cur_x>=w-1||cur_y<1||cur_y>=h-1||nxt_x<1||nxt_x>=w-1||nxt_y<1||nxt_y>=h-1)continue;
        float Ix = (get_pixel_bilinear(c_g,w,h,cur_x+1,cur_y) - get_pixel_bilinear(c_g,w,h,cur_x-1,cur_y)) * 0.5f;
        float Iy = (get_pixel_bilinear(c_g,w,h,cur_x,cur_y+1) - get_pixel_bilinear(c_g,w,h,cur_x,cur_y-1)) * 0.5f;
        float It = get_pixel_bilinear(p_g,w,h,nxt_x,nxt_y) - get_pixel_bilinear(c_g,w,h,cur_x,cur_y);
        G[0]+=Ix*Ix; G[1]+=Ix*Iy; G[3]+=Iy*Iy; b[0]-=Ix*It; b[1]-=Iy*It; }
      G[2]=G[1]; float det=G[0]*G[3]-G[1]*G[2]; if(fabs(det)<1e-6)break;
      float vx=(G[3]*b[0]-G[1]*b[1])/det, vy=(G[0]*b[1]-G[2]*b[0])/det; back_dx+=vx; back_dy+=vy;
    }
    float fb_err = (dx+back_dx)*(dx+back_dx) + (dy+back_dy)*(dy+back_dy);
    float tx=p.x+dx, ty=p.y+dy; if(fb_err < 0.5f && tx>=2 && tx<w-2 && ty>=2 && ty<h-2){ res[i]=(Corner){tx,ty,p.pt_idx}; ok[i]=1; }
  }
  for(int i=0; i<p_pts->size; i++) if(ok[i]){ match_vec_push(m,(Match){i,c_pts->size,0}); corner_vec_push(c_pts,res[i]); }
  free(p_g1); free(c_g1); free(p_g2); free(c_g2); free(p_g3); free(c_g3); free(res); free(ok);
}

typedef struct { unsigned char thumb[256]; int frame_id; Pose pose; CornerVec corners; unsigned char* gray; } KFEntry;
typedef struct { KFEntry* data; int size, cap; } KFDB;
static void kfdb_push(KFDB* db, KFEntry e) { if(db->size == db->cap){ db->cap = db->cap ? db->cap * 2 : 64; db->data = xrealloc(db->data, db->cap * sizeof(KFEntry)); } db->data[db->size++] = e; }
static void gen_thumbnail(const unsigned char* g, int w, int h, unsigned char* out) { for(int y=0; y<16; y++) for(int x=0; x<16; x++) out[y*16+x] = g[(y*h/16)*w + (x*w/16)]; }
static int find_loop_candidate(KFDB* db, const unsigned char* thumb, int current_id) { int bi=-1; uint32_t bs=0xFFFFFFFF; for(int i=0; i<db->size-50; i++) { uint32_t s=0; for(int j=0; j<256; j++) s+=abs(thumb[j]-db->data[i].thumb[j]); if(s<bs){bs=s;bi=i;} } return (bs<3500)?bi:-1; }

static int verify_loop(KFDB* db, int lidx, const unsigned char* cgray, int w, int h, double fx, double fy, double cx, double cy, Pose* out_pose) {
  KFEntry* kf = &db->data[lidx]; MatchVec matches = {0}; CornerVec cpts = {0};
  for (int i=0; i<kf->corners.size; i+=2) {
    Corner p = kf->corners.data[i]; float best_ncc = -1; int best_x=-1, best_y=-1;
    for (int y=(int)p.y-20; y<(int)p.y+20; y+=2) for (int x=(int)p.x-20; x<(int)p.x+20; x+=2) {
      if (x<4 || x>=w-4 || y<4 || y>=h-4) continue;
      float ncc=0, s1=0, s2=0; for(int dy=-3; dy<=3; dy++) for(int dx=-3; dx<=3; dx++) {
        float v1=kf->gray[((int)p.y+dy)*w+(int)p.x+dx], v2=cgray[(y+dy)*w+x+dx]; ncc+=v1*v2; s1+=v1*v1; s2+=v2*v2;
      }
      ncc /= sqrt(s1*s2+1e-6); if(ncc > best_ncc) { best_ncc=ncc; best_x=x; best_y=y; }
    }
    if (best_ncc > 0.95) { match_vec_push(&matches, (Match){i, cpts.size, 0}); corner_vec_push(&cpts, (Corner){(float)best_x, (float)best_y, -1}); }
  }
  Pose rel; unsigned char* mask; int inl; int ok = estimate_pose_E(&kf->corners, &cpts, &matches, fx, fy, cx, cy, &rel, &mask, &inl);
  if(ok && inl > 20) { pose_compose_relative(&rel, &kf->pose, out_pose); free(mask); free(matches.data); free(cpts.data); return 1; }
  free(matches.data); free(cpts.data); return 0;
}

// Alternating local BA over the last BA_WINDOW keyframes: refine each KF's
// pose, then refine each observed map point with its pooled 3x3 normal
// equations. Complexity is O(window * n_corners) per iteration, not
// O(map_size * window * n_corners) — the original implementation walked
// the entire map per point which became quadratic as the map grew.
#define BA_WINDOW 3
#define BA_ITERS  5

static void local_ba(KFDB* db, Map* map, double fx, double fy, double cx, double cy) {
  if (db->size < BA_WINDOW) return;
  int k0 = db->size - BA_WINDOW;
  double* Hs = calloc((size_t)map->size * 9, sizeof(double));
  double* bs = calloc((size_t)map->size * 3, sizeof(double));
  unsigned char* touched = calloc((size_t)map->size, 1);
  for (int iter = 0; iter < BA_ITERS; iter++) {
    for (int k = k0; k < db->size; k++) {
      refine_pose_lm(map, &db->data[k].corners, fx, fy, cx, cy, &db->data[k].pose);
    }
    memset(Hs, 0, (size_t)map->size * 9 * sizeof(double));
    memset(bs, 0, (size_t)map->size * 3 * sizeof(double));
    memset(touched, 0, (size_t)map->size);
    for (int k = k0; k < db->size; k++) {
      KFEntry* kf = &db->data[k];
      double R[9], t[3]; pose_get_rotation(&kf->pose, R); pose_get_translation(&kf->pose, t);
      for (int j = 0; j < kf->corners.size; j++) {
        int pi = kf->corners.data[j].pt_idx;
        if (pi < 0 || map->data[pi].obs < 2) continue;
        MapPoint p = map->data[pi];
        double cp[3] = {
          R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
          R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
          R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2] };
        if (cp[2] < 0.1) continue;
        double inv_z = 1.0 / cp[2], inv_z2 = inv_z * inv_z;
        double u = fx * cp[0] * inv_z + cx, v = fy * cp[1] * inv_z + cy;
        double du = kf->corners.data[j].x - u, dv = kf->corners.data[j].y - v;
        double J[2][3] = {
          { fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2, fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2, fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2 },
          { fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2, fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2, fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2 } };
        double* Hp = &Hs[pi*9];
        double* bp = &bs[pi*3];
        for (int r = 0; r < 3; r++) {
          bp[r] += J[0][r] * du + J[1][r] * dv;
          for (int c = 0; c < 3; c++) Hp[r*3+c] += J[0][r]*J[0][c] + J[1][r]*J[1][c];
        }
        touched[pi] = 1;
      }
    }
    for (int i = 0; i < map->size; i++) {
      if (!touched[i]) continue;
      double* H = &Hs[i*9]; double* b = &bs[i*3];
      H[0] += 1e-4; H[4] += 1e-4; H[8] += 1e-4;
      double det = H[0]*(H[4]*H[8]-H[5]*H[7]) - H[1]*(H[3]*H[8]-H[5]*H[6]) + H[2]*(H[3]*H[7]-H[4]*H[6]);
      if (fabs(det) < 1e-9) continue;
      double dx = (b[0]*(H[4]*H[8]-H[5]*H[7]) - H[1]*(b[1]*H[8]-H[5]*b[2]) + H[2]*(b[1]*H[7]-H[4]*b[2])) / det;
      double dy = (H[0]*(b[1]*H[8]-H[5]*b[2]) - b[0]*(H[3]*H[8]-H[5]*H[6]) + H[2]*(H[3]*b[2]-b[1]*H[6])) / det;
      double dz = (H[0]*(H[4]*b[2]-b[1]*H[7]) - H[1]*(H[3]*b[2]-b[1]*H[6]) + b[0]*(H[3]*H[7]-H[4]*H[6])) / det;
      map->data[i].x += dx; map->data[i].y += dy; map->data[i].z += dz;
    }
  }
  free(Hs); free(bs); free(touched);
}

static void ensure_parent_dir(const char* p){ char b[512], *s; snprintf(b,512,"%s",p); s=strrchr(b,'/'); if(s){*s='\0'; char c[640]; snprintf(c,640,"mkdir -p \"%s\"",b); (void)!system(c);} }
static void write_metrics_json(FILE* f, const Config* cfg, const FrameStatVec* s, int pts, int tri, double dur){
  int kf=0; double av=0; for(int i=0;i<s->size;i++){if(s->data[i].is_keyframe)kf++; if(i>0)av+=s->data[i].inliers;} if(s->size>1)av/=(s->size-1);
  fprintf(f,"{\n  \"frames\": %d, \"points\": %d, \"duration_sec\": %f, \"video_path\": \"%s\", \"keyframes\": %d, \"tri_points_total\": %d, \"avg_inliers_after_first\": %f, \"timeline\": [\n",s->size,pts,dur,cfg->video_path,kf,tri,av);
  for(int i=0;i<s->size;i++) fprintf(f,"    {\"frame_id\": %d, \"inliers\": %d, \"is_keyframe\": %s, \"points_added\": %d, \"points_total\": %d, \"xyz\": [%f,%f,%f]}%s\n",s->data[i].frame_id,s->data[i].inliers,s->data[i].is_keyframe?"true":"false",s->data[i].points_added,s->data[i].points_total,s->data[i].xyz[0],s->data[i].xyz[1],s->data[i].xyz[2],(i+1<s->size)?",":"");
  fprintf(f,"  ]\n}\n");
}

static double now_seconds(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return (double)ts.tv_sec+(double)ts.tv_nsec*1e-9; }
static Config parse_args(int argc, char** argv){ Config c={"test_kitti984.mp4",5.0,30.0,NULL,20,5.0,15000}; for(int i=1;i<argc;i++){ if(!strcmp(argv[i],"--video_path")&&i+1<argc)c.video_path=argv[++i]; else if(!strcmp(argv[i],"--seconds")&&i+1<argc)c.seconds=atof(argv[++i]); else if(!strcmp(argv[i],"--timeout")&&i+1<argc)c.timeout=atof(argv[++i]); else if(!strcmp(argv[i],"--metrics_out")&&i+1<argc)c.metrics_out=argv[++i]; else if(argv[i][0]!='-')c.video_path=argv[i]; } return c; }

int main(int argc, char** argv) {
  Config cfg=parse_args(argc,argv); FFmpegCap* cap=ffmpeg_open(cfg.video_path); if(!cap){fprintf(stderr,"Failed pipe.\n");return 1;}
  srand(1);
  int w=640,h=480; unsigned char *raw=malloc(w*h*3),*pgray=calloc(w*h,1),*cgray=calloc(w*h,1),*cblur=calloc(w*h,1);
  FrameLite prev={0},curr={0}; FrameStatVec stats={0}; Pose lkf_pose; pose_identity(&lkf_pose);
  KFDB kf_db = {0}; Map map = {0}; int frame_id=0,pts=0,tri=0; double start=now_seconds();
  double fx=525.0, fy=525.0, cx=319.5, cy=239.5;
  float flow_dx = 0.0f, flow_dy = 0.0f;  // constant-velocity prior for LK
  int last_kf_frame = -100;              // enforce minimum KF spacing
  while(frame_id < (int)(25.0*cfg.seconds) && ffmpeg_read(cap,raw)){
    if(now_seconds()-start > cfg.timeout)break; bgr_to_gray(raw,w,h,cgray); blur_3x3(cgray,w,h,cblur); MatchVec matches={0}; CornerVec tracked={0}; Pose pose,rel; unsigned char* mask=NULL; int inl=0,mkf=0,added=0;
    if(frame_id==0){ extract_corners_pure(cblur,w,h,&curr.corners,2000); mkf=1; pose_identity(&pose); }
    else { track_corners_pure_lk(pgray,cblur,w,h,&prev.corners,&tracked,&matches,flow_dx,flow_dy);
      // Update constant-velocity prior from this frame's mean tracked flow.
      if (matches.size > 0) {
        double sum_dx = 0, sum_dy = 0;
        for (int j = 0; j < matches.size; j++) {
          Corner a = prev.corners.data[matches.data[j].query_idx];
          Corner b = tracked.data[matches.data[j].train_idx];
          sum_dx += b.x - a.x; sum_dy += b.y - a.y;
        }
        flow_dx = (float)(sum_dx / matches.size);
        flow_dy = (float)(sum_dy / matches.size);
      }
      if(estimate_pose_PnP(&map,&tracked,fx,fy,cx,cy,&pose,&inl)){ refine_pose_lm(&map, &tracked, fx, fy, cx, cy, &pose); }
      else if(estimate_pose_E(&prev.corners,&tracked,&matches,fx,fy,cx,cy,&rel,&mask,&inl)) { pose_compose_relative(&rel,&prev.pose,&pose); refine_pose_lm(&map, &tracked, fx, fy, cx, cy, &pose); }
      else pose=prev.pose;
      // Adaptive keyframe policy: track ratio drops below 60% of the feature
      // count at the last KF, camera rotated more than kf_max_rot_deg since
      // the last KF, or pose estimation collapsed. The fixed frame%10 force
      // is gone.
      // KF triggers: pose collapsed, tracking thinned below an absolute
      // floor, big rotation since the last KF, or enough frames passed that
      // we must insert a new baseline. We also enforce a minimum KF spacing
      // so back-to-back KFs never happen -- consecutive KFs have zero baseline
      // and produce garbage triangulation.
      int frames_since_kf = frame_id - last_kf_frame;
      int track_low = (tracked.size < 400);
      int pose_weak = (inl < 15);
      int too_old = (frames_since_kf >= 8);
      int rot_big = rotation_degrees_between(&lkf_pose, &pose) > cfg.kf_max_rot_deg;
      if (frames_since_kf >= 3 && (track_low || pose_weak || rot_big || too_old)) mkf = 1;
      if(mkf){
        // Grow the map at every KF: bump obs counts on already-triangulated
        // points and triangulate any still-unassigned matches. Previously this
        // was gated on the E-estimation mask, so PnP-stable frames never added
        // any new points and the map froze a few dozen points in.
        for (int j = 0; j < matches.size; j++) {
          int ti = matches.data[j].train_idx;
          if (tracked.data[ti].pt_idx != -1) { map.data[tracked.data[ti].pt_idx].obs++; continue; }
          if (mask && !mask[j]) continue;  // if we do have an E-mask, honor it
          double X[3];
          if (triangulate_point(&prev.pose, &pose,
                                prev.corners.data[matches.data[j].query_idx],
                                tracked.data[ti], fx, fy, cx, cy, X)) {
            tracked.data[ti].pt_idx = map.size;
            map_push(&map, (MapPoint){X[0], X[1], X[2], 1});
            pts++; added++; tri++;
          }
        }
        if(tracked.size<1200) extract_corners_pure(cblur,w,h,&tracked,2000);
        unsigned char thumb[256]; gen_thumbnail(cgray,w,h,thumb); int lidx=find_loop_candidate(&kf_db,thumb,frame_id);
        if(lidx>=0) { Pose loop_pose; if(verify_loop(&kf_db, lidx, cgray, w, h, fx, fy, cx, cy, &loop_pose)) { printf("LOOP CLOSED: %d with %d\n", frame_id, kf_db.data[lidx].frame_id); pose = loop_pose; } }
        KFEntry e; memcpy(e.thumb,thumb,256); e.frame_id=frame_id; e.pose=pose; e.corners.size=tracked.size; e.corners.data=malloc(tracked.size*sizeof(Corner)); memcpy(e.corners.data,tracked.data,tracked.size*sizeof(Corner)); e.gray=malloc(w*h); memcpy(e.gray,cgray,w*h); kfdb_push(&kf_db,e);
        local_ba(&kf_db, &map, fx, fy, cx, cy); // Refine map and recent keyframes
        lkf_pose = pose;              // anchor rotation reference to every KF
        last_kf_frame = frame_id;
      }
      curr.corners=tracked;
    }
    curr.pose=pose; double c[3]; camera_center_from_pose(&pose,c); frame_stat_vec_push(&stats,(FrameStat){frame_id,inl,mkf,added,pts,{c[0],c[1],c[2]}});
    if((frame_id+1)%10==0)printf("Frames=%d Pts=%d KF=%d Map=%d\n",frame_id+1,pts,kf_db.size,map.size);
    memcpy(pgray,cblur,w*h); free(prev.corners.data); prev=curr; memset(&curr,0,sizeof(curr)); free(matches.data); free(mask); frame_id++;
  }
  const char* out=cfg.metrics_out?cfg.metrics_out:"runs/pure_c_metrics.json"; ensure_parent_dir(out); FILE* f=fopen(out,"wb"); if(f){write_metrics_json(f,&cfg,&stats,pts,tri,now_seconds()-start);fclose(f);}
  ffmpeg_close(cap); free(raw); free(pgray); free(cgray); free(cblur); free(prev.corners.data); free(stats.data); free(map.data); return 0;
}
