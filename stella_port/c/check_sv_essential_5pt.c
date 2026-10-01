/* SV_PORT_SOURCES: check_sv_essential_5pt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c sv_linalg.c */
/* SPDX-License-Identifier: MIT */
#include "sv_solve_essential_ransac.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

typedef struct {FILE *f;unsigned n,frame,iter;double *a,*b;unsigned char *mask;size_t bad,total;} comparison;
static void read_exact(comparison *c,void *out,size_t n){if(fread(out,1,n,c->f)!=n){fprintf(stderr,"truncated trace frame %u\n",c->frame);exit(2);}}
static void compare(comparison *c,const void *a,const void *b,size_t n,size_t width,const char *stage){
 const unsigned char *x=a,*y=b;
 for(size_t i=0;i<n;i++){c->total++;if(memcmp(x+i*width,y+i*width,width)){if(c->bad<4)fprintf(stderr,"frame %u iter %u %s[%zu] differs\n",c->frame,c->iter,stage,i);c->bad++;}}
}
static void inliers(comparison*c,float cost,unsigned count,const unsigned char *mask){
 float native_cost;unsigned native_count;read_exact(c,&native_cost,4);read_exact(c,&native_count,4);
 unsigned char *m=malloc(c->n?c->n:1);if(!m)exit(2);read_exact(c,m,c->n);
 compare(c,&cost,&native_cost,1,4,"cost");compare(c,&count,&native_count,1,4,"inlier count");compare(c,mask,m,c->n,1,"mask");free(m);
}
static void trace(void *user,unsigned iter,const unsigned *indices,unsigned sample,const double *candidates,unsigned count,const float *costs,const unsigned *counts,const sv_essential_5pt_trace *minimal,const sv_essential_result *best){
 comparison *c=user;c->iter=iter;unsigned ns,nc,ix[10];read_exact(c,&ns,4);if(ns!=sample || ns>10)exit(2);read_exact(c,ix,ns*4);
 compare(c,indices,ix,ns,4,"sample");
 sv_essential_5pt_trace native;read_exact(c,&native,sizeof(native));
 #define CMP(field) compare(c,minimal->field,native.field,sizeof(native.field)/sizeof(double),8,#field)
 CMP(constraint);CMP(basis);CMP(polynomial);CMP(eliminated);CMP(action);CMP(eigen_real);CMP(eigen_imag);CMP(vectors_real);CMP(vectors_imag);
 #undef CMP
 compare(c,&minimal->rank,&native.rank,1,sizeof(int),"rank");
 compare(c,&minimal->count,&native.count,1,sizeof(int),"candidate count");
 read_exact(c,&nc,4);if(nc!=count || nc>10){fprintf(stderr,"candidate count changed\n");exit(2);}
 double es[90];read_exact(c,es,nc*72);compare(c,candidates,es,nc*9,8,"candidate E");
 for(unsigned i=0;i<count;i++){
  float cost;unsigned n=sv_essential_check_inliers(candidates+9*i,c->a,c->b,c->n,c->mask,&cost);
  compare(c,&cost,costs+i,1,4,"callback cost");compare(c,&n,counts+i,1,4,"callback count");
  inliers(c,costs[i],counts[i],c->mask);
 }
 float best_cost;unsigned best_count;read_exact(c,&best_cost,4);read_exact(c,&best_count,4);
 compare(c,&best->cost,&best_cost,1,4,"best cost");compare(c,&best->inliers,&best_count,1,4,"best count");
 if(best_cost<FLT_MAX){double e[9];read_exact(c,e,72);compare(c,best->E,e,9,8,"best E");}
}
static int check_file(const char *path,unsigned frame,size_t *bad,size_t *total){
 comparison c={0};c.frame=frame;c.f=fopen(path,"rb");if(!c.f)return 2;
 unsigned iters;read_exact(&c,&c.n,4);read_exact(&c,&iters,4);if(c.n>100000 || iters>10000 || !iters){fclose(c.f);return 2;}
 c.a=malloc((c.n?c.n:1)*24);c.b=malloc((c.n?c.n:1)*24);c.mask=malloc(c.n?c.n:1);unsigned char *mask=malloc(c.n?c.n:1);
 if(!c.a || !c.b || !c.mask || !mask)exit(2);
 for(unsigned i=0;i<c.n;i++){int pair[2];read_exact(&c,pair,8);read_exact(&c,c.a+3*i,24);read_exact(&c,c.b+3*i,24);}
 sv_essential_result result;int rc=sv_essential_ransac(c.a,c.b,c.n,iters,1,5,NULL,&result,mask,trace,&c);
 if(rc){fprintf(stderr,"solver failed frame %u\n",frame);return 2;}
 /* All captured fallback calls recompute from >=8 RANSAC inliers. */
 inliers(&c,result.cost,result.inliers,mask);
 unsigned valid;float cost;read_exact(&c,&valid,4);read_exact(&c,&cost,4);
 unsigned got_valid=(unsigned)result.valid;compare(&c,&got_valid,&valid,1,4,"valid");compare(&c,&result.cost,&cost,1,4,"final cost");
 if(valid){double e[9];read_exact(&c,e,72);compare(&c,result.E,e,9,8,"final E");}
 read_exact(&c,c.mask,c.n);compare(&c,mask,c.mask,c.n,1,"final mask");
 if(fgetc(c.f)!=EOF || ferror(c.f))return 2;
 fclose(c.f);free(c.a);free(c.b);free(c.mask);free(mask);*bad+=c.bad;*total+=c.total;return c.bad?1:0;
}
int main(int argc,char **argv){
 char folder[4096],path[4096];const char *seq;
 if(argc==3){seq=argv[1];if(snprintf(folder,sizeof(folder),"%s",argv[2])>=(int)sizeof(folder))return 2;}
 else if(argc==4 || argc==5){seq=argv[1];if(snprintf(folder,sizeof(folder),"%s/../../reference_essential/fixtures/%s",argv[3],seq)>=(int)sizeof(folder))return 2;}
 else return 2;
 if(snprintf(path,sizeof(path),"%s/frames.txt",folder)>=(int)sizeof(path))return 2;
 FILE *list=fopen(path,"r");if(!list)return 2;
 unsigned frame,cases=0;size_t bad=0,total=0;int code=0;
 while(fscanf(list,"%u",&frame)==1){
  if(snprintf(path,sizeof(path),"%s/%u.trace",folder,frame)>=(int)sizeof(path))return 2;
  int rc=check_file(path,frame,&bad,&total);if(rc>code)code=rc;if(rc==2)break;cases++;
 }
 if(ferror(list) || !feof(list) || !cases || !total)code=2;
 fclose(list);
 printf("%s: %zu/%zu\n",seq,bad,total);return code;
}
