#ifndef SV_RELOC_TRACE_COMPARE_H
#define SV_RELOC_TRACE_COMPARE_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct {FILE*f;size_t bad,total,events;const char*case_name;} trace_comparison;
static void trace_read(FILE*f,void*p,size_t n){if(fread(p,1,n,f)!=n){fprintf(stderr,"truncated trace\n");exit(2);}}
static void compare_trace(void*user,const char*name,const double*p,unsigned n){
 trace_comparison*c=user;unsigned len,count;char expected[128];trace_read(c->f,&len,4);if(len>=sizeof(expected))exit(2);trace_read(c->f,expected,len);expected[len]=0;trace_read(c->f,&count,4);
 if(strcmp(name,expected)||n!=count){fprintf(stderr,"%s event %zu: %s[%u] != %s[%u]\n",c->case_name,c->events,name,n,expected,count);exit(2);}
 for(unsigned i=0;i<n;i++){double v;trace_read(c->f,&v,8);c->total++;if(memcmp(&v,p+i,8)){if(c->bad<8)fprintf(stderr,"%s event %zu %s[%u]: C=%a ref=%a\n",c->case_name,c->events,name,i,p[i],v);c->bad++;}}
 c->events++;
}
static void compare_scalar(trace_comparison*c,const char*name,double x){compare_trace(c,name,&x,1);}
static void compare_frame(trace_comparison*c,const char*name,const sv_tr_frame*f){
 char s[80];snprintf(s,sizeof(s),"%s_valid",name);compare_scalar(c,s,f->pose_valid);
 if(f->pose_valid){snprintf(s,sizeof(s),"%s_pose",name);compare_trace(c,s,f->pose_cw,16);}
 double*v=malloc((f->obs->num_kp?f->obs->num_kp:1)*sizeof(double));if(!v)exit(2);for(unsigned i=0;i<f->obs->num_kp;i++)v[i]=f->lm[i];snprintf(s,sizeof(s),"%s_landmarks",name);compare_trace(c,s,v,f->obs->num_kp);free(v);snprintf(s,sizeof(s),"%s_ref",name);compare_scalar(c,s,f->ref_kf);
}
#endif
