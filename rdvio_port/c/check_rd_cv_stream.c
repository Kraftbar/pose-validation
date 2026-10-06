/* SPDX-License-Identifier: MIT
 * Online replay of patch 0008. Reads actual RD-VIO operations, preserving the
 * full byte comparison while avoiding multi-gigabyte image dump storage. */
#include "rd_cv.h"
#include <stdio.h>
#include <string.h>
static FILE *f;
static size_t bytes[4],bad[4],cases[4],undefined_err;
static unsigned kind;
static int failed;
static void *record(const char *label,size_t *n) {
    char name[32];uint32_t z;
    if(fread(name,1,32,f)!=32||fread(&z,4,1,f)!=1||z>100000000||!memchr(name,0,32)||strcmp(name,label))goto error;
    void *p=malloc(z?z:1);if(!p)goto error;
    if(z&&fread(p,1,z,f)!=z) {free(p);goto error;}
    *n=z;return p;
error:fprintf(stderr,"invalid stream record %s\n",label);failed=1;return NULL;
}
static void compare(const char *label,const void *p,size_t n,const uint8_t *status) {
    size_t z;uint8_t *want=record(label,&z);if(!want)return;
    if(z!=n) {fprintf(stderr,"size %s %zu != %zu\n",label,z,n);free(want);failed=1;return;}
    for(size_t i=0;i<n;i++) {
        /* OpenCV does not specify or initialize err when status=0. Native
         * buffers are only observed; no sentinel is inserted in RD-VIO. */
        if(status&&!status[i/4]) {undefined_err++;continue;}
        bytes[kind]++;
        if(want[i]!=((const uint8_t*)p)[i]) {
            if(bad[kind]<10)fprintf(stderr,"kind=%u case=%zu %s byte=%zu expected=%02x got=%02x\n",kind,cases[kind],label,i,want[i],((const uint8_t*)p)[i]);
            bad[kind]++;
        }
    }
    free(want);
}
static int operation(int w,int h) {
    size_t n=(size_t)w*h,z;
    uint8_t *a=record("input",&z),*b=NULL,*ca=NULL,*s=NULL,*rs=NULL;
    rd_cv_keypoint *kp=NULL;rd_cv_point *p=NULL,*flow=NULL,*rev=NULL;
    float *e=NULL,*re=NULL;rd_cv_pyramid pa={0},pb={0};
    if(!a||z!=n)goto error;
    if(kind==1) {
        ca=malloc(n);if(!ca||!rd_cv_clahe(a,w,h,ca)||!rd_cv_build_pyramid(ca,w,h,&pa))goto error;
        compare("clahe",ca,n,NULL);uint32_t levels=(uint32_t)pa.count;compare("levels",&levels,4,NULL);
        for(int i=0;i<pa.count;i++) {
            rd_cv_level *l=&pa.level[i];size_t len=(size_t)l->step*(l->height+42);
            compare("image_padded",l->image,len,NULL);compare("derivative_padded",l->deriv,len*4,NULL);
        }
    } else if(kind==2) {
        int *max=record("max_points",&z);if(!max)goto error;
        int limit=*max;free(max);if(z!=4||limit<0)goto error;
        size_t nk=0;if(!rd_cv_gftt(a,w,h,limit,1,&kp,&nk))goto error;
        compare("gftt_harris",kp,nk*sizeof(*kp),NULL);
        rd_cv_sort_keypoints(kp,nk);compare("rdvio_sorted",kp,nk*sizeof(*kp),NULL);
    } else {
        b=record("input_next",&z);if(!b||z!=n)goto error;
        p=record("points",&z);if(!p||z%sizeof(*p))goto error;size_t np=z/sizeof(*p);
        flow=record("initial_flow",&z);if(!flow||z!=np*sizeof(*flow))goto error;
        if(!rd_cv_build_pyramid(a,w,h,&pa)||!rd_cv_build_pyramid(b,w,h,&pb))goto error;
        s=malloc(np?np:1);rs=malloc(np?np:1);e=calloc(np+1,4);re=calloc(np+1,4);rev=malloc((np+1)*sizeof(*rev));
        if(!s||!rs||!e||!re||!rev)goto error;
        memcpy(rev,p,np*sizeof(*p));
        if(!rd_cv_lk(&pa,&pb,p,flow,s,e,np))goto error;
        compare("forward_points",flow,np*8,NULL);compare("forward_status",s,np,NULL);compare("forward_err",e,np*4,s);
        if(!rd_cv_lk(&pb,&pa,flow,rev,rs,re,np))goto error;
        compare("reverse_points",rev,np*8,NULL);compare("reverse_status",rs,np,NULL);compare("reverse_err",re,np*4,rs);
        rd_cv_track_status(p,flow,rev,s,rs,np,w,h);compare("rdvio_status",s,np,NULL);
    }
    goto done;
error:failed=1;
done:
    free(a);free(b);free(ca);free(s);free(rs);free(kp);free(p);free(flow);free(rev);free(e);free(re);
    rd_cv_free_pyramid(&pa);rd_cv_free_pyramid(&pb);return failed;
}
int main(int argc,char **argv) {
    if(argc!=2) {fprintf(stderr,"usage: check_rd_cv_stream fifo-or-file\n");return 2;}
    f=fopen(argv[1],"rb");if(!f) {perror(argv[1]);return 2;}
    for(;;) {
        uint32_t dims[2];size_t got=fread(&kind,1,4,f);
        if(!got&&feof(f))break;
        if(got!=4||kind<1||kind>3||fread(dims,4,2,f)!=2||dims[0]<1||dims[1]<1||dims[0]>16384||dims[1]>16384) {failed=1;break;}
        cases[kind]++;if(operation((int)dims[0],(int)dims[1]))break;
        if(cases[kind]%100==0) {printf("progress kind=%u cases=%zu mismatches=%zu\n",kind,cases[kind],bad[kind]);fflush(stdout);}
    }
    fclose(f);size_t all=0,errors=0;
    for(unsigned k=1;k<=3;k++) {printf("kind=%u cases=%zu bytes=%zu mismatches=%zu\n",k,cases[k],bytes[k],bad[k]);all+=bytes[k];errors+=bad[k];}
    if(failed||!cases[1]||!cases[2]||!cases[3]) {all++;errors++;}
    printf("unspecified failed-track err bytes excluded=%zu\n",undefined_err);
    printf("m7_stream: %zu/%zu\n",errors,all);return errors?1:0;
}
