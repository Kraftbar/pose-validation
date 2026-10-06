/* SPDX-License-Identifier: MIT
 * Fixture I/O belongs only to the harness. The port uses the five permitted
 * standard headers and has no dependency on stdio or a native library. */
#include "rd_cv.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static FILE *f;
static size_t total,bad,records;
static int failed;
static void *read_record(const char *label,size_t *n) {
    char name[32];uint32_t size;
    if(fread(name,1,32,f)!=32||fread(&size,4,1,f)!=1||size>100000000)goto error;
    if(!memchr(name,0,32)||strcmp(name,label))goto error;
    void *p=malloc(size?size:1);if(!p)goto error;
    if(size&&fread(p,1,size,f)!=size) {free(p);goto error;}
    *n=size;return p;
error:
    fprintf(stderr,"invalid/truncated fixture at %s\n",label);failed=1;return NULL;
}
static void compare(void *u,const char *label,const void *got,size_t n) {
    (void)u;
    if(failed)return;
    size_t z;uint8_t *want=read_record(label,&z);if(!want)return;
    if(z!=n) {fprintf(stderr,"size %s expected %zu got %zu\n",label,z,n);failed=1;free(want);return;}
    size_t mismatch=0;for(size_t i=0;i<n;i++)if(want[i]!=((const uint8_t*)got)[i]) {
        if(mismatch==0)fprintf(stderr,"%s first byte=%zu expected=%02x got=%02x\n",label,i,want[i],((const uint8_t*)got)[i]);
        mismatch++;
    }
    printf("%s: %zu/%zu\n",label,mismatch,n);
    total+=n;bad+=mismatch;records++;free(want);
}
static int fixture(const char *path) {
    f=strcmp(path,"-")==0?stdin:fopen(path,"rb");if(!f) {perror(path);return 1;}
    char magic[8];uint32_t hdr[3];
    if(fread(magic,1,8,f)!=8||memcmp(magic,"RDCV001\0",8)||fread(hdr,4,3,f)!=3||hdr[0]<1||hdr[1]<1||hdr[0]>16384||hdr[1]>16384) {fclose(f);return 1;}
    int w=(int)hdr[0],h=(int)hdr[1];size_t n=(size_t)w*h,z=0;
    uint8_t *a=read_record("input",&z),*b=NULL;
    uint8_t *ca=NULL,*cb=NULL,*s=NULL,*rs=NULL;
    float *r=NULL,*err=NULL,*re=NULL;double *norms=NULL;
    rd_cv_keypoint *k=NULL,*ke=NULL;rd_cv_point *pts=NULL,*flow=NULL,*rev=NULL;
    rd_cv_pyramid pa={0},pb={0};
    if(!a||z!=n)goto error;
    b=read_record("input_next",&z);if(!b||z!=n)goto error;
    ca=malloc(n);cb=malloc(n);r=malloc(n*4);if(!ca||!cb||!r)goto error;
    if(!rd_cv_clahe(a,w,h,ca)||!rd_cv_clahe(b,w,h,cb))goto error;
    compare(NULL,"clahe",ca,n);compare(NULL,"clahe_next",cb,n);
    if(!rd_cv_build_pyramid(ca,w,h,&pa)||!rd_cv_build_pyramid(cb,w,h,&pb))goto error;
    uint32_t levels=(uint32_t)pa.count;compare(NULL,"levels",&levels,4);
    for(int p=0;p<2;p++) {
        rd_cv_pyramid *py=p?&pb:&pa;
        for(int i=0;i<py->count;i++) {
            rd_cv_level *l=&py->level[i];size_t z=(size_t)l->step*(l->height+42);
            compare(NULL,"image_padded",l->image,z);compare(NULL,"derivative_padded",l->deriv,z*4);
        }
    }
    rd_cv_trace trace={compare,NULL};
    if(!rd_cv_corner_response(ca,w,h,1,r,&trace))goto error;
    compare(NULL,"harris",r,n*4);
    if(!rd_cv_corner_response(ca,w,h,0,r,NULL))goto error;
    compare(NULL,"min_eigen",r,n*4);
    size_t nk=0,ne=0;
    if(!rd_cv_gftt(ca,w,h,200,1,&k,&nk)||!rd_cv_gftt(ca,w,h,200,0,&ke,&ne))goto error;
    compare(NULL,"gftt_harris",k,nk*sizeof(*k));compare(NULL,"gftt_eigen",ke,ne*sizeof(*ke));
    rd_cv_sort_keypoints(k,nk);compare(NULL,"rdvio_sorted",k,nk*sizeof(*k));
    pts=read_record("points",&z);if(!pts||z%sizeof(*pts))goto error;
    size_t np=z/sizeof(*pts);if(np!=nk+8)goto error;
    /* Independently regenerated detector output supplies real LK points. */
    for(size_t i=0;i<nk;i++)pts[i]=k[i].pt;
    flow=read_record("initial_flow",&z);if(!flow||z!=np*sizeof(*flow))goto error;
    rev=malloc(np*sizeof(*rev));s=malloc(np);rs=malloc(np);err=malloc(np*4);re=malloc(np*4);norms=malloc(np*8);
    if(!rev||!s||!rs||!err||!re||!norms)goto error;
    memcpy(rev,pts,np*sizeof(*pts));
    for(size_t i=0;i<np;i++) {uint32_t u=0x7fc12345;memcpy(err+i,&u,4);memcpy(re+i,&u,4);}
    if(!rd_cv_lk(&pa,&pb,pts,flow,s,err,np))goto error;
    compare(NULL,"forward_points",flow,np*8);compare(NULL,"forward_status",s,np);compare(NULL,"forward_err",err,np*4);
    if(!rd_cv_lk(&pb,&pa,flow,rev,rs,re,np))goto error;
    compare(NULL,"reverse_points",rev,np*8);compare(NULL,"reverse_status",rs,np);compare(NULL,"reverse_err",re,np*4);
    for(size_t i=0;i<np;i++) {rd_cv_point d={pts[i].x-rev[i].x,pts[i].y-rev[i].y};norms[i]=rd_cv_norm(d);}
    compare(NULL,"norm",norms,np*8);
    rd_cv_track_status(pts,flow,rev,s,rs,np,w,h);compare(NULL,"rdvio_status",s,np);
    if(fgetc(f)!=EOF)goto error;
    goto done;
error: failed=1;
done:
    free(a);free(b);free(ca);free(cb);free(r);free(k);free(ke);free(pts);free(flow);free(rev);free(s);free(rs);free(err);free(re);free(norms);
    rd_cv_free_pyramid(&pa);rd_cv_free_pyramid(&pb);fclose(f);return failed;
}
int main(int argc,char **argv) {
    if(argc<2) {fprintf(stderr,"usage: check_rd_cv fixture.bin [...]\n");return 2;}
    for(int i=1;i<argc;i++) {printf("CASE %s\n",argv[i]);if(fixture(argv[i]))break;}
    if(failed) {bad++;total++;}
    printf("cases=%d records=%zu\n",argc-1,records);
    printf("m7: %zu/%zu\n",bad,total);
    return bad||failed?1:0;
}
