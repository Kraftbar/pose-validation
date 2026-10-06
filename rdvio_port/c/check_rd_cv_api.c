/* SPDX-License-Identifier: MIT */
#include "rd_cv.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int bad,total;
#define CHECK(x) do { total++;if(!(x)) {bad++;fprintf(stderr,"API check failed line %d\n",__LINE__);} } while(0)
int main(void) {
    uint8_t src[43*43],a[43*43],b[43*43];
    for(size_t i=0;i<sizeof(src);i++)src[i]=(uint8_t)(i*13);
    CHECK(!rd_cv_clahe(NULL,43,43,a));
    CHECK(!rd_cv_clahe(src,0,43,a));
    CHECK(!rd_cv_clahe(src,16385,43,a));
    CHECK(rd_cv_clahe(src,43,43,a));
    memcpy(b,src,sizeof(b));CHECK(rd_cv_clahe(b,43,43,b));CHECK(!memcmp(a,b,sizeof(a)));
    rd_cv_pyramid p={0};CHECK(rd_cv_build_pyramid(src,43,43,&p));
    CHECK(p.count==2&&p.level[1].width==22&&p.level[1].height==22);
    CHECK(!rd_cv_build_pyramid(src,43,43,&p));
    CHECK(rd_cv_lk(&p,&p,NULL,NULL,NULL,NULL,0));
    rd_cv_point pts={NAN,1},flow={1,1};uint8_t s=123;float e=456;
    CHECK(!rd_cv_lk(&p,&p,&pts,&flow,&s,&e,1));CHECK(s==123&&e==456&&flow.x==1&&flow.y==1);
    rd_cv_free_pyramid(&p);rd_cv_free_pyramid(&p);CHECK(p.count==0&&p.level[0].image==NULL);
    CHECK(!rd_cv_lk(&p,&p,NULL,NULL,NULL,NULL,0));
    CHECK(rd_cv_build_pyramid(src,1,1,&p)&&p.count==1);rd_cv_free_pyramid(&p);
    rd_cv_keypoint *k=NULL;size_t n=123;
    CHECK(rd_cv_gftt(src,1,1,200,1,&k,&n));CHECK(n==0);free(k);
    CHECK(!rd_cv_gftt(src,43,43,-1,1,&k,&n));
    rd_cv_sort_keypoints(NULL,0);rd_cv_free_pyramid(NULL);
    printf("m7_api: %d/%d\n",bad,total);return bad?1:0;
}
